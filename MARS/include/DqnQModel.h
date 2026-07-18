#pragma once

// Header-only Q-function model used by the DQN online allocation search.
//
// QModel is EITHER a linear model (hidden == 0, the default) OR a
// 1-hidden-layer tanh MLP (hidden > 0). The linear path is written to be
// numerically identical to the original LinearQModel it replaces: the
// constructor draws nothing from the rng when hidden == 0, and
// accumulate_grad/apply_grad reduce to the exact same arithmetic (same
// operation order) as the previous hand-rolled linear SGD update.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <string>
#include <vector>

// Numerically stable logistic sigmoid, shared by QModel::accumulate_grad_logistic
// and any caller that needs to turn a QModel's raw predict() output (treated
// as a logit) into a probability (e.g. the decomposed-scoring f_head; see
// DqnAllocationSearch.cpp construct_order_v3).
inline double dqn_sigmoid(double z)
{
  if (z >= 0.0)
  {
    const double e = std::exp(-z);
    return 1.0 / (1.0 + e);
  }
  const double e = std::exp(z);
  return e / (1.0 + e);
}

struct QModel
{
  std::size_t dim = 0;    // input (feature) dimension
  std::size_t hidden = 0; // 0 => linear model; >0 => hidden units of the MLP

  // Linear params (used when hidden == 0): Q(s,a) = w . phi(s,a).
  std::vector<double> w;

  // 1-hidden-layer tanh MLP params (used when hidden > 0):
  //   z[j] = b1[j] + sum_i W1[j*dim+i] * x[i]
  //   a[j] = tanh(z[j])
  //   Q    = b2 + sum_j w2[j] * a[j]
  std::vector<double> W1; // hidden*dim, row-major: W1[j*dim + i]
  std::vector<double> b1; // hidden
  std::vector<double> w2; // hidden
  double b2 = 0.0;

  // Critical: for hidden == 0 this must NOT draw anything from rng, so the
  // linear (default) configuration leaves the rng stream untouched relative
  // to the previous LinearQModel(dim) constructor.
  QModel(std::size_t dim_in, std::size_t hidden_in, std::mt19937 &rng)
      : dim(dim_in), hidden(hidden_in)
  {
    if (hidden == 0)
    {
      w.assign(dim, 0.0);
      return;
    }

    W1.assign(hidden * dim, 0.0);
    b1.assign(hidden, 0.0);
    w2.assign(hidden, 0.0);
    b2 = 0.0;

    const double r1 = 1.0 / std::sqrt(static_cast<double>(std::max<std::size_t>(dim, 1)));
    const double r2 = 1.0 / std::sqrt(static_cast<double>(std::max<std::size_t>(hidden, 1)));
    std::uniform_real_distribution<double> dist_w1(-r1, r1);
    std::uniform_real_distribution<double> dist_w2(-r2, r2);
    for (auto &v : W1)
      v = dist_w1(rng);
    for (auto &v : w2)
      v = dist_w2(rng);
  }

  double predict(const std::vector<double> &x) const
  {
    if (hidden == 0)
    {
      double q = 0.0;
      for (std::size_t i = 0; i < w.size() && i < x.size(); ++i)
        q += w[i] * x[i];
      return q;
    }

    double q = b2;
    for (std::size_t j = 0; j < hidden; ++j)
    {
      double z = b1[j];
      for (std::size_t i = 0; i < dim && i < x.size(); ++i)
        z += W1[j * dim + i] * x[i];
      const double a = std::tanh(z);
      q += w2[j] * a;
    }
    return q;
  }

  // Gradient of the Huber loss L(predict(x), target) wrt all params for ONE
  // sample, ACCUMULATED into `grad` (caller must zero-init `grad` first).
  // `grad` uses the same flat layout as get_params(): [w...] for the linear
  // model, or [W1..., b1..., w2..., b2] for the MLP.
  void accumulate_grad(const std::vector<double> &x, double target, double huber_delta,
                       std::vector<double> &grad) const
  {
    const double err = predict(x) - target;
    const double e = std::max(-huber_delta, std::min(huber_delta, err));

    if (hidden == 0)
    {
      for (std::size_t i = 0; i < dim && i < x.size(); ++i)
        grad[i] += e * x[i];
      return;
    }

    // Recompute the forward activations needed for backprop.
    std::vector<double> a(hidden, 0.0);
    for (std::size_t j = 0; j < hidden; ++j)
    {
      double z = b1[j];
      for (std::size_t i = 0; i < dim && i < x.size(); ++i)
        z += W1[j * dim + i] * x[i];
      a[j] = std::tanh(z);
    }

    const std::size_t off_W1 = 0;
    const std::size_t off_b1 = off_W1 + hidden * dim;
    const std::size_t off_w2 = off_b1 + hidden;
    const std::size_t off_b2 = off_w2 + hidden;

    grad[off_b2] += e;
    for (std::size_t j = 0; j < hidden; ++j)
    {
      grad[off_w2 + j] += e * a[j];
      const double delta = e * w2[j] * (1.0 - a[j] * a[j]);
      grad[off_b1 + j] += delta;
      for (std::size_t i = 0; i < dim && i < x.size(); ++i)
        grad[off_W1 + j * dim + i] += delta * x[i];
    }
  }

  // Gradient of the weighted logistic (binary cross-entropy on sigmoid(z))
  // loss wrt all params for ONE sample, ACCUMULATED into `grad` (caller must
  // zero-init `grad` first). z = predict(x) is treated as a logit; `y` in
  // {0,1} is the binary label; `weight` is a per-sample loss weight (e.g.
  // positive-class upweighting -- see DqnAllocationSearch.cpp
  // train_model_logistic). dL/dz = weight * (sigmoid(z) - y), backpropagated
  // through the same linear/MLP forward pass as accumulate_grad's Huber path
  // above -- kept as a separate entry point so that path stays textually
  // untouched. Same flat grad layout as accumulate_grad/get_params.
  void accumulate_grad_logistic(const std::vector<double> &x, double y, double weight,
                                std::vector<double> &grad) const
  {
    const double z = predict(x);
    const double e = weight * (dqn_sigmoid(z) - y);

    if (hidden == 0)
    {
      for (std::size_t i = 0; i < dim && i < x.size(); ++i)
        grad[i] += e * x[i];
      return;
    }

    // Recompute the forward activations needed for backprop.
    std::vector<double> a(hidden, 0.0);
    for (std::size_t j = 0; j < hidden; ++j)
    {
      double zz = b1[j];
      for (std::size_t i = 0; i < dim && i < x.size(); ++i)
        zz += W1[j * dim + i] * x[i];
      a[j] = std::tanh(zz);
    }

    const std::size_t off_W1 = 0;
    const std::size_t off_b1 = off_W1 + hidden * dim;
    const std::size_t off_w2 = off_b1 + hidden;
    const std::size_t off_b2 = off_w2 + hidden;

    grad[off_b2] += e;
    for (std::size_t j = 0; j < hidden; ++j)
    {
      grad[off_w2 + j] += e * a[j];
      const double delta = e * w2[j] * (1.0 - a[j] * a[j]);
      grad[off_b1 + j] += delta;
      for (std::size_t i = 0; i < dim && i < x.size(); ++i)
        grad[off_W1 + j * dim + i] += delta * x[i];
    }
  }

  // Apply an accumulated minibatch gradient: for each param p,
  //   p -= lr * (grad[p]/batch + l2*p)
  // Written to reproduce the previous linear SGD update exactly (same
  // operation order: divide-then-add-l2, then multiply-then-subtract).
  void apply_grad(const std::vector<double> &grad, std::size_t batch, double lr, double l2)
  {
    const double batch_d = static_cast<double>(batch);

    if (hidden == 0)
    {
      for (std::size_t i = 0; i < w.size(); ++i)
      {
        const double g = grad[i] / batch_d + l2 * w[i];
        w[i] -= lr * g;
      }
      return;
    }

    std::size_t idx = 0;
    for (std::size_t k = 0; k < W1.size(); ++k, ++idx)
    {
      const double g = grad[idx] / batch_d + l2 * W1[k];
      W1[k] -= lr * g;
    }
    for (std::size_t k = 0; k < b1.size(); ++k, ++idx)
    {
      const double g = grad[idx] / batch_d + l2 * b1[k];
      b1[k] -= lr * g;
    }
    for (std::size_t k = 0; k < w2.size(); ++k, ++idx)
    {
      const double g = grad[idx] / batch_d + l2 * w2[k];
      w2[k] -= lr * g;
    }
    {
      const double g = grad[idx] / batch_d + l2 * b2;
      b2 -= lr * g;
      ++idx;
    }
  }

  std::vector<double> get_params() const
  {
    if (hidden == 0)
      return w;

    std::vector<double> p;
    p.reserve(num_params());
    p.insert(p.end(), W1.begin(), W1.end());
    p.insert(p.end(), b1.begin(), b1.end());
    p.insert(p.end(), w2.begin(), w2.end());
    p.push_back(b2);
    return p;
  }

  // Inverse of get_params(); layout must match exactly (used for
  // finite-difference gradient testing).
  void set_params(const std::vector<double> &p)
  {
    if (hidden == 0)
    {
      w = p;
      return;
    }

    std::size_t off = 0;
    W1.assign(p.begin() + static_cast<long>(off),
              p.begin() + static_cast<long>(off + hidden * dim));
    off += hidden * dim;
    b1.assign(p.begin() + static_cast<long>(off),
              p.begin() + static_cast<long>(off + hidden));
    off += hidden;
    w2.assign(p.begin() + static_cast<long>(off),
              p.begin() + static_cast<long>(off + hidden));
    off += hidden;
    b2 = p[off];
  }

  std::size_t num_params() const
  {
    return hidden == 0 ? dim : (hidden * dim + hidden + hidden + 1);
  }

  // Plain-text weight serialization: "<dim> <hidden>\n" followed by
  // get_params() space-separated on the second line. Used to warm-start a
  // fresh QModel from weights trained in an earlier run (fine-tune mode).
  bool save(const std::string &path) const
  {
    std::filesystem::path p(path);
    if (p.has_parent_path())
      std::filesystem::create_directories(p.parent_path());
    std::ofstream ofs(path);
    if (!ofs.is_open())
    {
      std::cerr << "[QModel] Failed to open weights file for writing: "
                << path << std::endl;
      return false;
    }

    ofs << dim << " " << hidden << "\n";
    ofs << std::setprecision(17);
    const std::vector<double> params = get_params();
    for (std::size_t i = 0; i < params.size(); ++i)
    {
      if (i > 0)
        ofs << " ";
      ofs << params[i];
    }
    ofs << "\n";
    return true;
  }

  // Inverse of save(). Leaves this model's params (w / W1,b1,w2,b2)
  // completely untouched and returns false on any failure: missing file,
  // malformed header/params, or a dim/hidden mismatch against this model's
  // own dim/hidden (a stale or incompatible weights file). Callers should
  // fall back to the freshly-initialized model on false.
  bool load(const std::string &path)
  {
    std::ifstream ifs(path);
    if (!ifs.is_open())
    {
      std::cerr << "[QModel] Failed to open weights file for reading: "
                << path << std::endl;
      return false;
    }

    std::size_t file_dim = 0, file_hidden = 0;
    if (!(ifs >> file_dim >> file_hidden))
    {
      std::cerr << "[QModel] Malformed weights file (could not parse "
                   "dim/hidden header): "
                << path << std::endl;
      return false;
    }
    if (file_dim != dim || file_hidden != hidden)
    {
      std::cerr << "[QModel] Weights file dim/hidden mismatch: file has dim="
                << file_dim << " hidden=" << file_hidden << ", model expects dim="
                << dim << " hidden=" << hidden << " (" << path
                << "). Keeping current params." << std::endl;
      return false;
    }

    const std::size_t expected = num_params();
    std::vector<double> params;
    params.reserve(expected);
    double v;
    while (ifs >> v)
      params.push_back(v);
    if (params.size() != expected)
    {
      std::cerr << "[QModel] Malformed weights file (expected " << expected
                << " params, got " << params.size() << "): " << path << std::endl;
      return false;
    }

    set_params(params);
    return true;
  }
};
