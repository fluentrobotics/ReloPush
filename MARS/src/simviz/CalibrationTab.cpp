#include "CalibrationTab.h"

#include <QCheckBox>
#include <QDateTime>
#include <QFont>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHideEvent>
#include <QKeyEvent>
#include <QMessageBox>
#include <QMetaObject>
#include <QPainter>
#include <QPen>
#include <QSignalBlocker>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <sstream>

namespace simviz
{

namespace
{

constexpr int kDutyTtlMs = 500;       // driver's raw-command deadman -- see CalibrationTab.h's header comment.
constexpr int kDutyResendMs = 100;    // resend cadence while a drive key is held (< ttl_ms/2 with margin).
constexpr int kMocapPollMs = 30;
constexpr int kControlRcvTimeoutMs = 500;

// Splits a "tcp://<host>:<port>" endpoint string into its host part (used to
// prefill the Connection strip's host field from a robot's configured
// vesc_endpoint -- see MARS/config/real_robots.json). No existing helper for
// this in the codebase (every other site only ever BUILDS this string, never
// parses it back apart -- see CalibrationTab's own design notes); returns
// "" if `endpoint` doesn't look like "tcp://host:port".
std::string parse_tcp_host(const std::string &endpoint)
{
  const std::string prefix = "tcp://";
  if (endpoint.rfind(prefix, 0) != 0)
    return "";
  const std::string rest = endpoint.substr(prefix.size());
  const auto colon = rest.find(':');
  if (colon == std::string::npos)
    return rest;
  return rest.substr(0, colon);
}

QString dot_style(bool fresh)
{
  return QString("background-color: %1; border-radius: 6px; border: 1px solid black;")
      .arg(fresh ? "#43A047" : "#9E9E9E");
}

} // namespace

// ===========================================================================
// TeleopWorker
// ===========================================================================

TeleopWorker::TeleopWorker(std::string host, int control_port, int telemetry_port,
                            std::string robot_topic, std::chrono::steady_clock::time_point epoch)
    : host_(std::move(host)), control_port_(control_port), telemetry_port_(telemetry_port),
      robot_topic_(std::move(robot_topic)), epoch_(epoch)
{
}

TeleopWorker::~TeleopWorker()
{
  // Sockets are only ever touched on the worker thread; by the time this
  // destructor runs (after QThread::quit()+wait() in
  // CalibrationTab::teardown_worker()), the worker thread's event loop has
  // already stopped, so no further do_*() slot can fire concurrently with
  // this teardown.
}

double TeleopWorker::now_s() const
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - epoch_).count();
}

void TeleopWorker::start()
{
  ctx_ = std::make_unique<zmq::context_t>(1);
  ensure_control_socket();

  telem_sock_ = std::make_unique<zmq::socket_t>(*ctx_, zmq::socket_type::sub);
  telem_sock_->set(zmq::sockopt::linger, 0);
  telem_sock_->set(zmq::sockopt::rcvtimeo, 0);
  const std::string telem_endpoint = "tcp://" + host_ + ":" + std::to_string(telemetry_port_);
  telem_sock_->connect(telem_endpoint);
  telem_sock_->set(zmq::sockopt::subscribe, "/" + robot_topic_ + "/vesc_telemetry");

  poll_timer_ = new QTimer(this);
  connect(poll_timer_, &QTimer::timeout, this, &TeleopWorker::poll_telemetry);
  poll_timer_->start(30);
}

void TeleopWorker::ensure_control_socket()
{
  control_sock_ = std::make_unique<zmq::socket_t>(*ctx_, zmq::socket_type::req);
  control_sock_->set(zmq::sockopt::linger, 0);
  control_sock_->set(zmq::sockopt::rcvtimeo, kControlRcvTimeoutMs);
  control_sock_->set(zmq::sockopt::sndtimeo, kControlRcvTimeoutMs);
  const std::string endpoint = "tcp://" + host_ + ":" + std::to_string(control_port_);
  control_sock_->connect(endpoint);
}

// REQ/REP with a single retry-after-resocket on timeout/send failure --
// mirrors MPC/src/CalibClient.cpp's ZmqDriverClient::request() exactly (see
// this file's header comment on why this file reimplements the wire format
// rather than linking that class directly).
TeleopWorker::ReqResult TeleopWorker::send_request(const nlohmann::json &req_json)
{
  ReqResult out;
  const std::string req_str = req_json.dump();
  for (int attempt = 0; attempt < 2; ++attempt)
  {
    zmq::message_t msg(req_str.data(), req_str.size());
    auto sent = control_sock_->send(msg, zmq::send_flags::none);
    if (!sent.has_value())
    {
      ensure_control_socket();
      continue;
    }
    zmq::message_t reply;
    auto recvd = control_sock_->recv(reply, zmq::recv_flags::none);
    if (!recvd.has_value())
    {
      // Timed out -- REQ socket state is desynced; must recreate before any
      // future send (including the retry below).
      ensure_control_socket();
      continue;
    }
    const std::string reply_str(static_cast<const char *>(reply.data()), reply.size());
    const nlohmann::json j = nlohmann::json::parse(reply_str, nullptr, false);
    if (j.is_discarded() || !j.is_object())
    {
      out.ok = false;
      out.error = "malformed control reply JSON";
      return out;
    }
    out.ok = j.value("ok", false);
    out.error = j.value("error", std::string());
    return out;
  }
  out.ok = false;
  out.error = "driver control REQ timed out twice (host " + host_ + " unreachable?)";
  return out;
}

void TeleopWorker::do_ping()
{
  const ReqResult r = send_request({{"cmd", "ping"}});
  emit control_reply("ping", r.ok, QString::fromStdString(r.error));
}

void TeleopWorker::do_set_source(QString source)
{
  const ReqResult r = send_request({{"cmd", "set_source"}, {"source", source.toStdString()}});
  emit control_reply("set_source", r.ok, QString::fromStdString(r.error));
}

void TeleopWorker::do_raw_duty(double value, int ttl_ms)
{
  const ReqResult r =
      send_request({{"cmd", "raw"}, {"mode", "duty"}, {"value", value}, {"ttl_ms", ttl_ms}});
  emit control_reply("raw", r.ok, QString::fromStdString(r.error));
}

void TeleopWorker::do_servo(double value)
{
  const ReqResult r = send_request({{"cmd", "servo"}, {"value", value}});
  emit control_reply("servo", r.ok, QString::fromStdString(r.error));
}

void TeleopWorker::do_stop()
{
  const ReqResult r = send_request({{"cmd", "stop"}});
  emit control_reply("stop", r.ok, QString::fromStdString(r.error));
}

void TeleopWorker::do_shutdown_blocking()
{
  do_stop();
  do_set_source(QString::fromStdString("ackermann"));
}

void TeleopWorker::poll_telemetry()
{
  // Drain everything currently pending -- feed EVERY frame to the GUI (see
  // this class's telemetry_frame() doc comment), not just the newest.
  while (true)
  {
    zmq::message_t topic_msg;
    auto r1 = telem_sock_->recv(topic_msg, zmq::recv_flags::dontwait);
    if (!r1.has_value())
      break;
    if (!topic_msg.more())
      break;
    zmq::message_t payload_msg;
    auto r2 = telem_sock_->recv(payload_msg, zmq::recv_flags::none);
    if (!r2.has_value())
      break;
    const std::string payload(static_cast<const char *>(payload_msg.data()), payload_msg.size());
    const nlohmann::json j = nlohmann::json::parse(payload, nullptr, false);
    if (j.is_discarded() || !j.is_object())
      continue;
    const double t = j.value("t", now_s());
    const double erpm = j.value("erpm", 0.0);
    const double duty = j.value("duty", 0.0);
    const double current_motor = j.value("current_motor", 0.0);
    const double current_in = j.value("current_in", 0.0);
    const double v_in = j.value("v_in", 0.0);
    emit telemetry_frame(t, erpm, duty, current_motor, current_in, v_in);
  }
}

void TeleopWorker::ping()
{
  QMetaObject::invokeMethod(this, "do_ping", Qt::QueuedConnection);
}

void TeleopWorker::set_source(const std::string &source)
{
  QMetaObject::invokeMethod(this, "do_set_source", Qt::QueuedConnection,
                             Q_ARG(QString, QString::fromStdString(source)));
}

void TeleopWorker::raw_duty(double value, int ttl_ms)
{
  QMetaObject::invokeMethod(this, "do_raw_duty", Qt::QueuedConnection, Q_ARG(double, value),
                             Q_ARG(int, ttl_ms));
}

void TeleopWorker::servo(double value)
{
  QMetaObject::invokeMethod(this, "do_servo", Qt::QueuedConnection, Q_ARG(double, value));
}

void TeleopWorker::stop()
{
  QMetaObject::invokeMethod(this, "do_stop", Qt::QueuedConnection);
}

void TeleopWorker::shutdown_blocking()
{
  // BlockingQueuedConnection: blocks the CALLING (GUI) thread until
  // do_shutdown_blocking() has actually run to completion on the worker
  // thread -- see ITeleopCommandSink::shutdown_blocking()'s doc comment for
  // why this one method is allowed to do that. Calling this FROM the worker
  // thread itself would deadlock (Qt detects same-thread Blocking calls and
  // runs them directly instead -- safe here since this is never invoked
  // reentrantly from a do_*() slot).
  QMetaObject::invokeMethod(this, "do_shutdown_blocking", Qt::BlockingQueuedConnection);
}

// ===========================================================================
// CoverageBinsWidget
// ===========================================================================

CoverageBinsWidget::CoverageBinsWidget(QWidget *parent) : QWidget(parent)
{
  setMinimumSize(300, 90);
}

void CoverageBinsWidget::set_data(const mpc::CoverageReport &report)
{
  report_ = report;
  update();
}

void CoverageBinsWidget::paintEvent(QPaintEvent * /*event*/)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rect(), Qt::white);

  if (report_.bins.empty())
  {
    painter.setPen(Qt::darkGray);
    painter.drawText(rect(), Qt::AlignCenter, "(no bins yet)");
    return;
  }

  double min_c = report_.bins.front().center;
  double max_c = report_.bins.front().center;
  for (const auto &b : report_.bins)
  {
    min_c = std::min(min_c, b.center);
    max_c = std::max(max_c, b.center);
  }
  const double span = std::max(max_c - min_c, 1e-6);

  const int padding = 10;
  const double avail_w = std::max(width() - 2 * padding, 1);
  const int bin_px_w =
      std::max(static_cast<int>(avail_w / (static_cast<double>(report_.bins.size()) * 1.6)), 12);
  const int bar_top = padding;
  const int bar_h = std::max(height() - 2 * padding - 16, 20);

  for (const auto &bin : report_.bins)
  {
    const double frac = span > 0.0 ? (bin.center - min_c) / span : 0.5;
    const int cx = padding + static_cast<int>(frac * avail_w);
    QRect r(cx - bin_px_w / 2, bar_top, bin_px_w, bar_h);

    QColor fill;
    switch (bin.state)
    {
    case mpc::BinState::kEmpty: fill = QColor("#BDBDBD"); break;
    case mpc::BinState::kDone: fill = QColor("#43A047"); break;
    case mpc::BinState::kPartial: fill = QColor("#F2C14E"); break;
    case mpc::BinState::kNotApplicable: fill = QColor("#EEEEEE"); break;
    }
    painter.setPen(QPen(Qt::black, 1));
    painter.setBrush(fill);
    painter.drawRect(r);

    if (bin.state == mpc::BinState::kPartial && bin.target > 0)
    {
      const double frac_fill = std::clamp(static_cast<double>(bin.count) / bin.target, 0.0, 1.0);
      const int fill_h = static_cast<int>(r.height() * frac_fill);
      QRect fill_r(r.left(), r.bottom() - fill_h, r.width(), fill_h);
      painter.setBrush(QColor("#DAB785"));
      painter.drawRect(fill_r);
    }
    if (bin.state == mpc::BinState::kNotApplicable)
    {
      painter.setPen(QPen(QColor("#9E9E9E"), 1));
      for (int x = r.left(); x < r.right(); x += 4)
        painter.drawLine(x, r.top(), x, r.bottom());
    }

    painter.setPen(Qt::black);
    painter.drawText(QRect(cx - bin_px_w, bar_top + bar_h + 2, bin_px_w * 2, 14), Qt::AlignCenter,
                      QString::number(bin.center, 'f', 2));
  }
}

// ===========================================================================
// FitPlotWidget
// ===========================================================================

FitPlotWidget::FitPlotWidget(QWidget *parent) : QWidget(parent)
{
  setMinimumSize(300, 180);
}

void FitPlotWidget::set_data(std::vector<Point> scatter, std::vector<Point> curve, double rms,
                              bool ok, std::string error, std::string x_label,
                              std::string y_label)
{
  scatter_ = std::move(scatter);
  curve_ = std::move(curve);
  rms_ = rms;
  ok_ = ok;
  error_ = std::move(error);
  x_label_ = std::move(x_label);
  y_label_ = std::move(y_label);
  update();
}

void FitPlotWidget::paintEvent(QPaintEvent * /*event*/)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rect(), Qt::white);

  if (scatter_.empty() && curve_.empty())
  {
    painter.setPen(Qt::darkGray);
    painter.drawText(rect(), Qt::AlignCenter, "(no fit yet -- click 'Fit & preview')");
    return;
  }

  double min_x = 0.0, max_x = 0.0, min_y = 0.0, max_y = 0.0;
  bool first = true;
  auto expand = [&](double x, double y)
  {
    if (first)
    {
      min_x = max_x = x;
      min_y = max_y = y;
      first = false;
      return;
    }
    min_x = std::min(min_x, x);
    max_x = std::max(max_x, x);
    min_y = std::min(min_y, y);
    max_y = std::max(max_y, y);
  };
  for (const auto &p : scatter_) expand(p.x, p.y);
  for (const auto &p : curve_) expand(p.x, p.y);
  const double span_x = std::max(max_x - min_x, 1e-6);
  const double span_y = std::max(max_y - min_y, 1e-6);

  const int padding = 24;
  const int text_h = 18;
  const double avail_w = std::max(width() - 2 * padding, 1);
  const double avail_h = std::max(height() - 2 * padding - text_h, 1);

  auto map = [&](double x, double y) -> QPointF
  {
    const double px = padding + (x - min_x) / span_x * avail_w;
    const double py = padding + avail_h - (y - min_y) / span_y * avail_h;
    return QPointF(px, py);
  };

  painter.setPen(Qt::lightGray);
  painter.drawRect(QRectF(padding, padding, avail_w, avail_h));

  painter.setPen(QPen(QColor("#56B4E9"), 2));
  for (size_t i = 1; i < curve_.size(); ++i)
    painter.drawLine(map(curve_[i - 1].x, curve_[i - 1].y), map(curve_[i].x, curve_[i].y));

  painter.setPen(QPen(Qt::black, 1));
  painter.setBrush(QColor("#D5896F"));
  for (const auto &p : scatter_)
  {
    const QPointF sp = map(p.x, p.y);
    painter.drawEllipse(sp, 2.5, 2.5);
  }

  painter.setPen(ok_ ? Qt::darkGreen : Qt::darkRed);
  QString status = ok_ ? QString("OK  rms=%1  n=%2 pts").arg(rms_, 0, 'f', 3).arg(scatter_.size())
                        : QString("FAILED: %1").arg(QString::fromStdString(error_));
  painter.drawText(QRect(padding, height() - text_h, width() - 2 * padding, text_h),
                    Qt::AlignLeft | Qt::AlignVCenter, status);

  painter.setPen(Qt::darkGray);
  painter.drawText(QRect(0, 0, width(), text_h), Qt::AlignCenter,
                    QString("%1 vs %2").arg(QString::fromStdString(x_label_))
                        .arg(QString::fromStdString(y_label_)));
}

// ===========================================================================
// CalibrationTab
// ===========================================================================

CalibrationTab::CalibrationTab(SimVizManager &manager, QWidget *parent)
    : QWidget(parent), manager_(manager), epoch_(std::chrono::steady_clock::now())
{
  setFocusPolicy(Qt::StrongFocus);
  build_ui();

  mocap_poll_timer_ = new QTimer(this);
  connect(mocap_poll_timer_, &QTimer::timeout, this,
          [this]()
          {
            if (!core_)
              return;
            const std::string robot = robot_combo_->currentText().toStdString();
            auto timed = manager_.mocap_manager().latest_timed_pose(robot);
            if (!timed)
              return;
            if (have_mocap_sample_ && timed->t <= last_mocap_sample_t_)
              return; // not a genuinely new sample.
            have_mocap_sample_ = true;
            last_mocap_sample_t_ = timed->t;
            last_mocap_wall_ = std::chrono::steady_clock::now();
            have_mocap_wall_ = true;
            feed_pose_for_test(now_s(), timed->pose.x, timed->pose.y, timed->pose.yaw);
          });
  mocap_poll_timer_->start(kMocapPollMs);

  resend_timer_ = new QTimer(this);
  connect(resend_timer_, &QTimer::timeout, this, &CalibrationTab::on_resend_timer);
  resend_timer_->start(kDutyResendMs);

  recompute_steering_notches();
  start_new_session();
}

CalibrationTab::~CalibrationTab()
{
  disable_teleop(); // safety net -- no-op if teleop was never enabled; see doc comment.
  teardown_worker();
}

double CalibrationTab::now_s() const
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now() - epoch_).count();
}

void CalibrationTab::build_ui()
{
  QVBoxLayout *root = new QVBoxLayout(this);

  // --- Connection strip. ---
  QGroupBox *conn_group = new QGroupBox("Connection", this);
  QGridLayout *conn_layout = new QGridLayout(conn_group);
  int row = 0;

  conn_layout->addWidget(new QLabel("Robot:", conn_group), row, 0);
  robot_combo_ = new QComboBox(conn_group);
  {
    std::vector<std::string> names;
    for (const auto &kv : manager_.config().real_robot_endpoints)
      names.push_back(kv.first);
    std::sort(names.begin(), names.end());
    if (names.empty())
      names = {"robot1", "robot2", "robot3", "robot4"};
    for (const auto &n : names)
      robot_combo_->addItem(QString::fromStdString(n));
    const int default_index = robot_combo_->findText("robot2");
    robot_combo_->setCurrentIndex(default_index >= 0 ? default_index : 0);
  }
  conn_layout->addWidget(robot_combo_, row, 1);

  conn_layout->addWidget(new QLabel("Host:", conn_group), row, 2);
  host_edit_ = new QLineEdit(conn_group);
  {
    const std::string robot = robot_combo_->currentText().toStdString();
    const auto it = manager_.config().real_robot_endpoints.find(robot);
    std::string host = (it != manager_.config().real_robot_endpoints.end())
                            ? parse_tcp_host(it->second.vesc_endpoint)
                            : "";
    if (host.empty())
      host = "127.0.0.1";
    host_edit_->setText(QString::fromStdString(host));
  }
  conn_layout->addWidget(host_edit_, row, 3);
  connect(robot_combo_, &QComboBox::currentTextChanged, this,
          [this](const QString &robot)
          {
            const auto it = manager_.config().real_robot_endpoints.find(robot.toStdString());
            if (it != manager_.config().real_robot_endpoints.end())
            {
              const std::string host = parse_tcp_host(it->second.vesc_endpoint);
              if (!host.empty())
                host_edit_->setText(QString::fromStdString(host));
            }
            start_new_session();
          });
  ++row;

  conn_layout->addWidget(new QLabel("Control port:", conn_group), row, 0);
  control_port_spin_ = new QSpinBox(conn_group);
  control_port_spin_->setRange(1, 65535);
  control_port_spin_->setValue(3460);
  conn_layout->addWidget(control_port_spin_, row, 1);

  conn_layout->addWidget(new QLabel("Telemetry port:", conn_group), row, 2);
  telemetry_port_spin_ = new QSpinBox(conn_group);
  telemetry_port_spin_->setRange(1, 65535);
  telemetry_port_spin_->setValue(3560);
  conn_layout->addWidget(telemetry_port_spin_, row, 3);
  ++row;

  connect_button_ = new QPushButton("Connect (ping)", conn_group);
  connect(connect_button_, &QPushButton::clicked, this, &CalibrationTab::on_connect_clicked);
  conn_layout->addWidget(connect_button_, row, 0);

  QHBoxLayout *lights_row = new QHBoxLayout();
  auto add_light = [&](const QString &label) -> QLabel *
  {
    lights_row->addWidget(new QLabel(label, conn_group));
    QLabel *dot = new QLabel(conn_group);
    dot->setFixedSize(12, 12);
    dot->setStyleSheet(dot_style(false));
    lights_row->addWidget(dot);
    return dot;
  };
  control_status_dot_ = add_light("REP");
  telemetry_status_dot_ = add_light("telem");
  mocap_status_dot_ = add_light("mocap");
  lights_row->addStretch(1);
  conn_layout->addLayout(lights_row, row, 1, 1, 3);
  ++row;

  v_in_label_ = new QLabel("v_in: -- V", conn_group);
  conn_layout->addWidget(v_in_label_, row, 0);
  current_label_ = new QLabel("motor current: -- A", conn_group);
  conn_layout->addWidget(current_label_, row, 1);
  conn_layout->addWidget(new QLabel("Current abort:", conn_group), row, 2);
  current_abort_spin_ = new QDoubleSpinBox(conn_group);
  current_abort_spin_->setRange(1.0, 100.0);
  current_abort_spin_->setValue(25.0);
  current_abort_spin_->setSuffix(" A");
  conn_layout->addWidget(current_abort_spin_, row, 3);
  ++row;

  abort_banner_ = new QLabel(conn_group);
  abort_banner_->setWordWrap(true);
  abort_banner_->setVisible(false);
  conn_layout->addWidget(abort_banner_, row, 0, 1, 4);

  root->addWidget(conn_group);

  // --- Task panel. ---
  QGroupBox *task_group = new QGroupBox("Task", this);
  QHBoxLayout *task_layout = new QHBoxLayout(task_group);
  task_layout->addWidget(new QLabel("Driving test:", task_group));
  task_combo_ = new QComboBox(task_group);
  task_combo_->addItem("Velocity map");
  task_combo_->addItem("Steering map");
  connect(task_combo_, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
          &CalibrationTab::on_task_changed);
  task_layout->addWidget(task_combo_);

  task_layout->addWidget(new QLabel("Notches:", task_group));
  notch_min_spin_ = new QDoubleSpinBox(task_group);
  notch_min_spin_->setRange(0.0, 1.0);
  notch_min_spin_->setDecimals(3);
  notch_min_spin_->setSingleStep(0.01);
  notch_min_spin_->setValue(0.05);
  task_layout->addWidget(notch_min_spin_);
  notch_max_spin_ = new QDoubleSpinBox(task_group);
  notch_max_spin_->setRange(0.0, 1.0);
  notch_max_spin_->setDecimals(3);
  notch_max_spin_->setSingleStep(0.01);
  notch_max_spin_->setValue(0.95);
  task_layout->addWidget(notch_max_spin_);
  notch_count_spin_ = new QSpinBox(task_group);
  notch_count_spin_->setRange(3, 41);
  notch_count_spin_->setValue(13);
  task_layout->addWidget(notch_count_spin_);
  connect(notch_min_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &CalibrationTab::on_notch_config_changed);
  connect(notch_max_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          &CalibrationTab::on_notch_config_changed);
  connect(notch_count_spin_, QOverload<int>::of(&QSpinBox::valueChanged), this,
          &CalibrationTab::on_notch_config_changed);

  load_session_button_ = new QPushButton("Load previous session", task_group);
  connect(load_session_button_, &QPushButton::clicked, this,
          &CalibrationTab::on_load_session_clicked);
  task_layout->addWidget(load_session_button_);
  task_layout->addStretch(1);
  root->addWidget(task_group);

  // --- Coverage panel. ---
  QGroupBox *coverage_group = new QGroupBox("Coverage", this);
  QVBoxLayout *coverage_layout = new QVBoxLayout(coverage_group);
  coverage_widget_ = new CoverageBinsWidget(coverage_group);
  coverage_layout->addWidget(coverage_widget_);
  instruction_label_ = new QLabel(coverage_group);
  instruction_label_->setWordWrap(true);
  QFont bold_font = instruction_label_->font();
  bold_font.setBold(true);
  instruction_label_->setFont(bold_font);
  coverage_layout->addWidget(instruction_label_);
  QHBoxLayout *coverage_stats_row = new QHBoxLayout();
  coverage_percent_label_ = new QLabel("Coverage: 0.0%", coverage_group);
  coverage_stats_row->addWidget(coverage_percent_label_);
  coverage_counts_label_ = new QLabel("Accepted 0 / Total 0", coverage_group);
  coverage_stats_row->addWidget(coverage_counts_label_);
  top_rejection_label_ = new QLabel("Top rejection: none", coverage_group);
  coverage_stats_row->addWidget(top_rejection_label_);
  coverage_stats_row->addStretch(1);
  coverage_layout->addLayout(coverage_stats_row);
  root->addWidget(coverage_group);

  // --- Teleop panel. ---
  QGroupBox *teleop_group = new QGroupBox("Teleop", this);
  QVBoxLayout *teleop_layout = new QVBoxLayout(teleop_group);
  QHBoxLayout *teleop_top_row = new QHBoxLayout();
  teleop_enable_checkbox_ = new QCheckBox("Enable teleop", teleop_group);
  connect(teleop_enable_checkbox_, &QCheckBox::toggled, this, &CalibrationTab::on_teleop_toggle);
  teleop_top_row->addWidget(teleop_enable_checkbox_);
  invert_drive_checkbox_ = new QCheckBox("Invert drive keys", teleop_group);
  teleop_top_row->addWidget(invert_drive_checkbox_);
  invert_steering_checkbox_ = new QCheckBox("Invert steering keys", teleop_group);
  teleop_top_row->addWidget(invert_steering_checkbox_);
  teleop_top_row->addStretch(1);
  teleop_layout->addLayout(teleop_top_row);

  legend_label_ = new QLabel(
      "W/S: forward/back (hold)   A/D: steer notch left/right   +/-: duty magnitude   "
      "SPACE/X: stop",
      teleop_group);
  teleop_layout->addWidget(legend_label_);

  QHBoxLayout *teleop_readout_row = new QHBoxLayout();
  duty_magnitude_label_ = new QLabel(teleop_group);
  teleop_readout_row->addWidget(duty_magnitude_label_);
  notch_label_ = new QLabel("notch +0", teleop_group);
  teleop_readout_row->addWidget(notch_label_);
  live_erpm_label_ = new QLabel("erpm: --", teleop_group);
  teleop_readout_row->addWidget(live_erpm_label_);
  live_mocap_v_label_ = new QLabel("mocap v: -- m/s", teleop_group);
  teleop_readout_row->addWidget(live_mocap_v_label_);
  teleop_readout_row->addStretch(1);
  teleop_layout->addLayout(teleop_readout_row);
  {
    std::ostringstream oss;
    oss << "duty magnitude: " << duty_magnitude_;
    duty_magnitude_label_->setText(QString::fromStdString(oss.str()));
  }
  root->addWidget(teleop_group);

  // --- Fit panel. ---
  QGroupBox *fit_group = new QGroupBox("Fit", this);
  QVBoxLayout *fit_layout = new QVBoxLayout(fit_group);
  fit_plot_widget_ = new FitPlotWidget(fit_group);
  fit_layout->addWidget(fit_plot_widget_);
  QHBoxLayout *fit_buttons_row = new QHBoxLayout();
  fit_button_ = new QPushButton("Fit && preview", fit_group);
  connect(fit_button_, &QPushButton::clicked, this, &CalibrationTab::on_fit_clicked);
  fit_buttons_row->addWidget(fit_button_);
  export_button_ = new QPushButton("Export", fit_group);
  connect(export_button_, &QPushButton::clicked, this, &CalibrationTab::on_export_clicked);
  fit_buttons_row->addWidget(export_button_);
  install_button_ = new QPushButton("Install to robot...", fit_group);
  connect(install_button_, &QPushButton::clicked, this, &CalibrationTab::on_install_clicked);
  fit_buttons_row->addWidget(install_button_);
  fit_buttons_row->addStretch(1);
  fit_layout->addLayout(fit_buttons_row);
  fit_status_label_ = new QLabel(fit_group);
  fit_status_label_->setWordWrap(true);
  fit_layout->addWidget(fit_status_label_);
  root->addWidget(fit_group);
}

// ---------------------------------------------------------------------
// Session / core lifecycle.
// ---------------------------------------------------------------------

std::string CalibrationTab::session_dir() const
{
  const std::string robot = robot_combo_->currentText().toStdString();
  const std::string date = QDateTime::currentDateTime().toString("yyyyMMdd").toStdString();
  return "results/robot_calib/" + date + "_" + robot;
}

std::string CalibrationTab::session_csv_path_for(mpc::Task task) const
{
  return session_dir() + (task == mpc::Task::kVelocity ? "/samples_velocity.csv"
                                                         : "/samples_steering.csv");
}

void CalibrationTab::recompute_steering_notches()
{
  const double lo = notch_min_spin_->value();
  const double hi = notch_max_spin_->value();
  const int n = std::max(2, notch_count_spin_->value());
  notches_.clear();
  notches_.reserve(static_cast<size_t>(n));
  for (int i = 0; i < n; ++i)
  {
    const double frac = (n == 1) ? 0.0 : static_cast<double>(i) / (n - 1);
    notches_.push_back(lo + frac * (hi - lo));
  }
  const int center = (n - 1) / 2;
  notch_index_ = 0;
  current_servo_value_ = notches_[static_cast<size_t>(center)];
  if (notch_label_)
    notch_label_->setText("notch +0");
}

void CalibrationTab::start_new_session()
{
  recompute_steering_notches();

  current_task_ = task_combo_->currentIndex() == 1 ? mpc::Task::kSteering : mpc::Task::kVelocity;

  core_cfg_ = mpc::CalibrationConfig{};
  core_cfg_.robot_name = robot_combo_->currentText().toStdString();
  core_cfg_.steering.notches = notches_;

  core_ = std::make_unique<mpc::CalibrationCore>(core_cfg_);

  const std::string dir = session_dir();
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  core_->set_session_csv_path(session_csv_path_for(current_task_), /*append=*/true);

  last_velocity_fit_.reset();
  last_steering_fit_.reset();
  if (fit_status_label_)
    fit_status_label_->setText("");
  if (fit_plot_widget_)
    fit_plot_widget_->set_data({}, {}, 0.0, false, "", "", "");

  update_instruction_and_coverage();
}

void CalibrationTab::on_task_changed(int /*index*/)
{
  start_new_session();
}

void CalibrationTab::on_notch_config_changed()
{
  start_new_session();
}

void CalibrationTab::on_load_session_clicked()
{
  if (!core_)
    return;
  const std::string path = session_csv_path_for(current_task_);
  std::string err;
  if (!core_->load_session_csv(path, &err))
  {
    QMessageBox::warning(this, "Calibration",
                          QString("Failed to load previous session '%1': %2")
                              .arg(QString::fromStdString(path))
                              .arg(QString::fromStdString(err)));
    return;
  }
  update_instruction_and_coverage();
}

void CalibrationTab::update_instruction_and_coverage()
{
  if (!core_)
    return;

  const mpc::CoverageReport report =
      current_task_ == mpc::Task::kVelocity ? core_->velocity_coverage() : core_->steering_coverage();
  coverage_widget_->set_data(report);
  instruction_label_->setText(QString::fromStdString(report.instruction.text));
  coverage_percent_label_->setText(QString("Coverage: %1%").arg(report.percent, 0, 'f', 1));
  coverage_counts_label_->setText(
      QString("Accepted %1 / Total %2").arg(core_->accepted_samples()).arg(core_->total_samples()));

  mpc::RejectReason top = mpc::RejectReason::kNone;
  int top_count = 0;
  for (const auto &kv : core_->rejection_counts())
  {
    if (kv.first == mpc::RejectReason::kNone)
      continue;
    if (kv.second > top_count)
    {
      top_count = kv.second;
      top = kv.first;
    }
  }
  top_rejection_label_->setText(top_count > 0
                                     ? QString("Top rejection: %1 (%2)")
                                           .arg(QString::fromStdString(mpc::to_string(top)))
                                           .arg(top_count)
                                     : QString("Top rejection: none"));
}

// ---------------------------------------------------------------------
// Ingestion (real worker signals AND test injection funnel through here).
// ---------------------------------------------------------------------

void CalibrationTab::feed_pose_for_test(double t, double x, double y, double yaw)
{
  if (have_prev_mocap_)
  {
    const double dt = t - prev_mocap_t_;
    if (dt > 1e-6)
    {
      const double dx = x - prev_mocap_x_;
      const double dy = y - prev_mocap_y_;
      const double v = std::sqrt(dx * dx + dy * dy) / dt;
      live_mocap_v_label_->setText(QString("mocap v: %1 m/s").arg(v, 0, 'f', 3));
    }
  }
  prev_mocap_x_ = x;
  prev_mocap_y_ = y;
  prev_mocap_t_ = t;
  have_prev_mocap_ = true;

  if (!core_)
    return;
  core_->feed_pose(t, x, y, yaw);
  update_instruction_and_coverage();
}

void CalibrationTab::feed_telemetry_for_test(double t, double erpm, double duty,
                                              double current_motor, double current_in,
                                              double v_in)
{
  last_telemetry_wall_ = std::chrono::steady_clock::now();
  have_telemetry_ever_ = true;
  last_v_in_ = v_in;
  last_current_motor_ = current_motor;
  last_erpm_ = erpm;
  Q_UNUSED(current_in);
  Q_UNUSED(duty);

  apply_current_abort_if_needed(current_motor);

  if (core_)
  {
    core_->feed_telemetry(t, erpm, duty, current_motor, v_in);
    update_instruction_and_coverage();
  }
}

void CalibrationTab::on_worker_telemetry(double t, double erpm, double duty,
                                          double current_motor, double current_in, double v_in)
{
  feed_telemetry_for_test(t, erpm, duty, current_motor, current_in, v_in);
}

void CalibrationTab::on_control_reply(QString cmd, bool ok, QString error)
{
  Q_UNUSED(cmd);
  Q_UNUSED(error);
  last_control_ok_ = ok;
  have_control_reply_ever_ = true;
}

// ---------------------------------------------------------------------
// Connection.
// ---------------------------------------------------------------------

void CalibrationTab::set_command_sink_for_test(ITeleopCommandSink *sink)
{
  test_sink_ = sink;
  sink_ = sink;
}

void CalibrationTab::teardown_worker()
{
  const bool sink_was_worker = (worker_ && sink_ == worker_.get());
  if (worker_thread_)
  {
    worker_thread_->quit();
    worker_thread_->wait();
    worker_thread_.reset();
  }
  worker_.reset();
  if (sink_was_worker)
    sink_ = test_sink_;
}

void CalibrationTab::on_connect_clicked()
{
  disable_teleop(); // safety net before swapping the sink out from under an active session.
  teardown_worker();

  if (test_sink_)
  {
    sink_ = test_sink_;
  }
  else
  {
    const std::string host = host_edit_->text().toStdString();
    const int control_port = control_port_spin_->value();
    const int telemetry_port = telemetry_port_spin_->value();
    const std::string robot = robot_combo_->currentText().toStdString();

    worker_ = std::make_unique<TeleopWorker>(host, control_port, telemetry_port, robot, epoch_);
    worker_thread_ = std::make_unique<QThread>();
    worker_->moveToThread(worker_thread_.get());
    connect(worker_thread_.get(), &QThread::started, worker_.get(), &TeleopWorker::start);
    connect(worker_.get(), &TeleopWorker::control_reply, this, &CalibrationTab::on_control_reply);
    connect(worker_.get(), &TeleopWorker::telemetry_frame, this,
            &CalibrationTab::on_worker_telemetry);
    worker_thread_->start();
    sink_ = worker_.get();
  }

  if (sink_)
    sink_->ping();
}

void CalibrationTab::refresh()
{
  update_connection_lights();

  if (!core_)
    return;
  const std::string robot = robot_combo_->currentText().toStdString();
  auto timed = manager_.mocap_manager().latest_timed_pose(robot);
  if (timed && (!have_mocap_sample_ || timed->t > last_mocap_sample_t_))
  {
    have_mocap_sample_ = true;
    last_mocap_sample_t_ = timed->t;
    last_mocap_wall_ = std::chrono::steady_clock::now();
    have_mocap_wall_ = true;
    feed_pose_for_test(now_s(), timed->pose.x, timed->pose.y, timed->pose.yaw);
  }
}

void CalibrationTab::update_connection_lights()
{
  control_status_dot_->setStyleSheet(dot_style(have_control_reply_ever_ && last_control_ok_));

  const auto now = std::chrono::steady_clock::now();
  const bool telemetry_fresh =
      have_telemetry_ever_ && std::chrono::duration<double>(now - last_telemetry_wall_).count() < 1.0;
  telemetry_status_dot_->setStyleSheet(dot_style(telemetry_fresh));

  const bool mocap_fresh =
      have_mocap_wall_ && std::chrono::duration<double>(now - last_mocap_wall_).count() < 1.0;
  mocap_status_dot_->setStyleSheet(dot_style(mocap_fresh));

  v_in_label_->setText(QString("v_in: %1 V").arg(last_v_in_, 0, 'f', 2));
  current_label_->setText(QString("motor current: %1 A").arg(last_current_motor_, 0, 'f', 2));
  live_erpm_label_->setText(QString("erpm: %1").arg(last_erpm_, 0, 'f', 0));
}

// ---------------------------------------------------------------------
// Teleop guard semantics (SAFETY -- see CalibrationTab.h's header comment).
// ---------------------------------------------------------------------

void CalibrationTab::on_teleop_toggle(bool checked)
{
  if (checked)
  {
    current_abort_tripped_ = false;
    abort_banner_->setVisible(false);
    enable_teleop();
  }
  else
  {
    disable_teleop();
  }
}

void CalibrationTab::enable_teleop()
{
  teleop_enabled_ = true;
  if (sink_)
  {
    sink_->ping();
    sink_->set_source("calib");
  }
  setFocus(Qt::OtherFocusReason);
}

void CalibrationTab::disable_teleop()
{
  if (!teleop_enabled_)
    return;
  teleop_enabled_ = false;
  forward_held_ = false;
  backward_held_ = false;
  current_signed_duty_ = 0.0;
  if (sink_)
    sink_->shutdown_blocking();
}

void CalibrationTab::apply_current_abort_if_needed(double current_motor)
{
  if (!teleop_enabled_ || current_abort_tripped_)
    return;
  const double threshold = current_abort_spin_->value();
  if (std::fabs(current_motor) < threshold)
    return;

  current_abort_tripped_ = true;
  send_stop_and_clear_drive_state();
  disable_teleop();
  {
    const QSignalBlocker blocker(teleop_enable_checkbox_);
    teleop_enable_checkbox_->setChecked(false);
  }
  abort_banner_->setText(QString("CURRENT ABORT: motor current %1 A reached the %2 A threshold -- "
                                  "teleop disabled. Re-enable to continue.")
                              .arg(current_motor, 0, 'f', 1)
                              .arg(threshold, 0, 'f', 1));
  abort_banner_->setStyleSheet("background-color:#D5896F; color:white; font-weight:bold; padding:4px;");
  abort_banner_->setVisible(true);
}

void CalibrationTab::hideEvent(QHideEvent *event)
{
  if (teleop_enabled_)
  {
    disable_teleop();
    const QSignalBlocker blocker(teleop_enable_checkbox_);
    teleop_enable_checkbox_->setChecked(false);
  }
  QWidget::hideEvent(event);
}

// ---------------------------------------------------------------------
// Keyboard teleop.
// ---------------------------------------------------------------------

void CalibrationTab::send_current_duty()
{
  const int sign_mult = invert_drive_checkbox_->isChecked() ? -1 : 1;
  double value = 0.0;
  if (forward_held_)
    value = sign_mult * duty_magnitude_;
  else if (backward_held_)
    value = -sign_mult * duty_magnitude_;
  current_signed_duty_ = value;

  if (sink_)
    sink_->raw_duty(value, kDutyTtlMs);
  if (core_)
    core_->feed_command(now_s(), "duty", value, current_servo_value_);
}

void CalibrationTab::send_stop_and_clear_drive_state()
{
  forward_held_ = false;
  backward_held_ = false;
  current_signed_duty_ = 0.0;
  if (sink_)
    sink_->stop();
  if (core_)
    core_->feed_command(now_s(), "duty", 0.0, current_servo_value_);
}

void CalibrationTab::step_notch(int delta)
{
  if (notches_.empty())
    return;
  const int count = static_cast<int>(notches_.size());
  const int center = (count - 1) / 2;
  int abs_index = std::clamp(center + notch_index_ + delta, 0, count - 1);
  notch_index_ = abs_index - center;
  current_servo_value_ = notches_[static_cast<size_t>(abs_index)];
  notch_label_->setText(
      QString("notch %1%2").arg(notch_index_ >= 0 ? "+" : "").arg(notch_index_));

  if (sink_)
    sink_->servo(current_servo_value_);
  if (core_)
    core_->feed_command(now_s(), "duty", current_signed_duty_, current_servo_value_);
}

void CalibrationTab::on_resend_timer()
{
  if (!teleop_enabled_)
    return;
  if (forward_held_ || backward_held_)
    send_current_duty();
}

void CalibrationTab::keyPressEvent(QKeyEvent *event)
{
  if (event->isAutoRepeat())
  {
    event->ignore();
    return;
  }
  if (!teleop_enabled_)
  {
    QWidget::keyPressEvent(event);
    return;
  }

  switch (event->key())
  {
  case Qt::Key_W:
    forward_held_ = true;
    backward_held_ = false;
    send_current_duty();
    break;
  case Qt::Key_S:
    backward_held_ = true;
    forward_held_ = false;
    send_current_duty();
    break;
  case Qt::Key_A:
    step_notch(invert_steering_checkbox_->isChecked() ? +1 : -1);
    break;
  case Qt::Key_D:
    step_notch(invert_steering_checkbox_->isChecked() ? -1 : +1);
    break;
  case Qt::Key_Plus:
  case Qt::Key_Equal:
    duty_magnitude_ = std::min(duty_magnitude_ + 0.005, 0.08);
    duty_magnitude_label_->setText(QString("duty magnitude: %1").arg(duty_magnitude_, 0, 'f', 3));
    if (forward_held_ || backward_held_)
      send_current_duty();
    break;
  case Qt::Key_Minus:
    duty_magnitude_ = std::max(duty_magnitude_ - 0.005, 0.0);
    duty_magnitude_label_->setText(QString("duty magnitude: %1").arg(duty_magnitude_, 0, 'f', 3));
    if (forward_held_ || backward_held_)
      send_current_duty();
    break;
  case Qt::Key_Space:
  case Qt::Key_X:
    send_stop_and_clear_drive_state();
    break;
  default:
    QWidget::keyPressEvent(event);
    return;
  }
  event->accept();
}

void CalibrationTab::keyReleaseEvent(QKeyEvent *event)
{
  if (event->isAutoRepeat())
  {
    event->ignore();
    return;
  }
  if (!teleop_enabled_)
  {
    QWidget::keyReleaseEvent(event);
    return;
  }

  if (event->key() == Qt::Key_W && forward_held_)
  {
    send_stop_and_clear_drive_state();
    event->accept();
    return;
  }
  if (event->key() == Qt::Key_S && backward_held_)
  {
    send_stop_and_clear_drive_state();
    event->accept();
    return;
  }
  QWidget::keyReleaseEvent(event);
}

// ---------------------------------------------------------------------
// Fit / export / install.
// ---------------------------------------------------------------------

void CalibrationTab::on_fit_clicked()
{
  if (!core_)
    return;

  if (current_task_ == mpc::Task::kVelocity)
  {
    const mpc::VelocityFitResult fit = core_->fit_velocity_map();
    last_velocity_fit_ = fit;

    std::vector<FitPlotWidget::Point> scatter;
    for (const auto &bin : core_->velocity_bin_samples())
      for (const auto &s : bin)
        scatter.push_back({s.v, s.erpm});
    std::vector<FitPlotWidget::Point> curve;
    for (const auto &pt : fit.table)
      curve.push_back({pt.v, pt.erpm});
    fit_plot_widget_->set_data(scatter, curve, fit.rms, fit.ok, fit.error, "v (m/s)", "erpm");

    fit_status_label_->setText(
        fit.ok ? QString("Velocity fit OK: rms=%1 erpm, n=%2, min_reliable_erpm=%3")
                     .arg(fit.rms, 0, 'f', 1)
                     .arg(fit.n_samples)
                     .arg(fit.min_reliable_erpm, 0, 'f', 0)
               : QString("Velocity fit FAILED: %1").arg(QString::fromStdString(fit.error)));
  }
  else
  {
    const double wheel_base = core_cfg_.wheel_base;
    const mpc::SteeringFitResult fit = core_->fit_steering_map(wheel_base);
    last_steering_fit_ = fit;

    std::vector<FitPlotWidget::Point> scatter;
    for (const auto &notch : core_->steering_notch_samples())
      for (const auto &s : notch)
        if (std::fabs(s.v) > 1e-6)
          scatter.push_back({s.servo_cmd, std::atan(wheel_base * s.omega / s.v)});
    std::vector<FitPlotWidget::Point> curve;
    for (const auto &pt : fit.points)
      curve.push_back({pt.servo, pt.delta});
    fit_plot_widget_->set_data(scatter, curve, fit.residual_rms, fit.ok, fit.error, "servo",
                                "delta (rad)");

    fit_status_label_->setText(
        fit.ok ? QString("Steering fit OK: residual_rms=%1 rad, n=%2, delta range=[%3, %4]")
                     .arg(fit.residual_rms, 0, 'f', 3)
                     .arg(fit.n_samples)
                     .arg(fit.delta_min, 0, 'f', 3)
                     .arg(fit.delta_max, 0, 'f', 3)
               : QString("Steering fit FAILED: %1").arg(QString::fromStdString(fit.error)));
  }
}

void CalibrationTab::on_export_clicked()
{
  const std::string robot = robot_combo_->currentText().toStdString();
  const std::string dir = session_dir();
  std::error_code ec;
  std::filesystem::create_directories(dir, ec);
  std::string err;

  if (current_task_ == mpc::Task::kVelocity)
  {
    if (!last_velocity_fit_ || !last_velocity_fit_->ok)
    {
      QMessageBox::warning(this, "Calibration", "Run 'Fit & preview' first (velocity fit not OK).");
      return;
    }
    const std::string path = dir + "/velocity_calib.json";
    if (!mpc::export_velocity_calib(*last_velocity_fit_, robot, path, &err))
    {
      QMessageBox::warning(this, "Calibration",
                            QString("Export failed: %1").arg(QString::fromStdString(err)));
      return;
    }
    fit_status_label_->setText(QString("Exported: %1").arg(QString::fromStdString(path)));
  }
  else
  {
    if (!last_steering_fit_ || !last_steering_fit_->ok)
    {
      QMessageBox::warning(this, "Calibration", "Run 'Fit & preview' first (steering fit not OK).");
      return;
    }
    const std::string path = dir + "/steering_angle_map.json";
    if (!mpc::export_steering_map(*last_steering_fit_, robot, core_cfg_.wheel_base, path, &err))
    {
      QMessageBox::warning(this, "Calibration",
                            QString("Export failed: %1").arg(QString::fromStdString(err)));
      return;
    }
    fit_status_label_->setText(QString("Exported: %1").arg(QString::fromStdString(path)));
  }
}

void CalibrationTab::on_install_clicked()
{
  // IMPLEMENTED, deliberately never invoked by anything other than a real
  // user click -- see this file's header comment / the task spec's "never
  // execute it yourself" instruction. The offscreen test does not click
  // this button.
  const auto reply = QMessageBox::question(
      this, "Install to robot",
      "This will scp the exported calibration file to the robot's ~/.vesc/ directory over the "
      "network. Continue?",
      QMessageBox::Yes | QMessageBox::No);
  if (reply != QMessageBox::Yes)
    return;

  const std::string dir = session_dir();
  const std::string filename =
      current_task_ == mpc::Task::kVelocity ? "velocity_calib.json" : "steering_angle_map.json";
  const std::string local_path = dir + "/" + filename;
  if (!std::filesystem::exists(local_path))
  {
    QMessageBox::warning(this, "Install to robot", "Export first -- no exported file found.");
    return;
  }

  const std::string host = host_edit_->text().toStdString();
  const QString remote = QString("robot@%1:~/.vesc/").arg(QString::fromStdString(host));

  QProcess *proc = new QProcess(this);
  connect(proc, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished), this,
          [this, proc](int exit_code, QProcess::ExitStatus status)
          {
            fit_status_label_->setText(exit_code == 0 && status == QProcess::NormalExit
                                            ? "Install to robot: scp completed."
                                            : "Install to robot: scp FAILED.");
            proc->deleteLater();
          });
  proc->start("scp", {QString::fromStdString(local_path), remote});
}

} // namespace simviz
