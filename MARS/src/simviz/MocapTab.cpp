#include "MocapTab.h"

#include <QAbstractItemView>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFont>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLineEdit>
#include <QMessageBox>
#include <QPainter>
#include <QPen>
#include <QPlainTextEdit>
#include <QPolygonF>
#include <QSignalBlocker>
#include <QSplitter>
#include <QTransform>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace simviz
{

namespace
{

const QColor kBodyDotColors[] = {
    QColor("#70A288"), QColor("#DAB785"), QColor("#D5896F"),
    QColor("#CC79A7"), QColor("#E69F00"), QColor("#56B4E9"),
};
constexpr int kBodyDotColorCount = 6;

const QColor kPlannerAxisXColor("#D5896F");
const QColor kPlannerAxisYColor("#70A288");
const QColor kMotiveAxisXColor("#CC79A7");
const QColor kMotiveAxisYColor("#56B4E9"); // Motive's +Z in-plane arrow (this glyph's "other" axis).

// MAPPING DROPDOWN: the ScenarioLess mapping table's planner-name column
// (rebuild_mapping_table() below) used to be free-text -- fixed to this
// project's actual planner robot names instead, index 0 ("") meaning
// "unmapped" (same convention the Scenario-mode "Mapped body" combo already
// uses for its own index-0 "(none)" entry). Kept as a single named constant
// -- rather than inlined at each of rebuild_mapping_table()'s/
// on_save_mapping()'s use sites -- so the fixed robot1..robot4 set only
// needs to change in one place if this project's fleet size ever does.
const char *const kFixedPlannerNames[] = {"", "robot1", "robot2", "robot3", "robot4"};

QString state_color_name(MocapState state)
{
  switch (state)
  {
  case MocapState::Disconnected:
    return "#9E9E9E"; // gray
  case MocapState::Connecting:
    return "#F2C14E"; // yellow
  case MocapState::Connected:
    return "#43A047"; // green
  case MocapState::Error:
    return "#D5896F"; // red (project palette, not pure #F00)
  }
  return "#9E9E9E";
}

QString format_hms(double seconds)
{
  if (seconds < 0.0)
    return "--:--";
  const int total = static_cast<int>(seconds);
  const int m = (total / 60) % 60;
  const int h = total / 3600;
  const int s = total % 60;
  QString out;
  if (h > 0)
    out = QString("%1:%2:%3").arg(h).arg(m, 2, 10, QChar('0')).arg(s, 2, 10, QChar('0'));
  else
    out = QString("%1:%2").arg(m, 2, 10, QChar('0')).arg(s, 2, 10, QChar('0'));
  return out;
}

// Reverse-looks-up the Motive name whose alias VALUE equals `target` in
// `mapping`'s "aliases" object (motive_name -> planner_name), or empty if
// none does.
std::string reverse_alias_lookup(const nlohmann::json &mapping, const std::string &target)
{
  if (mapping.contains("aliases") && mapping["aliases"].is_object())
  {
    for (auto it = mapping["aliases"].begin(); it != mapping["aliases"].end(); ++it)
    {
      if (it.value().is_string() && it.value().get<std::string>() == target)
        return it.key();
    }
  }
  return std::string();
}

// Small oriented-arrow polygon, ported from SimVizWindow.cpp's
// draw_oriented_arrow() (itself ported from VisualizationWidget.cpp's
// drawOrientedArrow) -- same tail-to-head triangle-with-notch shape and the
// same "-yaw_deg" rotation convention, valid here because MocapTransformWidget's
// own `map` lambda (see paintEvent() below) Y-flips exactly like
// SimVizWindow.cpp's map_coord() does. Unlike that ported original (which
// takes a `scale` MULTIPLIER on a fixed 14px base size), this takes an
// ABSOLUTE pixel length for the arrow's head-to-tail span, so callers can
// world-scale it (see paintEvent()'s arrow_len_px, ~0.15m clamped to a
// minimum on-screen size).
void draw_oriented_arrow(QPainter &painter, const QPointF &position, double yaw,
                          const QColor &color, double arrow_length_px)
{
  const double arrow_width = arrow_length_px * 0.5;
  const double tail_length = arrow_length_px * (6.0 / 14.0);

  QPolygonF arrow;
  arrow << QPointF(-tail_length, -arrow_width / 2) << QPointF(0, -arrow_width / 2)
        << QPointF(0, -arrow_width) << QPointF(arrow_length_px, 0) << QPointF(0, arrow_width)
        << QPointF(0, arrow_width / 2) << QPointF(-tail_length, arrow_width / 2);

  QTransform transform;
  const double yaw_deg = yaw * 180.0 / M_PI;
  transform.translate(position.x(), position.y());
  transform.rotate(-yaw_deg);
  const QPolygonF transformed = transform.map(arrow);

  painter.setBrush(QBrush(color));
  painter.setPen(QPen(Qt::black, 1));
  painter.drawPolygon(transformed);
}

constexpr double kBodyArrowLengthM = 0.15;
constexpr double kBodyArrowMinPx = 10.0;

} // namespace

// ===========================================================================
// MocapTransformWidget
// ===========================================================================

MocapTransformWidget::MocapTransformWidget(QWidget *parent) : QWidget(parent)
{
  setMinimumSize(260, 220);
}

void MocapTransformWidget::set_data(const TransformInfo &info, std::vector<BodyPoint> bodies)
{
  info_ = info;
  bodies_ = std::move(bodies);
  update();
}

void MocapTransformWidget::paintEvent(QPaintEvent * /*event*/)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rect(), Qt::white);

  // Data-derived logical viewport: planner origin + Motive origin (via the
  // matrix) + every live body position, inflated by a margin -- see this
  // class's header comment for why there is no fixed workspace rect to
  // anchor to here.
  double min_x = -1.0, max_x = 1.0, min_y = -1.0, max_y = 1.0;
  auto expand = [&](double x, double y)
  {
    min_x = std::min(min_x, x);
    max_x = std::max(max_x, x);
    min_y = std::min(min_y, y);
    max_y = std::max(max_y, y);
  };
  expand(0.0, 0.0);
  expand(info_.matrix.tx, info_.matrix.ty);
  for (const auto &b : bodies_)
    expand(b.pose.x, b.pose.y);

  const double span = std::max({max_x - min_x, max_y - min_y, 0.5});
  const double margin = span * 0.3;
  min_x -= margin;
  max_x += margin;
  min_y -= margin;
  max_y += margin;

  const double workspace_w = std::max(max_x - min_x, 1e-3);
  const double workspace_h = std::max(max_y - min_y, 1e-3);

  const int padding = 14;
  const double avail_w = std::max(width() - 2 * padding, 1);
  const double avail_h = std::max(height() - 2 * padding, 1);
  const double aspect = workspace_w / workspace_h;
  const double avail_aspect = avail_w / avail_h;

  double draw_w, draw_h, rect_x, rect_y;
  if (avail_aspect > aspect)
  {
    draw_h = avail_h;
    draw_w = aspect * draw_h;
    rect_x = (width() - draw_w) / 2.0;
    rect_y = padding;
  }
  else
  {
    draw_w = avail_w;
    draw_h = draw_w / aspect;
    rect_x = padding;
    rect_y = (height() - draw_h) / 2.0;
  }
  const double scale = std::min(draw_w / workspace_w, draw_h / workspace_h);
  const double drawn_w = workspace_w * scale;
  const double drawn_h = workspace_h * scale;
  const double offset_x = rect_x + (draw_w - drawn_w) / 2.0;
  const double offset_y = rect_y + (draw_h - drawn_h) / 2.0;

  auto map = [&](double x, double y) -> QPointF
  {
    return QPointF(offset_x + (x - min_x) * scale, offset_y + drawn_h - (y - min_y) * scale);
  };

  const double axis_len = std::min(workspace_w, workspace_h) * 0.15;

  // Planner frame axes at its origin.
  const QPointF planner_origin = map(0.0, 0.0);
  painter.setPen(QPen(kPlannerAxisXColor, 2));
  painter.drawLine(planner_origin, map(axis_len, 0.0));
  painter.setPen(QPen(kPlannerAxisYColor, 2));
  painter.drawLine(planner_origin, map(0.0, axis_len));
  painter.setPen(Qt::black);
  painter.drawText(planner_origin + QPointF(4, 14), "planner");

  // Motive frame axes: Motive's OWN FIXED viewport convention -- +X right,
  // +Z down-screen, +Y the up/out-of-plane axis -- drawn INDEPENDENTLY of
  // info_.y_up (that flag governs how live POSE DATA is mapped via
  // mocap_rotation(), not how Motive's own axes are drawn; see this class's
  // header doc comment). ONLY the calibration matrix (info_.matrix --
  // theta/translation from the UI) orients/places these; mocap_rotation()'s
  // y_up-conditional flip is deliberately NOT applied here anymore (PRE-
  // round behavior: this glyph's shape used to change depending on the
  // y_up checkbox, which was confusing since y_up doesn't change where
  // Motive's OWN axes point in Motive's own viewport).
  auto motive_point = [&](double mx, double my) -> QPointF
  {
    return QPointF(info_.matrix.a * mx + info_.matrix.b * my + info_.matrix.tx,
                    info_.matrix.c * mx + info_.matrix.d * my + info_.matrix.ty);
  };
  const QPointF motive_origin_data = motive_point(0.0, 0.0);
  const QPointF motive_x_data = motive_point(axis_len, 0.0); // Motive's +X.
  // Motive's +Z is always the in-plane "other" axis now (not conditional on
  // y_up) -- local my is NEGATIVE so that, per map()'s Y-flipped screen
  // convention (larger data-y draws HIGHER on screen -- see map()'s own
  // definition above: offset_y + drawn_h - (y - min_y) * scale), an
  // un-rotated (theta=0) calibration draws it visibly BELOW the origin on
  // screen, matching the user's own spec ("with Motive is Y-up unchecked,
  // the z axis should point toward down with 0 rotation").
  const QPointF motive_z_data = motive_point(0.0, -axis_len);
  const QPointF motive_origin = map(motive_origin_data.x(), motive_origin_data.y());

  QPen motive_x_pen(kMotiveAxisXColor);
  motive_x_pen.setStyle(Qt::DashLine);
  motive_x_pen.setWidth(2);
  painter.setPen(motive_x_pen);
  const QPointF motive_x_screen = map(motive_x_data.x(), motive_x_data.y());
  painter.drawLine(motive_origin, motive_x_screen);
  painter.drawText(motive_x_screen + QPointF(4, -4), "Motive +X");

  QPen motive_z_pen(kMotiveAxisYColor);
  motive_z_pen.setStyle(Qt::DashLine);
  motive_z_pen.setWidth(2);
  painter.setPen(motive_z_pen);
  const QPointF motive_z_screen = map(motive_z_data.x(), motive_z_data.y());
  painter.drawLine(motive_origin, motive_z_screen);
  painter.drawText(motive_z_screen + QPointF(4, -4), "Motive +Z");

  painter.setPen(Qt::black);
  painter.drawText(motive_origin + QPointF(4, 14), "Motive origin");

  // Identity matrix -> frames coincide (expected until calibrated).
  const bool near_identity =
      std::abs(info_.matrix.a - 1.0) < 1e-6 && std::abs(info_.matrix.b) < 1e-6 &&
      std::abs(info_.matrix.c) < 1e-6 && std::abs(info_.matrix.d - 1.0) < 1e-6 &&
      std::abs(info_.matrix.tx) < 1e-6 && std::abs(info_.matrix.ty) < 1e-6;
  if (near_identity)
  {
    painter.setPen(Qt::darkGray);
    painter.drawText(QRect(padding, padding, width() - 2 * padding, 18), Qt::AlignLeft,
                      "identity / uncalibrated");
  }

  // Live mapped body positions (already planner-frame -- see this class's
  // header comment), drawn as ORIENTED ARROWS (not plain dots) so heading
  // is visible too -- world-scaled to ~0.15m via this widget's own
  // fit-to-viewport `scale`, clamped to a minimum on-screen size so a body
  // stays visible even when the viewport is zoomed far out.
  const double arrow_len_px = std::max(kBodyArrowLengthM * scale, kBodyArrowMinPx);
  for (size_t i = 0; i < bodies_.size(); ++i)
  {
    const QColor color = kBodyDotColors[i % kBodyDotColorCount];
    const QPointF p = map(bodies_[i].pose.x, bodies_[i].pose.y);
    draw_oriented_arrow(painter, p, bodies_[i].pose.yaw, color, arrow_len_px);
    painter.setPen(Qt::black);
    painter.drawText(p + QPointF(7, -6), QString::fromStdString(bodies_[i].name));
  }
}

// ===========================================================================
// MocapTab
// ===========================================================================

MocapTab::MocapTab(SimVizManager &manager, QWidget *parent) : QWidget(parent), manager_(manager)
{
  QVBoxLayout *root = new QVBoxLayout(this);

  // --- Top row: indicator + state text + uptime + connect button. ---
  QHBoxLayout *top_row = new QHBoxLayout();
  indicator_ = new QLabel(this);
  indicator_->setFixedSize(16, 16);
  top_row->addWidget(indicator_);
  state_label_ = new QLabel("Disconnected", this);
  top_row->addWidget(state_label_);
  uptime_label_ = new QLabel(this);
  top_row->addWidget(uptime_label_);
  top_row->addStretch(1);
  connect_button_ = new QPushButton("Connect", this);
  connect(connect_button_, &QPushButton::clicked, this, &MocapTab::on_connect_toggle);
  top_row->addWidget(connect_button_);
  root->addLayout(top_row);

  QSplitter *splitter = new QSplitter(Qt::Horizontal, this);

  // --- Left: live rigid-body list. ---
  QWidget *left_panel = new QWidget(splitter);
  QVBoxLayout *left_layout = new QVBoxLayout(left_panel);
  left_layout->addWidget(new QLabel("Rigid bodies", left_panel));
  // 12 columns: identity/age, then the STAGED TRANSFORM READOUT in pipeline
  // order -- raw Motive -> after mocap rotation -> after calibration
  // (+yaw_offset, the live published pose) -- see update_body_table()'s
  // doc comment.
  body_table_ = new QTableWidget(0, 12, left_panel);
  body_table_->setHorizontalHeaderLabels({"Motive", "id", "age (s)", "raw x", "raw y/z", "raw yaw",
                                           "mocap x", "mocap y", "mocap yaw", "x", "y", "yaw"});
  body_table_->horizontalHeader()->setStretchLastSection(true);
  body_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  left_layout->addWidget(body_table_);
  splitter->addWidget(left_panel);

  // --- Middle: mapping table + save/move buttons. ---
  QWidget *mid_panel = new QWidget(splitter);
  QVBoxLayout *mid_layout = new QVBoxLayout(mid_panel);
  mid_layout->addWidget(new QLabel("Robot mapping", mid_panel));
  mapping_table_ = new QTableWidget(0, 5, mid_panel);
  mapping_table_->setHorizontalHeaderLabels(
      {"Robot", "Mapped body", "Yaw offset", "VESC endpoint", "Fresh"});
  mapping_table_->horizontalHeader()->setStretchLastSection(true);
  mid_layout->addWidget(mapping_table_);

  QHBoxLayout *mapping_buttons_row = new QHBoxLayout();
  save_mapping_button_ = new QPushButton("Save Mapping", mid_panel);
  connect(save_mapping_button_, &QPushButton::clicked, this, &MocapTab::on_save_mapping);
  mapping_buttons_row->addWidget(save_mapping_button_);
  mapping_buttons_row->addStretch(1);
  mid_layout->addLayout(mapping_buttons_row);
  mapping_status_label_ = new QLabel(mid_panel);
  mapping_status_label_->setWordWrap(true);
  mid_layout->addWidget(mapping_status_label_);

  move_to_initial_button_ = new QPushButton("Move robots to initial poses", mid_panel);
  connect(move_to_initial_button_, &QPushButton::clicked, this,
          &MocapTab::on_move_to_initial_poses);
  mid_layout->addWidget(move_to_initial_button_);
  stage_real_status_label_ = new QLabel(mid_panel);
  stage_real_status_label_->setWordWrap(true);
  mid_layout->addWidget(stage_real_status_label_);

  splitter->addWidget(mid_panel);

  // --- Right: transform visual + numeric panel. ---
  QWidget *right_panel = new QWidget(splitter);
  QVBoxLayout *right_layout = new QVBoxLayout(right_panel);
  right_layout->addWidget(new QLabel("Motive <-> planner transform", right_panel));
  transform_widget_ = new MocapTransformWidget(right_panel);
  right_layout->addWidget(transform_widget_, /*stretch=*/1);
  transform_numbers_label_ = new QLabel(right_panel);
  transform_numbers_label_->setWordWrap(true);
  QFont mono("Monospace");
  mono.setStyleHint(QFont::TypeWriter);
  transform_numbers_label_->setFont(mono);
  right_layout->addWidget(transform_numbers_label_);

  // --- Calibration group box (part B): Motive-planar -> planner
  // calibration, typed fields matching apply_planar_transform()'s exact
  // composition (see on_apply_calibration()'s doc comment). ---
  QGroupBox *calib_group = new QGroupBox("Calibration", right_panel);
  QFormLayout *calib_form = new QFormLayout(calib_group);

  calib_tx_spin_ = new QDoubleSpinBox(calib_group);
  calib_tx_spin_->setRange(-1000.0, 1000.0);
  calib_tx_spin_->setDecimals(3);
  calib_tx_spin_->setSuffix(" m");
  calib_form->addRow("Translation X:", calib_tx_spin_);

  calib_ty_spin_ = new QDoubleSpinBox(calib_group);
  calib_ty_spin_->setRange(-1000.0, 1000.0);
  calib_ty_spin_->setDecimals(3);
  calib_ty_spin_->setSuffix(" m");
  calib_form->addRow("Translation Y:", calib_ty_spin_);

  calib_theta_deg_spin_ = new QDoubleSpinBox(calib_group);
  calib_theta_deg_spin_->setRange(-360.0, 360.0);
  calib_theta_deg_spin_->setDecimals(2);
  calib_theta_deg_spin_->setSuffix(" deg");
  calib_form->addRow("Rotation:", calib_theta_deg_spin_);

  calib_y_up_checkbox_ = new QCheckBox("Motive is Y-up", calib_group);
  calib_form->addRow(calib_y_up_checkbox_);

  // BUG FIX (stale-state clobber): mark each field dirty ONLY on a live
  // user-driven signal -- load_calibration_fields() wraps its own
  // setValue()/setChecked() calls in a QSignalBlocker specifically so THIS
  // connection never fires for a programmatic reload (see MocapTab.h's
  // calib_*_dirty_ doc comment and on_apply_calibration()'s below).
  connect(calib_tx_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          [this]() { calib_tx_dirty_ = true; });
  connect(calib_ty_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          [this]() { calib_ty_dirty_ = true; });
  connect(calib_theta_deg_spin_, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
          [this]() { calib_theta_dirty_ = true; });
  connect(calib_y_up_checkbox_, &QCheckBox::toggled, this,
          [this]() { calib_y_up_dirty_ = true; });

  calib_apply_button_ = new QPushButton("Apply (restarts bridge)", calib_group);
  calib_apply_button_->setToolTip(
      "Writes translation X/Y, rotation, and the Y-up flag to the map-config as "
      "x0/y0/theta0/y_up, and REMOVES any mocap_to_world_matrix (it would otherwise "
      "supersede these values -- see the map-config's own _doc). If the OptiTrack "
      "bridge is currently running, it is stopped and restarted so the new "
      "calibration takes effect immediately; otherwise the file is just saved.");
  connect(calib_apply_button_, &QPushButton::clicked, this, &MocapTab::on_apply_calibration);
  calib_form->addRow(calib_apply_button_);

  calib_status_label_ = new QLabel(calib_group);
  calib_status_label_->setWordWrap(true);
  calib_form->addRow(calib_status_label_);

  right_layout->addWidget(calib_group);

  splitter->addWidget(right_panel);

  splitter->setStretchFactor(0, 1);
  splitter->setStretchFactor(1, 1);
  splitter->setStretchFactor(2, 1);
  root->addWidget(splitter, /*stretch=*/1);

  update_indicator();
  update_body_table();
  rebuild_mapping_table();
  update_move_button_enabled();
  update_transform_panel();
  load_calibration_fields();
}

void MocapTab::refresh()
{
  update_indicator();
  update_body_table();
  rebuild_mapping_table();
  update_mapping_table_live_columns();
  update_move_button_enabled();
  update_transform_panel();

  // stage_real_progress_text() is only non-empty while a STAGE_REAL
  // sequence is active (see its doc comment in SimVizCore.h) -- cleared
  // back to "" the moment the sequence ends (success or failure), matching
  // exec_mgr_'s own Idle/Running/Done/Err readout in the Simulation tab's
  // status bar for a completed/failed leg.
  if (manager_.is_stage_real_active())
  {
    stage_real_status_label_->setText(
        QString("Moving: %1").arg(QString::fromStdString(manager_.stage_real_progress_text())));
  }
  else
  {
    stage_real_status_label_->setText("");
  }
}

void MocapTab::update_indicator()
{
  const MocapState state = manager_.mocap_manager().state();

  QString style = QString("background-color: %1; border-radius: 8px; border: 1px solid black;")
                      .arg(state_color_name(state));
  indicator_->setStyleSheet(style);

  QString text = mocap_state_name(state);
  if (state == MocapState::Error)
  {
    const std::string reason = manager_.mocap_manager().error_reason();
    if (!reason.empty())
      text += QString(" (%1)").arg(QString::fromStdString(reason));
  }
  state_label_->setText(text);

  // Uptime bookkeeping: starts counting the first time we observe a
  // non-Disconnected state after having been Disconnected (i.e. right after
  // a successful Connect), stops/resets back to Disconnected.
  if (state == MocapState::Disconnected)
  {
    uptime_running_ = false;
    uptime_label_->setText("");
  }
  else
  {
    if (!uptime_running_)
    {
      uptime_timer_.start();
      uptime_running_ = true;
    }
    uptime_label_->setText(QString("uptime %1").arg(format_hms(uptime_timer_.elapsed() / 1000.0)));
  }
  last_seen_state_ = state;

  connect_button_->setText(state == MocapState::Disconnected || state == MocapState::Error
                                ? "Connect"
                                : "Disconnect");
}

// BUG report: this used to key rows off MocapManager::bodies() (the live
// published-name-keyed topic list) and GUESS at a "Motive" column value via
// reverse-alias-lookup-or-passthrough -- correct only when a body was
// actually aliased (published_name then legitimately equals the alias'
// planner name) OR when the body's real Motive name happened to already be
// topic-safe (auto-discovery's sanitize_topic_name() is a no-op for it).
// Any OTHER Motive name (e.g. containing a space) was shown SANITIZED, not
// as Motive actually names it. Now keyed off motive_assets() (the modeldef
// inventory) instead, so the name column is always the real Motive name;
// live pose/freshness for each asset is cross-referenced via
// expected_published_name() (best-effort, see its own doc comment -- pose
// display only, never identity).
//
// STAGED TRANSFORM READOUT (part C of this task -- the user's own words:
// "rethink" the OptiTrack calibration UI so the pipeline is visible, not a
// black box): columns 3-8 show the RAW Motive-frame values and the
// intermediate MOCAP-ROTATION-stage values, reconstructed from the live
// published pose (columns 9-11, unchanged "x"/"y"/"yaw" -- the final,
// fully-calibrated planner pose) via compute_staged_transform_view()'s
// algebraic inversion of the known calibration matrix + yaw_offset -- see
// that function's own doc comment in MocapCore.h for why this is exact
// (mocap_rotation() is a fixed, parameter-free, invertible map) and why it
// needs no bridge/wire-protocol change (the calibration matrix and
// yaw_offset are already available locally via transform_info()). "raw
// y/z" shows whichever of Motive's y_m/z_m axes is actually recoverable
// from the published pose (the OTHER one is the height axis the wire
// protocol never transmits -- see StagedTransformView::raw_height_unknown_axis)
// prefixed with "y=" or "z=" so the reader always knows which axis they're
// looking at. A singular calibration matrix (misconfigured
// mocap_to_world_matrix) shows "n/a" in columns 3-8 rather than garbage.
void MocapTab::update_body_table()
{
  const std::vector<MocapManager::Body> bodies = manager_.mocap_manager().bodies();
  const std::vector<MotiveAssetEntry> assets = manager_.mocap_manager().motive_assets();
  const nlohmann::json &mapping = manager_.mocap_manager().mapping();
  const TransformInfo info = manager_.mocap_manager().transform_info();

  auto find_body = [&](const std::string &published_name) -> const MocapManager::Body *
  {
    for (const auto &b : bodies)
      if (b.published_name == published_name)
        return &b;
    return nullptr;
  };

  auto set_item = [&](int row, int col, const QString &text)
  {
    QTableWidgetItem *item = body_table_->item(row, col);
    if (!item)
    {
      item = new QTableWidgetItem();
      body_table_->setItem(row, col, item);
    }
    item->setText(text);
  };

  // Fills columns 3-8 (raw x / raw y-or-z / raw yaw / mocap x / mocap y /
  // mocap yaw) from a live pose + the robot/published name compute_staged_
  // transform_view() needs to look up its yaw_offset; "-" in every one of
  // those columns if there's no live pose yet, "n/a" instead if the
  // calibration matrix is singular (inversion genuinely impossible, not
  // just "no data yet" -- see StagedTransformView::ok's doc comment).
  auto set_staged_columns = [&](int row, const MocapManager::Body *b, const std::string &robot_name)
  {
    if (!b || !b->has_pose)
    {
      for (int col : {3, 4, 5, 6, 7, 8})
        set_item(row, col, "-");
      return;
    }
    simviz::StagedTransformView view =
        simviz::compute_staged_transform_view(info, b->pose, robot_name);
    if (!view.ok)
    {
      for (int col : {3, 4, 5, 6, 7, 8})
        set_item(row, col, "n/a");
      return;
    }
    // The KNOWN ground-plane axis is whichever of y_m/z_m is NOT the
    // reported unknown (height) one -- see StagedTransformView's doc
    // comment.
    const bool known_axis_is_z = (view.raw_height_unknown_axis == "y");
    const double known_axis_value = known_axis_is_z ? view.raw_z_m : view.raw_y_m;
    set_item(row, 3, QString::number(view.raw_x_m, 'f', 3));
    set_item(row, 4,
              QString("%1=%2").arg(known_axis_is_z ? "z" : "y")
                  .arg(known_axis_value, 0, 'f', 3));
    set_item(row, 5, QString::number(view.raw_yaw, 'f', 3));
    set_item(row, 6, QString::number(view.mocap_x, 'f', 3));
    set_item(row, 7, QString::number(view.mocap_y, 'f', 3));
    set_item(row, 8, QString::number(view.mocap_yaw, 'f', 3));
  };

  if (!assets.empty())
  {
    body_table_->setRowCount(static_cast<int>(assets.size()));
    for (int row = 0; row < static_cast<int>(assets.size()); ++row)
    {
      const MotiveAssetEntry &a = assets[static_cast<size_t>(row)];
      const std::string published_name = expected_published_name(a.name, mapping);
      const MocapManager::Body *b = find_body(published_name);

      set_item(row, 0, QString::fromStdString(a.name));
      set_item(row, 1, QString::number(a.id));
      set_item(row, 2, (b && b->has_pose) ? QString::number(b->age_s, 'f', 2) : "-");
      set_staged_columns(row, b, published_name);
      set_item(row, 9, (b && b->has_pose) ? QString::number(b->pose.x, 'f', 3) : "-");
      set_item(row, 10, (b && b->has_pose) ? QString::number(b->pose.y, 'f', 3) : "-");
      set_item(row, 11, (b && b->has_pose) ? QString::number(b->pose.yaw, 'f', 3) : "-");
    }
    return;
  }

  // Modeldef inventory not received yet -- auto-discovery mode withholds
  // publishing ANY body until it is (see optitrack_zmq_bridge.cpp's
  // "waiting for NAT_MODELDEF before anything publishes"), so `bodies` is
  // normally empty here too; this branch is defensive, not the common
  // case. Shows a "name pending" placeholder that resolves to the real
  // Motive name the moment motive_assets() is populated (next tick()'s
  // drain_stdout() call after the inventory line arrives).
  body_table_->setRowCount(static_cast<int>(bodies.size()));
  for (int row = 0; row < static_cast<int>(bodies.size()); ++row)
  {
    const MocapManager::Body &b = bodies[static_cast<size_t>(row)];
    set_item(row, 0, "(name pending)");
    set_item(row, 1, "?");
    set_item(row, 2, b.has_pose ? QString::number(b.age_s, 'f', 2) : "-");
    set_staged_columns(row, &b, b.published_name);
    set_item(row, 9, b.has_pose ? QString::number(b.pose.x, 'f', 3) : "-");
    set_item(row, 10, b.has_pose ? QString::number(b.pose.y, 'f', 3) : "-");
    set_item(row, 11, b.has_pose ? QString::number(b.pose.yaw, 'f', 3) : "-");
  }
}

void MocapTab::rebuild_mapping_table()
{
  auto model = manager_.execution_manager().scenario_model();

  if (model)
  {
    if (mapping_table_mode_ == MappingTableMode::Scenario && model == mapping_table_model_)
      return; // already built for this scenario identity.

    mapping_table_mode_ = MappingTableMode::Scenario;
    mapping_table_model_ = model;
    mapping_table_body_keys_.clear();
    mapping_table_->setRowCount(0);

    const auto &robots = model->robots_sorted_by_name();
    mapping_table_->setRowCount(static_cast<int>(robots.size()));

    const std::vector<MotiveAssetEntry> assets = manager_.mocap_manager().motive_assets();
    const nlohmann::json &mapping = manager_.mocap_manager().mapping();
    const TransformInfo info = manager_.mocap_manager().transform_info();
    const auto &endpoints = manager_.config().real_robot_endpoints;

    for (int row = 0; row < static_cast<int>(robots.size()); ++row)
    {
      EntityMeta *robot = robots[static_cast<size_t>(row)];
      if (!robot)
        continue;
      const std::string name = robot->name;

      QTableWidgetItem *name_item = new QTableWidgetItem(QString::fromStdString(name));
      name_item->setFlags(name_item->flags() & ~Qt::ItemIsEditable);
      mapping_table_->setItem(row, 0, name_item);

      QComboBox *combo = new QComboBox(mapping_table_);
      combo->addItem("(none)");
      const std::string current_motive_alias = reverse_alias_lookup(mapping, name);
      int select_index = 0;
      for (const auto &a : assets)
      {
        combo->addItem(QString::fromStdString(a.name));
        if (a.name == current_motive_alias)
          select_index = combo->count() - 1;
      }
      combo->setCurrentIndex(select_index);
      mapping_table_->setCellWidget(row, 1, combo);

      double yaw_offset = 0.0;
      auto yaw_it = info.yaw_offsets.find(name);
      if (yaw_it != info.yaw_offsets.end())
        yaw_offset = yaw_it->second;
      QDoubleSpinBox *yaw_spin = new QDoubleSpinBox(mapping_table_);
      yaw_spin->setRange(-180.0, 180.0);
      yaw_spin->setDecimals(2);
      yaw_spin->setSuffix(" deg");
      {
        const QSignalBlocker blocker(yaw_spin);
        yaw_spin->setValue(yaw_offset * 180.0 / M_PI);
      }
      // Per-row dirty tracking (mirrors calib_tx_spin_'s connect -- see
      // MocapTab.h's mapping_yaw_dirty_ doc comment): only a live
      // user-driven valueChanged() sets this; the setValue() above is
      // QSignalBlocker-wrapped so it never does.
      connect(yaw_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
              [this, name]() { mapping_yaw_dirty_[name] = true; });
      mapping_table_->setCellWidget(row, 2, yaw_spin);

      QLineEdit *vesc_edit = new QLineEdit(mapping_table_);
      std::string vesc_value;
      auto ep_it = endpoints.find(name);
      if (ep_it != endpoints.end() && !ep_it->second.vesc_endpoint.empty())
        vesc_value = ep_it->second.vesc_endpoint;
      else
        vesc_value =
            "tcp://127.0.0.1:" + std::to_string(manager_.config().vesc_port_start + row);
      vesc_edit->setText(QString::fromStdString(vesc_value));
      mapping_table_->setCellWidget(row, 3, vesc_edit);

      QLabel *fresh_dot = new QLabel(mapping_table_);
      fresh_dot->setFixedSize(12, 12);
      mapping_table_->setCellWidget(row, 4, fresh_dot);
    }
    return;
  }

  // No scenario loaded: one row per DETECTED Motive body (motive_assets(),
  // same source AND same (modeldef) order the rigid-body table keys off --
  // see this file's header comment for the full column design), reconciled
  // additively against the table's CURRENT rows so an in-progress selection
  // change in another row's planner-name QComboBox (fixed
  // ""/robot1..robot4 choices, see kFixedPlannerNames) is never touched by
  // a row that didn't change.
  mapping_table_model_.reset();

  const std::vector<MotiveAssetEntry> assets = manager_.mocap_manager().motive_assets();

  if (mapping_table_mode_ != MappingTableMode::ScenarioLess)
  {
    // Entering scenario-less mode fresh (first build, or a scenario was
    // just unloaded) -- start from an empty table.
    mapping_table_mode_ = MappingTableMode::ScenarioLess;
    mapping_table_->setRowCount(0);
    mapping_table_body_keys_.clear();
  }

  // Remove rows whose body a fresh modeldef (e.g. after a bridge restart --
  // motive_assets() is cleared on every start_bridge()) no longer reports.
  for (int row = static_cast<int>(mapping_table_body_keys_.size()) - 1; row >= 0; --row)
  {
    const std::string &key = mapping_table_body_keys_[static_cast<size_t>(row)];
    const bool still_present = std::any_of(
        assets.begin(), assets.end(), [&](const MotiveAssetEntry &a) { return a.name == key; });
    if (!still_present)
    {
      mapping_table_->removeRow(row);
      mapping_table_body_keys_.erase(mapping_table_body_keys_.begin() + row);
    }
  }

  // Add rows for newly-discovered bodies (appended -- existing rows keep
  // their exact widget instances, so any in-progress edit survives).
  const nlohmann::json &mapping = manager_.mocap_manager().mapping();
  const TransformInfo info = manager_.mocap_manager().transform_info();
  for (const MotiveAssetEntry &a : assets)
  {
    if (std::find(mapping_table_body_keys_.begin(), mapping_table_body_keys_.end(), a.name) !=
        mapping_table_body_keys_.end())
      continue; // already has a row.

    const int row = mapping_table_->rowCount();
    mapping_table_->insertRow(row);
    mapping_table_body_keys_.push_back(a.name);

    QTableWidgetItem *body_item = new QTableWidgetItem(QString::fromStdString(a.name));
    body_item->setFlags(body_item->flags() & ~Qt::ItemIsEditable);
    mapping_table_->setItem(row, 0, body_item);

    // BUG report / MAPPING DROPDOWN: this was a free-text QLineEdit, which
    // let a typo'd or stale planner name silently fail to match any real
    // scenario robot. Fixed-choice QComboBox instead -- "" (unmapped) plus
    // the project's four planner robot names -- mirrors the Scenario mode
    // branch above's "Mapped body" combo (fixed to the DETECTED bodies
    // there; fixed to the KNOWN planner names here, the reverse direction).
    // A pre-existing alias outside this fixed list (e.g. hand-edited into
    // the JSON) has no matching item and the combo falls back to ""
    // (index 0) -- see kFixedPlannerNames' doc comment below.
    QComboBox *robot_combo = new QComboBox(mapping_table_);
    for (const char *name : kFixedPlannerNames)
      robot_combo->addItem(name);
    std::string preset_alias;
    if (mapping.contains("aliases") && mapping["aliases"].is_object())
    {
      auto it = mapping["aliases"].find(a.name);
      if (it != mapping["aliases"].end() && it.value().is_string())
        preset_alias = it.value().get<std::string>();
    }
    const int preset_index = robot_combo->findText(QString::fromStdString(preset_alias));
    robot_combo->setCurrentIndex(preset_index >= 0 ? preset_index : 0);
    mapping_table_->setCellWidget(row, 1, robot_combo);

    double yaw_offset = 0.0;
    if (!preset_alias.empty())
    {
      auto yaw_it = info.yaw_offsets.find(preset_alias);
      if (yaw_it != info.yaw_offsets.end())
        yaw_offset = yaw_it->second;
    }
    QDoubleSpinBox *yaw_spin = new QDoubleSpinBox(mapping_table_);
    yaw_spin->setRange(-180.0, 180.0);
    yaw_spin->setDecimals(2);
    yaw_spin->setSuffix(" deg");
    {
      const QSignalBlocker blocker(yaw_spin);
      yaw_spin->setValue(yaw_offset * 180.0 / M_PI);
    }
    // Dirty-tracking key is the ROW key (Motive name, `a.name`) -- NOT the
    // published/planner name (which can change later via the alias combo
    // without recreating this row) -- see MocapTab.h's mapping_yaw_dirty_
    // doc comment.
    const std::string motive_key = a.name;
    connect(yaw_spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
            [this, motive_key]() { mapping_yaw_dirty_[motive_key] = true; });
    mapping_table_->setCellWidget(row, 2, yaw_spin);

    // No scenario loaded -> no EntityMeta to key real_robot_endpoints by;
    // the VESC-endpoint column is only meaningful in Scenario mode.
    QTableWidgetItem *vesc_placeholder = new QTableWidgetItem("(no scenario)");
    vesc_placeholder->setFlags(vesc_placeholder->flags() & ~Qt::ItemIsEditable);
    mapping_table_->setItem(row, 3, vesc_placeholder);

    QLabel *fresh_dot = new QLabel(mapping_table_);
    fresh_dot->setFixedSize(12, 12);
    mapping_table_->setCellWidget(row, 4, fresh_dot);
  }
}

void MocapTab::update_mapping_table_live_columns()
{
  const double freshness = manager_.config().mocap.freshness_window_s;

  if (mapping_table_mode_ == MappingTableMode::Scenario)
  {
    auto model = mapping_table_model_;
    if (!model)
      return;

    const auto &robots = model->robots_sorted_by_name();

    for (int row = 0; row < static_cast<int>(robots.size()) && row < mapping_table_->rowCount();
         ++row)
    {
      EntityMeta *robot = robots[static_cast<size_t>(row)];
      if (!robot)
        continue;

      QComboBox *combo = qobject_cast<QComboBox *>(mapping_table_->cellWidget(row, 1));
      if (combo)
      {
        const std::vector<MotiveAssetEntry> assets = manager_.mocap_manager().motive_assets();
        for (const auto &a : assets)
        {
          bool present = false;
          for (int i = 0; i < combo->count(); ++i)
          {
            if (combo->itemText(i).toStdString() == a.name)
            {
              present = true;
              break;
            }
          }
          if (!present)
            combo->addItem(QString::fromStdString(a.name));
        }

        // Auto-select this robot's configured alias once its Motive body is
        // discovered (bodies aren't known -- and so aren't in the combo -- at
        // rebuild_mapping_table() time if the bridge hadn't connected yet).
        // Only while still on the default "(none)" -- never overrides a
        // choice the user (or an earlier auto-select) already made.
        if (combo->currentIndex() == 0)
        {
          const std::string alias =
              reverse_alias_lookup(manager_.mocap_manager().mapping(), robot->name);
          if (!alias.empty())
          {
            const int idx = combo->findText(QString::fromStdString(alias));
            if (idx > 0)
              combo->setCurrentIndex(idx);
          }
        }
      }

      QLabel *fresh_dot = qobject_cast<QLabel *>(mapping_table_->cellWidget(row, 4));
      if (fresh_dot)
      {
        bool fresh = false;
        for (const auto &b : manager_.mocap_manager().bodies())
        {
          if (b.published_name == robot->name && b.has_pose && b.age_s >= 0.0 &&
              b.age_s < freshness)
          {
            fresh = true;
            break;
          }
        }
        fresh_dot->setStyleSheet(QString("background-color: %1; border-radius: 6px; border: 1px "
                                          "solid black;")
                                      .arg(fresh ? "#43A047" : "#9E9E9E"));
      }
    }
    return;
  }

  if (mapping_table_mode_ != MappingTableMode::ScenarioLess)
    return;

  // Scenario-less: refresh ONLY the freshness dot per row -- column 1's
  // QComboBox (fixed unmapped/robot1..robot4 choices, see
  // rebuild_mapping_table()) holds a user-changeable SELECTION, never
  // touched here (matching the Scenario-mode branch's "never clobber a
  // mid-edit" contract); the alias pre-selection only happens once, at
  // row-creation time in rebuild_mapping_table(), from the on-disk config
  // as of that moment.
  const nlohmann::json &mapping = manager_.mocap_manager().mapping();
  const std::vector<MocapManager::Body> bodies = manager_.mocap_manager().bodies();

  for (int row = 0;
       row < static_cast<int>(mapping_table_body_keys_.size()) && row < mapping_table_->rowCount();
       ++row)
  {
    const std::string &motive_name = mapping_table_body_keys_[static_cast<size_t>(row)];
    const std::string expected_published = expected_published_name(motive_name, mapping);

    QLabel *fresh_dot = qobject_cast<QLabel *>(mapping_table_->cellWidget(row, 4));
    if (!fresh_dot)
      continue;
    bool fresh = false;
    for (const auto &b : bodies)
    {
      if (b.published_name == expected_published && b.has_pose && b.age_s >= 0.0 &&
          b.age_s < freshness)
      {
        fresh = true;
        break;
      }
    }
    fresh_dot->setStyleSheet(QString("background-color: %1; border-radius: 6px; border: 1px "
                                      "solid black;")
                                  .arg(fresh ? "#43A047" : "#9E9E9E"));
  }
}

std::vector<MocapTab::RobotStagePreview> MocapTab::build_stage_preview() const
{
  std::vector<RobotStagePreview> out;
  auto model = manager_.execution_manager().scenario_model();
  if (!model)
    return out;

  const double freshness = manager_.config().mocap.freshness_window_s;
  for (EntityMeta *robot : model->robots_sorted_by_name())
  {
    if (!robot)
      continue;
    RobotStagePreview p;
    p.name = robot->name;
    p.is_mapped = robot_is_mapped(manager_.mocap_manager().mapping(), robot->name);
    p.target = model->timetable().get_pose(robot, 0.0);
    for (const auto &b : manager_.mocap_manager().bodies())
    {
      if (b.published_name == robot->name && b.has_pose && b.age_s >= 0.0 && b.age_s < freshness)
      {
        p.has_fresh_current = true;
        p.current = b.pose;
        break;
      }
    }
    out.push_back(p);
  }
  return out;
}

void MocapTab::update_move_button_enabled()
{
  std::vector<QString> missing;

  auto model = manager_.execution_manager().scenario_model();
  if (!model || model->robots_sorted_by_name().empty())
    missing.push_back("no scenario loaded");

  if (!manager_.config().real_mode)
    missing.push_back("mars_sim_viz not started with --real-mode");

  if (manager_.mocap_manager().state() != MocapState::Connected)
    missing.push_back("OptiTrack bridge not Connected");

  if (manager_.execution_manager().is_busy())
    missing.push_back("a run is already in progress");

  if (manager_.is_stage_real_active())
    missing.push_back("already moving to initial poses");

  // UNMAPPED ROBOTS ARE SIM-ONLY: this button no longer requires EVERY
  // scenario robot to have a fresh mocap body -- only that at least one
  // MAPPED robot does (mirrors stage_real_robots()'s own relaxed
  // precondition, MocapCore.h's robot_is_mapped()). A mapped robot that
  // currently lacks a fresh pose still blocks (it is expected to be
  // trackable); an unmapped robot never blocks -- it is treated as
  // simulation-only.
  size_t mapped_fresh_count = 0;
  if (model)
  {
    const std::vector<RobotStagePreview> preview = build_stage_preview();
    for (const auto &p : preview)
    {
      if (!p.is_mapped)
        continue;
      if (p.has_fresh_current)
        ++mapped_fresh_count;
      else
      {
        missing.push_back(QString("mapped robot '%1' has no fresh mocap body")
                               .arg(QString::fromStdString(p.name)));
      }
    }
    if (mapped_fresh_count == 0)
      missing.push_back("no scenario robot is mapped+fresh in Robot mapping (unmapped robots are "
                         "treated as simulation-only)");
  }

  const bool enabled = missing.empty();
  move_to_initial_button_->setEnabled(enabled);
  if (enabled)
  {
    move_to_initial_button_->setToolTip(
        "Sequentially drives every MAPPED scenario robot from its current (mocap) pose to its "
        "planner-assumed initial pose. Robots not present in Robot mapping are treated as "
        "simulation-only and are skipped.");
  }
  else
  {
    QString tip = "Cannot move robots yet:\n";
    for (const auto &m : missing)
      tip += QString(" - %1\n").arg(m);
    move_to_initial_button_->setToolTip(tip);
  }
}

void MocapTab::update_transform_panel()
{
  const TransformInfo info = manager_.mocap_manager().transform_info();

  std::vector<MocapTransformWidget::BodyPoint> points;
  for (const auto &b : manager_.mocap_manager().bodies())
  {
    if (!b.has_pose)
      continue;
    points.push_back({b.published_name, b.pose});
  }
  transform_widget_->set_data(info, points);

  std::ostringstream oss;
  oss << "matrix: a=" << info.matrix.a << " b=" << info.matrix.b << " tx=" << info.matrix.tx
      << "\n        c=" << info.matrix.c << " d=" << info.matrix.d << " ty=" << info.matrix.ty
      << "\n";
  oss << "rotation: " << info.rotation_deg << " deg   y_up: " << (info.y_up ? "true" : "false")
      << "\n";
  oss << "source: " << (info.matrix_from_legacy ? "legacy x0/y0/theta0" : "mocap_to_world_matrix")
      << "\n";
  if (info.non_rigid_warning)
    oss << "WARNING: " << info.warning << "\n";
  if (info.parse_error)
    oss << "ERROR: " << info.error << "\n";
  if (!info.yaw_offsets.empty())
  {
    oss << "yaw_offset:";
    for (const auto &[name, v] : info.yaw_offsets)
      oss << " " << name << "=" << v;
  }
  transform_numbers_label_->setText(QString::fromStdString(oss.str()));
}

void MocapTab::load_calibration_fields()
{
  const nlohmann::json &mapping = manager_.mocap_manager().mapping();
  const double x0 = mapping.value("x0", 0.0);
  const double y0 = mapping.value("y0", 0.0);
  const double theta0 = mapping.value("theta0", 0.0);
  const bool y_up = mapping.value("y_up", false);

  // QSignalBlocker per widget: this is a PROGRAMMATIC reload, never a user
  // edit -- must not trip the calib_*_dirty_ flags the valueChanged()/
  // toggled() connections above set (see MocapTab.h's doc comment on those
  // flags).
  {
    const QSignalBlocker tx_blocker(calib_tx_spin_);
    calib_tx_spin_->setValue(x0);
  }
  {
    const QSignalBlocker ty_blocker(calib_ty_spin_);
    calib_ty_spin_->setValue(y0);
  }
  {
    const QSignalBlocker theta_blocker(calib_theta_deg_spin_);
    calib_theta_deg_spin_->setValue(theta0 * 180.0 / M_PI);
  }
  {
    const QSignalBlocker y_up_blocker(calib_y_up_checkbox_);
    calib_y_up_checkbox_->setChecked(y_up);
  }

  calib_tx_dirty_ = false;
  calib_ty_dirty_ = false;
  calib_theta_dirty_ = false;
  calib_y_up_dirty_ = false;
}

void MocapTab::on_connect_toggle()
{
  const MocapState state = manager_.mocap_manager().state();
  if (state == MocapState::Disconnected || state == MocapState::Error)
  {
    std::string err;
    if (!manager_.mocap_manager().start_bridge(&err))
    {
      QMessageBox::warning(this, "OptiTrack",
                            QString("Failed to connect: %1").arg(QString::fromStdString(err)));
    }
  }
  else
  {
    manager_.mocap_manager().stop_bridge();
  }
  update_indicator();
}

void MocapTab::on_save_mapping()
{
  auto model = mapping_table_model_;
  std::vector<std::string> errors;

  // Reload the in-memory mapping from disk BEFORE composing any writes below
  // (same "pick up an out-of-band on-disk edit before overwriting" rationale
  // as on_apply_calibration()'s own reload -- see that method's doc
  // comment). This MUST happen before the alias/yaw-offset edits below (not
  // just before the final save_mapping() call) because reload_mapping()
  // DISCARDS unsaved in-memory edits -- reloading afterward would wipe out
  // the very set_alias()/set_yaw_offset() calls this function is about to
  // make.
  {
    std::string reload_err;
    if (!manager_.mocap_manager().reload_mapping(&reload_err))
    {
      const QString text = QString("Failed to reload mapping before save: %1")
                                .arg(QString::fromStdString(reload_err));
      mapping_status_label_->setText(text);
      QMessageBox::warning(this, "OptiTrack", text);
      return;
    }
  }

  // BUG report: this used to call manager_.save_real_robots_config()
  // UNCONDITIONALLY below, every click of "Save Mapping" -- including in
  // ScenarioLess mode (no VESC column exists there at all) or in Scenario
  // mode when the user only touched an alias combo, never a VESC-endpoint
  // field. That meant a real_robots_config_path resolution problem (or
  // simply no configured path) surfaced as a save failure on EVERY mapping
  // save, not just ones that actually touched VESC endpoints. Tracked via
  // QLineEdit::isModified() (Qt clears this automatically on every
  // programmatic setText(), i.e. every row-(re)build's pre-fill, and sets
  // it only on an actual user keystroke) -- gating BOTH the
  // set_real_robot_endpoint() call and the save behind it means "was
  // edited" and "would change on-disk content" collapse into the same
  // condition, so there's nothing left to separately check.
  bool any_vesc_edit = false;

  if (model)
  {
    const auto &robots = model->robots_sorted_by_name();
    for (int row = 0; row < static_cast<int>(robots.size()) && row < mapping_table_->rowCount();
         ++row)
    {
      EntityMeta *robot = robots[static_cast<size_t>(row)];
      if (!robot)
        continue;

      QComboBox *combo = qobject_cast<QComboBox *>(mapping_table_->cellWidget(row, 1));
      if (combo && combo->currentIndex() > 0)
      {
        manager_.mocap_manager().set_alias(combo->currentText().toStdString(), robot->name);
      }

      // Per-row reload-then-merge for yaw offset (Change 1, step 5): an
      // untouched row's on-disk "yaw_offset"[robot->name] entry is left
      // exactly as the reload above picked it up (set_yaw_offset() simply
      // isn't called for it); only a row whose spinbox the user actually
      // edited this session (mapping_yaw_dirty_[robot->name]) overwrites it.
      QDoubleSpinBox *yaw_spin = qobject_cast<QDoubleSpinBox *>(mapping_table_->cellWidget(row, 2));
      if (yaw_spin && mapping_yaw_dirty_[robot->name])
      {
        manager_.mocap_manager().set_yaw_offset(robot->name, yaw_spin->value() * M_PI / 180.0);
      }

      QLineEdit *vesc_edit = qobject_cast<QLineEdit *>(mapping_table_->cellWidget(row, 3));
      if (vesc_edit && vesc_edit->isModified())
      {
        RealRobotEndpoints ep = manager_.config().real_robot_endpoints.count(robot->name)
                                     ? manager_.config().real_robot_endpoints.at(robot->name)
                                     : RealRobotEndpoints{};
        ep.vesc_endpoint = vesc_edit->text().toStdString();
        manager_.set_real_robot_endpoint(robot->name, ep);
        any_vesc_edit = true;
      }
    }
  }
  else if (mapping_table_mode_ == MappingTableMode::ScenarioLess)
  {
    // No scenario loaded -> no VESC endpoint to write (see
    // rebuild_mapping_table()'s doc comment); only alias edits apply here.
    // Column 1 is a fixed-choice QComboBox (unmapped/robot1..robot4 -- see
    // rebuild_mapping_table()'s row-construction below), not free text.
    for (int row = 0; row < static_cast<int>(mapping_table_body_keys_.size()) &&
                      row < mapping_table_->rowCount();
         ++row)
    {
      const std::string &motive_name = mapping_table_body_keys_[static_cast<size_t>(row)];
      QComboBox *robot_combo = qobject_cast<QComboBox *>(mapping_table_->cellWidget(row, 1));
      if (!robot_combo)
        continue;
      const std::string planner_name = robot_combo->currentText().toStdString();
      if (!planner_name.empty())
        manager_.mocap_manager().set_alias(motive_name, planner_name);

      // Per-row reload-then-merge for yaw offset, same contract as the
      // Scenario-mode branch above -- but here the yaw_offset is keyed on
      // whatever planner name THIS row's alias combo currently resolves to
      // (same "currentText() at save time" convention set_alias() above
      // already uses), not the row's own identity key (motive_name). Skipped
      // entirely (row untouched on disk) if dirty but no planner name is
      // currently selected -- there is nothing to key the write on.
      QDoubleSpinBox *yaw_spin = qobject_cast<QDoubleSpinBox *>(mapping_table_->cellWidget(row, 2));
      if (yaw_spin && mapping_yaw_dirty_[motive_name] && !planner_name.empty())
      {
        manager_.mocap_manager().set_yaw_offset(planner_name, yaw_spin->value() * M_PI / 180.0);
      }
    }
  }

  std::string mapping_err;
  if (!manager_.mocap_manager().save_mapping(&mapping_err))
    errors.push_back("mapping: " + mapping_err);

  if (any_vesc_edit)
  {
    std::string real_robots_err;
    if (!manager_.save_real_robots_config(&real_robots_err))
      errors.push_back("real-robots config: " + real_robots_err);
  }

  if (!errors.empty())
  {
    QString text = "Failed to save:\n";
    for (const auto &e : errors)
      text += QString::fromStdString(" - " + e + "\n");
    mapping_status_label_->setText(text);
    QMessageBox::warning(this, "OptiTrack", text);
    return;
  }

  // Clear every VESC-endpoint QLineEdit's modified flag now that its
  // current text is exactly what's on disk -- otherwise a SECOND click of
  // "Save Mapping" with no further edits would see isModified() still true
  // (Qt only clears it on an explicit setText()/setModified(false), never
  // on its own after a save) and needlessly re-write real_robots_config.
  if (model)
  {
    const auto &robots = model->robots_sorted_by_name();
    for (int row = 0; row < static_cast<int>(robots.size()) && row < mapping_table_->rowCount();
         ++row)
    {
      QLineEdit *vesc_edit = qobject_cast<QLineEdit *>(mapping_table_->cellWidget(row, 3));
      if (vesc_edit)
        vesc_edit->setModified(false);
    }
  }

  // Clear every row's yaw-offset dirty flag now that the mapping just saved
  // reflects exactly what every dirty row's spinbox held, and re-sync every
  // spinbox's value from the (now-authoritative) saved mapping -- in place,
  // NOT via a full rebuild_mapping_table() row-recreation (would destroy
  // widget identity for any OTHER still-mid-edit row -- see
  // mapping_yaw_dirty_'s doc comment). setValue() is QSignalBlocker-wrapped
  // so this programmatic re-sync never re-dirties the row it just cleared.
  {
    const TransformInfo saved_info = manager_.mocap_manager().transform_info();
    if (model)
    {
      const auto &robots = model->robots_sorted_by_name();
      for (int row = 0; row < static_cast<int>(robots.size()) && row < mapping_table_->rowCount();
           ++row)
      {
        EntityMeta *robot = robots[static_cast<size_t>(row)];
        if (!robot)
          continue;
        QDoubleSpinBox *yaw_spin =
            qobject_cast<QDoubleSpinBox *>(mapping_table_->cellWidget(row, 2));
        if (yaw_spin)
        {
          double yaw_offset = 0.0;
          auto it = saved_info.yaw_offsets.find(robot->name);
          if (it != saved_info.yaw_offsets.end())
            yaw_offset = it->second;
          const QSignalBlocker blocker(yaw_spin);
          yaw_spin->setValue(yaw_offset * 180.0 / M_PI);
        }
        mapping_yaw_dirty_[robot->name] = false;
      }
    }
    else if (mapping_table_mode_ == MappingTableMode::ScenarioLess)
    {
      const nlohmann::json &saved_mapping = manager_.mocap_manager().mapping();
      for (int row = 0; row < static_cast<int>(mapping_table_body_keys_.size()) &&
                        row < mapping_table_->rowCount();
           ++row)
      {
        const std::string &motive_name = mapping_table_body_keys_[static_cast<size_t>(row)];
        QComboBox *robot_combo = qobject_cast<QComboBox *>(mapping_table_->cellWidget(row, 1));
        QDoubleSpinBox *yaw_spin =
            qobject_cast<QDoubleSpinBox *>(mapping_table_->cellWidget(row, 2));
        if (yaw_spin)
        {
          double yaw_offset = 0.0;
          std::string planner_name;
          if (robot_combo)
            planner_name = robot_combo->currentText().toStdString();
          if (!planner_name.empty())
          {
            const TransformInfo info_now = parse_transform_info(saved_mapping);
            auto it = info_now.yaw_offsets.find(planner_name);
            if (it != info_now.yaw_offsets.end())
              yaw_offset = it->second;
          }
          const QSignalBlocker blocker(yaw_spin);
          yaw_spin->setValue(yaw_offset * 180.0 / M_PI);
        }
        mapping_yaw_dirty_[motive_name] = false;
      }
    }
  }

  mapping_status_label_->setText(
      "Saved. Mapping edits (aliases and yaw offsets) only take effect on the bridge's NEXT "
      "connect (it reads its config file only at startup).");

  const MocapState state = manager_.mocap_manager().state();
  if (state == MocapState::Connecting || state == MocapState::Connected)
  {
    const auto reply = QMessageBox::question(
        this, "OptiTrack",
        "Mapping saved. The bridge only reads its config at startup -- restart it now to apply "
        "the change?",
        QMessageBox::Yes | QMessageBox::No);
    if (reply == QMessageBox::Yes)
    {
      manager_.mocap_manager().stop_bridge();
      std::string start_err;
      if (!manager_.mocap_manager().start_bridge(&start_err))
      {
        QMessageBox::warning(
            this, "OptiTrack",
            QString("Failed to restart bridge: %1").arg(QString::fromStdString(start_err)));
      }
    }
  }

  rebuild_mapping_table(); // no-op unless the scenario/detected-body set itself
                            // also changed; kept for symmetry/clarity.
  update_mapping_table_live_columns();
}

void MocapTab::on_move_to_initial_poses()
{
  auto model = manager_.execution_manager().scenario_model();
  if (!model)
    return;

  const std::vector<RobotStagePreview> preview = build_stage_preview();

  QDialog dialog(this);
  dialog.setWindowTitle("Move real robots to initial poses");
  QVBoxLayout *layout = new QVBoxLayout(&dialog);
  layout->addWidget(new QLabel(
      "The following MAPPED robots will move sequentially; unmapped robots are simulation-only "
      "and are not moved:",
      &dialog));

  QPlainTextEdit *text = new QPlainTextEdit(&dialog);
  text->setReadOnly(true);
  QFont mono("Monospace");
  mono.setStyleHint(QFont::TypeWriter);
  text->setFont(mono);
  // UNMAPPED ROBOTS ARE SIM-ONLY: this listing mirrors stage_real_robots()'s
  // own split -- mapped+fresh robots show the actual move, an unmapped
  // robot is labeled "not present (simulate only)", and a mapped-but-stale
  // robot (rare -- it currently blocks the button itself, see
  // update_move_button_enabled()) is called out separately rather than
  // silently omitted.
  std::ostringstream oss;
  for (const auto &p : preview)
  {
    if (!p.is_mapped)
    {
      oss << p.name << ": not present (simulate only)\n";
      continue;
    }
    if (!p.has_fresh_current)
    {
      oss << p.name << ": MAPPED but no fresh mocap pose (will block the move)\n";
      continue;
    }
    const double dx = p.target.x - p.current.x;
    const double dy = p.target.y - p.current.y;
    const double dist = std::sqrt(dx * dx + dy * dy);
    oss << p.name << ": (" << p.current.x << ", " << p.current.y << ", " << p.current.yaw
        << ") -> (" << p.target.x << ", " << p.target.y << ", " << p.target.yaw
        << ")  dist=" << dist << "m\n";
  }
  text->setPlainText(QString::fromStdString(oss.str()));
  layout->addWidget(text, /*stretch=*/1);

  QDialogButtonBox *buttons = new QDialogButtonBox(&dialog);
  QPushButton *confirm = buttons->addButton("Move real robots", QDialogButtonBox::AcceptRole);
  buttons->addButton(QDialogButtonBox::Cancel);
  connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
  connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
  layout->addWidget(buttons);
  (void)confirm;

  if (dialog.exec() != QDialog::Accepted)
    return;

  std::string err;
  if (!manager_.stage_real_robots(model, &err))
  {
    QMessageBox::warning(this, "OptiTrack",
                          QString("Failed to start: %1").arg(QString::fromStdString(err)));
  }
}

// Apply semantics (part B): writes x0/y0/theta0/y_up to the map-config
// (theta0 converted from the UI's degrees to radians -- config storage
// convention matches OptiTrackCore.h's apply_planar_transform(), which
// takes theta0 in radians) and REMOVES "mocap_to_world_matrix" entirely --
// see set_calibration_in_mapping()'s doc comment for why the removal is
// required (the matrix would otherwise silently supersede these values).
// The typed translation-X/Y + rotation fields compose EXACTLY the way
// apply_planar_transform() does (p' = R(theta0)*p + [x0,y0], rotate then
// translate) -- the same composition parse_transform_info()'s legacy
// branch already derives into TransformInfo::matrix, which is what
// MocapTransformWidget's Motive-axes rendering (this file's `apply` lambda
// in paintEvent()) already draws, so no separate math needs to be kept in
// sync here. If the bridge is currently Connecting/Connected, it is
// stopped and restarted (the bridge only reads its config at startup) so
// the calibration takes effect immediately; otherwise the file is just
// saved.
//
// BUG FIX (stale-state clobber): this used to write ALL FOUR fields from
// the GUI's snapshot unconditionally -- so a field the user never touched
// this session (e.g. y_up/theta0, hand-corrected on disk by someone else,
// or just left over from an earlier bridge session) got silently
// overwritten with whatever value the GUI happened to have loaded at
// construction/last refresh time, discarding the on-disk correction. Fixed
// per-field, gated by calib_*_dirty_ (see MocapTab.h's doc comment on
// those): (1) snapshot the GUI's current entries AND what the fields were
// last loaded FROM (mapping()'s calibration fields, pre-reload) BEFORE
// touching anything; (2) reload_mapping() the in-memory mapping from disk
// right before composing the write, so an untouched field picks up
// whatever is CURRENTLY on disk, not a stale GUI snapshot; (3) per field,
// use the user's GUI value if (and only if) calib_*_dirty_ is true for it,
// else the freshly-reloaded on-disk value; (4) if a DIRTY field's on-disk
// value also changed since it was loaded (someone/something edited the
// file between load and this Apply), the user's explicit entry still wins,
// but the divergence is reported via a message box so nothing is silently
// overwritten unnoticed.
void MocapTab::on_apply_calibration()
{
  const double gui_x0 = calib_tx_spin_->value();
  const double gui_y0 = calib_ty_spin_->value();
  const double gui_theta0 = calib_theta_deg_spin_->value() * M_PI / 180.0;
  const bool gui_y_up = calib_y_up_checkbox_->isChecked();

  // Copy (NOT a reference -- mapping() returns a reference to the live
  // in-memory json, which reload_mapping() below mutates in place) of what
  // the fields were last loaded FROM, so a dirty field can be checked for
  // divergence against the fresh reload.
  const nlohmann::json pre_reload = manager_.mocap_manager().mapping();
  const double pre_x0 = pre_reload.value("x0", 0.0);
  const double pre_y0 = pre_reload.value("y0", 0.0);
  const double pre_theta0 = pre_reload.value("theta0", 0.0);
  const bool pre_y_up = pre_reload.value("y_up", false);

  std::string reload_err;
  if (!manager_.mocap_manager().reload_mapping(&reload_err))
  {
    const QString text = QString("Failed to reload calibration before apply: %1")
                              .arg(QString::fromStdString(reload_err));
    calib_status_label_->setText(text);
    QMessageBox::warning(this, "OptiTrack", text);
    return;
  }

  const nlohmann::json post_reload = manager_.mocap_manager().mapping();
  const double disk_x0 = post_reload.value("x0", 0.0);
  const double disk_y0 = post_reload.value("y0", 0.0);
  const double disk_theta0 = post_reload.value("theta0", 0.0);
  const bool disk_y_up = post_reload.value("y_up", false);

  constexpr double kEps = 1e-9;
  std::vector<QString> divergences;

  // Per-field merge: an untouched (non-dirty) field keeps the fresh
  // on-disk value unconditionally; a dirty field uses the user's GUI entry
  // -- reporting (not blocking on) a divergence if the on-disk value also
  // moved since it was loaded.
  auto merge = [&](bool dirty, const QString &label, double gui_val, double pre_val,
                    double disk_val, double display_scale, const QString &unit) -> double
  {
    if (!dirty)
      return disk_val;
    if (std::abs(pre_val - disk_val) > kEps)
    {
      const QString pre_str = QString::number(pre_val * display_scale, 'f', 4) + unit;
      const QString new_str = QString::number(gui_val * display_scale, 'f', 4) + unit;
      divergences.push_back(QString("%1: old-disk %2 -> new value %3").arg(label, pre_str, new_str));
    }
    return gui_val;
  };

  const double final_x0 =
      merge(calib_tx_dirty_, "Translation X", gui_x0, pre_x0, disk_x0, 1.0, " m");
  const double final_y0 =
      merge(calib_ty_dirty_, "Translation Y", gui_y0, pre_y0, disk_y0, 1.0, " m");
  const double final_theta0 = merge(calib_theta_dirty_, "Rotation", gui_theta0, pre_theta0,
                                     disk_theta0, 180.0 / M_PI, " deg");
  bool final_y_up = disk_y_up;
  if (calib_y_up_dirty_)
  {
    if (pre_y_up != disk_y_up)
    {
      divergences.push_back(QString("Motive is Y-up: old-disk %1 -> new value %2")
                                 .arg(pre_y_up ? "true" : "false", gui_y_up ? "true" : "false"));
    }
    final_y_up = gui_y_up;
  }

  manager_.mocap_manager().set_calibration(final_x0, final_y0, final_theta0, final_y_up);

  std::string save_err;
  if (!manager_.mocap_manager().save_mapping(&save_err))
  {
    const QString text =
        QString("Failed to save calibration: %1").arg(QString::fromStdString(save_err));
    calib_status_label_->setText(text);
    QMessageBox::warning(this, "OptiTrack", text);
    load_calibration_fields();
    return;
  }

  if (!divergences.empty())
  {
    QString text =
        "The on-disk calibration changed since these fields were last loaded. Your edited "
        "field(s) were applied anyway (untouched fields kept the on-disk value):\n";
    for (const auto &d : divergences)
      text += " - " + d + "\n";
    QMessageBox::information(this, "OptiTrack", text);
  }

  const MocapState state = manager_.mocap_manager().state();
  if (state == MocapState::Connecting || state == MocapState::Connected)
  {
    manager_.mocap_manager().stop_bridge();
    std::string start_err;
    if (!manager_.mocap_manager().start_bridge(&start_err))
    {
      const QString text = QString("Calibration saved, but bridge restart failed: %1")
                                .arg(QString::fromStdString(start_err));
      calib_status_label_->setText(text);
      QMessageBox::warning(this, "OptiTrack", text);
      load_calibration_fields();
      update_transform_panel();
      return;
    }
    calib_status_label_->setText("Calibration saved; bridge restarted to apply it.");
  }
  else
  {
    calib_status_label_->setText("Calibration saved (bridge not running).");
  }

  load_calibration_fields();
  update_transform_panel();
}

} // namespace simviz
