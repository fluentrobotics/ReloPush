#include "SimVizWindow.h"

#include <PHAstar/Point.h> // get_corners() -- shared footprint-rotation math (also used by
                            // include/PHAstar/Visualization.h), reused here rather than
                            // re-deriving VisualizationWidget's own local rotation lambda.

#include <QAction>
#include <QCheckBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QMenu>
#include <QMenuBar>
#include <QMessageBox>
#include <QPainter>
#include <QPen>
#include <QStatusBar>
#include <QToolBar>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace simviz
{

namespace
{

// Mirrors MARS/include/PHAstarPushDemoOptions.h:506-513's robot_trace_colors
// palette. Duplicated as a small local literal (rather than pulling in
// PHAstarPushDemoOptions.h, an unrelated CLI-options aggregate with a much
// larger dependency footprint) -- see this task's ground truth on reusing
// the MATH/patterns, not the classes.
const QColor kRobotTraceColors[] = {
    QColor("#70A288"), QColor("#DAB785"), QColor("#D5896F"),
    QColor("#CC79A7"), QColor("#E69F00"), QColor("#56B4E9"),
};
constexpr int kRobotTraceColorCount = 6;

const QColor kObjectFillColor("#3D5A80");
const QColor kGoalOutlineColor("#E76F51");

constexpr double kTrailWindowS = 2.5;

// Small oriented-arrow polygon, ported from VisualizationWidget.cpp's
// drawOrientedArrow (:440-476) -- tail-to-head triangle-with-notch shape,
// rotated/translated onto `position` at heading `yaw`. Used as a heading
// indicator layered on top of each robot's footprint. `scale` (default 1.0)
// shrinks the whole polygon uniformly -- FEATURE B's reference-pose overlay
// passes a smaller value for its "small heading arrow" per spec.
void draw_oriented_arrow(QPainter &painter, const QPointF &position, double yaw,
                          const QColor &color, float scale = 1.0f)
{
  const float arrow_length = 14.0f * scale;
  const float arrow_width = 7.0f * scale;
  const float tail_length = 6.0f * scale;

  QPolygonF arrow;
  arrow << QPointF(-tail_length, -arrow_width / 2) << QPointF(0, -arrow_width / 2)
        << QPointF(0, -arrow_width) << QPointF(arrow_length, 0) << QPointF(0, arrow_width)
        << QPointF(0, arrow_width / 2) << QPointF(-tail_length, arrow_width / 2);

  QTransform transform;
  const float yaw_deg = static_cast<float>(yaw * 180.0 / M_PI);
  transform.translate(position.x(), position.y());
  transform.rotate(-yaw_deg);
  const QPolygonF transformed = transform.map(arrow);

  painter.setBrush(QBrush(color));
  painter.setPen(QPen(Qt::black, 1));
  painter.drawPolygon(transformed);
}

// GUI-MODE STARTUP DEFAULTS: SimVizWindow (this file) is the ONLY place
// mars_sim_viz --headless's bare SimVizManager is never routed through, so
// adjusting the config copy here changes GUI-launch behavior without
// touching SimVizConfig's own field defaults (SimVizCore.h) or
// simviz_main.cpp's CLI-flag defaults -- headless mode, and every test that
// drives ExecutionManager/SimVizManager directly rather than through
// SimVizWindow, still sees 0% noise / stall off / overlays off unless a
// flag is explicitly passed. Motor (accel) + steering noise default to 10%,
// motor stall defaults to ENABLED at SimVizConfig::stall_level's existing
// 0.10 m/s value, so a freshly-opened window has both switched on with no
// user touch. Noise sigma fields have no CLI "explicit zero" spelling
// distinct from "never passed" -- both read back as 0.0 -- so (like every
// other *_reflects_initial_config seam in this file) an explicit
// --noise-sigma-pct=/--steer-noise-sigma-pct= still wins over this default
// for any NONZERO value; only the true default (0.0) is bumped to 10%.
// stall_enabled has the same "no explicit off" shape as --deadband itself
// (there is no negating flag), so unconditionally defaulting it to true
// here is a pure no-op when --deadband was already passed.
SimVizConfig apply_gui_startup_defaults(SimVizConfig config)
{
  constexpr double kGuiDefaultNoiseSigmaPct = 0.10;
  if (config.noise_sigma_pct == 0.0)
    config.noise_sigma_pct = kGuiDefaultNoiseSigmaPct;
  if (config.steer_noise_sigma_pct == 0.0)
    config.steer_noise_sigma_pct = kGuiDefaultNoiseSigmaPct;
  config.stall_enabled = true;
  return config;
}

} // namespace

// ===========================================================================
// SimVizCanvas
// ===========================================================================

SimVizCanvas::SimVizCanvas(QWidget *parent) : QWidget(parent)
{
  setMinimumSize(400, 300);
}

void SimVizCanvas::set_scenario(std::shared_ptr<ScenarioModel> model)
{
  model_ = std::move(model);
  trails_.clear();
  last_seen_tag_.clear();
  update();
}

void SimVizCanvas::set_execution_manager(ExecutionManager *exec_mgr)
{
  exec_mgr_ = exec_mgr;
  update();
}

void SimVizCanvas::set_show_goal_outlines(bool show)
{
  show_goal_outlines_ = show;
  update();
}

void SimVizCanvas::set_show_reference_poses(bool show)
{
  show_reference_poses_ = show;
  update();
}

void SimVizCanvas::sample_trails()
{
  if (!model_ || !exec_mgr_)
    return;

  const auto now = std::chrono::steady_clock::now();

  for (EntityMeta *robot : model_->robots_sorted_by_name())
  {
    if (!robot)
      continue;
    const auto live = exec_mgr_->localization().latest_pose(robot->name);
    if (!live || !live->has_pose)
      continue;

    auto tag_it = last_seen_tag_.find(robot->name);
    const bool is_new =
        (tag_it == last_seen_tag_.end()) || (live->last_update_steady_s != tag_it->second);
    if (!is_new)
      continue;

    last_seen_tag_[robot->name] = live->last_update_steady_s;
    trails_[robot->name].push_back(TrailSample{live->pose, now});
  }

  for (auto &[name, deque] : trails_)
  {
    while (!deque.empty() &&
           std::chrono::duration<double>(now - deque.front().observed_at).count() >
               kTrailWindowS)
    {
      deque.pop_front();
    }
  }
}

std::optional<double>
SimVizCanvas::last_sample_age_s(const std::string &robot_name,
                                 std::chrono::steady_clock::time_point now) const
{
  auto it = trails_.find(robot_name);
  if (it == trails_.end() || it->second.empty())
    return std::nullopt;
  return std::chrono::duration<double>(now - it->second.back().observed_at).count();
}

void SimVizCanvas::paintEvent(QPaintEvent * /*event*/)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);
  painter.fillRect(rect(), Qt::white);

  if (!model_)
  {
    painter.setPen(Qt::darkGray);
    painter.drawText(rect(), Qt::AlignCenter, "No scenario loaded (File -> Open .scn.b64)");
    return;
  }

  const Params &params = model_->params();

  // FEATURE 1 (boundaries): the planner's workspace rect (params.min/max_x/y)
  // bounds each robot's REFERENCE POINT, not its physical body -- a robot's
  // footprint can extend up to `front_length` beyond that point (in the
  // direction of travel, e.g. while pushing). LF = the largest front_length
  // among the scenario's robot entities is therefore the largest amount any
  // robot's body can protrude past the inner (reference-point) boundary; the
  // workspace rect inflated by LF on every side is the OUTER boundary --
  // the true maximum physical extent any robot footprint can reach. Falls
  // back to 0.36 (this codebase's common default robot front_length --
  // see e.g. PHAstarPushDemoOptions.h) only when the scenario has no robot
  // entities at all to measure.
  double lf = 0.36;
  {
    const auto &robots = model_->robots_sorted_by_name();
    if (!robots.empty())
    {
      double max_front = 0.0;
      for (EntityMeta *robot : robots)
      {
        if (robot)
          max_front = std::max(max_front, robot->size.front_length);
      }
      lf = max_front;
    }
  }

  const double outer_min_x = params.min_x - lf;
  const double outer_max_x = params.max_x + lf;
  const double outer_min_y = params.min_y - lf;
  const double outer_max_y = params.max_y + lf;

  // The fit/aspect transform frames the OUTER rect (+ the existing pixel
  // margin below) rather than the inner workspace rect, so a robot body
  // protruding past the reference-point boundary is never clipped by the
  // canvas edge.
  const double workspace_width = std::max(outer_max_x - outer_min_x, 1e-3);
  const double workspace_height = std::max(outer_max_y - outer_min_y, 1e-3);

  // Aspect-preserving fit-and-center, ported from VisualizationWidget.cpp's
  // paintEvent (:185-230).
  const int padding = 10;
  const double available_w = std::max(width() - 2 * padding, 1);
  const double available_h = std::max(height() - 2 * padding, 1);
  const double aspect_ratio = workspace_width / workspace_height;
  const double available_ratio = available_w / available_h;

  QRectF draw_rect;
  if (available_ratio > aspect_ratio)
  {
    const double draw_h = available_h;
    const double draw_w = aspect_ratio * draw_h;
    draw_rect = QRectF((width() - draw_w) / 2.0, padding, draw_w, draw_h);
  }
  else
  {
    const double draw_w = available_w;
    const double draw_h = draw_w / aspect_ratio;
    draw_rect = QRectF(padding, (height() - draw_h) / 2.0, draw_w, draw_h);
  }

  const double scale = std::min(draw_rect.width() / workspace_width,
                                 draw_rect.height() / workspace_height);
  const double drawn_w = workspace_width * scale;
  const double drawn_h = workspace_height * scale;
  const double offset_x = draw_rect.left() + (draw_rect.width() - drawn_w) / 2.0;
  const double offset_y = draw_rect.top() + (draw_rect.height() - drawn_h) / 2.0;

  // Maps workspace coords (which may have non-zero min_x/min_y -- see
  // include/PHAstar/Visualization.h:753's same handling) into widget
  // coords, Y-flipped so larger y is higher on screen. Origin is now the
  // OUTER rect's corner (see workspace_width/height above), so the inner
  // boundary and every entity footprint still map correctly relative to it.
  auto map_coord = [&](double x, double y) -> QPointF
  {
    const double wx = offset_x + (x - outer_min_x) * scale;
    const double wy = offset_y + drawn_h - (y - outer_min_y) * scale;
    return QPointF(wx, wy);
  };

  // Inner boundary: the planner's actual workspace rect (bounds each
  // robot's REFERENCE POINT). Restyled dashed + semi-transparent so it
  // reads as the "logical" bound rather than a hard physical wall.
  QRectF inner_boundary(map_coord(params.min_x, params.max_y),
                         map_coord(params.max_x, params.min_y));
  QColor inner_color(Qt::black);
  inner_color.setAlpha(100); // ~90-110: semi-transparent
  QPen inner_pen(inner_color);
  inner_pen.setStyle(Qt::DashLine);
  painter.setPen(inner_pen);
  painter.setBrush(Qt::NoBrush);
  painter.drawRect(inner_boundary);

  // Outer boundary: the inner rect inflated by LF on every side -- the max
  // physical extent any robot body can reach. Solid pen, normal (full)
  // opacity so it reads as the true hard bound.
  QRectF outer_boundary(map_coord(outer_min_x, outer_max_y),
                         map_coord(outer_max_x, outer_min_y));
  QPen outer_pen(Qt::black);
  outer_pen.setStyle(Qt::SolidLine);
  painter.setPen(outer_pen);
  painter.setBrush(Qt::NoBrush);
  painter.drawRect(outer_boundary);

  auto footprint_polygon = [&](const Pose &p, const OccuRect &size) -> QPolygonF
  {
    QPolygonF poly;
    if (size.front_length <= 0.0 && size.rear_length <= 0.0 && size.width <= 0.0)
    {
      // No footprint geometry recorded -- draw a small placeholder marker
      // instead of nothing so the entity is still visible.
      constexpr double kMarkerHalf = 0.05;
      Corners c = get_corners(p.x, p.y, p.yaw, kMarkerHalf, kMarkerHalf, 2 * kMarkerHalf);
      for (const auto &pt : c)
        poly << map_coord(pt.x, pt.y);
      return poly;
    }
    Corners c = get_corners(p.x, p.y, p.yaw, size.front_length, size.rear_length, size.width);
    for (const auto &pt : c)
      poly << map_coord(pt.x, pt.y);
    return poly;
  };

  const double plan_time = exec_mgr_ ? exec_mgr_->plan_time() : 0.0;
  const bool running = exec_mgr_ && exec_mgr_->state() == RunState::Running;

  // --- Objects: timetable-replayed pose (piecewise-constant hold, per
  // ScenarioModel::object_pose_at / TimeTable::get_pose) + dashed goal
  // outline.
  for (EntityMeta *object : model_->objects_sorted_by_name())
  {
    if (!object)
      continue;
    const Pose pose = model_->object_pose_at(object, plan_time);

    painter.setPen(QPen(Qt::black, 1));
    QColor fill = kObjectFillColor;
    fill.setAlpha(160);
    painter.setBrush(fill);
    painter.drawPolygon(footprint_polygon(pose, object->size));

    // ISSUE 2: the dashed goal-pose outline is opt-in (default hidden --
    // see show_goal_outlines_'s doc comment in SimVizWindow.h), toggled via
    // the View menu's "Show goal outlines" action.
    if (show_goal_outlines_)
    {
      if (auto *obj_meta = dynamic_cast<ObjectMeta *>(object))
      {
        QPen dashed(kGoalOutlineColor);
        dashed.setStyle(Qt::DashLine);
        dashed.setWidth(2);
        painter.setPen(dashed);
        painter.setBrush(Qt::NoBrush);
        painter.drawPolygon(footprint_polygon(obj_meta->goal_pose, object->size));
      }
    }
  }

  // --- Robots: live localization pose (fallback: initial_pose) + fading
  // trail + footprint + heading arrow + name label.
  const auto &robots = model_->robots_sorted_by_name();
  for (size_t i = 0; i < robots.size(); ++i)
  {
    EntityMeta *robot = robots[i];
    if (!robot)
      continue;
    const QColor color = kRobotTraceColors[i % kRobotTraceColorCount];

    Pose pose = robot->initial_pose;
    if (exec_mgr_)
    {
      const auto live = exec_mgr_->localization().latest_pose(robot->name);
      if (live && live->has_pose)
        pose = live->pose;
    }

    // Fading trail.
    auto trail_it = trails_.find(robot->name);
    if (trail_it != trails_.end() && trail_it->second.size() >= 2)
    {
      const auto now = std::chrono::steady_clock::now();
      const auto &samples = trail_it->second;
      for (size_t s = 1; s < samples.size(); ++s)
      {
        const double age =
            std::chrono::duration<double>(now - samples[s].observed_at).count();
        const double alpha = std::clamp(1.0 - age / kTrailWindowS, 0.0, 1.0);
        if (alpha <= 0.0)
          continue;
        QColor trail_color = color;
        trail_color.setAlphaF(0.6 * alpha);
        QPen trail_pen(trail_color, 2);
        painter.setPen(trail_pen);
        painter.drawLine(map_coord(samples[s - 1].pose.x, samples[s - 1].pose.y),
                          map_coord(samples[s].pose.x, samples[s].pose.y));
      }
    }

    QColor fill = color;
    fill.setAlpha(150);
    painter.setPen(QPen(Qt::black, 1));
    painter.setBrush(fill);
    painter.drawPolygon(footprint_polygon(pose, robot->size));

    draw_oriented_arrow(painter, map_coord(pose.x, pose.y), pose.yaw, color);

    painter.setPen(Qt::black);
    painter.drawText(map_coord(pose.x, pose.y) + QPointF(8, -8),
                      QString::fromStdString(robot->name));
  }

  // --- FEATURE B: reference-pose overlay -- for each robot, an UNFILLED,
  // DASHED footprint outline + small heading arrow at the scenario
  // timetable's pose for THIS robot at the current plan clock (same
  // clock/table the object replay above uses; ScenarioModel/TimeTable's
  // public get_pose() interpolates for ROBOT entities -- see TimeTable::
  // pose_at()), in that robot's own palette color at reduced alpha (~120).
  // No fill -- visually distinct from the solid actual-pose footprint drawn
  // above. Opt-in (default hidden), toggled via the View menu's "Show
  // reference poses" action. Frozen while PAUSED / static after DONE come
  // for free: plan_time itself is frozen/stopped in those states.
  if (show_reference_poses_)
  {
    for (size_t i = 0; i < robots.size(); ++i)
    {
      EntityMeta *robot = robots[i];
      if (!robot)
        continue;

      QColor ref_color = kRobotTraceColors[i % kRobotTraceColorCount];
      ref_color.setAlpha(120);

      const Pose ref_pose = model_->timetable().get_pose(robot, plan_time);

      QPen ref_pen(ref_color);
      ref_pen.setStyle(Qt::DashLine);
      ref_pen.setWidth(2);
      painter.setPen(ref_pen);
      painter.setBrush(Qt::NoBrush);
      painter.drawPolygon(footprint_polygon(ref_pose, robot->size));

      draw_oriented_arrow(painter, map_coord(ref_pose.x, ref_pose.y), ref_pose.yaw, ref_color,
                           0.7f);
    }
  }

  (void)running; // reserved for future use (e.g. dimming when not running)
}

// ===========================================================================
// SimVizWindow
// ===========================================================================

SimVizWindow::SimVizWindow(SimVizConfig config, QWidget *parent)
    // GUI-MODE STARTUP DEFAULTS: config_ is declared before manager_ in
    // SimVizWindow.h, so member-initialization order (which follows
    // DECLARATION order, not this list's order) guarantees config_ already
    // holds the GUI-defaulted values by the time manager_(config_, this)
    // constructs its ExecutionManager from it -- see
    // apply_gui_startup_defaults()'s doc comment above for what changes and
    // why headless mode/every non-SimVizWindow test is unaffected.
    : QMainWindow(parent), config_(apply_gui_startup_defaults(std::move(config))),
      manager_(config_, this)
{
  setWindowTitle("mars_sim_viz");

  canvas_ = new SimVizCanvas(this);
  canvas_->set_execution_manager(&manager_.execution_manager());

  // RIGHT-SIDE PANEL: fixed-width column alongside the canvas, both under a
  // plain QHBoxLayout container set as the central widget -- a QDockWidget
  // would let the panel be dragged/floated/closed out from under the
  // always-present "Motor stall" controls, which this task's "fixed
  // right-hand panel" spec rules out. Canvas gets all the stretch; the
  // panel is fixed-width.
  QWidget *central = new QWidget(this);
  QHBoxLayout *central_layout = new QHBoxLayout(central);
  central_layout->setContentsMargins(0, 0, 0, 0);
  central_layout->addWidget(canvas_, /*stretch=*/1);

  QWidget *right_panel = new QWidget(this);
  right_panel->setFixedWidth(220);
  QVBoxLayout *panel_layout = new QVBoxLayout(right_panel);

  // MOTOR STALL: "Enable stall (min speed)" checkbox (GUI-MODE STARTUP
  // DEFAULT: checked -- see apply_gui_startup_defaults() above; headless/
  // SimVizConfig::stall_enabled's own field default stays UNCHECKED) +
  // "Stall level" spinbox (m/s, [0.00,0.25], step 0.01, default 0.10 --
  // unchanged, enabled only while checked). Initialized from
  // manager_.execution_manager().stall_enabled()/stall_level() (NOT
  // hardcoded) so a CLI-supplied initial --deadband/--stall-level= is
  // reflected from first paint -- same rationale as the toolbar noise
  // sliders below. Every change forwards straight to
  // ExecutionManager::set_stall_enabled()/set_stall_level(), which (1) is
  // what every FUTURE spawn reads, and (2) live-publishes the combined
  // sim_config payload to the currently active run (if any) -- see those
  // methods' doc comments in SimVizCore.h.
  QGroupBox *stall_group = new QGroupBox("Motor stall", right_panel);
  QVBoxLayout *stall_layout = new QVBoxLayout(stall_group);
  const bool initial_stall_enabled = manager_.execution_manager().stall_enabled();
  const double initial_stall_level = manager_.execution_manager().stall_level();
  stall_checkbox_ = new QCheckBox("Enable stall (min speed)", stall_group);
  stall_checkbox_->setChecked(initial_stall_enabled);
  stall_layout->addWidget(stall_checkbox_);
  stall_level_spin_ = new QDoubleSpinBox(stall_group);
  stall_level_spin_->setRange(0.00, 0.25);
  stall_level_spin_->setSingleStep(0.01);
  stall_level_spin_->setDecimals(2);
  stall_level_spin_->setSuffix(" m/s");
  stall_level_spin_->setValue(initial_stall_level);
  stall_level_spin_->setEnabled(initial_stall_enabled);
  stall_layout->addWidget(stall_level_spin_);
  connect(stall_checkbox_, &QCheckBox::toggled, this,
          [this](bool checked)
          {
            stall_level_spin_->setEnabled(checked);
            manager_.execution_manager().set_stall_enabled(checked);
          });
  connect(stall_level_spin_, qOverload<double>(&QDoubleSpinBox::valueChanged), this,
          [this](double value) { manager_.execution_manager().set_stall_level(value); });
  panel_layout->addWidget(stall_group);

  // STAGING PHASE: "Stage first" checkbox (default UNCHECKED, mirrors
  // manager_.staging_enabled() so a CLI-supplied initial --stage-first is
  // reflected from first paint). Forwards straight to
  // manager_.set_staging_enabled() -- unlike the stall/noise controls
  // above, this has no live-publish side effect on an already-running run
  // (staging is only ever consulted at EXECUTE time -- see
  // SimVizManager::start_execution_with_staging()'s doc comment).
  stage_first_checkbox_ = new QCheckBox("Stage first", right_panel);
  stage_first_checkbox_->setChecked(manager_.staging_enabled());
  connect(stage_first_checkbox_, &QCheckBox::toggled, this,
          [this](bool checked) { manager_.set_staging_enabled(checked); });
  panel_layout->addWidget(stage_first_checkbox_);

  // RIGHT-SIDE ROBOT MONITOR PANEL: cells are created lazily by
  // sync_monitor_panel_robots() (called from refresh_display(), wired to
  // the repaint timer below) once a scenario is loaded -- this is just
  // where they get appended.
  monitor_cells_layout_ = new QVBoxLayout();
  panel_layout->addLayout(monitor_cells_layout_);
  panel_layout->addStretch(1);

  central_layout->addWidget(right_panel, /*stretch=*/0);

  // DESIGN part C: central widget is now a QTabWidget -- tab 0 "Simulation"
  // is exactly the `central` widget built above (canvas + right panel,
  // UNCHANGED -- every existing seam/test that reaches through canvas()
  // keeps working identically); tab 1 "OptiTrack" is the new MocapTab.
  // Toolbar/menus/status bar stay window-level (added to `this` below, not
  // reparented into either tab), matching this task's design doc.
  tabs_ = new QTabWidget(this);
  tabs_->addTab(central, "Simulation");
  mocap_tab_ = new MocapTab(manager_, this);
  tabs_->addTab(mocap_tab_, "OptiTrack");
  // Calibration tab (teleop-driven velocity/steering map capture -- see
  // CalibrationTab.h's own header comment): added after the OptiTrack tab,
  // same "just another SimVizManager-driven tab" convention MocapTab
  // established.
  calibration_tab_ = new CalibrationTab(manager_, this);
  tabs_->addTab(calibration_tab_, "Calibration");
  setCentralWidget(tabs_);

  QMenu *file_menu = menuBar()->addMenu("&File");
  QAction *open_action = file_menu->addAction("&Open .scn.b64...");
  connect(open_action, &QAction::triggered, this, &SimVizWindow::on_open_action);

  // ISSUE 2: View menu -- "Show goal outlines" checkable action, default
  // UNCHECKED (the dashed #E76F51 boxes are hidden by default; see
  // SimVizCanvas::show_goal_outlines_'s doc comment). Checking it re-enables
  // the existing drawing code, which stays gated behind the flag.
  QMenu *view_menu = menuBar()->addMenu("&View");
  show_goal_outlines_action_ = view_menu->addAction("Show goal outlines");
  show_goal_outlines_action_->setCheckable(true);
  show_goal_outlines_action_->setChecked(false);
  connect(show_goal_outlines_action_, &QAction::toggled, this,
          [this](bool checked) { canvas_->set_show_goal_outlines(checked); });

  // FEATURE B / GUI-MODE STARTUP DEFAULTS: "Show reference poses" now
  // defaults to CHECKED (was unchecked) -- the connect() call is made
  // BEFORE setChecked(true) below (unlike "Show goal outlines" above, which
  // stays default-off) so that initial setChecked(true) fires its own
  // toggled(true) signal through the already-live connection and reaches
  // canvas_->set_show_reference_poses(true) immediately, exactly as if a
  // user had just clicked the menu item -- rather than requiring a second,
  // separate propagation call here.
  show_reference_poses_action_ = view_menu->addAction("Show reference poses");
  show_reference_poses_action_->setCheckable(true);
  connect(show_reference_poses_action_, &QAction::toggled, this,
          [this](bool checked) { canvas_->set_show_reference_poses(checked); });
  show_reference_poses_action_->setChecked(true);

  QToolBar *toolbar = addToolBar("Controls");

  // LOAD/START SPLIT: enabled only once a scenario is loaded and nothing is
  // running/staging (see start_available()'s doc comment); disabled at
  // construction since nothing has been loaded yet.
  start_action_ = toolbar->addAction("Start");
  connect(start_action_, &QAction::triggered, this, &SimVizWindow::on_start_action);
  start_action_->setEnabled(false);

  pause_resume_action_ = toolbar->addAction("Pause");
  connect(pause_resume_action_, &QAction::triggered, this,
          &SimVizWindow::on_pause_resume_action);
  pause_resume_action_->setEnabled(false);

  restart_action_ = toolbar->addAction("Restart");
  connect(restart_action_, &QAction::triggered, this, &SimVizWindow::on_restart_action);
  restart_action_->setEnabled(false);

  QAction *abort_action = toolbar->addAction("Abort");
  connect(abort_action, &QAction::triggered, this, &SimVizWindow::on_abort_action);

  // FEATURE C: ACCEL-channel noise-sigma slider (integer percent 0..25).
  // Every change forwards straight to ExecutionManager::
  // set_noise_sigma_pct(), which (1) is what every FUTURE spawn's
  // --noise-sigma-pct= reads, and (2) live-publishes {"noise_sigma_pct": v,
  // ...} to every robot in the CURRENTLY active run (if any) over the
  // sim-only "/<robot>/sim_config" topic -- see that method's doc comment in
  // SimVizCore.h.
  //
  // Initialized from manager_.execution_manager().noise_sigma_pct() (NOT a
  // hardcoded value) so a CLI-supplied initial --noise-sigma-pct= (threaded
  // in via SimVizConfig -> ExecutionManager's constructor, see
  // simviz_main.cpp) is reflected in the slider/label from first paint, not
  // just after the user manually drags it. GUI-MODE STARTUP DEFAULTS:
  // apply_gui_startup_defaults() (above) has already bumped config_'s
  // noise_sigma_pct from 0 to 10% by this point (unless a nonzero CLI value
  // was supplied), so this slider reads 10, not 0, on a plain `mars_sim_viz`
  // launch -- headless mode's own default stays 0.
  toolbar->addSeparator();
  toolbar->addWidget(new QLabel("Motor (accel) noise σ:", this));
  const int initial_noise_percent = static_cast<int>(
      std::lround(manager_.execution_manager().noise_sigma_pct() * 100.0));
  noise_slider_ = new QSlider(Qt::Horizontal, this);
  noise_slider_->setRange(0, 25);
  noise_slider_->setValue(initial_noise_percent);
  noise_slider_->setFixedWidth(120);
  toolbar->addWidget(noise_slider_);
  noise_value_label_ =
      new QLabel(QString("σ = %1% of limit").arg(initial_noise_percent), this);
  toolbar->addWidget(noise_value_label_);
  connect(noise_slider_, &QSlider::valueChanged, this,
          [this](int percent)
          {
            manager_.execution_manager().set_noise_sigma_pct(percent / 100.0);
            noise_value_label_->setText(QString("σ = %1% of limit").arg(percent));
          });

  // PART (1) STEERING-NOISE SPLIT: independent STEER-channel slider,
  // mirroring the ACCEL-channel one above exactly (same range/default,
  // forwards to ExecutionManager::set_steer_noise_sigma_pct(), initialized
  // from manager_.execution_manager().steer_noise_sigma_pct() so a
  // CLI-supplied initial --steer-noise-sigma-pct= is reflected from first
  // paint).
  toolbar->addSeparator();
  toolbar->addWidget(new QLabel("Steering noise σ:", this));
  const int initial_steer_noise_percent = static_cast<int>(
      std::lround(manager_.execution_manager().steer_noise_sigma_pct() * 100.0));
  steer_noise_slider_ = new QSlider(Qt::Horizontal, this);
  steer_noise_slider_->setRange(0, 25);
  steer_noise_slider_->setValue(initial_steer_noise_percent);
  steer_noise_slider_->setFixedWidth(120);
  toolbar->addWidget(steer_noise_slider_);
  steer_noise_value_label_ =
      new QLabel(QString("σ = %1% of limit").arg(initial_steer_noise_percent), this);
  toolbar->addWidget(steer_noise_value_label_);
  connect(steer_noise_slider_, &QSlider::valueChanged, this,
          [this](int percent)
          {
            manager_.execution_manager().set_steer_noise_sigma_pct(percent / 100.0);
            steer_noise_value_label_->setText(QString("σ = %1% of limit").arg(percent));
          });

  status_label_ = new QLabel("IDLE", this);
  statusBar()->addWidget(status_label_);

  tick_timer_ = new QTimer(this);
  connect(tick_timer_, &QTimer::timeout, this, &SimVizWindow::on_tick);

  repaint_timer_ = new QTimer(this);
  connect(repaint_timer_, &QTimer::timeout, this, &SimVizWindow::refresh_display);

  resize(1120, 700);
}

void SimVizWindow::start()
{
  manager_.start();
  tick_timer_->start(50);
  repaint_timer_->start(33);
}

bool SimVizWindow::open_scenario_file(const QString &path)
{
  std::shared_ptr<ScenarioModel> model;
  try
  {
    model = std::make_shared<ScenarioModel>(ScenarioModel::load_from_file(path.toStdString()));
  }
  catch (const std::exception &ex)
  {
    QMessageBox::warning(this, "mars_sim_viz",
                          QString("Failed to load scenario: %1").arg(ex.what()));
    return false;
  }

  if (manager_.execution_manager().is_busy())
  {
    QMessageBox::warning(this, "mars_sim_viz",
                          "A run is already in progress; abort it before starting another.");
    return false;
  }

  canvas_->set_scenario(model);

  // LOAD/START SPLIT: load + preview only -- NOT
  // manager_.start_execution_with_staging() anymore (see this file's header
  // doc comment). loaded_model_/loaded_path_ are what on_start_action()
  // (the toolbar Start button) actually executes, at PRESS time rather than
  // load time, so the operator can connect/map mocap in between.
  loaded_model_ = model;
  loaded_path_ = path.toStdString();

  update_status_bar();
  return true;
}

bool SimVizWindow::start_available() const
{
  if (!loaded_model_)
    return false;
  const ExecutionManager &mgr = manager_.execution_manager();
  return !mgr.is_busy() && !manager_.is_staging_active() && !manager_.is_stage_real_active();
}

void SimVizWindow::on_start_action()
{
  if (!start_available())
    return;

  // STAGING PHASE: go through SimVizManager's shared entry point (rather
  // than ExecutionManager::start_execution() directly) so a GUI-driven
  // Start honors config_.staging_enabled / the "Stage first" checkbox
  // exactly like a headless/control-socket EXECUTE does. When staging is
  // off (the default), this is a byte-identical passthrough to the old
  // call -- see start_execution_with_staging()'s doc comment in
  // SimVizCore.h. The mixed-fleet real-vs-sim decision (ExecutionManager::
  // start_execution()'s robot_is_real) is evaluated INSIDE this call, i.e.
  // at PRESS time, using whatever mocap connection/mapping state exists
  // right now.
  std::string err;
  const bool accepted = manager_.start_execution_with_staging(loaded_model_, &err);
  if (!accepted)
  {
    QMessageBox::warning(this, "mars_sim_viz",
                          err.empty() ? QString("Execution manager rejected the run (busy).")
                                      : QString::fromStdString(err));
    return;
  }

  // FEATURE 2C: so restart() (triggered via the control socket OR the
  // toolbar's Restart button) can reload this same file later -- mirrors
  // what execute_scenario_file() does for its own (control-socket/--execute)
  // callers.
  manager_.note_scenario_path(loaded_path_);

  update_status_bar();
}

void SimVizWindow::on_tick()
{
  manager_.tick();
}

void SimVizWindow::on_open_action()
{
  const QString path =
      QFileDialog::getOpenFileName(this, "Open scenario", QString(), "Scenarios (*.scn.b64)");
  if (path.isEmpty())
    return;
  open_scenario_file(path);
}

void SimVizWindow::on_abort_action()
{
  manager_.execution_manager().abort();
  update_status_bar();
}

void SimVizWindow::on_pause_resume_action()
{
  ExecutionManager &mgr = manager_.execution_manager();
  const std::string reply = mgr.is_paused() ? manager_.resume() : manager_.pause();
  if (reply.rfind("ERR", 0) == 0)
    QMessageBox::warning(this, "mars_sim_viz", QString::fromStdString(reply));
  update_status_bar();
}

void SimVizWindow::on_restart_action()
{
  const std::string reply = manager_.restart();
  if (reply.rfind("ERR", 0) == 0)
  {
    QMessageBox::warning(this, "mars_sim_viz", QString::fromStdString(reply));
  }
  else
  {
    // restart() reloads a brand-new ScenarioModel via execute_scenario_file()
    // (fresh EntityMeta pointers, fresh trajectories) -- keep the canvas's
    // own model_ (and its trails_, cleared by set_scenario()) in sync with
    // it, mirroring what open_scenario_file() does for a normal File->Open.
    canvas_->set_scenario(manager_.execution_manager().scenario_model());
  }
  update_status_bar();
}

void SimVizWindow::sync_monitor_panel_robots()
{
  // LOAD/START SPLIT: scenario_model() stays null until Start is actually
  // pressed -- fall back to loaded_model_ so the monitor panel populates as
  // soon as a scenario is loaded, not only once it starts running.
  auto model = manager_.execution_manager().scenario_model();
  if (!model)
    model = loaded_model_;
  if (model == monitor_panel_model_)
    return; // already built for this scenario (or both null -- nothing loaded yet).

  // A genuinely new scenario (or none at all) loaded -- clear whatever
  // cells exist and rebuild from scratch, mirroring SimVizCanvas::
  // set_scenario()'s own "clear on new model" handling for trails.
  for (auto &[name, cell] : monitor_cells_)
  {
    if (cell.widget)
      cell.widget->deleteLater();
  }
  monitor_cells_.clear();
  monitor_panel_model_ = model;

  if (!model)
    return;

  const auto &robots = model->robots_sorted_by_name();
  for (size_t i = 0; i < robots.size(); ++i)
  {
    EntityMeta *robot = robots[i];
    if (!robot)
      continue;
    // Same palette/index convention as SimVizCanvas::paintEvent()'s robot
    // loop (kRobotTraceColors, this file's anonymous namespace above), so a
    // robot's monitor cell reads as the same color as its canvas footprint.
    const QColor color = kRobotTraceColors[i % kRobotTraceColorCount];

    MonitorCell cell;
    cell.widget = new QWidget();
    QVBoxLayout *cell_layout = new QVBoxLayout(cell.widget);
    cell_layout->setContentsMargins(4, 4, 4, 4);

    QHBoxLayout *header_layout = new QHBoxLayout();
    cell.swatch_label = new QLabel(cell.widget);
    cell.swatch_label->setFixedSize(12, 12);
    cell.swatch_label->setStyleSheet(
        QString("background-color: %1; border: 1px solid black;").arg(color.name()));
    header_layout->addWidget(cell.swatch_label);
    cell.name_label = new QLabel(QString::fromStdString(robot->name), cell.widget);
    cell.name_label->setStyleSheet(QString("font-weight: bold; color: %1;").arg(color.name()));
    header_layout->addWidget(cell.name_label);
    header_layout->addStretch(1);
    cell_layout->addLayout(header_layout);

    // "—" placeholders until telemetry arrives (e.g. real hardware without
    // a telemetry source) -- update_monitor_panel_values() fills these in.
    cell.v_label = new QLabel("v: —", cell.widget);
    cell_layout->addWidget(cell.v_label);
    cell.steering_label = new QLabel("δ: —", cell.widget);
    cell_layout->addWidget(cell.steering_label);
    cell.accel_label = new QLabel("a: —", cell.widget);
    cell_layout->addWidget(cell.accel_label);
    cell.state_label = new QLabel("—", cell.widget);
    cell_layout->addWidget(cell.state_label);
    // LAG READOUT (three-clock lag-diagnosis task): same "—" until a live
    // pose lets update_monitor_panel_values() compute a real value.
    cell.lag_label = new QLabel("lag: —", cell.widget);
    cell_layout->addWidget(cell.lag_label);

    monitor_cells_layout_->addWidget(cell.widget);
    monitor_cells_[robot->name] = cell;
  }
}

void SimVizWindow::update_monitor_panel_values()
{
  ExecutionManager &mgr = manager_.execution_manager();
  for (auto &[name, cell] : monitor_cells_)
  {
    // LAG READOUT (three-clock lag-diagnosis task): independent of the
    // telemetry-gated block below -- lag_seconds() only needs a live POSE
    // (a separate PUB topic from telemetry, see LocalizationListener's doc
    // comment), so this updates even before/without telemetry ever
    // arriving. > 2s is styled red (brief's "color/tag it when > 2s"); "—"
    // (default color) until lag_seconds() has anything to report.
    if (cell.lag_label)
    {
      auto lag = mgr.lag_seconds(name);
      if (lag.has_value())
      {
        cell.lag_label->setText(QString("lag: %1s").arg(*lag, 0, 'f', 2));
        cell.lag_label->setStyleSheet(*lag > 2.0 ? "color: red; font-weight: bold;" : "");
      }
      else
      {
        cell.lag_label->setText("lag: —");
        cell.lag_label->setStyleSheet("");
      }
    }

    auto telem = mgr.localization().latest_telemetry(name);
    if (!telem || !telem->has_telemetry)
    {
      // No telemetry has arrived yet for this robot this run (or ever --
      // e.g. real hardware with no telemetry source) -- "—" for every
      // field, including the state tag (nothing is actually known about
      // moving/watchdog state either without at least one sample).
      cell.v_label->setText("v: —");
      cell.steering_label->setText("δ: —");
      cell.accel_label->setText("a: —");
      cell.state_label->setText("—");
      continue;
    }

    cell.v_label->setText(QString("v: %1 m/s").arg(telem->v, 0, 'f', 2));
    cell.steering_label->setText(QString("δ: %1 rad").arg(telem->steering, 0, 'f', 2));
    cell.accel_label->setText(QString("a: %1 m/s²").arg(telem->accel, 0, 'f', 2));

    // Priority order per spec: STALLED first (moving=0 but a nonzero
    // command is still being asked for), then WATCHDOG, else the plain
    // MOVING/IDLE reading of the moving flag.
    QString state_tag;
    if (!telem->moving && std::fabs(telem->v_cmd) > 0.02)
      state_tag = "STALLED";
    else if (telem->watchdog)
      state_tag = "WATCHDOG";
    else
      state_tag = telem->moving ? "MOVING" : "IDLE";
    cell.state_label->setText(state_tag);
  }
}

std::optional<SimVizWindow::MonitorCellText>
SimVizWindow::monitor_cell_text(const std::string &robot_name) const
{
  auto it = monitor_cells_.find(robot_name);
  if (it == monitor_cells_.end())
    return std::nullopt;
  const MonitorCell &cell = it->second;
  MonitorCellText out;
  out.velocity = cell.v_label ? cell.v_label->text().toStdString() : "";
  out.steering = cell.steering_label ? cell.steering_label->text().toStdString() : "";
  out.accel = cell.accel_label ? cell.accel_label->text().toStdString() : "";
  out.state_tag = cell.state_label ? cell.state_label->text().toStdString() : "";
  out.lag = cell.lag_label ? cell.lag_label->text().toStdString() : "";
  return out;
}

void SimVizWindow::refresh_display()
{
  canvas_->sample_trails();
  canvas_->update();
  sync_monitor_panel_robots();
  update_monitor_panel_values();
  update_status_bar();
  // DESIGN part C: OptiTrack tab's own live-data refresh -- same "one call
  // per repaint tick, also directly callable by tests" convention as every
  // other block in this method.
  if (mocap_tab_)
    mocap_tab_->refresh();
  if (calibration_tab_)
    calibration_tab_->refresh();
}

void SimVizWindow::update_status_bar()
{
  ExecutionManager &mgr = manager_.execution_manager();
  std::ostringstream oss;
  // LOAD/START SPLIT: a load with no execution started yet reads "Loaded"
  // rather than the ExecutionManager's own "Idle" (which no longer
  // distinguishes "nothing loaded" from "loaded, waiting for Start").
  // FEATURE 2C: status bar shows PAUSED distinctly from a normally-running
  // run.
  if (mgr.is_paused())
    oss << "Paused";
  else if (mgr.state() == RunState::Idle && loaded_model_ && !mgr.scenario_model())
    oss << "Loaded";
  else
    oss << run_state_name(mgr.state());
  const std::string label = mgr.label();
  if (!label.empty())
    oss << "  label=" << label;
  if (mgr.state() == RunState::Running || mgr.state() == RunState::Done)
    oss << "  t=" << mgr.plan_time();
  if (mgr.state() == RunState::Err)
    oss << "  err=" << mgr.error_reason();

  // LOAD/START SPLIT: scenario_model() stays null until Start is actually
  // pressed -- fall back to loaded_model_ so the per-robot pose readout
  // below (and Restart's enablement) reflects a freshly-loaded scenario
  // immediately.
  auto model = mgr.scenario_model();
  if (!model)
    model = loaded_model_;

  // FEATURE 2C: toolbar Pause<->Resume toggle (label flips, enabled only in
  // RUNNING/PAUSED) + Restart (enabled whenever a scenario has ever been
  // loaded).
  const bool running_or_paused = (mgr.state() == RunState::Running);
  pause_resume_action_->setEnabled(running_or_paused);
  pause_resume_action_->setText(mgr.is_paused() ? "Resume" : "Pause");
  restart_action_->setEnabled(mgr.scenario_model() != nullptr);
  // LOAD/START SPLIT: enabled iff a scenario is loaded and nothing is
  // running/staging -- see start_available()'s doc comment.
  start_action_->setEnabled(start_available());
  if (model)
  {
    oss << "  |";
    const auto now = std::chrono::steady_clock::now();
    for (EntityMeta *robot : model->robots_sorted_by_name())
    {
      if (!robot)
        continue;
      oss << "  " << robot->name << ":";
      auto live = mgr.localization().latest_pose(robot->name);
      if (!live || !live->has_pose)
      {
        oss << "no-data";
        continue;
      }
      // Age approximated via the canvas's own trail-sampling observation
      // time (see SimVizCanvas::sample_trails' doc comment) rather than
      // LocalizationListener's internal steady-clock domain, which is not
      // directly comparable to a fresh steady_clock::now() read taken here.
      auto age = canvas_->last_sample_age_s(robot->name, now);
      if (age)
        oss << QString::number(*age, 'f', 2).toStdString() << "s";
      else
        oss << "~0s";
    }
  }

  status_label_->setText(QString::fromStdString(oss.str()));
}

} // namespace simviz
