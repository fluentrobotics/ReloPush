/**
 * @file sh_astar.cpp
 * @author Licheng Wen (wenlc@zju.edu.cn)
 * @brief The implement of Spatiotemporal Hybrid-State Astar for single_agent
 * @date 2020-11-12
 *
 * @copyright Copyright (c) 2020
 *
 */

#ifndef PATH_PLANNING_TOOLS_H
#define PATH_PLANNING_TOOLS_H

#include <math.h>
#include <ompl/base/State.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <reloPush/parse_testdata.h>

#include <boost/functional/hash.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>

typedef ompl::base::SE2StateSpace::StateType OmplState;
typedef boost::geometry::model::d2::point_xy<double> Point;
typedef boost::geometry::model::segment<Point> Segment;

#include "hybrid_astar.hpp"
#include <omplTools/State.h>
#include <reloPush/params.h>
//#include "timer.hpp"

using libMultiRobotPlanning::HybridAStar;
using libMultiRobotPlanning::Neighbor;
using libMultiRobotPlanning::PlanResult;
using namespace libMultiRobotPlanning;

struct PlanningContext{
  bool allow_reverse = true;
  float turning_radius = 1.0f; //r
  float deltat;
  float speed_limit = 0.38f;
  float LF = 0.38;

  float xyResolution;
  float yawResolution;
  std::vector<double> dx;
  std::vector<double> dy;
  std::vector<double> dyaw;

  PlanningContext();

  PlanningContext(bool use_reverse, float turning_r, float LF_in ,float speed_lim = 0.385);

  void update_dx();
  void update_dy();
  void update_dyaw();  
};

namespace Constants {
  static float steer_limit_push = 0.185; // 0.185
  static float steer_limit_nonpush = 0.28; // 0.28
  static float speed_limit = 0.36f; //0.4 // slightly slower than driving speed
  static float L = 0.29f;
  // [m] --- The minimum turning radius of the vehicle
  static float r_push = L / tanf(fabs(steer_limit_push));
  static float r_nonpush = L / tanf(fabs(steer_limit_nonpush));
  //extern float r; // non-push as default
  //static float r = 0.5;
  //static const float r = 3;
  //static const float deltat = 6.75 / 180.0 * M_PI;
  static float deltat_push = speed_limit / r_push / 1.5;
  static float deltat_nonpush = speed_limit / r_nonpush / 1.5;
  //extern float deltat; // non-push as default
  // [#] --- A movement cost penalty for turning (choosing non straight motion
  // primitives)
  static const float penaltyTurning = 50;
  // [#] --- A movement cost penalty for reversing (choosing motion primitives >
  // 2)
  static const float penaltyReversing = 3.0; //8
  // [#] --- A movement cost penalty for change of direction (changing from
  // primitives < 3 to primitives > 2)
  static const float penaltyCOD = 5.0;

  //extern bool allow_reverse; // only when not pushing

  static float heuristicWeight = 1.0f;

  // map resolution
  static const float mapResolution = 0.1;

  static const float xyResolution_push = r_push * deltat_push;
  static const float xyResolution_nonpush = r_nonpush * deltat_nonpush;
  //extern float xyResolution; // non-push as default

  static const float yawResolution_push = deltat_push;
  static const float yawResolution_nonpush = deltat_nonpush;
  //extern float yawResolution; // non-push as default

  // width of car
  static const float carWidth = 0.285;
  // distance from rear to vehicle front end
  static const float LF_nonpush = 0.3;  //0.38
  static const float LF_push = 0.45; // 0.65
  // distance from rear to vehicle back end
  static const float LB = 0.12;
  // obstacle default radius
  static const float obsRadius = 0.075;

  // R = 3, 6.75 DEG
  //extern double dx[];
  //extern double dy[];
  //extern double dyaw[];

  float normalizeHeadingRad(float t);

  /*
  void update_dx();
  void update_dy();
  void update_dyaw();
  void update_dx_dy_dyaw();
  void switch_to_pushing();
  void switch_to_nonpushing();
  */
}  // namespace Constants


// calculate agent collision more precisely BUT need LONGER time
// #define PRCISE_COLLISION
namespace std {
template <>
struct hash<State> {
  size_t operator()(const State& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.x);
    boost::hash_combine(seed, s.y);
    boost::hash_combine(seed, s.yaw);
    return seed;
  }
};
}  // namespace std

using Action = int;  // Action < 6

// for checking state validity of a path
enum StateValidity {valid, collision, out_of_boundary};
enum PlanValidity {success, start_inval, goal_inval, no_sol};

typedef PlanResult<State, Action, double> PlanResultType;

struct PathPlanResult : PlanResultType
{
  State start_pose;
  State goal_pose;
  State nominal_goal_pose; // if planned with pre-push pose, store the original goal pose here
  State obs_rm; // need to remove this obstacle from env before planning
  State obs_add;
  PlanValidity validity;
  
  PathPlanResult()
  {
    start_pose = State();
    goal_pose = State();
    success = false;
  }
  PathPlanResult(State& start_in, State& goal_in) 
                : start_pose(start_in), goal_pose(goal_in)
  {
    success = false;
  }
  PathPlanResult(State& start_in, State& goal_in, PlanValidity val)
                : start_pose(start_in), goal_pose(goal_in), validity(val)
                {
                  success = false;
                }
  PathPlanResult(State& start_in, State& goal_in, State& obs_to_rm, State& obs_to_add) 
                : start_pose(start_in), goal_pose(goal_in), obs_rm(obs_to_rm), obs_add(obs_to_add)
  {
    success = false;
  }
};


typedef std::shared_ptr<PathPlanResult> PathPlanResultPtr;

// bool and reason
struct StateValiditySet{
  std::pair<bool, StateValidity> data;

  StateValiditySet(bool is_valid, StateValidity validity) : data(is_valid,validity) {}

  operator bool() const {
        return data.first;
    }

    StateValidity get_validity()
    {
      return data.second;
    }
};

class Environment {
 public:

  Environment(){};

/*
  Environment(size_t maxx, size_t maxy, std::unordered_set<State> obstacles,
              State goal = State(0,0,0))
      : m_obstacles(std::move(obstacles)),
        m_goal(goal)  // NOLINT
  {
    m_dimx = static_cast<int>(maxx / Constants::mapResolution);
    m_dimy = static_cast<int>(maxy / Constants::mapResolution);
    // std::cout << "env build " << m_dimx << " " << m_dimy << " "
    //           << m_obstacles.size() << std::endl;
    holonomic_cost_map = std::vector<std::vector<double>>(
        m_dimx, std::vector<double>(m_dimy, 0));
    m_goal = State(goal.x, goal.y, Constants::normalizeHeadingRad(goal.yaw));
    updateCostmap();

    //test dynamic_obs
    
    dynamic_obs.insert(std::pair<int,State>(1,State(2.46318, 2.44999, 0,1)));
    dynamic_obs.insert(std::pair<int,State>(2,State(2.06318, 2.44999, 0,2)));
    dynamic_obs.insert(std::pair<int,State>(3,State(1.76318, 2.44999, 0,3)));
    dynamic_obs.insert(std::pair<int,State>(4,State(1.46318, 2.44999, 0,4)));
    dynamic_obs.insert(std::pair<int,State>(5,State(1.16318, 2.44999, 0,5)));
    dynamic_obs.insert(std::pair<int,State>(6,State(0.86318, 2.44999, 0,6)));
    dynamic_obs.insert(std::pair<int,State>(7,State(0.56318, 2.44999, 0,7)));
    std::cout << "test dobs addeed" << std::endl;
    
  }
  */
  
  Environment(float maxx, float maxy, std::unordered_set<State> obstacles, float turning_rad_in, float LF_in, bool use_reverse,
            State goal = State(0,0,0), float speed = 0.385f)
    : m_obstacles(std::move(obstacles)),
      m_goal(goal)  // NOLINT
  {
    // set planning context
    planCont = PlanningContext(use_reverse, turning_rad_in, LF_in, speed);

    m_dimx = static_cast<int>(maxx / static_cast<float>(Constants::mapResolution));
    m_dimy = static_cast<int>(maxy / static_cast<float>(Constants::mapResolution));
    // std::cout << "env build " << m_dimx << " " << m_dimy << " "
    //           << m_obstacles.size() << std::endl;
    holonomic_cost_map = std::vector<std::vector<double>>(
        m_dimx, std::vector<double>(m_dimy, 0));
    m_goal = State(goal.x, goal.y, Constants::normalizeHeadingRad(goal.yaw));
    updateCostmap();
  }

  void changeGoal(State goal_in)
  {
    m_goal = State(goal_in.x, goal_in.y, Constants::normalizeHeadingRad(goal_in.yaw));
    updateCostmap();
  }

  void changeLF(float lf)
  {
    planCont.LF = lf;
  }

  struct compare_node {
    bool operator()(const std::pair<State, double>& n1,
                    const std::pair<State, double>& n2) const {
      return (n1.second > n2.second);
    }
  };

  /*
  uint64_t calcIndex(const State& s) {
    return (uint64_t)(Constants::normalizeHeadingRad(s.yaw) /
                      Constants::yawResolution) *
               (m_dimx * Constants::mapResolution / Constants::xyResolution) *
               (m_dimy * Constants::mapResolution / Constants::xyResolution) +
           (uint64_t)(s.y / Constants::xyResolution) *
               (m_dimx * Constants::mapResolution / Constants::xyResolution) +
           (uint64_t)(s.x / Constants::xyResolution);
  }
  */
  uint64_t calcIndex(const State& s) {
    return (uint64_t)(Constants::normalizeHeadingRad(s.yaw) /
                      planCont.yawResolution) *
               (m_dimx * Constants::mapResolution / planCont.xyResolution) *
               (m_dimy * Constants::mapResolution / planCont.xyResolution) +
           (uint64_t)(s.y / planCont.xyResolution) *
               (m_dimx * Constants::mapResolution / planCont.xyResolution) +
           (uint64_t)(s.x / planCont.xyResolution);
  }

/*
   int admissibleHeuristic(const State &s) {
    double reedsSheppCost = 0;
    // non-holonomic-without-obstacles heuristic: use a Reeds-Shepp (or Dubins if reversing not allowed)
    std::unique_ptr<ompl::base::SE2StateSpace> path(
        Constants::allow_reverse ? (ompl::base::SE2StateSpace *)(new ompl::base::ReedsSheppStateSpace(Constants::r))
                                 : (ompl::base::SE2StateSpace *)(new ompl::base::DubinsStateSpace(Constants::r))
    );
    OmplState* rsStart = (OmplState *)path->allocState();
    OmplState* rsEnd = (OmplState *)path->allocState();
    rsStart->setXY(s.x, s.y);
    rsStart->setYaw(s.yaw);
    rsEnd->setXY(m_goal.x, m_goal.y);
    rsEnd->setYaw(m_goal.yaw);
    reedsSheppCost = path->distance(rsStart, rsEnd);
    path->freeState(rsStart);
    path->freeState(rsEnd);
    // std::cout << "ReedsShepps cost:" << reedsSheppCost << std::endl;
    // Euclidean distance
    double euclideanCost =
        sqrt(pow(m_goal.x - s.x, 2) + pow(m_goal.y - s.y, 2));
    // std::cout << "Euclidean cost:" << euclideanCost << std::endl;
    // holonomic-with-obstacles heuristic
    double twoDoffset = sqrt(pow((s.x - static_cast<int>(s.x)) -
                                     (m_goal.x - static_cast<int>(m_goal.x)),
                                 2) +
                             pow((s.y - static_cast<int>(s.y)) -
                                     (m_goal.y - static_cast<int>(m_goal.y)),
                                 2));
    double twoDCost =
        holonomic_cost_map[static_cast<int>(s.x / Constants::mapResolution)]
                          [static_cast<int>(s.y / Constants::mapResolution)] -
        twoDoffset;
    // std::cout << "holonomic cost:" << twoDCost << std::endl;

    return Constants::heuristicWeight * std::max({reedsSheppCost, euclideanCost, twoDCost});
  }
  */

    int admissibleHeuristic(const State &s) {
      double reedsSheppCost = 0;
      // non-holonomic-without-obstacles heuristic: use a Reeds-Shepp (or Dubins if reversing not allowed)
      std::unique_ptr<ompl::base::SE2StateSpace> path(
          planCont.allow_reverse ? (ompl::base::SE2StateSpace *)(new ompl::base::ReedsSheppStateSpace(planCont.turning_radius))
                                  : (ompl::base::SE2StateSpace *)(new ompl::base::DubinsStateSpace(planCont.turning_radius))
      );
      OmplState* rsStart = (OmplState *)path->allocState();
      OmplState* rsEnd = (OmplState *)path->allocState();
      rsStart->setXY(s.x, s.y);
      rsStart->setYaw(s.yaw);
      rsEnd->setXY(m_goal.x, m_goal.y);
      rsEnd->setYaw(m_goal.yaw);
      reedsSheppCost = path->distance(rsStart, rsEnd);
      path->freeState(rsStart);
      path->freeState(rsEnd);
      // std::cout << "ReedsShepps cost:" << reedsSheppCost << std::endl;
      // Euclidean distance
      double euclideanCost =
          sqrt(pow(m_goal.x - s.x, 2) + pow(m_goal.y - s.y, 2));
      // std::cout << "Euclidean cost:" << euclideanCost << std::endl;
      // holonomic-with-obstacles heuristic
      double twoDoffset = sqrt(pow((s.x - static_cast<int>(s.x)) -
                                      (m_goal.x - static_cast<int>(m_goal.x)),
                                  2) +
                              pow((s.y - static_cast<int>(s.y)) -
                                      (m_goal.y - static_cast<int>(m_goal.y)),
                                  2));
      double twoDCost =
          holonomic_cost_map[static_cast<int>(s.x / Constants::mapResolution)]
                            [static_cast<int>(s.y / Constants::mapResolution)] -
          twoDoffset;
      // std::cout << "holonomic cost:" << twoDCost << std::endl;

      return Constants::heuristicWeight * std::max({reedsSheppCost, euclideanCost, twoDCost});
  }

  /*
   bool isSolution(
      const State &state, double gscore,
      std::unordered_map<State, std::tuple<State, Action, double, double>,
                         std::hash<State>> &_camefrom) {
    
    bool isSol = Constants::allow_reverse ? isSolutionWithReverse(state, gscore, _camefrom) : isSolutionWithoutReverse(state, gscore, _camefrom);
    
    return isSol;
  }
  */

  bool isSolution(
    const State &state, double gscore,
    std::unordered_map<State, std::tuple<State, Action, double, double>,
                        std::hash<State>> &_camefrom) {
    
    bool isSol = planCont.allow_reverse ? isSolutionWithReverse(state, gscore, _camefrom) : isSolutionWithoutReverse(state, gscore, _camefrom);
    
    return isSol;
  }

  bool isSolutionWithReverse(
      const State& state, double gscore,
      std::unordered_map<State, std::tuple<State, Action, double, double>,
                         std::hash<State>>& _camefrom) {
    double goal_distance =
        sqrt(pow(state.x - m_goal.x, 2) + pow(state.y - m_goal.y, 2));
    if (goal_distance > 2 * (Constants::LB + planCont.LF)) return false;

    //ompl::base::ReedsSheppStateSpace reedsSheppSpace(Constants::r);
    ompl::base::ReedsSheppStateSpace reedsSheppSpace(planCont.turning_radius);
    OmplState* rsStart = (OmplState*)reedsSheppSpace.allocState();
    OmplState* rsEnd = (OmplState*)reedsSheppSpace.allocState();
    rsStart->setXY(state.x, state.y);
    rsStart->setYaw(-state.yaw);
    rsEnd->setXY(m_goal.x, m_goal.y);
    rsEnd->setYaw(-m_goal.yaw);
    ompl::base::ReedsSheppStateSpace::ReedsSheppPath reedsShepppath =
        reedsSheppSpace.reedsShepp(rsStart, rsEnd);

    std::vector<State> path;
    std::unordered_map<State, std::tuple<State, Action, double, double>,
                       std::hash<State>>
        cameFrom;
    cameFrom.clear();
    path.emplace_back(state);
    for (auto pathidx = 0; pathidx < 5; pathidx++) {
      if (fabs(reedsShepppath.length_[pathidx]) < 1e-6) continue;
      double deltat = 0, dx = 0, act = 0, cost = 0;
      switch (reedsShepppath.type_[pathidx]) {
        case 0:  // RS_NOP
          continue;
          break;
        case 1:  // RS_LEFT
          deltat = -reedsShepppath.length_[pathidx];
          //dx = Constants::r * sin(-deltat);
          dx = planCont.turning_radius * sin(-deltat);
          // dy = Constants::r * (1 - cos(-deltat));
          act = 2;
          //cost = reedsShepppath.length_[pathidx] * Constants::r * Constants::penaltyTurning;
          cost = reedsShepppath.length_[pathidx] * planCont.turning_radius * Constants::penaltyTurning;
          break;
        case 2:  // RS_STRAIGHT
          deltat = 0;
          //dx = reedsShepppath.length_[pathidx] * Constants::r;
          dx = reedsShepppath.length_[pathidx] * planCont.turning_radius;
          // dy = 0;
          act = 0;
          cost = dx;
          break;
        case 3:  // RS_RIGHT
          deltat = reedsShepppath.length_[pathidx];
          //dx = Constants::r * sin(deltat);
          dx = planCont.turning_radius * sin(deltat);
          // dy = -Constants::r * (1 - cos(deltat));
          act = 1;
          //cost = reedsShepppath.length_[pathidx] * Constants::r * Constants::penaltyTurning;
          cost = reedsShepppath.length_[pathidx] * planCont.turning_radius * Constants::penaltyTurning;
          break;
        default:
          std::cout << "\033[1m\033[31m"
                    << "Warning: Receive unknown ReedsSheppPath type"
                    << "\033[0m\n";
          break;
      }
      if (cost < 0) {
        cost = -cost * Constants::penaltyReversing;
        act = act + 3;
      }
      State s = path.back();
      std::vector<std::pair<State, double>> next_path =
          generatePath(s, act, deltat, dx);
      // State next_s(s.x + dx * cos(-s.yaw) - dy * sin(-s.yaw),
      //              s.y + dx * sin(-s.yaw) + dy * cos(-s.yaw),
      //              Constants::normalizeHeadingRad(s.yaw + deltat));
      for (auto iter = next_path.begin(); iter != next_path.end(); iter++) {
        State next_s = iter->first;
        if (!stateValid(next_s))
          return false;
        else {
          gscore += iter->second;
          if (!(next_s == path.back())) {
            cameFrom.insert(std::make_pair<>(
                next_s,
                std::make_tuple<>(path.back(), act, iter->second, gscore)));
          }
          path.emplace_back(next_s);
        }
      }
    }

    m_goal = path.back();
    // auto iter = cameFrom.find(getGoal());
    // do {
    //   std::cout << " From " << std::get<0>(iter->second)
    //             << " to Node:" << iter->first
    //             << " with ACTION: " << std::get<1>(iter->second) << " cost "
    //             << std::get<2>(iter->second) << " g_score "
    //             << std::get<3>(iter->second) << std::endl;
    //   iter = cameFrom.find(std::get<0>(iter->second));
    // } while (calcIndex(std::get<0>(iter->second)) != calcIndex(state));
    // std::cout << " From " << std::get<0>(iter->second)
    //           << " to Node:" << iter->first
    //           << " with ACTION: " << std::get<1>(iter->second) << " cost "
    //           << std::get<2>(iter->second) << " g_score "
    //           << std::get<3>(iter->second) << std::endl;

    _camefrom.insert(cameFrom.begin(), cameFrom.end());
    return true;
  }

  ompl::base::DubinsStateSpace::DubinsPath findDubins(State& start, State& goal)
  {
    //ompl::base::DubinsStateSpace dubinsSpace(Constants::r);
    ompl::base::DubinsStateSpace dubinsSpace(planCont.turning_radius);
    OmplState *dubinsStart = (OmplState *)dubinsSpace.allocState();
    OmplState *dubinsEnd = (OmplState *)dubinsSpace.allocState();
    dubinsStart->setXY(start.x, start.y);
    dubinsStart->setYaw(-start.yaw);
    dubinsEnd->setXY(goal.x, goal.y);
    dubinsEnd->setYaw(-goal.yaw);
    ompl::base::DubinsStateSpace::DubinsPath dubinsPath =
        dubinsSpace.dubins(dubinsStart, dubinsEnd);

    for (auto pathidx = 0; pathidx < 3; pathidx++) {
      switch (dubinsPath.type_[pathidx]) {
        case 0:  // DUBINS_LEFT
          std::cout << "Left" << std::endl;
          break;
        case 1:  // DUBINS_STRAIGHT
          std::cout << "Straight" << std::endl;
          break;
        case 2:  // DUBINS_RIGHT
          std::cout << "Right" << std::endl;
          break;
        default:
          std::cout << "\033[1m\033[31m"
                    << "Warning: Receive unknown DubinsPath type"
                    << "\033[0m\n";
          break;
      }
      std::cout << fabs(dubinsPath.length_[pathidx]) << std::endl;
    }
    
    return dubinsPath;
  }

  bool isSolutionWithoutReverse(
      const State &state, double gscore,
      std::unordered_map<State, std::tuple<State, Action, double, double>,
                         std::hash<State>> &_camefrom) {
    double goal_distance =
        sqrt(pow(state.x - getGoal().x, 2) + pow(state.y - getGoal().y, 2));
    if (goal_distance > 10 * (Constants::LB + planCont.LF)) return false;
    //ompl::base::DubinsStateSpace dubinsSpace(Constants::r);
    ompl::base::DubinsStateSpace dubinsSpace(planCont.turning_radius);
    OmplState *dubinsStart = (OmplState *)dubinsSpace.allocState();
    OmplState *dubinsEnd = (OmplState *)dubinsSpace.allocState();
    dubinsStart->setXY(state.x, state.y);
    dubinsStart->setYaw(-state.yaw);
    dubinsEnd->setXY(getGoal().x, getGoal().y);
    dubinsEnd->setYaw(-getGoal().yaw);
    ompl::base::DubinsStateSpace::DubinsPath dubinsPath =
        dubinsSpace.dubins(dubinsStart, dubinsEnd);
    dubinsSpace.freeState(dubinsStart);
    dubinsSpace.freeState(dubinsEnd);

    std::vector<State> path;
    std::unordered_map<State, std::tuple<State, Action, double, double>,
                       std::hash<State>>
        cameFrom;
    cameFrom.clear();
    path.emplace_back(state);
    for (auto pathidx = 0; pathidx < 3; pathidx++) {
      if (fabs(dubinsPath.length_[pathidx]) < 1e-6) continue;
      double deltat = 0, dx = 0, act = 0, cost = 0;
      switch (dubinsPath.type_[pathidx]) {
        case 0:  // DUBINS_LEFT
          deltat = -dubinsPath.length_[pathidx];
          //dx = Constants::r * sin(-deltat);
          dx = planCont.turning_radius * sin(-deltat);
          // dy = Constants::r * (1 - cos(-deltat));
          act = 2;
          //cost = dubinsPath.length_[pathidx] * Constants::r * Constants::penaltyTurning;
          cost = dubinsPath.length_[pathidx] * planCont.turning_radius * Constants::penaltyTurning;
          break;
        case 1:  // DUBINS_STRAIGHT
          deltat = 0;
          //dx = dubinsPath.length_[pathidx] * Constants::r;
          dx = dubinsPath.length_[pathidx] * planCont.turning_radius;
          // dy = 0;
          act = 0;
          cost = dx;
          break;
        case 2:  // DUBINS_RIGHT
          deltat = dubinsPath.length_[pathidx];
         //dx = Constants::r * sin(deltat);
         dx = planCont.turning_radius * sin(deltat);
          // dy = -Constants::r * (1 - cos(deltat));
          act = 1;
          //cost = dubinsPath.length_[pathidx] * Constants::r * Constants::penaltyTurning;
          cost = dubinsPath.length_[pathidx] * planCont.turning_radius * Constants::penaltyTurning;
          break;
        default:
          std::cout << "\033[1m\033[31m"
                    << "Warning: Receive unknown DubinsPath type"
                    << "\033[0m\n";
          break;
      }


      State s = path.back();

      /*
      std::vector<std::pair<State, double>> next_path;
      if (generatePath(s, act, deltat, dx, next_path)) {
        for (auto iter = next_path.begin(); iter != next_path.end(); iter++) {
          State next_s = iter->first;
          gscore += iter->second;
          if (!(next_s == path.back())) {
            cameFrom.insert(std::make_pair<>(
                next_s,
                std::make_tuple<>(path.back(), act, iter->second, gscore)));
          }
          path.emplace_back(next_s);
        }
        
      } else {
        return false;
      }
      */

     std::vector<std::pair<State, double>> next_path =
          generatePath(s, act, deltat, dx);
      // State next_s(s.x + dx * cos(-s.yaw) - dy * sin(-s.yaw),
      //              s.y + dx * sin(-s.yaw) + dy * cos(-s.yaw),
      //              Constants::normalizeHeadingRad(s.yaw + deltat));
      for (auto iter = next_path.begin(); iter != next_path.end(); iter++) {
        State next_s = iter->first;
        if (!stateValid(next_s))
          return false;
        else {
          gscore += iter->second;
          if (!(next_s == path.back())) {
            cameFrom.insert(std::make_pair<>(
                next_s,
                std::make_tuple<>(path.back(), act, iter->second, gscore)));
          }
          path.emplace_back(next_s);
        }
      }
    }

    m_goal = path.back();
    _camefrom.insert(cameFrom.begin(), cameFrom.end());
    return true;
  }

/*
  void getNeighbors(const State& s, Action action,
                    std::vector<Neighbor<State, Action, double>>& neighbors) {
    neighbors.clear();
    double g = Constants::dx[0];
    //for (Action act = 0; act < 6; act++) {  // has 6 directions for Reeds-Shepp
    for (Action act = 0; act < (Constants::allow_reverse ? 6 : 3); act++) {  // has 6 directions for Reeds-Shepp, 3 for Dubins
      double xSucc, ySucc, yawSucc;
      double g = Constants::dx[0];
      xSucc = s.x + Constants::dx[act] * cos(-s.yaw) -
              Constants::dy[act] * sin(-s.yaw);
      ySucc = s.y + Constants::dx[act] * sin(-s.yaw) +
              Constants::dy[act] * cos(-s.yaw);
      yawSucc = Constants::normalizeHeadingRad(s.yaw + Constants::dyaw[act]);
      if (act != action) {  // penalize turning
        g = g * Constants::penaltyTurning;
        if (act >= 3)  // penalize change of direction
          g = g * Constants::penaltyCOD;
      }
      if (act > 3) {  // backwards
        g = g * Constants::penaltyReversing;
      }
      State tempState(xSucc, ySucc, yawSucc, s.time+1);
      if (stateValid(tempState)) {
        neighbors.emplace_back(
            Neighbor<State, Action, double>(tempState, act, g));
      }
    }
     // wait
    g = Constants::dx[0];
    State tempState(s.x, s.y, s.yaw, s.time + 1);
    if (stateValid(tempState)) {
      neighbors.emplace_back(Neighbor<State, Action, double>(tempState, 6, g));
    }
  }
  */

  void getNeighbors(const State& s, Action action,
                    std::vector<Neighbor<State, Action, double>>& neighbors) {
    neighbors.clear();
    double g = planCont.dx[0];
    //for (Action act = 0; act < 6; act++) {  // has 6 directions for Reeds-Shepp
    for (Action act = 0; act < (planCont.allow_reverse ? 6 : 3); act++) {  // has 6 directions for Reeds-Shepp, 3 for Dubins
      double xSucc, ySucc, yawSucc;
      double g = planCont.dx[0];
      xSucc = s.x + planCont.dx[act] * cos(-s.yaw) -
              planCont.dy[act] * sin(-s.yaw);
      ySucc = s.y + planCont.dx[act] * sin(-s.yaw) +
              planCont.dy[act] * cos(-s.yaw);
      yawSucc = Constants::normalizeHeadingRad(s.yaw + planCont.dyaw[act]);
      if (act != action) {  // penalize turning
        g = g * Constants::penaltyTurning;
        if (act >= 3)  // penalize change of direction
          g = g * Constants::penaltyCOD;
      }
      if (act > 3) {  // backwards
        g = g * Constants::penaltyReversing;
      }
      State tempState(xSucc, ySucc, yawSucc, s.time+1);
      if (stateValid(tempState)) {
        neighbors.emplace_back(
            Neighbor<State, Action, double>(tempState, act, g));
      }
    }
     // wait
    g = planCont.dx[0];
    State tempState(s.x, s.y, s.yaw, s.time + 1);
    if (stateValid(tempState)) {
      neighbors.emplace_back(Neighbor<State, Action, double>(tempState, 6, g));
    }
  }

  void onExpandNode(const State& s, int /*fScore*/, int /*gScore*/) {
    Ecount++;
    // std::cout << "Expand " << Ecount << " new Node:" << s << std::endl;
  }

  void onDiscover(const State& s, double fScore, double gScore) {
    Dcount++;
    // std::cout << "Discover " << Dcount << "  Node:" << s << " f:" << fScore
    //           << " g:" << gScore << std::endl;
  }

 public:
  State getGoal() { return m_goal; }
  int Ecount = 0;
  int Dcount = 0;

  void add_obs(State obs_in)
  {
    m_obstacles.insert(obs_in);
    updateCostmap();
  }

  std::vector<State> takeout_start_collision(const State& s)
  {
    double x_ind = s.x / Constants::mapResolution;
    double y_ind = s.y / Constants::mapResolution;

    std::vector<State> took_out(0);

    Eigen::Matrix2f rot;
    rot << cos(-s.yaw), -sin(-s.yaw), sin(-s.yaw), cos(-s.yaw);
    for (auto it = m_obstacles.begin(); it != m_obstacles.end(); ) {
        Eigen::Matrix<float, 1, 2> obs;
        obs << it->x - s.x, it->y - s.y;
        auto rotated_obs = obs * rot;
        if (rotated_obs(0) > -Constants::LB - Constants::obsRadius &&
            rotated_obs(0) < planCont.LF + Constants::obsRadius &&
            rotated_obs(1) > -Constants::carWidth / 2.0 - Constants::obsRadius &&
            rotated_obs(1) < Constants::carWidth / 2.0 + Constants::obsRadius)
        {
            took_out.push_back(*it);
            it = m_obstacles.erase(it); // Remove the element and get the iterator to the next element
        } else {
            ++it; // Move to the next element
        }
    }
    return took_out;
  }

  void remove_obs(const State& s)
  {
    for (auto it = m_obstacles.begin(); it != m_obstacles.end(); it++) {
      if(it->x == s.x && it->y == s.y)
      {
        it = m_obstacles.erase(it);
        break;
      }
    }
  }


  StateValiditySet stateValid(const State& s, float car_width = Constants::carWidth, float obs_rad = Constants::obsRadius,
                                  float LB = Constants::LB) {
    //dynamic obstacles
    auto it = dynamic_obs.equal_range(s.time);
    for (auto itr = it.first; itr != it.second; ++itr) {
      if (s.agentCollision(itr->second,planCont.LF,Constants::carWidth)) return StateValiditySet(false, StateValidity::collision);
    }
    auto itlow = dynamic_obs.lower_bound(-s.time);
    auto itup = dynamic_obs.upper_bound(-1);
    for (auto it = itlow; it != itup; ++it)
      if (s.agentCollision(it->second,planCont.LF,Constants::carWidth)) return StateValiditySet(false, StateValidity::collision);;


    Eigen::Matrix2f rot;
    rot << cos(-s.yaw), -sin(-s.yaw), sin(-s.yaw), cos(-s.yaw);
    for (auto it = m_obstacles.begin(); it != m_obstacles.end(); it++) {
      Eigen::Matrix<float, 1, 2> obs;
      obs << it->x - s.x, it->y - s.y;
      auto rotated_obs = obs * rot;
      if (rotated_obs(0) > -LB - obs_rad &&
          rotated_obs(0) < planCont.LF + obs_rad &&
          rotated_obs(1) > -car_width / 2.0 - obs_rad &&
          rotated_obs(1) < car_width / 2.0 + obs_rad)
        {
          //std::cout << "x: " << obs(0) << " y: " << obs(1) << std::endl;
          //std::cout << "x: " << rotated_obs(0) << " y: " << rotated_obs(1) << std::endl;
          return StateValiditySet(false, StateValidity::collision);;
        }
    }

    // boundary
    double x_ind = s.x / Constants::mapResolution;
    double y_ind = s.y / Constants::mapResolution;
    if (x_ind < 0 || x_ind >= m_dimx || y_ind < 0 || y_ind >= m_dimy)
      return StateValiditySet(false, StateValidity::out_of_boundary);;

    return StateValiditySet(true, StateValidity::valid);
    // Eigen::Matrix2f rot;
    // double yaw = M_PI / 2;
    // rot << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
    // Eigen::Matrix<float, 1, 2> temp;
    // temp << 1, 2;
    // auto ro = temp * rot;
    // std::cout << ro(0) << ro(1) << std::endl;
  }

  std::unordered_set<State> get_obs()
  {
    return m_obstacles;
  }

 private:
  void updateCostmap() {
    boost::heap::fibonacci_heap<std::pair<State, double>,
                                boost::heap::compare<compare_node>>
        heap;
    heap.clear();

    std::set<std::pair<int, int>> temp_obs_set;
    for (auto it = m_obstacles.begin(); it != m_obstacles.end(); it++) {
      temp_obs_set.insert(
          std::make_pair(static_cast<int>(it->x / Constants::mapResolution),
                         static_cast<int>(it->y / Constants::mapResolution)));
    }

    int goal_x = static_cast<int>(m_goal.x / Constants::mapResolution);
    int goal_y = static_cast<int>(m_goal.y / Constants::mapResolution);
    heap.push(std::make_pair(State(goal_x, goal_y, 0), 0));

    while (!heap.empty()) {
      std::pair<State, double> node = heap.top();
      heap.pop();

      int x = node.first.x;
      int y = node.first.y;
      for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++) {
          if (dx == 0 && dy == 0) continue;
          int new_x = x + dx;
          int new_y = y + dy;
          if (new_x == goal_x && new_y == goal_y) continue;
          if (new_x >= 0 && new_x < m_dimx && new_y >= 0 && new_y < m_dimy &&
              holonomic_cost_map[new_x][new_y] == 0 &&
              temp_obs_set.find(std::make_pair(new_x, new_y)) ==
                  temp_obs_set.end()) {
            holonomic_cost_map[new_x][new_y] =
                holonomic_cost_map[x][y] +
                sqrt(pow(dx * Constants::mapResolution, 2) +
                     pow(dy * Constants::mapResolution, 2));
            heap.push(std::make_pair(State(new_x, new_y, 0),
                                     holonomic_cost_map[new_x][new_y]));
          }
        }
    }

    // for (size_t i = 0; i < m_dimx; i++) {
    //   for (size_t j = 0; j < m_dimy; j++)
    //     std::cout << holonomic_cost_map[i][j] << "\t";
    //   std::cout << std::endl;
    // }
  }

/*
  std::vector<std::pair<State, double>> generatePath(State startState, int act,
                                                     double deltaSteer,
                                                     double deltaLength) {
    std::vector<std::pair<State, double>> result;
    double xSucc, ySucc, yawSucc, dx, dy, dyaw, ratio;
    result.emplace_back(std::make_pair<>(startState, 0));
    if (act == 0 || act == 3) {
      for (size_t i = 0; i < (size_t)(deltaLength / Constants::dx[act]); i++) {
        State s = result.back().first;
        xSucc = s.x + Constants::dx[act] * cos(-s.yaw) -
                Constants::dy[act] * sin(-s.yaw);
        ySucc = s.y + Constants::dx[act] * sin(-s.yaw) +
                Constants::dy[act] * cos(-s.yaw);
        yawSucc = Constants::normalizeHeadingRad(s.yaw + Constants::dyaw[act]);
        result.emplace_back(
            std::make_pair<>(State(xSucc, ySucc, yawSucc,s.time+1), Constants::dx[0]));
      }
      ratio =
          (deltaLength - static_cast<int>(deltaLength / Constants::dx[act]) *
                             Constants::dx[act]) /
          Constants::dx[act];
      dyaw = 0;
      dx = ratio * Constants::dx[act];
      dy = 0;
    } else {
      for (size_t i = 0; i < (size_t)(deltaSteer / Constants::dyaw[act]); i++) {
        State s = result.back().first;
        xSucc = s.x + Constants::dx[act] * cos(-s.yaw) -
                Constants::dy[act] * sin(-s.yaw);
        ySucc = s.y + Constants::dx[act] * sin(-s.yaw) +
                Constants::dy[act] * cos(-s.yaw);
        yawSucc = Constants::normalizeHeadingRad(s.yaw + Constants::dyaw[act]);
        result.emplace_back(
            std::make_pair<>(State(xSucc, ySucc, yawSucc,s.time+1),
                             Constants::dx[0] * Constants::penaltyTurning));
      }
      ratio =
          (deltaSteer - static_cast<int>(deltaSteer / Constants::dyaw[act]) *
                            Constants::dyaw[act]) /
          Constants::dyaw[act];
      dyaw = ratio * Constants::dyaw[act];
      dx = Constants::r * sin(dyaw);
      dy = -Constants::r * (1 - cos(dyaw));
      if (act == 2 || act == 5) {
        dx = -dx;
        dy = -dy;
      }
    }
    State s = result.back().first;
    xSucc = s.x + dx * cos(-s.yaw) - dy * sin(-s.yaw);
    ySucc = s.y + dx * sin(-s.yaw) + dy * cos(-s.yaw);
    yawSucc = Constants::normalizeHeadingRad(s.yaw + dyaw);
    result.emplace_back(std::make_pair<>(State(xSucc, ySucc, yawSucc,s.time+1),
                                         ratio * Constants::dx[0]));
    // std::cout << "Have generate " << result.size() << " path segments:\n\t";
    // for (auto iter = result.begin(); iter != result.end(); iter++)
    //   std::cout << iter->first << ":" << iter->second << "->";
    // std::cout << std::endl;

    return result;
  }
  */

   std::vector<std::pair<State, double>> generatePath(State startState, int act,
                                                     double deltaSteer,
                                                     double deltaLength) {
      std::vector<std::pair<State, double>> result;
      double xSucc, ySucc, yawSucc, dx, dy, dyaw, ratio;
      result.emplace_back(std::make_pair<>(startState, 0));
      if (act == 0 || act == 3) {
        for (size_t i = 0; i < (size_t)(deltaLength / planCont.dx[act]); i++) {
          State s = result.back().first;
          xSucc = s.x + planCont.dx[act] * cos(-s.yaw) -
                  planCont.dy[act] * sin(-s.yaw);
          ySucc = s.y + planCont.dx[act] * sin(-s.yaw) +
                  planCont.dy[act] * cos(-s.yaw);
          yawSucc = Constants::normalizeHeadingRad(s.yaw + planCont.dyaw[act]);
          result.emplace_back(
              std::make_pair<>(State(xSucc, ySucc, yawSucc,s.time+1), planCont.dx[0]));
        }
        ratio =
            (deltaLength - static_cast<int>(deltaLength / planCont.dx[act]) *
                              planCont.dx[act]) /
            planCont.dx[act];
        dyaw = 0;
        dx = ratio * planCont.dx[act];
        dy = 0;
      } else {
        for (size_t i = 0; i < (size_t)(deltaSteer / planCont.dyaw[act]); i++) {
          State s = result.back().first;
          xSucc = s.x + planCont.dx[act] * cos(-s.yaw) -
                  planCont.dy[act] * sin(-s.yaw);
          ySucc = s.y + planCont.dx[act] * sin(-s.yaw) +
                  planCont.dy[act] * cos(-s.yaw);
          yawSucc = Constants::normalizeHeadingRad(s.yaw + planCont.dyaw[act]);
          result.emplace_back(
              std::make_pair<>(State(xSucc, ySucc, yawSucc,s.time+1),
                              planCont.dx[0] * Constants::penaltyTurning));
        }
        ratio =
            (deltaSteer - static_cast<int>(deltaSteer / planCont.dyaw[act]) *
                              planCont.dyaw[act]) /
            planCont.dyaw[act];
        dyaw = ratio * planCont.dyaw[act];
        dx = planCont.turning_radius * sin(dyaw);
        dy = -planCont.turning_radius * (1 - cos(dyaw));
        if (act == 2 || act == 5) {
          dx = -dx;
          dy = -dy;
        }
      }
      State s = result.back().first;
      xSucc = s.x + dx * cos(-s.yaw) - dy * sin(-s.yaw);
      ySucc = s.y + dx * sin(-s.yaw) + dy * cos(-s.yaw);
      yawSucc = Constants::normalizeHeadingRad(s.yaw + dyaw);
      result.emplace_back(std::make_pair<>(State(xSucc, ySucc, yawSucc,s.time+1),
                                          ratio * planCont.dx[0]));
      // std::cout << "Have generate " << result.size() << " path segments:\n\t";
      // for (auto iter = result.begin(); iter != result.end(); iter++)
      //   std::cout << iter->first << ":" << iter->second << "->";
      // std::cout << std::endl;

      return result;
    }

  int m_dimx;
  int m_dimy;
  std::unordered_set<State> m_obstacles;
  std::vector<std::vector<double>> holonomic_cost_map;
  State m_goal;

  PlanningContext planCont;

  std::multimap<int, State> dynamic_obs;
};

PathPlanResultPtr planHybridAstar(State start, State goal_in, Environment& env, int64_t timeout_ms = 0 ,bool print_res = false, float car_width = Constants::carWidth, float obs_rad = Constants::obsRadius);

#endif