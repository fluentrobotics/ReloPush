#ifndef PLANHYBRIDASTAR_HPP
#define PLANHYBRIDASTAR_HPP

#include <PlanningContext.hpp>

PathPlanResultPtr planHybridAstar(ReloPush::State start, ReloPush::State goal_in,
                                  Environment& env, bool allow_reverse, int64_t timeout_ms = 0,
                                  bool print_res = false, float car_width = Constants::carWidth, float obs_rad = Constants::obsRadius);

PathPlanResultPtr planHybridAstar(ReloPush::State start_in, ReloPush::State goal_in, PlanningContext& ctx, bool allow_reverse);

#endif // PLANHYBRIDASTAR_HPP
