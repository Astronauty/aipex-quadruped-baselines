#pragma once
#include <Eigen/Dense>
#include <unordered_map>
#include <vector>
#include <string>
#include <functional>
#include <utility>
#include <limits>

#include "swing_trajectory_from_planner.hpp"

namespace swing {

struct SwingWindow {
  std::string leg;  
  double t0;        // lift-off 
  double t2;        // touch-down
};

class SwingManager {
public:
  using LegID = std::string;
  explicit SwingManager(const PlannerHooks& hooks,
                        std::initializer_list<LegID> legs = {"FR","FL","RR","RL"})
  : hooks_(hooks)
  {
    for (const auto& leg : legs) {
      Slot s{ SwingFromPlanner(hooks_), {}, false, 0.0, 0.0, -std::numeric_limits<double>::infinity() };
      s.params = default_params_;
      s.sfp.params = s.params;
      slots_.emplace(leg, std::move(s));
    }
  }
  inline void setScheduleProvider(std::function<std::vector<SwingWindow>(double)> fn) {
    schedule_fn_ = std::move(fn);
  }

  inline void setDefaultParams(const SwingParams& p) {
    default_params_ = p;
    for (auto& [_, slot] : slots_) {
      slot.params = p;          
      slot.sfp.params = slot.params; 
    }
  }

  
  inline void setLegParams(const LegID& leg, const SwingParams& p) {
    auto it = slots_.find(leg);
    if (it == slots_.end()) return;
    it->second.params = p;
    it->second.sfp.params = p;
  }

  inline SwingParams getLegParams(const LegID& leg) const {
    auto it = slots_.find(leg);
    return (it!=slots_.end()? it->second.params : default_params_);
  }

  // Manually start a leg given an explicit window. Returns true on success.
  inline bool startLeg(const LegID& leg, double t0, double t2) {
    auto it = slots_.find(leg);
    if (it == slots_.end() || !(t2 > t0)) return false;
    Slot& s = it->second;
    if (s.active && s.t0 == t0) return true; // already running this window
    s.sfp.params = s.params; // ensure sfp sees latest
    if (!s.sfp.buildForWindow(leg, t0, t2)) return false;
    s.active = true; s.t0 = t0; s.t2 = t2; s.last_started_t0 = t0;
    return true;
  }

  
  inline void deactivate(const LegID& leg) {
    auto it = slots_.find(leg); if (it==slots_.end()) return; it->second.active = false;
  }

  inline bool isActive(const LegID& leg) const {
    auto it = slots_.find(leg); return (it!=slots_.end() && it->second.active);
  }

  inline std::pair<double,double> currentWindow(const LegID& leg) const {
    auto it = slots_.find(leg); if (it==slots_.end()) return {0.0,0.0};
    return {it->second.t0, it->second.t2};
  }

  
  inline void update(double now) {    
    if (schedule_fn_) {
      const auto windows = schedule_fn_(now);
      for (const auto& w : windows) {
        auto it = slots_.find(w.leg);
        if (it == slots_.end()) continue;
        Slot& s = it->second;
        const bool should_start = (now >= w.t0) && (now < w.t2) && (!s.active) && (s.last_started_t0 != w.t0);
        if (should_start) {
          startLeg(w.leg, w.t0, w.t2);
        }
      }
    }
   
    for (auto& [leg, s] : slots_) {
      if (s.active && now > s.t2) s.active = false;
    }
  }

  inline bool evaluate(const LegID& leg, double t,
                       Eigen::Vector3d& pos,
                       Eigen::Vector3d& vel,
                       Eigen::Vector3d& acc) const {
    auto it = slots_.find(leg); if (it==slots_.end()) return false;
    const Slot& s = it->second; if (!s.active || t > s.t2) return false;
    s.sfp.evaluate(t, pos, vel, acc);
    return true;
  }

  inline void evaluateAll(double t,
                          const std::function<void(const LegID&,
                                                   const Eigen::Vector3d&,
                                                   const Eigen::Vector3d&,
                                                   const Eigen::Vector3d&)>& cb) const {
    for (const auto& [leg, s] : slots_) {
      if (!s.active || t > s.t2) continue;
      Eigen::Vector3d p,v,a; s.sfp.evaluate(t, p, v, a);
      cb(leg, p, v, a);
    }
  }

private:
  struct Slot {
    SwingFromPlanner sfp;  
    SwingParams      params; 
    bool   active{false};
    double t0{0.0}, t2{0.0};
    double last_started_t0{-std::numeric_limits<double>::infinity()};
  };

  PlannerHooks hooks_{};
  std::unordered_map<LegID, Slot> slots_;
  SwingParams default_params_{}; 
  std::function<std::vector<SwingWindow>(double)> schedule_fn_{};
};

} 
