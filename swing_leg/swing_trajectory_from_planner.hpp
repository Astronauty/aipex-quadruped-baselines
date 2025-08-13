#pragma once
#include <Eigen/Dense>
#include <functional>
#include <optional>
#include <string>
#include <utility>
#include "swing_trajectory.hpp"

namespace swing {

struct PlannerHooks {
  
  std::function<std::pair<double,double>(const std::string& leg)> getSwingWindow;
  std::function<Eigen::Vector3d(const std::string& leg, double t0)> getLiftOffWorld;  
  std::function<Eigen::Vector3d(const std::string& leg, double t2)> getTouchDownWorld;
};


struct SwingParams {
  double swing_height{0.07};     
  double mid_tau{0.50};          
  double y_offset{0.0};        
  Eigen::Vector3d v0{0,0,0};     
  Eigen::Vector3d v2{0,0,0};    
};


class SwingFromPlanner {
public:
  explicit SwingFromPlanner(PlannerHooks hooks) : hooks_(std::move(hooks)) {}

  
  SwingParams params;


  inline bool build(const std::string& leg) {
    if (!hooks_.getSwingWindow || !hooks_.getLiftOffWorld || !hooks_.getTouchDownWorld)
      return false;

    auto [t0, t2] = hooks_.getSwingWindow(leg);
    if (!(t2 > t0)) return false; 

    const Eigen::Vector3d P0 = hooks_.getLiftOffWorld(leg, t0);
    const Eigen::Vector3d P2 = hooks_.getTouchDownWorld(leg, t2);

    last_leg_ = leg; t_begin_ = t0; t_end_ = t2;

    SwingLeg3DSpline::BuildArgs a;
    a.policy       = SwingLeg3DSpline::BuildPolicy::AutoMidClamped;
    a.t0           = t0; a.t2 = t2;
    a.P0           = P0; a.P2 = P2;
    a.swing_height = params.swing_height;
    a.mid_tau      = params.mid_tau;
    a.y_offset     = params.y_offset;
    a.v0           = params.v0;
    a.v2           = params.v2;

    traj_.build(a);
    built_ = true;
    return true;
  }

  
  inline bool buildForWindow(const std::string& leg, double t0, double t2) {
    if (!hooks_.getLiftOffWorld || !hooks_.getTouchDownWorld) return false;
    if (!(t2 > t0)) return false;

    const Eigen::Vector3d P0 = hooks_.getLiftOffWorld(leg, t0);
    const Eigen::Vector3d P2 = hooks_.getTouchDownWorld(leg, t2);

    last_leg_ = leg; t_begin_ = t0; t_end_ = t2;

    SwingLeg3DSpline::BuildArgs a;
    a.policy       = SwingLeg3DSpline::BuildPolicy::AutoMidClamped;
    a.t0           = t0; a.t2 = t2;
    a.P0           = P0; a.P2 = P2;
    a.swing_height = params.swing_height;
    a.mid_tau      = params.mid_tau;
    a.y_offset     = params.y_offset;
    a.v0           = params.v0;
    a.v2           = params.v2;

    traj_.build(a);
    built_ = true;
    return true;
  }

  inline void evaluate(double t, Eigen::Vector3d& pos, Eigen::Vector3d& vel, Eigen::Vector3d& acc) const {
    traj_.evaluate3(t, pos, vel, acc);
  }

  inline const SwingLeg3DSpline& traj() const { return traj_; }
  inline double tBegin() const { return t_begin_; }
  inline double tEnd()   const { return t_end_; }
  inline const std::string& leg() const { return last_leg_; }
  inline bool built() const { return built_; }

private:
  PlannerHooks hooks_{};
  SwingLeg3DSpline traj_{};
  bool built_{false};
  double t_begin_{0.0}, t_end_{0.0};
  std::string last_leg_{};
};

} 
