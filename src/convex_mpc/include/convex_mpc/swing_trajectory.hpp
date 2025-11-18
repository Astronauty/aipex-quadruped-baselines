#pragma once
#include <Eigen/Dense>
#include <algorithm>
#include <string>
#include <utility>

// 1D cubic Hermite
struct Segment1D {
  double a{}, b{}, c{}, d{};
  double t0{}, t1{};
};

class CubicSpline1D {
public:
  inline void build(double t0,double y0,double t1,double y1,double t2,double y2) {
    buildClamped(t0,y0,t1,y1,t2,y2, 0.0, 0.0);
  }

  inline void buildClamped(double t0,double y0,double t1,double y1,double t2,double y2,double v0,double v2) {
    const double h0 = t1 - t0, h1 = t2 - t1;
    const double m0 = v0, m2 = v2;
    
    const double m1 =
        ( 6.0*(y2 - y1)/(h1*h1)
        + 6.0*(y1 - y0)/(h0*h0)
        - 2.0*(m0/h0 + m2/h1) )
      / ( 4.0*(1.0/h0 + 1.0/h1) );

    const double Y[3]={y0,y1,y2}, T[3]={t0,t1,t2}, M[3]={m0,m1,m2};
    for (int i=0;i<2;++i){
      const double h=T[i+1]-T[i];
      const double yi=Y[i], yj=Y[i+1], mi=M[i], mj=M[i+1];
      seg_[i].t0=T[i]; seg_[i].t1=T[i+1];
      seg_[i].a = yi;
      seg_[i].b = mi;
      seg_[i].c = 3.0*(yj-yi)/(h*h) - (2.0*mi + mj)/h;
      seg_[i].d = 2.0*(yi-yj)/(h*h*h) + (mi + mj)/(h*h);
    }
  }

  inline void evaluate(double t, double& y,double& ydot,double& yddot) const {
    const Segment1D& s = (t <= seg_[0].t1 ? seg_[0] : seg_[1]);
    const double dt = t - s.t0;
    y    = s.a + s.b*dt + s.c*dt*dt + s.d*dt*dt*dt;
    ydot = s.b + 2.0*s.c*dt + 3.0*s.d*dt*dt;
    yddot= 2.0*s.c + 6.0*s.d*dt;
  }

private:
  Segment1D seg_[2];
};

// 3D spline
class SwingLeg3DSpline {
public:
  enum class BuildPolicy { ExplicitMid, AutoMid, AutoMidClamped };

  struct BuildArgs {
    BuildPolicy     policy{BuildPolicy::AutoMid};
    double          t0{0.0}, t1{0.0}, t2{0.0};
    Eigen::Vector3d P0{0,0,0}, P1{0,0,0}, P2{0,0,0};
    double          swing_height{0.08};
    double          mid_tau{0.5};
    double          y_offset{0.0};
    Eigen::Vector3d v0{0,0,0}, v2{0,0,0};
  };

  inline void build3(double t0,const Eigen::Vector3d& P0,double t1,const Eigen::Vector3d& P1,double t2,const Eigen::Vector3d& P2) {
    sx_.buildClamped(t0,P0.x(),t1,P1.x(),t2,P2.x(), 0.0,0.0);
    sy_.buildClamped(t0,P0.y(),t1,P1.y(),t2,P2.y(), 0.0,0.0);
    sz_.buildClamped(t0,P0.z(),t1,P1.z(),t2,P2.z(), 0.0,0.0);
    t_begin_ = t0; t_end_ = t2;
  }

  inline void build3AutoMid(double t0,const Eigen::Vector3d& P0,double t2,const Eigen::Vector3d& P2,double swing_height,double mid_tau=0.5,double y_offset=0.0) {
    mid_tau = std::clamp(mid_tau, 0.0, 1.0);
    const double t1 = t0 + mid_tau*(t2 - t0);
    Eigen::Vector3d P1 = P0 + mid_tau*(P2 - P0);
    P1.y() += y_offset;
    const double z_lin_mid = P0.z() + mid_tau*(P2.z() - P0.z());
    P1.z() = z_lin_mid + swing_height;
    build3(t0,P0,t1,P1,t2,P2);
  }

  inline void build3Clamped(double t0,const Eigen::Vector3d& P0,double t1,const Eigen::Vector3d& P1,double t2,const Eigen::Vector3d& P2,const Eigen::Vector3d& v0=Eigen::Vector3d::Zero(),const Eigen::Vector3d& v2=Eigen::Vector3d::Zero()) {
    sx_.buildClamped(t0,P0.x(),t1,P1.x(),t2,P2.x(), v0.x(),v2.x());
    sy_.buildClamped(t0,P0.y(),t1,P1.y(),t2,P2.y(), v0.y(),v2.y());
    sz_.buildClamped(t0,P0.z(),t1,P1.z(),t2,P2.z(), v0.z(),v2.z());
    t_begin_ = t0; t_end_ = t2;
  }

  inline void build3ClampedAutoMid(double t0,const Eigen::Vector3d& P0,
                                   double t2,const Eigen::Vector3d& P2,
                                   double swing_height,
                                   const Eigen::Vector3d& v0=Eigen::Vector3d::Zero(),
                                   const Eigen::Vector3d& v2=Eigen::Vector3d::Zero(),
                                   double mid_tau=0.5,double y_offset=0.0) {
    mid_tau = std::clamp(mid_tau, 0.0, 1.0);
    const double t1 = t0 + mid_tau*(t2 - t0);
    Eigen::Vector3d P1 = P0 + mid_tau*(P2 - P0);
    P1.y() += y_offset;
    const double z_lin_mid = P0.z() + mid_tau*(P2.z() - P0.z());
    P1.z() = z_lin_mid + swing_height;
    build3Clamped(t0,P0,t1,P1,t2,P2, v0,v2);
  }

  inline void build(const BuildArgs& a) {
    switch (a.policy) {
      case BuildPolicy::ExplicitMid:
        build3(a.t0,a.P0,a.t1,a.P1,a.t2,a.P2); break;
      case BuildPolicy::AutoMid:
        build3AutoMid(a.t0,a.P0,a.t2,a.P2,a.swing_height,a.mid_tau,a.y_offset); break;
      case BuildPolicy::AutoMidClamped:
        build3ClampedAutoMid(a.t0,a.P0,a.t2,a.P2,a.swing_height,a.v0,a.v2,a.mid_tau,a.y_offset); break;
    }
  }

  inline void evaluate3(double t,Eigen::Vector3d& p,Eigen::Vector3d& v,Eigen::Vector3d& a) const {
    double px,py,pz, vx,vy,vz, ax,ay,az;
    sx_.evaluate(t,px,vx,ax); sy_.evaluate(t,py,vy,ay); sz_.evaluate(t,pz,vz,az);
    p = {px,py,pz}; v = {vx,vy,vz}; a = {ax,ay,az};
  }

  inline double tBegin() const { return t_begin_; }
  inline double tEnd()   const { return t_end_;   }

private:
  CubicSpline1D sx_,sy_,sz_;
  double t_begin_{0.0}, t_end_{0.0};
};

class SwingLegTrajectory {
public:
  SwingLegTrajectory(double t_start, double t_end, Eigen::Vector3d p_start, Eigen::Vector3d p_end, double swing_height)
  : t0_(t_start), t2_(t_end), height_(swing_height), p0_(p_start), p2_(p_end)
  {
    spline_.build3ClampedAutoMid(t0_, p_start, t2_, p_end, height_);
  }

  // // Build once you know P0 (foot at lift-off) and P2 (planned touchdown)
  // void buildAutoMid(const Eigen::Vector3d& P0, const Eigen::Vector3d& P2,
  //                   double mid_tau=0.5, double y_offset=0.0) {
  //   spline_.build3ClampedAutoMid(t0_, P0, t2_, P2, height_,
  //                                Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
  //                                mid_tau, y_offset);
  // }

  /**
   * @brief Evaluate the swing leg trajectory at 
   * @param t Time in seconds
   * @param p Position output (3D)
   * @param v Velocity output (3D)
   * @param a Acceleration output (3D)
   * 
   * Note: t should be in [t0, t2]
   */
  void evaluate(double t, Eigen::Vector3d& p, Eigen::Vector3d& v, Eigen::Vector3d& a) const {
    spline_.evaluate3(t, p, v, a);
  }

  double t0() const { return t0_; }
  double t2() const { return t2_; }

  inline void set_start_position(const Eigen::Vector3d& p) { p0_ = p; }
  inline void set_end_position(const Eigen::Vector3d& p) { p2_ = p; }

  inline Eigen::Vector3d get_start_position() const { return p0_; }
  inline Eigen::Vector3d get_end_position() const { return p2_; }
  inline double get_swing_height() const { return height_; }
  
  inline double get_start_time() const { return t0_; }
  inline double get_end_time() const { return t2_; }
private:
  double t0_{}, t2_{}, height_{};
  Eigen::Vector3d p0_{}, p2_{};
  SwingLeg3DSpline spline_;
};



