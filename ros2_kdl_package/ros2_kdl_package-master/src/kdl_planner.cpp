#include "kdl_planner.h"
// #include <cmath>


KDLPlanner::KDLPlanner(){}

KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
{
    velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
}

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

void KDLPlanner::CreateTrajectoryFromFrames(std::vector<KDL::Frame> &_frames,
                                            double _radius, double _eqRadius
                                            )
{
    path_ = new KDL::Path_RoundedComposite(_radius,_eqRadius,new KDL::RotationalInterpolation_SingleAxis());

    for (unsigned int i = 0; i < _frames.size(); i++)
    {
        path_->Add(_frames[i]);
    }
    path_->Finish();

    velpref_->SetProfile(0,path_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_, velpref_);
}

void KDLPlanner::createCircPath(KDL::Frame &_F_start,
                                KDL::Vector &_V_centre,
                                KDL::Vector& _V_base_p,
                                KDL::Rotation& _R_base_end,
                                double alpha,
                                double eqradius
                                )
 
   // Traiettoria circolare sul piano y - z
    double xi = trajInit_.x();  // posizione x rimane costante
    double yi = trajInit_.y();  // posizione iniziale su y
    double zi = trajInit_.z();  // posizione iniziale su z
    double r = trajRadius_;     

    double theta = 2 * M_PI * s;  

    traj.pos.x() = xi;
    traj.pos.y() = yi - r * cos(theta);  
    traj.pos.z() = zi - r * sin(theta);  

    // velocità e accelerazione nel moto circolare
    traj.vel.x() = 0.0;
    traj.vel.y() = -r * 2 * M_PI * v * sin(theta);  // Velocità su y
    traj.vel.z() = -r * 2 * M_PI * v * cos(theta);  // Velocità su z

    traj.acc.x() = 0.0;
    traj.acc.y() = -r * (2 * M_PI * a * sin(theta) + (2 * M_PI * v * v) * cos(theta));  // Accelerazione su y
    traj.acc.z() = -r * (2 * M_PI * a * cos(theta) - (2 * M_PI * v * v) * sin(theta));  // Accelerazione su z

    return traj;
}

{
    KDL::RotationalInterpolation_SingleAxis* otraj;
    otraj = new KDL::RotationalInterpolation_SingleAxis();
    otraj->SetStartEnd(_F_start.M,_R_base_end);
    path_circle_ = new KDL::Path_Circle(_F_start,
                                        _V_centre,
                                        _V_base_p,
                                        _R_base_end,
                                        alpha,
                                        otraj,
                                        eqradius);
    velpref_->SetProfile(0,path_circle_->PathLength());
    traject_ = new KDL::Trajectory_Segment(path_circle_, velpref_);
}

KDL::Trajectory* KDLPlanner::getTrajectory()
{
	return traject_;
}

 // Implementazione della traiettoria cubica polinomiale
    void KDLPlanner::cubic_poly_trajectory(double &t, double T, double s0, double sf, double v0, double vf, double a0, double af, double &s, double &v, double &a) {
    double a3, a2, a1, a0;

    double delta_s = sf - s0;
    a3 = (2 * (v0 + vf) - 3 * delta_s) / (T * T * T);
    a2 = (3 * delta_s - 2 * (v0 + vf)) / (T * T);
    a1 = v0;
    a0 = s0;

    // Calcolo posizione, velocità ed accelerazioneal tempo t
    s = a3 * t * t * t + a2 * t * t + a1 * t + a0; // Posizione
    v = 3 * a3 * t * t + 2 * a2 * t + a1;          // Velocità
    a = 6 * a3 * t + 2 * a2;                        // Accelerazione

    return true;
}            

trajectory_point KDLPlanner::compute_trajectory(double time){
  {
    trajectory_point traj;    
    trajectory_point traj;

    
    double s0 = trajInit_.x(), sf = trajEnd_.x(); 
    double v0 = 0.0, vf = 0.0; 
    double a0 = 0.0, af = 0.0; 
    double T = trajDuration_;  

     
    double s, v, a;
    if (cubic_poly_trajectory(time, T, s0, sf, v0, vf, a0, af, s, v, a))
    {
        
    {
        ROS_ERROR("Cubic polynomial trajectory generation failed.");        
  
      return traj;
  }
  
  }

}

  Eigen::Vector3d ddot_traj_c = -1.0/(std::pow(accDuration_,2)-trajDuration_*accDuration_)*(trajEnd_-trajInit_);

  if(time <= accDuration_)
  {
    traj.pos = trajInit_ + 0.5*ddot_traj_c*std::pow(time,2);
    traj.vel = ddot_traj_c*time;
    traj.acc = ddot_traj_c;
  }
  else if(time <= trajDuration_-accDuration_)
  {
    traj.pos = trajInit_ + ddot_traj_c*accDuration_*(time-accDuration_/2);
    traj.vel = ddot_traj_c*accDuration_;
    traj.acc = Eigen::Vector3d::Zero();
  }
  else
  {
    traj.pos = trajEnd_ - 0.5*ddot_traj_c*std::pow(trajDuration_-time,2);
    traj.vel = ddot_traj_c*(trajDuration_-time);
    traj.acc = -ddot_traj_c;
  }

  return traj;

