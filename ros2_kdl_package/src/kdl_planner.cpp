#include "kdl_planner.h"


KDLPlanner::KDLPlanner(){}

// KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
// {
//     velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
// }

KDLPlanner::KDLPlanner(double _trajDuration, double _accDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
    trajDuration_ = _trajDuration;
    accDuration_ = _accDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;
}

KDLPlanner::KDLPlanner(double _trajDuration, double _maxAcc)
{
    trajDuration_ = _trajDuration;
    maxAcc_ = _maxAcc;

    double a0_=0;
    double a1_=0;//we want that sdot(0) is equal to 0
    double a2_=3/std::pow(trajDuration_,2);
    double a3_=-2/std::pow(trajDuration_,3);

   
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

void KDLPlanner::compute_trapezoidal_velocity_point(double t, double tc,double & s,double & sdot,double & sdotdot)
{

  if(t <= tc)
  {
    s= 0.5*maxAcc_*std::pow(t,2);
    sdot=maxAcc_*t;
    sdotdot=maxAcc_;
  }
  else if(t <= trajDuration_-tc)
  {
    s =maxAcc_*tc*(t-tc/2);
    sdot=maxAcc_*tc;
    sdotdot=0;  
  }
  else
  {
    s= 1 - 0.5*maxAcc_*std::pow(trajDuration_-t,2);
    sdot=maxAcc_*(trajDuration_-t);
    sdotdot=-maxAcc_;  
  }

  
}

void KDLPlanner::cubic_polynomial(double t,double & s,double & sdot,double & sdotdot)
{
  s=a3_*std::pow(t,3)+a2_*std::pow(t,2)+a1_*t+a0_;
  sdot=3*a3_*std::pow(t,2)+2*a2_*t+a1_;
  sdotdot=6*a3_*t+2*a2_;

}





trajectory_point KDLPlanner::compute_trajectory(double time)
{
  /* trapezoidal velocity profile with accDuration_ acceleration time period and trajDuration_ total duration.
     time = current time
     trajDuration_  = final time
     accDuration_   = acceleration time
     trajInit_ = trajectory initial point
     trajEnd_  = trajectory final point */

  trajectory_point traj;

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

}
