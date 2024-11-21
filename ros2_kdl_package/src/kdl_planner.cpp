#include "kdl_planner.h"


KDLPlanner::KDLPlanner(){}

// KDLPlanner::KDLPlanner(double _maxVel, double _maxAcc)
// {
//     velpref_ = new KDL::VelocityProfile_Trap(_maxVel,_maxAcc);
// }

KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, Eigen::Vector3d _trajEnd)
{
   //aggiungi i coefficienti per determinare la cubica
    trajDuration_ = _trajDuration;
    trajInit_ = _trajInit;
    trajEnd_ = _trajEnd;

    a0_=0;
    a1_=0;
    a2_=3/std::pow(trajDuration_,2);//We want that the velocity at the start 
    a3_=-2/std::pow(trajDuration_,3);//and at the end is zero
}

// KDLPlanner::KDLPlanner(double _trajDuration, double _maxAcc)
// {
//     //We want that the velocity at the start and at the end is zero
//     trajDuration_ = _trajDuration;
//     maxAcc_ = _maxAcc;

//     double a0_=0;
//     double a1_=0;
//     double a2_=3/std::pow(trajDuration_,2);
//     double a3_=-2/std::pow(trajDuration_,3);

   
// }

KDLPlanner::KDLPlanner(double _trajDuration, Eigen::Vector3d _trajInit, double _trajRadius,KDL::Frame _EEInitFrame)
{

   
    trajDuration_ = _trajDuration;
   
    trajRadius_=_trajRadius;
    trajInit_=_trajInit;
    EEInitFrame_=_EEInitFrame;

    a0_=0;
    a1_=0;
    a2_=3/std::pow(trajDuration_,2);//We want that the velocity at the start 
    a3_=-2/std::pow(trajDuration_,3);//and at the end is zero


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
  double maxAcc_=1.0/(-(std::pow(tc,2))+trajDuration_*tc);  
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





trajectory_point KDLPlanner::compute_trajectory(double time,std::string traj_type )
{
    double s=0;
    double sdot=0;
    double sdotdot=0;
   trajectory_point traj;
  if(traj_type=="linear_trajectory")
  {
      cubic_polynomial(time,s,sdot,sdotdot);        
        traj.pos = trajInit_ + s*(trajEnd_-trajInit_);
        traj.vel = sdot*(trajEnd_-trajInit_);
        traj.acc = sdotdot*(trajEnd_-trajInit_);
        //std::cout<<"ascissa: "<<s<<std::endl;       
  } 
  else if (traj_type=="circular_trajectory")
  {
      cubic_polynomial(time,s,sdot,sdotdot);   
      traj.pos(0)=EEInitFrame_.p.x();
      traj.pos(1)=EEInitFrame_.p.y()+trajRadius_-trajRadius_*cos(2*M_PI*s);
      traj.pos(2)=EEInitFrame_.p.z()-trajRadius_*sin(2*M_PI*s);

      traj.vel(0)=0;
      traj.vel(1)=2*M_PI*trajRadius_*sdot*sin(2*M_PI*s);
      traj.vel(2)=-2*M_PI*trajRadius_*sdot*cos(2*M_PI*s);

      traj.acc(0)=0;
      traj.acc(1)=2*M_PI*trajRadius_*(sdotdot*sin(2*M_PI*s)+sdot*sdot*cos(2*M_PI*s)*2*M_PI);
      traj.acc(2)=-2*M_PI*trajRadius_*(sdotdot*cos(2*M_PI*s)-2*M_PI*sdot*sdot*sin(2*M_PI*s));

      //for (unsigned int i=0;i<3;i++){std::cout<<"traiettoria"<<traj.pos(i)<<std::endl;}
      //std::cout<<"ascissa: "<<s<<std::endl;
  }
  else
  {
     std::cout<<"ERRORE SCRITTURA STRINGA"<<std::endl;
  }
  return traj;
}

trajectory_point KDLPlanner::compute_trajectoryTrapezoidal(double time, double tc, std::string traj_type )
{
    double s=0;
    double sdot=0;
    double sdotdot=0;
  trajectory_point traj;
  if(traj_type=="linear_trajectory")
  {
      compute_trapezoidal_velocity_point(time,tc,s,sdot,sdotdot);
        traj.pos = trajInit_ + s*(trajEnd_-trajInit_);
        traj.vel = sdot*(trajEnd_-trajInit_);
        traj.acc = sdotdot*(trajEnd_-trajInit_);
        //for (unsigned int i=0;i<3;i++){std::cout<<"traiettoria vel"<<traj.vel(i)<<std::endl;}
      }
  else if (traj_type=="circular_trajectory")
  {
      compute_trapezoidal_velocity_point(time,tc,s,sdot,sdotdot);
      traj.pos(0)=EEInitFrame_.p.x();
      traj.pos(1)=EEInitFrame_.p.y()+trajRadius_-trajRadius_*cos(2*M_PI*s);
      traj.pos(2)=EEInitFrame_.p.z()-trajRadius_*sin(2*M_PI*s);

      traj.vel(0)=0;
      traj.vel(1)=2*M_PI*trajRadius_*sdot*sin(2*M_PI*s);
      traj.vel(2)=-2*M_PI*trajRadius_*sdot*cos(2*M_PI*s);

      traj.acc(0)=0;
      traj.acc(1)=2*M_PI*trajRadius_*(sdotdot*sin(2*M_PI*s)+sdot*sdot*cos(2*M_PI*s)*2*M_PI);
      traj.acc(2)=-2*M_PI*trajRadius_*(sdotdot*cos(2*M_PI*s)-2*M_PI*sdot*sdot*sin(2*M_PI*s));
  }
  else
  {
     std::cout<<"ERRORE SCRITTURA STRINGA"<<std::endl;
  }
  return traj;
}