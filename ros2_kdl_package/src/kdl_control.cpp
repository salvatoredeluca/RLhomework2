#include "kdl_control.h"

KDLController::KDLController()
{
  robot_=nullptr;
}

void KDLController::setController(KDLRobot &_robot)
{
    robot_=&_robot;
}

KDLController::KDLController(KDLRobot &_robot)
{
    robot_ = &_robot;
}

Eigen::VectorXd KDLController::idCntr(KDL::JntArray &_qd,
                                      KDL::JntArray &_dqd,
                                      KDL::JntArray &_ddqd,
                                      double _Kp, double _Kd)
{
    // read current state
    Eigen::VectorXd q = robot_->getJntValues();
    Eigen::VectorXd dq = robot_->getJntVelocities();

    // calculate errors
    Eigen::VectorXd e = _qd.data - q;
    Eigen::VectorXd de = _dqd.data - dq;

    Eigen::VectorXd ddqd = _ddqd.data;
    
    return robot_->getJsim() * (ddqd + _Kd*de + _Kp*e)
            + robot_->getCoriolis(); //+ robot_->getGravity() /*friction compensation?*/;
}

Eigen::VectorXd KDLController::idCntr(KDL::Frame &_desPos,
                                      KDL::Twist &_desVel,
                                      KDL::Twist &_desAcc,
                                      double _Kpp, double _Kpo,
                                      double _Kdp, double _Kdo)
{
    Vector6d xtilde;
    Vector6d xtildedot;

    Eigen::Matrix<double,6,6> Kp;
    Eigen::Matrix<double,6,6> Kd;

    Kp.block(0,0,3,3) = _Kpp*Eigen::Matrix3d::Identity();
    Kp.block(3,3,3,3) = _Kpo*Eigen::Matrix3d::Identity();
    Kd.block(0,0,3,3) = _Kdp*Eigen::Matrix3d::Identity();
    Kd.block(3,3,3,3) = _Kdo*Eigen::Matrix3d::Identity();
    
    computeErrors(_desPos,robot_->getEEFrame(),_desVel,robot_->getEEVelocity(),xtilde,xtildedot);

    for (unsigned int i=0;i<3;i++){xtilde(i)=_Kpp*xtilde(i);xtildedot(i)=_Kdp*xtildedot(i);}
    for (unsigned int i=3;i<6;i++){xtilde(i)=_Kpo*xtilde(i);xtildedot(i)=_Kdo*xtildedot(i);}
    
    Eigen::Matrix<double,7,1> y;
    y<<pseudoinverse(robot_->getEEJacobian().data)*(toEigen(_desAcc)+xtilde+xtildedot-robot_->getEEJacDot()*robot_->getJntVelocities());
    
    return  robot_->getJsim()*y + robot_->getCoriolis(); //+ robot_->getGravity();
}

Eigen::VectorXd KDLController::PDplusGravity(KDL::JntArray &_qd,double _Kp,double _Kd)
{
    Eigen::VectorXd e(7);
    
    for(unsigned int i=0;i<robot_->getNrJnts();i++)
    {
        e(i)=_qd.data[i]-robot_->getJntValues()[i];
    }
    return _Kp*e-_Kd*robot_->getJntVelocities();//+robot->getGravity();
}

