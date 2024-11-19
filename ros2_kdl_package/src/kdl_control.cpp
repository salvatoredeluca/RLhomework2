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
    Eigen::Matrix<double,6,1> xtilde;
    Eigen::Matrix<double,6,1> xtildedot;

    computeErrors(_desPos,robot_->getEEFrame(),_desVel,robot_->getEEVelocity(),xtilde,xtildedot);
   
    Eigen::VectorXd y=pseudoinverse(robot_->getEEJacobian().data)*(toEigen(_desAcc)+_Kdp*xtildedot+_Kpp*xtilde-robot_->getEEJacDot()*robot_->getJntVelocities());
    
    return  robot_->getJsim()*y + robot_->getCoriolis(); //+ robot_->getGravity();
}

