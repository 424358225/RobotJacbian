//
// Created by zhangXX on 2022/12/12.
//

#include "Robot.h"


Eigen::Matrix4d DHJoint::GetMat()
{
    Eigen::Matrix4d Tix = Eigen::Matrix4d::Identity();
    Tix << cos(theta) , -sin(theta)*cos(alpha) , sin(theta)*sin(alpha)   ,dx*cos(theta) ,
    sin(theta) , cos(theta)*cos(alpha)  , -cos(theta)*sin(alpha)  ,dx*sin(theta) ,
    0                , sin(alpha)                   , cos(alpha)     ,  d  ,
    0,0,0,1;
    return Tix;
}

DHJoint::DHJoint(float _dx, float _alpha, float _d)
    : dx(_dx), alpha(_alpha), d(_d)
    {

}

void DHJoint::setJoint(float j)
{
    theta = j * math::DEG2RAD;
}


void AutoDofRobot::load(std::vector<double> _dx, std::vector<double> _dz, std::vector<double> _rx)
{
    assert(_dx.size() == 6);
    assert(_dz.size() == 6);
    assert(_rx.size() == 6);

    dhs.clear();
    for(int i = 0;i<6;i++){
        DHJoint  dj(_dx[i],_rx[i],_dz[i]);
        dhs.push_back(dj);
    }
}

void AutoDofRobot::setJoints(std::vector<double> jts)
{
    assert(jts.size() <= dhs.size());
    for(int i = 0;i<dhs.size();i++){
        dhs[i].setJoint(jts[i]);
    }
}

Eigen::Matrix4d AutoDofRobot::GetTransformMatrix() {
    Eigen::Matrix4d mall = Eigen::Matrix4d::Identity();
    for(int i = 0;i<dhs.size();i++){
        mall *= dhs[i].GetMat();
    }
    return mall;
}

void AutoDofRobot::setJoint(int index,double value)
{
    int i = index;
    assert(index < GetDof());
    dhs[i].setJoint(value);
}

Eigen::MatrixXd robotics::toolbox::calculateJacobian(AutoDofRobot robot, std::vector<double> jointAnglesA) {
    int dof = robot.GetDof();
    Eigen::MatrixXd jacobian = Eigen::MatrixXd::Identity(dof,dof);
    std::vector<double> jointAnglesB(dof);

    robot.setJoints(jointAnglesA);
    // use anglesA to get the hand matrix
    Eigen::Matrix4d T = robot.GetTransformMatrix();

    int i,j;
    for(i=0;i<dof;++i) {  // for each axis
        for(j=0;j<dof;++j) {
            jointAnglesB[j]=jointAnglesA[j];
        }
        // use anglesB to get the hand matrix after a tiiiiny adjustment on one axis.
        jointAnglesB[i] += math::DIFFERENTIAL;
        robot.setJoints(jointAnglesB);
        Eigen::Matrix4d Tnew = robot.GetTransformMatrix();

        // 使用数字法计算
        Eigen::Matrix4d dT = Eigen::Matrix4d::Identity();
        dT = Tnew - T;
        dT *= (1.0/math::DIFFERENTIAL * math::DEG2RAD);

        jacobian(i,0)=dT(0,3);
        jacobian(i,1)=dT(1,3);
        jacobian(i,2)=dT(2,3);

        Eigen::Matrix3d T3 = T.block(0,0,3,3);
        Eigen::Matrix3d dT3 = dT.block(0,0,3,3);
        dT3.transpose();
        // T3.transpose();  // inverse of a rotation matrix is its transpose
        Eigen::Matrix3d skewSymmetric = Eigen::Matrix3d ::Identity();
        skewSymmetric = dT3 * T3;

        //[  0 -Wz  Wy]
        //[ Wz   0 -Wx]
        //[-Wy  Wx   0]
        jacobian(i,3)=skewSymmetric(1,2);  // Wx
        jacobian(i,4)=skewSymmetric(2,0);  // Wy
        jacobian(i,5)=skewSymmetric(0,1);  // Wz

    }
    return jacobian;
}

bool robotics::toolbox::get_singularValues_Per(Eigen::MatrixXd &origin) {
    // 进行svd分解
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(origin);
    // 构建SVD分解结果
//    Eigen::MatrixXd U = svd_holder.matrixU();
//    Eigen::MatrixXd V = svd_holder.matrixV();
    Eigen::MatrixXd D = svd_holder.singularValues();
    double condition = D(D.size()-1);
    std::cout << "Minimum value for int: " << std::numeric_limits<float>::epsilon() << std::endl;
    std::cout << "奇异数据:" << std::endl;
    std::cout << condition << std::endl;
    //return  std::abs(condition) > std::numeric_limits<double>::epsilon() ? false : true;
    return  std::abs(condition) > math::EPSINON ? false : true;
}


