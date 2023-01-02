//
// Created by zhangXX on 2022/12/12.
//

#ifndef UNTITLED_ROBOT_H
#define UNTITLED_ROBOT_H

#include <vector>
#include <Eigen/SVD>
#include <Eigen/Core>
#include <iostream>

namespace math {
    static const float RAD2DEG = 180.0f / static_cast<float>(M_PI);
    static const float DEG2RAD = static_cast<float>(M_PI) / 180.0f;
    static const float EPSINON = 0.0002;
    static const float DIFFERENTIAL  = 0.5;
}



/// DH参数表
class DHJoint{
public:
    Eigen::Matrix4d GetMat();

    DHJoint(float _dx, float _alpha, float _d);

    void setJoint(float j);
private:
    float dx;   //dx
    float alpha;    // rx
    float theta;    // rz
    float d;        // dz
};

/// 定义6轴机器人
class AutoDofRobot{
public:
    void test_init(){
        std::vector<double> rz;
        std::vector<double> dz = {330,   0,   0, -420, 0, -80};
        std::vector<double> dx = { 50, 440, -35,    0, 0,   0};
        std::vector<double> rx = {-M_PI/2, M_PI, M_PI/2, M_PI/2, M_PI/2, M_PI};
        load(dx,dz,rx);
    }

    /// 加载DH参数
    void load(std::vector<double> _dx,std::vector<double> _dz,std::vector<double> _rx) ;

    /// 设置总轴数据
    void setJoints(std::vector<double> jts);

    /// 设置某个轴的轴数据
    void setJoint(int index,double value);

    /// 获取自由度
    int GetDof(){return 6;}

    /// 生成最终的4*4矩阵
    Eigen::Matrix4d GetTransformMatrix();

    std::vector<DHJoint> dhs;
};



namespace robotics{
    class toolbox{
    public:
        /// 计算雅各比矩阵
        static Eigen::MatrixXd calculateJacobian(AutoDofRobot robot, std::vector<double> jointAnglesA);

        /// 计算奇异值
        static bool get_singularValues_Per(Eigen::MatrixXd & origin);
    };
}

#endif //UNTITLED_ROBOT_H
