
#include <vector>
#include <Eigen/SVD>
#include <Eigen/Core>
#include "Robot.h"
using namespace std;

int main() {
    AutoDofRobot robot;
    robot.test_init();
    std::vector<double> joint = {-28,-41,150,0,43,1};
    // robot.setJoint(joint);
    auto A = robotics::toolbox::calculateJacobian(robot,joint);
    // ¥Ú”°æÿ’ÛA
    cout << "æÿ’ÛAŒ™:" << endl;
    cout << A << endl;

    auto dj = robotics::toolbox::get_singularValues_Per(A);
    cout << "∆Ê“Ï÷µº∆À„:" << endl;
    cout << dj << endl;
    return 0;
}
