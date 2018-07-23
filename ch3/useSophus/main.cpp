#include <iostream>
#include <cmath>
using namespace std;


#include <Eigen/Core>
#include <Eigen/Geometry>

#include "sophus/so3.h"
#include "sophus/se3.h"

int main()
{
    Eigen::Matrix3d R = Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(0,0,1)).toRotationMatrix();
    Sophus::SO3 SO3_R(R);
    Sophus::SO3 SO3_v(0,0,M_PI/2);
    Eigen::Quaterniond q(R);
    Sophus::SO3 SO3_q(q);

    cout << "SO(3) from matrix: "<<SO3_R << endl;
    cout << "SO(3) from vector: "<<SO3_v << endl;
    cout << "SO(3) from quaternion: "<<SO3_q << endl;
    cout << "=====================================================================================" << endl;

    Eigen::Vector3d so3 = SO3_R.log();
    cout<<"so3 = " <<so3.transpose()<<endl;
    cout<<"so3 hat= "<<Sophus::SO3::hat(so3)<<endl;
    cout<<"so3 hat vee = "<<Sophus::SO3::vee(Sophus::SO3::hat(so3)).transpose();

    cout << "=====================================================================================" << endl;

    return 0;
}
