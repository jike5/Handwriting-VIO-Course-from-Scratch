#include <iostream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "sophus/so3.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char **argv){
    // w
    Vector3d v_w(0.01, 0.02, 0.03);
    Sophus::SO3d SO3_w = Sophus::SO3d::exp(v_w);
    Matrix3d R_w = SO3_w.matrix();
    Quaterniond Q_w(R_w);

    // R
    Matrix3d R = Matrix3d::Identity();
    Sophus::SO3d SO3_R(R);
    Sophus::SO3d SO3_R_ed = SO3_R * SO3_w;
    cout << "R_updated = \n" << SO3_R_ed.matrix() << endl;

    // Q
    Quaterniond Q(R);
    Quaterniond Q_ed = Q * Q_w;
    cout << "Q_updated = " << Q_ed.coeffs().transpose() << endl;
    // cout << "Q_updated_norm = " << Q_ed.normalized().coeffs().transpose() << endl;
    // cout << "Q_ed.norm = " << Q_ed.coeffs().transpose() << endl;
    cout << "R_Q_ed = \n" << Q_ed.matrix() << endl;
    return 0;
}