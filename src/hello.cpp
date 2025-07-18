#include "hello.h"
#include <iostream>
#include <Eigen/Dense>
#include <kdl/chain.hpp>
#include <kdl/frames.hpp>

using namespace Eigen;
using namespace std;

void Hello::say_hello() {
    cout << "Hello World!" << endl;
}




int main() {

    Hello hello;
    hello.say_hello();


    cout << "！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！egien垂霞編！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！"  << endl;
    cout << "Eigen井云: " << EIGEN_WORLD_VERSION << "." 
         << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << endl;
    
    // 幹秀3x3裳專
    Matrix3d m = Matrix3d::Random();
    cout << "昧字3x3裳專:\n" << m << endl;
    
    // 裳專塰麻
    Matrix3d result = m * m.transpose();
    cout << "\n裳專核參凪廬崔:\n" << result << endl;
    
    // 箔盾�瀰垠蹴耗� Ax = b
    Vector3d b = Vector3d::Random();
    Vector3d x = m.colPivHouseholderQr().solve(b);
    cout << "\n箔盾 Ax = b:" << endl;
    cout << "A =\n" << m << endl;
    cout << "b = " << b.transpose() << endl;
    cout << "x = " << x.transpose() << endl;
    cout << "刮屬 Ax = " << (m * x).transpose() << endl;
    
    cout << "！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！orocos_kdl垂霞編！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！"  << endl;
    // 幹秀KDL全
    KDL::Chain chain;
    // 耶紗匯倖購准
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                                  KDL::Frame(KDL::Vector(1.0, 0.0, 0.0))));
    cout << "KDL全幹秀撹孔��購准方楚: " << chain.getNrOfJoints() << endl;
    

    return 0;
}