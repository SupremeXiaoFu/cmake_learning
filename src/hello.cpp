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


    cout << "����������������������������������������������������������������������������������������������������egien����ԡ���������������������������������������������������������������������������������������������������������������"  << endl;
    cout << "Eigen�汾: " << EIGEN_WORLD_VERSION << "." 
         << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << endl;
    
    // ����3x3����
    Matrix3d m = Matrix3d::Random();
    cout << "���3x3����:\n" << m << endl;
    
    // ��������
    Matrix3d result = m * m.transpose();
    cout << "\n���������ת��:\n" << result << endl;
    
    // ������Է����� Ax = b
    Vector3d b = Vector3d::Random();
    Vector3d x = m.colPivHouseholderQr().solve(b);
    cout << "\n��� Ax = b:" << endl;
    cout << "A =\n" << m << endl;
    cout << "b = " << b.transpose() << endl;
    cout << "x = " << x.transpose() << endl;
    cout << "��֤ Ax = " << (m * x).transpose() << endl;
    
    cout << "����������������������������������������������������������������������������������������������������orocos_kdl����ԡ���������������������������������������������������������������������������������������������������������������"  << endl;
    // ����KDL��
    KDL::Chain chain;
    // ���һ���ؽ�
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                                  KDL::Frame(KDL::Vector(1.0, 0.0, 0.0))));
    cout << "KDL�������ɹ����ؽ�����: " << chain.getNrOfJoints() << endl;
    

    return 0;
}