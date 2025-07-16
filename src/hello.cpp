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


    cout << "¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ªegien¿â²âÊÔ¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª"  << endl;
    cout << "Eigen°æ±¾: " << EIGEN_WORLD_VERSION << "." 
         << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << endl;
    
    // ´´½¨3x3¾ØÕó
    Matrix3d m = Matrix3d::Random();
    cout << "Ëæ»ú3x3¾ØÕó:\n" << m << endl;
    
    // ¾ØÕóÔËËã
    Matrix3d result = m * m.transpose();
    cout << "\n¾ØÕó³ËÒÔÆä×ªÖÃ:\n" << result << endl;
    
    // Çó½âÏßÐÔ·½³Ì×é Ax = b
    Vector3d b = Vector3d::Random();
    Vector3d x = m.colPivHouseholderQr().solve(b);
    cout << "\nÇó½â Ax = b:" << endl;
    cout << "A =\n" << m << endl;
    cout << "b = " << b.transpose() << endl;
    cout << "x = " << x.transpose() << endl;
    cout << "ÑéÖ¤ Ax = " << (m * x).transpose() << endl;
    
    cout << "¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ªorocos_kdl¿â²âÊÔ¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª¡ª"  << endl;
    // ´´½¨KDLÁ´
    KDL::Chain chain;
    // Ìí¼ÓÒ»¸ö¹Ø½Ú
    chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                                  KDL::Frame(KDL::Vector(1.0, 0.0, 0.0))));
    cout << "KDLÁ´´´½¨³É¹¦£¬¹Ø½ÚÊýÁ¿: " << chain.getNrOfJoints() << endl;
    

    return 0;
}