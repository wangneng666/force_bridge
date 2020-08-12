/***********************四元数转RPY**********************************/
// x y z w A B C
#include "rosconsole/macros_generated.h"
#include <math.h>
class matrixUtily
{
public:
    static  void dumpDVec(const std::vector<double> & vec,int size, const std::string & name){
        ROS_INFO_STREAM(name.c_str() <<" "<< vec.at(0)<<" "<< vec.at(1)<<" "<< vec.at(2)<<" "<< vec.at(3)<<" "<< vec.at(4)<<" "<< vec.at(5)<<" ");
    }

    static void QtoETool(double q1, double q2, double q3, double q0, double& A, double &B, double &C){
            A = atan2(2*(q0*q1+q2*q3),1-2*(q1*q1+q2*q2));
            B = asin(2*(q0*q2-q1*q3));
            C = atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3));
    }
};
