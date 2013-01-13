#include <Eigen/Dense>
#ifndef CONSTRAINT
#define CONSTRAINT
using namespace Eigen;
class Constraint
{

public:
        Constraint(){}
        int i;
        int j;
        Vector3d z;
        Matrix3d omega;

};

#endif
