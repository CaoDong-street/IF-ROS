/**
 * @file math.h
 * @brief Complete math calculations in the controller
 */

#ifndef TRACKER_MATH_H
#define TRACKER_MATH_H
#include "loadController/common_include.h"
namespace ptc {
/// Math class
class Math
{
private:
    ros::Time time_now;

public:
    Math()
    {
        g = -9.8;
        ezI = Vec3(0, 0, 1);
    }
    double g;
    Vec3 ezI;
    /// Calculate the time from the last setTime to the present, in s
    double getPassTime();
    /// Setting to update the current time
    void setTime(const ros::Time &now);
    /// Compute the vectorial difference
    Vec3 getVectorDiff(const Vec3 &cVector, const Vec3 &pVector, const double delta_t);
    /// Calculate the second-order difference
    Vec3 getVectorDiff(const Vec3 &cVector, const Vec3 &pVector, const Vec3 &ppVector, const double delta_t);

    /// Vector to antisymmetric matrix
    Mat33 vectorToAntisymmetricMatrix(const Vec3 &vector);
    /// antisymmetric matrix to Vector
    Vec3 antisymmetricMatrixToVector(const Mat33 &antisymmetricMatrix);

    ///  normalize
    Vec3 normalized(const Vec3 &inputVec3);
};

} // namespace ptc

#endif // TRACKER_MATH_H
