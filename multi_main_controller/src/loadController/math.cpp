#include <loadController/math.h>

namespace ptc {
Vec3 Math::getVectorDiff(const Vec3 &cVector, const Vec3 &pVector, const double delta_t)
{
    return (cVector - pVector) / delta_t;
}

Vec3 Math::getVectorDiff(const Vec3 &cVector, const Vec3 &pVector, const Vec3 &ppVector, const double delta_t)
{
    return (cVector - 2 * pVector + ppVector) / delta_t / delta_t;
}

Mat33 Math::vectorToAntisymmetricMatrix(const Vec3 &vector)
{
    Mat33 antisymmetricMatrix;
    antisymmetricMatrix << 0, -vector.z(), vector.y(), vector.z(), 0, -vector.x(), -vector.y(), vector.x(), 0;
    return antisymmetricMatrix;
};

Vec3 Math::antisymmetricMatrixToVector(const Mat33 &antisymmetricMatrix)
{
    Vec3 vector(antisymmetricMatrix(7), antisymmetricMatrix(2), antisymmetricMatrix(3));
    return vector;
}

double Math::getPassTime()
{
    ros::Time now = ros::Time::now();
    double now_sec = (double)(now.nsec) * 1e-9 + (double)(now.sec);
    double last_sec = (double)(time_now.nsec) * 1e-9 + (double)(time_now.sec);
    return now_sec - last_sec;
}

void Math::setTime(const ros::Time &now) { time_now = now; }

Vec3 Math::normalized(const Vec3 &inputVec3) { return inputVec3 / inputVec3.norm(); }

} // namespace ptc