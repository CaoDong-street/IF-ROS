/**
 * @file dataStructure.h
 * @brief Define the data structures used in the controller
 */

#ifndef LOADCONTROLLER_DATASTRUCTURE_H
#define LOADCONTROLLER_DATASTRUCTURE_H

#include "loadController/common_include.h"

/**
 * @brief Type 1:
 *          feedback data
 *          × quadrotor position velocity
 *          × payload position
 *          × quadrotor attitude quaternion, angular velocity, angular acceleration
 */
typedef struct
{
    Vec3 cBodyPosition;
    Vec3 cBodyVelocity;
    Vec3 cBodyAcc;

    // payload position in the body coordinate system
    Vec3 cPayloadPos_B;

    Vec3 cPayloadPosition;
    Vec3 pPayloadPosition;
    Vec3 cPayloadVelocity;
    Vec3 pPayloadVelocity;
    Vec3 cPayloadAcc;
    Vec3 pPayloadAcc;
    Vec3 cPayloadJerk;
    Vec3 pPayloadJerk;

    // feedback quaternion
    Eigen::Quaterniond cBodyQuaternion;
    Eigen::Matrix3d cBodyRotationMatrix;

    // body angular velocity
    Vec3 cBodyAngularVelocity;
    Vec3 pBodyAngularVelocity;
    Vec3 ppBodyAngularVelocity;

} Feedback_t;

/**
 * @brief  Type 2:
 *          calculated states
 *          × quadrotor acceleration
 *          × payload velocity, payload acceleration
 *          × quadrotor attitude Euler angles, rotation matrix
 *          × Vector of the direction of tension of the payload
 */
typedef struct
{
    // time
    double dt;
    double past;

    // body position velocity acceleration
    Vec3 pBodyPosition;
    Vec3 pBodyVelocity;
    Vec3 cBodyAcceleration;

    // payload position velocity acceleration
    Vec3 cPayloadPosition;
    Vec3 pPayloadPosition;

    Vec3 cPayloadVelocity;
    Vec3 pPayloadVelocity;

    Vec3 cPayloadAcceleration;

    // body angular velocity
    Vec3 cBodyAngularVelocity;
    Vec3 pBodyAngularVelocity;
    Vec3 ppBodyAngularVelocity;

    Mat33 cRotationMatrix;
    Mat33 pRotationMatrix;
    Mat33 ppRotationMatrix;

    // payload tension direction vector
    Vec3 cPayloadVector;
    Vec3 pPayloadVector;
    Vec3 ppPayloadVector;

    // rate of change of payload tension direction vector
    Vec3 cPayloadVectorD;
    Vec3 pPayloadVectorD;
    Vec3 cPayloadVectorDD;

    // payload tension direction vector
    Vec3 cPayloadOrientation;
    Vec3 pPayloadOrientation;
    Vec3 ppPayloadOrientation;

    // Rate of change of payload tension direction vector
    Vec3 cPayloadOrientationD;
    Vec3 cPayloadOrientationDD;

    Vec3 cBodyAngularAcceleration;
    Vec3 pBodyAngularAcceleration;

    Vec3 accumulatePayloadPositionError;
    Vec3 accumulateBodyPositionError;

} Calculate_t;

/**
 * @brief  Type 2:
 *         desired state:
 *
 */
typedef struct
{
    Vec3 cPayloadPosition;
    Vec3 pPayloadPosition;
    Vec3 ppPayloadPosition;

    Vec3 cPayloadVelocity;
    Vec3 cPayloadAcceleration;
    Vec3 cPayloadJerk;
    Vec3 cPayloadSnap;

    Vec3 cPayloadVector;
    Vec3 pPayloadVector;
    Vec3 ppPayloadVector;
    Vec3 cPayloadVectorD;
    Vec3 cPayloadVectorDD;

    Vec3 cPayloadOrientation;
    Vec3 pPayloadOrientation;
    Vec3 ppPayloadOrientation;
    Vec3 cPayloadOrientationD;
    Vec3 pPayloadOrientationD;

    Vec3 cPayloadOrientationDD;

    Mat33 cRotationMatrix;
    Vec3 cBodyPosition;
    Vec3 pBodyPosition;
    Vec3 cBodyVelocity;
    Vec3 cBodyAcc;
    Eigen::Quaterniond cBodyQuat;
    double yaw;

    Vec3 cBodyEulerAngle;
    Vec3 pBodyEulerAngle;
    Vec3 ppBodyEulerAngle;
    Vec3 cBodyAngularVelocity;
    Vec3 cBodyAngularAcceleration;

    Vec3 forceVector;

    Vec3 Tp;
    double norm_Tp;
    Vec3 q;

    Vec3 dTp;
    double dnorm_Tp;
    Vec3 dq;

    Vec3 d2Tp;
    double d2norm_Tp;
    Vec3 d2q;

    Vec3 d3Tp;
    double d3norm_Tp;
    Vec3 d3q;

    Vec3 omega;
    Vec3 domega;

    Vec3 cWd;
    Vec3 pWd;
    Vec3 dWd;
} Desired_t;
/**
 * @brief  Type 4:
 *         output :
 *
 */
typedef struct
{
    Vec3 M;
    Vec3 eulerAngular;
    double thrust;
    geometry_msgs::Quaternion quaternion;
    Vec4 revs;

} Output_t;

/**
 * @brief  Type 5:
 *         controller variables:
 *
 */
typedef struct
{
    Vec3 relative_desired_PayloadVector; // Cable direction vector in relative configuration coordinate system(desired)

    Vec3 desired_PayloadVector; // Cable direction vector in world coordinate system(desired)
    // Cable direction in relative configuration coordinate system(desired), r_id
    Vec3 relative_desired_PayloadOrientation;
    Vec3 desired_PayloadOrientation;   // Cable direction vector in configuration coordinate system(desired), q_id
    Vec3 desired_PayloadOrientationD;  // d(q_id)/dt
    Vec3 desired_PayloadOrientationDD; // d(d(q_id)/dt)/dt
    Vec3 desired_PayloadOrientationW;  // Angular velocity of the cable in the world coordinate system(desired), w_id
    Vec3 desired_PayloadOrientationWD; // d(w_id)/dt
    Vec3 cPayloadOrientation;          // Cable direction vector in world coordinate system, q_i
    Vec3 cPayloadOrientationD;         // d(q_i)/dt
    Vec3 cPayloadOrientationW;         // w_i
    Vec3 desired_miu;                  // desired virtual force
    Vec3 miu;                          // virtual force
    Vec3 F_n;                          // quadrotor control
    Vec3 F;                            // quadrotor output tension, u
    Vec3 u_parallel;                   // parallel component
    Vec3 u_vertical;                   // vertical component

    // parameters of the direction vector q
    Vec3 k_q;
    Vec3 k_w;
    // parameters of the quadrotor rotation matrix R
    Vec3 k_R;
    Vec3 k_Omega;
    //  parameters of the quadrotor position
    Vec3 k_bx;
    Vec3 k_bv;

    // parameters of the direction vector q
    Vec3 Xk_q;
    Vec3 Xk_w;
    // parameters of the quadrotor rotation matrix R
    Vec3 Xk_R;
    Vec3 Xk_Omega;
    //  parameters of the quadrotor position
    Vec3 Xk_bx;
    Vec3 Xk_bv;

    Vec3 accumulateError;   // accumulated quadrotor position error
    Vec3 errorRotation;     // quadrotor rotation matrix error
    Vec3 errorAngular;      // quadrotor angular velocity error
    Vec3 errorOrientation;  // cable direction vector error e_q_i
    Vec3 errorOrientationD; // e_w_i
    Vec3 errorBodyPos;      // quadrotor position error
    Vec3 errorBodyVel;      // quadrotor velocity error

    Vec3 eulerAngle;             // quadrotor Euler Angle
    Mat33 desiredRotationMatrix; // desired rotation matrix, R_c_i
    Vec3 b1_des;
    Vec3 b2_des;
    Vec3 b3_des;

    double relative_desired_PayloadVector_x;

} controller_t;

/**
 * @brief  Type 6:
 *         quadrotor physics:
 *
 */
typedef struct
{
    double mass;
    double length;
} quadrotor_physics;

/**
 * @brief  Type 7:
 *         payload physics:
 *
 */
typedef struct
{
    double mass;
} payload_physics;

#endif // LOADCONTROLLER_DATASTRUCTURE_H
