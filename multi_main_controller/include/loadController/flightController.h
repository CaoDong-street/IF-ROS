/**
 * @file flightController.h
 * @brief  Implement payload controller
 */

#ifndef FLIGHT_CONTROLLER_H
#define FLIGHT_CONTROLLER_H
#include "loadController/math.h"
#include "loadController/common_include.h"
#include "loadController/dataStructure.h"
#include "trajGenerator/trajectory_generator_waypoint.h"

#define PI 3.1415926535

Eigen::Vector3d e3(Eigen::Vector3d::UnitZ());

class PayloadController
{
public:
    ptc::Math math;

    Vec3 sd_past;

    Vec3 desired_UAV1and2vector; // desired direction of formation
    Vec3 distance_target;        // distance between payload position and target position

    // pub
    nav_msgs::Odometry desiredLoadPos; // Desired payload position
    nav_msgs::Odometry desiredPos;     // Desired quadrotor1 position
    nav_msgs::Odometry desiredPos1;    // Desired quadrotor2 position
    nav_msgs::Odometry feedbackLoad;   // feedback payload position
    nav_msgs::Odometry feedbackLoad1;

    /// controller parameters
    Vec3 ky_p;
    Vec3 ky_d;

    /// payload physics
    payload_physics physics_payload;

    /// quadrotor1 feedback states
    Feedback_t feedback;
    /// quadrotor1 calculated states
    Calculate_t calculate;
    /// quadrotor1 desired states
    Desired_t desired;
    /// quadrotor1 output
    Output_t output;
    /// quadrotor1 controller variables
    controller_t UAVC;
    /// quadrotor1 physics
    quadrotor_physics physics;

    /// quadrotor2 feedback states
    Feedback_t feedback1;
    /// quadrotor2 calculated states
    Calculate_t calculate1;
    /// quadrotor2 desired states
    Desired_t desired1;
    /// quadrotor2 output
    Output_t output1;
    /// quadrotor2 controller variables
    controller_t UAVC1;
    /// quadrotor2 physics
    quadrotor_physics physics1;

    PayloadController();

    ~PayloadController();

    // Initialize the quadrotor and payload parameters
    void initializeParameter(double inputMassQuadcopter, double inputMassQuadcopter1, double inputMassPayload,
                             double inputLength, double inputLength1, double payload_quadrotor_1_Vector_x,
                             double payload_quadrotor_2_Vector_x);

    ///  Update quadrotor1 IMU state
    void updateImu(const sensor_msgs::ImuConstPtr &imu_msg);

    ///  Update quadrotor2 IMU state
    void updateImu1(const sensor_msgs::ImuConstPtr &imu_msg);

    /// Update quadrotor1 odometer state
    void updateOdom(const nav_msgs::OdometryConstPtr &odom_msg);

    /// Update quadrotor2 odometer state
    void updateOdom1(const nav_msgs::OdometryConstPtr &odom_msg);

    /// Update payload state, quadrotor1
    void updatePayload(const geometry_msgs::Twist &payload_msg);

    /// Update payload state, quadrotor2
    void updatePayload1(const geometry_msgs::Twist &payload_msg);

    /// get the propeller speed of quadrotor 1
    Vec4 getRevs();

    /// get the propeller speed of quadrotor 2
    Vec4 getRevs1();

    /// get the lift force of quadrotor 1
    double getLiftForce();

    /// get the lift force of quadrotor 2
    double getLiftForce1();

    /// get the quaternion of quadrotor 1
    geometry_msgs::Quaternion getQuaternion();

    /// get the quaternion of quadrotor 2
    geometry_msgs::Quaternion getQuaternion1();

    /// Set the formation
    void set_Vector_x(double payload_quadrotor_1_Vector_x, double payload_quadrotor_2_Vector_x);

    // Initialize PID parameters
    void initializePIDParameter(double ky_p1, double ky_p2, double ky_p3, double ky_d1, double ky_d2, double ky_d3,
                                double k_q1, double k_q2, double k_q3, double k_w1, double k_w2, double k_w3,
                                double k_R1, double k_R2, double k_R3, double k_Omega1, double k_Omega2,
                                double k_Omega3, double k_bx1, double k_bx2, double k_bx3, double k_bv1, double k_bv2,
                                double k_bv3, double Xk_q1, double Xk_q2, double Xk_q3, double Xk_w1, double Xk_w2,
                                double Xk_w3, double Xk_R1, double Xk_R2, double Xk_R3, double Xk_Omega1,
                                double Xk_Omega2, double Xk_Omega3, double Xk_bx1, double Xk_bx2, double Xk_bx3,
                                double Xk_bv1, double Xk_bv2, double Xk_bv3);

    /// continuous  controller
    void Continuous_Controller(const Vec3 &desiredPayloadPos, const Vec3 &desiredPayloadVel,
                               const Vec3 &desiredPayloadAcc, const Vec3 &desiredPayloadJerk,
                               const Vec3 &desiredPayloadSnap, const Vec3 &targetpoint,
                               const Vec3 &average_desiredPayloadVel_Orientation);
    /// cross controller
    void Cross_Controller(const Vec3 &desiredPayloadPos, const Vec3 &desiredPayloadVel, const Vec3 &desiredPayloadAcc,
                          const Vec3 &desiredPayloadJerk, const Vec3 &desiredPayloadSnap, const Vec3 &targetpoint,
                          const Vec3 &average_desiredPayloadVel_Orientation);
    /// transition controller
    void Transition_Controller(const Vec3 &desiredU1Pos, const Vec3 &desiredU1Vel, const Vec3 &desiredU1Acc,
                               const Vec3 &desiredU2Pos, const Vec3 &desiredU2Vel, const Vec3 &desiredU2Acc,
                               const Vec3 &payloadPos);

private:
    /// quadrotor1, Control distribution that translates pulling force and torque into corresponding propeller speeds.
    void propellerController();
    /// quadrotor2, Control distribution that translates pulling force and torque into corresponding propeller speeds.
    void propellerController1();
};

/// Transition Trajectory Generator
void Transition_Trajectory_Generator(const std::vector<double> &temp_txt,
                                     std::vector<TrajectoryGeneratorWaypoint> &vect_Traj_UAVT1,
                                     std::vector<TrajectoryGeneratorWaypoint> &vect_Traj_UAVT2,
                                     std::vector<bool> &vect_T_Traj_lable, const double payload_quadrotor_Vector_x_1,
                                     const double payload_quadrotor_Vector_x_2, const double length,
                                     const double length1, const double TVel, const double TAcc, const int Torder = 4);

/// Transition Trajectory Generator
void Transition_Trajectory_Generator(const geometry_msgs::PoseStamped &start_waypoints_payload,
                                     const nav_msgs::Path &waypoints_payload,
                                     std::vector<TrajectoryGeneratorWaypoint> &vect_Traj_UAVT1,
                                     std::vector<TrajectoryGeneratorWaypoint> &vect_Traj_UAVT2,
                                     std::vector<bool> &vect_T_Traj_lable, const nav_msgs::Path &waypoints_quadrotor_1,
                                     const nav_msgs::Path &waypoints_quadrotor_2, const double length,
                                     const double length1, const double TVel, const double TAcc, const int Torder = 4);

#endif // FLIGHT_CONTROLLER_H
