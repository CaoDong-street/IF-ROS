#include <iostream>
#include <fstream>
#include "loadController/flightController.h"

using namespace std;

PayloadController::PayloadController() {}

PayloadController::~PayloadController() {}

void PayloadController::initializeParameter(double inputMassQuadcopter, double inputMassQuadcopter1,
                                            double inputMassPayload, double inputLength, double inputLength1,
                                            double payload_quadrotor_1_Vector_x, double payload_quadrotor_2_Vector_x)
{
    physics.mass = inputMassQuadcopter;
    physics1.mass = inputMassQuadcopter1;
    physics_payload.mass = inputMassPayload;
    physics.length = inputLength;
    physics1.length = inputLength1;
    UAVC.relative_desired_PayloadVector_x = payload_quadrotor_1_Vector_x;
    UAVC1.relative_desired_PayloadVector_x = -payload_quadrotor_2_Vector_x;
}

void PayloadController::updateImu(const sensor_msgs::ImuConstPtr &imu_msg)
{
    feedback.cBodyQuaternion = Eigen::Quaterniond(imu_msg->orientation.w, imu_msg->orientation.x,
                                                  imu_msg->orientation.y, imu_msg->orientation.z);
    feedback.cBodyAcc =
        Vec3(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
    feedback.cBodyAngularVelocity =
        Vec3(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);

    feedback.cBodyRotationMatrix = feedback.cBodyQuaternion.toRotationMatrix();
}

void PayloadController::updateImu1(const sensor_msgs::ImuConstPtr &imu_msg)
{
    feedback1.cBodyQuaternion = Eigen::Quaterniond(imu_msg->orientation.w, imu_msg->orientation.x,
                                                   imu_msg->orientation.y, imu_msg->orientation.z);
    feedback1.cBodyAcc =
        Vec3(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
    feedback1.cBodyAngularVelocity =
        Vec3(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);

    feedback1.cBodyRotationMatrix = feedback1.cBodyQuaternion.toRotationMatrix();
}

void PayloadController::updateOdom(const nav_msgs::OdometryConstPtr &odom_msg)
{
    feedback.cBodyPosition =
        Vec3(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
    feedback.cBodyVelocity =
        Vec3(odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y, odom_msg->twist.twist.linear.z);
}

void PayloadController::updateOdom1(const nav_msgs::OdometryConstPtr &odom_msg)
{
    feedback1.cBodyPosition =
        Vec3(odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, odom_msg->pose.pose.position.z);
    feedback1.cBodyVelocity =
        Vec3(odom_msg->twist.twist.linear.x, odom_msg->twist.twist.linear.y, odom_msg->twist.twist.linear.z);
}

void PayloadController::updatePayload(const geometry_msgs::Twist &payload_msg)
{
    calculate.dt = ros::Time::now().toSec() - calculate.past;
    calculate.past = ros::Time::now().toSec();

    feedback.cPayloadPos_B = Vec3(payload_msg.linear.x, payload_msg.linear.y, payload_msg.linear.z);

    Vec3 cPayloadVector = feedback.cPayloadPos_B - feedback.cBodyPosition;
    feedback.pPayloadPosition = feedback.cPayloadPosition;
    feedback.cPayloadPosition = feedback.cPayloadPos_B;

    calculate.pPayloadVector = calculate.cPayloadVector;
    calculate.cPayloadVector = cPayloadVector;
    calculate.cPayloadVectorD = math.getVectorDiff(calculate.cPayloadVector, calculate.pPayloadVector, calculate.dt);

    double alpha = 0.5;
    feedback.pPayloadVelocity = feedback.cPayloadVelocity;
    feedback.pPayloadAcc = feedback.cPayloadAcc;
    feedback.pPayloadJerk = feedback.cPayloadJerk;
    Vec3 cPayloadVelocity = math.getVectorDiff(feedback.cPayloadPosition, feedback.pPayloadPosition, calculate.dt);
    feedback.cPayloadVelocity = alpha * cPayloadVelocity + (1 - alpha) * feedback.cPayloadVelocity;
    Vec3 cPayloadAcc = math.getVectorDiff(feedback.cPayloadVelocity, feedback.pPayloadVelocity, calculate.dt);
    feedback.cPayloadAcc = alpha * cPayloadAcc + (1 - alpha) * feedback.cPayloadAcc;
    Vec3 cPayloadJerk = math.getVectorDiff(feedback.cPayloadAcc, feedback.pPayloadAcc, calculate.dt);
    feedback.cPayloadJerk = alpha * cPayloadJerk + (1 - alpha) * feedback.cPayloadJerk;

    feedbackLoad.header.frame_id = "map";
    feedbackLoad.header.stamp = ros::Time::now();

    feedbackLoad.pose.pose.position.x = feedback.cPayloadPosition.x();
    feedbackLoad.pose.pose.position.y = feedback.cPayloadPosition.y();
    feedbackLoad.pose.pose.position.z = feedback.cPayloadPosition.z();

    feedbackLoad.twist.twist.linear.x = feedback.cPayloadVelocity.x();
    feedbackLoad.twist.twist.linear.y = feedback.cPayloadVelocity.y();
    feedbackLoad.twist.twist.linear.z = feedback.cPayloadVelocity.z();

    calculate.cPayloadOrientation = calculate.cPayloadVector.normalized();
    calculate.cPayloadOrientationD = calculate.cPayloadVectorD.normalized();
}

void PayloadController::updatePayload1(const geometry_msgs::Twist &payload_msg)
{
    calculate1.dt = ros::Time::now().toSec() - calculate1.past;
    calculate1.past = ros::Time::now().toSec();

    feedback1.cPayloadPos_B = Vec3(payload_msg.linear.x, payload_msg.linear.y, payload_msg.linear.z);

    Vec3 cPayloadVector1 = feedback1.cPayloadPos_B - feedback1.cBodyPosition;
    feedback1.pPayloadPosition = feedback1.cPayloadPosition;
    feedback1.cPayloadPosition = feedback1.cPayloadPos_B;

    calculate1.pPayloadVector = calculate1.cPayloadVector;
    calculate1.cPayloadVector = cPayloadVector1;
    calculate1.cPayloadVectorD =
        math.getVectorDiff(calculate1.cPayloadVector, calculate1.pPayloadVector, calculate1.dt);

    double alpha = 0.5;
    feedback1.pPayloadVelocity = feedback1.cPayloadVelocity;
    feedback1.pPayloadAcc = feedback1.cPayloadAcc;
    feedback1.pPayloadJerk = feedback1.cPayloadJerk;
    Vec3 cPayloadVelocity = math.getVectorDiff(feedback1.cPayloadPosition, feedback1.pPayloadPosition, calculate1.dt);
    feedback1.cPayloadVelocity = alpha * cPayloadVelocity + (1 - alpha) * feedback1.cPayloadVelocity;
    Vec3 cPayloadAcc = math.getVectorDiff(feedback1.cPayloadVelocity, feedback1.pPayloadVelocity, calculate1.dt);
    feedback1.cPayloadAcc = alpha * cPayloadAcc + (1 - alpha) * feedback1.cPayloadAcc;
    Vec3 cPayloadJerk = math.getVectorDiff(feedback1.cPayloadAcc, feedback1.pPayloadAcc, calculate1.dt);
    feedback1.cPayloadJerk = alpha * cPayloadJerk + (1 - alpha) * feedback1.cPayloadJerk;

    feedbackLoad1.header.frame_id = "map";
    feedbackLoad1.header.stamp = ros::Time::now();

    feedbackLoad1.pose.pose.position.x = feedback1.cPayloadPosition.x();
    feedbackLoad1.pose.pose.position.y = feedback1.cPayloadPosition.y();
    feedbackLoad1.pose.pose.position.z = feedback1.cPayloadPosition.z();

    feedbackLoad1.twist.twist.linear.x = feedback1.cPayloadVelocity.x();
    feedbackLoad1.twist.twist.linear.y = feedback1.cPayloadVelocity.y();
    feedbackLoad1.twist.twist.linear.z = feedback1.cPayloadVelocity.z();

    calculate1.cPayloadOrientation = calculate1.cPayloadVector.normalized();
    calculate1.cPayloadOrientationD = calculate1.cPayloadVectorD.normalized();
}

void PayloadController::propellerController()
{
    Eigen::Matrix4d Minvese;
    double sqrt2 = sqrt(2);
    Minvese << 1, -sqrt2, sqrt2, 1, 1, -sqrt2, -sqrt2, -1, 1, sqrt2, -sqrt2, 1, 1, sqrt2, sqrt2, -1;
    Minvese = 0.25 * Minvese;

    Eigen::Vector4d input(output.thrust, output.M.x(), output.M.y(), output.M.z());
    output.revs = Minvese * input;

    if (output.revs.x() < 0)
    {
        output.revs.x() = 0;
    }
    if (output.revs.y() < 0)
    {
        output.revs.y() = 0;
    }
    if (output.revs.z() < 0)
    {
        output.revs.z() = 0;
    }
    if (output.revs.w() < 0)
    {
        output.revs.w() = 0;
    }
    output.revs.x() = sqrt(output.revs.x());
    output.revs.y() = sqrt(output.revs.y());
    output.revs.z() = sqrt(output.revs.z());
    output.revs.w() = sqrt(output.revs.w());
}

void PayloadController::propellerController1()
{
    Eigen::Matrix4d Minvese;
    double sqrt2 = sqrt(2);
    Minvese << 1, -sqrt2, sqrt2, 1, 1, -sqrt2, -sqrt2, -1, 1, sqrt2, -sqrt2, 1, 1, sqrt2, sqrt2, -1;
    Minvese = 0.25 * Minvese;

    Eigen::Vector4d input1(output1.thrust, output1.M.x(), output1.M.y(), output1.M.z());
    output1.revs = Minvese * input1;

    if (output1.revs.x() < 0)
    {
        output1.revs.x() = 0;
    }
    if (output1.revs.y() < 0)
    {
        output1.revs.y() = 0;
    }
    if (output1.revs.z() < 0)
    {
        output1.revs.z() = 0;
    }
    if (output1.revs.w() < 0)
    {
        output1.revs.w() = 0;
    }
    output1.revs.x() = sqrt(output1.revs.x());
    output1.revs.y() = sqrt(output1.revs.y());
    output1.revs.z() = sqrt(output1.revs.z());
    output1.revs.w() = sqrt(output1.revs.w());
}

Vec4 PayloadController::getRevs() { return output.revs; }

Vec4 PayloadController::getRevs1() { return output1.revs; }

double PayloadController::getLiftForce() { return output.thrust; }

double PayloadController::getLiftForce1() { return output1.thrust; }

geometry_msgs::Quaternion PayloadController::getQuaternion() { return output.quaternion; }

geometry_msgs::Quaternion PayloadController::getQuaternion1() { return output1.quaternion; }

void PayloadController::set_Vector_x(double payload_quadrotor_1_Vector_x, double payload_quadrotor_2_Vector_x)
{
    UAVC.relative_desired_PayloadVector_x = payload_quadrotor_1_Vector_x;
    UAVC1.relative_desired_PayloadVector_x = -payload_quadrotor_2_Vector_x;
}

void PayloadController::initializePIDParameter(double ky_p1, double ky_p2, double ky_p3, double ky_d1, double ky_d2,
                                               double ky_d3, double k_q1, double k_q2, double k_q3, double k_w1,
                                               double k_w2, double k_w3, double k_R1, double k_R2, double k_R3,
                                               double k_Omega1, double k_Omega2, double k_Omega3, double k_bx1,
                                               double k_bx2, double k_bx3, double k_bv1, double k_bv2, double k_bv3,
                                               double Xk_q1, double Xk_q2, double Xk_q3, double Xk_w1, double Xk_w2,
                                               double Xk_w3, double Xk_R1, double Xk_R2, double Xk_R3, double Xk_Omega1,
                                               double Xk_Omega2, double Xk_Omega3, double Xk_bx1, double Xk_bx2,
                                               double Xk_bx3, double Xk_bv1, double Xk_bv2, double Xk_bv3)
{
    ky_p = Vec3(ky_p1, ky_p2, ky_p3);
    ky_d = Vec3(ky_d1, ky_d2, ky_d3);

    UAVC.k_q = Vec3(k_q1, k_q2, k_q3);
    UAVC1.k_q = UAVC.k_q;

    UAVC.k_w = Vec3(k_w1, k_w2, k_w3);
    UAVC1.k_w = UAVC.k_w;

    UAVC.k_R = Vec3(k_R1, k_R2, k_R3);
    UAVC1.k_R = UAVC.k_R;

    UAVC.k_Omega = Vec3(k_Omega1, k_Omega2, k_Omega3);
    UAVC1.k_Omega = UAVC.k_Omega;

    UAVC.k_bx = Vec3(k_bx1, k_bx2, k_bx3);
    UAVC1.k_bx = UAVC.k_bx;

    UAVC.k_bv = Vec3(k_bv1, k_bv2, k_bv3);
    UAVC1.k_bv = UAVC.k_bv;

    UAVC.Xk_q = Vec3(Xk_q1, Xk_q2, Xk_q3);
    UAVC1.Xk_q = UAVC.Xk_q;

    UAVC.Xk_w = Vec3(Xk_w1, Xk_w2, Xk_w3);
    UAVC1.Xk_w = UAVC.Xk_w;

    UAVC.Xk_R = Vec3(Xk_R1, Xk_R2, Xk_R3);
    UAVC1.Xk_R = UAVC.Xk_R;

    UAVC.Xk_Omega = Vec3(Xk_Omega1, Xk_Omega2, Xk_Omega3);
    UAVC1.Xk_Omega = UAVC.Xk_Omega;

    UAVC.Xk_bx = Vec3(Xk_bx1, Xk_bx2, Xk_bx3);
    UAVC1.Xk_bx = UAVC.Xk_bx;

    UAVC.Xk_bv = Vec3(Xk_bv1, Xk_bv2, Xk_bv3);
    UAVC1.Xk_bv = UAVC.Xk_bv;
}

void PayloadController::Continuous_Controller(const Vec3 &desiredPayloadPos, const Vec3 &desiredPayloadVel,
                                              const Vec3 &desiredPayloadAcc, const Vec3 &desiredPayloadJerk,
                                              const Vec3 &desiredPayloadSnap, const Vec3 &targetpoint,
                                              const Vec3 &average_desiredPayloadVel_Orientation)
{
    desired.cPayloadPosition = desiredPayloadPos;
    desired.cPayloadVelocity = desiredPayloadVel;
    desired.cPayloadAcceleration = desiredPayloadAcc;
    desired.cPayloadJerk = desiredPayloadJerk;
    desired.cPayloadSnap = desiredPayloadSnap;

    desired1.cPayloadPosition = desiredPayloadPos;
    desired1.cPayloadVelocity = desiredPayloadVel;
    desired1.cPayloadAcceleration = desiredPayloadAcc;
    desired1.cPayloadJerk = desiredPayloadJerk;
    desired1.cPayloadSnap = desiredPayloadSnap;

    {
        Vec3 desired_UAV1and2vector = desired.cPayloadVelocity;
        Vec3 distance_target = targetpoint - feedback.cPayloadPosition;
        if (distance_target.norm() < 0.15)
        {
            desired_UAV1and2vector = Vec3(0, 0, 0);
        }
    }

    Vec3 F_d;
    Vec3 Fd_D;
    Vec3 Fd_DD;

    Vec3 errorPayloadPosition = feedback.cPayloadPosition - desired.cPayloadPosition;
    Vec3 errorPayloadVelocity = feedback.cPayloadVelocity - desired.cPayloadVelocity;
    Vec3 errorPayloadAcc = feedback.cPayloadAcc - desired.cPayloadAcceleration;
    Vec3 errorPayloadJerk = feedback.cPayloadJerk - desired.cPayloadJerk;
    F_d = -errorPayloadPosition.cwiseProduct(ky_p) - errorPayloadVelocity.cwiseProduct(ky_d) +
          physics_payload.mass * (desired.cPayloadAcceleration + e3 * 9.8);
    Fd_D = -errorPayloadVelocity.cwiseProduct(ky_p) - errorPayloadAcc.cwiseProduct(ky_d) +
           physics_payload.mass * (desired.cPayloadJerk);
    Fd_DD = -errorPayloadAcc.cwiseProduct(ky_p) - errorPayloadAcc.cwiseProduct(ky_d) +
            physics_payload.mass * (desired.cPayloadSnap);

    Vec3 sd = e3.cross(desiredPayloadVel);
    if (abs(desiredPayloadPos.x()) > 0.1)
    {
        if (desiredPayloadVel.norm() < 0.001)
        {
            sd = sd_past;
        }
    }
    sd_past = sd;

    Vec3 Q_3 = F_d / F_d.norm();
    Vec3 Q_2 = F_d.cross(sd);
    Q_2 = Q_2 / Q_2.norm();
    Vec3 Q_1 = F_d.cross(F_d.cross(sd));
    Q_1 = -Q_1 / Q_1.norm();

    Mat33 Q;
    Q.col(0) = Q_1;
    Q.col(1) = Q_2;
    Q.col(2) = Q_3;

    double norm_Fd = F_d.norm();
    double norm_FdD = Fd_D.dot(F_d) / norm_Fd;
    Vec3 FdCrosSd = F_d.cross(sd);
    Vec3 FdCrosSd_D = Fd_D.cross(sd);
    double norm_FdCrosSd = FdCrosSd.norm();
    double norm_FdCrosSdD = FdCrosSd_D.dot(FdCrosSd) / norm_FdCrosSd;
    Vec3 FdCroFdCroSd = F_d.cross(FdCrosSd);
    Vec3 FdCroFdCroSd_D = Fd_D.cross(FdCrosSd) + F_d.cross(FdCrosSd_D);
    double norm_FdCroFdCroSd = FdCroFdCroSd.norm();
    double norm_FdCroFdCroSdD = FdCroFdCroSd_D.dot(FdCroFdCroSd) / norm_FdCroFdCroSd;

    Vec3 Q_1d = -((FdCroFdCroSd_D * norm_FdCroFdCroSd - FdCroFdCroSd * norm_FdCroFdCroSdD) /
                  (norm_FdCroFdCroSd * norm_FdCroFdCroSd));
    Vec3 Q_2d = ((FdCrosSd_D * norm_FdCrosSd - FdCrosSd * norm_FdCrosSdD) / (norm_FdCrosSd * norm_FdCrosSd));
    Vec3 Q_3d = ((Fd_D * norm_Fd - F_d * norm_FdD) / (norm_Fd * norm_Fd));
    Mat33 QD;
    QD.col(0) = Q_1d;
    QD.col(1) = Q_2d;
    QD.col(2) = Q_3d;

    double norm_FdDD = ((Fd_DD.dot(F_d) + Fd_D.dot(Fd_D)) * norm_Fd - Fd_D.dot(F_d) * norm_FdD) / (norm_Fd * norm_Fd);
    Vec3 FdCrosSd_DD = Fd_DD.cross(sd);
    double norm_FdcroSdDD = ((FdCrosSd_DD.dot(FdCrosSd) + FdCrosSd_D.dot(FdCrosSd_D)) * norm_FdCrosSd -
                             FdCrosSd_D.dot(FdCrosSd) * norm_FdCrosSdD) /
                            (norm_FdCrosSd * norm_FdCrosSd);
    Vec3 FdCroFdCroSd_DD = 2 * Fd_D.cross(Fd_D.cross(sd)) + Fd_DD.cross(FdCrosSd) + F_d.cross(FdCrosSd_DD);
    double norm_FdCroFdCroSdDD =
        ((FdCroFdCroSd_DD.dot(FdCroFdCroSd) + FdCroFdCroSd_D.dot(FdCroFdCroSd_D)) * norm_FdCroFdCroSd -
         FdCroFdCroSd_D.dot(FdCroFdCroSd) * norm_FdCroFdCroSdD) /
        (norm_FdCroFdCroSd * norm_FdCroFdCroSd);

    Vec3 Q_1dd = -(FdCroFdCroSd_DD * norm_FdCroFdCroSd - FdCroFdCroSd * norm_FdCroFdCroSdDD) /
                     (norm_FdCroFdCroSd * norm_FdCroFdCroSd) +
                 2 * ((FdCroFdCroSd_D * norm_FdCroFdCroSd - FdCroFdCroSd * norm_FdCroFdCroSdD) * norm_FdCroFdCroSdD) /
                     (norm_FdCroFdCroSd * norm_FdCroFdCroSd * norm_FdCroFdCroSd);

    Vec3 Q_2dd = (FdCrosSd_DD * norm_FdCrosSd - FdCrosSd * norm_FdcroSdDD) / (norm_FdCrosSd * norm_FdCrosSd) +
                 2 * ((FdCrosSd_D * norm_FdCrosSd - FdCrosSd * norm_FdCrosSdD) * norm_FdCrosSdD) /
                     (norm_FdCrosSd * norm_FdCrosSd * norm_FdCrosSd);

    Vec3 Q_3dd = (Fd_DD * norm_Fd - F_d * norm_FdDD) / (norm_Fd * norm_Fd) +
                 2 * ((Fd_D * norm_Fd - F_d * norm_FdD) * norm_FdD) / (norm_Fd * norm_Fd * norm_Fd);
    Mat33 QDD;
    QDD.col(0) = Q_1dd;
    QDD.col(1) = Q_2dd;
    QDD.col(2) = Q_3dd;

    UAVC.relative_desired_PayloadVector =
        Vec3(UAVC.relative_desired_PayloadVector_x, 0,
             -sqrt(pow(physics.length, 2) - pow(UAVC.relative_desired_PayloadVector_x, 2)));
    UAVC.relative_desired_PayloadOrientation = UAVC.relative_desired_PayloadVector.normalized();
    UAVC.desired_PayloadVector = Q * UAVC.relative_desired_PayloadVector;
    UAVC.desired_PayloadOrientation = Q * UAVC.relative_desired_PayloadOrientation;

    UAVC1.relative_desired_PayloadVector =
        Vec3(UAVC1.relative_desired_PayloadVector_x, 0,
             -sqrt(pow(physics1.length, 2) - pow(UAVC1.relative_desired_PayloadVector_x, 2)));
    UAVC1.relative_desired_PayloadOrientation = UAVC1.relative_desired_PayloadVector.normalized();
    UAVC1.desired_PayloadVector = Q * UAVC1.relative_desired_PayloadVector;
    UAVC1.desired_PayloadOrientation = Q * UAVC1.relative_desired_PayloadOrientation;

    UAVC.cPayloadOrientation = calculate.cPayloadOrientation;
    UAVC.cPayloadOrientationD = calculate.cPayloadOrientationD;
    UAVC.desired_PayloadOrientationD = QD * UAVC.relative_desired_PayloadOrientation;
    UAVC.desired_PayloadOrientationDD = QDD * UAVC.relative_desired_PayloadOrientation;
    UAVC.cPayloadOrientationW = UAVC.cPayloadOrientation.cross(UAVC.cPayloadOrientationD);
    UAVC.desired_PayloadOrientationW = UAVC.desired_PayloadOrientation.cross(UAVC.desired_PayloadOrientationD);
    UAVC.desired_PayloadOrientationWD = UAVC.desired_PayloadOrientation.cross(UAVC.desired_PayloadOrientationDD);

    UAVC1.cPayloadOrientation = calculate1.cPayloadOrientation;
    UAVC1.cPayloadOrientationD = calculate1.cPayloadOrientationD;
    UAVC1.desired_PayloadOrientationD = QD * UAVC1.relative_desired_PayloadOrientation;
    UAVC1.desired_PayloadOrientationDD = QDD * UAVC1.relative_desired_PayloadOrientation;
    UAVC1.cPayloadOrientationW = UAVC1.cPayloadOrientation.cross(UAVC1.cPayloadOrientationD);
    UAVC1.desired_PayloadOrientationW = UAVC1.desired_PayloadOrientation.cross(UAVC1.desired_PayloadOrientationD);
    UAVC1.desired_PayloadOrientationWD = UAVC1.desired_PayloadOrientation.cross(UAVC1.desired_PayloadOrientationDD);

    desired.cBodyPosition = desired.cPayloadPosition - UAVC.desired_PayloadVector;

    desired.cBodyVelocity = desired.cPayloadVelocity;

    desired1.cBodyPosition = desired1.cPayloadPosition - UAVC1.desired_PayloadVector;

    desired1.cBodyVelocity = desired1.cPayloadVelocity;

    UAVC.errorBodyPos = feedback.cBodyPosition - desired.cBodyPosition;
    UAVC.errorBodyVel = feedback.cBodyVelocity - desired.cBodyVelocity;
    UAVC.accumulateError += UAVC.errorBodyPos;
    UAVC1.errorBodyPos = feedback1.cBodyPosition - desired1.cBodyPosition;
    UAVC1.errorBodyVel = feedback1.cBodyVelocity - desired1.cBodyVelocity;
    UAVC1.accumulateError += UAVC1.errorBodyPos;

    UAVC.errorOrientation = UAVC.desired_PayloadOrientation.cross(UAVC.cPayloadOrientation);

    UAVC1.errorOrientation = UAVC1.desired_PayloadOrientation.cross(UAVC1.cPayloadOrientation);

    UAVC.errorOrientationD =
        UAVC.cPayloadOrientationW +
        UAVC.cPayloadOrientation.cross(UAVC.cPayloadOrientation.cross(UAVC.desired_PayloadOrientationW));

    UAVC1.errorOrientationD =
        UAVC1.cPayloadOrientationW +
        UAVC1.cPayloadOrientation.cross(UAVC1.cPayloadOrientation.cross(UAVC1.desired_PayloadOrientationW));

    UAVC.miu = -F_d.dot(UAVC.cPayloadOrientation) * UAVC.cPayloadOrientation /
               ((UAVC.relative_desired_PayloadOrientation.dot(e3)) *
                (UAVC.relative_desired_PayloadOrientation + UAVC1.relative_desired_PayloadOrientation).norm());
    UAVC1.miu = -F_d.dot(UAVC1.cPayloadOrientation) * UAVC1.cPayloadOrientation /
                ((UAVC1.relative_desired_PayloadOrientation.dot(e3)) *
                 (UAVC.relative_desired_PayloadOrientation + UAVC1.relative_desired_PayloadOrientation).norm());

    UAVC.u_parallel = physics.mass * physics.length * UAVC.cPayloadOrientationW.norm() *
                          UAVC.cPayloadOrientationW.norm() * UAVC.cPayloadOrientation +
                      UAVC.miu;
    UAVC.u_vertical =
        physics.mass * physics.length *
        UAVC.cPayloadOrientation.cross(
            -UAVC.errorOrientation.cwiseProduct(UAVC.k_q) - UAVC.errorOrientationD.cwiseProduct(UAVC.k_w) -
            UAVC.cPayloadOrientation.dot(UAVC.desired_PayloadOrientationW) * UAVC.cPayloadOrientationD -
            UAVC.cPayloadOrientation.cross(UAVC.cPayloadOrientation.cross(UAVC.desired_PayloadOrientationWD)));

    UAVC.F_n = -UAVC.errorBodyPos.cwiseProduct(UAVC.k_bx) - UAVC.errorBodyVel.cwiseProduct(UAVC.k_bv) +
               (physics.mass) * desired.cPayloadAcceleration + (physics.mass) * (e3 * 9.8);
    UAVC.F = UAVC.u_parallel + UAVC.u_vertical + UAVC.F_n;

    UAVC1.u_parallel = physics1.mass * physics1.length * UAVC1.cPayloadOrientationW.norm() *
                           UAVC1.cPayloadOrientationW.norm() * UAVC1.cPayloadOrientation +
                       UAVC1.miu;
    UAVC1.u_vertical =
        physics1.mass * physics1.length *
        UAVC1.cPayloadOrientation.cross(
            -UAVC1.errorOrientation.cwiseProduct(UAVC1.k_q) - UAVC1.errorOrientationD.cwiseProduct(UAVC1.k_w) -
            UAVC1.cPayloadOrientation.dot(UAVC1.desired_PayloadOrientationW) * UAVC1.cPayloadOrientationD -
            UAVC1.cPayloadOrientation.cross(UAVC1.cPayloadOrientation.cross(UAVC1.desired_PayloadOrientationWD)));
    UAVC1.F_n = -UAVC1.errorBodyPos.cwiseProduct(UAVC1.k_bx) - UAVC1.errorBodyVel.cwiseProduct(UAVC1.k_bv) +
                (physics1.mass) * desired1.cPayloadAcceleration + (physics1.mass) * (e3 * 9.8);
    UAVC1.F = UAVC1.u_parallel + UAVC1.u_vertical + UAVC1.F_n;

    UAVC.b1_des = Vec3(cos(0), sin(0), 0);
    UAVC.b3_des = UAVC.F / UAVC.F.norm();
    UAVC.b2_des = UAVC.b3_des.cross(UAVC.b1_des);
    UAVC.b2_des /= UAVC.b2_des.norm();

    UAVC.desiredRotationMatrix;
    UAVC.desiredRotationMatrix.col(0) = UAVC.b2_des.cross(UAVC.b3_des);
    UAVC.desiredRotationMatrix.col(1) = UAVC.b2_des;
    UAVC.desiredRotationMatrix.col(2) = UAVC.b3_des;

    UAVC.eulerAngle = UAVC.desiredRotationMatrix.eulerAngles(0, 1, 2);

    output.quaternion =
        tf::createQuaternionMsgFromRollPitchYaw(UAVC.eulerAngle[0], UAVC.eulerAngle[1], UAVC.eulerAngle[2]);

    desiredPos.header.frame_id = "map";
    desiredPos.header.stamp = ros::Time::now();

    desiredPos.pose.pose.position.x = desired.cBodyPosition.x();
    desiredPos.pose.pose.position.y = desired.cBodyPosition.y();
    desiredPos.pose.pose.position.z = desired.cBodyPosition.z();
    desiredPos.pose.pose.orientation = output.quaternion;

    desiredPos.twist.twist.linear.x = desired.cBodyVelocity.x();
    desiredPos.twist.twist.linear.y = desired.cBodyVelocity.y();
    desiredPos.twist.twist.linear.z = desired.cBodyVelocity.z();

    UAVC.errorRotation =
        0.5 * math.antisymmetricMatrixToVector((UAVC.desiredRotationMatrix.transpose() * feedback.cBodyRotationMatrix -
                                                feedback.cBodyRotationMatrix.transpose() * UAVC.desiredRotationMatrix));
    UAVC.errorAngular = feedback.cBodyAngularVelocity;

    output.M = -UAVC.errorRotation.cwiseProduct(UAVC.k_R) + UAVC.errorAngular.cwiseProduct(UAVC.k_Omega);

    output.thrust = UAVC.b3_des.transpose() * UAVC.F;

    UAVC1.b1_des = Vec3(cos(0), sin(0), 0);
    UAVC1.b3_des = UAVC1.F / UAVC1.F.norm();

    UAVC1.b2_des = UAVC1.b3_des.cross(UAVC1.b1_des);
    UAVC1.b2_des /= UAVC1.b2_des.norm();

    UAVC1.desiredRotationMatrix.col(0) = UAVC1.b2_des.cross(UAVC1.b3_des);
    UAVC1.desiredRotationMatrix.col(1) = UAVC1.b2_des;
    UAVC1.desiredRotationMatrix.col(2) = UAVC1.b3_des;

    UAVC1.eulerAngle = UAVC1.desiredRotationMatrix.eulerAngles(0, 1, 2);

    output1.quaternion =
        tf::createQuaternionMsgFromRollPitchYaw(UAVC1.eulerAngle[0], UAVC1.eulerAngle[1], UAVC1.eulerAngle[2]);

    desiredPos1.header.frame_id = "map";
    desiredPos1.header.stamp = ros::Time::now();

    desiredPos1.pose.pose.position.x = desired1.cBodyPosition.x();
    desiredPos1.pose.pose.position.y = desired1.cBodyPosition.y();
    desiredPos1.pose.pose.position.z = desired1.cBodyPosition.z();
    desiredPos1.pose.pose.orientation = output1.quaternion;

    desiredPos1.twist.twist.linear.x = desired1.cBodyVelocity.x();
    desiredPos1.twist.twist.linear.y = desired1.cBodyVelocity.y();
    desiredPos1.twist.twist.linear.z = desired1.cBodyVelocity.z();

    UAVC1.errorRotation = 0.5 * math.antisymmetricMatrixToVector(
                                    (UAVC1.desiredRotationMatrix.transpose() * feedback1.cBodyRotationMatrix -
                                     feedback1.cBodyRotationMatrix.transpose() * UAVC1.desiredRotationMatrix));
    UAVC1.errorAngular = feedback1.cBodyAngularVelocity;

    output1.M = -UAVC1.errorRotation.cwiseProduct(UAVC1.k_R) + UAVC1.errorAngular.cwiseProduct(UAVC1.k_Omega);

    output1.thrust = UAVC1.b3_des.transpose() * UAVC1.F;
#ifdef VREP
    propellerController();
    propellerController1();
#endif
}

void PayloadController::Cross_Controller(const Vec3 &desiredPayloadPos, const Vec3 &desiredPayloadVel,
                                         const Vec3 &desiredPayloadAcc, const Vec3 &desiredPayloadJerk,
                                         const Vec3 &desiredPayloadSnap, const Vec3 &targetpoint,
                                         const Vec3 &average_desiredPayloadVel_Orientation)
{
    desired.cPayloadPosition = desiredPayloadPos;
    desired.cPayloadVelocity = desiredPayloadVel;
    desired.cPayloadAcceleration = desiredPayloadAcc;
    desired.cPayloadJerk = desiredPayloadJerk;
    desired.cPayloadSnap = desiredPayloadSnap;

    desired1.cPayloadPosition = desiredPayloadPos;
    desired1.cPayloadVelocity = desiredPayloadVel;
    desired1.cPayloadAcceleration = desiredPayloadAcc;
    desired1.cPayloadJerk = desiredPayloadJerk;
    desired1.cPayloadSnap = desiredPayloadSnap;

    {
        Vec3 desired_UAV1and2vector = desired.cPayloadVelocity;
        Vec3 distance_target = targetpoint - feedback.cPayloadPosition;
        if (distance_target.norm() < 0.15)
        {
            desired_UAV1and2vector = Vec3(0, 0, 0);
        }
    }

    Vec3 F_d;
    Vec3 Fd_D;
    Vec3 Fd_DD;

    Vec3 errorPayloadPosition = feedback.cPayloadPosition - desired.cPayloadPosition;
    Vec3 errorPayloadVelocity = feedback.cPayloadVelocity - desired.cPayloadVelocity;
    Vec3 errorPayloadAcc = feedback.cPayloadAcc - desired.cPayloadAcceleration;
    Vec3 errorPayloadJerk = feedback.cPayloadJerk - desired.cPayloadJerk;
    F_d = -errorPayloadPosition.cwiseProduct(ky_p) - errorPayloadVelocity.cwiseProduct(ky_d) +
          physics_payload.mass * (desired.cPayloadAcceleration + e3 * 9.8);
    Fd_D = -errorPayloadVelocity.cwiseProduct(ky_p) - errorPayloadAcc.cwiseProduct(ky_d) +
           physics_payload.mass * (desired.cPayloadJerk);
    Fd_DD = -errorPayloadAcc.cwiseProduct(ky_p) - errorPayloadAcc.cwiseProduct(ky_d) +
            physics_payload.mass * (desired.cPayloadSnap);

    Vec3 sd = e3.cross(average_desiredPayloadVel_Orientation);

    Vec3 Q_3 = F_d / F_d.norm();
    Vec3 Q_2 = F_d.cross(sd);
    Q_2 = Q_2 / Q_2.norm();
    Vec3 Q_1 = F_d.cross(F_d.cross(sd));
    Q_1 = -Q_1 / Q_1.norm();

    Mat33 Q;
    Q.col(0) = Q_1;
    Q.col(1) = Q_2;
    Q.col(2) = Q_3;

    double norm_Fd = F_d.norm();
    double norm_FdD = Fd_D.dot(F_d) / norm_Fd;
    Vec3 FdCrosSd = F_d.cross(sd);
    Vec3 FdCrosSd_D = Fd_D.cross(sd);
    double norm_FdCrosSd = FdCrosSd.norm();
    double norm_FdCrosSdD = FdCrosSd_D.dot(FdCrosSd) / norm_FdCrosSd;
    Vec3 FdCroFdCroSd = F_d.cross(FdCrosSd);
    Vec3 FdCroFdCroSd_D = Fd_D.cross(FdCrosSd) + F_d.cross(FdCrosSd_D);
    double norm_FdCroFdCroSd = FdCroFdCroSd.norm();
    double norm_FdCroFdCroSdD = FdCroFdCroSd_D.dot(FdCroFdCroSd) / norm_FdCroFdCroSd;

    Vec3 Q_1d = -((FdCroFdCroSd_D * norm_FdCroFdCroSd - FdCroFdCroSd * norm_FdCroFdCroSdD) /
                  (norm_FdCroFdCroSd * norm_FdCroFdCroSd));
    Vec3 Q_2d = ((FdCrosSd_D * norm_FdCrosSd - FdCrosSd * norm_FdCrosSdD) / (norm_FdCrosSd * norm_FdCrosSd));
    Vec3 Q_3d = ((Fd_D * norm_Fd - F_d * norm_FdD) / (norm_Fd * norm_Fd));
    Mat33 QD;
    QD.col(0) = Q_1d;
    QD.col(1) = Q_2d;
    QD.col(2) = Q_3d;

    double norm_FdDD = ((Fd_DD.dot(F_d) + Fd_D.dot(Fd_D)) * norm_Fd - Fd_D.dot(F_d) * norm_FdD) / (norm_Fd * norm_Fd);
    Vec3 FdCrosSd_DD = Fd_DD.cross(sd);
    double norm_FdcroSdDD = ((FdCrosSd_DD.dot(FdCrosSd) + FdCrosSd_D.dot(FdCrosSd_D)) * norm_FdCrosSd -
                             FdCrosSd_D.dot(FdCrosSd) * norm_FdCrosSdD) /
                            (norm_FdCrosSd * norm_FdCrosSd);
    Vec3 FdCroFdCroSd_DD = 2 * Fd_D.cross(Fd_D.cross(sd)) + Fd_DD.cross(FdCrosSd) + F_d.cross(FdCrosSd_DD);
    double norm_FdCroFdCroSdDD =
        ((FdCroFdCroSd_DD.dot(FdCroFdCroSd) + FdCroFdCroSd_D.dot(FdCroFdCroSd_D)) * norm_FdCroFdCroSd -
         FdCroFdCroSd_D.dot(FdCroFdCroSd) * norm_FdCroFdCroSdD) /
        (norm_FdCroFdCroSd * norm_FdCroFdCroSd);

    Vec3 Q_1dd = -(FdCroFdCroSd_DD * norm_FdCroFdCroSd - FdCroFdCroSd * norm_FdCroFdCroSdDD) /
                     (norm_FdCroFdCroSd * norm_FdCroFdCroSd) +
                 2 * ((FdCroFdCroSd_D * norm_FdCroFdCroSd - FdCroFdCroSd * norm_FdCroFdCroSdD) * norm_FdCroFdCroSdD) /
                     (norm_FdCroFdCroSd * norm_FdCroFdCroSd * norm_FdCroFdCroSd);

    Vec3 Q_2dd = (FdCrosSd_DD * norm_FdCrosSd - FdCrosSd * norm_FdcroSdDD) / (norm_FdCrosSd * norm_FdCrosSd) +
                 2 * ((FdCrosSd_D * norm_FdCrosSd - FdCrosSd * norm_FdCrosSdD) * norm_FdCrosSdD) /
                     (norm_FdCrosSd * norm_FdCrosSd * norm_FdCrosSd);

    Vec3 Q_3dd = (Fd_DD * norm_Fd - F_d * norm_FdDD) / (norm_Fd * norm_Fd) +
                 2 * ((Fd_D * norm_Fd - F_d * norm_FdD) * norm_FdD) / (norm_Fd * norm_Fd * norm_Fd);
    Mat33 QDD;
    QDD.col(0) = Q_1dd;
    QDD.col(1) = Q_2dd;
    QDD.col(2) = Q_3dd;

    UAVC.relative_desired_PayloadVector =
        Vec3(UAVC.relative_desired_PayloadVector_x, 0,
             -sqrt(pow(physics.length, 2) - pow(UAVC.relative_desired_PayloadVector_x, 2)));
    UAVC.relative_desired_PayloadOrientation = UAVC.relative_desired_PayloadVector.normalized();
    UAVC.desired_PayloadVector = Q * UAVC.relative_desired_PayloadVector;
    UAVC.desired_PayloadOrientation = Q * UAVC.relative_desired_PayloadOrientation;

    UAVC1.relative_desired_PayloadVector =
        Vec3(UAVC1.relative_desired_PayloadVector_x, 0,
             -sqrt(pow(physics1.length, 2) - pow(UAVC1.relative_desired_PayloadVector_x, 2)));
    UAVC1.relative_desired_PayloadOrientation = UAVC1.relative_desired_PayloadVector.normalized();
    UAVC1.desired_PayloadVector = Q * UAVC1.relative_desired_PayloadVector;
    UAVC1.desired_PayloadOrientation = Q * UAVC1.relative_desired_PayloadOrientation;

    UAVC.cPayloadOrientation = calculate.cPayloadOrientation;
    UAVC.cPayloadOrientationD = calculate.cPayloadOrientationD;
    UAVC.desired_PayloadOrientationD = QD * UAVC.relative_desired_PayloadOrientation;
    UAVC.desired_PayloadOrientationDD = QDD * UAVC.relative_desired_PayloadOrientation;
    UAVC.cPayloadOrientationW = UAVC.cPayloadOrientation.cross(UAVC.cPayloadOrientationD);
    UAVC.desired_PayloadOrientationW = UAVC.desired_PayloadOrientation.cross(UAVC.desired_PayloadOrientationD);
    UAVC.desired_PayloadOrientationWD = UAVC.desired_PayloadOrientation.cross(UAVC.desired_PayloadOrientationDD);

    UAVC1.cPayloadOrientation = calculate1.cPayloadOrientation;
    UAVC1.cPayloadOrientationD = calculate1.cPayloadOrientationD;
    UAVC1.desired_PayloadOrientationD = QD * UAVC1.relative_desired_PayloadOrientation;
    UAVC1.desired_PayloadOrientationDD = QDD * UAVC1.relative_desired_PayloadOrientation;
    UAVC1.cPayloadOrientationW = UAVC1.cPayloadOrientation.cross(UAVC1.cPayloadOrientationD);
    UAVC1.desired_PayloadOrientationW = UAVC1.desired_PayloadOrientation.cross(UAVC1.desired_PayloadOrientationD);
    UAVC1.desired_PayloadOrientationWD = UAVC1.desired_PayloadOrientation.cross(UAVC1.desired_PayloadOrientationDD);

    desired.cBodyPosition = desired.cPayloadPosition - UAVC.desired_PayloadVector;

    desired.cBodyVelocity = desired.cPayloadVelocity;

    desired1.cBodyPosition = desired1.cPayloadPosition - UAVC1.desired_PayloadVector;

    desired1.cBodyVelocity = desired1.cPayloadVelocity;

    UAVC.errorBodyPos = feedback.cBodyPosition - desired.cBodyPosition;
    UAVC.errorBodyVel = feedback.cBodyVelocity - desired.cBodyVelocity;
    UAVC.accumulateError += UAVC.errorBodyPos;
    UAVC1.errorBodyPos = feedback1.cBodyPosition - desired1.cBodyPosition;
    UAVC1.errorBodyVel = feedback1.cBodyVelocity - desired1.cBodyVelocity;
    UAVC1.accumulateError += UAVC1.errorBodyPos;

    UAVC.errorOrientation = UAVC.desired_PayloadOrientation.cross(UAVC.cPayloadOrientation);

    UAVC1.errorOrientation = UAVC1.desired_PayloadOrientation.cross(UAVC1.cPayloadOrientation);

    UAVC.errorOrientationD =
        UAVC.cPayloadOrientationW +
        UAVC.cPayloadOrientation.cross(UAVC.cPayloadOrientation.cross(UAVC.desired_PayloadOrientationW));

    UAVC1.errorOrientationD =
        UAVC1.cPayloadOrientationW +
        UAVC1.cPayloadOrientation.cross(UAVC1.cPayloadOrientation.cross(UAVC1.desired_PayloadOrientationW));

    UAVC.miu = -F_d.dot(UAVC.cPayloadOrientation) * UAVC.cPayloadOrientation /
               ((UAVC.relative_desired_PayloadOrientation.dot(e3)) *
                (UAVC.relative_desired_PayloadOrientation + UAVC1.relative_desired_PayloadOrientation).norm());
    UAVC1.miu = -F_d.dot(UAVC1.cPayloadOrientation) * UAVC1.cPayloadOrientation /
                ((UAVC1.relative_desired_PayloadOrientation.dot(e3)) *
                 (UAVC.relative_desired_PayloadOrientation + UAVC1.relative_desired_PayloadOrientation).norm());

    UAVC.u_parallel = physics.mass * physics.length * UAVC.cPayloadOrientationW.norm() *
                          UAVC.cPayloadOrientationW.norm() * UAVC.cPayloadOrientation +
                      UAVC.miu;
    UAVC.u_vertical =
        physics.mass * physics.length *
        UAVC.cPayloadOrientation.cross(
            -UAVC.errorOrientation.cwiseProduct(UAVC.k_q) - UAVC.errorOrientationD.cwiseProduct(UAVC.k_w) -
            UAVC.cPayloadOrientation.dot(UAVC.desired_PayloadOrientationW) * UAVC.cPayloadOrientationD -
            UAVC.cPayloadOrientation.cross(UAVC.cPayloadOrientation.cross(UAVC.desired_PayloadOrientationWD)));

    UAVC.F_n = -UAVC.errorBodyPos.cwiseProduct(UAVC.k_bx) - UAVC.errorBodyVel.cwiseProduct(UAVC.k_bv) +
               (physics.mass) * desired.cPayloadAcceleration + (physics.mass) * (e3 * 9.8);
    UAVC.F = UAVC.u_parallel + UAVC.u_vertical + UAVC.F_n;

    UAVC1.u_parallel = physics1.mass * physics1.length * UAVC1.cPayloadOrientationW.norm() *
                           UAVC1.cPayloadOrientationW.norm() * UAVC1.cPayloadOrientation +
                       UAVC1.miu;
    UAVC1.u_vertical =
        physics1.mass * physics1.length *
        UAVC1.cPayloadOrientation.cross(
            -UAVC1.errorOrientation.cwiseProduct(UAVC1.k_q) - UAVC1.errorOrientationD.cwiseProduct(UAVC1.k_w) -
            UAVC1.cPayloadOrientation.dot(UAVC1.desired_PayloadOrientationW) * UAVC1.cPayloadOrientationD -
            UAVC1.cPayloadOrientation.cross(UAVC1.cPayloadOrientation.cross(UAVC1.desired_PayloadOrientationWD)));
    UAVC1.F_n = -UAVC1.errorBodyPos.cwiseProduct(UAVC1.k_bx) - UAVC1.errorBodyVel.cwiseProduct(UAVC1.k_bv) +
                (physics1.mass) * desired1.cPayloadAcceleration + (physics1.mass) * (e3 * 9.8);
    UAVC1.F = UAVC1.u_parallel + UAVC1.u_vertical + UAVC1.F_n;

    UAVC.b1_des = Vec3(cos(0), sin(0), 0);
    UAVC.b3_des = UAVC.F / UAVC.F.norm();
    UAVC.b2_des = UAVC.b3_des.cross(UAVC.b1_des);
    UAVC.b2_des /= UAVC.b2_des.norm();

    UAVC.desiredRotationMatrix;
    UAVC.desiredRotationMatrix.col(0) = UAVC.b2_des.cross(UAVC.b3_des);
    UAVC.desiredRotationMatrix.col(1) = UAVC.b2_des;
    UAVC.desiredRotationMatrix.col(2) = UAVC.b3_des;

    UAVC.eulerAngle = UAVC.desiredRotationMatrix.eulerAngles(0, 1, 2);

    output.quaternion =
        tf::createQuaternionMsgFromRollPitchYaw(UAVC.eulerAngle[0], UAVC.eulerAngle[1], UAVC.eulerAngle[2]);

    desiredPos.header.frame_id = "map";
    desiredPos.header.stamp = ros::Time::now();

    desiredPos.pose.pose.position.x = desired.cBodyPosition.x();
    desiredPos.pose.pose.position.y = desired.cBodyPosition.y();
    desiredPos.pose.pose.position.z = desired.cBodyPosition.z();
    desiredPos.pose.pose.orientation = output.quaternion;

    desiredPos.twist.twist.linear.x = desired.cBodyVelocity.x();
    desiredPos.twist.twist.linear.y = desired.cBodyVelocity.y();
    desiredPos.twist.twist.linear.z = desired.cBodyVelocity.z();

    UAVC.errorRotation =
        0.5 * math.antisymmetricMatrixToVector((UAVC.desiredRotationMatrix.transpose() * feedback.cBodyRotationMatrix -
                                                feedback.cBodyRotationMatrix.transpose() * UAVC.desiredRotationMatrix));
    UAVC.errorAngular = feedback.cBodyAngularVelocity;

    output.M = -UAVC.errorRotation.cwiseProduct(UAVC.k_R) + UAVC.errorAngular.cwiseProduct(UAVC.k_Omega);

    output.thrust = UAVC.b3_des.transpose() * UAVC.F;

    UAVC1.b1_des = Vec3(cos(0), sin(0), 0);
    UAVC1.b3_des = UAVC1.F / UAVC1.F.norm();

    UAVC1.b2_des = UAVC1.b3_des.cross(UAVC1.b1_des);
    UAVC1.b2_des /= UAVC1.b2_des.norm();

    UAVC1.desiredRotationMatrix.col(0) = UAVC1.b2_des.cross(UAVC1.b3_des);
    UAVC1.desiredRotationMatrix.col(1) = UAVC1.b2_des;
    UAVC1.desiredRotationMatrix.col(2) = UAVC1.b3_des;

    UAVC1.eulerAngle = UAVC1.desiredRotationMatrix.eulerAngles(0, 1, 2);

    output1.quaternion =
        tf::createQuaternionMsgFromRollPitchYaw(UAVC1.eulerAngle[0], UAVC1.eulerAngle[1], UAVC1.eulerAngle[2]);

    desiredPos1.header.frame_id = "map";
    desiredPos1.header.stamp = ros::Time::now();

    desiredPos1.pose.pose.position.x = desired1.cBodyPosition.x();
    desiredPos1.pose.pose.position.y = desired1.cBodyPosition.y();
    desiredPos1.pose.pose.position.z = desired1.cBodyPosition.z();
    desiredPos1.pose.pose.orientation = output1.quaternion;

    desiredPos1.twist.twist.linear.x = desired1.cBodyVelocity.x();
    desiredPos1.twist.twist.linear.y = desired1.cBodyVelocity.y();
    desiredPos1.twist.twist.linear.z = desired1.cBodyVelocity.z();

    UAVC1.errorRotation = 0.5 * math.antisymmetricMatrixToVector(
                                    (UAVC1.desiredRotationMatrix.transpose() * feedback1.cBodyRotationMatrix -
                                     feedback1.cBodyRotationMatrix.transpose() * UAVC1.desiredRotationMatrix));
    UAVC1.errorAngular = feedback1.cBodyAngularVelocity;

    output1.M = -UAVC1.errorRotation.cwiseProduct(UAVC1.k_R) + UAVC1.errorAngular.cwiseProduct(UAVC1.k_Omega);

    output1.thrust = UAVC1.b3_des.transpose() * UAVC1.F;
#ifdef VREP
    propellerController();
    propellerController1();
#endif
}

void PayloadController::Transition_Controller(const Vec3 &desiredU1Pos, const Vec3 &desiredU1Vel,
                                              const Vec3 &desiredU1Acc, const Vec3 &desiredU2Pos,
                                              const Vec3 &desiredU2Vel, const Vec3 &desiredU2Acc,
                                              const Vec3 &payloadPos)
{
    desired.cBodyPosition = desiredU1Pos;
    desired.cBodyVelocity = desiredU1Vel;
    desired.cBodyAcc = desiredU1Acc;
    desired.cPayloadPosition = payloadPos;
    desired.cPayloadVelocity << 0, 0, 0;
    desired.cPayloadAcceleration << 0, 0, 0;
    desired.cPayloadJerk << 0, 0, 0;
    desired.cPayloadSnap << 0, 0, 0;

    desired1.cBodyPosition = desiredU2Pos;
    desired1.cBodyVelocity = desiredU2Vel;
    desired1.cBodyAcc = desiredU2Acc;
    desired1.cPayloadPosition = payloadPos;
    desired1.cPayloadVelocity << 0, 0, 0;
    desired1.cPayloadAcceleration << 0, 0, 0;
    desired1.cPayloadJerk << 0, 0, 0;
    desired1.cPayloadSnap << 0, 0, 0;

    Vec3 F_d;
    Vec3 Fd_D;
    Vec3 Fd_DD;

    Vec3 errorPayloadPosition = Vec3(0, 0, 0);
    Vec3 errorPayloadVelocity = Vec3(0, 0, 0);
    Vec3 errorPayloadAcc = Vec3(0, 0, 0);
    Vec3 errorPayloadJerk = Vec3(0, 0, 0);
    F_d = -errorPayloadPosition.cwiseProduct(ky_p) - errorPayloadVelocity.cwiseProduct(ky_d) +
          physics_payload.mass * (desired.cPayloadAcceleration + e3 * 9.8);
    Fd_D = -errorPayloadVelocity.cwiseProduct(ky_p) - errorPayloadAcc.cwiseProduct(ky_d) +
           physics_payload.mass * (desired.cPayloadJerk);
    Fd_DD = -errorPayloadAcc.cwiseProduct(ky_p) - errorPayloadAcc.cwiseProduct(ky_d) +
            physics_payload.mass * (desired.cPayloadSnap);

    UAVC.relative_desired_PayloadVector =
        Vec3(UAVC.relative_desired_PayloadVector_x, 0,
             -sqrt(pow(physics.length, 2) - pow(UAVC.relative_desired_PayloadVector_x, 2)));
    UAVC.relative_desired_PayloadOrientation = UAVC.relative_desired_PayloadVector.normalized();
    UAVC.desired_PayloadVector = -(desiredU1Pos - payloadPos);
    UAVC.desired_PayloadOrientation = UAVC.desired_PayloadVector.normalized();

    UAVC1.relative_desired_PayloadVector =
        Vec3(UAVC1.relative_desired_PayloadVector_x, 0,
             -sqrt(pow(physics1.length, 2) - pow(UAVC1.relative_desired_PayloadVector_x, 2)));
    UAVC1.relative_desired_PayloadOrientation = UAVC1.relative_desired_PayloadVector.normalized();
    UAVC1.desired_PayloadVector = -(desiredU2Pos - payloadPos);
    UAVC1.desired_PayloadOrientation = UAVC1.desired_PayloadVector.normalized();

    UAVC.cPayloadOrientation = calculate.cPayloadOrientation;
    UAVC.cPayloadOrientationD = calculate.cPayloadOrientationD;
    UAVC.desired_PayloadOrientationD = -desiredU1Vel / physics.length;
    UAVC.desired_PayloadOrientationDD = -desiredU1Acc / physics.length;
    UAVC.cPayloadOrientationW = UAVC.cPayloadOrientation.cross(UAVC.cPayloadOrientationD);
    UAVC.desired_PayloadOrientationW = UAVC.desired_PayloadOrientation.cross(UAVC.desired_PayloadOrientationD);
    UAVC.desired_PayloadOrientationWD = UAVC.desired_PayloadOrientation.cross(UAVC.desired_PayloadOrientationDD);

    UAVC1.cPayloadOrientation = calculate1.cPayloadOrientation;
    UAVC1.cPayloadOrientationD = calculate1.cPayloadOrientationD;
    UAVC1.desired_PayloadOrientationD = -desiredU2Vel / physics1.length;
    UAVC1.desired_PayloadOrientationDD = -desiredU2Acc / physics1.length;
    UAVC1.cPayloadOrientationW = UAVC1.cPayloadOrientation.cross(UAVC1.cPayloadOrientationD);
    UAVC1.desired_PayloadOrientationW = UAVC1.desired_PayloadOrientation.cross(UAVC1.desired_PayloadOrientationD);
    UAVC1.desired_PayloadOrientationWD = UAVC1.desired_PayloadOrientation.cross(UAVC1.desired_PayloadOrientationDD);

    UAVC.errorBodyPos = feedback.cBodyPosition - desired.cBodyPosition;
    UAVC.errorBodyVel = feedback.cBodyVelocity - desired.cBodyVelocity;
    UAVC.accumulateError += UAVC.errorBodyPos;
    UAVC1.errorBodyPos = feedback1.cBodyPosition - desired1.cBodyPosition;
    UAVC1.errorBodyVel = feedback1.cBodyVelocity - desired1.cBodyVelocity;
    UAVC1.accumulateError += UAVC1.errorBodyPos;

    UAVC.errorOrientation = UAVC.desired_PayloadOrientation.cross(UAVC.cPayloadOrientation);

    UAVC1.errorOrientation = UAVC1.desired_PayloadOrientation.cross(UAVC1.cPayloadOrientation);

    UAVC.errorOrientationD =
        UAVC.cPayloadOrientationW +
        UAVC.cPayloadOrientation.cross(UAVC.cPayloadOrientation.cross(UAVC.desired_PayloadOrientationW));

    UAVC1.errorOrientationD =
        UAVC1.cPayloadOrientationW +
        UAVC1.cPayloadOrientation.cross(UAVC1.cPayloadOrientation.cross(UAVC1.desired_PayloadOrientationW));

    Eigen::Matrix<double, 3, 2> A;
    A.col(0) = UAVC.desired_PayloadVector;
    A.col(1) = UAVC1.desired_PayloadVector;

    Vec2 miu_k = (A.transpose() * A).inverse() * A.transpose() * F_d;

    UAVC.miu = miu_k(0) * UAVC.desired_PayloadVector;
    UAVC1.miu = miu_k(1) * UAVC1.desired_PayloadVector;

    UAVC.miu = -F_d.dot(UAVC.cPayloadOrientation) * UAVC.cPayloadOrientation /
               ((UAVC.relative_desired_PayloadOrientation.dot(e3)) *
                (UAVC.relative_desired_PayloadOrientation + UAVC1.relative_desired_PayloadOrientation).norm());
    UAVC1.miu = -F_d.dot(UAVC1.cPayloadOrientation) * UAVC1.cPayloadOrientation /
                ((UAVC1.relative_desired_PayloadOrientation.dot(e3)) *
                 (UAVC.relative_desired_PayloadOrientation + UAVC1.relative_desired_PayloadOrientation).norm());

    UAVC.u_parallel = physics.mass * physics.length * UAVC.cPayloadOrientationW.norm() *
                          UAVC.cPayloadOrientationW.norm() * UAVC.cPayloadOrientation +
                      UAVC.miu;

    UAVC.u_vertical =
        physics.mass * physics.length *
        UAVC.cPayloadOrientation.cross(
            -UAVC.errorOrientation.cwiseProduct(UAVC.Xk_q) - UAVC.errorOrientationD.cwiseProduct(UAVC.Xk_w) -
            UAVC.cPayloadOrientation.dot(UAVC.desired_PayloadOrientationW) * UAVC.cPayloadOrientationD -
            UAVC.cPayloadOrientation.cross(UAVC.cPayloadOrientation.cross(UAVC.desired_PayloadOrientationWD)));

    UAVC.F_n = -UAVC.errorBodyPos.cwiseProduct(UAVC.Xk_bx) - UAVC.errorBodyVel.cwiseProduct(UAVC.Xk_bv) +
               physics.mass * desiredU1Acc + (physics.mass) * (e3 * 9.8);
    UAVC.F = UAVC.u_parallel + UAVC.u_vertical + UAVC.F_n;

    UAVC1.u_parallel = physics1.mass * physics1.length * UAVC1.cPayloadOrientationW.norm() *
                           UAVC1.cPayloadOrientationW.norm() * UAVC1.cPayloadOrientation +
                       UAVC1.miu;
    UAVC1.u_vertical =
        physics1.mass * physics1.length *
        UAVC1.cPayloadOrientation.cross(
            -UAVC1.errorOrientation.cwiseProduct(UAVC1.Xk_q) - UAVC1.errorOrientationD.cwiseProduct(UAVC1.Xk_w) -
            UAVC1.cPayloadOrientation.dot(UAVC1.desired_PayloadOrientationW) * UAVC1.cPayloadOrientationD -
            UAVC1.cPayloadOrientation.cross(UAVC1.cPayloadOrientation.cross(UAVC1.desired_PayloadOrientationWD)));

    UAVC1.F_n = -UAVC1.errorBodyPos.cwiseProduct(UAVC1.Xk_bx) - UAVC1.errorBodyVel.cwiseProduct(UAVC1.Xk_bv) +
                physics1.mass * desiredU2Acc + (physics1.mass) * (e3 * 9.8);
    UAVC1.F = UAVC1.u_parallel + UAVC1.u_vertical + UAVC1.F_n;

    UAVC.b1_des = Vec3(cos(0), sin(0), 0);

    UAVC.b3_des = UAVC.F / UAVC.F.norm();

    UAVC.b2_des = UAVC.b3_des.cross(UAVC.b1_des);
    UAVC.b2_des /= UAVC.b2_des.norm();

    UAVC.desiredRotationMatrix;
    UAVC.desiredRotationMatrix.col(0) = UAVC.b2_des.cross(UAVC.b3_des);
    UAVC.desiredRotationMatrix.col(1) = UAVC.b2_des;
    UAVC.desiredRotationMatrix.col(2) = UAVC.b3_des;

    UAVC.eulerAngle = UAVC.desiredRotationMatrix.eulerAngles(0, 1, 2);

    output.quaternion =
        tf::createQuaternionMsgFromRollPitchYaw(UAVC.eulerAngle[0], UAVC.eulerAngle[1], UAVC.eulerAngle[2]);

    desiredPos.header.frame_id = "map";
    desiredPos.header.stamp = ros::Time::now();

    desiredPos.pose.pose.position.x = desired.cBodyPosition.x();
    desiredPos.pose.pose.position.y = desired.cBodyPosition.y();
    desiredPos.pose.pose.position.z = desired.cBodyPosition.z();
    desiredPos.pose.pose.orientation = output.quaternion;

    desiredPos.twist.twist.linear.x = desired.cBodyVelocity.x();
    desiredPos.twist.twist.linear.y = desired.cBodyVelocity.y();
    desiredPos.twist.twist.linear.z = desired.cBodyVelocity.z();

    UAVC.errorRotation =
        0.5 * math.antisymmetricMatrixToVector((UAVC.desiredRotationMatrix.transpose() * feedback.cBodyRotationMatrix -
                                                feedback.cBodyRotationMatrix.transpose() * UAVC.desiredRotationMatrix));

    UAVC.errorAngular = feedback.cBodyAngularVelocity;

    output.M = -UAVC.errorRotation.cwiseProduct(UAVC.Xk_R) + UAVC.errorAngular.cwiseProduct(UAVC.Xk_Omega);

    output.thrust = UAVC.b3_des.transpose() * UAVC.F;

    UAVC1.b1_des = Vec3(cos(0), sin(0), 0);

    UAVC1.b3_des = UAVC1.F / UAVC1.F.norm();

    UAVC1.b2_des = UAVC1.b3_des.cross(UAVC1.b1_des);
    UAVC1.b2_des /= UAVC1.b2_des.norm();

    UAVC1.desiredRotationMatrix.col(0) = UAVC1.b2_des.cross(UAVC1.b3_des);
    UAVC1.desiredRotationMatrix.col(1) = UAVC1.b2_des;
    UAVC1.desiredRotationMatrix.col(2) = UAVC1.b3_des;

    UAVC1.eulerAngle = UAVC1.desiredRotationMatrix.eulerAngles(0, 1, 2);

    output1.quaternion =
        tf::createQuaternionMsgFromRollPitchYaw(UAVC1.eulerAngle[0], UAVC1.eulerAngle[1], UAVC1.eulerAngle[2]);

    desiredPos1.header.frame_id = "map";
    desiredPos1.header.stamp = ros::Time::now();

    desiredPos1.pose.pose.position.x = desired1.cBodyPosition.x();
    desiredPos1.pose.pose.position.y = desired1.cBodyPosition.y();
    desiredPos1.pose.pose.position.z = desired1.cBodyPosition.z();
    desiredPos1.pose.pose.orientation = output1.quaternion;

    desiredPos1.twist.twist.linear.x = desired1.cBodyVelocity.x();
    desiredPos1.twist.twist.linear.y = desired1.cBodyVelocity.y();
    desiredPos1.twist.twist.linear.z = desired1.cBodyVelocity.z();

    UAVC1.errorRotation = 0.5 * math.antisymmetricMatrixToVector(
                                    (UAVC1.desiredRotationMatrix.transpose() * feedback1.cBodyRotationMatrix -
                                     feedback1.cBodyRotationMatrix.transpose() * UAVC1.desiredRotationMatrix));

    UAVC1.errorAngular = feedback1.cBodyAngularVelocity;

    output1.M = -UAVC1.errorRotation.cwiseProduct(UAVC1.Xk_R) + UAVC1.errorAngular.cwiseProduct(UAVC1.Xk_Omega);

    output1.thrust = UAVC1.b3_des.transpose() * UAVC1.F;
#ifdef VREP
    propellerController();
    propellerController1();
#endif
}

void Transition_Trajectory_Generator(const std::vector<double> &temp_txt,
                                     std::vector<TrajectoryGeneratorWaypoint> &vect_Traj_UAVT1,
                                     std::vector<TrajectoryGeneratorWaypoint> &vect_Traj_UAVT2,
                                     std::vector<bool> &vect_T_Traj_lable, const double payload_quadrotor_Vector_x_1,
                                     const double payload_quadrotor_Vector_x_2, const double length,
                                     const double length1, const double TVel, const double TAcc, const int Torder)
{
    double T_Traj_limit = PI / 12;
    TrajectoryGeneratorWaypoint TrajectoryGeneratorUseless(TVel, TAcc, Torder);
    for (int i = 0; i < temp_txt.size() / 3 - 1; i++)
    {
        if (i == 0)
        {
            double theta1 = 1.0 / 2 * PI;
            Vec2 direc2;
            direc2 << temp_txt[(i + 1) * 3] - temp_txt[i * 3], temp_txt[(i + 1) * 3 + 1] - temp_txt[i * 3 + 1];
            Vec2 refer;
            refer << 1, 0;
            double theta2 = acos(direc2.dot(refer) / direc2.norm());

            if (direc2[1] < 0)
            {
                theta2 = 2 * PI - theta2;
            }

            if (abs(theta2 - theta1) < T_Traj_limit)
            {
                vect_T_Traj_lable.push_back(false);
                vect_Traj_UAVT1.push_back(TrajectoryGeneratorUseless);
                vect_Traj_UAVT2.push_back(TrajectoryGeneratorUseless);
            }
            else
            {
                vect_T_Traj_lable.push_back(true);
                double tmp1 = theta2 - theta1;
                std::vector<double> thetaSet;
                double interval = PI / 12;
                if (tmp1 > 0)
                {
                    double a = 2 * PI + theta1 - theta2;
                    if (a > tmp1)
                    {
                        int num = ceil(tmp1 / interval);
                        for (int j = 0; j <= num; j++)
                        {
                            if (j != num)
                            {
                                thetaSet.push_back(theta1 + j * interval);
                            }
                            else
                            {
                                thetaSet.push_back(theta2);
                            }
                        }
                    }
                    else
                    {
                        int num = ceil(a / interval);
                        for (int j = 0; j <= num; j++)
                        {
                            if (j == 0)
                            {
                                thetaSet.push_back(2 * PI + theta1);
                            }
                            else if (j != num)
                            {
                                thetaSet.push_back(2 * PI + theta1 - j * interval);
                            }
                            else
                            {
                                thetaSet.push_back(theta2);
                            }
                        }
                    }
                }
                else
                {
                    double a = 2 * PI + theta2 - theta1;
                    if (a > -tmp1)
                    {
                        int num = ceil(-tmp1 / interval);
                        for (int j = 0; j <= num; j++)
                        {
                            if (j == num)
                            {
                                thetaSet.push_back(theta2);
                            }
                            else
                            {
                                thetaSet.push_back(theta1 - j * interval);
                            }
                        }
                    }
                    else
                    {
                        int num = ceil(a / interval) + 1;
                        for (int j = 0; j <= num; j++)
                        {
                            if (j != num)
                            {
                                thetaSet.push_back(theta1 + j * interval);
                            }
                            else
                            {
                                thetaSet.push_back(theta2 + 2 * PI);
                            }
                        }
                    }
                }

                if (abs(thetaSet[thetaSet.size() - 1] - thetaSet[thetaSet.size() - 2]) < 1e-3)
                {
                    thetaSet.pop_back();
                }

                Eigen::MatrixXd pointSet1(2, thetaSet.size());
                Eigen::MatrixXd pointSet2(2, thetaSet.size());
                Vec2 center;
                center << temp_txt[i * 3], temp_txt[i * 3 + 1];

                for (int j = 0; j < thetaSet.size(); j++)
                {
                    Vec2 thetaSet_tmp1;
                    thetaSet_tmp1 << payload_quadrotor_Vector_x_1 * cos(thetaSet[j] - PI / 2),
                        payload_quadrotor_Vector_x_1 * sin(thetaSet[j] - PI / 2);
                    Vec2 thetaSet_tmp2;
                    thetaSet_tmp2 << payload_quadrotor_Vector_x_2 * cos(thetaSet[j] + PI / 2),
                        payload_quadrotor_Vector_x_2 * sin(thetaSet[j] + PI / 2);
                    pointSet1.col(j) = center + thetaSet_tmp1;
                    pointSet2.col(j) = center + thetaSet_tmp2;
                }

                TrajectoryGeneratorWaypoint TrajectoryGenerator1(TVel, TAcc, Torder);

                nav_msgs::Path waypoints1;
                waypoints1.header.frame_id = std::string("world");
                waypoints1.header.stamp = ros::Time::now();

                geometry_msgs::PoseStamped temp_wpt1;
                temp_wpt1.header.frame_id = "map";
                temp_wpt1.pose.orientation.w = 1;
                temp_wpt1.pose.orientation.z = 0;
                temp_wpt1.pose.orientation.y = 0;
                temp_wpt1.pose.orientation.x = 0;

                for (int j = 0; j < thetaSet.size(); j++)
                {
                    temp_wpt1.header.stamp = ros::Time::now();
                    temp_wpt1.pose.position.x = pointSet1(0, j);
                    temp_wpt1.pose.position.y = pointSet1(1, j);
                    temp_wpt1.pose.position.z =
                        temp_txt[(i)*3 + 2] + sqrt(pow(length, 2) - pow(payload_quadrotor_Vector_x_1, 2));
                    waypoints1.poses.push_back(temp_wpt1);
                }

                Eigen::MatrixXd margin_constraint1(2, 6);
                margin_constraint1 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

                TrajectoryGenerator1.isTraj = TrajectoryGenerator1.trajGeneration(waypoints1, margin_constraint1);

                if (TrajectoryGenerator1.isTraj)
                {
                    vect_Traj_UAVT1.push_back(TrajectoryGenerator1);
                }
                TrajectoryGeneratorWaypoint TrajectoryGenerator2(TVel, TAcc, Torder);

                nav_msgs::Path waypoints2;
                waypoints2.header.frame_id = std::string("world");
                waypoints2.header.stamp = ros::Time::now();

                geometry_msgs::PoseStamped temp_wpt2;
                temp_wpt2.header.frame_id = "map";
                temp_wpt2.pose.orientation.w = 1;
                temp_wpt2.pose.orientation.z = 0;
                temp_wpt2.pose.orientation.y = 0;
                temp_wpt2.pose.orientation.x = 0;

                for (int j = 0; j < thetaSet.size(); j++)
                {
                    temp_wpt2.header.stamp = ros::Time::now();
                    temp_wpt2.pose.position.x = pointSet2(0, j);
                    temp_wpt2.pose.position.y = pointSet2(1, j);
                    temp_wpt2.pose.position.z =
                        temp_txt[(i)*3 + 2] + sqrt(pow(length, 2) - pow(payload_quadrotor_Vector_x_2, 2));
                    waypoints2.poses.push_back(temp_wpt2);
                }

                Eigen::MatrixXd margin_constraint2(2, 6);
                margin_constraint2 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

                TrajectoryGenerator2.isTraj = TrajectoryGenerator2.trajGeneration(waypoints2, margin_constraint2);

                if (TrajectoryGenerator2.isTraj)
                {
                    vect_Traj_UAVT2.push_back(TrajectoryGenerator2);
                }
            }
        }
        if (i != temp_txt.size() / 3 - 2)
        {
            Vec2 direc1;
            direc1 << temp_txt[(i + 1) * 3] - temp_txt[i * 3], temp_txt[(i + 1) * 3 + 1] - temp_txt[i * 3 + 1];
            Vec2 direc2;
            direc2 << temp_txt[(i + 2) * 3] - temp_txt[(i + 1) * 3],
                temp_txt[(i + 2) * 3 + 1] - temp_txt[(i + 1) * 3 + 1];
            Vec2 refer;
            refer << 1, 0;
            double theta1 = acos(direc1.dot(refer) / direc1.norm());
            double theta2 = acos(direc2.dot(refer) / direc2.norm());

            if (direc1[1] < 0)
            {
                theta1 = 2 * PI - theta1;
            }
            if (direc2[1] < 0)
            {
                theta2 = 2 * PI - theta2;
            }
            if (abs(theta2 - theta1) < T_Traj_limit)
            {
                vect_T_Traj_lable.push_back(false);
                vect_Traj_UAVT1.push_back(TrajectoryGeneratorUseless);
                vect_Traj_UAVT2.push_back(TrajectoryGeneratorUseless);
            }
            else
            {
                vect_T_Traj_lable.push_back(true);

                double tmp1 = theta2 - theta1;
                std::vector<double> thetaSet;
                double interval = PI / 12;
                if (tmp1 > 0)
                {
                    double a = 2 * PI + theta1 - theta2;
                    if (a > tmp1)
                    {
                        int num = ceil(tmp1 / interval);
                        for (int j = 0; j <= num; j++)
                        {
                            if (j != num)
                            {
                                thetaSet.push_back(theta1 + j * interval);
                            }
                            else
                            {
                                thetaSet.push_back(theta2);
                            }
                        }
                    }
                    else
                    {
                        int num = ceil(a / interval);
                        for (int j = 0; j <= num; j++)
                        {
                            if (j == 0)
                            {
                                thetaSet.push_back(2 * PI + theta1);
                            }
                            else if (j != num)
                            {
                                thetaSet.push_back(2 * PI + theta1 - j * interval);
                            }
                            else
                            {
                                thetaSet.push_back(theta2);
                            }
                        }
                    }
                }
                else
                {
                    double a = 2 * PI + theta2 - theta1;
                    if (a > -tmp1)
                    {
                        int num = ceil(-tmp1 / interval) + 1;
                        for (int j = 0; j <= num; j++)
                        {
                            if (j == num)
                            {
                                thetaSet.push_back(theta2);
                            }
                            else
                            {
                                thetaSet.push_back(theta1 - j * interval);
                            }
                        }
                    }
                    else
                    {
                        int num = ceil(a / interval);
                        for (int j = 0; j <= num; j++)
                        {
                            if (j != num)
                            {
                                thetaSet.push_back(theta1 + j * interval);
                            }
                            else
                            {
                                thetaSet.push_back(theta2 + 2 * PI);
                            }
                        }
                    }
                }
                if (abs(thetaSet[thetaSet.size() - 1] - thetaSet[thetaSet.size() - 2]) < 1e-3)
                {
                    thetaSet.pop_back();
                }

                Eigen::MatrixXd pointSet1(2, thetaSet.size());
                Eigen::MatrixXd pointSet2(2, thetaSet.size());
                Vec2 center;
                center << temp_txt[(i + 1) * 3], temp_txt[(i + 1) * 3 + 1];

                for (int j = 0; j < thetaSet.size(); j++)
                {
                    Vec2 thetaSet_tmp1;
                    thetaSet_tmp1 << payload_quadrotor_Vector_x_1 * cos(thetaSet[j] - PI / 2),
                        payload_quadrotor_Vector_x_1 * sin(thetaSet[j] - PI / 2);
                    Vec2 thetaSet_tmp2;
                    thetaSet_tmp2 << payload_quadrotor_Vector_x_2 * cos(thetaSet[j] + PI / 2),
                        payload_quadrotor_Vector_x_2 * sin(thetaSet[j] + PI / 2);
                    pointSet1.col(j) = center + thetaSet_tmp1;
                    pointSet2.col(j) = center + thetaSet_tmp2;
                }

                TrajectoryGeneratorWaypoint TrajectoryGenerator1(TVel, TAcc, Torder);

                nav_msgs::Path waypoints1;
                waypoints1.header.frame_id = std::string("world");
                waypoints1.header.stamp = ros::Time::now();

                geometry_msgs::PoseStamped temp_wpt1;
                temp_wpt1.header.frame_id = "map";
                temp_wpt1.pose.orientation.w = 1;
                temp_wpt1.pose.orientation.z = 0;
                temp_wpt1.pose.orientation.y = 0;
                temp_wpt1.pose.orientation.x = 0;

                for (int j = 0; j < thetaSet.size(); j++)
                {
                    temp_wpt1.header.stamp = ros::Time::now();
                    temp_wpt1.pose.position.x = pointSet1(0, j);
                    temp_wpt1.pose.position.y = pointSet1(1, j);
                    temp_wpt1.pose.position.z =
                        temp_txt[(i + 1) * 3 + 2] + sqrt(pow(length, 2) - pow(payload_quadrotor_Vector_x_1, 2));
                    waypoints1.poses.push_back(temp_wpt1);
                }

                Eigen::MatrixXd margin_constraint1(2, 6);
                margin_constraint1 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

                TrajectoryGenerator1.isTraj = TrajectoryGenerator1.trajGeneration(waypoints1, margin_constraint1);

                if (TrajectoryGenerator1.isTraj)
                {
                    vect_Traj_UAVT1.push_back(TrajectoryGenerator1);
                }
                TrajectoryGeneratorWaypoint TrajectoryGenerator2(TVel, TAcc, Torder);

                nav_msgs::Path waypoints2;
                waypoints2.header.frame_id = std::string("world");
                waypoints2.header.stamp = ros::Time::now();

                geometry_msgs::PoseStamped temp_wpt2;
                temp_wpt2.header.frame_id = "map";
                temp_wpt2.pose.orientation.w = 1;
                temp_wpt2.pose.orientation.z = 0;
                temp_wpt2.pose.orientation.y = 0;
                temp_wpt2.pose.orientation.x = 0;

                for (int j = 0; j < thetaSet.size(); j++)
                {
                    temp_wpt2.header.stamp = ros::Time::now();
                    temp_wpt2.pose.position.x = pointSet2(0, j);
                    temp_wpt2.pose.position.y = pointSet2(1, j);
                    temp_wpt2.pose.position.z =
                        temp_txt[(i + 1) * 3 + 2] + sqrt(pow(length, 2) - pow(payload_quadrotor_Vector_x_2, 2));
                    waypoints2.poses.push_back(temp_wpt2);
                }

                Eigen::MatrixXd margin_constraint2(2, 6);
                margin_constraint2 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

                TrajectoryGenerator2.isTraj = TrajectoryGenerator2.trajGeneration(waypoints2, margin_constraint2);

                if (TrajectoryGenerator2.isTraj)
                {
                    vect_Traj_UAVT2.push_back(TrajectoryGenerator2);
                }
            }
        }
    }
}

void Transition_Trajectory_Generator(const geometry_msgs::PoseStamped &start_waypoints_payload,
                                     const nav_msgs::Path &waypoints_payload,
                                     std::vector<TrajectoryGeneratorWaypoint> &vect_Traj_UAVT1,
                                     std::vector<TrajectoryGeneratorWaypoint> &vect_Traj_UAVT2,
                                     std::vector<bool> &vect_T_Traj_lable, const nav_msgs::Path &waypoints_quadrotor_1,
                                     const nav_msgs::Path &waypoints_quadrotor_2, const double length,
                                     const double length1, const double TVel, const double TAcc, const int Torder)
{
    unsigned int T_cnt_traj = 0;
    double T_Traj_limit = PI / 12;
    TrajectoryGeneratorWaypoint TrajectoryGeneratorUseless(TVel, TAcc, Torder);
    double payload_quadrotor_Vector_x_1, payload_quadrotor_Vector_x_2;
    payload_quadrotor_Vector_x_1 =
        sqrt(pow(length, 2) - pow(waypoints_payload.poses[T_cnt_traj].pose.position.z -
                                      waypoints_quadrotor_1.poses[T_cnt_traj].pose.position.z,
                                  2));
    payload_quadrotor_Vector_x_2 =
        sqrt(pow(length1, 2) - pow(waypoints_payload.poses[T_cnt_traj].pose.position.z -
                                       waypoints_quadrotor_2.poses[T_cnt_traj].pose.position.z,
                                   2));
    T_cnt_traj++;
    for (int i = 0; i < waypoints_payload.poses.size() - 1; i++)
    {
        if (i == 0)
        {
            double theta1 = 1.0 / 2 * PI;
            Vec2 direc2;
            direc2 << waypoints_payload.poses[i].pose.position.x - start_waypoints_payload.pose.position.x,
                waypoints_payload.poses[i].pose.position.y - start_waypoints_payload.pose.position.y;
            Vec2 refer;
            refer << 1, 0;
            double theta2 = acos(direc2.dot(refer) / direc2.norm());

            if (direc2[1] < 0)
            {
                theta2 = 2 * PI - theta2;
            }

            if (abs(theta2 - theta1) < T_Traj_limit)
            {
                vect_T_Traj_lable.push_back(false);
                vect_Traj_UAVT1.push_back(TrajectoryGeneratorUseless);
                vect_Traj_UAVT2.push_back(TrajectoryGeneratorUseless);
                payload_quadrotor_Vector_x_1 =
                    sqrt(pow(length, 2) - pow(waypoints_payload.poses[T_cnt_traj].pose.position.z -
                                                  waypoints_quadrotor_1.poses[T_cnt_traj].pose.position.z,
                                              2));
                payload_quadrotor_Vector_x_2 =
                    sqrt(pow(length1, 2) - pow(waypoints_payload.poses[T_cnt_traj].pose.position.z -
                                                   waypoints_quadrotor_2.poses[T_cnt_traj].pose.position.z,
                                               2));
                T_cnt_traj++;
            }
            else
            {
                vect_T_Traj_lable.push_back(true);

                double tmp1 = theta2 - theta1;
                std::vector<double> thetaSet;
                double interval = PI / 12;
                if (tmp1 > 0)
                {
                    double a = 2 * PI + theta1 - theta2;
                    if (a > tmp1)
                    {
                        int num = ceil(tmp1 / interval);
                        for (int j = 0; j <= num; j++)
                        {
                            if (j != num)
                            {
                                thetaSet.push_back(theta1 + j * interval);
                            }
                            else
                            {
                                thetaSet.push_back(theta2);
                            }
                        }
                    }
                    else
                    {
                        int num = ceil(a / interval);
                        for (int j = 0; j <= num; j++)
                        {
                            if (j == 0)
                            {
                                thetaSet.push_back(2 * PI + theta1);
                            }
                            else if (j != num)
                            {
                                thetaSet.push_back(2 * PI + theta1 - j * interval);
                            }
                            else
                            {
                                thetaSet.push_back(theta2);
                            }
                        }
                    }
                }
                else
                {
                    double a = 2 * PI + theta2 - theta1;
                    if (a > -tmp1)
                    {
                        int num = ceil(-tmp1 / interval);
                        for (int j = 0; j <= num; j++)
                        {
                            if (j == num)
                            {
                                thetaSet.push_back(theta2);
                            }
                            else
                            {
                                thetaSet.push_back(theta1 - j * interval);
                            }
                        }
                    }
                    else
                    {
                        int num = ceil(a / interval) + 1;
                        for (int j = 0; j <= num; j++)
                        {
                            if (j != num)
                            {
                                thetaSet.push_back(theta1 + j * interval);
                            }
                            else
                            {
                                thetaSet.push_back(theta2 + 2 * PI);
                            }
                        }
                    }
                }

                if (abs(thetaSet[thetaSet.size() - 1] - thetaSet[thetaSet.size() - 2]) < 1e-3)
                {
                    thetaSet.pop_back();
                }

                Eigen::MatrixXd pointSet1(2, thetaSet.size());
                Eigen::MatrixXd pointSet2(2, thetaSet.size());
                Vec2 center;
                center << start_waypoints_payload.pose.position.x, start_waypoints_payload.pose.position.y;

                for (int j = 0; j < thetaSet.size(); j++)
                {
                    Vec2 thetaSet_tmp1;
                    thetaSet_tmp1 << payload_quadrotor_Vector_x_1 * cos(thetaSet[j] - PI / 2),
                        payload_quadrotor_Vector_x_1 * sin(thetaSet[j] - PI / 2);
                    Vec2 thetaSet_tmp2;
                    thetaSet_tmp2 << payload_quadrotor_Vector_x_2 * cos(thetaSet[j] + PI / 2),
                        payload_quadrotor_Vector_x_2 * sin(thetaSet[j] + PI / 2);
                    pointSet1.col(j) = center + thetaSet_tmp1;
                    pointSet2.col(j) = center + thetaSet_tmp2;
                }

                TrajectoryGeneratorWaypoint TrajectoryGenerator1(TVel, TAcc, Torder);

                nav_msgs::Path waypoints1;
                waypoints1.header.frame_id = std::string("world");
                waypoints1.header.stamp = ros::Time::now();

                geometry_msgs::PoseStamped temp_wpt1;
                temp_wpt1.header.frame_id = "map";
                temp_wpt1.pose.orientation.w = 1;
                temp_wpt1.pose.orientation.z = 0;
                temp_wpt1.pose.orientation.y = 0;
                temp_wpt1.pose.orientation.x = 0;

                for (int j = 0; j < thetaSet.size(); j++)
                {
                    temp_wpt1.header.stamp = ros::Time::now();
                    temp_wpt1.pose.position.x = pointSet1(0, j);
                    temp_wpt1.pose.position.y = pointSet1(1, j);
                    temp_wpt1.pose.position.z = start_waypoints_payload.pose.position.z +
                                                sqrt(pow(length, 2) - pow(payload_quadrotor_Vector_x_1, 2));
                    waypoints1.poses.push_back(temp_wpt1);
                }

                Eigen::MatrixXd margin_constraint1(2, 6);
                margin_constraint1 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

                TrajectoryGenerator1.isTraj = TrajectoryGenerator1.trajGeneration(waypoints1, margin_constraint1);

                if (TrajectoryGenerator1.isTraj)
                {
                    vect_Traj_UAVT1.push_back(TrajectoryGenerator1);
                }
                TrajectoryGeneratorWaypoint TrajectoryGenerator2(TVel, TAcc, Torder);

                nav_msgs::Path waypoints2;
                waypoints2.header.frame_id = std::string("world");
                waypoints2.header.stamp = ros::Time::now();

                geometry_msgs::PoseStamped temp_wpt2;
                temp_wpt2.header.frame_id = "map";
                temp_wpt2.pose.orientation.w = 1;
                temp_wpt2.pose.orientation.z = 0;
                temp_wpt2.pose.orientation.y = 0;
                temp_wpt2.pose.orientation.x = 0;

                for (int j = 0; j < thetaSet.size(); j++)
                {
                    temp_wpt2.header.stamp = ros::Time::now();
                    temp_wpt2.pose.position.x = pointSet2(0, j);
                    temp_wpt2.pose.position.y = pointSet2(1, j);
                    temp_wpt2.pose.position.z = start_waypoints_payload.pose.position.z +
                                                sqrt(pow(length1, 2) - pow(payload_quadrotor_Vector_x_2, 2));
                    waypoints2.poses.push_back(temp_wpt2);
                }

                Eigen::MatrixXd margin_constraint2(2, 6);
                margin_constraint2 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

                TrajectoryGenerator2.isTraj = TrajectoryGenerator2.trajGeneration(waypoints2, margin_constraint2);

                if (TrajectoryGenerator2.isTraj)
                {
                    vect_Traj_UAVT2.push_back(TrajectoryGenerator2);
                }

                payload_quadrotor_Vector_x_1 =
                    sqrt(pow(length, 2) - pow(waypoints_payload.poses[T_cnt_traj].pose.position.z -
                                                  waypoints_quadrotor_1.poses[T_cnt_traj].pose.position.z,
                                              2));
                payload_quadrotor_Vector_x_2 =
                    sqrt(pow(length1, 2) - pow(waypoints_payload.poses[T_cnt_traj].pose.position.z -
                                                   waypoints_quadrotor_2.poses[T_cnt_traj].pose.position.z,
                                               2));
                T_cnt_traj++;
            }
        }

        if (i == 0)
        {
            Vec2 direc1;
            direc1 << waypoints_payload.poses[i].pose.position.x - start_waypoints_payload.pose.position.x,
                waypoints_payload.poses[i].pose.position.y - start_waypoints_payload.pose.position.y;
            Vec2 direc2;
            direc2 << waypoints_payload.poses[i + 1].pose.position.x - waypoints_payload.poses[i].pose.position.x,
                waypoints_payload.poses[i + 1].pose.position.y - waypoints_payload.poses[i].pose.position.y;
            Vec2 refer;
            refer << 1, 0;
            double theta1 = acos(direc1.dot(refer) / direc1.norm());
            double theta2 = acos(direc2.dot(refer) / direc2.norm());

            if (direc1[1] < 0)
            {
                theta1 = 2 * PI - theta1;
            }
            if (direc2[1] < 0)
            {
                theta2 = 2 * PI - theta2;
            }

            if (abs(theta2 - theta1) < T_Traj_limit)
            {
                vect_T_Traj_lable.push_back(false);
                vect_Traj_UAVT1.push_back(TrajectoryGeneratorUseless);
                vect_Traj_UAVT2.push_back(TrajectoryGeneratorUseless);
                payload_quadrotor_Vector_x_1 =
                    sqrt(pow(length, 2) - pow(waypoints_payload.poses[T_cnt_traj].pose.position.z -
                                                  waypoints_quadrotor_1.poses[T_cnt_traj].pose.position.z,
                                              2));
                payload_quadrotor_Vector_x_2 =
                    sqrt(pow(length1, 2) - pow(waypoints_payload.poses[T_cnt_traj].pose.position.z -
                                                   waypoints_quadrotor_2.poses[T_cnt_traj].pose.position.z,
                                               2));
                T_cnt_traj++;
            }
            else
            {
                vect_T_Traj_lable.push_back(true);

                double tmp1 = theta2 - theta1;
                std::vector<double> thetaSet;
                double interval = PI / 12;
                if (tmp1 > 0)
                {
                    double a = 2 * PI + theta1 - theta2;
                    if (a > tmp1)
                    {
                        int num = ceil(tmp1 / interval);
                        for (int j = 0; j <= num; j++)
                        {
                            if (j != num)
                            {
                                thetaSet.push_back(theta1 + j * interval);
                            }
                            else
                            {
                                thetaSet.push_back(theta2);
                            }
                        }
                    }
                    else
                    {
                        int num = ceil(a / interval);
                        for (int j = 0; j <= num; j++)
                        {
                            if (j == 0)
                            {
                                thetaSet.push_back(2 * PI + theta1);
                            }
                            else if (j != num)
                            {
                                thetaSet.push_back(2 * PI + theta1 - j * interval);
                            }
                            else
                            {
                                thetaSet.push_back(theta2);
                            }
                        }
                    }
                }
                else
                {
                    double a = 2 * PI + theta2 - theta1;
                    if (a > -tmp1)
                    {
                        int num = ceil(-tmp1 / interval) + 1;
                        for (int j = 0; j <= num; j++)
                        {
                            if (j == num)
                            {
                                thetaSet.push_back(theta2);
                            }
                            else
                            {
                                thetaSet.push_back(theta1 - j * interval);
                            }
                        }
                    }
                    else
                    {
                        int num = ceil(a / interval);
                        for (int j = 0; j <= num; j++)
                        {
                            if (j != num)
                            {
                                thetaSet.push_back(theta1 + j * interval);
                            }
                            else
                            {
                                thetaSet.push_back(theta2 + 2 * PI);
                            }
                        }
                    }
                }
                if (abs(thetaSet[thetaSet.size() - 1] - thetaSet[thetaSet.size() - 2]) < 1e-3)
                {
                    thetaSet.pop_back();
                }

                Eigen::MatrixXd pointSet1(2, thetaSet.size());
                Eigen::MatrixXd pointSet2(2, thetaSet.size());
                Vec2 center;
                center << waypoints_payload.poses[i].pose.position.x, waypoints_payload.poses[i].pose.position.y;

                for (int j = 0; j < thetaSet.size(); j++)
                {
                    Vec2 thetaSet_tmp1;
                    thetaSet_tmp1 << payload_quadrotor_Vector_x_1 * cos(thetaSet[j] - PI / 2),
                        payload_quadrotor_Vector_x_1 * sin(thetaSet[j] - PI / 2);
                    Vec2 thetaSet_tmp2;
                    thetaSet_tmp2 << payload_quadrotor_Vector_x_2 * cos(thetaSet[j] + PI / 2),
                        payload_quadrotor_Vector_x_2 * sin(thetaSet[j] + PI / 2);
                    pointSet1.col(j) = center + thetaSet_tmp1;
                    pointSet2.col(j) = center + thetaSet_tmp2;
                }

                TrajectoryGeneratorWaypoint TrajectoryGenerator1(TVel, TAcc, Torder);

                nav_msgs::Path waypoints1;
                waypoints1.header.frame_id = std::string("world");
                waypoints1.header.stamp = ros::Time::now();

                geometry_msgs::PoseStamped temp_wpt1;
                temp_wpt1.header.frame_id = "map";
                temp_wpt1.pose.orientation.w = 1;
                temp_wpt1.pose.orientation.z = 0;
                temp_wpt1.pose.orientation.y = 0;
                temp_wpt1.pose.orientation.x = 0;

                for (int j = 0; j < thetaSet.size(); j++)
                {
                    temp_wpt1.header.stamp = ros::Time::now();
                    temp_wpt1.pose.position.x = pointSet1(0, j);
                    temp_wpt1.pose.position.y = pointSet1(1, j);
                    temp_wpt1.pose.position.z = waypoints_payload.poses[i].pose.position.z +
                                                sqrt(pow(length, 2) - pow(payload_quadrotor_Vector_x_1, 2));
                    waypoints1.poses.push_back(temp_wpt1);
                }

                Eigen::MatrixXd margin_constraint1(2, 6);
                margin_constraint1 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

                TrajectoryGenerator1.isTraj = TrajectoryGenerator1.trajGeneration(waypoints1, margin_constraint1);

                if (TrajectoryGenerator1.isTraj)
                {
                    vect_Traj_UAVT1.push_back(TrajectoryGenerator1);
                }
                TrajectoryGeneratorWaypoint TrajectoryGenerator2(TVel, TAcc, Torder);

                nav_msgs::Path waypoints2;
                waypoints2.header.frame_id = std::string("world");
                waypoints2.header.stamp = ros::Time::now();

                geometry_msgs::PoseStamped temp_wpt2;
                temp_wpt2.header.frame_id = "map";
                temp_wpt2.pose.orientation.w = 1;
                temp_wpt2.pose.orientation.z = 0;
                temp_wpt2.pose.orientation.y = 0;
                temp_wpt2.pose.orientation.x = 0;

                for (int j = 0; j < thetaSet.size(); j++)
                {
                    temp_wpt2.header.stamp = ros::Time::now();
                    temp_wpt2.pose.position.x = pointSet2(0, j);
                    temp_wpt2.pose.position.y = pointSet2(1, j);
                    temp_wpt2.pose.position.z = waypoints_payload.poses[i].pose.position.z +
                                                sqrt(pow(length, 2) - pow(payload_quadrotor_Vector_x_2, 2));
                    waypoints2.poses.push_back(temp_wpt2);
                }

                Eigen::MatrixXd margin_constraint2(2, 6);
                margin_constraint2 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

                TrajectoryGenerator2.isTraj = TrajectoryGenerator2.trajGeneration(waypoints2, margin_constraint2);

                if (TrajectoryGenerator2.isTraj)
                {
                    vect_Traj_UAVT2.push_back(TrajectoryGenerator2);
                }
                payload_quadrotor_Vector_x_1 =
                    sqrt(pow(length, 2) - pow(waypoints_payload.poses[T_cnt_traj].pose.position.z -
                                                  waypoints_quadrotor_1.poses[T_cnt_traj].pose.position.z,
                                              2));
                payload_quadrotor_Vector_x_2 =
                    sqrt(pow(length1, 2) - pow(waypoints_payload.poses[T_cnt_traj].pose.position.z -
                                                   waypoints_quadrotor_2.poses[T_cnt_traj].pose.position.z,
                                               2));
                T_cnt_traj++;
            }
        }

        if (i != waypoints_payload.poses.size() - 2)
        {
            Vec2 direc1;
            direc1 << waypoints_payload.poses[i + 1].pose.position.x - waypoints_payload.poses[i].pose.position.x,
                waypoints_payload.poses[i + 1].pose.position.y - waypoints_payload.poses[i].pose.position.y;
            Vec2 direc2;
            direc2 << waypoints_payload.poses[i + 2].pose.position.x - waypoints_payload.poses[i + 1].pose.position.x,
                waypoints_payload.poses[i + 2].pose.position.y - waypoints_payload.poses[i + 1].pose.position.y;
            Vec2 refer;
            refer << 1, 0;
            double theta1 = acos(direc1.dot(refer) / direc1.norm());
            double theta2 = acos(direc2.dot(refer) / direc2.norm());

            if (direc1[1] < 0)
            {
                theta1 = 2 * PI - theta1;
            }
            if (direc2[1] < 0)
            {
                theta2 = 2 * PI - theta2;
            }

            if (abs(theta2 - theta1) < T_Traj_limit)
            {
                vect_T_Traj_lable.push_back(false);
                vect_Traj_UAVT1.push_back(TrajectoryGeneratorUseless);
                vect_Traj_UAVT2.push_back(TrajectoryGeneratorUseless);
                payload_quadrotor_Vector_x_1 =
                    sqrt(pow(length, 2) - pow(waypoints_payload.poses[T_cnt_traj].pose.position.z -
                                                  waypoints_quadrotor_1.poses[T_cnt_traj].pose.position.z,
                                              2));
                payload_quadrotor_Vector_x_2 =
                    sqrt(pow(length1, 2) - pow(waypoints_payload.poses[T_cnt_traj].pose.position.z -
                                                   waypoints_quadrotor_2.poses[T_cnt_traj].pose.position.z,
                                               2));
                T_cnt_traj++;
            }
            else
            {
                vect_T_Traj_lable.push_back(true);

                double tmp1 = theta2 - theta1;
                std::vector<double> thetaSet;
                double interval = PI / 12;
                if (tmp1 > 0)
                {
                    double a = 2 * PI + theta1 - theta2;
                    if (a > tmp1)
                    {
                        int num = ceil(tmp1 / interval);
                        for (int j = 0; j <= num; j++)
                        {
                            if (j != num)
                            {
                                thetaSet.push_back(theta1 + j * interval);
                            }
                            else
                            {
                                thetaSet.push_back(theta2);
                            }
                        }
                    }
                    else
                    {
                        int num = ceil(a / interval);
                        for (int j = 0; j <= num; j++)
                        {
                            if (j == 0)
                            {
                                thetaSet.push_back(2 * PI + theta1);
                            }
                            else if (j != num)
                            {
                                thetaSet.push_back(2 * PI + theta1 - j * interval);
                            }
                            else
                            {
                                thetaSet.push_back(theta2);
                            }
                        }
                    }
                }
                else
                {
                    double a = 2 * PI + theta2 - theta1;
                    if (a > -tmp1)
                    {
                        int num = ceil(-tmp1 / interval) + 1;
                        for (int j = 0; j <= num; j++)
                        {
                            if (j == num)
                            {
                                thetaSet.push_back(theta2);
                            }
                            else
                            {
                                thetaSet.push_back(theta1 - j * interval);
                            }
                        }
                    }
                    else
                    {
                        int num = ceil(a / interval);
                        for (int j = 0; j <= num; j++)
                        {
                            if (j != num)
                            {
                                thetaSet.push_back(theta1 + j * interval);
                            }
                            else
                            {
                                thetaSet.push_back(theta2 + 2 * PI);
                            }
                        }
                    }
                }
                if (abs(thetaSet[thetaSet.size() - 1] - thetaSet[thetaSet.size() - 2]) < 1e-3)
                {
                    thetaSet.pop_back();
                }

                Eigen::MatrixXd pointSet1(2, thetaSet.size());
                Eigen::MatrixXd pointSet2(2, thetaSet.size());
                Vec2 center;
                center << waypoints_payload.poses[i + 1].pose.position.x,
                    waypoints_payload.poses[i + 1].pose.position.y;

                for (int j = 0; j < thetaSet.size(); j++)
                {
                    Vec2 thetaSet_tmp1;
                    thetaSet_tmp1 << payload_quadrotor_Vector_x_1 * cos(thetaSet[j] - PI / 2),
                        payload_quadrotor_Vector_x_1 * sin(thetaSet[j] - PI / 2);
                    Vec2 thetaSet_tmp2;
                    thetaSet_tmp2 << payload_quadrotor_Vector_x_2 * cos(thetaSet[j] + PI / 2),
                        payload_quadrotor_Vector_x_2 * sin(thetaSet[j] + PI / 2);
                    pointSet1.col(j) = center + thetaSet_tmp1;
                    pointSet2.col(j) = center + thetaSet_tmp2;
                }

                TrajectoryGeneratorWaypoint TrajectoryGenerator1(TVel, TAcc, Torder);

                nav_msgs::Path waypoints1;
                waypoints1.header.frame_id = std::string("world");
                waypoints1.header.stamp = ros::Time::now();

                geometry_msgs::PoseStamped temp_wpt1;
                temp_wpt1.header.frame_id = "map";
                temp_wpt1.pose.orientation.w = 1;
                temp_wpt1.pose.orientation.z = 0;
                temp_wpt1.pose.orientation.y = 0;
                temp_wpt1.pose.orientation.x = 0;

                for (int j = 0; j < thetaSet.size(); j++)
                {
                    temp_wpt1.header.stamp = ros::Time::now();
                    temp_wpt1.pose.position.x = pointSet1(0, j);
                    temp_wpt1.pose.position.y = pointSet1(1, j);
                    temp_wpt1.pose.position.z = waypoints_payload.poses[i + 1].pose.position.z +
                                                sqrt(pow(length, 2) - pow(payload_quadrotor_Vector_x_1, 2));
                    waypoints1.poses.push_back(temp_wpt1);
                }

                Eigen::MatrixXd margin_constraint1(2, 6);
                margin_constraint1 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

                TrajectoryGenerator1.isTraj = TrajectoryGenerator1.trajGeneration(waypoints1, margin_constraint1);

                if (TrajectoryGenerator1.isTraj)
                {
                    vect_Traj_UAVT1.push_back(TrajectoryGenerator1);
                }
                TrajectoryGeneratorWaypoint TrajectoryGenerator2(TVel, TAcc, Torder);

                nav_msgs::Path waypoints2;
                waypoints2.header.frame_id = std::string("world");
                waypoints2.header.stamp = ros::Time::now();

                geometry_msgs::PoseStamped temp_wpt2;
                temp_wpt2.header.frame_id = "map";
                temp_wpt2.pose.orientation.w = 1;
                temp_wpt2.pose.orientation.z = 0;
                temp_wpt2.pose.orientation.y = 0;
                temp_wpt2.pose.orientation.x = 0;

                for (int j = 0; j < thetaSet.size(); j++)
                {
                    temp_wpt2.header.stamp = ros::Time::now();
                    temp_wpt2.pose.position.x = pointSet2(0, j);
                    temp_wpt2.pose.position.y = pointSet2(1, j);
                    temp_wpt2.pose.position.z = waypoints_payload.poses[i + 1].pose.position.z +
                                                sqrt(pow(length, 2) - pow(payload_quadrotor_Vector_x_2, 2));
                    waypoints2.poses.push_back(temp_wpt2);
                }

                Eigen::MatrixXd margin_constraint2(2, 6);
                margin_constraint2 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

                TrajectoryGenerator2.isTraj = TrajectoryGenerator2.trajGeneration(waypoints2, margin_constraint2);

                if (TrajectoryGenerator2.isTraj)
                {
                    vect_Traj_UAVT2.push_back(TrajectoryGenerator2);
                }
                payload_quadrotor_Vector_x_1 =
                    sqrt(pow(length, 2) - pow(waypoints_payload.poses[T_cnt_traj].pose.position.z -
                                                  waypoints_quadrotor_1.poses[T_cnt_traj].pose.position.z,
                                              2));
                payload_quadrotor_Vector_x_2 =
                    sqrt(pow(length1, 2) - pow(waypoints_payload.poses[T_cnt_traj].pose.position.z -
                                                   waypoints_quadrotor_2.poses[T_cnt_traj].pose.position.z,
                                               2));
                T_cnt_traj++;
            }
        }
    }
}
