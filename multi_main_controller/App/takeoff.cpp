#include "loadController/common_include.h"
#include "loadController/flightController.h"
#include "loadController/node.h"
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include "trajGenerator/trajectory_generator_waypoint.h"

#include <fstream>
#include <iostream>
#include <vector>

std::vector<TrajectoryGeneratorWaypoint> vect_Traj;
std::vector<TrajectoryGeneratorWaypoint> vect_Traj_UAVT1;
std::vector<TrajectoryGeneratorWaypoint> vect_Traj_UAVT2;
std::vector<bool> vect_T_Traj_lable;
std::string waypoints_txt_path;

double Vel, Acc;
int dev_order;
double rope_length;
bool continuous_lable;
bool input_waypoints_mode; // true, from txt. false, from ros.
bool rcvpayloadWaypointsCallBack_lable, rcvquadrotor1WaypointsCallBack_lable, rcvquadrotor2WaypointsCallBack_lable;
double payload_quadrotor_1_Vector_x, payload_quadrotor_2_Vector_x;
double origin_Vector_x_1, origin_Vector_x_2;

double start_position_x;
double start_position_y;
double start_position_z;

ros::Subscriber payload_way_pts_sub, quadrotor_1_way_pts_sub, quadrotor_2_way_pts_sub;

nav_msgs::Path quadrotor_1_waypoints, quadrotor_2_waypoints, payload_waypoints;

double visualization_traj_width;

/// Trajectory generation based on payload waypoints
void rcvpayloadWaypointsCallBack(const nav_msgs::Path &wp);

/// Trajectory generation based on quadrotor1 waypoints
void rcvquadrotor1WaypointsCallBack(const nav_msgs::Path &wp);

/// Trajectory generation based on quadrotor2 waypoints
void rcvquadrotor2WaypointsCallBack(const nav_msgs::Path &wp);

void rcvpayloadWaypointsCallBack(const nav_msgs::Path &wp)
{
    if (continuous_lable)
    {
        TrajectoryGeneratorWaypoint TrajectoryGenerator(Vel, Acc, dev_order);
        nav_msgs::Path waypoints;
        waypoints.header.frame_id = std::string("world");
        waypoints.header.stamp = ros::Time::now();

        geometry_msgs::PoseStamped temp_wpt;
        temp_wpt.header.frame_id = "map";
        temp_wpt.pose.orientation.w = 1;
        temp_wpt.pose.orientation.z = 0;
        temp_wpt.pose.orientation.y = 0;
        temp_wpt.pose.orientation.x = 0;

        temp_wpt.header.stamp = ros::Time::now();
        temp_wpt.pose.position.x = start_position_x;
        temp_wpt.pose.position.y = start_position_y;
        temp_wpt.pose.position.z = start_position_z;
        waypoints.poses.push_back(temp_wpt);

        for (int k = 0; k < (int)wp.poses.size(); k++)
        {
            temp_wpt.header.stamp = ros::Time::now();
            temp_wpt.pose.position.x = wp.poses[k].pose.position.x;
            temp_wpt.pose.position.y = wp.poses[k].pose.position.y;
            temp_wpt.pose.position.z = wp.poses[k].pose.position.z;
            waypoints.poses.push_back(temp_wpt);
        }
        Eigen::MatrixXd margin_constraint(2, 6);
        margin_constraint << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
        TrajectoryGenerator.isTraj = TrajectoryGenerator.trajGeneration(waypoints, margin_constraint);
        if (TrajectoryGenerator.isTraj)
        {
            vect_Traj.push_back(TrajectoryGenerator);
        }
    }
    else
    {
        {
            TrajectoryGeneratorWaypoint TrajectoryGenerator(Vel, Acc, dev_order);
            nav_msgs::Path waypoints;
            waypoints.header.frame_id = std::string("world");
            waypoints.header.stamp = ros::Time::now();

            geometry_msgs::PoseStamped temp_wpt;
            temp_wpt.header.frame_id = "map";
            temp_wpt.pose.orientation.w = 1;
            temp_wpt.pose.orientation.z = 0;
            temp_wpt.pose.orientation.y = 0;
            temp_wpt.pose.orientation.x = 0;

            temp_wpt.header.stamp = ros::Time::now();
            temp_wpt.pose.position.x = start_position_x;
            temp_wpt.pose.position.y = start_position_y;
            temp_wpt.pose.position.z = start_position_z;
            waypoints.poses.push_back(temp_wpt);

            temp_wpt.header.stamp = ros::Time::now();
            temp_wpt.pose.position.x = wp.poses[0].pose.position.x;
            temp_wpt.pose.position.y = wp.poses[0].pose.position.y;
            temp_wpt.pose.position.z = wp.poses[0].pose.position.z;
            waypoints.poses.push_back(temp_wpt);
            Eigen::MatrixXd margin_constraint(2, 6);
            margin_constraint << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

            TrajectoryGenerator.isTraj = TrajectoryGenerator.trajGeneration(waypoints, margin_constraint);
            if (TrajectoryGenerator.isTraj)
            {
                vect_Traj.push_back(TrajectoryGenerator);
            }
        }
        for (int k = 0; k < (int)wp.poses.size() - 1; k++)
        {
            TrajectoryGeneratorWaypoint TrajectoryGenerator(Vel, Acc, dev_order);

            nav_msgs::Path waypoints;
            waypoints.header.frame_id = std::string("world");
            waypoints.header.stamp = ros::Time::now();

            geometry_msgs::PoseStamped temp_wpt;
            temp_wpt.header.frame_id = "map";
            temp_wpt.pose.orientation.w = 1;
            temp_wpt.pose.orientation.z = 0;
            temp_wpt.pose.orientation.y = 0;
            temp_wpt.pose.orientation.x = 0;

            temp_wpt.header.stamp = ros::Time::now();
            temp_wpt.pose.position.x = wp.poses[k].pose.position.x;
            temp_wpt.pose.position.y = wp.poses[k].pose.position.y;
            temp_wpt.pose.position.z = wp.poses[k].pose.position.z;
            waypoints.poses.push_back(temp_wpt);

            temp_wpt.header.stamp = ros::Time::now();
            temp_wpt.pose.position.x = wp.poses[k + 1].pose.position.x;
            temp_wpt.pose.position.y = wp.poses[k + 1].pose.position.y;
            temp_wpt.pose.position.z = wp.poses[k + 1].pose.position.z;
            waypoints.poses.push_back(temp_wpt);

            Eigen::MatrixXd margin_constraint(2, 6);
            margin_constraint << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
            TrajectoryGenerator.isTraj = TrajectoryGenerator.trajGeneration(waypoints, margin_constraint);
            if (TrajectoryGenerator.isTraj)
            {
                vect_Traj.push_back(TrajectoryGenerator);
            }
        }
    }
    payload_waypoints = wp;

    rcvpayloadWaypointsCallBack_lable = true;
}

void rcvquadrotor1WaypointsCallBack(const nav_msgs::Path &wp)
{
    quadrotor_1_waypoints = wp;
    rcvquadrotor1WaypointsCallBack_lable = true;
}

void rcvquadrotor2WaypointsCallBack(const nav_msgs::Path &wp)
{
    quadrotor_2_waypoints = wp;
    rcvquadrotor2WaypointsCallBack_lable = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_main_controller");
    ros::NodeHandle nh("~");
    ros::Rate rate(1000);
    ptc::Node n;

    rcvpayloadWaypointsCallBack_lable = false;
    rcvquadrotor1WaypointsCallBack_lable = false;
    rcvquadrotor2WaypointsCallBack_lable = false;

    nh.param("planning/dev_order", dev_order, 4);
    nh.param("continuous_lable", continuous_lable, false);
    nh.param("input_waypoints_mode", input_waypoints_mode, true);

    nh.param("vel", Vel, 0.25);
    nh.param("acc", Acc, 0.25);
    nh.param("length", rope_length, 0.89);

    nh.param("origin_Vector_x_1", origin_Vector_x_1, 0.4);
    nh.param("origin_Vector_x_2", origin_Vector_x_2, 0.4);

    nh.param("start_position/x", start_position_x, 0.4);
    nh.param("start_position/y", start_position_y, 0.0);
    nh.param("start_position/z", start_position_z, 2.2);

    nh.param("waypoints_txt_path", waypoints_txt_path,
             std::string("~/Inset_ws/src/InsetROS/multi_main_controller/config/waypoints.txt"));

    if (input_waypoints_mode)
    {
        std::vector<double> temp_txt;
        double read_txt;
        std::ifstream waypoints_txt;
        waypoints_txt.open(waypoints_txt_path, std::ios::in);
        while (!waypoints_txt.eof())
        {
            waypoints_txt >> read_txt;
            temp_txt.push_back(read_txt);
        }
        waypoints_txt.close();

        if (continuous_lable)
        {
            TrajectoryGeneratorWaypoint TrajectoryGenerator(Vel, Acc, dev_order);

            nav_msgs::Path waypoints;
            waypoints.header.frame_id = std::string("world");
            waypoints.header.stamp = ros::Time::now();

            geometry_msgs::PoseStamped temp_wpt;
            temp_wpt.header.frame_id = "map";
            temp_wpt.pose.orientation.w = 1;
            temp_wpt.pose.orientation.z = 0;
            temp_wpt.pose.orientation.y = 0;
            temp_wpt.pose.orientation.x = 0;

            for (int i = 0; i < temp_txt.size() / 3; ++i)
            {
                temp_wpt.header.stamp = ros::Time::now();
                temp_wpt.pose.position.x = temp_txt[i * 3];
                temp_wpt.pose.position.y = temp_txt[i * 3 + 1];
                temp_wpt.pose.position.z = temp_txt[i * 3 + 2];
                waypoints.poses.push_back(temp_wpt);
            }

            Eigen::MatrixXd margin_constraint(2, 6);
            margin_constraint << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

            TrajectoryGenerator.isTraj = TrajectoryGenerator.trajGeneration(waypoints, margin_constraint);

            if (TrajectoryGenerator.isTraj)
            {
                vect_Traj.push_back(TrajectoryGenerator);
            }
            std::vector<double> start_temp_txt;
            start_temp_txt.push_back(temp_txt[0 * 3]);
            start_temp_txt.push_back(temp_txt[0 * 3 + 1]);
            start_temp_txt.push_back(temp_txt[0 * 3 + 2]);
            start_temp_txt.push_back(temp_txt[1 * 3]);
            start_temp_txt.push_back(temp_txt[1 * 3 + 1]);
            start_temp_txt.push_back(temp_txt[1 * 3 + 2]);
            Transition_Trajectory_Generator(start_temp_txt, vect_Traj_UAVT1, vect_Traj_UAVT2, vect_T_Traj_lable,
                                            origin_Vector_x_1, origin_Vector_x_2, rope_length, rope_length, Vel, Acc);
        }
        else
        {
            for (int i = 0; i < temp_txt.size() / 3 - 1; i++)
            {
                TrajectoryGeneratorWaypoint TrajectoryGenerator(Vel, Acc, dev_order);

                nav_msgs::Path waypoints;
                waypoints.header.frame_id = std::string("world");
                waypoints.header.stamp = ros::Time::now();

                geometry_msgs::PoseStamped temp_wpt;
                temp_wpt.header.frame_id = "map";
                temp_wpt.pose.orientation.w = 1;
                temp_wpt.pose.orientation.z = 0;
                temp_wpt.pose.orientation.y = 0;
                temp_wpt.pose.orientation.x = 0;

                temp_wpt.header.stamp = ros::Time::now();
                temp_wpt.pose.position.x = temp_txt[i * 3];
                temp_wpt.pose.position.y = temp_txt[i * 3 + 1];
                temp_wpt.pose.position.z = temp_txt[i * 3 + 2];
                waypoints.poses.push_back(temp_wpt);

                temp_wpt.header.stamp = ros::Time::now();
                temp_wpt.pose.position.x = temp_txt[(i + 1) * 3];
                temp_wpt.pose.position.y = temp_txt[(i + 1) * 3 + 1];
                temp_wpt.pose.position.z = temp_txt[(i + 1) * 3 + 2];
                waypoints.poses.push_back(temp_wpt);

                Eigen::MatrixXd margin_constraint(2, 6);
                margin_constraint << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;

                TrajectoryGenerator.isTraj = TrajectoryGenerator.trajGeneration(waypoints, margin_constraint);

                if (TrajectoryGenerator.isTraj)
                {
                    vect_Traj.push_back(TrajectoryGenerator);
                }
            }
            Transition_Trajectory_Generator(temp_txt, vect_Traj_UAVT1, vect_Traj_UAVT2, vect_T_Traj_lable,
                                            origin_Vector_x_1, origin_Vector_x_2, rope_length, rope_length, Vel, Acc);
        }
    }
    else
    {
        payload_way_pts_sub = nh.subscribe("payload_waypoints", 1, rcvpayloadWaypointsCallBack);
        quadrotor_1_way_pts_sub = nh.subscribe("quadrotor_1_waypoints", 1, rcvquadrotor1WaypointsCallBack);
        quadrotor_2_way_pts_sub = nh.subscribe("quadrotor_2_waypoints", 1, rcvquadrotor2WaypointsCallBack);
        bool status = ros::ok();
        while (status && !(rcvpayloadWaypointsCallBack_lable))
        {
            ros::spinOnce();
            status = ros::ok();
            rate.sleep();
        }
        while (status && !(rcvquadrotor1WaypointsCallBack_lable))
        {
            ros::spinOnce();
            status = ros::ok();
            rate.sleep();
        }
        while (status && !(rcvquadrotor2WaypointsCallBack_lable))
        {
            ros::spinOnce();
            status = ros::ok();
            rate.sleep();
        }
        geometry_msgs::PoseStamped start_position;
        start_position.pose.position.x = start_position_x;
        start_position.pose.position.y = start_position_y;
        start_position.pose.position.z = start_position_z;

        Transition_Trajectory_Generator(start_position, payload_waypoints, vect_Traj_UAVT1, vect_Traj_UAVT2,
                                        vect_T_Traj_lable, quadrotor_1_waypoints, quadrotor_2_waypoints, rope_length,
                                        rope_length, Vel, Acc);
    }

    double massQuadcopter1 = 0.52;
    double massQuadcopter2 = 0.52;
    double massPayload = 0.168;
    float length1 = rope_length;
    float length2 = rope_length;

    double ky_p1, ky_p2, ky_p3;
    double ky_d1, ky_d2, ky_d3;
    double k_q1, k_q2, k_q3;
    double k_w1, k_w2, k_w3;
    double k_R1, k_R2, k_R3;
    double k_Omega1, k_Omega2, k_Omega3;
    double k_bx1, k_bx2, k_bx3;
    double k_bv1, k_bv2, k_bv3;
    double Xk_q1, Xk_q2, Xk_q3;
    double Xk_w1, Xk_w2, Xk_w3;
    double Xk_R1, Xk_R2, Xk_R3;
    double Xk_Omega1, Xk_Omega2, Xk_Omega3;
    double Xk_bx1, Xk_bx2, Xk_bx3;
    double Xk_bv1, Xk_bv2, Xk_bv3;

    nh.param("ky_p1", ky_p1, 0.0);
    nh.param("ky_p2", ky_p2, 0.0);
    nh.param("ky_p3", ky_p3, 0.0);

    nh.param("ky_d1", ky_d1, 0.0);
    nh.param("ky_d2", ky_d2, 0.0);
    nh.param("ky_d3", ky_d3, 0.0);

    nh.param("k_q1", k_q1, 1.5);
    nh.param("k_q2", k_q2, 1.5);
    nh.param("k_q3", k_q3, 1.5);

    nh.param("k_w1", k_w1, 0.1);
    nh.param("k_w2", k_w2, 0.1);
    nh.param("k_w3", k_w3, 0.1);

    nh.param("k_R1", k_R1, 3.0);
    nh.param("k_R2", k_R2, 3.0);
    nh.param("k_R3", k_R3, 1.0);

    nh.param("k_Omega1", k_Omega1, 0.8);
    nh.param("k_Omega2", k_Omega2, 0.8);
    nh.param("k_Omega3", k_Omega3, 1.0);

    nh.param("k_bx1", k_bx1, 2.0);
    nh.param("k_bx2", k_bx2, 2.0);
    nh.param("k_bx3", k_bx3, 1.5);

    nh.param("k_bv1", k_bv1, 2.5);
    nh.param("k_bv2", k_bv2, 2.5);
    nh.param("k_bv3", k_bv3, 2.5);

    nh.param("Xk_q1", Xk_q1, 1.5);
    nh.param("Xk_q2", Xk_q2, 1.5);
    nh.param("Xk_q3", Xk_q3, 1.5);

    nh.param("Xk_w1", Xk_w1, 0.1);
    nh.param("Xk_w2", Xk_w2, 0.1);
    nh.param("Xk_w3", Xk_w3, 0.1);

    nh.param("Xk_R1", Xk_R1, 3.0);
    nh.param("Xk_R2", Xk_R2, 3.0);
    nh.param("Xk_R3", Xk_R3, 1.0);

    nh.param("Xk_Omega1", Xk_Omega1, 0.8);
    nh.param("Xk_Omega2", Xk_Omega2, 0.8);
    nh.param("Xk_Omega3", Xk_Omega3, 1.0);

    nh.param("Xk_bx1", Xk_bx1, 2.0);
    nh.param("Xk_bx2", Xk_bx2, 2.0);
    nh.param("Xk_bx3", Xk_bx3, 1.5);

    nh.param("Xk_bv1", Xk_bv1, 2.5);
    nh.param("Xk_bv2", Xk_bv2, 2.5);
    nh.param("Xk_bv3", Xk_bv3, 2.5);

    n.controller.initializePIDParameter(
        ky_p1, ky_p2, ky_p3, ky_d1, ky_d2, ky_d3, k_q1, k_q2, k_q3, k_w1, k_w2, k_w3, k_R1, k_R2, k_R3, k_Omega1,
        k_Omega2, k_Omega3, k_bx1, k_bx2, k_bx3, k_bv1, k_bv2, k_bv3, Xk_q1, Xk_q2, Xk_q3, Xk_w1, Xk_w2, Xk_w3, Xk_R1,
        Xk_R2, Xk_R3, Xk_Omega1, Xk_Omega2, Xk_Omega3, Xk_bx1, Xk_bx2, Xk_bx3, Xk_bv1, Xk_bv2, Xk_bv3);

    double startTime = 0;
    unsigned int cnt_traj = 0;
    unsigned int cnt_traj_T = 0;
    unsigned int cnt_traj_waypoint = 0;
    bool position_vrep_get_lable = false;
    Vec3 cpayload_waypoint;

    if (!input_waypoints_mode)
    {
        payload_quadrotor_1_Vector_x =
            sqrt(pow(length1, 2) - pow(payload_waypoints.poses[cnt_traj].pose.position.z -
                                           quadrotor_1_waypoints.poses[cnt_traj].pose.position.z,
                                       2));
        payload_quadrotor_2_Vector_x =
            sqrt(pow(length2, 2) - pow(payload_waypoints.poses[cnt_traj].pose.position.z -
                                           quadrotor_2_waypoints.poses[cnt_traj].pose.position.z,
                                       2));
    }
    else
    {
        payload_quadrotor_1_Vector_x = origin_Vector_x_1;
        payload_quadrotor_2_Vector_x = origin_Vector_x_2;
    }

    n.controller.initializeParameter(massQuadcopter1, massQuadcopter2, massPayload, length1, length2,
                                     payload_quadrotor_1_Vector_x, payload_quadrotor_2_Vector_x);

    if (continuous_lable)
    {
        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();

            while (!position_vrep_get_lable)
            {
                if (n.controller.feedback.cPayloadPosition.norm() != 0)
                {
                    position_vrep_get_lable = true;
                }
                else
                {
                    ros::spinOnce();
                    rate.sleep();
                }
            }

            if (vect_Traj.empty())
            {
                Eigen::Vector3d average_desiredPayloadVel_Orientation;
                average_desiredPayloadVel_Orientation << 0, 1, 0;
                Vec3 inputDesiredPos = Vec3(start_position_x, start_position_y, start_position_z);
                Vec3 inputDesiredVel = Vec3(0, 0, 0);
                Vec3 inputDesiredAcc = Vec3(0, 0, 0);
                Eigen::Vector3d inputDesiredJerk = Vec3(0, 0, 0);
                Eigen::Vector3d inputDesiredSnap = Vec3(0, 0, 0);
                n.controller.Continuous_Controller(inputDesiredPos, inputDesiredVel, inputDesiredAcc, inputDesiredJerk,
                                                   inputDesiredSnap, inputDesiredPos,
                                                   average_desiredPayloadVel_Orientation);
            }
            else
            {
                if (startTime == 0)
                {
                    startTime = ros::Time::now().toSec();
                }
                double t = ros::Time::now().toSec() - startTime;

                if (cnt_traj_T == 0)
                {
                    if (vect_T_Traj_lable[cnt_traj_T])
                    {
                        while (vect_Traj_UAVT1[cnt_traj_T]._totalTime > t)
                        {
                            Eigen::Vector3d inputDesiredPp = vect_Traj[cnt_traj_T].getTrajectoryStates(0, 0);
                            Eigen::Vector3d inputDesiredPos1 = vect_Traj_UAVT1[cnt_traj_T].getTrajectoryStates(t, 0);
                            Eigen::Vector3d inputDesiredVel1 = vect_Traj_UAVT1[cnt_traj_T].getTrajectoryStates(t, 1);
                            Eigen::Vector3d inputDesiredAcc1 = vect_Traj_UAVT1[cnt_traj_T].getTrajectoryStates(t, 2);
                            Eigen::Vector3d inputDesiredPos2 = vect_Traj_UAVT2[cnt_traj_T].getTrajectoryStates(t, 0);
                            Eigen::Vector3d inputDesiredVel2 = vect_Traj_UAVT2[cnt_traj_T].getTrajectoryStates(t, 1);
                            Eigen::Vector3d inputDesiredAcc2 = vect_Traj_UAVT2[cnt_traj_T].getTrajectoryStates(t, 2);
                            n.controller.Transition_Controller(inputDesiredPos1, inputDesiredVel1, inputDesiredAcc1,
                                                               inputDesiredPos2, inputDesiredVel2, inputDesiredAcc2,
                                                               inputDesiredPp);

                            t = ros::Time::now().toSec() - startTime;

#ifdef VREP
                            n.publishToVrep();
#endif
                            ros::spinOnce();
                            rate.sleep();
                        }
                    }

                    if (cnt_traj_T < vect_Traj_UAVT1.size())
                    {
                        startTime += vect_Traj_UAVT1[cnt_traj_T]._totalTime;
                        cnt_traj_T = cnt_traj_T + 1;
                    }
                }
                t = ros::Time::now().toSec() - startTime;
                if (vect_Traj[cnt_traj]._totalTime > t)
                {
                    Eigen::Vector3d average_desiredPayloadVel_Orientation;
                    average_desiredPayloadVel_Orientation << 0, 1, 0;
                    Eigen::Vector3d targetpoint;
                    targetpoint = vect_Traj[cnt_traj].getTrajectoryStates(vect_Traj[cnt_traj]._totalTime - 0.0001, 0);
                    Eigen::Vector3d inputDesiredPos = vect_Traj[cnt_traj].getTrajectoryStates(t, 0);
                    Eigen::Vector3d inputDesiredVel = vect_Traj[cnt_traj].getTrajectoryStates(t, 1);
                    Eigen::Vector3d inputDesiredAcc = vect_Traj[cnt_traj].getTrajectoryStates(t, 2);
                    Eigen::Vector3d inputDesiredJerk = vect_Traj[cnt_traj].getTrajectoryStates(t, 3);
                    Eigen::Vector3d inputDesiredSnap = vect_Traj[cnt_traj].getTrajectoryStates(t, 4);
                    n.controller.Continuous_Controller(inputDesiredPos, inputDesiredVel, inputDesiredAcc,
                                                       inputDesiredJerk, inputDesiredSnap, inputDesiredPos,
                                                       average_desiredPayloadVel_Orientation);
                }
                else
                {
                    t = vect_Traj[cnt_traj]._totalTime - 0.001;

                    Eigen::Vector3d average_desiredPayloadVel_Orientation;
                    average_desiredPayloadVel_Orientation << 0, 1, 0;
                    Eigen::Vector3d inputDesiredPos = vect_Traj[cnt_traj].getTrajectoryStates(t, 0);
                    Eigen::Vector3d inputDesiredVel = vect_Traj[cnt_traj].getTrajectoryStates(t, 1);
                    Eigen::Vector3d inputDesiredAcc = vect_Traj[cnt_traj].getTrajectoryStates(t, 2);
                    Eigen::Vector3d inputDesiredJerk = vect_Traj[cnt_traj].getTrajectoryStates(t, 3);
                    Eigen::Vector3d inputDesiredSnap = vect_Traj[cnt_traj].getTrajectoryStates(t, 4);

                    n.controller.Continuous_Controller(inputDesiredPos, inputDesiredVel, inputDesiredAcc,
                                                       inputDesiredJerk, inputDesiredSnap, inputDesiredPos,
                                                       average_desiredPayloadVel_Orientation);

                    if (cnt_traj + 1 < vect_Traj.size())
                    {
                        startTime += vect_Traj[cnt_traj]._totalTime;
                        cnt_traj++;
                        if (!input_waypoints_mode)
                        {
                            payload_quadrotor_1_Vector_x =
                                sqrt(pow(length1, 2) - pow(payload_waypoints.poses[cnt_traj].pose.position.z -
                                                               quadrotor_1_waypoints.poses[cnt_traj].pose.position.z,
                                                           2));
                            payload_quadrotor_2_Vector_x =
                                sqrt(pow(length2, 2) - pow(payload_waypoints.poses[cnt_traj].pose.position.z -
                                                               quadrotor_2_waypoints.poses[cnt_traj].pose.position.z,
                                                           2));
                        }
                        else
                        {
                            payload_quadrotor_1_Vector_x = origin_Vector_x_1;
                            payload_quadrotor_2_Vector_x = origin_Vector_x_2;
                        }
                        n.controller.set_Vector_x(payload_quadrotor_1_Vector_x, payload_quadrotor_2_Vector_x);
                    }
                }
            }
#ifdef VREP
            n.publishToVrep();
#endif
        }
    }
    else
    {
        while (ros::ok())
        {
            ros::spinOnce();
            rate.sleep();

            while (!position_vrep_get_lable)
            {
                if (n.controller.feedback.cPayloadPosition.norm() != 0)
                {
                    position_vrep_get_lable = true;
                }
                else
                {
                    ros::spinOnce();
                    rate.sleep();
                }
            }

            if (vect_Traj.empty())
            {
                Vec3 inputDesiredPos = Vec3(start_position_x, start_position_y, start_position_z);
                Eigen::Vector3d average_desiredPayloadVel_Orientation;
                average_desiredPayloadVel_Orientation << 0, 1, 0;
                Vec3 inputDesiredVel = Vec3(0, 0, 0);
                Vec3 inputDesiredAcc = Vec3(0, 0, 0);
                Eigen::Vector3d inputDesiredJerk = Vec3(0, 0, 0);
                Eigen::Vector3d inputDesiredSnap = Vec3(0, 0, 0);
                n.controller.Cross_Controller(inputDesiredPos, inputDesiredVel, inputDesiredAcc, inputDesiredJerk,
                                              inputDesiredSnap, inputDesiredPos, average_desiredPayloadVel_Orientation);
            }
            else
            {
                if (startTime == 0)
                {
                    startTime = ros::Time::now().toSec();
                }
                double t = ros::Time::now().toSec() - startTime;

                if (cnt_traj_T == 0)
                {
                    if (vect_T_Traj_lable[cnt_traj_T])
                    {
                        while (vect_Traj_UAVT1[cnt_traj_T]._totalTime > t)
                        {
                            Eigen::Vector3d inputDesiredPp = vect_Traj[cnt_traj_T].getTrajectoryStates(0, 0);
                            Eigen::Vector3d inputDesiredPos1 = vect_Traj_UAVT1[cnt_traj_T].getTrajectoryStates(t, 0);
                            Eigen::Vector3d inputDesiredVel1 = vect_Traj_UAVT1[cnt_traj_T].getTrajectoryStates(t, 1);
                            Eigen::Vector3d inputDesiredAcc1 = vect_Traj_UAVT1[cnt_traj_T].getTrajectoryStates(t, 2);
                            Eigen::Vector3d inputDesiredPos2 = vect_Traj_UAVT2[cnt_traj_T].getTrajectoryStates(t, 0);
                            Eigen::Vector3d inputDesiredVel2 = vect_Traj_UAVT2[cnt_traj_T].getTrajectoryStates(t, 1);
                            Eigen::Vector3d inputDesiredAcc2 = vect_Traj_UAVT2[cnt_traj_T].getTrajectoryStates(t, 2);
                            n.controller.Transition_Controller(inputDesiredPos1, inputDesiredVel1, inputDesiredAcc1,
                                                               inputDesiredPos2, inputDesiredVel2, inputDesiredAcc2,
                                                               inputDesiredPp);

                            t = ros::Time::now().toSec() - startTime;
#ifdef VREP
                            n.publishToVrep();
#endif
                            ros::spinOnce();
                            rate.sleep();
                        }
                    }

                    if (cnt_traj_T < vect_Traj_UAVT1.size())
                    {
                        startTime += vect_Traj_UAVT1[cnt_traj_T]._totalTime;
                        cnt_traj_T = cnt_traj_T + 1;
                    }
                }
                t = ros::Time::now().toSec() - startTime;
                if (vect_Traj[cnt_traj]._totalTime > t)
                {
                    Eigen::Vector3d targetpoint, beginpoint, average_desiredPayloadVel_Orientation;
                    targetpoint = vect_Traj[cnt_traj].getTrajectoryStates(vect_Traj[cnt_traj]._totalTime - 0.0001, 0);
                    beginpoint = vect_Traj[cnt_traj].getTrajectoryStates(0.0001, 0);
                    average_desiredPayloadVel_Orientation = targetpoint - beginpoint;
                    average_desiredPayloadVel_Orientation = average_desiredPayloadVel_Orientation.normalized();
                    Eigen::Vector3d inputDesiredPos = vect_Traj[cnt_traj].getTrajectoryStates(t, 0);
                    Eigen::Vector3d inputDesiredVel = vect_Traj[cnt_traj].getTrajectoryStates(t, 1);
                    Eigen::Vector3d inputDesiredAcc = vect_Traj[cnt_traj].getTrajectoryStates(t, 2);
                    Eigen::Vector3d inputDesiredJerk = vect_Traj[cnt_traj].getTrajectoryStates(t, 3);
                    Eigen::Vector3d inputDesiredSnap = vect_Traj[cnt_traj].getTrajectoryStates(t, 4);
                    n.controller.Cross_Controller(inputDesiredPos, inputDesiredVel, inputDesiredAcc, inputDesiredJerk,
                                                  inputDesiredSnap, inputDesiredPos,
                                                  average_desiredPayloadVel_Orientation);
                }
                else
                {
                    t = vect_Traj[cnt_traj]._totalTime - 0.001;
                    Eigen::Vector3d targetpoint, beginpoint, average_desiredPayloadVel_Orientation;
                    targetpoint = vect_Traj[cnt_traj].getTrajectoryStates(vect_Traj[cnt_traj]._totalTime - 0.0001, 0);
                    beginpoint = vect_Traj[cnt_traj].getTrajectoryStates(0.0001, 0);
                    average_desiredPayloadVel_Orientation = targetpoint - beginpoint;
                    average_desiredPayloadVel_Orientation = average_desiredPayloadVel_Orientation.normalized();
                    Eigen::Vector3d inputDesiredPos = vect_Traj[cnt_traj].getTrajectoryStates(t, 0);
                    Eigen::Vector3d inputDesiredVel = vect_Traj[cnt_traj].getTrajectoryStates(t, 1);
                    Eigen::Vector3d inputDesiredAcc = vect_Traj[cnt_traj].getTrajectoryStates(t, 2);
                    Eigen::Vector3d inputDesiredJerk = vect_Traj[cnt_traj].getTrajectoryStates(t, 3);
                    Eigen::Vector3d inputDesiredSnap = vect_Traj[cnt_traj].getTrajectoryStates(t, 4);
                    n.controller.Cross_Controller(inputDesiredPos, inputDesiredVel, inputDesiredAcc, inputDesiredJerk,
                                                  inputDesiredSnap, inputDesiredPos,
                                                  average_desiredPayloadVel_Orientation);
                    if (cnt_traj + 1 < vect_Traj.size())
                    {
                        startTime += vect_Traj[cnt_traj]._totalTime;
                        cnt_traj++;
                        if (!input_waypoints_mode)
                        {
                            payload_quadrotor_1_Vector_x =
                                sqrt(pow(length1, 2) - pow(payload_waypoints.poses[cnt_traj].pose.position.z -
                                                               quadrotor_1_waypoints.poses[cnt_traj].pose.position.z,
                                                           2));
                            payload_quadrotor_2_Vector_x =
                                sqrt(pow(length2, 2) - pow(payload_waypoints.poses[cnt_traj].pose.position.z -
                                                               quadrotor_2_waypoints.poses[cnt_traj].pose.position.z,
                                                           2));
                        }
                        else
                        {
                            payload_quadrotor_1_Vector_x = origin_Vector_x_1;
                            payload_quadrotor_2_Vector_x = origin_Vector_x_2;
                        }
                        n.controller.set_Vector_x(payload_quadrotor_1_Vector_x, payload_quadrotor_2_Vector_x);

                        {
                            if (vect_T_Traj_lable[cnt_traj_T])
                            {
                                double t1 = ros::Time::now().toSec() - startTime;
                                while (vect_Traj_UAVT1[cnt_traj_T]._totalTime > t1)
                                {
                                    Eigen::Vector3d inputDesiredPos1 =
                                        vect_Traj_UAVT1[cnt_traj_T].getTrajectoryStates(t1, 0);
                                    Eigen::Vector3d inputDesiredVel1 =
                                        vect_Traj_UAVT1[cnt_traj_T].getTrajectoryStates(t1, 1);
                                    Eigen::Vector3d inputDesiredAcc1 =
                                        vect_Traj_UAVT1[cnt_traj_T].getTrajectoryStates(t1, 2);
                                    Eigen::Vector3d inputDesiredPos2 =
                                        vect_Traj_UAVT2[cnt_traj_T].getTrajectoryStates(t1, 0);
                                    Eigen::Vector3d inputDesiredVel2 =
                                        vect_Traj_UAVT2[cnt_traj_T].getTrajectoryStates(t1, 1);
                                    Eigen::Vector3d inputDesiredAcc2 =
                                        vect_Traj_UAVT2[cnt_traj_T].getTrajectoryStates(t1, 2);

                                    n.controller.Transition_Controller(
                                        inputDesiredPos1, inputDesiredVel1, inputDesiredAcc1, inputDesiredPos2,
                                        inputDesiredVel2, inputDesiredAcc2, inputDesiredPos);

                                    t1 = ros::Time::now().toSec() - startTime;

#ifdef VREP
                                    n.publishToVrep();
#endif
                                    ros::spinOnce();
                                    rate.sleep();
                                }
                            }

                            if (cnt_traj_T < vect_Traj_UAVT1.size())
                            {
                                startTime += vect_Traj_UAVT1[cnt_traj_T]._totalTime + 0.5;
                                cnt_traj_T = cnt_traj_T + 1;
                            }
                        }
                    }
                }
            }
#ifdef VREP
            n.publishToVrep();
#endif
        }
    }
    return 0;
}
