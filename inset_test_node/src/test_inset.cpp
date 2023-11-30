#include "bag_reader.hpp"
#include "data_ros_utils.h"
#include <ros/ros.h>
#include "std_msgs/Bool.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include "inset_util/inset.h"

Eigen::VectorXd quadrotor_1_old(2);
Eigen::VectorXd quadrotor_2_old(2);
Eigen::VectorXd payload_old(2);
bool callback_lable;
bool payload_waypoint_receive_lable;
bool quadrotor_1_waypoint_receive_lable;
bool quadrotor_2_waypoint_receive_lable;
void move_formation(inset::formation& formation, const Eigen::Vector2d move_center)
{
    Eigen::VectorXd quadrotor_1 = formation.get_quadrotor_1();
    Eigen::VectorXd quadrotor_2 = formation.get_quadrotor_2();
    Eigen::VectorXd payload = formation.get_payload();
    Eigen::MatrixXd center_x_numerator(3, 3);
    Eigen::MatrixXd center_x_denominator(3, 3);
    Eigen::MatrixXd center_y_numerator(3, 3);
    Eigen::MatrixXd center_y_denominator(3, 3);
    Eigen::VectorXd xq1_center(2);
    Eigen::VectorXd xq2_center(2);
    Eigen::VectorXd xp_center(2);
    center_x_numerator << 1, pow(quadrotor_1(0), 2) + pow(quadrotor_1(1), 2), quadrotor_1(1), 1,
        pow(quadrotor_2(0), 2) + pow(quadrotor_2(1), 2), quadrotor_2(1), 1, pow(payload(0), 2) + pow(payload(1), 2),
        payload(1);
    center_x_denominator << 1, quadrotor_1(0), quadrotor_1(1), 1, quadrotor_2(0), quadrotor_2(1), 1, payload(0),
        payload(1);
    center_y_numerator << 1, quadrotor_1(0), pow(quadrotor_1(0), 2) + pow(quadrotor_1(1), 2), 1, quadrotor_2(0),
        pow(quadrotor_2(0), 2) + pow(quadrotor_2(1), 2), 1, payload(0), pow(payload(0), 2) + pow(payload(1), 2);
    center_y_denominator << 1, quadrotor_1(0), quadrotor_1(1), 1, quadrotor_2(0), quadrotor_2(1), 1, payload(0),
        payload(1);
    double center_x = 0.5 * center_x_numerator.determinant() / center_x_denominator.determinant();
    double center_y = 0.5 * center_y_numerator.determinant() / center_y_denominator.determinant();
    xq1_center << quadrotor_1(0) - center_x, quadrotor_1(1) - center_y;
    xq2_center << quadrotor_2(0) - center_x, quadrotor_2(1) - center_y;
    xp_center << payload(0) - center_x, payload(1) - center_y;
    quadrotor_1 = move_center + xq1_center;
    quadrotor_2 = move_center + xq2_center;
    payload = move_center + xp_center;
    formation.set_quadrotor_1(quadrotor_1);
    formation.set_quadrotor_2(quadrotor_2);
    formation.set_payload(payload);
}

void iris_quadrotor_1_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    quadrotor_1_old(0) = msg->pose.pose.position.x;
    quadrotor_1_old(1) = msg->pose.pose.position.y;
    callback_lable = true;
}

void iris_quadrotor_2_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    quadrotor_2_old(0) = msg->pose.pose.position.x;
    quadrotor_2_old(1) = msg->pose.pose.position.y;
    callback_lable = true;
}

void payload_iris_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    payload_old(0) = msg->pose.pose.position.x;
    payload_old(1) = msg->pose.pose.position.y;
    callback_lable = true;
}

void quadrotor_1_waypoint_receive_callback(const std_msgs::Bool& msg) { quadrotor_1_waypoint_receive_lable = msg.data; }

void quadrotor_2_waypoint_receive_callback(const std_msgs::Bool& msg) { quadrotor_2_waypoint_receive_lable = msg.data; }

void payload_waypoint_receive_callback(const std_msgs::Bool& msg) { payload_waypoint_receive_lable = msg.data; }

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_inset");
    ros::NodeHandle nh("~");

    ros::Publisher _iris_pub_quadrotor_1 = nh.advertise<nav_msgs::Odometry>("iris_out_quadrotor_1", 1);
    ros::Publisher _iris_pub_quadrotor_2 = nh.advertise<nav_msgs::Odometry>("iris_out_quadrotor_2", 1);
    ros::Publisher _iris_pub_payload = nh.advertise<nav_msgs::Odometry>("iris_out_payload", 1);

    ros::Publisher _waypoint_pub_quadrotor_1 = nh.advertise<geometry_msgs::PoseStamped>("/quadrotor_1_waypoint", 1);
    ros::Publisher _waypoint_pub_quadrotor_2 = nh.advertise<geometry_msgs::PoseStamped>("/quadrotor_2_waypoint", 1);
    ros::Publisher _waypoint_pub_payload = nh.advertise<geometry_msgs::PoseStamped>("/payload_waypoint", 1);

    ros::Subscriber sub_iris_quadrotor_1 = nh.subscribe("iris_in_quadrotor_1", 100, iris_quadrotor_1_callback);
    ros::Subscriber sub_iris_quadrotor_2 = nh.subscribe("iris_in_quadrotor_2", 100, iris_quadrotor_2_callback);
    ros::Subscriber sub_iris_payload = nh.subscribe("iris_in_payload", 100, payload_iris_callback);

    ros::Subscriber sub_quadrotor_1_waypoint_receive =
        nh.subscribe("/quadrotor_1_waypoint_receive_lable", 100, quadrotor_1_waypoint_receive_callback);
    ros::Subscriber sub_quadrotor_2_waypoint_receive =
        nh.subscribe("/quadrotor_2_waypoint_receive_lable", 100, quadrotor_2_waypoint_receive_callback);
    ros::Subscriber sub_payload_waypoint_receive =
        nh.subscribe("/payload_waypoint_receive_lable", 100, payload_waypoint_receive_callback);

    std::string xyz_directory;
    bool save_xyz;
    bool start_lable;
    bool end_lable;
    double forward_length;
    double backward_length;
    std::vector<double> obs_origin_Param;
    std::vector<double> initial_point_Param;
    std::vector<double> obs_attitude_Param_x;
    std::vector<double> obs_attitude_Param_z;
    nh.param("xyz_directory", xyz_directory, std::string("xyz_directory"));
    nh.param("save_xyz", save_xyz, false);
    nh.param("start_lable", start_lable, true);
    nh.param("end_lable", end_lable, false);
    nh.param("forward_length", forward_length, 1.0);
    nh.param("backward_length", backward_length, 1.0);
    nh.getParam("obs_origin_Param", obs_origin_Param);
    nh.getParam("initial_point_Param", initial_point_Param);
    nh.getParam("obs_attitude_Param_x", obs_attitude_Param_x);
    nh.getParam("obs_attitude_Param_z", obs_attitude_Param_z);

    Vec3f obs_origin_point;
    Vec3f obs_origin_point_map;
    Vec3f obs_attitude_x;
    obs_attitude_x << obs_attitude_Param_x[0], obs_attitude_Param_x[1], obs_attitude_Param_x[2];
    obs_attitude_x.normalize();
    Vec3f obs_attitude_z;
    obs_attitude_z << obs_attitude_Param_z[0], obs_attitude_Param_z[1], obs_attitude_Param_z[2];
    obs_attitude_z.normalize();
    Mat3f obs_attitude;
    Mat3f obs_attitude_map;
    obs_origin_point << obs_origin_Param[0], obs_origin_Param[1], obs_origin_Param[2];
    obs_origin_point_map << 0, 0, 0;
    obs_attitude_map << 1, 0, 0, 0, 0, 1, 0, -1, 0;
    obs_attitude.col(0) = obs_attitude_x;
    obs_attitude.col(1) = obs_attitude_z.cross(obs_attitude_x);
    obs_attitude.col(2) = obs_attitude_z;

    iris::IRISProblem problem(2);
    Eigen::Vector2d initial_point(initial_point_Param[0], initial_point_Param[1]);
    problem.setSeedPoint(initial_point);

    std::string file_name, topic_name;
    nh.param("bag_file", file_name, std::string("cloud"));
    nh.param("bag_topic", topic_name, std::string("cloud"));
    sensor_msgs::PointCloud cloud_map = read_bag<sensor_msgs::PointCloud>(file_name, topic_name);
    cloud_map.header.frame_id = "map";

    vec_Vec3f obs_map = InsetROS::cloud_to_vec(cloud_map);
    vec_Vec3f obs =
        InsetROS::inverse_mapping_vec(obs_map, obs_origin_point, obs_attitude, obs_origin_point_map, obs_attitude_map);
    sensor_msgs::PointCloud cloud = InsetROS::vec_to_cloud(obs);
    cloud.header.frame_id = "map";
    if (save_xyz)
    {
        std::ofstream xyz(xyz_directory);
        for (unsigned int i = 0; i < cloud.points.size(); i++)
        {
            xyz << cloud.points[i].x << " " << cloud.points[i].y << " " << cloud.points[i].z << std::endl;
        }
#ifndef NDEBUG
        ROS_INFO("xyz saved!");
#endif
    }

    vec_Vec2f obs_map_2d;
    for (const auto& it : obs_map) obs_map_2d.push_back(it.topRows<2>());
    for (const auto& it : obs_map_2d) problem.addObstacle(it);
    iris::IRISOptions options;
    iris::IRISRegion region = inflate_region(problem, options);
    Eigen::MatrixXd n_show = region.polyhedron.getn();
    Eigen::MatrixXd p_show = region.polyhedron.getp();
    Eigen::MatrixXd IRISA = region.polyhedron.getA();
    Eigen::VectorXd IRISb = region.polyhedron.getB();

    double length;
    double vector_1_2_Length;
    std::vector<double> limit_bound_Param;
    std::vector<double> limit_rate_Param;
    nh.getParam("limit_bound_Param", limit_bound_Param);
    nh.getParam("limit_rate_Param", limit_rate_Param);
    nh.getParam("length", length);
    nh.getParam("vector_1_2_Length", vector_1_2_Length);
    Eigen::Vector4d limit_bound(limit_bound_Param[0], limit_bound_Param[1], limit_bound_Param[2], limit_bound_Param[3]);
    Eigen::Vector2d limit_rate(limit_rate_Param[0], limit_rate_Param[1]);

    quadrotor_1_old(0) = vector_1_2_Length;
    quadrotor_1_old(1) = 0;
    quadrotor_2_old(0) = 0;
    quadrotor_2_old(1) = 0;
    payload_old(0) = (quadrotor_1_old(0) + quadrotor_2_old(0)) / 2.0;
    payload_old(1) = -sqrt(pow(length, 2) - pow((quadrotor_1_old(0) + quadrotor_2_old(0)) / 2.0, 2));

    callback_lable = false;
    payload_waypoint_receive_lable = false;
    quadrotor_1_waypoint_receive_lable = false;
    quadrotor_2_waypoint_receive_lable = false;
    ros::Rate rate(100);
    bool status = ros::ok();
    if (!start_lable)
    {
        while (status && (!callback_lable))
        {
            ros::spinOnce();
            status = ros::ok();
            rate.sleep();
        }
    }

    Eigen::MatrixXd IRISC = region.ellipsoid.getC();
    Eigen::VectorXd IRISD = region.ellipsoid.getD();
    inset::formation inset_formation(quadrotor_1_old, quadrotor_2_old, payload_old);

    inset_formation.construct_formation(IRISA, IRISb, IRISC, IRISD, limit_bound, limit_rate, length, initial_point);

    auto change_lable = inset_formation.get_change_lable();
    if (!change_lable)
    {
        Eigen::Vector2d move_center(initial_point(0) * 3, initial_point(1));
        move_formation(inset_formation, move_center);
    }
    auto quadrotor_1_point = inset_formation.get_quadrotor_1();
    auto quadrotor_2_point = inset_formation.get_quadrotor_2();
    auto payload_point = inset_formation.get_payload();
    Vec3f quadrotor_1_pos_map = Vec3f(quadrotor_1_point(0), quadrotor_1_point(1), 0);
    Vec3f quadrotor_2_pos_map = Vec3f(quadrotor_2_point(0), quadrotor_2_point(1), 0);
    Vec3f payload_pos_map = Vec3f(payload_point(0), payload_point(1), 0);
    Vec3f quadrotor_1_pos = InsetROS::inverse_mapping_vec(quadrotor_1_pos_map, obs_origin_point, obs_attitude,
                                                          obs_origin_point_map, obs_attitude_map);
    Vec3f quadrotor_2_pos = InsetROS::inverse_mapping_vec(quadrotor_2_pos_map, obs_origin_point, obs_attitude,
                                                          obs_origin_point_map, obs_attitude_map);
    Vec3f payload_pos = InsetROS::inverse_mapping_vec(payload_pos_map, obs_origin_point, obs_attitude,
                                                      obs_origin_point_map, obs_attitude_map);

    Vec3f quadrotor_1_pos_map_forward = Vec3f(quadrotor_1_point(0), quadrotor_1_point(1), forward_length);
    Vec3f quadrotor_2_pos_map_forward = Vec3f(quadrotor_2_point(0), quadrotor_2_point(1), forward_length);
    Vec3f payload_pos_map_forward = Vec3f(payload_point(0), payload_point(1), forward_length);
    Vec3f quadrotor_1_pos_forward = InsetROS::inverse_mapping_vec(quadrotor_1_pos_map_forward, obs_origin_point,
                                                                  obs_attitude, obs_origin_point_map, obs_attitude_map);
    Vec3f quadrotor_2_pos_forward = InsetROS::inverse_mapping_vec(quadrotor_2_pos_map_forward, obs_origin_point,
                                                                  obs_attitude, obs_origin_point_map, obs_attitude_map);
    Vec3f payload_pos_forward = InsetROS::inverse_mapping_vec(payload_pos_map_forward, obs_origin_point, obs_attitude,
                                                              obs_origin_point_map, obs_attitude_map);

    Vec3f quadrotor_1_pos_map_backward = Vec3f(quadrotor_1_point(0), quadrotor_1_point(1), -backward_length);
    Vec3f quadrotor_2_pos_map_backward = Vec3f(quadrotor_2_point(0), quadrotor_2_point(1), -backward_length);
    Vec3f payload_pos_map_backward = Vec3f(payload_point(0), payload_point(1), -backward_length);
    Vec3f quadrotor_1_pos_backward = InsetROS::inverse_mapping_vec(
        quadrotor_1_pos_map_backward, obs_origin_point, obs_attitude, obs_origin_point_map, obs_attitude_map);
    Vec3f quadrotor_2_pos_backward = InsetROS::inverse_mapping_vec(
        quadrotor_2_pos_map_backward, obs_origin_point, obs_attitude, obs_origin_point_map, obs_attitude_map);
    Vec3f payload_pos_backward = InsetROS::inverse_mapping_vec(payload_pos_map_backward, obs_origin_point, obs_attitude,
                                                               obs_origin_point_map, obs_attitude_map);

    geometry_msgs::PoseStamped payload_waypoint_backward, payload_waypoint_forward, payload_waypoint;
    geometry_msgs::PoseStamped quadrotor_1_waypoint_backward, quadrotor_1_waypoint_forward, quadrotor_1_waypoint;
    geometry_msgs::PoseStamped quadrotor_2_waypoint_backward, quadrotor_2_waypoint_forward, quadrotor_2_waypoint;

    payload_waypoint_forward.header.seq = 0;
    payload_waypoint_forward.header.stamp = ros::Time::now();
    payload_waypoint_forward.header.frame_id = "map";
    payload_waypoint_forward.pose.position.x = payload_pos_forward(0);
    payload_waypoint_forward.pose.position.y = payload_pos_forward(1);
    payload_waypoint_forward.pose.position.z = payload_pos_forward(2);
    payload_waypoint_forward.pose.orientation.w = 1.0;
    payload_waypoint_forward.pose.orientation.x = 0.0;
    payload_waypoint_forward.pose.orientation.y = 0.0;
    payload_waypoint_forward.pose.orientation.z = 0.0;

    payload_waypoint.header.seq = 0;
    payload_waypoint.header.stamp = ros::Time::now();
    payload_waypoint.header.frame_id = "map";
    payload_waypoint.pose.position.x = payload_pos(0);
    payload_waypoint.pose.position.y = payload_pos(1);
    payload_waypoint.pose.position.z = payload_pos(2);
    payload_waypoint.pose.orientation.w = 1.0;
    payload_waypoint.pose.orientation.x = 0.0;
    payload_waypoint.pose.orientation.y = 0.0;
    payload_waypoint.pose.orientation.z = 0.0;

    payload_waypoint_backward.header.seq = 0;
    payload_waypoint_backward.header.stamp = ros::Time::now();
    payload_waypoint_backward.header.frame_id = "map";
    payload_waypoint_backward.pose.position.x = payload_pos_backward(0);
    payload_waypoint_backward.pose.position.y = payload_pos_backward(1);
    payload_waypoint_backward.pose.position.z = payload_pos_backward(2);
    payload_waypoint_backward.pose.orientation.w = 1.0;
    payload_waypoint_backward.pose.orientation.x = 0.0;
    payload_waypoint_backward.pose.orientation.y = 0.0;
    payload_waypoint_backward.pose.orientation.z = 0.0;

    quadrotor_1_waypoint_forward.header.seq = 0;
    quadrotor_1_waypoint_forward.header.stamp = ros::Time::now();
    quadrotor_1_waypoint_forward.header.frame_id = "map";
    quadrotor_1_waypoint_forward.pose.position.x = quadrotor_1_pos_forward(0);
    quadrotor_1_waypoint_forward.pose.position.y = quadrotor_1_pos_forward(1);
    quadrotor_1_waypoint_forward.pose.position.z = quadrotor_1_pos_forward(2);
    quadrotor_1_waypoint_forward.pose.orientation.w = 1.0;
    quadrotor_1_waypoint_forward.pose.orientation.x = 0.0;
    quadrotor_1_waypoint_forward.pose.orientation.y = 0.0;
    quadrotor_1_waypoint_forward.pose.orientation.z = 0.0;

    quadrotor_1_waypoint.header.seq = 0;
    quadrotor_1_waypoint.header.stamp = ros::Time::now();
    quadrotor_1_waypoint.header.frame_id = "map";
    quadrotor_1_waypoint.pose.position.x = quadrotor_1_pos(0);
    quadrotor_1_waypoint.pose.position.y = quadrotor_1_pos(1);
    quadrotor_1_waypoint.pose.position.z = quadrotor_1_pos(2);
    quadrotor_1_waypoint.pose.orientation.w = 1.0;
    quadrotor_1_waypoint.pose.orientation.x = 0.0;
    quadrotor_1_waypoint.pose.orientation.y = 0.0;
    quadrotor_1_waypoint.pose.orientation.z = 0.0;

    quadrotor_1_waypoint_backward.header.seq = 0;
    quadrotor_1_waypoint_backward.header.stamp = ros::Time::now();
    quadrotor_1_waypoint_backward.header.frame_id = "map";
    quadrotor_1_waypoint_backward.pose.position.x = quadrotor_1_pos_backward(0);
    quadrotor_1_waypoint_backward.pose.position.y = quadrotor_1_pos_backward(1);
    quadrotor_1_waypoint_backward.pose.position.z = quadrotor_1_pos_backward(2);
    quadrotor_1_waypoint_backward.pose.orientation.w = 1.0;
    quadrotor_1_waypoint_backward.pose.orientation.x = 0.0;
    quadrotor_1_waypoint_backward.pose.orientation.y = 0.0;
    quadrotor_1_waypoint_backward.pose.orientation.z = 0.0;

    quadrotor_2_waypoint_forward.header.seq = 0;
    quadrotor_2_waypoint_forward.header.stamp = ros::Time::now();
    quadrotor_2_waypoint_forward.header.frame_id = "map";
    quadrotor_2_waypoint_forward.pose.position.x = quadrotor_2_pos_forward(0);
    quadrotor_2_waypoint_forward.pose.position.y = quadrotor_2_pos_forward(1);
    quadrotor_2_waypoint_forward.pose.position.z = quadrotor_2_pos_forward(2);
    quadrotor_2_waypoint_forward.pose.orientation.w = 1.0;
    quadrotor_2_waypoint_forward.pose.orientation.x = 0.0;
    quadrotor_2_waypoint_forward.pose.orientation.y = 0.0;
    quadrotor_2_waypoint_forward.pose.orientation.z = 0.0;

    quadrotor_2_waypoint.header.seq = 0;
    quadrotor_2_waypoint.header.stamp = ros::Time::now();
    quadrotor_2_waypoint.header.frame_id = "map";
    quadrotor_2_waypoint.pose.position.x = quadrotor_2_pos(0);
    quadrotor_2_waypoint.pose.position.y = quadrotor_2_pos(1);
    quadrotor_2_waypoint.pose.position.z = quadrotor_2_pos(2);
    quadrotor_2_waypoint.pose.orientation.w = 1.0;
    quadrotor_2_waypoint.pose.orientation.x = 0.0;
    quadrotor_2_waypoint.pose.orientation.y = 0.0;
    quadrotor_2_waypoint.pose.orientation.z = 0.0;

    quadrotor_2_waypoint_backward.header.seq = 0;
    quadrotor_2_waypoint_backward.header.stamp = ros::Time::now();
    quadrotor_2_waypoint_backward.header.frame_id = "map";
    quadrotor_2_waypoint_backward.pose.position.x = quadrotor_2_pos_backward(0);
    quadrotor_2_waypoint_backward.pose.position.y = quadrotor_2_pos_backward(1);
    quadrotor_2_waypoint_backward.pose.position.z = quadrotor_2_pos_backward(2);
    quadrotor_2_waypoint_backward.pose.orientation.w = 1.0;
    quadrotor_2_waypoint_backward.pose.orientation.x = 0.0;
    quadrotor_2_waypoint_backward.pose.orientation.y = 0.0;
    quadrotor_2_waypoint_backward.pose.orientation.z = 0.0;

    payload_waypoint_receive_lable = false;

    while (status && (!payload_waypoint_receive_lable))
    {
        ros::spinOnce();
        if (!payload_waypoint_receive_lable)
        {
            _waypoint_pub_payload.publish(payload_waypoint_forward);
            status = ros::ok();
            rate.sleep();
        }
    }
    payload_waypoint_receive_lable = false;

    while (status && (!payload_waypoint_receive_lable))
    {
        ros::spinOnce();
        if (!payload_waypoint_receive_lable)
        {
            _waypoint_pub_payload.publish(payload_waypoint);
            status = ros::ok();
            rate.sleep();
        }
    }

    payload_waypoint_receive_lable = false;

    while (status && (!payload_waypoint_receive_lable))
    {
        ros::spinOnce();
        if (!payload_waypoint_receive_lable)
        {
            _waypoint_pub_payload.publish(payload_waypoint_backward);
            status = ros::ok();
            rate.sleep();
        }
    }

    quadrotor_1_waypoint_receive_lable = false;

    while (status && (!quadrotor_1_waypoint_receive_lable))
    {
        ros::spinOnce();
        if (!quadrotor_1_waypoint_receive_lable)
        {
            _waypoint_pub_quadrotor_1.publish(quadrotor_1_waypoint_forward);
            status = ros::ok();
            rate.sleep();
        }
    }

    quadrotor_1_waypoint_receive_lable = false;

    while (status && (!quadrotor_1_waypoint_receive_lable))
    {
        ros::spinOnce();
        if (!quadrotor_1_waypoint_receive_lable)
        {
            _waypoint_pub_quadrotor_1.publish(quadrotor_1_waypoint);
            status = ros::ok();
            rate.sleep();
        }
    }

    quadrotor_1_waypoint_receive_lable = false;

    while (status && (!quadrotor_1_waypoint_receive_lable))
    {
        ros::spinOnce();
        if (!quadrotor_1_waypoint_receive_lable)
        {
            _waypoint_pub_quadrotor_1.publish(quadrotor_1_waypoint_backward);
            status = ros::ok();
            rate.sleep();
        }
    }
    quadrotor_2_waypoint_receive_lable = false;

    while (status && (!quadrotor_2_waypoint_receive_lable))
    {
        ros::spinOnce();
        if (!quadrotor_2_waypoint_receive_lable)
        {
            _waypoint_pub_quadrotor_2.publish(quadrotor_2_waypoint_forward);
            status = ros::ok();
            rate.sleep();
        }
    }

    quadrotor_2_waypoint_receive_lable = false;

    while (status && (!quadrotor_2_waypoint_receive_lable))
    {
        ros::spinOnce();
        if (!quadrotor_2_waypoint_receive_lable)
        {
            _waypoint_pub_quadrotor_2.publish(quadrotor_2_waypoint);
            status = ros::ok();
            rate.sleep();
        }
    }

    quadrotor_2_waypoint_receive_lable = false;

    while (status && (!quadrotor_2_waypoint_receive_lable))
    {
        ros::spinOnce();
        if (!quadrotor_2_waypoint_receive_lable)
        {
            _waypoint_pub_quadrotor_2.publish(quadrotor_2_waypoint_backward);
            status = ros::ok();
            rate.sleep();
        }
    }

    if (end_lable)
    {
        geometry_msgs::PoseStamped payload_waypoint;
        geometry_msgs::PoseStamped quadrotor_1_waypoint;
        geometry_msgs::PoseStamped quadrotor_2_waypoint;

        payload_waypoint_receive_lable = false;
        quadrotor_1_waypoint_receive_lable = false;
        quadrotor_2_waypoint_receive_lable = false;

        payload_waypoint.header.seq = 0;
        payload_waypoint.header.stamp = ros::Time::now();
        payload_waypoint.header.frame_id = "map";
        payload_waypoint.pose.position.x = 0.0;
        payload_waypoint.pose.position.y = 0.0;
        payload_waypoint.pose.position.z = -5.0;
        payload_waypoint.pose.orientation.w = 1.0;
        payload_waypoint.pose.orientation.x = 0.0;
        payload_waypoint.pose.orientation.y = 0.0;
        payload_waypoint.pose.orientation.z = 0.0;

        quadrotor_1_waypoint.header.seq = 0;
        quadrotor_1_waypoint.header.stamp = ros::Time::now();
        quadrotor_1_waypoint.header.frame_id = "map";
        quadrotor_1_waypoint.pose.position.x = 0.0;
        quadrotor_1_waypoint.pose.position.y = 0.0;
        quadrotor_1_waypoint.pose.position.z = -5.0;
        quadrotor_1_waypoint.pose.orientation.w = 1.0;
        quadrotor_1_waypoint.pose.orientation.x = 0.0;
        quadrotor_1_waypoint.pose.orientation.y = 0.0;
        quadrotor_1_waypoint.pose.orientation.z = 0.0;

        quadrotor_2_waypoint.header.seq = 0;
        quadrotor_2_waypoint.header.stamp = ros::Time::now();
        quadrotor_2_waypoint.header.frame_id = "map";
        quadrotor_2_waypoint.pose.position.x = 0.0;
        quadrotor_2_waypoint.pose.position.y = 0.0;
        quadrotor_2_waypoint.pose.position.z = -5.0;
        quadrotor_2_waypoint.pose.orientation.w = 1.0;
        quadrotor_2_waypoint.pose.orientation.x = 0.0;
        quadrotor_2_waypoint.pose.orientation.y = 0.0;
        quadrotor_2_waypoint.pose.orientation.z = 0.0;

        while (status && (!payload_waypoint_receive_lable))
        {
            ros::spinOnce();
            if (!payload_waypoint_receive_lable)
            {
                _waypoint_pub_payload.publish(payload_waypoint);
                status = ros::ok();
                rate.sleep();
            }
        }

        while (status && (!quadrotor_1_waypoint_receive_lable))
        {
            ros::spinOnce();
            if (!quadrotor_1_waypoint_receive_lable)
            {
                _waypoint_pub_quadrotor_1.publish(quadrotor_1_waypoint);
                status = ros::ok();
                rate.sleep();
            }
        }

        while (status && (!quadrotor_2_waypoint_receive_lable))
        {
            ros::spinOnce();
            if (!quadrotor_2_waypoint_receive_lable)
            {
                _waypoint_pub_quadrotor_2.publish(quadrotor_2_waypoint);
                status = ros::ok();
                rate.sleep();
            }
        }
    }

    while (status)
    {
        nav_msgs::Odometry iris_quadrotor_1;
        nav_msgs::Odometry iris_quadrotor_2;
        nav_msgs::Odometry iris_payload;

        _iris_pub_quadrotor_1.publish(iris_quadrotor_1);
        _iris_pub_quadrotor_2.publish(iris_quadrotor_2);
        _iris_pub_payload.publish(iris_payload);
        ros::spinOnce();
        status = ros::ok();
        rate.sleep();
    }
    return 0;
}
