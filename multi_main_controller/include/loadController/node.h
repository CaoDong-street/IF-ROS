/**
 * @file node.h
 * @brief  Initialize the payload controller
 */

#ifndef LOADCONTROLLER_NODE_H
#define LOADCONTROLLER_NODE_H

#include "loadController/flightController.h"
#include "loadController/common_include.h"

namespace ptc {

class Node
{
    ros::NodeHandle nh;

    ros::Publisher pubRotorRevs;
    ros::Publisher pubRotorRevs1;

    ros::Publisher pubDesiredPos1;
    ros::Publisher pubFeedbackLoad1;

    ros::Publisher pubDesiredPos;
    ros::Publisher pubFeedbackLoad;
    ros::Publisher pubLoadDesiredPos;

    ros::Subscriber subPayload;
    ros::Subscriber subPayload1;

    ros::Subscriber subOdometry;
    ros::Subscriber subIMU;

    ros::Subscriber subOdometry1;
    ros::Subscriber subIMU1;

private:
    void callbackOdometry(const nav_msgs::OdometryConstPtr &odom) { controller.updateOdom(odom); }

    void callbackOdometry1(const nav_msgs::OdometryConstPtr &odom) { controller.updateOdom1(odom); }

    void callbackImu(const sensor_msgs::ImuConstPtr &imu) { controller.updateImu(imu); }

    void callbackImu1(const sensor_msgs::ImuConstPtr &imu) { controller.updateImu1(imu); }

    void callbackPayload(const geometry_msgs::Twist &load_pose) { controller.updatePayload(load_pose); }

    void callbackPayload1(const geometry_msgs::Twist &load_pose) { controller.updatePayload1(load_pose); }

    void callbackEncoder(const geometry_msgs::TransformStamped::ConstPtr &msg)
    {
        Eigen::Quaterniond quat;
        quat.x() = msg->transform.rotation.x;
        quat.y() = msg->transform.rotation.y;
        quat.z() = msg->transform.rotation.z;
        quat.w() = msg->transform.rotation.w;

        controller.feedback.cBodyRotationMatrix = quat.toRotationMatrix();
        controller.feedback.cBodyPosition.x() = msg->transform.translation.x;
        controller.feedback.cBodyPosition.y() = msg->transform.translation.y;
        controller.feedback.cBodyPosition.z() = msg->transform.translation.z;
    }

public:
    PayloadController controller;

    Node()
    {
        memset(&controller, 0, sizeof(controller));
        // publish information about UAV1
        pubDesiredPos = nh.advertise<nav_msgs::Odometry>("/desiredPos", 1);
        pubFeedbackLoad = nh.advertise<nav_msgs::Odometry>("/feedbackLoad", 1);
        // publish information about payload
        pubLoadDesiredPos = nh.advertise<nav_msgs::Odometry>("/desiredLoadPos", 1);
        // publish information about UAV2
        pubDesiredPos1 = nh.advertise<nav_msgs::Odometry>("/desiredPos1", 1);
        pubFeedbackLoad1 = nh.advertise<nav_msgs::Odometry>("/feedbackLoad1", 1);

#ifdef VREP

        subPayload = nh.subscribe("/payload", 1, &Node::callbackPayload, this);
        subPayload1 = nh.subscribe("/payload", 1, &Node::callbackPayload1, this);
        // subscribe information about UAV1
        subIMU = nh.subscribe("/imu", 1, &Node::callbackImu, this);
        subOdometry = nh.subscribe("/odom", 1, &Node::callbackOdometry, this);
        // subscribe information about UAV2
        subIMU1 = nh.subscribe("/imu0", 1, &Node::callbackImu1, this);
        subOdometry1 = nh.subscribe("/odom0", 1, &Node::callbackOdometry1, this);

        pubRotorRevs = nh.advertise<std_msgs::Float64MultiArray>("/rotorRevs", 1);
        pubRotorRevs1 = nh.advertise<std_msgs::Float64MultiArray>("/rotorRevs0", 1);
#endif
    }

    void publishToVrep()
    {
        Vec4 revs = controller.getRevs();
        Vec4 revs1 = controller.getRevs1();
        std_msgs::Float64MultiArray revsArray;
        std_msgs::Float64MultiArray revsArray1;
        for (int i = 0; i < 4; i++)
        {
            if (isnanl(revs[i]))
            {
                revs[i] = 0;
            }
        }
        for (int i = 0; i < 4; i++)
        {
            if (isnanl(revs1[i]))
            {
                revs1[i] = 0;
            }
        }

        revsArray.data = {revs.x(), revs.y(), revs.z(), revs.w(), 0};
        revsArray1.data = {revs1.x(), revs1.y(), revs1.z(), revs1.w(), 0};

        pubRotorRevs.publish(revsArray);
        pubRotorRevs1.publish(revsArray1);
        pubDesiredPos.publish(controller.desiredPos);
        pubFeedbackLoad.publish(controller.feedbackLoad);
        pubLoadDesiredPos.publish(controller.desiredLoadPos);
        pubDesiredPos1.publish(controller.desiredPos1);
        pubFeedbackLoad1.publish(controller.feedbackLoad1);
    }
};
} // namespace ptc

#endif // LOADCONTROLLER_NODE_H
