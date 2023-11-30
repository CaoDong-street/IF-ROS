/**
 * @file trajectory_generator_waypoint.h
 * @brief  Initialize the payload controller
 */

#ifndef _TRAJECTORY_GENERATOR_WAYPOINT_H_
#define _TRAJECTORY_GENERATOR_WAYPOINT_H_

#include <ros/ros.h>
#include <eigen3/Eigen/Eigen>
#include <vector>
#include <math.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

class TrajectoryGeneratorWaypoint
{
private:
    int d_order; // Minimized order
    double _Vel, _Acc;

public:
    Eigen::MatrixXd _waypoints; // waypoint
    bool isTraj = false;
    double _totalTime = 0;

    Eigen::MatrixXd _polyCoeff;
    Eigen::VectorXd _polyTime;            // Time per trajectory
    Eigen::VectorXd _polyTime_accumulate; // Time at the path point relative to the starting point
    /// Initialization
    TrajectoryGeneratorWaypoint();

    /// Custom Initialization
    TrajectoryGeneratorWaypoint(const double Vel, const double Acc, const int order);

    ~TrajectoryGeneratorWaypoint();

    Eigen::VectorXd get_dF(const int k, const int m, const Eigen::MatrixXd &Path,
                           const Eigen::MatrixXd &margin_constraint = Eigen::MatrixXd::Zero(2, 6));

    /// One-dimensional time allocation
    double timeAllocation_1D(const double dis);

    /// time allocation,Calculate the time used for each segment of the path
    void timeAllocation(const Eigen::MatrixXd &Path);

    double Factorial(const int x);

    /// solving for a polynomial time function of a trajectory
    void PolyQPGeneration(const Eigen::MatrixXd &Path,
                          const Eigen::MatrixXd &margin_constraint = Eigen::MatrixXd::Zero(2, 6));

    /// Determining position and its derivatives based on the polynomial time function
    Eigen::Vector3d getPolyStates(const int k, const double t_seg, const int order);

    /// generate trajectory
    bool trajGeneration(const nav_msgs::PathConstPtr &wp,
                        const Eigen::MatrixXd &margin_constraint = Eigen::MatrixXd::Zero(2, 6));

    /// generate trajectory
    bool trajGeneration(const nav_msgs::Path &wp,
                        const Eigen::MatrixXd &margin_constraint = Eigen::MatrixXd::Zero(2, 6));

    /// Determining position and its derivatives based on the polynomial time function
    Eigen::Vector3d getTrajectoryStates(const double time_from_start, const int order);
};

#endif
