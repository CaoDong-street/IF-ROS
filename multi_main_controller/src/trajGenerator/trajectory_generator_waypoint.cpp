#include "trajGenerator/trajectory_generator_waypoint.h"
#include <iostream>

using namespace std;
using namespace Eigen;

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint(const double Vel, const double Acc, const int order)
{
    _Vel = Vel;
    _Acc = Acc;
    d_order = order;
}

TrajectoryGeneratorWaypoint::TrajectoryGeneratorWaypoint()
{
    _Vel = 0.25;
    _Acc = 0.25;
    d_order = 4;
}

TrajectoryGeneratorWaypoint::~TrajectoryGeneratorWaypoint() {}

VectorXd TrajectoryGeneratorWaypoint::get_dF(const int k, const int m, const Eigen::MatrixXd &Path,
                                             const Eigen::MatrixXd &margin_constraint)
{
    int n_boundary_constraint = (m - 1) + 2 * d_order;
    VectorXd dF = VectorXd::Zero(n_boundary_constraint);
    for (int i = 0; i <= m; ++i)
    {
        if (i == 0)
        {
            dF(0) = Path(0, k);
            dF(1) = margin_constraint(0, k);
            dF(2) = margin_constraint(0, k + 3);
        }
        else if (i == m)
        {
            dF(n_boundary_constraint - d_order) = Path(i, k);
            dF(n_boundary_constraint - d_order + 1) = margin_constraint(1, k);
            dF(n_boundary_constraint - d_order + 2) = margin_constraint(1, k + 3);
        }
        else
        {
            dF(d_order + i - 1) = Path(i, k);
        }
    }
    return dF;
}

double TrajectoryGeneratorWaypoint::timeAllocation_1D(const double dis)
{
    double T = 0;
    if (dis <= _Vel * _Vel / _Acc)
    {
        T = 2 * sqrt(dis / _Acc); // acceleration
    }
    else
    {
        T = _Vel / _Acc + (dis) / _Vel; // acceleration and then constant velocity
    }
    return T;
}

void TrajectoryGeneratorWaypoint::timeAllocation(const Eigen::MatrixXd &Path)
{
    VectorXd time(Path.rows() - 1);
    VectorXd time_accumulate(Path.rows() - 1);
    _totalTime = 0;

    double delta_x = 0;
    double delta_y = 0;
    double delta_z = 0;

    for (int i = 0; i < time.size(); ++i)
    {
        delta_x = fabs(Path(i + 1, 0) - Path(i, 0));
        delta_y = fabs(Path(i + 1, 1) - Path(i, 1));
        delta_z = fabs(Path(i + 1, 2) - Path(i, 2));

        double dis = sqrt(delta_x * delta_x + delta_y * delta_y + delta_z * delta_z);
        time(i) = timeAllocation_1D(dis);

        _totalTime += time(i);
        time_accumulate(i) = _totalTime;
    }
    _polyTime = time;
    _polyTime_accumulate = time_accumulate;
}

double TrajectoryGeneratorWaypoint::Factorial(const int x)
{
    double fac = 1;
    for (int i = x; i > 0; i--) fac = fac * i;
    return fac;
}

void TrajectoryGeneratorWaypoint::PolyQPGeneration(const Eigen::MatrixXd &Path,
                                                   const Eigen::MatrixXd &margin_constraint)
{
    int p_order = 2 * d_order - 1; // the order of polynomial
    int p_num1d = p_order + 1;     // the number of variables in each segment

    int m = _polyTime.size(); // the number of segments

    //    getQ
    MatrixXd Q = MatrixXd::Zero(m * p_num1d, m * p_num1d);
    MatrixXd Qj;
    for (int j = 0; j <= m - 1; ++j)
    {
        Qj = MatrixXd::Zero(p_num1d, p_num1d);
        for (int i = d_order; i <= p_order; ++i)
        {
            for (int l = i; l <= p_order; ++l)
            {
                Qj(i, l) = Factorial(i) / Factorial(i - d_order) * Factorial(l) / Factorial(l - d_order) *
                           pow(_polyTime(j), i + l - p_order) / (i + l - p_order);

                Qj(l, i) = Qj(i, l);
            }
        }
        Q.block(j * p_num1d, j * p_num1d, p_num1d, p_num1d) = Qj;
    }

    //  getM
    Eigen::MatrixXd M = MatrixXd::Zero(m * p_num1d, m * p_num1d);
    for (int j = 0; j <= m - 1; ++j)
    {
        for (int k = 0; k <= d_order - 1; ++k)
        {
            M(k + j * p_num1d, k + j * p_num1d) = Factorial(k);
            for (int i = k; i <= p_order; ++i)
            {
                M(d_order + k + j * p_num1d, i + j * p_num1d) =
                    Factorial(i) / Factorial(i - k) * pow(_polyTime(j), i - k);
            }
        }
    }

    //    getCt
    Eigen::MatrixXd Ct_start = MatrixXd::Zero(d_order, d_order * (m + 1));

    Ct_start.block(0, 0, d_order, d_order) = MatrixXd::Identity(d_order, d_order);

    //  Ct  for middle point
    Eigen::MatrixXd Ct_mid = MatrixXd::Zero(2 * d_order * (m - 1), d_order * (m + 1));
    Eigen::MatrixXd Cj;
    double start_idx_2 = 0;
    double start_idx_1 = 0;
    for (int j = 0; j <= m - 2; ++j)
    {
        Cj = MatrixXd::Zero(d_order, d_order * (m + 1));
        Cj(0, d_order + j) = 1;
        start_idx_2 = 2 * d_order + m - 1 + (d_order - 1) * j;
        Cj.block(1, start_idx_2, d_order - 1, d_order - 1) = MatrixXd::Identity(d_order - 1, d_order - 1);
        start_idx_1 = 2 * d_order * j;
        Eigen::MatrixXd Cj_adjacen(2 * d_order, d_order * (m + 1));
        Cj_adjacen << Cj, Cj;
        Ct_mid.block(start_idx_1, 0, 2 * d_order, d_order * (m + 1)) = Cj_adjacen;
    }

    //  Ct  for end point
    Eigen::MatrixXd Ct_end = MatrixXd::Zero(d_order, d_order * (m + 1));
    Ct_end.block(0, d_order + m - 1, d_order, d_order) = MatrixXd::Identity(d_order, d_order);

    Eigen::MatrixXd Ct(2 * d_order * m, d_order * (m + 1));
    Ct << Ct_start, Ct_mid, Ct_end;

    Eigen::MatrixXd C = Ct.transpose();
    Eigen::MatrixXd R = C * M.inverse().transpose() * Q * M.inverse() * Ct;

    int n_boundary_constraint = (m - 1) + 2 * d_order;
    int n_continuity_constraint = (d_order - 1) * (m - 1);

    Eigen::MatrixXd R_pp =
        R.block(n_boundary_constraint, n_boundary_constraint, n_continuity_constraint, n_continuity_constraint);
    Eigen::MatrixXd R_fp = R.block(0, n_boundary_constraint, n_boundary_constraint, n_continuity_constraint);

    VectorXd dF_x = get_dF(0, m, Path, margin_constraint);
    VectorXd dF_y = get_dF(1, m, Path, margin_constraint);
    VectorXd dF_z = get_dF(2, m, Path, margin_constraint);

    VectorXd dP_x = -R_pp.inverse() * R_fp.transpose() * dF_x;
    VectorXd dP_y = -R_pp.inverse() * R_fp.transpose() * dF_y;
    VectorXd dP_z = -R_pp.inverse() * R_fp.transpose() * dF_z;

    VectorXd d_x(dF_x.size() + dP_x.size(), 1);
    d_x << dF_x, dP_x;

    VectorXd d_y(dF_y.size() + dP_y.size());
    d_y << dF_y, dP_y;

    VectorXd d_z(dF_z.size() + dP_z.size());
    d_z << dF_z, dP_z;

    VectorXd Px(p_num1d * m), Py(p_num1d * m), Pz(p_num1d * m);

    VectorXd P_x = M.inverse() * Ct * d_x;
    VectorXd P_y = M.inverse() * Ct * d_y;
    VectorXd P_z = M.inverse() * Ct * d_z;

    MatrixXd PolyCoeff = MatrixXd::Zero(m, 3 * p_num1d); // position(x,y,z), so we need (3 * p_num1d) coefficients
    for (int i = 0; i < m; ++i)
    {
        PolyCoeff.block(i, 0, 1, p_num1d) = P_x.block(i * p_num1d, 0, p_num1d, 1).transpose();
    }
    for (int i = 0; i < m; ++i)
    {
        PolyCoeff.block(i, p_num1d, 1, p_num1d) = P_y.block(i * p_num1d, 0, p_num1d, 1).transpose();
    }
    for (int i = 0; i < m; ++i)
    {
        PolyCoeff.block(i, 2 * p_num1d, 1, p_num1d) = P_z.block(i * p_num1d, 0, p_num1d, 1).transpose();
    }

    _polyCoeff = PolyCoeff;
}

Vector3d TrajectoryGeneratorWaypoint::getPolyStates(const int k, const double t_seg, const int order)
{
    int _poly_num1D = _polyCoeff.cols() / 3;

    Vector3d ret;

    for (int dim = 0; dim < 3; dim++)
    {
        VectorXd coeff = (_polyCoeff.row(k)).segment(dim * _poly_num1D, _poly_num1D);
        VectorXd time = VectorXd::Zero(_poly_num1D);

        for (int j = order; j < _poly_num1D; j++)
            if (j == 0)
                time(j) = Factorial(j) * 1.0;
            else
                time(j) = Factorial(j) / Factorial(j - order) * pow(t_seg, j - order);

        ret(dim) = coeff.dot(time);
    }

    return ret;
}

bool TrajectoryGeneratorWaypoint::trajGeneration(const nav_msgs::PathConstPtr &wp,
                                                 const Eigen::MatrixXd &margin_constraint)
{
    bool isGenerated = false;
    std::vector<Eigen::Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int)wp->poses.size(); k++)
    {
        Eigen::Vector3d pt(wp->poses[k].pose.position.x, wp->poses[k].pose.position.y, wp->poses[k].pose.position.z);
        wp_list.push_back(pt);

        if (wp->poses[k].pose.position.z < 0.0) break;
    }

    Eigen::MatrixXd waypoints(wp_list.size(), 3);

    for (int k = 0; k < (int)wp_list.size(); k++) waypoints.row(k) = wp_list[k];

    _waypoints = waypoints;

    timeAllocation(_waypoints);

    PolyQPGeneration(_waypoints, margin_constraint);

    isGenerated = true;
    return isGenerated;
}

bool TrajectoryGeneratorWaypoint::trajGeneration(const nav_msgs::Path &wp, const Eigen::MatrixXd &margin_constraint)
{
    bool isGenerated = false;
    std::vector<Eigen::Vector3d> wp_list;
    wp_list.clear();

    for (int k = 0; k < (int)wp.poses.size(); k++)
    {
        Eigen::Vector3d pt(wp.poses[k].pose.position.x, wp.poses[k].pose.position.y, wp.poses[k].pose.position.z);
        wp_list.push_back(pt);

        if (wp.poses[k].pose.position.z < 0.0) break;
    }

    Eigen::MatrixXd waypoints(wp_list.size(), 3);
    for (int k = 0; k < (int)wp_list.size(); k++) waypoints.row(k) = wp_list[k];

    _waypoints = waypoints;

    timeAllocation(_waypoints);

    PolyQPGeneration(_waypoints, margin_constraint);

    isGenerated = true;
    return isGenerated;
}

Vector3d TrajectoryGeneratorWaypoint::getTrajectoryStates(const double time_from_start, const int order)
{
    double t_init = 0;
    double t_seg = 0;
    int seg_idx = 0; // Find the segment of the trajectory that corresponds to the time

    for (int i = 0; i < _polyTime.size(); i++)
    {
        if (time_from_start >= t_init + _polyTime(i))
        {
            t_init += _polyTime(i);
        }
        else
        {
            t_seg = time_from_start - t_init;
            seg_idx = i;
            break;
        }
    }

    Vector3d states = getPolyStates(seg_idx, t_seg, order);

    return states;
}
