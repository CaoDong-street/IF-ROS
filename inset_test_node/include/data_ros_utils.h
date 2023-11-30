/**
 * @file data_ros_utils.h
 * @brief Processing of data for data conversion
 */

#ifndef INSET_ROS_UTILS_H
#define INSET_ROS_UTILS_H

#include <inset_geometry/ellipsoid.h>
#include <inset_geometry/polyhedron.h>
#include <sensor_msgs/PointCloud.h>
#include <inset_ros_msgs/PolyhedronArray.h>
#include <inset_ros_msgs/EllipsoidArray.h>
#include <nav_msgs/Path.h>
#include "iris/iris.h"

namespace InsetROS {

/// vector to path
template <int Dim>
nav_msgs::Path vec_to_path(const vec_Vecf<Dim> &vs)
{
    nav_msgs::Path path;
    for (const auto &it : vs)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = it(0);
        pose.pose.position.y = it(1);
        pose.pose.position.z = Dim == 2 ? 0 : it(2);
        pose.pose.orientation.w = 1.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;

        path.poses.push_back(pose);
    }

    return path;
}

/// vector to cloud
inline sensor_msgs::PointCloud vec_to_cloud(const vec_Vec3f &pts)
{
    sensor_msgs::PointCloud cloud;
    cloud.points.resize(pts.size());

    for (unsigned int i = 0; i < pts.size(); i++)
    {
        cloud.points[i].x = pts[i](0);
        cloud.points[i].y = pts[i](1);
        cloud.points[i].z = pts[i](2);
    }
    return cloud;
}

/// cloud to vector
inline vec_Vec3f cloud_to_vec(const sensor_msgs::PointCloud &cloud)
{
    vec_Vec3f pts;
    pts.resize(cloud.points.size());
    for (unsigned int i = 0; i < cloud.points.size(); i++)
    {
        pts[i](0) = cloud.points[i].x;
        pts[i](1) = cloud.points[i].y;
        pts[i](2) = cloud.points[i].z;
    }

    return pts;
}

/// inset_ros_msgs::Polyhedron to Polyhedron3D
inline Polyhedron3D ros_to_polyhedron(const inset_ros_msgs::Polyhedron &msg)
{
    Polyhedron3D poly;
    for (unsigned int i = 0; i < msg.points.size(); i++)
    {
        Vec3f pt(msg.points[i].x, msg.points[i].y, msg.points[i].z);
        Vec3f n(msg.normals[i].x, msg.normals[i].y, msg.normals[i].z);
        poly.add(Hyperplane3D(pt, n));
    }
    return poly;
}

/// inset_ros_msgs::PolyhedronArray to polyhedron_array
inline vec_E<Polyhedron3D> ros_to_polyhedron_array(const inset_ros_msgs::PolyhedronArray &msg)
{
    vec_E<Polyhedron3D> polys(msg.polyhedrons.size());

    for (size_t i = 0; i < msg.polyhedrons.size(); i++) polys[i] = ros_to_polyhedron(msg.polyhedrons[i]);

    return polys;
}

/// Polyhedron2D to inset_ros_msgs::Polyhedron
inline inset_ros_msgs::Polyhedron polyhedron_to_ros(const Polyhedron2D &poly)
{
    inset_ros_msgs::Polyhedron msg;
    for (const auto &p : poly.hyperplanes())
    {
        geometry_msgs::Point pt, n;
        pt.x = p.p_(0);
        pt.y = p.p_(1);
        pt.z = 0;
        n.x = p.n_(0);
        n.y = p.n_(1);
        n.z = 0;
        msg.points.push_back(pt);
        msg.normals.push_back(n);
    }

    geometry_msgs::Point pt1, n1;
    pt1.x = 0, pt1.y = 0, pt1.z = 0.01;
    n1.x = 0, n1.y = 0, n1.z = 1;
    msg.points.push_back(pt1);
    msg.normals.push_back(n1);

    geometry_msgs::Point pt2, n2;
    pt2.x = 0, pt2.y = 0, pt2.z = -0.01;
    n2.x = 0, n2.y = 0, n2.z = -1;
    msg.points.push_back(pt2);
    msg.normals.push_back(n2);

    return msg;
}

/// Polyhedron3D to  inset_ros_msgs::Polyhedron
inline inset_ros_msgs::Polyhedron polyhedron_to_ros(const Polyhedron3D &poly)
{
    inset_ros_msgs::Polyhedron msg;
    for (const auto &p : poly.hyperplanes())
    {
        geometry_msgs::Point pt, n;
        pt.x = p.p_(0);
        pt.y = p.p_(1);
        pt.z = p.p_(2);
        n.x = p.n_(0);
        n.y = p.n_(1);
        n.z = p.n_(2);
        msg.points.push_back(pt);
        msg.normals.push_back(n);
    }

    return msg;
}

/// polyhedron_array to inset_ros_msgs::PolyhedronArray
template <int Dim>
inset_ros_msgs::PolyhedronArray polyhedron_array_to_ros(const vec_E<Polyhedron<Dim>> &vs)
{
    inset_ros_msgs::PolyhedronArray msg;
    for (const auto &v : vs) msg.polyhedrons.push_back(polyhedron_to_ros(v));
    return msg;
}

/// ellipsoid_array to inset_ros_msgs::EllipsoidArray
template <int Dim>
inset_ros_msgs::EllipsoidArray ellipsoid_array_to_ros(const vec_E<Ellipsoid<Dim>> &Es)
{
    inset_ros_msgs::EllipsoidArray ellipsoids;
    for (unsigned int i = 0; i < Es.size(); i++)
    {
        inset_ros_msgs::Ellipsoid ellipsoid;
        auto d = Es[i].d();
        ellipsoid.d[0] = d(0);
        ellipsoid.d[1] = d(1);
        ellipsoid.d[2] = Dim == 2 ? 0 : d(2);

        auto C = Es[i].C();
        for (int x = 0; x < 3; x++)
        {
            for (int y = 0; y < 3; y++)
            {
                if (x < Dim && y < Dim)
                    ellipsoid.E[3 * x + y] = C(x, y);
                else
                    ellipsoid.E[3 * x + y] = 0;
            }
        }
        ellipsoids.ellipsoids.push_back(ellipsoid);
    }

    return ellipsoids;
}

/// inverse mapping(vec_Vec3f)
inline vec_Vec3f inverse_mapping_vec(const vec_Vec3f &pts_map, const Vec3f &obs_origin_point,
                                     const Eigen::Matrix3d &obs_attitude, const Vec3f &obs_origin_point_map,
                                     const Eigen::Matrix3d &obs_attitude_map)
{
    vec_Vec3f pts;
    pts.resize(pts_map.size());
    Eigen::Matrix3d RotationMatrix = obs_attitude * obs_attitude_map.inverse();
    for (unsigned int i = 0; i < pts_map.size(); i++)
    {
        pts[i] = RotationMatrix * pts_map[i] + obs_origin_point - obs_origin_point_map;
    }
    return pts;
}

/// inverse mapping(Vec3f)
inline Vec3f inverse_mapping_vec(const Vec3f &pt_map, const Vec3f &obs_origin_point,
                                 const Eigen::Matrix3d &obs_attitude, const Vec3f &obs_origin_point_map,
                                 const Eigen::Matrix3d &obs_attitude_map)
{
    Vec3f pt;
    Eigen::Matrix3d RotationMatrix = obs_attitude * obs_attitude_map.inverse();
    pt = RotationMatrix * pt_map + obs_origin_point - obs_origin_point_map;

    return pt;
}

/// inverse mapping(Polyhedron3D)
inline Polyhedron3D inverse_mapping_n_p_2d(const MatD2f &p_map, const MatD2f &n_map, const Vec3f &obs_origin_point,
                                           const Mat3f &obs_attitude, const Vec3f &obs_origin_point_map,
                                           const Mat3f &obs_attitude_map)
{
    Polyhedron3D VS3d;
    Vec3f p_tmp;
    Vec3f n_tmp;
    Mat3f RotationMatrix = obs_attitude * obs_attitude_map.inverse();
    for (int i = 0; i < n_map.rows(); i++)
    {
        p_tmp << p_map.row(i)(0), p_map.row(i)(1), 0;
        n_tmp << n_map.row(i)(0), n_map.row(i)(1), 0;
        p_tmp = RotationMatrix * p_tmp + obs_origin_point - obs_origin_point_map;
        n_tmp = RotationMatrix * n_tmp;
        VS3d.add(Hyperplane3D(p_tmp, n_tmp));
    }
    p_tmp << 0, 0, 0.01;
    n_tmp << 0, 0, 1;
    p_tmp = RotationMatrix * p_tmp + obs_origin_point - obs_origin_point_map;
    n_tmp = RotationMatrix * n_tmp;
    VS3d.add(Hyperplane3D(p_tmp, n_tmp));

    p_tmp << 0, 0, -0.01;
    n_tmp << 0, 0, -1;
    p_tmp = RotationMatrix * p_tmp + obs_origin_point - obs_origin_point_map;
    n_tmp = RotationMatrix * n_tmp;
    VS3d.add(Hyperplane3D(p_tmp, n_tmp));
    return VS3d;
}

/// inverse mapping(Ellipsoid3D)
inline Ellipsoid3D inverse_mapping_C_D_2d(const Mat2f &C_map, const Vec2f &D_map, const Vec3f &obs_origin_point,
                                          const Mat3f &obs_attitude, const Vec3f &obs_origin_point_map,
                                          const Mat3f &obs_attitude_map)
{
    Mat3f C_tmp;
    Vec3f D_tmp;
    C_tmp << C_map(0, 0), C_map(0, 1), 0, C_map(1, 0), C_map(1, 1), 0, 0, 0, 0;
    D_tmp << D_map(0), D_map(1), 0;
    Mat3f RotationMatrix = obs_attitude * obs_attitude_map.inverse();
    C_tmp = RotationMatrix * C_tmp * RotationMatrix.transpose();
    D_tmp = RotationMatrix * D_tmp + obs_origin_point - obs_origin_point_map;

    Ellipsoid3D ES3d(C_tmp, D_tmp);
    return ES3d;
}
} // namespace InsetROS

#endif
