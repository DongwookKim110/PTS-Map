#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <Eigen/Dense>

#include <math.h>
#include <numeric>
#include <map>


template<typename Point>
bool point_z_cmp(Point a, Point b) {
    return a.z < b.z;
}

Eigen::Matrix4f calc_gravity_align_matrix(Eigen::Matrix4f pose, Eigen::Vector3f gravity){
    // cross product (gravity, (0,0,-1)) = (-gy, gx, 0) -> normalize
    float ux = - gravity[1] / sqrt(pow(gravity[0],2) + pow(gravity[1],2));
    float uy = gravity[0] / sqrt(pow(gravity[0],2) + pow(gravity[1],2));
    float uz = 0;
    // dot product (gravity, (0,0,-1))= (0,0,-gz)
    float theta = acos(-gravity[2] / sqrt(pow(gravity[0],2) + pow(gravity[1],2) + pow(gravity[2],2)));

    Eigen::Matrix4f gravity_align=Eigen::Matrix4f::Identity();
    gravity_align(0,0) = cos(theta) + ux * ux * (1-cos(theta));
    gravity_align(0,1) = ux * uy * (1-cos(theta)) - uz * sin(theta);
    gravity_align(0,2) = ux * uz * (1-cos(theta)) + uy * sin(theta);
    gravity_align(1,0) = uy * ux * (1-cos(theta)) + uz * sin(theta);
    gravity_align(1,1) = cos(theta) + uy * uy * (1-cos(theta));
    gravity_align(1,2) = uy * uz * (1-cos(theta)) - ux * sin(theta);
    gravity_align(2,0) = uz * ux * (1-cos(theta)) - uy * sin(theta);
    gravity_align(2,1) = uz * uy * (1-cos(theta)) + ux * sin(theta);
    gravity_align(2,2) = cos(theta) + uz * uz * (1-cos(theta));
    
    return gravity_align;

}

Eigen::Matrix4f odom2Eigen(const nav_msgs::Odometry::ConstPtr &odom_msg){
    Eigen::Matrix4f mat;
    double q0 = odom_msg->pose.pose.orientation.w;
    double q1 = odom_msg->pose.pose.orientation.x;
    double q2 = odom_msg->pose.pose.orientation.y;
    double q3 = odom_msg->pose.pose.orientation.z;
    mat(0,3) = odom_msg->pose.pose.position.x;
    mat(1,3) = odom_msg->pose.pose.position.y;
    mat(2,3) = odom_msg->pose.pose.position.z;
    mat(0,0) = 2*(q0*q0 + q1*q1)-1;
    mat(0,1) = 2*(q1*q2 - q0*q3);
    mat(0,2) = 2*(q1*q3 + q0*q2);
    mat(1,0) = 2*(q1*q2 + q0*q3);
    mat(1,1) = 2*(q0*q0 + q2*q2)-1;
    mat(1,2) = 2*(q2*q3 - q0*q1);
    mat(2,0) = 2*(q1*q3 - q0*q2);
    mat(2,1) = 2*(q2*q3 + q0*q1);
    mat(2,2) = 2*(q0*q0 + q3*q3)-1;
    mat(3,0) = 0.0;
    mat(3,1) = 0.0;
    mat(3,2) = 0.0;
    mat(3,3) = 1.0;

    return mat;
}

Eigen::Vector4f rot2quat(const Eigen::Matrix3f rotation){
    Eigen::Vector4f q; //x,y,z,w
    q(3) = sqrt(rotation(0,0) + rotation(1,1) + rotation(2,2) + 1)/2;
    q(0) = (rotation(2,1)-rotation(1,2)) / (4 * q(3));
    q(1) = (rotation(0,2)-rotation(2,0)) / (4 * q(3));
    q(2) = (rotation(1,0)-rotation(0,1)) / (4 * q(3));

    return q;
}

#endif