//
// Created by malintha on 11/28/17.
//

#ifndef PROJECT_DYNAMICSIMPL_H
#define PROJECT_DYNAMICSIMPL_H
#endif

#pragma once

#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Twist.h"
#include <tf/transform_listener.h>
#include "geoControlllerUtils.h"

using namespace Eigen;

class dynamicsImpl {
public:
    float temp_acc;

    dynamicsImpl(const ros::NodeHandle &n, std::string worldFrame, std::string bodyFrame) :
            worldFrame(worldFrame),
            bodyFrame(bodyFrame) {
        n.getParam("/crazyflie//crazyflie_add/tf_prefix", tf_prefix);
        ROS_INFO("### Initialized dynamicsImpl worldFrame:%s bodyFrame%s\n", worldFrame.data(), bodyFrame.data());
        transformListener.waitForTransform(worldFrame, bodyFrame, ros::Time(0), ros::Duration(10.0));

        ros::NodeHandle nh;
        m_subscribeImuMsgs = nh.subscribe("/crazyflie/imu", 1, &dynamicsImpl::ImuValRecieved, this);
        utils = new geoControllerUtils();
        utils->initializeMatrices(n);
        Omega = prev_Omega = utils->getOmega0();
        prev_R = utils->getR0();
        prev_x = utils->getX0();
        prev_x_dot = utils->getV0();
        dt = 0.002;
    }

    void ImuValRecieved(const sensor_msgs::Imu::ConstPtr &msg) {
        x_ddot << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
        Omega << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
        IMUreceive = true;
        //angular_velocity.y => forward (pitch), x => roll
    }


    void setdt(float dt) {
        this->dt = dt;
    }

    Vector3d *get_x_v_Omega() {
        tf::StampedTransform transform;
        transformListener.lookupTransform(worldFrame, bodyFrame, ros::Time(0), transform);
        x << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z();
        x_dot = (x - prev_x) / dt;
        x_arr[0] = x;
        x_arr[1] = x_dot;
        x_arr[2] = x_ddot;
        x_arr[3] = Omega;
        prev_x = x;
        prev_x_dot = x_dot;

        return x_arr;
    }

    Matrix3d getR() {
        Omega_dot = (Omega - prev_Omega) / dt;
        Matrix3d Omega_hat = utils->getSkewSymmetricMap(Omega);
        R = prev_R + dt * utils->getSkewSymmetricMap(Omega) +
            0.5 * std::pow(dt, 2) * utils->getSkewSymmetricMap(Omega_dot);
        return R;
    }


private:
//    ros::NodeHandle &nodeHandle;
    ros::Subscriber m_subscribeImuMsgs;
    tf::TransformListener transformListener;
    ros::Publisher m_pubNav;

    std::string worldFrame;
    std::string bodyFrame;

    float dt;

    std::string tf_prefix;

    Matrix3d prev_R;
    Matrix3d R;

    Vector3d Omega;
    Vector3d Omega_dot;
    Vector3d prev_Omega;

    Vector3d x_ddot;
    Vector3d x_dot;
    Vector3d x;
    Vector3d prev_x;
    Vector3d prev_x_dot;

    Vector3d x_arr[5];
    geoControllerUtils *utils;
    bool IMUreceive = false;

};

