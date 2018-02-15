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

    dynamicsImpl(const ros::NodeHandle &n, std::string worldFrame, std::string bodyFrame) :
            worldFrame(worldFrame),
            bodyFrame(bodyFrame) {
        n.getParam("/crazyflie//crazyflie_add/tf_prefix", tf_prefix);
        ROS_INFO("### Initialized dynamicsImpl worldFrame:%s bodyFrame:%s\n", worldFrame.data(), bodyFrame.data());
        transformListener.waitForTransform(worldFrame, bodyFrame, ros::Time(0), ros::Duration(10.0));

        ros::NodeHandle nh;
        utils = new geoControllerUtils();
        utils->initializeMatrices(n);
        Omega = utils->getOmega0();
        utils->getR0();
        prev_x = utils->getX0();
        utils->getV0();
        rpy_ned_prev << 0,0,0;
        dt = 0.02;
    }


    void setdt(float dt) {
        this->dt = dt;
    }

//todo: calculate the angular velocity using a library mavros

    Vector3d *get_x_v_Omega(tf::StampedTransform transform_now, double t_frame) {
        x << transform_now.getOrigin().x(), transform_now.getOrigin().y(), transform_now.getOrigin().z();
        RPY_now = (transformtoRPY(transform_now));
        R = getR(RPY_now(0),RPY_now(1), RPY_now(2));
        rpy_ned = utils->R2RPY(R);
        utils->publishRPY(rpy_ned[0], rpy_ned[1], rpy_ned[2]);
        Omega = (rpy_ned - rpy_ned_prev)/dt;

        x_dot = (x - prev_x) / dt;
        x_arr[0] = x;
        x_arr[1] = x_dot;
        x_arr[2] = Omega;
        prev_x = x;
        rpy_ned_prev = rpy_ned;
        return x_arr;
    }

    /**
     * all are measured ccw
     * @param transform
     * @return
     */
    Vector3d transformtoRPY(tf::StampedTransform transform) {
        tfScalar roll, pitch, yaw;
        Vector3d rpy;
        tf::Matrix3x3(tf::Quaternion(
                transform.getRotation().x(),
                transform.getRotation().y(),
                transform.getRotation().z(),
                transform.getRotation().w()
        )).getRPY(roll,pitch,yaw);
        rpy << roll, pitch, -yaw;
        return rpy;
    }

    /**
     * @param gamma roll
     * @param beta pitch
     * @param alpha yaw
     */
    Matrix3d getR(tfScalar gamma, tfScalar beta, tfScalar alpha) {
        Matrix3d R;
        R <<
          cos(alpha) * cos(beta), cos(alpha) * sin(beta) * sin(gamma) - sin(alpha) * cos(gamma), cos(alpha) * sin(beta) * cos(gamma) + sin(alpha) * sin(gamma),
                sin(alpha) * cos(beta), sin(alpha) * sin(beta) * sin(gamma) + cos(alpha) * cos(gamma), sin(alpha) * sin(beta) * cos(gamma) - cos(alpha) * sin(gamma),
                -sin(beta), cos(beta) * sin(gamma), cos(beta) * cos(gamma);
        Matrix3d transformation;
        transformation <<   1, 0,  0,
                            0, -1, 0,
                            0, 0, -1;
        R = transformation*R;
        return R;
    }

    Matrix3d getR() {
        return this->R;
    }

    Vector3d getRpy() {
        return RPY_now;
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

    Matrix3d R;

    Vector3d Omega;

    Vector3d RPY_now;
    Vector3d rpy_ned, rpy_ned_prev;

    Vector3d x_dot;
    Vector3d x;
    Vector3d prev_x;

    Vector3d x_arr[3];
    geoControllerUtils *utils;
    ros::Publisher debug_coeffs;
    tf::StampedTransform transform_t_1;

};

