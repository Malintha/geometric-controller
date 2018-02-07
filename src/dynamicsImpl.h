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
        ROS_INFO("### Initialized dynamicsImpl worldFrame:%s bodyFrame:%s\n", worldFrame.data(), bodyFrame.data());
        transformListener.waitForTransform(worldFrame, bodyFrame, ros::Time(0), ros::Duration(10.0));

        ros::NodeHandle nh;
        m_subscribeImuMsgs = nh.subscribe("/crazyflie/imu", 1, &dynamicsImpl::ImuValRecieved, this);
        utils = new geoControllerUtils();
        utils->initializeMatrices(n);
        Omega = prev_Omega = utils->getOmega0();
        prev_R = utils->getR0();
        prev_x = utils->getX0();
        prev_x_dot = utils->getV0();
        dt = 0.02;
    }

    void ImuValRecieved(const sensor_msgs::Imu::ConstPtr &msg) {
        x_ddot << msg->linear_acceleration.x*cos(M_PI/4), msg->linear_acceleration.y*cos(M_PI/4), msg->linear_acceleration.z;
//        Omega << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
        IMUreceive = true;

    }


    void setdt(float dt) {
        this->dt = dt;
    }

    Vector3d RPY_now, RPY_t_1;
    Vector3d rpy_ned_t, rpy_ned_t_1;


    Vector3d *get_x_v_Omega(tf::StampedTransform transform_now, double t_frame) {
        if(t_frame < 0.02)
            transform_t_1 = transform_now;

        x << transform_now.getOrigin().x(), transform_now.getOrigin().y(), transform_now.getOrigin().z();
        RPY_now = (transformtoRPY(transform_now));
        RPY_t_1 = (transformtoRPY(transform_t_1));
        Matrix3d R_t_ned = getR(RPY_now(0),RPY_now(1), RPY_now(2));
        Matrix3d R_t_1_ned = getR(RPY_t_1[0], RPY_t_1[1], RPY_t_1[2]);
        rpy_ned_t = utils->R2RPY(R_t_ned);
        rpy_ned_t_1 = utils->R2RPY(R_t_1_ned);
        this->R = R_t_ned;

        utils->publishRPY(rpy_ned_t[0], rpy_ned_t[1], rpy_ned_t[2]);

        Omega = (rpy_ned_t - rpy_ned_t_1)/dt;
        std::cout<<"Omega: "<<Omega[0]<<" "<<Omega[1]<<" "<<Omega[2]<<std::endl;
        for(int i=0;i<3;i++)
            utils->publishOmega(Omega[i],i);


        x_dot = (x - prev_x) / dt;
        x_arr[0] = x;
        x_arr[1] = x_dot;
        x_arr[2] = x_ddot;
        x_arr[3] = Omega;
        prev_x = x;
        prev_x_dot = x_dot;
        transform_t_1 = transform_now;
        return x_arr;
    }

    /**
     * roll is measured ccw
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
        rpy << roll, pitch, yaw;
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

    Matrix3d prev_R;
    Matrix3d R;

    Vector3d rpy;
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
    ros::Publisher debug_coeffs;
    tf::StampedTransform transform_t_1;

};

