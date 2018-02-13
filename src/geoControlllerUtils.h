//
// Created by malintha on 11/29/17.
//

#ifndef PROJECT_GEOCONTROLLLERUTILS_H
#define PROJECT_GEOCONTROLLLERUTILS_H
#endif

#pragma once

#include <eigen3/Eigen/Dense>
#include <ros/node_handle.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int16.h>

using namespace Eigen;


class geoControllerUtils {
public:
    geoControllerUtils() {

        m_geo_debug_f1 = nh.advertise<std_msgs::Float32>("geo_debug_f1", 10);
        m_geo_debug_f2 = nh.advertise<std_msgs::Float32>("geo_debug_f2", 10);
        m_geo_debug_f3 = nh.advertise<std_msgs::Float32>("geo_debug_f3", 10);
        m_geo_debug_f4 = nh.advertise<std_msgs::Float32>("geo_debug_f4", 10);

        m_geo_debug_r1 = nh.advertise<std_msgs::Int16>("geo_debug_ratio_m1", 10);
        m_geo_debug_r2 = nh.advertise<std_msgs::Int16>("geo_debug_ratio_m2", 10);
        m_geo_debug_r3 = nh.advertise<std_msgs::Int16>("geo_debug_ratio_m3", 10);
        m_geo_debug_r4 = nh.advertise<std_msgs::Int16>("geo_debug_ratio_m4", 10);

        m_geo_debug_totalthrust = nh.advertise<std_msgs::Float32>("geo_debug_totalthrust", 10);

        m_geo_debug_er_1 = nh.advertise<std_msgs::Float32>("geo_debug_er_1", 10);
        m_geo_debug_er_2 = nh.advertise<std_msgs::Float32>("geo_debug_er_2", 10);
        m_geo_debug_er_3 = nh.advertise<std_msgs::Float32>("geo_debug_er_3", 10);

        m_geo_debug_ex_1 = nh.advertise<std_msgs::Float32>("geo_debug_ex_1", 10);
        m_geo_debug_ex_2 = nh.advertise<std_msgs::Float32>("geo_debug_ex_2", 10);
        m_geo_debug_ex_3 = nh.advertise<std_msgs::Float32>("geo_debug_ex_3", 10);

        m_geo_debug_ev_1 = nh.advertise<std_msgs::Float32>("geo_debug_ev_1", 10);
        m_geo_debug_ev_2 = nh.advertise<std_msgs::Float32>("geo_debug_ev_2", 10);
        m_geo_debug_ev_3 = nh.advertise<std_msgs::Float32>("geo_debug_ev_3", 10);

        m_geo_debug_x_1 = nh.advertise<std_msgs::Float32>("geo_debug_x_1", 10);
        m_geo_debug_x_2 = nh.advertise<std_msgs::Float32>("geo_debug_x_2", 10);
        m_geo_debug_x_3 = nh.advertise<std_msgs::Float32>("geo_debug_x_3", 10);

        m_geo_debug_eOmega_1 = nh.advertise<std_msgs::Float32>("geo_debug_eOmega_1", 10);
        m_geo_debug_eOmega_2 = nh.advertise<std_msgs::Float32>("geo_debug_eOmega_2", 10);
        m_geo_debug_eOmega_3 = nh.advertise<std_msgs::Float32>("geo_debug_eOmega_3", 10);

        m_geo_debug_Omega_1 = nh.advertise<std_msgs::Float32>("geo_debug_Omega_1", 10);
        m_geo_debug_Omega_2 = nh.advertise<std_msgs::Float32>("geo_debug_Omega_2", 10);
        m_geo_debug_Omega_3 = nh.advertise<std_msgs::Float32>("geo_debug_Omega_3", 10);

        m_geo_debug_roll = nh.advertise<std_msgs::Float32>("geo_debug_roll", 10);
        m_geo_debug_pitch = nh.advertise<std_msgs::Float32>("geo_debug_pitch", 10);
        m_geo_debug_yaw = nh.advertise<std_msgs::Float32>("geo_debug_yaw", 10);

        m_geo_debug_m1 = nh.advertise<std_msgs::Float32>("geo_debug_m1", 10);
        m_geo_debug_m2 = nh.advertise<std_msgs::Float32>("geo_debug_m2", 10);
        m_geo_debug_m3 = nh.advertise<std_msgs::Float32>("geo_debug_m3", 10);
    }

    double get(const ros::NodeHandle &n, const std::string &name) {
        const std::string node_prefix = "/crazyflie/geocontroller/";
        std::string key = node_prefix+name;
        double value;
        n.getParam(key, value);
        return value;
    }

    double getTargetRatio(double targetThrust) {
        double targetRPM;
        if (targetThrust > 0)
            targetRPM = (-1.032633 + (sqrt(10195.96+852118*targetThrust))*0.01)*100000/4.26059;

        if (targetRPM > 65535)
            targetRPM = 65535;
        return targetRPM;
    }

    void initializeMatrices(const ros::NodeHandle &n) {
        x0 << get(n, "trajectory/x0/x01"), get(n, "trajectory/x0/x02"), get(n, "trajectory/x0/x03");
        v0 << get(n, "trajectory/v0/v01"), get(n, "trajectory/v0/v02"), get(n, "trajectory/v0/v03");
        R0 << get(n, "trajectory/R0/R0_r1/r1_1"), get(n, "trajectory/R0/R0_r1/r1_2"), get(n, "trajectory/R0/R0_r1/r1_3"),
                get(n, "trajectory/R0/R0_r2/r2_1"), get(n, "trajectory/R0/R0_r2/r2_2"), get(n, "trajectory/R0/R0_r2/r2_3"),
                get(n, "trajectory/R0/R0_r3/r3_1"), get(n, "trajectory/R0/R0_r3/r3_2"), get(n, "trajectory/R0/R0_r3/r3_3");
        Omega0 << get(n, "trajectory/Omega0/Omega01"), get(n, "trajectory/Omega0/Omega02"), get(n, "trajectory/Omega0/Omega03");
    }

    Vector3d getVeeMap(Matrix3d mat) {
        Vector3d vee_vec;
        vee_vec[0] = mat(2, 1);
        vee_vec[1] = mat(0, 2);
        vee_vec[2] = mat(1, 0);
        return vee_vec;
    }

    Matrix3d getSkewSymmetricMap(Vector3d vec) {
        Matrix3d hat_map(3, 3);
        hat_map << 0, -vec(2), vec(1),
                vec(2), 0, -vec(0),
                -vec(1), vec(0), 0;
        return hat_map;
    }

    /**
     * return RPY angles given a rotation matrix.
     * todo: check for all 4 quadrants
     * @param R
     * @return
     */
    Vector3d R2RPY(Matrix3d R) {
        Vector3d rpy;
        rpy <<  atan2(R(2,1),R(2,2)),
                atan2(-R(2,0),sqrt(R(2,1)*R(2,1) + R(2,2)*R(2,2))),
                atan2(R(1,0),R(0,0));
        return rpy;
    }

    Vector3d getOmega0() {
        return Omega0;
    }

    Matrix3d getR0() {
        return R0;
    }

    Vector3d getX0() {
        return x0;
    }
    Vector3d getV0() {
        return v0;
    }

    Vector3d *getTestV() {
        return &v0;
    }

    void publishThrusts(float thrust, int motorId){
        msg_f.data = thrust;
        if(motorId == 0)
            m_geo_debug_f1.publish(msg_f);
        else if(motorId == 1)
            m_geo_debug_f2.publish(msg_f);
        else if (motorId == 2)
            m_geo_debug_f3.publish(msg_f);
        else if (motorId == 3)
            m_geo_debug_f4.publish(msg_f);
        else
            m_geo_debug_totalthrust.publish(msg_f);
    }

    void publishMotorRatios(float motorRatio, int motorId){
        msg_r.data = motorRatio;
        if(motorId == 1)
            m_geo_debug_r1.publish(msg_r);
        else if(motorId == 2)
            m_geo_debug_r2.publish(msg_r);
        else if (motorId == 3)
            m_geo_debug_r3.publish(msg_r);
        else if (motorId == 4)
            m_geo_debug_r4.publish(msg_r);
    }

    void publishEr(double er, int i) {
        msg_er.data = er;
        if(i == 0)
            m_geo_debug_er_1.publish(msg_er);
        else if(i == 1)
            m_geo_debug_er_2.publish(msg_er);
        else if (i == 2)
            m_geo_debug_er_3.publish(msg_er);
    }

    void publishEx(double ex, int i) {
        msg_ex.data = ex;
        if(i == 0)
            m_geo_debug_ex_1.publish(msg_ex);
        else if(i == 1)
            m_geo_debug_ex_2.publish(msg_ex);
        else if (i == 2)
            m_geo_debug_ex_3.publish(msg_ex);
    }

    void publishEv(double ev, int i) {
        msg_ev.data = ev;
        if(i == 0)
            m_geo_debug_ev_1.publish(msg_ev);
        else if(i == 1)
            m_geo_debug_ev_2.publish(msg_ev);
        else if (i == 2)
            m_geo_debug_ev_3.publish(msg_ev);
    }

    void publishX(double x, int i) {
        msg_x.data = x;
        if(i == 0)
            m_geo_debug_x_1.publish(msg_x);
        else if(i == 1)
            m_geo_debug_x_2.publish(msg_x);
        else if (i == 2)
            m_geo_debug_x_3.publish(msg_x);
    }

    void publishEOmega(double eOmega, int i) {
        msg_eOmega.data = eOmega;
        if(i == 0)
            m_geo_debug_eOmega_1.publish(msg_eOmega);
        else if(i == 1)
            m_geo_debug_eOmega_2.publish(msg_eOmega);
        else if (i == 2)
            m_geo_debug_eOmega_3.publish(msg_eOmega);
    }

    void publishOmega(double Omega, int i) {
        msg_Omega.data = Omega;
        if(i == 0)
            m_geo_debug_Omega_1.publish(msg_Omega);
        else if(i == 1)
            m_geo_debug_Omega_2.publish(msg_Omega);
        else if (i == 2)
            m_geo_debug_Omega_3.publish(msg_Omega);
    }

    void publishMoment(double m, int i) {
        msg_m.data = m;
        if(i == 0)
            m_geo_debug_m1.publish(msg_m);
        else if(i == 1)
            m_geo_debug_m2.publish(msg_m);
        else if (i == 2)
            m_geo_debug_m3.publish(msg_m);
    }

    void publishRPY(float roll, float pitch, float yaw) {
        msg_rpy.data = roll;
        m_geo_debug_roll.publish(msg_rpy);
        msg_rpy.data = pitch;
        m_geo_debug_pitch.publish(msg_rpy);
        msg_rpy.data = yaw;
        m_geo_debug_yaw.publish(msg_rpy);
    }

private:
    Vector3d x0;
    Vector3d v0;
    Matrix3d R0;
    Vector3d Omega0;
    ros::NodeHandle nh;
    ros::Publisher m_geo_debug_f1;
    ros::Publisher m_geo_debug_f2;
    ros::Publisher m_geo_debug_f3;
    ros::Publisher m_geo_debug_f4;

    ros::Publisher m_geo_debug_r1;
    ros::Publisher m_geo_debug_r2;
    ros::Publisher m_geo_debug_r3;
    ros::Publisher m_geo_debug_r4;

    ros::Publisher m_geo_debug_totalthrust;

    ros::Publisher m_geo_debug_er_1;
    ros::Publisher m_geo_debug_er_2;
    ros::Publisher m_geo_debug_er_3;

    ros::Publisher m_geo_debug_ex_1;
    ros::Publisher m_geo_debug_ex_2;
    ros::Publisher m_geo_debug_ex_3;

    ros::Publisher m_geo_debug_ev_1;
    ros::Publisher m_geo_debug_ev_2;
    ros::Publisher m_geo_debug_ev_3;

    ros::Publisher m_geo_debug_x_1;
    ros::Publisher m_geo_debug_x_2;
    ros::Publisher m_geo_debug_x_3;

    ros::Publisher m_geo_debug_Omega_1;
    ros::Publisher m_geo_debug_Omega_2;
    ros::Publisher m_geo_debug_Omega_3;

    ros::Publisher m_geo_debug_eOmega_1;
    ros::Publisher m_geo_debug_eOmega_2;
    ros::Publisher m_geo_debug_eOmega_3;

    ros::Publisher m_geo_debug_roll;
    ros::Publisher m_geo_debug_pitch;
    ros::Publisher m_geo_debug_yaw;

    ros::Publisher m_geo_debug_m1;
    ros::Publisher m_geo_debug_m2;
    ros::Publisher m_geo_debug_m3;

    std_msgs::Float32 msg_f;
    std_msgs::Int16 msg_r;
    std_msgs::Float32 msg_er;
    std_msgs::Float32 msg_ex;
    std_msgs::Float32 msg_ev;
    std_msgs::Float32 msg_x;
    std_msgs::Float32 msg_Omega;
    std_msgs::Float32 msg_eOmega;
    std_msgs::Float32 msg_rpy;
    std_msgs::Float32 msg_m;


};

