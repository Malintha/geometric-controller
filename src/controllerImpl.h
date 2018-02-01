//
// Created by malintha on 11/27/17.
//

#ifndef PROJECT_TRACKING_CONTROL_H
#define PROJECT_TRACKING_CONTROL_H
#endif

#pragma once

#include <eigen3/Eigen/Dense>
#include "ros/ros.h"
#include "geoControlllerUtils.h"
#include "std_msgs/String.h"

using namespace Eigen;

class ControllerImpl {

public:
    ControllerImpl(const ros::NodeHandle &nodeHandle, dynamicsImpl* dynamics) : n(nodeHandle), dynamics(dynamics) {
        t_frame = 0;
        prev_R_d << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;
        prev_Omega_d << 0, 0, 0;

        geoControllerUtils utils;

        e1[0] = e2[1] = e3[2] = 1;
        e1[1] = e1[2] = e2[0] = e2[2] = e3[0] = e3[1] = 0;

        // set values from the yaml file
        float j11 = utils.get(n, "uav/J/J1");
        float j22 = utils.get(n, "uav/J/J2");
        float j33 = utils.get(n, "uav/J/J3");
        J << j11, 0, 0,
            0, j22, 0,
            0, 0, j33;

        m = utils.get(n, "uav/m");
        d = utils.get(n, "uav/d");
        ctf = utils.get(n, "uav/ctf");
        kx = utils.get(n, "controller/kx");
        kv = utils.get(n, "controller/kv");
        kr = utils.get(n, "controller/kr");
        kOmega = utils.get(n, "controller/kOmega");

        kr_t = utils.get(n, "takingoff/kr");
        kOmega_t = utils.get(n, "takingoff/kOmega");

        coeffMat << 1, 1, 1, 1,
                    0, -d, 0, d,
                    d, 0, -d, 0,
                    -ctf, ctf, -ctf, ctf;

        inv_coeffMat = Eigen::Inverse<Matrix4d>(coeffMat);

        x0 << utils.get(n, "trajectory/x0/x01"), utils.get(n, "trajectory/x0/x02"), utils.get(n, "trajectory/x0/x03");
        v0 << utils.get(n, "trajectory/v0/v01"), utils.get(n, "trajectory/v0/v02"), utils.get(n, "trajectory/v0/v03");
        R0 << utils.get(n, "trajectory/R0/R0_r1/r1_1"), utils.get(n, "trajectory/R0/R0_r1/r1_2"), utils.get(n, "trajectory/R0/R0_r1/r1_3"),
                utils.get(n, "trajectory/R0/R0_r2/r2_1"), utils.get(n, "trajectory/R0/R0_r2/r2_2"), utils.get(n, "trajectory/R0/R0_r2/r2_3"),
                utils.get(n, "trajectory/R0/R0_r3/r3_1"), utils.get(n, "trajectory/R0/R0_r3/r3_2"), utils.get(n, "trajectory/R0/R0_r3/r3_3");
        Omega0 << utils.get(n, "trajectory/Omega0/Omega01"), utils.get(n, "trajectory/Omega0/Omega02"), utils.get(n, "trajectory/Omega0/Omega03");

        ros::NodeHandle nh;
        debug_coeffs = nh.advertise<std_msgs::String>("geo_debug_coeffs", 1000);

    }

    void setDynamicsValues(Vector3d x, Vector3d v, Matrix3d R, Vector3d Omega) {
        if (isFirst) {
            this->x = x0;
            this->v = v0;
            this->R = R0;
            this->Omega = Omega0;
        }
        this->x = x;
        this->v = v;
        this->R = R;
        this->Omega = Omega;
    }

    void setInitValues(float dt, double time_frame, bool isTakingOff) {
        this->t_frame = time_frame;
        this->dt = dt;
        calculate_x_desired();
        calculate_ex_ev();
        calculate_Rd(isTakingOff);
        calculate_Omega_desired();
        calculate_eR_eOmega();
        if(isTakingOff) {
//            eR[2] = 0;
//            eOmega[2] = 0;
        }
    }

    /**
     * calculate the force vector. Should be invoked after dynamic values are set
     * @param e
     * @return
     */
    double getTotalForce() {
        Vector3d f_temp = -(-kx * ex - kv * ev - m * g * e3 + m * xddot_d);
        std_msgs::String msg;
        std::stringstream ss;
        Vector3d Re3 = R * e3;
        double f = f_temp.dot(Re3);
        ss << "kx: "<<kx<<" ex: "<<ex<<" kv: "<<kv<<" ev: "<<ev<<" m: "<<m<<" g "<<g<<" e3: "<<e3<<" xddot_d: "<<xddot_d<<"f: "<<f;
        msg.data = ss.str();
        debug_coeffs.publish(msg);
        return f;
    }

    Vector3d getMomentVector(bool isTakingoff) {
        if(isTakingoff) {
            kOmega = kOmega_t;
            kr = kr_t;
        }

        std::cout<<"eR: "<<eR[0]<<" , "<<eR[1]<<" , "<<eR[2]<<" eOmega: "<<eOmega[0]<<" , "<<eOmega[1]<<" , "<<eOmega[2]<<std::endl;
        Matrix3d Omega_hat = utils.getSkewSymmetricMap(Omega);
//        eR[2] = eR[2]*0.01;
//        eOmega[2] = eOmega[2]*0.1;
        M = -kr * eR - kOmega * eOmega + Omega.cross(J*Omega) -
            J*((Omega_hat * R.transpose() * R_d) * Omega_d -
                           (R.transpose() * R_d) * Omega_dot_d);
        std::cout << "Omega_d: " << Omega_d[0] << " " << Omega_d[1] << " " << Omega_d[2] << " Omega_dot_d: "
                  << Omega_dot_d[0] << " " << Omega_dot_d[1] << " " << Omega_dot_d[2] << std::endl;
        return M;
    }

    void calculate_ex_ev() {
        ex = x - x_d;
        ev = v - xdot_d;
        for (int i = 0; i < 3; i++) {
            utils.publishEx(ex[i],i);
            utils.publishEv(ev[i],i);
        }

        std::cout<<"ex: "<<ex[0]<<" , "<<ex[1]<<" , "<<ex[2]<<" ev: "<<ev[0]<<" , "<<ev[1]<<" , "<<ev[2]<<std::endl;
    }

    Vector3d getEx() {
        return this->ex;
    }

    void calculate_eR_eOmega() {
        Matrix3d eR_temp = 0.5 * (Eigen::Transpose<Matrix3d>(R_d) * R - Eigen::Transpose<Matrix3d>(R) * R_d);
        eR = utils.getVeeMap(eR_temp);
        eOmega = Omega - Eigen::Transpose<Matrix3d>(R) * R_d * Omega_d;

        for (int i=0;i<3;i++) {
            utils.publishEr(eR[i], i);
            utils.publishEOmega(eOmega[i], i);
        }
    }

    void calculate_Rd(bool isTakingOff) {
        Vector3d b3_d_nume = -kx * ex - kv * ev - m * g * e3 + m * xddot_d;
        b3_d = b3_d_nume / b3_d_nume.norm();
        b1_d << 1, 0, 0.2;
        Vector3d b2_d_nume = b3_d.cross(b1_d);
        b2_d = b2_d_nume / b2_d_nume.norm();
        R_d << b2_d.cross(b3_d), b2_d, b3_d;
//        if (isTakingOff) {
//            R_d << 1, 0, 0,
//                    0, 1, 0,
//                    0, 0, 1;
//        }
        std::cout<<"R_d:\n"<<R_d<<"\n";
    }

    void calculate_Omega_desired() {
        rpy_d << atan(-R_d(2,0)/sqrt(R_d(2,1)*R_d(2,1) + R_d(2,2)*R_d(2,2))),
                atan(R_d(2,1)/R_d(2,2)),
                atan(R_d(1,0)/R_d(0,0));
        Vector3d rpy = dynamics->getRpy();

        std::cout<<"rpy: "<<rpy[0]<<" , "<<rpy[1]<<" , "<<rpy[2]<<" rpy_d: "<<rpy_d[0]<<" , "<<rpy_d[1]<<" , "<<rpy_d[2]<<std::endl;
        Omega_d = (rpy_d - rpy)/dt;
        Omega_dot_d = (Omega_d - prev_Omega_d) / dt;
        prev_Omega_d = Omega_d;
    }

    void calculate_x_desired() {
        x_d[0] = 0;
        x_d[1] = 0;
        x_d[2] = 0.2;

        xdot_d[0] = 0;
        xdot_d[1] = 0;
        xdot_d[2] = 0.0;

        xddot_d[0] = 0;
        xddot_d[1] = 0;
        xddot_d[2] = 0;
    }

    Vector4d getMotorForceVector(double totForce, Vector3d momentVec){
        Vector4d f_m_vec(4,1);
        Vector4d mot_force_vec(4,1);
        f_m_vec << totForce, momentVec[0], momentVec[1], momentVec[2];

        std::cout<<"\nf_m_vec: "<<f_m_vec<<"\n";
        mot_force_vec = inv_coeffMat*f_m_vec;
        return mot_force_vec;
    }

    double getAttitudeError() {
        Matrix3d eye = Matrix<double, 3, 3>::Identity();
        double att_error = (0.5*(eye - Eigen::Transpose<Matrix3d>(R_d)*R)).trace();
        return att_error >= 0 ? att_error: 0;
    }

private:
    ros::NodeHandle n;
    float dt;
    double t_frame;

    // desired parameters
    Vector3d x_d;
    Vector3d xdot_d;
    Vector3d xddot_d;
    Vector3d b1_d;
    Vector3d b2_d;
    Vector3d b3_d;
    Matrix3d R_d;
    Matrix3d R_dot_d;
    Vector3d Omega_d;
    Vector3d Omega_dot_d;

    // actual parameters
    Vector3d x;
    Vector3d v;
    Matrix3d R;
    Vector3d Omega;
    Vector3d rpy_d;
    Vector3d Omega_dot;

    // initial parameters
    Vector3d x0;
    Vector3d v0;
    Matrix3d R0;
    Vector3d Omega0;
    Vector3d* test_v;

    // error vectors
    Vector3d ex;
    Vector3d ev;
    Vector3d eR;
    Vector3d eOmega;

    // drone specific and controller specific values
    Matrix3d J;
    float m;
    float d;
    float ctf;
    float kx, kv, kr, kOmega;
    float kx_t, kv_t, kr_t, kOmega_t;
    static const float g = -8.7;

    // translation vectors
    Vector3d e1;
    Vector3d e2;
    Vector3d e3;

    // force and moment vectors
    Vector3d f;
    Vector3d M;

    // other variables
    Matrix3d prev_R_d;
    Vector3d prev_Omega_d;
    bool isFirst = true;
    geoControllerUtils utils;
    Matrix4d coeffMat;
    Matrix4d inv_coeffMat;
    ros::Publisher debug_coeffs;
    dynamicsImpl* dynamics;

};