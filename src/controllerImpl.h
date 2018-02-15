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
    ControllerImpl(const ros::NodeHandle &nodeHandle, dynamicsImpl *dynamics) : n(nodeHandle), dynamics(dynamics) {
        prev_Omega_d << 0, 0, 0;
        geoControllerUtils utils;

        e1[0] = e2[1] = e3[2] = 1;
        e1[1] = e1[2] = e2[0] = e2[2] = e3[0] = e3[1] = 0;

        // set values from the yaml file
        float j11 = utils.get(n, "uav/J/J1");
        float j22 = utils.get(n, "uav/J/J2");
        float j33 = utils.get(n, "uav/J/J3");
        m = utils.get(n, "uav/m");
        d = utils.get(n, "uav/d");
        ctf = utils.get(n, "uav/ctf");
        kx = utils.get(n, "controller/kx");
        kv = utils.get(n, "controller/kv");
        kr = utils.get(n, "controller/kr");
        kOmega = utils.get(n, "controller/kOmega");

        coeffMat << 1, 1, 1, 1,
                0, -d, 0, d,
                d, 0, -d, 0,
                -ctf, ctf, -ctf, ctf;
        inv_coeffMat = Eigen::Inverse<Matrix4d>(coeffMat);
        x0 << utils.get(n, "trajectory/x0/x01"), utils.get(n, "trajectory/x0/x02"), utils.get(n, "trajectory/x0/x03");
        v0 << utils.get(n, "trajectory/v0/v01"), utils.get(n, "trajectory/v0/v02"), utils.get(n, "trajectory/v0/v03");
        R0 << utils.get(n, "trajectory/R0/R0_r1/r1_1"), utils.get(n, "trajectory/R0/R0_r1/r1_2"), utils.get(n,
                                                                                                            "trajectory/R0/R0_r1/r1_3"),
                utils.get(n, "trajectory/R0/R0_r2/r2_1"), utils.get(n, "trajectory/R0/R0_r2/r2_2"), utils.get(n,
                                                                                                              "trajectory/R0/R0_r2/r2_3"),
                utils.get(n, "trajectory/R0/R0_r3/r3_1"), utils.get(n, "trajectory/R0/R0_r3/r3_2"), utils.get(n,
                                                                                                              "trajectory/R0/R0_r3/r3_3");
        Omega0 << utils.get(n, "trajectory/Omega0/Omega01"), utils.get(n, "trajectory/Omega0/Omega02"), utils.get(n,
                                                                                                                  "trajectory/Omega0/Omega03");
        J << 16.571710e-6, 0.830806e-6, 0.718277e-6,
                0.830806e-6, 16.655602e-6, 1.800197e-6,
                0.718277e-6, 1.800197e-6, 29.261652e-6;
        R_d_t_1 << 1, 0, 0,
                0, 1, 0,
                0, 0, 1;
        ros::NodeHandle nh;
    }

    void setDynamicsValues(Vector3d x, Vector3d v, Matrix3d R, Vector3d Omega) {
        if (isFirst) {
            this->x = x0;
            this->v = v0;
            this->R = R0;
            this->Omega = Omega0;
            isFirst = false;
        }
        this->x = x;
        this->v = v;
        this->R = R;
        this->Omega = Omega;
    }

    void setInitValues(float dt, double time_frame) {
        this->dt = dt;
        this->time_frame = time_frame;
        set_x_desired();
        calculate_ex_ev();
//        set_Rd();
        calculate_Omega_desired();
        calculate_eR_eOmega();
    }

    /**
     * return the force vector. Should be invoked after dynamic values are set
     * @param e
     * @return
     */
    double getTotalForce() {
        return this->f;
    }

    Vector3d getMomentVector() {
        M = -kr * eR - kOmega * eOmega + Omega.cross(J * Omega) -
            J * (utils.getSkewSymmetricMap(Omega) * R.transpose() * R_d * Omega_d -
                 R.transpose() * R_d * Omega_d_dot);

//        std::cout<<"kr: "<<kr<<"eR: "<<eR<<" eOmega: "<<eOmega<<" Omega: "<<Omega<<" R: "<<R<<"R_d: "<<R_d<<" Omega_d: "<<Omega_d<<" Omega_d_dot: "<<Omega_d_dot;
        std::cout<<"M : "<<M;
//        M[2] = -M[2];
        for (int i = 0; i < 3; i++) {
            utils.publishMoment(M[i], i);
        }
        return M;
    }

    void calculate_ex_ev() {
        ex = x - x_d;
        ev = v - xdot_d;
        for (int i = 0; i < 3; i++) {
            utils.publishEx(ex[i], i);
            utils.publishEv(ev[i], i);
        }
    }

    void calculate_eR_eOmega() {
//        std::cout<<"R: \n"<<R<<std::endl;
        Matrix3d eR_temp = 0.5 * (R_d.transpose() * R - R.transpose() * R_d);
        eR = utils.getVeeMap(eR_temp);
        eOmega = Omega - R.transpose() * R_d * Omega_d;

        for (int i = 0; i < 3; i++) {
            utils.publishEr(eR[i], i);
//            utils.publishEOmega(eOmega[i], i);
        }
    }

    void set_Rd() {
        Vector3d b3_d, b2_d;
        Vector3d b3_d_nume = A;
        b3_d = -b3_d_nume / b3_d_nume.norm();
        Vector3d b2_d_nume = b3_d.cross(b1_d);
        b2_d = b2_d_nume / b2_d_nume.norm();
        R_d << b2_d.cross(b3_d), b2_d, b3_d;
        std::cout<<"R_d_firstWay: \n"<<R_d<<std::endl;

    }

    Vector3d A;
    void calculate_Omega_desired() {
        float g1 = this->g;
        b1_d << 1, 0, 0;
        Vector3d xd_3dot, xd_4dot, b1d_dot, b1d_2dot;
        A = -kx * ex - kv * ev - m * g1 * e3 + m * xd_2dot;
        this->f = -A.dot(R*e3);

        Vector3d L = R*e3;
        Vector3d Ldot = R*utils.getSkewSymmetricMap(Omega)*e3;



        Vector3d ea = g1*e3-f/m*L-xd_2dot;
        Vector3d Adot = -kx*ev-kv*ea+m*xd_3dot;

        float fdot = -Adot.dot(L)-A.dot(Ldot);
        Vector3d eb = -fdot/m*L-f/m*Ldot-xd_3dot;
        Vector3d Ad_dot = -kx*ea-kv*eb+m*xd_4dot;

        float nA = A.norm();
        Vector3d Ld = -A/nA;
        Vector3d Ld_dot = -Adot/nA+A*A.dot(Adot)/pow(nA,3);
        Vector3d Ld_2dot = -Ad_dot/nA+Adot/pow(nA,3)*(2*A.dot(Adot))
                           +A/pow(nA,3)*(Adot.dot(Adot)+A.dot(Ad_dot))
                           -3*A/pow(nA,5)*pow(A.dot(Adot),2);

        Vector3d Ld2 = -utils.getSkewSymmetricMap(b1_d)*Ld;
        Vector3d Ld2_dot = -utils.getSkewSymmetricMap(b1d_dot)*Ld-utils.getSkewSymmetricMap(b1_d)*Ld_dot;
        Vector3d Ld2_2dot = -utils.getSkewSymmetricMap(b1d_2dot)*Ld-2*utils.getSkewSymmetricMap(b1d_dot)*Ld_dot-utils.getSkewSymmetricMap(b1_d)*Ld_2dot;

        float nLd2 = Ld2.norm();
        Vector3d Rd2 = Ld2/nLd2;
        Vector3d Rd2dot = Ld2_dot/nLd2-Ld2.dot(Ld_2dot)/pow(nLd2,3)*Ld2;
        Vector3d Rd2_2dot = Ld2_2dot/nLd2-Ld2.dot(Ld2_dot)/pow(nLd2,3)*Ld2_dot
                            -Ld2_dot.dot(Ld2_dot)/pow(nLd2,3)*Ld2-Ld2.dot(Ld2_2dot)/pow(nLd2,3)*Ld2
                            -Ld2.dot(Ld2_dot)/pow(nLd2,3)*Ld2_dot
                            +3*pow(Ld2.dot(Ld2_dot),2)/pow(nLd2,5)*Ld2;

        Vector3d Rd1 = utils.getSkewSymmetricMap(Rd2)*Ld;

        Vector3d Rd1dot = utils.getSkewSymmetricMap(Rd2dot)*Ld+utils.getSkewSymmetricMap(Rd2)*Ld_dot;
        Vector3d Rd1_2dot = utils.getSkewSymmetricMap(Rd2_2dot)*Ld+2*utils.getSkewSymmetricMap(Rd2dot)*Ld_dot+utils.getSkewSymmetricMap(Rd2)*Ld_2dot;

        Matrix3d Rd, Rd_dot, Rd_2dot;
        Rd << Rd1, Rd2, Ld;
        this->R_d = Rd;
        std::cout<<"\nR_d: \n"<<Rd<<std::endl;
        std::cout<<"\nR: \n"<<R<<std::endl;

        Rd_dot << Rd1dot, Rd2dot, Ld_dot;

        Rd_2dot << Rd1_2dot, Rd2_2dot, Ld_2dot;
        Omega_d = utils.getVeeMap(Rd.transpose()*Rd_dot);
        Omega_d_dot = (Omega_d - prev_Omega_d) / dt;
        prev_Omega_d = Omega_d;


        Omega_d_dot = utils.getVeeMap(Rd.transpose()*Rd_2dot-utils.getSkewSymmetricMap(Omega_d)*utils.getSkewSymmetricMap(Omega_d));
        std::cout<<"\nOmega_d : "<<Omega_d<<std::endl;

        ////
//my way of eOmega
//        Vector3d rpy_d_now = utils.R2RPY(R_d);
//        rpy_d_t_1 = utils.R2RPY(R_d_t_1);
//        Omega_d = (rpy_d_now - rpy_d_t_1) / dt;
//        Omega_d_dot = (Omega_d - prev_Omega_d) / dt;
//        prev_Omega_d = Omega_d;
//
        for(int i=0;i<3;i++)
            utils.publishOmega(Omega_d[i],i);
//
//        R_d_t_1 = R;
    }
//todo:
/**my way of calculating eomega
 * eOmega behaves like er for all body fixed axises.
 *
 * other way
 * eOmega of b1 and b2 axises behaves the same as er of those axises. when the drone moves away from the goal
 * diagonally b3 eOmega stays increased.
 *
 */


    void set_x_desired() {
        x_d[0] = 0;
        x_d[1] = 0;
        x_d[2] = -0.15;

        xdot_d[0] = 0;
        xdot_d[1] = 0;
        xdot_d[2] = -0.00;

        xd_2dot[0] = 0;
        xd_2dot[1] = 0;
        xd_2dot[2] = 0;
    }

    Vector4d getMotorForceVector(double totForce, Vector3d momentVec) {

        Vector4d f_m_vec(4, 1);
        Vector4d mot_force_vec(4, 1);
        f_m_vec << totForce, momentVec[0], momentVec[1], momentVec[2];
        std::cout << "\ntotForce: " << totForce << "\n";
        mot_force_vec = inv_coeffMat * f_m_vec;

//        std::cout << "\nmot_force_vec: " << mot_force_vec << "\n";

        return mot_force_vec;
    }

    double getAttitudeError() {
        Matrix3d eye = Matrix<double, 3, 3>::Identity();
        double att_error = (0.5 * (eye - R_d.transpose() * R)).trace();
        return att_error >= 0 ? att_error : 0;
    }

private:
    ros::NodeHandle n;
    float dt;
    float f;
    // desired parameters
    Vector3d x_d;
    Vector3d xdot_d;
    Vector3d xd_2dot;
    Vector3d b1_d;
    Matrix3d R_d;
    Matrix3d R_d_t_1;
    Vector3d Omega_d;
    Vector3d Omega_d_dot;
    double time_frame;
    // actual parameters
    Vector3d x;
    Vector3d v;
    Matrix3d R;
    Vector3d Omega;

    // initial parameters
    Vector3d x0;
    Vector3d v0;
    Matrix3d R0;
    Vector3d Omega0;

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
    static const float g = 9.80665;

    // translation vectors
    Vector3d e1;
    Vector3d e2;
    Vector3d e3;

    // force and moment vectors
    Vector3d M;
    Vector3d rpy_d_t_1;
    // other variables
    Vector3d prev_Omega_d;
    bool isFirst = true;
    geoControllerUtils utils;
    Matrix4d coeffMat;
    Matrix4d inv_coeffMat;
    dynamicsImpl *dynamics;

};