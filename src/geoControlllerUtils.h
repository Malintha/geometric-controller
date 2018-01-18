//
// Created by malintha on 11/29/17.
//

#ifndef PROJECT_GEOCONTROLLLERUTILS_H
#define PROJECT_GEOCONTROLLLERUTILS_H
#endif

#pragma once

#include <eigen3/Eigen/Dense>
#include <ros/node_handle.h>

using namespace Eigen;


class geoControllerUtils {
public:
    geoControllerUtils() {
        thrustRPMMap.insert(std::pair<double, double >(0.0,0));
        thrustRPMMap.insert(std::pair<double, double >(1.6,4485));
        thrustRPMMap.insert(std::pair<double, double >(4.8,7570));
        thrustRPMMap.insert(std::pair<double, double >(7.9,9374));
        thrustRPMMap.insert(std::pair<double, double >(10.9,10885));
        thrustRPMMap.insert(std::pair<double, double >(13.9,12277));
        thrustRPMMap.insert(std::pair<double, double >(17.3,13522));
        thrustRPMMap.insert(std::pair<double, double >(21.0,14691));
        thrustRPMMap.insert(std::pair<double, double >(24.4,15924));
        thrustRPMMap.insert(std::pair<double, double >(28.6,17174));
        thrustRPMMap.insert(std::pair<double, double >(32.8,18179));
        thrustRPMMap.insert(std::pair<double, double >(37.3,19397));
        thrustRPMMap.insert(std::pair<double, double >(41.7,20539));
        thrustRPMMap.insert(std::pair<double, double >(46.0, 21692));
        thrustRPMMap.insert(std::pair<double, double >(51.9,22598));
        thrustRPMMap.insert(std::pair<double, double >(57.9,23882));

    }

    double get(const ros::NodeHandle &n, const std::string &name) {
        const std::string node_prefix = "/crazyflie/geocontroller/";
        std::string key = node_prefix+name;
        double value;
        n.getParam(key, value);
        return value;
    }

    double getTargetRPM(double targetThrust) {
        double x0, x1, y0, y1, targetRPM, targetThrust1;
        targetRPM = 0;
        targetThrust1 = abs(targetThrust);


        if(targetThrust1 > 57)
            targetThrust1 = 57;

        if(targetThrust != 0) {

            std::map<double, double>::iterator it = thrustRPMMap.begin();
            for (it = thrustRPMMap.begin(); it != thrustRPMMap.end(); ++it) {
                if (it->first > targetThrust1) {
                    std::map<double, double>::iterator it_prev = it--;
                    y0 = it_prev->first;
                    x0 = it_prev->second;
                    y1 = it->first;
                    x1 = it->second;
                    break;
                }
            }
            targetRPM = x1 - (((y1 - targetThrust1) * (x1 - x0)) / (y1 - y0));
        }
//        if (targetThrust < 0)
//            targetRPM = -targetRPM;
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

private:
    Vector3d x0;
    Vector3d v0;
    Matrix3d R0;
    Vector3d Omega0;
    std::map<double,double> thrustRPMMap;
};

