//
// Created by malintha on 11/22/17.
//
#include "ros/ros.h"
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include "dynamicsImpl.h"
#include "controllerImpl.h"

enum State {
    Idle = 0,
    Automatic = 1,
    TakingOff = 2,
    Landing = 3,
};

class GeoController {

public:
    GeoController(const std::string &worldFrame, const std::string &frame,
                  const ros::NodeHandle &n) : m_serviceTakeoff(), m_serviceLand(), m_state(Automatic), m_force(0),
                                              m_startZ(0), m_worldFrame(worldFrame), m_bodyFrame(frame) {
        ros::NodeHandle nh;
        m_listener.waitForTransform(m_worldFrame, m_bodyFrame, ros::Time(0), ros::Duration(10.0));
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        m_pubThrust = nh.advertise<geometry_msgs::Twist>("cmd_thrust", 1);

        m_serviceTakeoff = nh.advertiseService("takeoff", &GeoController::takeoff, this);
        m_serviceLand = nh.advertiseService("land", &GeoController::land, this);
        dynamics = new dynamicsImpl(n, m_worldFrame, m_bodyFrame);
        controllerImpl = new ControllerImpl(n, dynamics);
        counter_started = false;
        utils = new geoControllerUtils;
        ROS_INFO("###GeoController Initialized###\n");
    };

    void run(double frequency) {
        this->frequency = frequency;
        ros::Timer timer = node.createTimer(ros::Duration(1.0 / frequency), &GeoController::iteration, this);
        ros::spin();
    }

    void iteration(const ros::TimerEvent &e) {
        if (!counter_started) {
            this->t_frame = 0;
            counter_started = true;
        }

        float dt = e.current_real.toSec() - e.last_real.toSec();
        if (dt > 10)
            dt = 0.02;
        t_frame += dt;
        tf::StampedTransform transform = getTransform();
//        m_listener.lookupTransform(m_worldFrame, m_bodyFrame, ros::Time(0), transform);
        dynamics->setdt(dt);
        Vector3d *x_arr = dynamics->get_x_v_Omega(transform, t_frame);
        Matrix3d R = dynamics->getR();
//        std::cout << "R: " << R << std::endl;
        Vector3d x = x_arr[0];
        utils->publishX(x[2], 2);
        Vector3d x_dot = x_arr[1];
        Vector3d Omega = x_arr[2];
        controllerImpl->setDynamicsValues(x, x_dot, R, Omega);

        switch (m_state) {
            case TakingOff: {
                m_force = 0.1;
                    controllerImpl->setInitValues(dt, t_frame);
                    std::cout << "taking off: thrust: " << m_force << " z: " << transform.getOrigin().z() << "\n";
                    Vector3d M = controllerImpl->getMomentVector();
                    Vector4d motorForces = controllerImpl->getMotorForceVector(m_force, M);
                    publishToCf(m_force, motorForces);
            }
                break;
            case Landing: {
                geometry_msgs::Twist msg;
                msg.linear.x = 0;
                msg.linear.y = 0;
                msg.linear.z = 0;
                msg.angular.x = 0;
                m_pubThrust.publish(msg);
            }

            case Automatic: {
                double f;
                Vector3d momentVec;
                Vector4d mot_force_vec;

                if (R.minCoeff() < 0) {
//                    std::cout<<"\nt_frame: "<<t_frame<<std::endl;
                    controllerImpl->setInitValues(dt, t_frame);
                    f = controllerImpl->getTotalForce();
                    momentVec = controllerImpl->getMomentVector();
//                    if(transform.getOrigin().z() < 0.05) {
//                        momentVec[0] = momentVec[1] = momentVec[2] = 0;
//                    }
                    mot_force_vec = controllerImpl->getMotorForceVector(f, momentVec);
                    publishToCf(f, mot_force_vec);
                }
            }
                break;
            case Idle: {
            }
                break;
        }
    }


private:
    State m_state;
    ros::Publisher m_pubNav;
    ros::Publisher m_pubThrust;
    std::string m_worldFrame;
    std::string m_bodyFrame;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    tf::TransformListener m_listener;
    float m_force;
    float m_startZ; // only for landing to the start z value
    geometry_msgs::PoseStamped m_goal;
    dynamicsImpl *dynamics;
    float frequency;
    ros::NodeHandle node;
    ControllerImpl *controllerImpl;
    double t_frame;
    bool counter_started;
    geoControllerUtils *utils;
    Vector3d e3;

    bool takeoff(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        ROS_INFO("Takeoff requested!");
        tf::StampedTransform transform;
        m_state = TakingOff;

        return true;
    }

    bool land(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
        ROS_INFO("Landing requested!");
        m_state = Landing;
        return true;
    }

    void publishToCf(double f, Vector4d mot_force_vec) {
        double RPM1 = 0;
        double RPM2 = 0;
        double RPM3 = 0;
        double RPM4 = 0;

        RPM1 = utils->getTargetRatio(mot_force_vec[0]);
        RPM2 = utils->getTargetRatio(mot_force_vec[1]);
        RPM3 = utils->getTargetRatio(mot_force_vec[2]);
        RPM4 = utils->getTargetRatio(mot_force_vec[3]);

        for (int i = 0; i < 4; i++)
            utils->publishThrusts(mot_force_vec[i], i);
        utils->publishThrusts(f, 4);

//        std::cout << "MotorRatios: " << RPM1 << " " << RPM2 << " " << RPM3 << " " << RPM4 << "\n";

        utils->publishMotorRatios(RPM1, 1);
        utils->publishMotorRatios(RPM2, 2);
        utils->publishMotorRatios(RPM3, 3);
        utils->publishMotorRatios(RPM4, 4);

        geometry_msgs::Twist msg;
        msg.linear.x = RPM2;
        msg.linear.y = RPM3;
        msg.linear.z = RPM4;
        msg.angular.x = RPM1;
        m_pubThrust.publish(msg);
     }

    tf::StampedTransform getTransform() {
        tf::StampedTransform transform;
        m_listener.lookupTransform(m_worldFrame, m_bodyFrame, ros::Time(0), transform);
        transform.getOrigin().setY(-transform.getOrigin().y());
        transform.getOrigin().setZ(-transform.getOrigin().z());
        return transform;
    }

};


int main(int argc, char **argv) {
    ros::init(argc, argv, "controller");

    static ros::NodeHandle n("~");
    std::string worldFrame;
    n.param<std::string>("/crazyflie/geocontroller/worldFrame", worldFrame, "/world");
    std::string frame;
    n.getParam("/crazyflie/geocontroller/frame", frame);
    double frequency;
    n.param("frequency", frequency, 50.0);

    ROS_INFO("### Initializing geoController. worldFrame:%s bodyFrame: %s\n", worldFrame.data(), frame.data());

    GeoController controller(worldFrame, frame, n);
    controller.run(frequency);

    return 0;
}
