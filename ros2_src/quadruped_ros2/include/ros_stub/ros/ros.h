#ifndef ROS_ROS_H_SHIM
#define ROS_ROS_H_SHIM

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <cstdio>
#include <cmath>
#include <array>
#include <functional>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/int8.hpp>

namespace boost {
    template<typename T, std::size_t N>
    using array = std::array<T, N>;
}

// ── Standard ROS 2 Namespaces (Aliased for legacy code compatibility) ───────
namespace std_msgs {
    using Header = msg::Header;
    using Float32 = msg::Float32;
    using Int8 = msg::Int8;
}

namespace geometry_msgs {
    using Vector3 = msg::Vector3;
    using Point = msg::Point;
    using Quaternion = msg::Quaternion;
    using Pose = msg::Pose;
    using Twist = msg::Twist;
    using WrenchStamped = msg::WrenchStamped;
    using Vector3Ptr = std::shared_ptr<Vector3>;
    using Vector3ConstPtr = std::shared_ptr<const Vector3>;
    
    inline std::ostream& operator<<(std::ostream& os, const msg::Vector3& v) {
        return os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
    }
}

namespace sensor_msgs {
    using Joy = msg::Joy;
    using Imu = msg::Imu;
    using JointState = msg::JointState;
}

namespace nav_msgs {
    using Odometry = msg::Odometry;
}

namespace gazebo_msgs {
    struct LinkState {
        geometry_msgs::Pose pose;
        geometry_msgs::Twist twist;
    };
    struct GetLinkState {
        struct Request { std::string link_name, reference_frame; } request;
        struct Response { LinkState link_state; bool success; } response;
    };
}

namespace ros {
    
    struct Time {
        Time() : val(0.0) {}
        Time(double d) : val(d) {}
        static Time now() { 
            auto now = std::chrono::system_clock::now();
            return Time(std::chrono::duration<double>(now.time_since_epoch()).count());
        }
        double toSec() const { return val; }
        double val;
        
        Time operator+(double d) const { return Time(val + d); }
        Time operator-(double d) const { return Time(val - d); }
        double operator-(const Time& t) const { return val - t.val; }
        bool operator>(const Time& other) const { return val > other.val; }
        bool operator<(const Time& other) const { return val < other.val; }
        bool operator>=(const Time& other) const { return val >= other.val; }
        bool operator<=(const Time& other) const { return val <= other.val; }
    };
    
    struct Duration {
        Duration(double d) : val(d) {}
        double toSec() const { return val; }
        double val;
    };

    struct Subscriber {
        std::shared_ptr<void> sub_handle;
    };
    
    struct Publisher {
        std::shared_ptr<void> pub_handle;
        std::function<void(const void*)> publish_fn;
        template<typename M> void publish(const M& msg) { if (publish_fn) publish_fn(&msg); }
    };

    struct ServiceClient {
        template<typename Srv> bool call(Srv& srv) { return true; }
        bool isValid() const { return true; }
    };

    struct NodeHandle {
        NodeHandle() : node_ptr(nullptr) {}
        NodeHandle(const std::string& ns) : node_ptr(nullptr) {}
        NodeHandle(std::shared_ptr<rclcpp::Node> node) : node_ptr(node) {}
        
        template<typename T, typename M>
        Subscriber subscribe(const std::string& topic, int q, void(T::*fp)(const std::shared_ptr<const M>&), T* obj) {
            if (!node_ptr) return Subscriber();
            auto sub = node_ptr->create_subscription<M>(topic, q, [obj, fp](const std::shared_ptr<const M> msg) {
                (obj->*fp)(msg);
            });
            return {sub};
        }

        template<typename T, typename M>
        Subscriber subscribe(const std::string& topic, int q, void(T::*fp)(const M&), T* obj) {
            if (!node_ptr) return Subscriber();
            auto sub = node_ptr->create_subscription<M>(topic, q, [obj, fp](const std::shared_ptr<const M> msg) {
                (obj->*fp)(*msg);
            });
            return {sub};
        }

        template<typename M>
        Publisher advertise(const std::string& topic, int q) {
            if (!node_ptr) return Publisher();
            auto pub = node_ptr->create_publisher<M>(topic, q);
            Publisher p;
            p.pub_handle = pub;
            p.publish_fn = [pub](const void* msg) { if(msg) pub->publish(*(const M*)msg); };
            return p;
        }
        
        template<typename Srv> ServiceClient serviceClient(const std::string& name) { return ServiceClient(); }
        template<typename T> void param(const std::string& key, T& val, const T& def) { val = def; }

        std::shared_ptr<rclcpp::Node> node_ptr;
    };

    namespace package {
        inline std::string getPath(const std::string& name) {
            return "/workspace/quadruped-robot/quadruped";
        }
    }

    inline void init(int& argc, char** argv, const std::string& name) { }
    inline bool ok() { return rclcpp::ok(); }
    inline void spinOnce() { }
}

#define ROS_INFO printf
#define ROS_WARN printf
#define ROS_ERROR printf
#define ROS_INFO_STREAM(x) {std::cout << x << std::endl;}
#define ROS_WARN_STREAM(x) {std::cout << x << std::endl;}
#define ROS_ERROR_STREAM(x) {std::cerr << x << std::endl;}
#define ROS_DEBUG_STREAM(x) {std::cout << x << std::endl;}

namespace tf {
    struct Vector3 {
        double vx, vy, vz;
        Vector3(double x=0, double y=0, double z=0) : vx(x), vy(y), vz(z) {}
        double x() const { return vx; }
        double y() const { return vy; }
        double z() const { return vz; }
    };
    struct Quaternion {
        double qx, qy, qz, qw;
        Quaternion(double x=0, double y=0, double z=0, double w=1) : qx(x), qy(y), qz(z), qw(w) {}
    };
    struct Transform {
        geometry_msgs::Vector3 translation;
        geometry_msgs::Quaternion rotation;
        void setOrigin(const Vector3& v) { translation.x = v.vx; translation.y = v.vy; translation.z = v.vz; }
        template<typename T> void setRotation(const T& q) {}
    };
    struct StampedTransform : public Transform {
        StampedTransform(const Transform& t, ros::Time time, const std::string& p, const std::string& c) {}
    };
    struct TransformBroadcaster {
        void sendTransform(const StampedTransform& t) {}
    };
}

namespace unitree_legged_msgs {
    struct MotorCmd { int mode; float q, dq, ddq, Kp, Kd, tau; };
    struct MotorState { int mode; float q, dq, ddq, tauEst; };
    struct LowState {
        struct IMU { boost::array<float, 4> quaternion; boost::array<float, 3> gyroscope, accelerometer, rpy; } imu;
        boost::array<MotorState, 20> motorState;
        boost::array<int16_t, 4> footForce;
        struct Vec3 { float x, y, z; };
        boost::array<Vec3, 4> eeForce;
        using Ptr = std::shared_ptr<LowState>;
        using ConstPtr = std::shared_ptr<const LowState>;
    };
    struct LowCmd { int mode; boost::array<MotorCmd, 20> motorCmd; using Ptr = std::shared_ptr<LowCmd>; };
}

// ── Legacy Hardware SDK Stubs ────────────────────────────────────────────────
struct RobotState {
    struct MotorState { struct JointData { float pos, vel, tor; }; boost::array<JointData, 24> joint_data; } motor_state;
    struct IMU { boost::array<float, 4> quaternion; boost::array<float, 3> rpy; float angle_roll, angle_pitch, angle_yaw; float angular_velocity_roll, angular_velocity_pitch, angular_velocity_yaw; float acc_x, acc_y, acc_z; } imu;
    boost::array<int, 4> footForce;
    uint32_t tick;
};
struct RobotCmd { struct JointCmd { float pos, vel, kp, kd, tor; }; boost::array<JointCmd, 24> joint_cmd; };
class ParseCMD { public: void startWork() {} RobotState& get_recv() { static RobotState s; return s; } };
class SendToRobot { public: void init() {} void robot_state_init() {} void set_send(const RobotCmd& cmd) {} };

#define ROS_INFO printf
#define ROS_WARN printf
#define ROS_ERROR printf

#endif
