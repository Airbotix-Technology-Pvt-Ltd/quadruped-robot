#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <thread>
#include <chrono>


// Lite3 SDK messages (via drdds)
#include "drdds/msg/imu_data.hpp"
#include "drdds/msg/joints_data.hpp"
#include "drdds/msg/joints_data_cmd.hpp"

// MPC/WBC Core (Tophill Robotics)
#include "robots/qr_robot.h"
#include "exec/qr_robot_runner.h"

#include <csignal>
#include <atomic>

using namespace Quadruped;

// Global flag for clean shutdown
static std::atomic<bool> g_shutdown{false};

void SignalHandler(int) {
    g_shutdown.store(true);
}

/**
 * @brief Bridge class: Implements the TopHill qrRobot interface using Lite3 ROS2 topics.
 */
class qrRobotLite3 : public qrRobot {
public:
    qrRobotLite3(std::shared_ptr<rclcpp::Node> node, std::string homeDir) 
        : qrRobot("lite3", homeDir + "config/lite3/lite3_robot.yaml"), node_(node) 
    {
        // 0. Load Configuration (exactly as qrRobotLite3Sim does)
        robotConfig = YAML::LoadFile(configFilePath);
        auto rp = robotConfig["robot_params"];
        auto mp = robotConfig["motor_params"];

        // 1. Initial sensor state
        baseOrientation << 1.0f, 0.0f, 0.0f, 0.0f;
        baseRollPitchYaw << 0.0f, 0.0f, 0.0f;
        motorAngles.setZero();
        motorVelocities.setZero();
        motortorque.setZero();
        motorddq.setZero();
        footForce.setZero();
        footContact.fill(true);

        // 2. Mass and Inertia
        totalMass = rp["total_mass"].as<float>();
        bodyMass  = rp["body_mass"].as<float>();
        std::vector<float> totalInertiaVec = rp["total_inertia"].as<std::vector<float>>();
        totalInertia = Eigen::MatrixXf::Map(&totalInertiaVec[0], 3, 3);
        std::vector<float> bodyInertiaVec = rp["body_inertia"].as<std::vector<float>>();
        bodyInertia = Eigen::MatrixXf::Map(&bodyInertiaVec[0], 3, 3);
        std::vector<std::vector<float>> inertias = rp["links_inertia"].as<std::vector<std::vector<float>>>();
        std::vector<float> masses = rp["links_mass"].as<std::vector<float>>();
        std::vector<std::vector<float>> linksComPos_ = rp["links_com_pos"].as<std::vector<std::vector<float>>>();
        for (int legId=0; legId<NumLeg; ++legId) {
            Mat3<float> inertia = Eigen::MatrixXf::Map(&inertias[0][0], 3, 3);
            linkInertias.push_back(inertia);
            inertia = Eigen::MatrixXf::Map(&inertias[1][0], 3, 3);
            linkInertias.push_back(inertia);
            inertia = Eigen::MatrixXf::Map(&inertias[2][0], 3, 3);
            linkInertias.push_back(inertia);
            linkMasses.push_back(masses[0]);
            linkMasses.push_back(masses[1]);
            linkMasses.push_back(masses[2]);
            linksComPos.push_back(linksComPos_[0]);
            linksComPos.push_back(linksComPos_[1]);
            linksComPos.push_back(linksComPos_[2]);
        }

        // 3. Kinematics
        bodyHeight = rp["body_height"].as<float>();
        std::vector<float> abadLoc = rp["abad_location"].as<std::vector<float>>();
        abadLocation = Eigen::MatrixXf::Map(&abadLoc[0], 3, 1);
        hipLength      = rp["hip_l"].as<float>();
        upperLegLength = rp["upper_l"].as<float>();
        lowerLegLength = rp["lower_l"].as<float>();

        // Hip Offset (FR, FL, RR, RL)
        std::vector<std::vector<float>> hipOffsetList = rp["hip_offset"].as<std::vector<std::vector<float>>>();
        Vec3<float> ho0 = Eigen::MatrixXf::Map(&hipOffsetList[0][0], 3, 1);
        Vec3<float> ho1 = Eigen::MatrixXf::Map(&hipOffsetList[1][0], 3, 1);
        Vec3<float> ho2 = Eigen::MatrixXf::Map(&hipOffsetList[2][0], 3, 1);
        Vec3<float> ho3 = Eigen::MatrixXf::Map(&hipOffsetList[3][0], 3, 1);
        hipOffset << ho0, ho1, ho2, ho3;

        // Default Hip Positions
        std::vector<std::vector<float>> dhpList = rp["default_hip_positions"].as<std::vector<std::vector<float>>>();
        Vec3<float> dp0 = Eigen::MatrixXf::Map(&dhpList[0][0], 3, 1);
        Vec3<float> dp1 = Eigen::MatrixXf::Map(&dhpList[1][0], 3, 1);
        Vec3<float> dp2 = Eigen::MatrixXf::Map(&dhpList[2][0], 3, 1);
        Vec3<float> dp3 = Eigen::MatrixXf::Map(&dhpList[3][0], 3, 1);
        defaultHipPosition << dp0, dp1, dp2, dp3;

        // 4. Motor params
        float abadKp = mp["abad_p"].as<float>(), abadKd = mp["abad_d"].as<float>();
        float hipKp   = mp["hip_p"].as<float>(),   hipKd = mp["hip_d"].as<float>();
        float kneeKp  = mp["knee_p"].as<float>(),  kneeKd = mp["knee_d"].as<float>();
        Vec3<float> kps(abadKp, hipKp, kneeKp), kds(abadKd, hipKd, kneeKd);
        motorKps << kps, kps, kps, kps;
        motorKds << kds, kds, kds, kds;

        std::vector<float> jdList = mp["joint_directions"].as<std::vector<float>>();
        std::vector<float> joList = mp["joint_offsets"].as<std::vector<float>>();
        jointDirection = Eigen::MatrixXf::Map(&jdList[0], 12, 1);
        jointOffset    = Eigen::MatrixXf::Map(&joList[0], 12, 1);

        // 5. Stand / Sit angles
        float saAb   = rp["default_standup_angle"]["ab"].as<float>();
        float saHip  = rp["default_standup_angle"]["hip"].as<float>();
        float saKnee = rp["default_standup_angle"]["knee"].as<float>();
        Vec3<float> sa(saAb, saHip, saKnee);
        standUpMotorAngles << sa, sa, sa, sa;

        float sdAb   = rp["default_sitdown_angle"]["ab"].as<float>();
        float sdHip  = rp["default_sitdown_angle"]["hip"].as<float>();
        float sdKnee = rp["default_sitdown_angle"]["knee"].as<float>();
        Vec3<float> sd(sdAb, sdHip, sdKnee);
        sitDownMotorAngles << sd, sd, sd, sd;

        // 6. Control mode and reset (exactly as qrRobotLite3Sim)
        controlParams["mode"] = robotConfig["controller_params"]["mode"].as<int>();
        Reset();

        // 7. ROS 2 Comms
        sub_imu_ = node_->create_subscription<drdds::msg::ImuData>(
            "/IMU_DATA", 10, std::bind(&qrRobotLite3::ImuCallback, this, std::placeholders::_1));
        sub_joints_ = node_->create_subscription<drdds::msg::JointsData>(
            "/JOINTS_DATA", 10, std::bind(&qrRobotLite3::JointsCallback, this, std::placeholders::_1));
        pub_cmd_ = node_->create_publisher<drdds::msg::JointsDataCmd>("/JOINTS_CMD", 10);
        
        std::cout << "[Lite3 Bridge] Initialized." << std::endl;
    }

    void ReceiveObservation() override {
        // ROS2 handles this via callbacks. We just need to ensure the DataFlow is updated.
        if (node_) {
            rclcpp::spin_some(node_);
        }
        UpdateDataFlow();
    }

    void ApplyAction(const Eigen::MatrixXf &motor_commands, MotorMode motor_control_mode) override {
        drdds::msg::JointsDataCmd cmd;
        auto now = node_->get_clock()->now();
        cmd.header.stamp.sec = static_cast<int32_t>(now.seconds());
        cmd.header.stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1'000'000'000ULL);
        cmd.header.frame_id = 0;
        
        if (motor_control_mode == POSITION_MODE) {
            Eigen::Matrix<float, 12, 1> motorCommandsShaped = motor_commands;
            motorCommandsShaped = jointDirection.cwiseProduct(motorCommandsShaped) - jointOffset;
            for(int i=0; i<12; ++i) {
                cmd.data.joints_data[i].position = motorCommandsShaped[i];
                cmd.data.joints_data[i].kp = motorKps[i];
                cmd.data.joints_data[i].velocity = 0;
                cmd.data.joints_data[i].kd = motorKds[i];
                cmd.data.joints_data[i].torque = 0;
            }
        } else if (motor_control_mode == TORQUE_MODE) {
            Eigen::Matrix<float, 12, 1> motorCommandsShaped = motor_commands;
            motorCommandsShaped = jointDirection.cwiseProduct(motorCommandsShaped);
            for(int i=0; i<12; ++i) {
                cmd.data.joints_data[i].position = 0;
                cmd.data.joints_data[i].kp = 0;
                cmd.data.joints_data[i].velocity = 0;
                cmd.data.joints_data[i].kd = 0;
                cmd.data.joints_data[i].torque = motorCommandsShaped[i];
            }
        } else if (motor_control_mode == HYBRID_MODE) {
            Eigen::Matrix<float, 5, 12> motorCommandsShaped = motor_commands;
            Eigen::Matrix<float, 12, 1> angles = motorCommandsShaped.row(POSITION).transpose();
            motorCommandsShaped.row(POSITION) = (jointDirection.cwiseProduct(angles) - jointOffset).transpose();
            Eigen::Matrix<float, 12, 1> vels = motorCommandsShaped.row(VELOCITY).transpose();
            motorCommandsShaped.row(VELOCITY) = jointDirection.cwiseProduct(vels).transpose();
            Eigen::Matrix<float, 12, 1> tuas = motorCommandsShaped.row(TORQUE).transpose();
            motorCommandsShaped.row(TORQUE) = jointDirection.cwiseProduct(tuas).transpose();

            for(int i=0; i<12; ++i) {
                cmd.data.joints_data[i].position = motorCommandsShaped(POSITION, i);
                cmd.data.joints_data[i].kp = motorCommandsShaped(KP, i);
                cmd.data.joints_data[i].velocity = motorCommandsShaped(VELOCITY, i);
                cmd.data.joints_data[i].kd = motorCommandsShaped(KD, i);
                cmd.data.joints_data[i].torque = motorCommandsShaped(TORQUE, i);
            }
        }
        pub_cmd_->publish(cmd);
    }

    void ShutdownDamping() {
        drdds::msg::JointsDataCmd cmd;
        auto now = node_->get_clock()->now();
        cmd.header.stamp.sec = static_cast<int32_t>(now.seconds());
        cmd.header.stamp.nanosec = static_cast<uint32_t>(now.nanoseconds() % 1'000'000'000ULL);
        cmd.header.frame_id = 0;
        for(int i=0; i<12; ++i) {
            cmd.data.joints_data[i].position = 0;
            cmd.data.joints_data[i].kp = 0;
            cmd.data.joints_data[i].velocity = 0;
            cmd.data.joints_data[i].kd = 2.0; // Damping
            cmd.data.joints_data[i].torque = 0;
        }
        pub_cmd_->publish(cmd);
        std::cout << "[Lite3 Bridge] Shutdown damping applied." << std::endl;
    }

    void Step(const Eigen::MatrixXf &action, MotorMode motor_control_mode) override {
        ReceiveObservation();
        ApplyAction(action, motor_control_mode);
    }

private:
    void ImuCallback(const drdds::msg::ImuData::SharedPtr msg) {
        // Convert RPY to quat
        float roll = msg->data.roll;
        float pitch = msg->data.pitch;
        float yaw = msg->data.yaw;
        Eigen::Vector3f rpy(roll, pitch, yaw);
        
        baseOrientation = robotics::math::rpyToQuat(rpy);
        baseRollPitchYawRate << msg->data.omega_x, msg->data.omega_y, msg->data.omega_z;
        baseAccInBaseFrame << msg->data.acc_x, msg->data.acc_y, msg->data.acc_z;
        baseRollPitchYaw = rpy;
    }

    void JointsCallback(const drdds::msg::JointsData::SharedPtr msg) {
        for(int i=0; i<12; ++i) {
            motorAngles[i] = jointDirection[i] * (msg->data.joints_data[i].position + jointOffset[i]);
            motorVelocities[i] = jointDirection[i] * msg->data.joints_data[i].velocity;
        }
        isSim = true;
    }

    std::shared_ptr<rclcpp::Node> node_;
    rclcpp::Subscription<drdds::msg::ImuData>::SharedPtr sub_imu_;
    rclcpp::Subscription<drdds::msg::JointsData>::SharedPtr sub_joints_;
    rclcpp::Publisher<drdds::msg::JointsDataCmd>::SharedPtr pub_cmd_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("lite3_mpc_controller");

    // Signal handler for Ctrl-C
    std::signal(SIGINT, SignalHandler);

    // Path to config files (aligned with Docker layout)
    std::string homeDir = "/workspace/quadruped-robot/quadruped/";
    
    // 1. Create the Robot Bridge
    auto robot = new qrRobotLite3(node, homeDir);
    
    // 2. Ensure we have sensor data FIRST before starting the runner
    std::cout << "[MPC] Waiting for initial sensor data..." << std::endl;
    for(int i=0; i<100; ++i) {
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // 3. Initialize the Robot Runner
    // Explicitly create a NodeHandle stub from the rclcpp Node
    ros::NodeHandle nh(node);
    
    std::cout << "[MPC] Init Runner (Robot standing up)..." << std::endl;
    // The runner constructor already triggers Action::StandUp, which blocks for 5 seconds locally
    qrRobotRunner runner(robot, homeDir, nh);

    // 4. Main Control Loop (500Hz)
    rclcpp::Rate rate(500);
    std::cout << "[MPC] Controller Ready. Starting Gait..." << std::endl;
    
    while (rclcpp::ok() && !g_shutdown.load()) {
        rclcpp::spin_some(node);
        
        // Update State Estimators, Planners, and FSM
        runner.Update();
        
        // Execute the Step (publishes to /JOINTS_CMD)
        runner.Step();

        // Debug: Log planner targets every 1s
        static int debug_tick = 0;
        if (++debug_tick % 500 == 0) {
            auto cmd = runner.GetDesiredStateCommand();
            if (cmd) {
                // stateDes(6) is Desired Vx, stateDes(11) is Desired YawRate
                std::cout << "[MPC Debug] Target_Vx: " << cmd->stateDes(6) 
                          << " | Mode: " << (int)cmd->getJoyCtrlState() 
                          << " | Standing: " << (int)(runner.GetLocomotionController()->GetState() == 1) << std::endl;
            }
        }
        
        rate.sleep();
    }

    std::cout << "[MPC] Shutting down safely..." << std::endl;
    robot->ShutdownDamping();
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    rclcpp::shutdown();
    return 0;
}
