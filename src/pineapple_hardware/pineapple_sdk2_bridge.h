#ifndef PINEAPPLE_SDK2_BRIDGE_H
#define PINEAPPLE_SDK2_BRIDGE_H

#include <iostream>
#include <chrono>
#include <cstring>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include "../canfd/include/damiao.h"
#include "../imu/xsens_imu.hpp"

using namespace unitree::common;
using namespace unitree::robot;
using namespace std;

#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_LOWCMD "rt/lowcmd"
#define MOTOR_SENSOR_NUM 3
#define NUM_MOTOR_IDL_GO 20

struct MotorPosLimit {
    double lower;
    double upper;
};

struct MotorConfig {
    bool set_zero = false;
    vector<double> motor_offset;
    vector<double> direction;
    vector<MotorPosLimit> pos_limit;
};

class PineappleSdk2Bridge
{
public:
    PineappleSdk2Bridge(const char *dev_sn, const MotorConfig &motor_config);
    ~PineappleSdk2Bridge();

    void LowCmdGoHandler(const void *msg);
    void PublishLowStateGo();

    // Motor related
    void SetMotorToZero();
    void AddMotorOffset();

    // IMU related
    void InitXsensIMU();
    void ProcessXsensData();
    void CloseXsensIMU();

    ChannelSubscriberPtr<unitree_go::msg::dds_::LowCmd_> low_cmd_go_suber_;

    unitree_go::msg::dds_::LowState_ low_state_go_{};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowState_> low_state_go_puber_;

    ThreadPtr lowStatePuberThreadPtr;


    int num_motor_ = 8;
    int dim_motor_sensor_ = 0;

    int have_imu_ = true;
    bool set_zero_ = false;
    vector<uint16_t> can_id_list{0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08}; // L_hip L_thigh L_calf L_wheel R_hip R_thigh R_calf R_wheel
    vector<uint16_t> mst_id_list{0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 0x18};
    vector<int> motor_type{damiao::DM8006, damiao::DM8006, damiao::DM8009, damiao::DM6006, damiao::DM8006, damiao::DM8006, damiao::DM8009, damiao::DM6006};

    // Populated from config.yaml at construction time.
    vector<double> motor_offset;
    vector<double> direction;
    vector<MotorPosLimit> pos_limit_;

    private:

    void HandleMotorFault(int motor_idx);
    bool IsMotorConnected(int motor_idx) const;

    bool is_running = true;
    // Motor related
    std::shared_ptr<damiao::Motor_Control> motor_control;
    uint32_t nom_baud =1000000;
    uint32_t dat_baud =5000000;
    vector<damiao::DmActData> dm_data_list;
    vector<std::chrono::steady_clock::time_point> last_fault_recovery_time_;
    vector<std::chrono::steady_clock::time_point> last_disconnect_log_time_;
    double feedback_timeout_sec_ = 0.1;  // no feedback within this window => disconnected
    std::chrono::steady_clock::time_point init_time_;
    double init_grace_sec_ = 5.0;        // suppress disconnect reports during startup
    std::chrono::steady_clock::time_point last_poll_time_;
    double poll_interval_sec_ = 0.02;    // actively solicit feedback at 50 Hz

    // IMU related
    std::thread xsens_imu_thread;
    std::shared_ptr<ImuSharedData> xsens_imu_data;
    XsControl* xsens_control = nullptr;
    XsPortInfo xsens_mtPort;
    CallbackHandler xsens_callback;
    XsDevice* xsens_device = nullptr;

};

#endif
