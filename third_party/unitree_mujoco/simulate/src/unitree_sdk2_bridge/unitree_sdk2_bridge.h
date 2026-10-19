#include <mutex>
#include <atomic>
#ifndef UNITREE_SDK2_BRIDGE_H
#define UNITREE_SDK2_BRIDGE_H

#include <iostream>
#include <chrono>
#include <cstring>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/go2/WirelessController_.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <mujoco/mujoco.h>
#include "../joystick/joystick.h"

using namespace unitree::common;
using namespace unitree::robot;
using namespace std;

#define TOPIC_LOWSTATE "rt/lowstate"
#define TOPIC_HIGHSTATE "rt/sportmodestate"
#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_WIRELESS_CONTROLLER "rt/wirelesscontroller"
#define MOTOR_SENSOR_NUM 3
#define NUM_MOTOR_IDL_GO 20
#define NUM_MOTOR_IDL_HG 35

typedef union
{
    struct
    {
        uint8_t R1 : 1;
        uint8_t L1 : 1;
        uint8_t start : 1;
        uint8_t select : 1;
        uint8_t R2 : 1;
        uint8_t L2 : 1;
        uint8_t F1 : 1;
        uint8_t F2 : 1;
        uint8_t A : 1;
        uint8_t B : 1;
        uint8_t X : 1;
        uint8_t Y : 1;
        uint8_t up : 1;
        uint8_t right : 1;
        uint8_t down : 1;
        uint8_t left : 1;
    } components;
    uint16_t value;
} xKeySwitchUnion;

typedef struct
{
    uint8_t head[2];
    xKeySwitchUnion btn;
    float lx;
    float rx;
    float ry;
    float L2;
    float ly;

    uint8_t idle[16];
} xRockerBtnDataStruct;

// Defaults to xbox gamepad
struct JoystickId
{

    map<string, int> axis =
        {
            {"LX", 0}, // Left stick axis x
            {"LY", 1}, // Left stick axis y
            {"RX", 3}, // Right stick axis x
            {"RY", 4}, // Right stick axis y
            {"LT", 2}, // Left trigger
            {"RT", 5}, // Right trigger
            {"DX", 6}, // Directional pad x
            {"DY", 7}, // Directional pad y
    };

    map<string, int> button =
        {
            {"X", 2},
            {"Y", 3},
            {"B", 1},
            {"A", 0},
            {"LB", 4},
            {"RB", 5},
            {"SELECT", 6},
            {"START", 7},
    };
};

class UnitreeSdk2Bridge
{
public:
    UnitreeSdk2Bridge(mjModel *model, mjData *data);
    ~UnitreeSdk2Bridge();

    void LowCmdGoHandler(const void *msg);
    void LowCmdHgHandler(const void *msg);

    void SetPhysicsMutex(std::recursive_mutex *mtx) { physics_mtx_ = mtx; }
    void PublishLowStateGo();
    void PublishLowStateHg();
    void PublishHighState();
    void PublishWirelessController();
    void Run();
    void PrintSceneInformation();
    void CheckSensor();
    void SetupJoystick(string device, string js_type, int bits);

    ChannelSubscriberPtr<unitree_go::msg::dds_::LowCmd_> low_cmd_go_suber_;
    ChannelSubscriberPtr<unitree_hg::msg::dds_::LowCmd_> low_cmd_hg_suber_;

    unitree_go::msg::dds_::LowState_ low_state_go_{};
    unitree_hg::msg::dds_::LowState_ low_state_hg_{};
    unitree_go::msg::dds_::SportModeState_ high_state_{};
    unitree_go::msg::dds_::WirelessController_ wireless_controller_{};

    ChannelPublisherPtr<unitree_go::msg::dds_::LowState_> low_state_go_puber_;
    ChannelPublisherPtr<unitree_hg::msg::dds_::LowState_> low_state_hg_puber_;
    ChannelPublisherPtr<unitree_go::msg::dds_::SportModeState_> high_state_puber_;
    ChannelPublisherPtr<unitree_go::msg::dds_::WirelessController_> wireless_controller_puber_;

    ThreadPtr lowStatePuberThreadPtr;
    ThreadPtr HighStatePuberThreadPtr;
    ThreadPtr WirelessControllerPuberThreadPtr;

    xKeySwitchUnion dds_keys_ = {};
    xRockerBtnDataStruct wireless_remote_ = {};

    JoystickId js_id_;
    // Published by SetupJoystick() only after the device is open and js_id_ and
    // max_value_ are filled in. The 500 Hz publisher threads start in the
    // constructor, before SetupJoystick() runs, and test this pointer -- it used
    // to be uninitialised, so they could dereference garbage.
    std::atomic<Joystick *> js_{nullptr};
    // getState() reads the device into the shared Joystick::event_ and writes
    // button_/axis_. Two threads call it (lowstate -> GetWirelessRemote, and
    // wirelesscontroller), so every joystick read holds this.
    std::mutex js_mtx_;
    int max_value_ = (1 << 15); // 16 bits joystick

    mjData *mj_data_;
    mjModel *mj_model_;

    int num_motor_ = 0;
    int dim_motor_sensor_ = 0;

    int have_imu_ = false;
    int have_frame_sensor_ = false;
    int idl_type_ = 0; // 0: unitree_go, 1: unitree_hg

    // Contact geoms for the four feet, in the FL/FR/RL/RR order the foot_force
    // field uses. Resolved once from the model; -1 means the model does not name
    // that geom, in which case its contact force is reported as zero.
    int foot_geom_ids_[4] = {-1, -1, -1, -1};

    // Physics mutex, owned by main()'s Simulate instance. The publisher threads
    // read mjData while PhysicsLoop is inside mj_step writing it; locking this
    // serialises the two. Null means unguarded (the old, crash-prone behaviour).
    std::recursive_mutex *physics_mtx_ = nullptr;

private:
    void GetWirelessRemote();
};

#endif
