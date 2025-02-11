#include <chrono>
#include <cstdio>
#include <cmath>
#include <cassert>
#include "Haply_HardwareAPI/HardwareAPI.h"
#include "unitree_arm_sdk/control/unitreeArm.h"

namespace API = Haply::HardwareAPI;
using namespace std::chrono_literals;

double GRIPPER_OPEN = -1.0;
double GRIPPER_CLOSED = 0.0;
bool hasGripper = true;
bool linearize = false;

bool linear_step(Vec6 ideal_goal, UNITREE_ARM::unitreeArm* arm, double linear_vel){
    Vec6 current_pos = arm->lowstate->endPosture;
    double dt = arm->_ctrlComp->dt;
    Vec6 full_new = ideal_goal - current_pos;

    float full_length = full_new.norm();
    float max_length = linear_vel * dt;

    bool success;
    HomoMat goal_mat;
    if(full_length <= max_length){
        goal_mat = UNITREE_ARM::postureToHomo(ideal_goal);
    }
    else{
        Vec6 new_goal = current_pos + full_new * max_length;
        goal_mat = UNITREE_ARM::postureToHomo(new_goal);
    }

    Vec6 q_goal;

    if ( arm->_ctrlComp->armModel->inverseKinematics(goal_mat, Vec6::Zero(), q_goal, true) ){
        arm->q = q_goal;

        arm->setArmCmd(arm->q, arm->qd, arm->tau);
        return true;
    }
    else{
        std::cout << "IK FAIL\n";
        return false;
    }
}

bool joint_step(Vec6 ideal_goal, UNITREE_ARM::unitreeArm* arm, double joint_vel){
    Vec6 ideal_pos;

    if ( arm->_ctrlComp->armModel->inverseKinematics(UNITREE_ARM::postureToHomo(ideal_goal), Vec6::Zero(), ideal_pos, true) ){
        Vec6 current_pos = arm->_ctrlComp->lowcmd->getQ();
        Vec6 delta_pos = ideal_pos - current_pos;

        // Should be switched to max instead of norm
        double delta_len = delta_pos.norm();
        delta_pos = delta_pos / delta_len;
        double dt = arm->_ctrlComp->dt;

        if(delta_len <= dt * joint_vel){
            arm->q = ideal_pos;
        }
        else{
            Vec6 new_pos = current_pos + delta_pos * dt * joint_vel;
            arm->q = new_pos;
        }

        arm->setArmCmd(arm->q, arm->qd, arm->tau);
        return true;
    }
    else{
        std::cout << "IK FAIL\n";
        return false;
    }
}

int main(int argc, const char *argv[]) {
    if (argc < 3) {
        std::fprintf(stderr, "usage: haply_z1_cpp [inverse-port] [handle-port]\n");
        return 1;
    }

    UNITREE_ARM::unitreeArm arm(hasGripper);
    arm.sendRecvThread->start();
    arm.backToStart();
    arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    arm.setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);

    std::vector<double> KP, KD;
    KP = arm._ctrlComp->lowcmd->kp;
    KD = arm._ctrlComp->lowcmd->kd;
    arm.sendRecvThread->shutdown();

    UNITREE_ARM::Posture goal_posture;
    goal_posture.rx = 0.0;
    goal_posture.ry = 0.0;
    goal_posture.rz = 0.0;

    API::IO::SerialStream inv_stream{argv[1]};
    API::Devices::Inverse3 inv_device{&inv_stream};
    (void)inv_device.DeviceWakeup();

    Haply::HardwareAPI::IO::SerialStream serial_stream(argv[2]);
    Haply::HardwareAPI::Devices::Handle handle(&serial_stream);

    typedef std::chrono::high_resolution_clock clock;
    auto next = clock::now();
    auto delay = 1000us; // 1kHz

    arm._ctrlComp->dt = 0.001;

    API::Devices::Inverse3::EndEffectorStateResponse state;
    Haply::HardwareAPI::Devices::Handle::VersegripStatusResponse data;

    Quat quat_tip;
    double gripper_pos = GRIPPER_CLOSED;
    bool gripper_reset = false;

    while (true) {
        next += delay;

        API::Devices::Inverse3::EndEffectorForceRequest request;
        state = inv_device.EndEffectorForce(request);

        data = handle.GetVersegripStatus();

        if(data.buttons == 1){
            break;
        }

        quat_tip[0] = data.quaternion[0];
        quat_tip[1] = data.quaternion[1];
        quat_tip[2] = data.quaternion[2];
        quat_tip[3] = data.quaternion[3];

        Vec3 rpy = robo::rotMatToRPY(robo::quatToRotMat(quat_tip));

        goal_posture.x = state.position[1] + 0.6;
        goal_posture.y = -state.position[0] + 0.0;
        goal_posture.z = state.position[2] + 0.2;

        goal_posture.rx = -rpy[1];
        goal_posture.ry = rpy[0];
        goal_posture.rz = rpy[2];

        joint_step(UNITREE_ARM::PosturetoVec6(goal_posture), &arm, 5.0);

        if(data.buttons == 2){
            if(gripper_reset){
                gripper_reset = false;

                if(gripper_pos == GRIPPER_OPEN){
                    gripper_pos = GRIPPER_CLOSED;
                }
                else{
                    gripper_pos = GRIPPER_OPEN;
                }

                arm.setGripperCmd(gripper_pos, 0.0);

            }
        }
        else{gripper_reset = true;}

        arm.sendRecv();

        while (next > clock::now())
            ;
    }

    arm.sendRecvThread->start();
    arm.setFsm(UNITREE_ARM::ArmFSMState::JOINTCTRL);
    arm.backToStart();
    arm.setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
    arm.sendRecvThread->shutdown();
    return 0;
}
