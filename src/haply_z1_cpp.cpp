#include <chrono>
#include <cstdio>
#include <cmath>
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
    if (argc < 2) {
        std::fprintf(stderr, "usage: haply_z1_cpp [com-port]\n");
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

    API::IO::SerialStream stream{argv[1]};
    API::Devices::Inverse3 device{&stream};
    (void)device.DeviceWakeup();

    typedef std::chrono::high_resolution_clock clock;
    auto next = clock::now();
    auto delay = 1000us; // 1kHz

    arm._ctrlComp->dt = 0.001;

    API::Devices::Inverse3::EndEffectorStateResponse state;

    while (true) {
        next += delay;

        API::Devices::Inverse3::EndEffectorForceRequest request;

        state = device.EndEffectorForce(request);

        goal_posture.x = state.position[1] + 0.6;
        goal_posture.y = -state.position[0] - 0.1;
        goal_posture.z = state.position[2] + 0.2;

        joint_step(UNITREE_ARM::PosturetoVec6(goal_posture), &arm, 2.0);

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
