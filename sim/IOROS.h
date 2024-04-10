#ifndef IOROS_H
#define IOROS_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include "interface/IOInterface.h"
#include "message/MotorCmd.h"
#include "message/MotorState.h"
#include "message/IMU.h"
#include "message/Cartesian.h"

class IOROS : public IOInterface{
public:
    IOROS();
    ~IOROS();
    bool sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
private:
    ros::NodeHandle _nm;
    std::string _rname; // robot_name retrieved from ROS parameter server
    ros::Subscriber _servo_sub[7];
    ros::Publisher _servo_pub[7];
    ros::Subscriber _trunk_imu_sub;
    ros::Subscriber _robot_joint_states_sub;
    unitree_legged_msgs::MotorState _joint_state[7];
    unitree_legged_msgs::MotorCmd _joint_cmd[7];
    unitree_legged_msgs::IMU _trunk_imu;
    unitree_legged_msgs::Cartesian _robot_joint_state;
    tf::TransformListener _tf_listener;
    tf::TransformBroadcaster _tf_broadcaster;
    void _sendCmd(const LowlevelCmd *cmd);
    void _recvState(LowlevelState *state);
    void _initRecv();
    void _initSend();

    void _joint00Callback(const unitree_legged_msgs::MotorState& msg);
    void _joint01Callback(const unitree_legged_msgs::MotorState& msg);
    void _joint02Callback(const unitree_legged_msgs::MotorState& msg);
    void _joint03Callback(const unitree_legged_msgs::MotorState& msg);
    void _joint04Callback(const unitree_legged_msgs::MotorState& msg);
    void _joint05Callback(const unitree_legged_msgs::MotorState& msg);
    void _gripperCallback(const unitree_legged_msgs::MotorState& msg);
    void _trunkImuCallback(const unitree_legged_msgs::IMU& msg);
    void _robotJointStatesCallback(const unitree_legged_msgs::MotorState& msg);
};

#endif  // IOROS_H
