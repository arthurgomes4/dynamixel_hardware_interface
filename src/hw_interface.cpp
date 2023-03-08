#include <cmath>

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>

#include "dynamixel_sdk/dynamixel_sdk.h"

#define NUM_JOINTS            1
#define BAUDRATE              1000000
#define DEVICE_NAME           "/dev/ttyUSB0"
#define PROTOCOL_VERSION      2.0  

double position_to_radians(int position) {
  return (position / 2048.0 - 1.0) * M_PI;
}

int radians_to_position(double radians) {
  return round((radians / M_PI + 1.0) * 2048.0);
}

struct dynamixelServo
{
  uint8_t dynamixel_id = 1;
  uint8_t addr_torque_enable = 64;
  uint8_t addr_goal_position = 116;
  uint8_t addr_present_position = 132;
  uint8_t addr_operating_mode = 11;
  uint8_t PID[3] = {100,1,10};
};

class myRobot : public hardware_interface::RobotHW
{
    hardware_interface::JointStateInterface joint_state;
    hardware_interface::PositionJointInterface joint_position;

    double cmd[NUM_JOINTS] = {0};
    double pos[NUM_JOINTS] = {0};
    double vel[NUM_JOINTS] = {0};
    double eff[NUM_JOINTS] = {0};

    dynamixelServo servo;

    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    public:
    myRobot() 
    { 
        // connect and register the joint state interface
        hardware_interface::JointStateHandle state_handle_a("joint1", &pos[0], &vel[0], &eff[0]);
        joint_state.registerHandle(state_handle_a);

        registerInterface(&joint_state);

        // connect and register the joint position interface
        hardware_interface::JointHandle pos_handle_a(joint_state.getHandle("joint1"), &cmd[0]);
        joint_position.registerHandle(pos_handle_a);

        registerInterface(&joint_position);

        init();
    }

    ~myRobot()
    {
        //close the port
        portHandler->closePort();
        ROS_INFO_STREAM("port: " << DEVICE_NAME << " closed. Shutting down HW Interface");
    }

    void init(){
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

        // open the port
        portHandler = dynamixel::PortHandler::getPortHandler(DEVICE_NAME);
        packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

        if (!portHandler->openPort()) {
            ROS_ERROR("Failed to open the port!");
        }

        if (!portHandler->setBaudRate(BAUDRATE)) {
            ROS_ERROR("Failed to set the baudrate!");
        }

        //set in position mode
        std::array<std::pair<int, int>, 3> addressValues = {{{servo.addr_torque_enable, 0}, 
                                                             {servo.addr_operating_mode, 3}, 
                                                             {servo.addr_torque_enable, 1}}};
        for(auto &x : addressValues){
            dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, servo.dynamixel_id, x.first, x.second, &dxl_error);
            if (dxl_comm_result != COMM_SUCCESS) {
                ROS_ERROR("Failed to set Dynamixel ID %d in control mode %d", servo.dynamixel_id, servo.addr_operating_mode);
            }
        }

        ROS_INFO_STREAM("Node set up with device: " << DEVICE_NAME);
    }
    void read(){

        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;
        uint32_t position = 0;
        dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, 
                                                       servo.dynamixel_id, 
                                                       servo.addr_present_position, 
                                                       &position, 
                                                       &dxl_error);
        pos[0] = position_to_radians(position);

        if (dxl_comm_result == COMM_SUCCESS) {
            ROS_INFO("getPosition : [ID:%d] -> [POSITION:%f]", servo.dynamixel_id, pos[0]);
        } else {
            ROS_INFO("Failed to get position! Result: %d", dxl_comm_result);
        }
        return;
    }

    void write()
    {
        uint8_t dxl_error = 0;
        int dxl_comm_result = COMM_TX_FAIL;

       
        uint32_t position = radians_to_position(cmd[0]);

        dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, 
                                                        servo.dynamixel_id, 
                                                        servo.addr_goal_position, 
                                                        position, 
                                                        &dxl_error);
        if (dxl_comm_result == COMM_SUCCESS) {
            ROS_INFO("setPosition : [ID:%d] [POSITION:%f]",servo.dynamixel_id, cmd[0]);
        } else {
            ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
        }
    }

};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "hw_interface");
  ros::NodeHandle nh;

  myRobot robot;
  controller_manager::ControllerManager cm(&robot);
  
  ros::Time previousTime = ros::Time::now();
  ros::Time currentTime = ros::Time::now();
  
  ros::AsyncSpinner spinner(3);
  spinner.start();
  while (ros::ok())
  {
     robot.read();
     previousTime = currentTime;
     currentTime = ros::Time::now();
     cm.update(currentTime, currentTime - previousTime);
     robot.write();
     sleep(1);
  }

  return 0;
}