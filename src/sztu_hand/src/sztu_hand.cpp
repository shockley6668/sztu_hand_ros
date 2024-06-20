#include <ros/ros.h>
#include "std_msgs/String.h"
#include "sensor_msgs/JointState.h"
#include <vector>
#include "dynamixel_sdk/dynamixel_sdk.h"
using namespace std;
using namespace dynamixel;

// Control table address
#define ADDR_TORQUE_ENABLE    64
#define ADDR_PRESENT_POSITION 132
#define ADDR_GOAL_POSITION    116

// Protocol version
#define PROTOCOL_VERSION      2.0             // Default Protocol version of DYNAMIXEL X series.

// Default setting
#define DXL1_ID               1               // DXL1 ID
#define DXL2_ID               2               // DXL2 ID
#define BAUDRATE              57600           // Default Baudrate of DYNAMIXEL X series
#define DEVICE_NAME           "/dev/ttyUSB0"  // [Linux] To find assigned port, use "$ ls /dev/ttyUSB*" command

PortHandler * portHandler = PortHandler::getPortHandler(DEVICE_NAME);
PacketHandler * packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, 4);
GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4);

void transf(uint32_t pos,uint8_t* param_goal_position)
{
  param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(pos));
  param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(pos));
  param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(pos));
  param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(pos));
}
void jointstatesCallback(const sensor_msgs::JointStateConstPtr& msg)
{
  vector<uint32_t> pos(msg->position.size());
  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;
  int dxl_addparam_result = false;
  vector<uint8_t[4]> param_goal_position(4);

 // pos=msg.position;
  pos[0]=(unsigned int)(2000+(1000*msg->position[5]*180/3.1415926)/90);
  pos[1]=(unsigned int)(2000+(1000*msg->position[4]*180/3.1415926)/90);
  pos[2]=(unsigned int)(2000+(1000*msg->position[6]*180/3.1415926)/90);
  pos[3]=(unsigned int)(2000+(1000*msg->position[7]*180/3.1415926)/90);
  for(int i=0;i<4;i++)
  {
    transf(pos[i],param_goal_position[i]);
  }
  for(int i=0;i<4;i++)
  {
    dxl_addparam_result = groupSyncWrite.addParam((uint8_t)i+1, param_goal_position[i]);
    if (dxl_addparam_result != true) {
      ROS_ERROR( "Failed to addparam to groupSyncWrite for Dynamixel ID %d", i+1);
    }
  }
  dxl_comm_result = groupSyncWrite.txPacket();
  if (dxl_comm_result == COMM_SUCCESS) {
    for(int i=0;i<4;i++)
      ROS_INFO("setPosition : [ID:%d] [POSITION:%d]", i+1, pos[i]);
    
  } else {
    ROS_ERROR("Failed to set position! Result: %d", dxl_comm_result);
  }
  groupSyncWrite.clearParam();
  // ROS_INFO("1:[%f] 2:[%f] 3:[%f] 4:[%f] ",pos[0],pos[1],pos[2],pos[4]);
 
}

int main(int argc, char **argv)
{
 

  uint8_t dxl_error = 0;
  int dxl_comm_result = COMM_TX_FAIL;

  if (!portHandler->openPort()) {
    ROS_ERROR("Failed to open the port!");
    return -1;
  }

  if (!portHandler->setBaudRate(BAUDRATE)) {
    ROS_ERROR("Failed to set the baudrate!");
    return -1;
  }
  for(uint8_t i=1;i<5;i++)
  {
    dxl_comm_result = packetHandler->write1ByteTxRx(
    portHandler, i, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
      ROS_ERROR("Failed to enable torque for Dynamixel ID %d", i);
      return -1;
    }
  }

  ros::init(argc, argv, "sztu_hand_node");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/joint_states", 1000, jointstatesCallback);
  ros::spin();
  portHandler->closePort();
  return 0;
}