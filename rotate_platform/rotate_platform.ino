/*
 *  Control Rotate Platfom
 */

#include <DynamixelSerial.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <trainning_data_builder/data.h>

// Global variable
int Pre_Position,Position; 
char m_Position[5];

// ROS
ros::NodeHandle  nh;
using trainning_data_builder::data;

// ROS service callback function
void callback(const data::Request &req, data::Response &res)
{
  // In AX12, 5 degree equals to 14 plus
  // If Ax12 rotate more than 5 degree, AX12 STOP
  int tmp = 0;
  while(tmp < 14)
  {
    Dynamixel.turn(1,LEFT,100);
    // Read position from AX12
    Position = Dynamixel.readPosition(1);
    tmp = Position-Pre_Position;
    
    // Prevent degree over from 360 to 0
    if(tmp < 0)
    {
      Pre_Position = Position;
    }
    res.success = false;
  }
  
  Pre_Position = Position;
  // AX12 Stop rotate
  Dynamixel.turn(1,LEFT,0);
  res.success = true;
}

// Declare the server
ros::ServiceServer<data::Request, data::Response> server("/trainning_data_builder/save",&callback);

void setup(){ // Configuration
  // Initial ROS node
  nh.initNode();
  nh.advertiseService(server);

  // Initial AX12
  Dynamixel.begin(1000000,2); //2=data control
  delay(1000);
  Dynamixel.setEndless(1,ON);
  delay(1000);
  Dynamixel.action();
  Pre_Position = Dynamixel.readPosition(1);
}

void loop(){
  nh.spinOnce();
  delay(10);
}
