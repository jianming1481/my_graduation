#include <DynamixelSerial.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <trainning_data_builder/data.h>

ros::NodeHandle  nh;
std_msgs::String str_msg;
int Pre_Position,Position; 
char m_Position[5];
String str;
using trainning_data_builder::data;
ros::ServiceClient<data::Request, data::Response> client("/trainning_data_builder/save");

void setup(){ // Configuration
  nh.initNode();
  nh.serviceClient(client);
  while(!nh.connected()) nh.spinOnce();
  nh.loginfo("Startup complete");
  Dynamixel.begin(1000000,2); //2=data control
  delay(1000);
  Dynamixel.setEndless(1,ON);
  delay(1000);
  Dynamixel.action();
  Pre_Position = Dynamixel.readPosition(1);
}

void loop(){
  Position = Dynamixel.readPosition(1);
  data::Request req;
  data::Response res;
  int tmp = Position-Pre_Position;
  if((tmp)>14)
  {
     Dynamixel.turn(1,LEFT,0);
     Pre_Position = Position;
     req.str = "trainning_data";
     client.call(req, res);
     while(!res.success)
     {
      delay(1);
      client.call(req, res);
     }
     if(res.success)
     {
      delay(1000);
     }
     delay(100);
  }
  if(tmp < 0)
  {
    Pre_Position = Position;
  }
  Dynamixel.turn(1,LEFT,100);
  delay(100);

  nh.spinOnce();
}
