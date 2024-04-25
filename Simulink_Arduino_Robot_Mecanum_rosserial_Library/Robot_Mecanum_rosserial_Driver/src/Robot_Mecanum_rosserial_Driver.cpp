//Robot_Mecanum_rosserial_Driver.cpp
#include <Arduino.h>
#include "Robot_Mecanum_rosserial_Driver.h"
#include "ros.h"
#include <std_msgs/Float32MultiArray.h>

void arrayCallback(const std_msgs::Float32MultiArray& array_msg);

ros::NodeHandle nh;
bool _new_data;
float _ref_w1, _ref_w2, _ref_w3, _ref_w4;
ros::Subscriber<std_msgs::Float32MultiArray> sub("robot_references", &arrayCallback);
float sens_data[10];
std_msgs::Float32MultiArray output_array_msg;
ros::Publisher output_array_pub("robot_sensors", &output_array_msg);

void arrayCallback(const std_msgs::Float32MultiArray& array_msg) {
  _new_data = true;
  _ref_w1 = array_msg.data[0];
  _ref_w2 = array_msg.data[1];
  _ref_w3 = array_msg.data[2];
  _ref_w4 = array_msg.data[3];
}

extern "C" void Robot_Mecanum_rosserial_Driver_Init()
{ 
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(output_array_pub);
  output_array_msg.data = sens_data;
} 

extern "C" void Robot_Mecanum_rosserial_Driver_Step(float* ref_w1, float* ref_w2, float* ref_w3, float* ref_w4, float a_x, float a_y, float w_z, float theta, float w1, float w2, float w3, float w4, float V, float I)
{ 
  // Verifica si hay nuevos datos en el mensaje de entrada
  if (_new_data) {
    *ref_w1 = _ref_w1;
    *ref_w2 = _ref_w2;
    *ref_w3 = _ref_w3;
    *ref_w4 = _ref_w4;
  }

  // Simplemente coloca algunos datos en el array de salida como ejemplo
  output_array_msg.data_length = 10;
  output_array_msg.data[0] = a_x;
  output_array_msg.data[1] = a_y;
  output_array_msg.data[2] = w_z;
  output_array_msg.data[3] = theta;
  output_array_msg.data[4] = w1;
  output_array_msg.data[5] = w2;
  output_array_msg.data[6] = w3;
  output_array_msg.data[7] = w4;
  output_array_msg.data[8] = V;
  output_array_msg.data[9] = I;
  
  output_array_pub.publish(&output_array_msg);
  
  nh.spinOnce();
} 

extern "C" void Robot_Mecanum_rosserial_Driver_Terminate() 
{
    
} 