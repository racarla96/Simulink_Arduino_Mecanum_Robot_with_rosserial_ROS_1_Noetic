#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

float sens_data[10];
float refs_data[4];

ros::NodeHandle nh;
std_msgs::Float32MultiArray output_array_msg;

void arrayCallback(const std_msgs::Float32MultiArray& array_msg) {
  // Aquí puedes manejar los datos recibidos en el array_msg
  // Por ejemplo, imprimir los elementos del array
  for (int i = 0; i < array_msg.data_length; i++) {
    output_array_msg.data[i] = array_msg.data[i];
  }
}

ros::Subscriber<std_msgs::Float32MultiArray> sub("robot_references", &arrayCallback);
ros::Publisher output_array_pub("robot_sensors", &output_array_msg);

void setup() {
  nh.getHardware()->setBaud(115200);
   nh.initNode();
  nh.subscribe(sub);
  nh.advertise(output_array_pub);

  output_array_msg.data = sens_data;
}

void loop() {
  // Simplemente coloca algunos datos en el array de salida como ejemplo
  output_array_msg.data_length = 10;
//  output_array_msg.data[0] = 0.0;
//  output_array_msg.data[1] = 1.1;
//  output_array_msg.data[2] = 2.2;
//  output_array_msg.data[3] = 3.3;
//  output_array_msg.data[4] = 4.4;
//  output_array_msg.data[5] = 5.5;
//  output_array_msg.data[6] = 6.6;
//  output_array_msg.data[7] = 7.7;
//  output_array_msg.data[8] = 8.8;
//  output_array_msg.data[9] = 9.9;

  output_array_pub.publish(&output_array_msg);
  
  nh.spinOnce();
  delay(10); // Ajusta el retardo según sea necesario
}
