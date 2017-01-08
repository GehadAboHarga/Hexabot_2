#include <ros.h>
#include <ros/time.h>
#include <sensor_msgs/LaserScan.h>
#define USE_USBCON

ros::NodeHandle  nh;
sensor_msgs::LaserScan range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);

String command_ultra, command_encoders;
int ultraL1, ultraL2, ultraR1, ultraR2, ultraF, ultraB;
float ultra_all[6];
long encoderL, encoderR;
String part1_ultra, part2_ultra, part3_ultra, part4_ultra, part5_ultra, part6_ultra;
String part1_encoders, part2_encoders;
char c_ultra, c_encoders;
unsigned long previousmills = 0, currentmills = 0;
const long interval = 200;


/*-------------------Functions init---------------------*/
int count = 0;
void r_ultra();
void r_encoders();
void parse_ultra(String command_ultra1);
void parse_encoders(String command_encoders1);
void t_data1();
void from_dxdy_to_xytheta(long pos_l, long pos_r);
long range_time;
char frameid[] = "/ultrasound";


void setup() {
  Serial3.begin(115200);
  Serial2.begin(115200);
  Serial1.begin(115200);
  nh.initNode();
  nh.advertise(pub_range);
  range_msg.angle_min = -3.009;
  range_msg.angle_max = 3.18;
  range_msg.angle_increment = 1.04;
  range_msg.header.frame_id =  frameid;
  range_msg.range_min = 0.0;
  range_msg.range_max = 2.5;
  range_msg.ranges_length = 6;
}

void loop()
{
  r_ultra();
  if ( millis() >= range_time ){
      int r =0;
      range_msg.ranges = ultra_all;
      range_msg.header.stamp = nh.now();
      pub_range.publish(&range_msg);
      range_time =  millis() + 50;
    }    
    nh.spinOnce();
}


//receiving from ultrasonics
void r_ultra(){
  if (Serial3.available()) {
    c_ultra = Serial3.read();

    if (c_ultra == '\n') {
      parse_ultra(command_ultra);
      command_ultra = "";
    }
    else {
      command_ultra += c_ultra;
    }
  }
}



//receiving from encoders
void r_encoders(){
  if (Serial2.available()) {
    c_encoders = Serial2.read();

    if (c_encoders == '\n') {
      parse_encoders(command_encoders);
      command_encoders = "";
    }
    else {
      command_encoders += c_encoders;
    }
  }
}



//parse Ultrasonic data
void parse_ultra(String command_ultra1){
  part1_ultra = command_ultra1.substring(0, command_ultra1.indexOf("\t"));
  command_ultra1.remove(0, command_ultra1.indexOf("\t")+1);
  part2_ultra = command_ultra1.substring(0,command_ultra1.indexOf("\t"));
  command_ultra1.remove(0, command_ultra1.indexOf("\t")+1);
  part3_ultra = command_ultra1.substring(0,command_ultra1.indexOf("\t"));
  command_ultra1.remove(0, command_ultra1.indexOf("\t")+1);
  part4_ultra = command_ultra1.substring(0,command_ultra1.indexOf("\t"));
  command_ultra1.remove(0, command_ultra1.indexOf("\t")+1);
  part5_ultra = command_ultra1.substring(0, command_ultra1.indexOf("\t"));
  command_ultra1.remove(0, command_ultra1.indexOf("\t")+1);
  part6_ultra = command_ultra1.substring(0,command_ultra1.indexOf("\n"));
  
  ultraL1 = part1_ultra.toFloat();
  ultraL2 = part2_ultra.toFloat();
  ultraR1 = part3_ultra.toFloat();
  ultraR2 = part4_ultra.toFloat();
  ultraF = part5_ultra.toFloat();
  ultraB = part6_ultra.toFloat();
  ultra_all[0]=ultraL1;
  ultra_all[1]=ultraL2;
  ultra_all[2]=ultraR1;
  ultra_all[3]=ultraR2;
  ultra_all[4]=ultraF;
  ultra_all[5]=ultraB;
  
}



//parse GPS data
void parse_encoders(String command_encoders1){
  part1_encoders = command_encoders1.substring(0, command_encoders1.indexOf("\t"));
  command_encoders1.remove(0, command_encoders1.indexOf("\t")+1);
  part2_encoders = command_encoders1.substring(0,command_encoders1.indexOf("\n"));
  
  encoderL = part1_encoders.toFloat();
  encoderR = part2_encoders.toFloat();
}
