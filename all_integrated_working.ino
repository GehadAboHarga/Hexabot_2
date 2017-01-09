int hexabot =0;
String command_ultra, command_encoders;
int ultraL1, ultraL2, ultraR1, ultraR2, ultraF, ultraB;
long encoderL, encoderR;
String part1_ultra, part2_ultra, part3_ultra, part4_ultra, part5_ultra, part6_ultra;
String part1_encoders, part2_encoders;
char c_ultra, c_encoders;
unsigned long previousmills = 0, currentmills = 0;
const long interval = 200;
float ultra_all[6];
/*-----------------ROS Odometry------------------------*/

#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#define USE_USBCON
#include <ArduinoHardware.h>
#include <geometry_msgs/Twist.h>


ros::NodeHandle  nh;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

sensor_msgs::LaserScan range_msg;
ros::Publisher pub_range( "/ultrasound", &range_msg);


char base_link[] = "/base_link";
char odom[] = "/odom";
double theta=0, x=0, y=0;


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

/*---------------------Main code------------------------*/
// Ultrasonics Nano on Serial 3
// Encoders Nano on Serial 2
// Raspberry pi on Serial 1

#define EN_L 9
#define IN1_L 25
#define IN2_L 24

#define EN_R 8
#define IN1_R 22
#define IN2_R 23

double omega_r=0, omega_l=0;
double wheel_rad = 6.5/200.0, wheel_sep = 29.5/100.0;


int lowSpeed = 220;
int highSpeed = 30;
double speed_ang=0, speed_lin=0;


void messageCb( const geometry_msgs::Twist& msg){
   speed_ang = msg.angular.z;
   speed_lin = msg.linear.x;
   omega_r = (speed_lin/wheel_rad) + ((speed_ang*wheel_sep)/(2.0*wheel_rad));
   omega_l = (speed_lin/wheel_rad) - ((speed_ang*wheel_sep)/(2.0*wheel_rad));
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &messageCb );

void Motors_init();
void MotorL(int Pulse_Width1);
void MotorR(int Pulse_Width2);


void setup(){
  nh.initNode();
   Motors_init();
    nh.subscribe(sub);
  broadcaster.init(nh);
  Serial3.begin(115200);
  Serial2.begin(115200);
  Serial1.begin(115200);
  nh.advertise(pub_range);
  range_msg.angle_min = -3.009;
  range_msg.angle_max = 3.18;
  range_msg.angle_increment = 1.04;
  range_msg.header.frame_id =  frameid;
  range_msg.range_min = 0.0;
  range_msg.range_max = 2.5;
  range_msg.ranges_length = 6;
  
}


void loop() {
  r_ultra();
  r_encoders();
   if ( millis() >= range_time ){
      int r =0;
      range_msg.ranges = ultra_all;
      range_msg.header.stamp = nh.now();
      pub_range.publish(&range_msg);
      range_time =  millis() + 50;
      
    }    
    
  from_dxdy_to_xytheta(encoderL, encoderR);
  if(theta > 3.14) theta=-3.14;
  if(theta < -3.14) theta=+3.14;
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  
  t.transform.rotation = tf::createQuaternionFromYaw(theta);
  t.header.stamp = nh.now();

  if (hexabot > 100){
  broadcaster.sendTransform(t);   
  hexabot = 0;
    }
    hexabot = hexabot +1;
 
  nh.spinOnce();
  
  /*
  r_ultra();
  r_encoders();
  */
  MotorL(omega_l*10);
  MotorR(omega_r*10);
  nh.spinOnce();
}


void Motors_init(){
  pinMode(EN_L, OUTPUT);
  pinMode(EN_R, OUTPUT);
  pinMode(IN1_L, OUTPUT);
  pinMode(IN2_L, OUTPUT);
  pinMode(IN1_R, OUTPUT);
  pinMode(IN2_R, OUTPUT);
  digitalWrite(EN_L, LOW);
  digitalWrite(EN_R, LOW);
  digitalWrite(IN1_L, LOW);
  digitalWrite(IN2_L, LOW);
  digitalWrite(IN1_R, LOW);
  digitalWrite(IN2_R, LOW);
}


void MotorL(int Pulse_Width1){
  
  if (Pulse_Width1 > 0){
      analogWrite(EN_L, Pulse_Width1);
      digitalWrite(IN1_L, HIGH);
      digitalWrite(IN2_L, LOW);
  }
  if (Pulse_Width1 < 0){
      Pulse_Width1=abs(Pulse_Width1);
      analogWrite(EN_L, Pulse_Width1);
      digitalWrite(IN1_L, LOW);
      digitalWrite(IN2_L, HIGH);
  }
  if (Pulse_Width1 == 0){
      analogWrite(EN_L, Pulse_Width1);
      digitalWrite(IN1_L, LOW);
      digitalWrite(IN2_L, LOW);
  }
}

void MotorR(int Pulse_Width2){
  
  if (Pulse_Width2 > 0){
      analogWrite(EN_R, Pulse_Width2);
      digitalWrite(IN1_R, LOW);
      digitalWrite(IN2_R, HIGH);
  }
  if (Pulse_Width2 < 0){
      Pulse_Width2=abs(Pulse_Width2);
      analogWrite(EN_R, Pulse_Width2);
      digitalWrite(IN1_R, HIGH);
      digitalWrite(IN2_R, LOW);
  }
  if (Pulse_Width2 == 0){
      analogWrite(EN_R, Pulse_Width2);
      digitalWrite(IN1_R, LOW);
      digitalWrite(IN2_R, LOW);
  }
}
/*------------------------------------Functions---------------------------------*/
// cmd_vel






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

void t_data(){
  
  Serial.print(ultraL1);        //1
  Serial.print("\t");
  
  Serial.print(ultraL2);         //2
  Serial.print("\t");
  
  Serial.print(ultraR1);        //3
  Serial.print("\t");
  
  Serial.print(ultraR2);        //4
  Serial.print("\t");
  
  Serial.print(ultraF);        //5
  Serial.print("\t");
  
  Serial.print(ultraB);        //6
  Serial.print("\t");
  
  Serial.print(encoderL);        //7
  Serial.print("\t");
  
  Serial.print(encoderR);        //8
  Serial.println();
}

void from_dxdy_to_xytheta(long pos_l, long pos_r){
  theta = (1/0.29)*(pos_r-pos_l)*0.01;
  x = 0.5*(pos_l+pos_r)*cos(theta)*(1/10.0);
  y = 0.5*(pos_l+pos_r)*sin(theta)*(1/10.0);
  }

