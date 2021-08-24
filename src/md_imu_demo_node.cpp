#include "md_imu/md_imu.h"

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "md_imu/rpm.h"

MD_IMU mdimu; //declare object

static int operating_mode=2;           //start mode = cmd_vel mode

void modeCallback(const std_msgs::Int8::ConstPtr& msg){

  if(operating_mode != msg->data){
    mdimu.Torque_OFF();
    //ROS_INFO("modecallback1");
  }
  operating_mode = msg->data;

  //ROS_INFO("modecallback2");

  if(operating_mode == 1){}
    //ROS_INFO("Operating Mode : RPM control mode");
  else if(operating_mode == 2){}
    //ROS_INFO("Operating Mode : CMD_VEL control mode");
  else if(operating_mode == 0){
    ROS_INFO("TQ_OFF!!");
    mdimu.Torque_OFF();
  }
 else if(operating_mode == 3){
    //ROS_INFO("Dymanic cmd_vel mode");
 }
else if(operating_mode == 4){
    //ROS_INFO("Dymanic cmd_vel mode");
 }
  else
    ROS_WARN("Invalid control mode number");
}

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg){

  float linear_x = msg->linear.x;
  float angular_z = msg->angular.z;
  float vel_arr[2] = {linear_x,angular_z};

  if(operating_mode == 2 || operating_mode==3 || operating_mode==4){
     mdimu.contol_vel(vel_arr);
    //ROS_INFO("Linear_x : %f angular_z : %f",linear_x,angular_z);
  }
}

void rpmCallback(const md_imu::rpm::ConstPtr& msg){

  if(operating_mode == 1){
    //ROS_INFO("rpmcallback1");
    int r_rpm = (msg->r_rpm)*-1;
    int l_rpm = msg->l_rpm;
    mdimu.send_RPM(r_rpm,l_rpm);
  }
  //ROS_INFO("rpmcallback2");

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mdimu");
  ros::NodeHandle nh;


  ros::Publisher present_rpm_pub = nh.advertise<md_imu::rpm>("/present_rpm", 1000);
  ros::Publisher led_status_pub = nh.advertise<std_msgs::UInt16>("/md_driver_status", 1000); // to use gui status led
  ros::Subscriber mode_sub = nh.subscribe("/mode", 1000, modeCallback);
  ros::Subscriber cmd_vel_sub = nh.subscribe("/cmd_vel", 1000, cmd_velCallback);
  ros::Subscriber rpm_sub = nh.subscribe("/rpm", 1000, rpmCallback);


  ros::Publisher angle_pub = nh.advertise<std_msgs::Float32>("/angle/x", 1000);
  ros::Publisher gyro_z_pub = nh.advertise<std_msgs::Float32>("/gyro/z", 1000);
  ros::Publisher angle_y_pub = nh.advertise<std_msgs::Float32>("/angle/y", 1000);
  ros::Publisher angle_z_pub = nh.advertise<std_msgs::Float32>("/angle/z", 1000);

  //ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu", 1000);

  ros::Rate loop_rate(100);

  mdimu.initialize_md_imu_driver();
  mdimu.Reset_ENC();

  //mdimu.set_sync_tx_cycle(5);  //10ms = 100hz
  //mdimu.save_params();
  //mdimu.software_reset();

  mdimu.set_sync_req(false);

  //send_RPM(100,100);

  struct Encoder_data enc_data;

  //md.Encoder_REQ();
  mdimu.imu_req();
  loop_rate.sleep();

  while (ros::ok())
  {
    mdimu.imu_read();
    //mdimu.read_sync_data();
    //enc_data = mdimu.read_Encoder();

    //ROS_INFO("R_posi : %d   L_posi : %d",enc_data.R_posi,enc_data.L_posi);

    md_imu::rpm msg;
    std_msgs::UInt16 status_msg;

    msg.r_rpm = r_rpm_g;
    msg.l_rpm = l_rpm_g;

    std_msgs::Float32 gyro_z_msg;
    std_msgs::Float32 angle_y_msg;
    std_msgs::Float32 angle_z_msg;

    gyro_z_msg.data = mdimu.gyro_z;
    angle_y_msg.data = mdimu.angle_y;
    angle_z_msg.data = mdimu.angle_z;

    status_msg.data = 1;

    //present_rpm_pub.publish(msg);
    led_status_pub.publish(status_msg);
    gyro_z_pub.publish(gyro_z_msg);
    angle_y_pub.publish(angle_y_msg);
    angle_z_pub.publish(angle_z_msg);

    ros::spinOnce();

    //mdimu.Encoder_REQ();
    mdimu.imu_req();
    loop_rate.sleep();
  }

  return 0;
}
