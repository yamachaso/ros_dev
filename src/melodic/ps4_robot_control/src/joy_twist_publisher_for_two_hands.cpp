#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>

using namespace std;

class TwistPublisher{
public:
  TwistPublisher()
   : nh_(), pnh_("~"), last_msg(sensor_msgs::Joy()), default_max(0.04),
     is_right(true), max_pressure(0.0), min_pressure(0.0), 
     max_pressure_high_limit(1.5), max_pressure_low_limit(-0.3),
     pitch(0), roll(0)
  {
    cmd_pub_ = {
      nh_.advertise<geometry_msgs::Twist>("/cmd_vel/right", 1), 
      nh_.advertise<geometry_msgs::Twist>("/cmd_vel/left", 1)
    };
    now_cmd_pub_ = &cmd_pub_[0];
    hand_pressure_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/hand_ref_pressure", 1);
    joy_sub_ = nh_.subscribe("joy", 10, &TwistPublisher::joyCallback, this);

  }



  void joyCallback(const sensor_msgs::Joy& joy_msg) {
    // last_joy_ = joy_msg;

    int assign_x = 1; //axes, 左joyの左右
    int assign_y = 0; //axes, 左joyの上下
    int assign_z = 5; //axes, 右joyの上下
    int assign_yaw = 4; //buttons, L1
    int assign_yaw_r = 5; //buttons, R1

    int assign_grab = 2; //buttons, 丸
    int assign_release = 0; //buttons, 四角

    int max_increment = 3; // buttons, 三角
    int max_decrement = 1; // buttons, バツ

    int switch_left = 6; // buttons, L2
    int switch_right = 7; // buttons, R2


    float max_x = default_max;
    float max_y = default_max;
    float max_z = default_max;
    float max_yaw = default_max * 10;

    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = max_x * joy_msg.axes[assign_x];
    cmd_vel.linear.y = max_y * joy_msg.axes[assign_y];
    cmd_vel.linear.z = max_z * joy_msg.axes[assign_z];

    float kaiten = joy_msg.buttons[assign_yaw] - joy_msg.buttons[assign_yaw_r];
    cmd_vel.angular.z = max_yaw * kaiten;

    now_cmd_pub_->publish(cmd_vel);

    if(joy_msg.buttons[assign_grab] > 0) {
      max_pressure = 0.8;
      std_msgs::Float64MultiArray hand_ref_pressure;
      hand_ref_pressure.data.assign(2, min_pressure);
      if(is_right) {
        hand_ref_pressure.data[1] = max_pressure;
      } else {
        hand_ref_pressure.data[0] = max_pressure;
      }
      hand_pressure_pub_.publish(hand_ref_pressure);
    }
    if(joy_msg.buttons[assign_release] > 0) {
      max_pressure = 0.0;
      std_msgs::Float64MultiArray hand_ref_pressure;
      hand_ref_pressure.data.assign(2, min_pressure);
      hand_pressure_pub_.publish(hand_ref_pressure);
    }
    if(joy_msg.buttons[max_increment] > 0 and last_msg.buttons[max_increment] == 0) {
      max_pressure += 0.1;
      max_pressure = min(max_pressure, max_pressure_high_limit);
      std_msgs::Float64MultiArray hand_ref_pressure;
      hand_ref_pressure.data.assign(2, min_pressure);
      if(is_right) {
        hand_ref_pressure.data[1] = max_pressure;
      } else {
        hand_ref_pressure.data[0] = max_pressure;
      }
      hand_pressure_pub_.publish(hand_ref_pressure);
    }
    if(joy_msg.buttons[max_decrement] > 0 and last_msg.buttons[max_decrement] == 0) {
      max_pressure -= 0.1;
      max_pressure = max(max_pressure, max_pressure_low_limit);
      std_msgs::Float64MultiArray hand_ref_pressure;
      hand_ref_pressure.data.assign(2, min_pressure);
      if(is_right) {
        hand_ref_pressure.data[1] = max_pressure;
      } else {
        hand_ref_pressure.data[0] = max_pressure;
      }
      hand_pressure_pub_.publish(hand_ref_pressure);
    }
    if(joy_msg.buttons[switch_right] > 0 and last_msg.buttons[switch_right] == 0) {
      max_pressure = 0.0;
      now_cmd_pub_ = &cmd_pub_[0];
      is_right = true;
    }
    if(joy_msg.buttons[switch_left] > 0 and last_msg.buttons[switch_left] == 0) {
      max_pressure = 0.0;
      now_cmd_pub_ = &cmd_pub_[1];
      is_right = false;
    }

    

    last_msg = joy_msg;
  }


  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  vector<ros::Publisher> cmd_pub_;
  ros::Publisher* now_cmd_pub_;
  ros::Publisher hand_pressure_pub_;
  ros::Subscriber joy_sub_;

  sensor_msgs::Joy last_msg;

  float default_max;
  bool is_right;
  float max_pressure;
  float min_pressure;
  float max_pressure_high_limit;
  float max_pressure_low_limit;
  int pitch;
  int roll;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_twist_publisher");
  TwistPublisher twist_publisher;
  ros::spin();
  return 0;
}
