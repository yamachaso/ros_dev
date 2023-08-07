#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Joy.h>

class TwistPublisher{
public:
  TwistPublisher() : nh_(), pnh_("~") {
    cmd_pub_ = nh_.advertise<geometry_msgs::WrenchStamped>("/myrobot/left_cartesian_force_controller/target_wrench", 1);
    joy_sub_ = nh_.subscribe("joy", 10, &TwistPublisher::joyCallback, this);
    // timer_ = nh_.createTimer(ros::Duration(0.1), &TwistPublisher::timerCallback, this);
  }

  void joyCallback(const sensor_msgs::Joy& joy_msg) {
    // last_joy_ = joy_msg;

    int assign_x = 1; //axes
    int assign_y = 0; //axes
    int assign_z = 5; //axes
    int assign_yaw = 4; //buttons
    int assign_yaw_r = 5; //buttons


    float default_max = 30.0;
    float max_x = default_max;
    float max_y = default_max;
    float max_z = default_max;
    float max_yaw = default_max;

    geometry_msgs::WrenchStamped msg;
    msg.wrench.force.x = max_x * joy_msg.axes[assign_x];
    msg.wrench.force.y = max_y * joy_msg.axes[assign_y] * -1;
    msg.wrench.force.z = max_z * joy_msg.axes[assign_z] * -1;

    float kaiten = joy_msg.buttons[assign_yaw_r] - joy_msg.buttons[assign_yaw];
    msg.wrench.torque.z = max_z * kaiten;

    cmd_pub_.publish(msg);

  }


  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher cmd_pub_;
  ros::Subscriber joy_sub_;

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "basic_twist_publisher");
  TwistPublisher twist_publisher;
  ros::spin();
  return 0;
}
