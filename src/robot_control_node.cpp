#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>

class RobotControlNode {
public:
    RobotControlNode()
    {
        ros::NodeHandle nh;

        sub_ = nh.subscribe("/line_sensors", 1, &RobotControlNode::callback, this);
        pub_ = nh.advertise<geometry_msgs::Twist>("/youbot_base/mecanum_drive_controller/cmd_vel", 1);

        left_sens_ = 2;
        right_sens_ = 2;
        base_speed_ = 0.08;
        turn_koeff_ = 0.25;
        direction_ = -1;

        ROS_INFO("RobotControlNode started");
    }

    void callback(const std_msgs::Int32MultiArray::ConstPtr& msg)
    {
        // Имитируем задержку time.sleep(0.01)
        ros::Duration(0.001).sleep();

        if (msg->data.size() < 2) {
            ROS_WARN("Invalid sensor array size!");
            return;
        }

        left_sens_  = msg->data[0];
        right_sens_ = msg->data[1];

        double angular_z = 0;
        double linear_x = 0;

        if (left_sens_ == 1 && right_sens_ == 1){
            angular_z = 0;
            linear_x = 1;
        }
        else if (left_sens_ == 0 && right_sens_ == 1){
            angular_z = 1;
            linear_x = 0;
        }
        else if (left_sens_ == 1 && right_sens_ == 0){
            angular_z = -1;
            linear_x = 0;
        }
        else {
            angular_z = 0;
            linear_x = 0;
        }

        // int error = left_sens_ - right_sens_;

        // double angular_z = -turn_koeff_ * error;
        // double linear_x  = base_speed_;

        geometry_msgs::Twist cmd;
        cmd.linear.x = direction_ * base_speed_ * linear_x;
        cmd.angular.z = direction_ * turn_koeff_ * angular_z;

        pub_.publish(cmd);

        ROS_INFO("L=%d R=%d angle=%.3f", left_sens_, right_sens_, angular_z);
    }

    int getLeft()  const { return left_sens_; }
    int getRight() const { return right_sens_; }

private:
    ros::Subscriber sub_;
    ros::Publisher  pub_;

    int left_sens_;
    int right_sens_;
    double base_speed_;
    double turn_koeff_;
    int direction_;
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_control_node");
    RobotControlNode node;

    ros::Rate rate(10);

    while (ros::ok()) {
        ROS_INFO("data: %d %d", node.getLeft(), node.getRight());
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}