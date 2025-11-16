#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <serial/serial.h>
#include <regex>
#include <string>

class LineSensorNode {
public:
    LineSensorNode() {
        ros::NodeHandle nh;

        pub_ = nh.advertise<std_msgs::Int32MultiArray>("/line_sensors", 1);

        try {
            ser_.setPort("/dev/ttyACM0");
            ser_.setBaudrate(9600);
            serial::Timeout to = serial::Timeout::simpleTimeout(1000);
            ser_.setTimeout(to);
            ser_.open();
        }
        catch (serial::IOException& e) {
            ROS_ERROR("Unable to open serial port!");
        }

        if (ser_.isOpen()) {
            ROS_INFO("Serial port initialized");
        } else {
            ROS_ERROR("Serial port not opened!");
        }
    }

    void readSerial() {
        if (!ser_.isOpen()) return;

        std::string line = ser_.readline(256, "\n");

        // Убираем переносы строк
        line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
        if (line.empty())
            return;

        try {
            std::regex pattern("first=(\\d), second=(\\d)");
            std::smatch match;

            if (std::regex_search(line, match, pattern)) {
                int left  = std::stoi(match[1]);
                int right = std::stoi(match[2]);

                std_msgs::Int32MultiArray msg;
                msg.data.push_back(left);
                msg.data.push_back(right);

                pub_.publish(msg);
            }
            else {
                ROS_WARN_STREAM("Invalid data: " << line);
            }
        }
        catch (...) {
            ROS_WARN_STREAM("Error parsing line: " << line);
        }
    }

private:
    ros::Publisher pub_;
    serial::Serial ser_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "line_sensor_node");
    LineSensorNode node;

    ros::Rate rate(100); // 10 Hz

    while (ros::ok()) {
        node.readSerial();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}