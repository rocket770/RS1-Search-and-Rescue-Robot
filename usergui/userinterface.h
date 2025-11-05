#ifndef USERINTERFACE_H
#define USERINTERFACE_H

#include <QMainWindow>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <QTimer> // check for ros messages

#include <QPixmap>

QT_BEGIN_NAMESPACE
namespace Ui { class userinterface; }
QT_END_NAMESPACE

class userinterface : public QMainWindow
{
    Q_OBJECT

public:
    userinterface(QWidget *parent = nullptr);
    ~userinterface();

    // These are auto generated when "go to slot" of a specific GUI add on
private slots:
    void on_leftButton_clicked();
    void on_forButton_clicked();
    void on_rightButton_clicked();
    void on_backButton_clicked();
    void on_stopButton_clicked();

    // To change between day and night
    void on_dayShift_clicked();

    // To show battery level
     void on_batteryLevel_valueChanged(int value);

    // To change between manual and autonomous
    void on_moveShift_clicked();

    // change the speed of robot
    void on_speedChange_valueChanged(double arg1);

    void rosmsgs() {
        rclcpp::spin_some(node_); // ros processes messages without blocking GUI event loop
    }

    void on_cameraSelection_currentIndexChanged(int index);

private:

    // For subscribing and publishing
    // https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html
    // https://robotisim.com/ros2-cpp-node-for-begginers/

    Ui::userinterface *ui;
    rclcpp::Node::SharedPtr node_; // shared pointer to ROS2 node object so I can interact with ROS2 and ensure to manage its memory
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subToImage;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity;
    rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr subtoBattery;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr resume_explore_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr toggle_time_client_;

    QTimer *ros_timer_;

    double linear_speed_ = 0.5;  // m/s
    double angular_speed_ = 1.0; // rad/s

    void obtainImage(const sensor_msgs::msg::Image::SharedPtr msg);
    void publishvelocity(double linear_x, double angular_z);
    void obtainBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg);
    void subToImageTopic(const std::string &topic_name);
    std::string current_image_topic_;

};
#endif // USERINTERFACE_H
