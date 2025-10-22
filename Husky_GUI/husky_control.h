#ifndef HUSKY_CONTROL_H
#define HUSKY_CONTROL_H

#include <QMainWindow>
#include <QTimer>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

namespace Ui { class Husky_Control; }

class Husky_Control : public QMainWindow {
    Q_OBJECT

public:
    Husky_Control(QWidget *parent = nullptr);
    ~Husky_Control();

private slots:
    void on_forButton_pressed();
    void on_forButton_released();
    void on_backButton_pressed();
    void on_backButton_released();
    void on_leftButton_pressed();
    void on_leftButton_released();
    void on_rightButton_pressed();
    void on_rightButton_released();
    void update_display();

private:
    Ui::Husky_Control *ui;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    geometry_msgs::msg::Twist twist_;
    double linear_speed_ = 0.5;
    double angular_speed_ = 1.0;
    double current_speed_ = 0.0;
    QTimer* timer_;
    void publish_velocity(double linear_x, double angular_z);
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
};
#endif // HUSKY_CONTROL_H
