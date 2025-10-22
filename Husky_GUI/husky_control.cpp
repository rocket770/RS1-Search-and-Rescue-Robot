//#include "husky_control.h"
//#include "./ui_husky_control.h"
//#include <QImage>
//#include <QPixmap>

//Husky_Control::Husky_Control(QWidget *parent)
//    : QMainWindow(parent)
//    , ui(new Ui::Husky_Control)
//{
//    ui->setupUi(this);

//    node_ = rclcpp::Node::make_shared("husky_control_node");
//        vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

//        connect(ui->forButton, &QPushButton::pressed, this, &Husky_Control::on_forButton_pressed);
//        connect(ui->forButton, &QPushButton::released, this, &Husky_Control::on_forButton_released);

//        std::thread([this]() { rclcpp::spin(node_); }).detach();
//}

//Husky_Control::~Husky_Control()
//{
//    delete ui;
//}

//void Husky_Control::publish_velocity(double linear_x, double angular_z) {
//    twist_.linear.x = linear_x;
//    twist_.angular.z = angular_z;
//    vel_pub_->publish(twist_);
//}

//void Husky_Control::on_forButton_pressed() {
//    publish_velocity(linear_speed_, 0.0);
//}

//void Husky_Control::on_forButton_released() {
//    publish_velocity(0.0, 0.0);
//}

//void Husky_Control::on_stopButton_clicked() {
//    publish_velocity(0.0, 0.0);
//}

#include "husky_control.h"
#include "ui_husky_control.h"
#include <QImage>
#include <QPixmap>

Husky_Control::Husky_Control(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::Husky_Control) {
    ui->setupUi(this);

    node_ = rclcpp::Node::make_shared("husky_control_node");
    vel_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
            current_speed_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
        });
    image_sub_ = node_->create_subscription<sensor_msgs::msg::Image>(
        "/camera/image_raw", 10, std::bind(&Husky_Control::image_callback, this, std::placeholders::_1));

    connect(ui->forButton, &QPushButton::pressed, this, &Husky_Control::on_forButton_pressed);
    connect(ui->forButton, &QPushButton::released, this, &Husky_Control::on_forButton_released);
    connect(ui->backButton, &QPushButton::pressed, this, &Husky_Control::on_backButton_pressed);
    connect(ui->backButton, &QPushButton::released, this, &Husky_Control::on_backButton_released);
    connect(ui->leftButton, &QPushButton::pressed, this, &Husky_Control::on_leftButton_pressed);
    connect(ui->leftButton, &QPushButton::released, this, &Husky_Control::on_leftButton_released);
    connect(ui->rightButton, &QPushButton::pressed, this, &Husky_Control::on_rightButton_pressed);
    connect(ui->rightButton, &QPushButton::released, this, &Husky_Control::on_rightButton_released);

    timer_ = new QTimer(this);
    connect(timer_, &QTimer::timeout, this, &Husky_Control::update_display);
    timer_->start(100);

    std::thread([this]() { rclcpp::spin(node_); }).detach();
}

Husky_Control::~Husky_Control() {
    delete ui;    delete timer_;
}

void Husky_Control::publish_velocity(double linear_x, double angular_z) {
    twist_.linear.x = linear_x;
    twist_.angular.z = angular_z;
    vel_pub_->publish(twist_);
}

void Husky_Control::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat img = cv_ptr->image;
        QImage qimg(img.data, img.cols, img.rows, img.step, QImage::Format_RGB888);
        ui->cameraDisplay->setPixmap(QPixmap::fromImage(qimg.rgbSwapped()));
        ui->cameraDisplay->resize(qimg.size());
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "cv_bridge exception: %s", e.what());
    }
}

void Husky_Control::update_display() {
    ui->velocity->display(current_speed_);
}

void Husky_Control::on_forButton_pressed() { publish_velocity(linear_speed_, 0.0); }
void Husky_Control::on_forButton_released() { publish_velocity(0.0, 0.0); }
void Husky_Control::on_backButton_pressed() { publish_velocity(-linear_speed_, 0.0); }
void Husky_Control::on_backButton_released() { publish_velocity(0.0, 0.0); }
void Husky_Control::on_leftButton_pressed() { publish_velocity(0.0, angular_speed_); }
void Husky_Control::on_leftButton_released() { publish_velocity(0.0, 0.0); }
void Husky_Control::on_rightButton_pressed() { publish_velocity(0.0, -angular_speed_); }
void Husky_Control::on_rightButton_released() { publish_velocity(0.0, 0.0); }
