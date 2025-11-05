#include "userinterface.h"
#include "./ui_userinterface.h"

#include "ros_image_to_qimage/ros_image_to_qimage.hpp"

userinterface::userinterface(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::userinterface)
{
    ui->setupUi(this);

    // the ros node
    node_ = std::make_shared<rclcpp::Node>("usergui_node");

    //current_image_topic_ = "/camera/image";
    //subToImageTopic(current_image_topic_);

    ui->cameraSelection->addItem(QString::fromStdString("/camera/image"));
    ui->cameraSelection->addItem(QString::fromStdString("/yolo_detector/detections/image"));
    ui->cameraSelection->addItem(QString::fromStdString("/night_vision/image"));

    // subscribing to the camera topic
    //subToImage = node_->create_subscription<sensor_msgs::msg::Image>(
    //    "/camera/image", rclcpp::SensorDataQoS(), std::bind(&userinterface::obtainImage, this, std::placeholders::_1));

    current_image_topic_ = "/camera/image";  // default topic
    subToImageTopic(current_image_topic_);


    // publish movement to robot
    velocity = node_->create_publisher<geometry_msgs::msg::Twist> (
        "/ui/move", 10);

    subtoBattery = node_->create_subscription<sensor_msgs::msg::BatteryState>(
        "/battery_state", rclcpp::SensorDataQoS(), std::bind(&userinterface::obtainBattery, this, std::placeholders::_1));
    
    resume_explore_client_ =
        node_->create_client<std_srvs::srv::Trigger>("/user/resume_explore");
    
    toggle_time_client_ =
        node_->create_client<std_srvs::srv::Trigger>("/toggle_time");

        
    // Timer to process ROS messages
    // since qt has its own event loop to avoid blocking use rclcpp instead of ros::spin
    // https://www.youtube.com/watch?v=Cg1DaNFnZyY
    // use the address instead of SLOT and SIGNAL
    ros_timer_ = new QTimer(this);
    connect(ros_timer_, &QTimer::timeout, this, &userinterface::rosmsgs);
    
    ros_timer_->start(50);  // check for messages every 50ms and to avoid too much CPU load


}

userinterface::~userinterface() {
    delete ui;
}

// Sending velocity
// https://stackoverflow.com/questions/43515772/subscribing-and-publishing-geometry-twist-messages-from-turtlesim
void userinterface::publishvelocity(double linear_x, double angular_z)
{
    auto msg = geometry_msgs::msg::Twist();
    msg.linear.x = linear_x;
    msg.angular.z = angular_z;
    velocity->publish(msg);
}

// Main buttons for movement
// Single input, no timer
// Otherwise separate time must be used
void userinterface::on_forButton_clicked() {
    publishvelocity(linear_speed_, 0.0);
}

void userinterface::on_backButton_clicked() {
    publishvelocity(-linear_speed_, 0.0);
}

void userinterface::on_leftButton_clicked() {
    publishvelocity(0.0, angular_speed_);
}

void userinterface::on_rightButton_clicked() {
    publishvelocity(0.0, -angular_speed_);
}

void userinterface::on_stopButton_clicked() {
    publishvelocity(0.0, 0.0);
}


// To change the environment within the simulation
void userinterface::on_dayShift_clicked()
{

    //  brief debounce so a double click dont spam requests
    ui->dayShift->setEnabled(false);

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();

    (void)toggle_time_client_->async_send_request(req);

    // the debounce, re enable after 250ms
    QTimer::singleShot(250, this, [this]{
        ui->dayShift->setEnabled(true);
    });
}

// Change from Autonomy to Manual movement
void userinterface::on_moveShift_clicked()
{
    //RCLCPP_INFO(node_->get_logger(), "Hit!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!.");

    //  brief debounce so a double click dont spam requests
    ui->moveShift->setEnabled(false);

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();

    (void)resume_explore_client_->async_send_request(req);

    // the debounce, re enable after 250ms
    QTimer::singleShot(250, this, [this]{
        ui->moveShift->setEnabled(true);
    });
}


// Change of battery
void userinterface::on_batteryLevel_valueChanged(int value)
{

}

void userinterface::obtainBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{

    // msg->percentage is 0.0 to 1.0
    int percentage = static_cast<int>(msg->percentage * 100.0);

    // Update QProgressBar
    ui->batteryLevel->setValue(percentage);
}

// For the camera
// https://index.ros.org/p/ros_image_to_qimage/#humble-overview
void userinterface::obtainImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Should turn the ros image from the husky cam to a QImage
    QImage qimg = ros_image_to_qimage::Convert(*msg);

    // Display it on the gui
    // https://stackoverflow.com/questions/6913575/programatically-setting-the-pixmap-of-a-qlabel-in-qt
    ui->cameraDisplay->setPixmap(QPixmap::fromImage(qimg));
}




void userinterface::on_speedChange_valueChanged(double arg1)
{

}





void userinterface::on_cameraSelection_currentIndexChanged(int index) {
    QString selected = ui->cameraSelection->itemText(index);
    std::string topic = selected.toStdString();

    if (topic != current_image_topic_) {
        subToImageTopic(topic);
    }
}

void userinterface::subToImageTopic(const std::string &topic_name) {
    subToImage.reset();

    subToImage = node_->create_subscription<sensor_msgs::msg::Image> (
                topic_name,
                rclcpp::SensorDataQoS(),
                std::bind(&userinterface::obtainImage, this, std::placeholders::_1)
                );
    current_image_topic_ = topic_name;
}

