#include "userinterface.h"
#include "./ui_userinterface.h"

#include "ros_image_to_qimage/ros_image_to_qimage.hpp"

userinterface::userinterface(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::userinterface)
{
    ui->setupUi(this);

    // the ros node
    // https://docs.ros.org/en/crystal/Tutorials/Writing-A-Simple-Cpp-Service-And-Client.html
    node_ = std::make_shared<rclcpp::Node>("usergui_node");

    //current_image_topic_ = "/camera/image";
    //subToImageTopic(current_image_topic_);

    // https://doc.qt.io/qt-6/qcombobox.html#addItem
    // Add camera topics to the drop down
    // Ensure Qstring from the regular string
    ui->cameraSelection->addItem(QString::fromStdString("/camera/image"));
    ui->cameraSelection->addItem(QString::fromStdString("/yolo_detector/detections/image"));
    ui->cameraSelection->addItem(QString::fromStdString("/night_vision/image"));

    // subscribing to the camera topic
    //subToImage = node_->create_subscription<sensor_msgs::msg::Image>(
    //    "/camera/image", rclcpp::SensorDataQoS(), std::bind(&userinterface::obtainImage, this, std::placeholders::_1));

    current_image_topic_ = "/camera/image";  // default topic
    subToImageTopic(current_image_topic_); // calls function to ensure we subscribe to that camera topic


    // publish movement to robot
    velocity = node_->create_publisher<geometry_msgs::msg::Twist> (
        "/ui/move", 10);

    // Obtain the battery node percentage
    subtoBattery = node_->create_subscription<sensor_msgs::msg::BatteryState>(
        "/battery_state", rclcpp::SensorDataQoS(), std::bind(&userinterface::obtainBattery, this, std::placeholders::_1));
    
    // client to receive service after 
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
// The idea is that it publishes the commands to ui/move topic 
// Calling the function publishes the speed defined in the header files
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


// To change the environment within the simulation between day and night
// Trigger service request for /toggle_tiem
// https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Service-And-Client.html
void userinterface::on_dayShift_clicked()
{

    // brief debounce so a double click dont spam requests
    ui->dayShift->setEnabled(false);

    // Request for trigger service
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();

    // send request for the shift to day / night
    (void)toggle_time_client_->async_send_request(req);

    // The debounce, re enable after 250ms, to avoid spam
    // https://doc.qt.io/qt-6/qtimer.html#singleShot
    QTimer::singleShot(250, this, [this]{
        ui->dayShift->setEnabled(true);
    });
}

// Change from Manual to Autonomous movement
void userinterface::on_moveShift_clicked()
{
    //RCLCPP_INFO(node_->get_logger(), "Hit!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!.");

    //  brief debounce so a double click dont spam requests
    ui->moveShift->setEnabled(false);

    // Make a request
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();

    // Send it to service to enable autonomous movement
    (void)resume_explore_client_->async_send_request(req);

    // the debounce, re enable after 250ms
    QTimer::singleShot(250, this, [this]{
        ui->moveShift->setEnabled(true);
    });
}


// Change of battery... utilised personalised function below
void userinterface::on_batteryLevel_valueChanged(int value)
{

}

// Obtain the level of the battery from the /battery_state topic
void userinterface::obtainBattery(const sensor_msgs::msg::BatteryState::SharedPtr msg)
{

    // msg->percentage is 0.0 to 1.0
    int percentage = static_cast<int>(msg->percentage * 100.0);

    // Update QProgressBar
    ui->batteryLevel->setValue(percentage);
}

// For the camera
// Package below
// https://index.ros.org/p/ros_image_to_qimage/#humble-overview
void userinterface::obtainImage(const sensor_msgs::msg::Image::SharedPtr msg) {
    // Converts the ros image from the husky cam to a QImage used by the GUI
    QImage qimg = ros_image_to_qimage::Convert(*msg);

    // Display it on the GUI
    // https://stackoverflow.com/questions/6913575/programatically-setting-the-pixmap-of-a-qlabel-in-qt
    ui->cameraDisplay->setPixmap(QPixmap::fromImage(qimg));
}

// Didn't update properly, only for manual movement
void userinterface::on_speedChange_valueChanged(double arg1)
{

}

// If the new selected camera topic is not the current one
// Change to the new topic
// Ensure to select the correct topic at the correct index
// https://doc.qt.io/qt-6/qcombobox.html
// Change from QString to standard string
// https://stackoverflow.com/questions/4214369/how-to-convert-qstring-to-stdstring
void userinterface::on_cameraSelection_currentIndexChanged(int index) {
    QString selected = ui->cameraSelection->itemText(index);
    std::string topic = selected.toStdString();

    if (topic != current_image_topic_) {
        subToImageTopic(topic);
    }
}

// Reset the current topic
// Ensure to create new subscription to new topic
void userinterface::subToImageTopic(const std::string &topic_name) {
    subToImage.reset();

    subToImage = node_->create_subscription<sensor_msgs::msg::Image> (
                topic_name,
                rclcpp::SensorDataQoS(),
                std::bind(&userinterface::obtainImage, this, std::placeholders::_1)
                );
    current_image_topic_ = topic_name;
}

