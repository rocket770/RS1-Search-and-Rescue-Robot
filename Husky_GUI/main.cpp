#include "husky_control.h"

#include <QApplication>
#include <QLocale>
#include <QTranslator>

// My addition
// /*

#include <rclcpp/rclcpp.hpp>

// */

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);
    Husky_Control w;
    w.show();
    int result = app.exec();
    rclcpp::shutdown();
    return result;
}
