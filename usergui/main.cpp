#include "userinterface.h"

#include <QApplication>
#include <QLocale>
#include <QTranslator>

#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // initialise ros2 communication

    QApplication a(argc, argv); // application object manages event loop

    QTranslator translator;
    const QStringList uiLanguages = QLocale::system().uiLanguages();
    for (const QString &locale : uiLanguages) {
        const QString baseName = "usergui_" + QLocale(locale).name();
        if (translator.load(":/i18n/" + baseName)) {
            a.installTranslator(&translator);
            break;
        }
    }
    userinterface w; // displays main window
    w.show();
//    return a.exec();

    int result = a.exec();
    rclcpp::shutdown(); // shuts down ros after qt
    return result;
}
