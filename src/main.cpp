#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/float64.hpp"
#include <QDebug>
#include <QGeoPositionInfoSource>
#include "gps_spitter.h"

#include "fastrtps/fastrtps_dll.h"
#include "fastrtps/transport/UDPv4TransportDescriptor.h"

static std::shared_ptr<rclcpp::Node> node;

void thread_function()
{
    auto pub = node->create_publisher<std_msgs::msg::Float64>(
            "test", 1);
    qInfo() << "starting publisher";

    while(1)
    {
        std_msgs::msg::Float64 test_float;
        test_float.data = 1.0;

        pub->publish(test_float);
    }
}

void spin_fn()
{
    qInfo() << "starting node";

    rclcpp::spin(node);
    rclcpp::shutdown();
    qInfo() << "ending node";

}

void setup_fastrtps()
{
    static eprosima::fastrtps::rtps::UDPv4TransportDescriptor descriptor;
    descriptor.interfaceWhiteList.emplace_back("192.168.50.172");

}

int main(int argc, char *argv[])
{

    QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);

    QGuiApplication app(argc, argv);
    //setup_fastrtps();
    qInfo() << "starting app";
    rclcpp::init(argc, argv);
    qInfo() << "starting node";

    node = std::make_shared<rclcpp::Node>("android_node");
    qInfo() << "starting threads";

    std::thread t1(&thread_function);   // t starts running
    //std::thread t2(&spin_fn);   // t starts running
    gps_spitter spitter(nullptr, node);
    QQmlApplicationEngine engine;
    const QUrl url(QStringLiteral("qrc:/main.qml"));
    QObject::connect(&engine, &QQmlApplicationEngine::objectCreated,
                     &app, [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
            QCoreApplication::exit(-1);
    }, Qt::QueuedConnection);
    qInfo() << "loading engines";
    engine.load(url);

    return app.exec();
}


