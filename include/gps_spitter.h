#ifndef GPS_SPITTER_H
#define GPS_SPITTER_H

#include <QGeoPositionInfoSource>
#include <QAccelerometer>
#include <QGyroscope>
#include <QCompass>
#include <QMagnetometer>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/magnetic_field.hpp>

class gps_spitter : public QObject
{
    Q_OBJECT
public:
    explicit gps_spitter(QObject *parent, std::shared_ptr<rclcpp::Node> node);

signals:
    void gps_signal(const QGeoPositionInfo &info);

private slots:
    void gps_cb(const QGeoPositionInfo &info);
    void imu_cb();
    void timer_cb();
    void mag_cb();



private:
    QGeoPositionInfoSource *source;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr mag_pub_;

    std::shared_ptr<rclcpp::Node> node_;

    QAccelerometer* accelerometer;
    QGyroscope* gyro;
    QCompass* compass;
    QMagnetometer* mag;
    QTimer* timer;

};


//#include "moc_gps_spitter.cpp"


#endif // GPS_SPITTER_H
