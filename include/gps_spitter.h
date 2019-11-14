#ifndef GPS_SPITTER_H
#define GPS_SPITTER_H

#include <QGeoPositionInfoSource>
#include <QAccelerometer>
#include <QGyroscope>
#include <QCompass>
#include <QTimer>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
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

private:
    QGeoPositionInfoSource *source;
    rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gps_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    QAccelerometer* accelerometer;
    QGyroscope* gyro;
    QCompass* compass;
    QTimer* timer;

};


#include "gps_spitter.moc"


#endif // GPS_SPITTER_H
