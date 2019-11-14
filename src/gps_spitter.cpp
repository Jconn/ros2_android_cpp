#include "gps_spitter.h"

#include <QDebug>
#include <QGeoPositionInfoSource>
#include <QAccelerometerReading>
#include <QAccelerometer>
#include <QTimer>
#include <QGyroscopeReading>
#include <QRotationSensor>
#include <QCompass>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/quaternion.hpp>

gps_spitter::gps_spitter(QObject *parent, std::shared_ptr<rclcpp::Node> node)
    : QObject(parent)
{
    source = QGeoPositionInfoSource::createDefaultSource(this);
    if (source) {
        qWarning() << "minimum update interval is " << source->minimumUpdateInterval();
        source->setUpdateInterval(50);
        connect(source, SIGNAL(positionUpdated(QGeoPositionInfo)),
                this, SLOT(gps_cb(QGeoPositionInfo)));
        source->startUpdates();
    }
    gps_pub_ = node->create_publisher<sensor_msgs::msg::NavSatFix>("gps/data", rclcpp::SensorDataQoS());
    imu_pub_ = node->create_publisher<sensor_msgs::msg::Imu>("imu/data", rclcpp::SensorDataQoS());

    timer = new QTimer(this);
    //20 millisecond interval
    accelerometer = new QAccelerometer(this);
    gyro = new QGyroscope(this);
    compass = new QCompass(this);

    accelerometer->start();
    gyro->start();
    compass->start();

    timer->callOnTimeout(std::bind(&gps_spitter::imu_cb, this) );
    timer->setInterval(20);
    timer->start();
    qDebug() << "available sensors are: " << QSensor::sensorTypes();
    qDebug() << "accel hz rate: " << accelerometer->availableDataRates();
    qDebug() << "gyro hz rate: " << gyro->availableDataRates();
    qDebug() << "compass hz rate: " << compass->availableDataRates();

}

void gps_spitter::imu_cb()
{
    //qInfo() << "imu cb";
    sensor_msgs::msg::Imu imu_msg;

    auto accel_reading = accelerometer->reading();
    auto gyro_reading = gyro->reading();
    auto compass_reading = compass->reading();
    if(!accel_reading)
    {
        qWarning() << "no accel reading";
        return;
    }
    if(!gyro_reading)
    {
        qWarning() << "no gyro reading";
        return;
    }
    if(!compass_reading)
    {
        qWarning() << "no compass reading";
        return;
    }


    auto now = QDateTime::currentDateTime();
    imu_msg.header.frame_id = "base_link";
    imu_msg.header.stamp.sec = now.currentSecsSinceEpoch();
    auto ns = (now.currentMSecsSinceEpoch() - now.currentSecsSinceEpoch()*1000)*1e6;
    imu_msg.header.stamp.nanosec = ns;

    imu_msg.linear_acceleration.x = accel_reading->x();
    imu_msg.linear_acceleration.y = accel_reading->y();
    imu_msg.linear_acceleration.z = accel_reading->z();

    imu_msg.angular_velocity.x = gyro_reading->x()/180.0 * M_PI;
    imu_msg.angular_velocity.y = gyro_reading->y()/180.0 * M_PI;
    imu_msg.angular_velocity.z = gyro_reading->z()/180.0 * M_PI;

    tf2::Quaternion myQuaternion;
    myQuaternion.setRPY(0, 0, compass_reading->azimuth()/180.0 * M_PI);
    myQuaternion.normalize();

    imu_msg.orientation.x = myQuaternion.getX();
    imu_msg.orientation.y = myQuaternion.getY();
    imu_msg.orientation.z = myQuaternion.getZ();
    imu_msg.orientation.w = myQuaternion.getW();

    imu_msg.linear_acceleration_covariance[0] = .01967;
    imu_msg.linear_acceleration_covariance[4] = .02223;
    imu_msg.linear_acceleration_covariance[8] = .061114;

    imu_msg.angular_velocity_covariance[0] = .00018;
    imu_msg.angular_velocity_covariance[4] = .00014641;
    imu_msg.angular_velocity_covariance[8] = .00007569;


    imu_msg.orientation_covariance[8] = (1.1 - compass_reading->calibrationLevel() ) * .27387;
    imu_pub_->publish(imu_msg);

/*
 * from some paper on imu values in phone:
 * https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4850991/
        Accelerometer [mg0]	Gyroscope [mrad/s]
        Parameter	X	Y	Z	X	Y	Z
Average	14.3	14.6	25.3	9.4	8.7	6.1
StDev	14.2	15.2	25.1	13.6	12.1	8.7
50th percentile	10.0	9.9	18.5	3.1	4.3	2.8
90th percentile	30.1	31.5	60.3	30.8	22.9	17.1
95th percentile	43.6	45.9	71.1	40.5	35.4	28.2
100th percentile	90.9	82.7	161.0	142.7	81.7	158.2


*/
}


void gps_spitter::gps_cb(const QGeoPositionInfo &info)
{

    if(!info.isValid())
    {
        //TODO: is this the no fix case?
        qWarning() << "invalid gps info, early return";

        return;
    }
    qDebug() << "Position updated:" << info;
    QDateTime timestamp = info.timestamp();
    QGeoCoordinate coord = info.coordinate();

    sensor_msgs::msg::NavSatFix gps_msg;
    gps_msg.header.frame_id = "base_link";
    gps_msg.header.stamp.sec = timestamp.toSecsSinceEpoch();

    int64_t msecs_quantized = timestamp.toSecsSinceEpoch() * 1000;
    int64_t msecs = timestamp.toMSecsSinceEpoch();
    int64_t nsecs_delta = (msecs - msecs_quantized) * 1e6;
    gps_msg.header.stamp.nanosec = nsecs_delta;

    gps_msg.latitude = coord.latitude();
    gps_msg.longitude = coord.longitude();
    gps_msg.altitude = coord.altitude();

    auto lat_accuracy = info.attribute(QGeoPositionInfo::Attribute::HorizontalAccuracy);
    auto alt_accuracy = info.attribute(QGeoPositionInfo::Attribute::VerticalAccuracy);

    double lat_variance = lat_accuracy * lat_accuracy;
    double alt_variance = alt_accuracy * alt_accuracy;
    // Position covariance [m^2] defined relative to a tangential plan
    // through the reported position. The components are East, North, and
    //Up (ENU), in row-major order.
    gps_msg.position_covariance[0] = lat_variance;
    gps_msg.position_covariance[4] = lat_variance;
    gps_msg.position_covariance[8] = alt_variance;
    gps_msg.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;

    //TODO: parse satellite info for more detailed information here
    gps_msg.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;

    gps_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;

    gps_pub_->publish(gps_msg);
}

