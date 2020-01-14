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
#include <sensor_msgs/msg/magnetic_field.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <geometry_msgs/msg/quaternion.hpp>

gps_spitter::gps_spitter(QObject *parent, std::shared_ptr<rclcpp::Node> node)
    : QObject(parent)
{
    node_ = node;
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
    mag_pub_ = node->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag_data", rclcpp::SensorDataQoS());

    timer = new QTimer(this);
    //20 millisecond interval
    accelerometer = new QAccelerometer(this);
    gyro = new QGyroscope(this);
    compass = new QCompass(this);
    mag = new QMagnetometer(this);

    accelerometer->start();
    gyro->start();
    compass->start();

    mag->setReturnGeoValues(true);
    mag->start();

    /*
     *
     * The magnetometer can report on either raw magnetic flux values or geomagnetic flux values.
     *  By default it returns raw magnetic flux values. The QMagnetometer::returnGeoValues property
     * must be set to return geomagnetic flux values.
        The primary difference between raw and geomagnetic values is that extra processing
        is done to eliminate local magnetic interference from the geomagnetic values so they represent
        only the effect of the Earth's magnetic field. This process is not perfect and the accuracy of each reading may change.
    */

    timer->callOnTimeout(std::bind(&gps_spitter::timer_cb, this) );
    timer->setInterval(500);
    timer->start();
    qDebug() << "accel hz rate: " << accelerometer->availableDataRates();
    qDebug() << "gyro hz rate: " << gyro->availableDataRates();
    qDebug() << "compass hz rate: " << compass->availableDataRates();
    qDebug() << "magnetometer hz rate: " << mag->availableDataRates();
    qDebug() << "available sensors are: " << QSensor::sensorTypes();

}

void gps_spitter::mag_cb()
{

    auto mag_reading = mag->reading();
    if(!mag_reading)
    {
        qWarning() << " no mag reading ";
    }
    if(mag_reading->calibrationLevel() < .3)
    {
        qWarning() << " mag cal value is bad: " << mag_reading->calibrationLevel();
    }
    sensor_msgs::msg::MagneticField mf_msg;
    auto now = node_->now();
    mf_msg.header.set__stamp(now);
    mf_msg.header.frame_id = "mag_link";
    mf_msg.magnetic_field.x = mag_reading->x();
    mf_msg.magnetic_field.y = mag_reading->y();
    mf_msg.magnetic_field.z = mag_reading->z();
    mag_pub_->publish(mf_msg);
}

void gps_spitter::timer_cb()
{
    imu_cb();
    mag_cb();
}



void gps_spitter::imu_cb()
{
    //qInfo() << "imu cb";
    sensor_msgs::msg::Imu imu_msg;

    auto accel_reading = accelerometer->reading();
    auto gyro_reading = gyro->reading();
    auto compass_reading = compass->reading();
    if(accel_reading)
    {

        imu_msg.linear_acceleration.x = accel_reading->x();
        imu_msg.linear_acceleration.y = accel_reading->y();
        imu_msg.linear_acceleration.z = accel_reading->z();
        imu_msg.linear_acceleration_covariance[0] = .01967;
        imu_msg.linear_acceleration_covariance[4] = .02223;
        imu_msg.linear_acceleration_covariance[8] = .061114;
    }
    else
    {
        qWarning() << "no accel reading";

        imu_msg.linear_acceleration_covariance[0] = -1;
        imu_msg.linear_acceleration_covariance[4] = -1;
        imu_msg.linear_acceleration_covariance[8] = -1;
    }
    if(gyro_reading)
    {

        imu_msg.angular_velocity.x = gyro_reading->x()/180.0 * M_PI;
        imu_msg.angular_velocity.y = gyro_reading->y()/180.0 * M_PI;
        imu_msg.angular_velocity.z = gyro_reading->z()/180.0 * M_PI;

        imu_msg.angular_velocity_covariance[0] = .00018;
        imu_msg.angular_velocity_covariance[4] = .00014641;
        imu_msg.angular_velocity_covariance[8] = .00007569;
    }
    else
    {
        imu_msg.angular_velocity_covariance[0] = -1;
        imu_msg.angular_velocity_covariance[4] = -1;
        imu_msg.angular_velocity_covariance[8] = -1;
        qWarning() << "no gyro reading";
    }
    if(compass_reading)
    {
        tf2::Quaternion myQuaternion;
        myQuaternion.setRPY(0, 0, compass_reading->azimuth()/180.0 * M_PI);
        myQuaternion.normalize();

        imu_msg.orientation.x = myQuaternion.getX();
        imu_msg.orientation.y = myQuaternion.getY();
        imu_msg.orientation.z = myQuaternion.getZ();
        imu_msg.orientation.w = myQuaternion.getW();

        imu_msg.orientation_covariance[8] = (1.1 - compass_reading->calibrationLevel() ) * .27387;

    }
    else{
        imu_msg.orientation_covariance[8] = -1;
        qWarning() << "no compass reading";
        return;
    }


    imu_msg.header.frame_id = "base_link";

    auto now = node_->now();
    imu_msg.header.set__stamp(now);

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

