/**
 * @file /rplidar_ros/include/rplidar_nodelet.hpp
 */
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef rplidar_ros_INCLUDE_RPLIDAR_NODELET_HPP_
#define rplidar_ros_INCLUDE_RPLIDAR_NODELET_HPP_

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nodelet/nodelet.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/Empty.h"
#include "boost/thread.hpp"

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header


using namespace rp::standalone::rplidar;

namespace rplidar_ros {

  class RPlidarNodelet: public nodelet::Nodelet
  {
  public:
    virtual void onInit();
    virtual void devicePoll();

    void read_scan();

    void publish_scan(ros::Publisher *pub,
                        rplidar_response_measurement_node_t *nodes,
                        size_t node_count, ros::Time start,
                        double scan_time, bool inverted,
                        float angle_min, float angle_max,
                        std::string frame_id);
    int init_driver(std::string& serial_port, int& serial_baudrate);
    bool checkRPLIDARHealth(RPlidarDriver * drv);
    bool stop_motor(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
    bool start_motor(std_srvs::Empty::Request &req,std_srvs::Empty::Response &res);
    bool reset_device(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool reset_scan(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);



    RPlidarNodelet(){};
    ~RPlidarNodelet(){};

  private:
    RPlidarDriver * drv;//
    bool initialised;
    ros::Time start_scan_time;
    ros::Time end_scan_time;
    u_result op_result;
    double scan_duration;
    bool angle_compensate;
    bool inverted;
    std::string frame_id;
    std::string serial_port;
    int serial_baudrate;
    int res;

    boost::mutex mutex_;
    ros::Publisher scan_pub;
    ros::ServiceServer stop_motor_service;
    ros::ServiceServer start_motor_service;
    ros::ServiceServer reset_device_service;
    ros::ServiceServer reset_scan_service;

    boost::shared_ptr<boost::thread> device_thread_;


  };


} // namespace rplidar_ros

#endif /* rplidar_ros_INCLUDE_RPLIDAR_NODELET_HPP_ */
