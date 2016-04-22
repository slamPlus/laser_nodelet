/**
 * @file /rplidar_ros/src/rplidar_nodelet.cpp
 */
/*****************************************************************************
** Includes
*****************************************************************************/

#include "pluginlib/class_list_macros.h"

#include "rplidar_ros/rplidar_nodelet.hpp"


#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define DEG2RAD(x) ((x)*M_PI/180.)



namespace rplidar_ros {

  void RPlidarNodelet::onInit()
  {
    drv = NULL;
    serial_baudrate = 115200;
    inverted = false;
    angle_compensate = true;


    initialised = false;
    res = 0;

    ros::NodeHandle &nh = this->getNodeHandle();
    ros::NodeHandle &nh_private = this->getPrivateNodeHandle();
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    nh_private.param<int>("serial_baudrate", serial_baudrate, 115200);
    nh_private.param<std::string>("frame_id", frame_id, "laser_frame");
    nh_private.param<bool>("inverted", inverted, "false");
    nh_private.param<bool>("angle_compensate", angle_compensate, "true");

    res = RPlidarNodelet::init_driver(serial_port, serial_baudrate);
    if (res < 0)
    {
      NODELET_ERROR_STREAM("Failed to initialise driver. Exiting. (Halt!)");
      //return res;
      //should exit nodelet here and not go any further
    } // should implement unloading method on misconfiguration
    else
    {
      initialised = true;


      stop_motor_service = nh_private.advertiseService("stop_motor", &RPlidarNodelet::stop_motor, this);
      start_motor_service = nh_private.advertiseService("start_motor", &RPlidarNodelet::start_motor, this);
      reset_device_service = nh_private.advertiseService("reset_device", &RPlidarNodelet::reset_device, this);
      reset_scan_service = nh_private.advertiseService("reset_scan", &RPlidarNodelet::reset_scan, this);

      scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);

      device_thread_ = boost::shared_ptr< boost::thread >
        (new boost::thread(boost::bind(&RPlidarNodelet::devicePoll, this)));
    }
  }

  void RPlidarNodelet::devicePoll()
  {
    while(ros::ok())
    {

      if (initialised)
      {
        boost::mutex::scoped_lock lock(mutex_);
        this->read_scan(); // publish inside this
      } // release mutex lock
      else
      {
        if (!drv)
        {
          NODELET_INFO("RPLidar: no driver initialised, trying to re-initialise it again.");
          res = init_driver(serial_port, serial_baudrate);
          if (res < 0)
          {
            NODELET_ERROR_STREAM("RPLidar: failed to re-initialise driver, will keep trying.");
            ros::Duration(1.0).sleep();
          }
          else
          {
            initialised = true;
          }
        }
        else
        {
          if (!checkRPLIDARHealth(drv))
          {
            NODELET_INFO("Bad health. Let's better re-initialise the driver.");
            res = init_driver(serial_port, serial_baudrate);
            if (res < 0)
            {
              NODELET_ERROR_STREAM("Failed to re-initialise driver. Will keep trying.");
              ros::Duration(1.0).sleep();
            }
            else
            {
              initialised = true;
            }
          }
          else
          {
            NODELET_WARN("RPLidar: driver is ok, but health check failed -> reset and restart scanning");
            u_result op_result;
            op_result = drv->stop();
            if (op_result == RESULT_OK)
            {
              op_result = drv->startScan();
              if (op_result == RESULT_OK)
              {
                initialised = true;
              }
              else
              {
                NODELET_ERROR("RPLidar: failed to restart scanning. (%i)", op_result);
              }
            }
            else
            {
              NODELET_ERROR("RPLidar: failed to stop scanning. (%i)", op_result);
            }
          }
        }
      }
    }
    // done!
    RPlidarDriver::DisposeDriver(&drv);
  }

  void RPlidarNodelet::read_scan()
  {
    // Read
    rplidar_response_measurement_node_t nodes[360*2];
    size_t   count = _countof(nodes);

    start_scan_time = ros::Time::now();
    op_result = drv->grabScanData(nodes, count);
    end_scan_time = ros::Time::now();
    scan_duration = (end_scan_time - start_scan_time).toSec() * 1e-3;

    if (op_result == RESULT_OK) {
        op_result = drv->ascendScanData(nodes, count);

        float angle_min = DEG2RAD(0.0f);
        float angle_max = DEG2RAD(359.0f);
        if (op_result == RESULT_OK) {
            if (angle_compensate) {
                const int angle_compensate_nodes_count = 360;
                const int angle_compensate_multiple = 1;
                int angle_compensate_offset = 0;
                rplidar_response_measurement_node_t angle_compensate_nodes[angle_compensate_nodes_count];
                memset(angle_compensate_nodes, 0, angle_compensate_nodes_count*sizeof(rplidar_response_measurement_node_t));
                int i = 0, j = 0;
                for( ; i < count; i++ ) {
                    if (nodes[i].distance_q2 != 0) {
                        float angle = (float)((nodes[i].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                        int angle_value = (int)(angle * angle_compensate_multiple);
                        if ((angle_value - angle_compensate_offset) < 0) angle_compensate_offset = angle_value;
                        for (j = 0; j < angle_compensate_multiple; j++) {
                            angle_compensate_nodes[angle_value-angle_compensate_offset+j] = nodes[i];
                        }
                    }
                }

                this->publish_scan(&scan_pub, angle_compensate_nodes, angle_compensate_nodes_count,
                         start_scan_time, scan_duration, inverted,
                         angle_min, angle_max,
                         frame_id);
            } else {
                int start_node = 0, end_node = 0;
                int i = 0;
                // find the first valid node and last valid node
                while (nodes[i++].distance_q2 == 0);
                start_node = i-1;
                i = count -1;
                while (nodes[i--].distance_q2 == 0);
                end_node = i+1;

                angle_min = DEG2RAD((float)(nodes[start_node].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);
                angle_max = DEG2RAD((float)(nodes[end_node].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f);

                this->publish_scan(&scan_pub, &nodes[start_node], end_node-start_node +1,
                         start_scan_time, scan_duration, inverted,
                         angle_min, angle_max,
                         frame_id);
           }
      } else if (op_result == RESULT_OPERATION_FAIL) {
            // All the data is invalid
            // SHOULD NOT PUBLISH ANY DATA FROM here
            // BECAUSE IT CAN CRASH THE PROGRAMS USING THE DATA

            /*
            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(359.0f);

            NODELET_WARN_STREAM("RPLidar: publishing invalid data; might burst! watch out..");

            this->publish_scan(&scan_pub, nodes, count,
                         start_scan_time, scan_duration, inverted,
                         angle_min, angle_max,
                         frame_id);
            */
        }

    }
    else if ( op_result == RESULT_OPERATION_TIMEOUT)
    {
      NODELET_WARN_STREAM("RPLidar: she's dead Jim! [timed out waiting for a full 360 scan]");
      initialised = false;
    }
  }

  void RPlidarNodelet::publish_scan(ros::Publisher *pub,
                    rplidar_response_measurement_node_t *nodes,
                    size_t node_count, ros::Time start,
                    double scan_time, bool inverted,
                    float angle_min, float angle_max,
                    std::string frame_id)
  {
      static int scan_count = 0;

      // Allocate a new shared pointer for zero-copy sharing to other nodelet
      sensor_msgs::LaserScanPtr scan_msg(new sensor_msgs::LaserScan);

      scan_msg->header.stamp = start;
      scan_msg->header.frame_id = frame_id;
      scan_count++;

      bool reversed = (angle_max > angle_min);
      if ( reversed ) {
        scan_msg->angle_min =  M_PI - angle_max;
        scan_msg->angle_max =  M_PI - angle_min;
      } else {
        scan_msg->angle_min =  M_PI - angle_min;
        scan_msg->angle_max =  M_PI - angle_max;
      }
      scan_msg->angle_increment =
          (scan_msg->angle_max - scan_msg->angle_min) / (double)(node_count-1);

      scan_msg->scan_time = scan_time;
      scan_msg->time_increment = scan_time / (double)(node_count-1);

      scan_msg->range_min = 0.15;
      scan_msg->range_max = 6.;

      scan_msg->intensities.resize(node_count);
      scan_msg->ranges.resize(node_count);
      bool reverse_data = (!inverted && reversed) || (inverted && !reversed);
      if (!reverse_data) {
          for (size_t i = 0; i < node_count; i++) {
              float read_value = (float) nodes[i].distance_q2/4.0f/1000;
              if (read_value == 0.0)
                  scan_msg->ranges[i] = std::numeric_limits<float>::infinity();
              else
                  scan_msg->ranges[i] = read_value;
              scan_msg->intensities[i] = (float) (nodes[i].sync_quality >> 2);
          }
      } else {
          for (size_t i = 0; i < node_count; i++) {
              float read_value = (float)nodes[i].distance_q2/4.0f/1000;
              if (read_value == 0.0)
                  scan_msg->ranges[node_count-1-i] = std::numeric_limits<float>::infinity();
              else
                  scan_msg->ranges[node_count-1-i] = read_value;
              scan_msg->intensities[node_count-1-i] = (float) (nodes[i].sync_quality >> 2);
          }
      }

      // Check if the data size is not equal to 360. if not don't publish it
      // This is strict check which could to lose to see only if node_count is 0
      // But 99% of the time RPlidar produces 360 sized data
      if (node_count == 360) {  // Only publish when data size is 360
        pub->publish(scan_msg);
      } else {
        NODELET_WARN_STREAM("RPLidar: not publishing since data count < 360 [" << node_count << "]");
      }
  }

  int RPlidarNodelet::init_driver(std::string& serial_port, int& serial_baudrate)
  {
    // check if there is an existing driver instance
    if (drv)
    {
      if (drv->isConnected())
      {
        NODELET_INFO_STREAM("RPlidar: disconnecting old driver instance.");
        drv->disconnect();
      }
      NODELET_INFO_STREAM("RPlidar: disposing old driver instance.");
      RPlidarDriver::DisposeDriver(&drv);
    }

    // create the driver instance
    drv = RPlidarDriver::CreateDriver(RPlidarDriver::DRIVER_TYPE_SERIALPORT);

    if (!drv)
    {
      NODELET_ERROR_STREAM("RPlidar: failed to create driver!");
      return -2;
    }

    // make connection...
    NODELET_INFO_STREAM("RPlidar: connecting on " << serial_port << " [" << serial_baudrate << "]");
    u_result result = drv->connect(serial_port.c_str(), (_u32)serial_baudrate);
    if (IS_FAIL(result))
    {
      NODELET_ERROR_STREAM("RPlidar: cannot bind to the specified serial port [" << serial_port << "]");
      RPlidarDriver::DisposeDriver(&drv);
      return -1;
    }

    // check health...
    if (!checkRPLIDARHealth(drv))
    {
      RPlidarDriver::DisposeDriver(&drv);
      return -1;
    }

    // start scan...can pass true to this to 'force' it, whatever that is
    u_result start_scan_result = drv->startScan();
    if ( start_scan_result != RESULT_OK )
    {
      NODELET_ERROR_STREAM("RPLidar: failed to put the device into scanning mode [" << start_scan_result << "]");
      RPlidarDriver::DisposeDriver(&drv);
      return -1;
    }

    return 0;
  }

  bool RPlidarNodelet::checkRPLIDARHealth(RPlidarDriver * drv)
  {
      u_result     op_result;
      rplidar_response_device_health_t healthinfo;

      op_result = drv->getHealth(healthinfo);
      if (IS_OK(op_result)) {
        switch (healthinfo.status) {
          case(RPLIDAR_STATUS_OK) : {
            return true;
          }
          case(RPLIDAR_STATUS_ERROR) : {
            NODELET_ERROR_STREAM("RPLidar: health check failed, please reboot the device");
            return false;
          }
          default: {
            NODELET_WARN_STREAM("RPLidar: health not ok, but unhandled status returned [" << healthinfo.status << "]");
            return true;
          }
        }
      } else {
        NODELET_ERROR("RPLidar: cannot retrieve the rplidar health status %x", op_result);
        return false;
      }
  }

  bool RPlidarNodelet::stop_motor(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res)
  {
    if(!drv)
          return false;

    NODELET_DEBUG("RPLidar : stopping the motor");
    drv->stop();
    drv->stopMotor();
    return true;
  }

  bool RPlidarNodelet::start_motor(std_srvs::Empty::Request &req,
                                  std_srvs::Empty::Response &res)
  {
    if(!drv)
          return false;
    NODELET_DEBUG("RPLidar : starting the motor");
    drv->startMotor();
    drv->startScan();;
    return true;
  }

  bool RPlidarNodelet::reset_device(std_srvs::Trigger::Request &req,
                    std_srvs::Trigger::Response &res)
  {
    if(!drv)
    {
      return false;
    }
    NODELET_INFO("RPLidar: resetting the device.");
    u_result op_result;
    op_result = drv->reset();
    if (op_result == RESULT_OK)
    {
      res.success = true;
    }
    else
    {
      NODELET_ERROR("RPLidar: failed to reset the device! (%i)", op_result);
      res.success = false;
    }
    return true;
  }

  bool RPlidarNodelet::reset_scan(std_srvs::Trigger::Request &req,
                  std_srvs::Trigger::Response &res)
  {
    if(!drv)
    {
      return false;
    }
    NODELET_DEBUG("RPLidar: stopping and restarting scanning.");
    u_result op_result;
    op_result = drv->stop();
    if (op_result == RESULT_OK)
    {
      op_result = drv->startScan();
      if (op_result == RESULT_OK)
      {
        res.success = true;
      }
      else
      {
        NODELET_ERROR("RPLidar: failed to restart scanning. (%i)", op_result);
        res.success = false;
      }
    }
    else
    {
      NODELET_ERROR("RPLidar: failed to stop scanning. (%i)", op_result);
      res.success = false;
    }
    return true;
  }

} // namespace rplidar_ros

PLUGINLIB_DECLARE_CLASS(rplidar_nodelet, RPlidarNodelet, rplidar_ros::RPlidarNodelet, nodelet::Nodelet);
