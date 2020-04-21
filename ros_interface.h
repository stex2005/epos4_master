#ifndef ROS_DRIVER_H
#define ROS_DRIVER_H

/* EsmaCAT specific includes */
#include "esmacat_epos4.h"

/* ROS specific includes */
#include "esmacat_pkg/esmacat_sensor.h"
#include "esmacat_pkg/esmacat_command.h"
#include <boost/thread.hpp>
#include "ros/ros.h"

class ros_interface
{
public:
  ros_interface(std::string topic_input)
    : topic_name(std::move(topic_input)), interim_encoder_counter(0), interim_enable(false)
//    boost_ROS_publish_thread(boost::thread(&ros_interface::ROS_publish_thread, this)),
//    boost_ROS_subscribe_thread(boost::thread(&ros_interface::ROS_subscribe_thread, this))
  {
    boost_ROS_publish_thread    = boost::thread(&ros_interface::ROS_publish_thread, this);
    boost_ROS_subscribe_thread  = boost::thread(&ros_interface::ROS_subscribe_thread, this);
    std::cout << "ROS interface objects instantiated" << std::endl;
  }

  ~ros_interface()
  {
    std::cout << "ROS interface threads joining" << std::endl;
    boost_ROS_publish_thread.join();
    boost_ROS_subscribe_thread.join();
  }

  /********************/
  /* ROS Sensor Stuff */
  /********************/

  struct ROS_sensor_msg
  {
    int32_t input_encoder_counter;
    int32_t interim_roscount;
    ROS_sensor_msg() : input_encoder_counter(0), interim_roscount(0) {}
  };
  void ROS_publish_thread();
  void set_sensor_msg(esmacat_epos4* ecat_epos);
  ROS_sensor_msg get_sensor_msg() const;

  /*********************/
  /* ROS Command Stuff */
  /*********************/

  struct ROS_command_msg
  {
    bool output_enable;
    ROS_command_msg() : output_enable(false) {}
  };
  void ROS_subscribe_thread();
  void ROS_subscribe_callback(const esmacat_pkg::esmacat_command::ConstPtr& msg);
  void set_command_msg(const ros_interface::ROS_command_msg* msg) ;
  ROS_command_msg get_command_msg() const;


private:

  const std::string topic_name;
  int32_t interim_encoder_counter;
  int32_t interim_timestamp;
  int32_t interim_roscount;
  bool    interim_enable;

  ROS_sensor_msg   pub_msg;
  ROS_command_msg  sub_msg;

  /****************/
  /* Boost Thread */
  /****************/

  boost::thread boost_ROS_publish_thread;
  boost::thread boost_ROS_subscribe_thread;

  mutable boost::mutex mtx_ros_sensor;
  mutable boost::mutex mtx_ros_command;
  ros_interface(){}//hide the default constructor so that it cannot be used outside the class unintentionally

};

#endif // ROS_DRIVER_H
