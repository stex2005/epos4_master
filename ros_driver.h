#ifndef ROS_DRIVER_H
#define ROS_DRIVER_H

/* EsmaCAT specific includes */
#include "esmacat_epos4.h"

/* ROS specific includes */
#include "esmacat_pkg/esmacat_publish.h"
#include "esmacat_pkg/esmacat_subscribe.h"
#include <boost/thread.hpp>
#include "ros/ros.h"

class ros_driver
{
public:
  ros_driver(std::string topic_input)
    : topic_name(std::move(topic_input)), ros_encoder_counter(0),
      boost_ROS_publish_thread(boost::thread(&ros_driver::ROS_publish_thread, this)),
      boost_ROS_subscribe_thread(boost::thread(&ros_driver::ROS_subscribe_thread, this))
  {
    std::cout << "ROS Driver objects instantiated" << std::endl;
  }

  ~ros_driver()
  {
      std::cout << "ROS threads joining" << std::endl;
      boost_ROS_publish_thread.join();
      boost_ROS_subscribe_thread.join();
  }

  void ROS_publish_thread();

  void ROS_subscribe_thread();
  void ROS_subscribe_callback(const esmacat_pkg::esmacat_subscribe::ConstPtr& msg);



private:

  const std::string topic_name;
  int32_t ros_encoder_counter;

  boost::thread boost_ROS_publish_thread;
  boost::thread boost_ROS_subscribe_thread;

  mutable boost::mutex mtx_ros_publish;
  mutable boost::mutex mtx_ros_subscribe;
  ros_driver(){}//hide the default constructor so that it cannot be used outside the class unintentionally

};

#endif // ROS_DRIVER_H
