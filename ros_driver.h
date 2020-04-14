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
    : topic_name(std::move(topic_input)), interim_encoder_counter(0), interim_enable(false),
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

  int64_t ros_count;
  /***********************/
  /* ROS Publisher Stuff */
  /***********************/

  struct ROS_publish_msg
  {
    int32_t input_encoder_counter;
    int64_t ros_count;
    ROS_publish_msg() : input_encoder_counter(0), ros_count(0) {}
  };
  void ROS_publish_thread();
  void set_pub_msg(esmacat_epos4* ecat_epos);
  ROS_publish_msg get_pub_msg() const;

  /************************/
  /* ROS Subscriber Stuff */
  /************************/

  struct ROS_subscribe_msg
  {
    bool output_enable;
    ROS_subscribe_msg() : output_enable(false) {}
  };
  void ROS_subscribe_thread();
  void ROS_subscribe_callback(const esmacat_pkg::esmacat_subscribe::ConstPtr& msg);
  void set_sub_msg(const ros_driver::ROS_subscribe_msg* msg) ;
  ROS_subscribe_msg get_sub_msg() const;


private:

  const std::string topic_name;
  int32_t interim_encoder_counter;
  bool    interim_enable;

  ROS_publish_msg   pub_msg;
  ROS_subscribe_msg  sub_msg;

  boost::thread boost_ROS_publish_thread;
  boost::thread boost_ROS_subscribe_thread;

  mutable boost::mutex mtx_ros_publish;
  mutable boost::mutex mtx_ros_subscribe;
  ros_driver(){}//hide the default constructor so that it cannot be used outside the class unintentionally

};

#endif // ROS_DRIVER_H
