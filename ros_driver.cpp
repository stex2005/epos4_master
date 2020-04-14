#include "ros_driver.h"
/* EsmaCAT specific includes */
#include "esmacat_epos4.h"
#include "my_app.h"

/* ROS specific includes */
#include "esmacat_pkg/esmacat_publish.h"
#include "esmacat_pkg/esmacat_subscribe.h"
#include <boost/thread.hpp>
#include "ros/ros.h"

/************************/
/* ROS Subscriber Thread */
/************************/

void ros_driver::ROS_subscribe_thread(){

  //Setup a subscriber that will get data from other ROS nodes
  ros::MultiThreadedSpinner spinner(0);
  ros::NodeHandle n;

  ros::Subscriber subscriber = n.subscribe("EsmaCAT_sub_" + topic_name, 1000, &ros_driver::ROS_subscribe_callback, this);

  spinner.spin(); //blocking spin call for this thread
}

/************************/
/* ROS Publisher Thread */
/************************/

void ros_driver::ROS_publish_thread(){

  esmacat_pkg::esmacat_publish msg_to_publish;

  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  ros::Publisher publisher = n.advertise<esmacat_pkg::esmacat_publish>("EsmaCAT_pub_" + topic_name, 1000);

  ros_count = 100;


  while (ros::ok()){

    ros_driver::ROS_publish_msg data_pub_interim = this->get_pub_msg();

    ros_driver::set_pub_msg();
    msg_to_publish.encoder = data_pub_interim.input_encoder_counter;
    msg_to_publish.num     = ros_count;

    //Send data to ROS nodes that are not in the hard real-time loop
    publisher.publish(msg_to_publish);
    ros_count++;
    loop_rate.sleep();
  }
}

/**************************/
/* ROS Subcriber Callback */
/**************************/

void ros_driver::ROS_subscribe_callback(const esmacat_pkg::esmacat_subscribe::ConstPtr& msg)
{
  //Use the ros_motordriver class to communicate data from other ROS nodes that will be used in the hard real-time loop
  ROS_INFO("Subscribe: [%li]", msg->num);

  ros_driver::ROS_subscribe_msg data_sub_interim;
  data_sub_interim.output_enable = msg->enable;

  this->set_sub_msg(&data_sub_interim);

}


/***********************/
/* ROS Mutex Functions */
/***********************/

void ros_driver::set_pub_msg(){//esmacat_epos4* ecat_epos){

  interim_encoder_counter    = ros_count;

  //apply boost lock for accessing private variables
  boost::lock_guard<boost::mutex> lock(mtx_ros_publish);

  pub_msg.input_encoder_counter                  = interim_encoder_counter;
  pub_msg.ros_count                              = ros_count;
}

ros_driver::ROS_publish_msg ros_driver::get_pub_msg() const{
  //apply boost lock for accessing private variables
  boost::lock_guard<boost::mutex> lock(mtx_ros_publish);
  return(pub_msg);
}


void ros_driver::set_sub_msg(const ros_driver::ROS_subscribe_msg* msg){
  //apply boost lock for accessing private variables
  boost::lock_guard<boost::mutex> lock(mtx_ros_subscribe);
  sub_msg = *msg;
}

ros_driver::ROS_subscribe_msg ros_driver::get_sub_msg() const{
  //apply boost lock for accessing private variables
  boost::lock_guard<boost::mutex> lock(mtx_ros_subscribe);
  return(sub_msg);
}



