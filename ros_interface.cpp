#include "ros_interface.h"

/** EsmaCAT specific includes **/
#include "esmacat_epos4.h"
#include "my_app.h"

/** ROS specific includes **/
#include "esmacat_pkg/esmacat_sensor.h"
#include "esmacat_pkg/esmacat_command.h"
#include <boost/thread.hpp>
#include "ros/ros.h"

/*************************/
/* ROS Subscriber Thread */
/*************************/

void ros_interface::ROS_subscribe_thread(){

  //Setup a subscriber that will get data from other ROS nodes
  ros::MultiThreadedSpinner spinner(0);
  ros::NodeHandle n;

  ros::Subscriber subscriber = n.subscribe("EsmaCAT_sub_" + topic_name, 1000, &ros_interface::ROS_subscribe_callback, this);

  spinner.spin(); //blocking spin call for this thread
}

/************************/
/* ROS Publisher Thread */
/************************/

void ros_interface::ROS_publish_thread(){

  esmacat_pkg::esmacat_sensor msg_to_publish;

  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  ros::Publisher publisher = n.advertise<esmacat_pkg::esmacat_sensor>("EsmaCAT_pub_" + topic_name, 1000);

  interim_roscount = 0;


  while (ros::ok()){

    ros_interface::ROS_sensor_msg data_pub_interim = this->get_sensor_msg();

    msg_to_publish.encoder = data_pub_interim.interim_encoder_counter;
    msg_to_publish.timestamp = data_pub_interim.interim_timestamp;

    //Send data to ROS nodes that are not in the hard real-time loop
    publisher.publish(msg_to_publish);
    interim_roscount++;
    loop_rate.sleep();
  }
}

/**************************/
/* ROS Subcriber Callback */
/**************************/

void ros_interface::ROS_subscribe_callback(const esmacat_pkg::esmacat_command::ConstPtr& msg)
{
  //Use the ros_motordriver class to communicate data from other ROS nodes that will be used in the hard real-time loop
  ROS_INFO("Subscribe State: [%d]", msg->enable);

  ros_interface::ROS_command_msg data_sub_interim;
  data_sub_interim.output_enable = msg->enable;

  this->set_command_msg(&data_sub_interim);

}


/***********************/
/* ROS Mutex Functions */
/***********************/

void ros_interface::set_sensor_msg(esmacat_epos4* ecat_epos)
  {

  interim_encoder_counter    = (int32_t) ecat_epos->get_position();
  interim_timestamp      = ecat_epos->get_elapsed_time();

  //apply boost lock for accessing private variables
  boost::lock_guard<boost::mutex> lock(mtx_ros_sensor);

  pub_msg.interim_encoder_counter                  = interim_encoder_counter;
  pub_msg.interim_timestamp                        = interim_timestamp;
}

ros_interface::ROS_sensor_msg ros_interface::get_sensor_msg() const{
  //apply boost lock for accessing private variables
  boost::lock_guard<boost::mutex> lock(mtx_ros_sensor);
  return(pub_msg);
}


void ros_interface::set_command_msg(const ros_interface::ROS_command_msg* msg){
  //apply boost lock for accessing private variables
  boost::lock_guard<boost::mutex> lock(mtx_ros_command);
  sub_msg = *msg;
}

ros_interface::ROS_command_msg ros_interface::get_command_msg() const{
  //apply boost lock for accessing private variables
  boost::lock_guard<boost::mutex> lock(mtx_ros_command);
  return(sub_msg);
}



