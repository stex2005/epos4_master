#include "ros_driver.h"

/* EsmaCAT specific includes */
#include "esmacat_epos4.h"

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

  ros::Subscriber subscriber = n.subscribe("EsmaCAT_" + topic_name, 1000, &ros_driver::ROS_subscribe_callback, this);

  spinner.spin(); //blocking spin call for this thread
}

/************************/
/* ROS Publisher Thread */
/************************/

void ros_driver::ROS_publish_thread(){

  esmacat_pkg::esmacat_publish data_to_publish;

  ros::NodeHandle n;
  ros::Rate loop_rate(100);
  ros::Publisher publisher = n.advertise<esmacat_pkg::esmacat_publish>("EsmaCAT_" + topic_name, 1000);

  int ros_count = 0;


  while (ros::ok()){

//    ros_driver::read data_read_interim = this->get_read_message();

//    data_to_send.encoder_counter                     = data_read_interim.EncoderCounter;
//    data_to_send.analog_input_from_external_source_0 = data_read_interim.AnalogInputFromExternalSource_0;
//    data_to_send.analog_input_from_external_source_1 = data_read_interim.AnalogInputFromExternalSource_1;
//    data_to_send.analog_input_from_ESCON_0           = data_read_interim.AnalogInputFromEscon_0;
//    data_to_send.analog_input_from_ESCON_1           = data_read_interim.AnalogInputFromEscon_1;
      data_to_publish.num = static_cast<long int>(sin(2*3.14*ros_count++/100)*1000);

    //Send data to ROS nodes that are not in the hard real-time loop
    publisher.publish(data_to_publish);

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

}
