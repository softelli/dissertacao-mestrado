#ifndef GAZEBO_BIG_LED_PLUGIN_HH
#define GAZEBO_BIG_LED_PLUGIN_HH

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/common/common.hh>
#include "ros/ros.h"
#include <std_msgs/Int8.h>

namespace gazebo
{
class BigLedPlugin : public VisualPlugin
{

    public: BigLedPlugin();
    public: ~BigLedPlugin();
    
    public: void Load(rendering::VisualPtr _parent, sdf::ElementPtr /*_sdf*/);
    // Called by the world update start event
    public: void OnUpdate();
    
    //Called by message received in topic
    public: void cmdSetLedColor(const std_msgs::Int8::ConstPtr& cmd_msg);

    //ros node handle
    private: ros::NodeHandle* rosNode;
    
    private: ros::Subscriber cmd_big_led;
    // Pointer to the bigLed
    private: rendering::VisualPtr model;
    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
};
}

#endif // big_led.h
