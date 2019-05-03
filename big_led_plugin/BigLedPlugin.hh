#ifndef GAZEBO_BIG_LED_PLUGIN_HH
#define GAZEBO_BIG_LED_PLUGIN_HH

#include <memory>
#include <gazebo/common/Plugin.hh>
//realocado de velodyne_plugin.cc
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
//realocado de BigLedPlugin.cc
#include <gazebo/common/Color.hh>
#include <gazebo/common/Events.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/rendering/Visual.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"

namespace gazebo
{
  // Forward declare private data class.
  class BigLedPluginPrivate;
  
  class GAZEBO_VISIBLE BigLedPlugin : public VisualPlugin
  {
    /// \brief Constructor.
    public: BigLedPlugin();

    /// \brief Destructor.
    public: ~BigLedPlugin();

    // Documentation inherited
    public: virtual void Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf);

    /// \brief Update the plugin once every iteration of simulation.
    private: void Update();

    /// \internal
    /// \brief Private data pointer
    private: std::unique_ptr<BigLedPluginPrivate> dataPtr;
    
    //--- novo -- //
    
     /// \brief Pointer to the model.
    private: physics::ModelPtr model;
   
    
    /// \brief Handle an incoming message from ROS
    /// \param[in] _msg A float value that is used to set
    public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg);
   
    /// \brief ROS helper function that processes messages
    private: void QueueThread();
    
    
    /// \brief Set 
    /// \param[in] _vel New target
    public: void SetVisualColors(const double &_cor);
    
    /// \brief Handle incoming message
    /// \param[in] _msg Repurpose a vector3 message. This function will
    /// only use the x component.
    private: void OnMsg(ConstVector3dPtr _msg);
 
    /// \brief A node used for transport
    private: transport::NodePtr node;

    /// \brief A subscriber to a named topic.
    private: transport::SubscriberPtr sub;
    
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;

    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;

    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;

    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
 
    
    //--- fim novo
    
    
    
  };
} 
#endif
