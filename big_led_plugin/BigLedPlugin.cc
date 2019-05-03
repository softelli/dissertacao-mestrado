#include "BigLedPlugin.hh"

namespace gazebo
{
  /// \internal
  /// \class BigLedPlugin BigLedPlugin.hh
  /// \brief Private data for the BigLedPlugin class.
  
  class BigLedPluginPrivate
  {
    /// \brief Visual whose color will be changed.
    public: rendering::VisualPtr visual;

    /// \brief Connects to rendering update event.
    public: event::ConnectionPtr updateConnection;
    
    ///\brief _cor
    public: common::Color cor;
    
    //\brief
    public: std::string topicName;
    
        
  };
  
}

using namespace gazebo;

GZ_REGISTER_VISUAL_PLUGIN(BigLedPlugin)

/////////////////////////////////////////////////
BigLedPlugin::BigLedPlugin() : dataPtr(new BigLedPluginPrivate)
{
}

/////////////////////////////////////////////////
BigLedPlugin::~BigLedPlugin()
{
}

/////////////////////////////////////////////////
void BigLedPlugin::Load(rendering::VisualPtr _visual, sdf::ElementPtr _sdf)
{
  ROS_INFO("### BigLedPlugin::Load ###");
  
  if (!_visual || !_sdf)
  {
    gzerr << "No visual or SDF element specified. Plugin won't load." <<
        std::endl;
    return;
  }
  this->dataPtr->visual = _visual;

  if (!_sdf->HasElement("sdf"))
  {
    gzerr << "An <sdf> tag can't be found within the plugin." << std::endl;
    return;
  }
  auto sdfElem = _sdf->GetElement("sdf");
 
    // -- inicio novo
    
      // Create the node
      this->node = transport::NodePtr(new transport::Node());
            
      //#if GAZEBO_MAJOR_VERSION < 8
      //this->node->Init(this->model->GetWorld()->GetName());
      this->node->Init(this->dataPtr->visual->GetName());
      //ROS_INFO("### BigLedPlugin:: this->node->Initialized ###");
      //#else
      //this->node->Init(this->model->GetWorld()->Name());
      //ROS_INFO("### BigLedPlugin:: usind Name() ###");
      //#endif
      
      // Configuring a default topic name
      std::string topicName = this->dataPtr->visual->GetName();
      topicName.replace(topicName.find_first_of(":"),topicName.size()-1,"");
      std::string preTopicName = topicName;
      ROS_INFO_STREAM("BigLedPlugin::preTopicName => " << preTopicName );
      if (_sdf->HasElement("robotNamespace")) {
	    topicName = _sdf->Get<std::string>("robotNamespace") + "/" + preTopicName + "/big_led_color";
	 }       
      
      ROS_INFO_STREAM("BigLedPlugin::topicName => " << topicName );

      // Subscribe to the topic, and register a callback      
      ///////this->sub = this->node->Subscribe(topicName, &BigLedPlugin::OnMsg, this);      
      
      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
	
	int argc = 0;
	char **argv = NULL;
	ros::init(argc, argv, preTopicName,ros::init_options::NoSigintHandler);
      }

      // Create our ROS node. This acts in a similar manner to the Gazebo node
      this->rosNode.reset(new ros::NodeHandle(preTopicName));
      

      // Create a named topic, and subscribe to it.
      ros::SubscribeOptions so =
	ros::SubscribeOptions::create<std_msgs::Float32>(
	    //"/" + this->model->GetName() + "/cor",
	    //"/" + this->dataPtr->visual->GetName() + "/cor", //verificar uma forma de pegar o nome sem ::
	    "big_led_color",
	    1,
	    boost::bind(&BigLedPlugin::OnRosMsg, this, _1),
	    ros::VoidPtr(), &this->rosQueue);
	
      //ROS_INFO("### BigLedPlugin::created a topic ###");
	
      this->rosSub = this->rosNode->subscribe(so);
      //ROS_INFO("### BigLedPlugin::subscripted a topic###");
      
      // Spin up the queue helper thread.
      this->rosQueueThread = std::thread(std::bind(&BigLedPlugin::QueueThread, this));
      //ROS_INFO("### BigLedPlugin::spinned to queue helper thread ###");
  
  // -- fim novo    

  //BigLedPlugin (Load original) Connect to the world update signal
  this->dataPtr->updateConnection = event::Events::ConnectPreRender(
      std::bind(&BigLedPlugin::Update, this));
  
}

/////////////////////////////////////////////////
void BigLedPlugin::Update()
{
  
if (!this->dataPtr->visual)
  {
    gzerr << "The visual is null." << std::endl;
    return;
  }

  common::Color diffuse;
  common::Color ambient;
  
  diffuse = this->dataPtr->visual->GetDiffuse();
  ambient = this->dataPtr->visual->GetAmbient();
  
  //--- if it is a new color
  if(diffuse.r != this->dataPtr->cor.r || ambient.r != this->dataPtr->cor.r){
      
      double transp = 0.0;
      //if color is black be transparent
      if(diffuse.r == 0.0){
	 transp = 1.0;
      }
      
      common::Color color(this->dataPtr->cor.r, 
			  this->dataPtr->cor.g, 
			  0.0, //generate yellow if  
			  transp); 
      this->dataPtr->visual->SetDiffuse(color);
      this->dataPtr->visual->SetAmbient(color);
      
      //ROS_INFO("### diffuse colors updated ###");
      diffuse = this->dataPtr->visual->GetDiffuse();
           
  } 
  
}

/// \brief ROS helper function that processes messages
void BigLedPlugin::QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
	this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
    
/// \brief Handle an incoming message from ROS
/// \param[in] _msg A float value that is used to set the velocity
 void BigLedPlugin::OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetVisualColors(_msg->data);
    }

/// \brief Set the color
/// \param[in] _cor New target color
void BigLedPlugin::SetVisualColors(const double &_cor)
    {
       common::Color color(_cor, _cor, _cor, 1.0);
       this->dataPtr->cor = color;
    }
