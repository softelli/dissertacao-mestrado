#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include "light_sensor_plugin.h"
#include "gazebo_plugins/gazebo_ros_camera.h"
#include <string>
#include <cstring>
#include <gazebo/sensors/Sensor.hh>
#include <gazebo/sensors/CameraSensor.hh>
#include <gazebo/sensors/SensorTypes.hh>
#include <sensor_msgs/Illuminance.h>
//#include <sdf/sdf.hh>
//include <sdf/Param.hh>

namespace gazebo
{
  // Register this plugin with the simulator
  GZ_REGISTER_SENSOR_PLUGIN(GazeboRosLight)

  // Constructor
  GazeboRosLight::GazeboRosLight():
  //_nh("light_sensor_plugin"),
  _fov(32), //horizontal
  _range(24) //vertical
  {
    //_sensorPublisher = _nh.advertise<sensor_msgs::Illuminance>("lightSensor", 1);
  }

  // Destructor
  GazeboRosLight::~GazeboRosLight()
  {
    ROS_DEBUG_STREAM_NAMED("light_sensor_plugin","Unloaded");
  }

  void GazeboRosLight::Load(sensors::SensorPtr _parent, sdf::ElementPtr _sdf)
  {
    
     
    
    if (!_parent || !_sdf)
    {
      ROS_FATAL_STREAM("No PARENT or SDF element specified. Plugin won't load." << std::endl);
      return;
    }
    
    if (!_sdf->HasElement("robotNamespace"))
    {
      ROS_INFO_STREAM("An <robotNamespace> tag can't be found within the plugin." << std::endl);  
    }
    
    if (!_sdf->HasElement("variacao"))
    {
      ROS_INFO("An <variacao> tag can't be found within the plugin. Adopting variacao = 0.1"); 
      this->_variacao = 0.1;
    } 
    else
    {
      
      //double v = _sdf->GetElement("variacao")->GetValueDouble();
      double v = 0.0;
      if(v <= 0.0 || v >= 1.0) {
	  ROS_INFO("A valid value of <variacao> tag can't be found within the plugin. Adopting variacao = 0.1");
	  v = 0.3;
      } 
      
      this->_variacao = v;
    }
         
   
    // Make sure the ROS node for Gazebo has already been initialized
    if (!ros::isInitialized())
      {
	ROS_FATAL_STREAM("--- ROS not initialized !!! ---" << std::endl);
	return;
      }
          
    CameraPlugin::Load(_parent, _sdf);         
    
    // copying from CameraPlugin into GazeboRosCameraUtils
    this->parentSensor_ = this->parentSensor;
    this->width_ = this->width;
    this->height_ = this->height;
    this->depth_ = this->depth;
    this->format_ = this->format;
    this->camera_ = this->camera;

    GazeboRosCameraUtils::Load(_parent, _sdf);
    
     // Configuring a default topic name
    std::string topicName = _parent->ParentName();
    topicName.replace(topicName.find_first_of(":"),topicName.size()-1,"");
    
    //ROS_INFO_STREAM("LightSensor topicName => " << topicName);
    
    // Create publisher  
    this->rosNodePtr = std::unique_ptr<ros::NodeHandle>(new ros::NodeHandle(topicName + "/light_sensor"));
    this->_sensorPublisher = this->rosNodePtr->advertise<sensor_msgs::Illuminance>("illuminance", 1);
    
  }

  // Update the controller
  void GazeboRosLight::OnNewFrame(const unsigned char *_image,
                                  unsigned int _width, 
				  unsigned int _height, 
				  unsigned int _depth,
                                  const std::string &_format
				 )
    {
    static int seq=0;
    
    //correcao de versao
    # if GAZEBO_MAJOR_VERSION >= 7
       common::Time sensor_update_time = this->parentSensor_->LastMeasurementTime();
    # else
       common::Time sensor_update_time = this->parentSensor_->GetLastMeasurementTime();
    # endif

    this->sensor_update_time_ = sensor_update_time; //this->parentSensor_->GetLastUpdateTime();

    if (!this->parentSensor->IsActive())
    {
      if ((*this->image_connect_count_) > 0)
      // do this first so there's chance for sensor to run once after activated
        this->parentSensor->SetActive(true);
    }
    else
    {
      if ((*this->image_connect_count_) > 0)
      {
        common::Time cur_time = this->world_->GetSimTime();
        if (cur_time - this->last_update_time_ >= this->update_period_)
        {
          this->PutCameraData(_image);
          this->PublishCameraInfo();
          this->last_update_time_ = cur_time;

          sensor_msgs::Illuminance msg;
          msg.header.stamp = ros::Time::now();
          msg.header.frame_id = "";
          msg.header.seq = seq;
	  
	  /*
	   * ver codigo original illuminance: 
	   * light_sensor_plugin.cpp.bkp's
           */
	  //definir o centro da imagem (considerando os blocos r, g, b)
	  int centro = (int) (_fov / 2) * 3;
	  double maximo = 0;
	  double mola = 0;
	  int qt_molas = 0;
	  
	  
	  
	  // p = 320 * (240/2 -  6/2) - 6/2
                                     //110   *  320      +         (320 - 300)/2  = 35210
	  int pixInicial = (int) ((_height - _range) / 2) * _width  + (int) ((_width - _fov)/2);
	  	  
	  //percorrer todos os pixels fov 
	  //para cada coluna
	  //_fov() eh horizontal
          //_range() eh vertical
	  for(int linha=0;linha<_range;++linha){
	      std::string strLine = "||";
	      //para cada coluna - ler a media do r, g, b
	      for(int coluna=0;coluna<_fov*3;coluna+=3){
		  //idx = p + l*w + c
	          int idx = pixInicial + (linha * _width * 3) + coluna;
		  //ler a media de iluminancia do RGB		  
		  double leitura = (_image[idx] + _image[idx+1] + _image[idx+2])/3.0;
		  std::uint8_t intLeitura = (int) leitura;
		  
		  if(intLeitura >= 100){
		      if(intLeitura == maximo) {
			  strLine +=  " + |";
		      } else {
			  strLine += std::to_string(intLeitura) + "|";
		      }
		  } else {
		    if(intLeitura >= 10){
		          strLine += " " + std::to_string(intLeitura) + "|";
		    } else {
		          strLine += "   |";
		    }
		  }
		  
		  
		  
		  //se nao temos o maximo correto
		  
		  if(leitura > maximo) {
		     
		     //atualizar o maximo
		     maximo = leitura;
		     //esquece os calculos anteriores
		     mola = 0.0;
		     qt_molas = 0;
		     //ROS_INFO_STREAM("changing maximo in fov[" << linha << "," << coluna << "] = " << leitura << std::endl);
		  } 
		  
		  double minimo = 0;
		  //se leitura dentro dos valores minimos permitidos
		  if(leitura < maximo){
		        // fator: (0.0 ... 1.0)
		        // tende a zero quando leitura tende a minimo
		        //double fator = (leitura - minimo) /(maximo - minimo);
			// forca proporcional (-1.0 ... 1.0)
			// tende a zero quando coluna tende ao centro
			double prop = (coluna + 1.5 - centro)/(_fov - centro); //1.5 de compensacao do centro, pois os pixels da coluna são de 3 em 3
			
			//mola += fator * prop;
			mola += prop;
			qt_molas++;
		  }		    
	      }
	      //ROS_INFO_STREAM(strLine << "|");
	  }
	
	  //divide by zero non passing
	  if(qt_molas == 0) qt_molas = 1;
	  
          msg.illuminance = mola/(qt_molas * 3);
               
          msg.variance = this->_variacao;

          this->_sensorPublisher.publish(msg);

          seq++;
        }
      }
    }
  }
}
