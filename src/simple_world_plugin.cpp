#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <ignition/math/Vector3.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/Model.hh>
#include <sdf/sdf.hh>
#include <gazebo/common/Plugin.hh>
#include <ros/ros.h>
#include <stdio.h>

namespace gazebo
{   
  class HelloWorld : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) 
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          boost::bind(&HelloWorld::OnUpdate, this));

      this->model->SetLinearVel(ignition::math::Vector3<double>(-4.0, 0.0, 4.0));
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      // Apply a small linear velocity to the model.
      // this->model->SetLinearVel(ignition::math::Vector3<double>(1.0, 0, 0));
    }

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(HelloWorld)
}


// #include <gazebo/common/Plugin.hh>
// #include <ros/ros.h>

// namespace gazebo
// {
// class WorldPluginTutorial : public WorldPlugin
// {
// public:
//   WorldPluginTutorial() : WorldPlugin()
//   {
//   }

//   void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
//   {
//     // Make sure the ROS node for Gazebo has already been initialized
//     if (!ros::isInitialized())
//     {
//       ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
//         << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
//       return;
//     }

//     ROS_WARN("Hello World!");
//   }

// };
// GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
// }


