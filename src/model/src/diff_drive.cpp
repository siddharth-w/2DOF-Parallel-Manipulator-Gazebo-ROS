// Basic C++ Headers
#include <boost/bind.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <thread>

// These header files can be found here : /usr/include/gazebo-<VERSION>/gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// ROS messages header files
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <nav_msgs/Odometry.h>
#include <rosgraph_msgs/Clock.h>
#include <gazebo_msgs/ModelStates.h>

// ROS heaaders
#include <ros/ros.h>
#include <ros/time.h>
#include <ros/callback_queue.h>
#include <ros/subscribe_options.h>

#define L 0.355 // distance between body center and wheel center
#define r 0.035 // wheel radius

#define pi 3.14159265

namespace gazebo {

    class DiffDrivePlugin : public ModelPlugin {	// Inherits the ModelPlugin Class. Various types of Plugins can be found here: /usr/gazebo-<VERSION>/gazebo/common/Plugins.hh

        private: std::unique_ptr<ros::NodeHandle> rosNode;

        private: ros::Subscriber rosSub;  // Subscriber Variable

        private: ros::Publisher rosPub;   // Publisher Variable

        private: ros::CallbackQueue rosQueue; //Queue for Subscriber Callbacks

        private: std::thread rosQueueThread;  // Separate thread variable for executing queue

        private: physics::WorldPtr world;     // Pointer to the Gazebo World

        private: physics::ModelPtr model;     // Pointer to the model spawned in Gazebo

        private: physics::JointPtr LeftWheel;   // Pointer to individual joints in a particular model within a world
        private: physics::JointPtr RightWheel;

        // Pointer to the update event connection
        private: event::ConnectionPtr updateConnection;

        private: void QueueThread()
        {
            static const double timeout = 0.01;
            while (this->rosNode->ok())
            {
                this->rosQueue.callAvailable(ros::WallDuration(timeout));
            }
        }

        // This function extracts and assigns a Pointer to each joint in the model
        // The model pointer is  used to access various elements in it. All functions can be found here: /usr/gazebo-<VERSION>/gazebo/physics/Model.hh
        public: void GetJointsFromModel()
        {
            ROS_INFO("Extracting joints from the model .......");
            unsigned int c = this->model->GetJointCount();
            this->LeftWheel = this->model->GetJoint("left_front");		// The param passed to the function should be same as the name given in the sdf file.
            this->RightWheel = this->model->GetJoint("right_front");
            ROS_INFO("%d joints were successfully extracted from the model !!", c);

        }

        // This function initializes ROS node for Gazebo. If it is already initialized, it renames the node handler.
        public: void ros_init()
        {
            ROS_INFO("Initializing ROS node for Gazebo .......");
            if (!ros::isInitialized())
            {
                ROS_FATAL_STREAM("A ROS node for Gazebo has not been initialized, unable to load plugin. "
                << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
                return;
            }
            this->rosNode.reset(new ros::NodeHandle("diff_drive"));
            ROS_INFO("ROS node was initialized with node handle \"diff_drive\"");

        }

        // Function to create the required publisher and subscribers within the "diff_drive" node created earlier
        public: void sample_create_pubsub()
        {
            // ROS_INFO("Creating Subscribers");
            // ros::SubscribeOptions so = ros::SubscribeOptions::create<msg_type>("/topic", 1, boost::bind(&DiffDrivePlugin::CallbackFunction, this, _1), ros::VoidPtr(), &this->rosQueue);
            // this->rosSub = this->rosNode->subscribe(so);
            // ROS_INFO("Successfully created subscribers");

            // ROS_INFO("Creating Publishers");
            // this->rosPub = this->rosNode->advertise<nav_msgs::Odometry>("odom", 1);
            // ROS_INFO("Successfully created Publishers");

        }

        // This function executes only once when the simulation is launched. It allows to extracts all the parameters within the world and assign pointers to them for manipulation/control later on.
        public: void Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/) // Since we used a ModelPlugin class above, this function returns a pointer to the model.
        {
            ROS_INFO("The robot model is loading .........");

            this->model = _model;
            this->world = _model->GetWorld();	// Assigns the pointer to the world in which the given model is loaded.

            GetJointsFromModel();

            ros_init();

            sample_create_pubsub();

            ROS_INFO("The Differential Drive Robot Model is loaded successfully !!");

            this->rosQueueThread = std::thread(std::bind(&DiffDrivePlugin::QueueThread, this));
            // Listen to the update event. This event is broadcast every simulation iteration.
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&DiffDrivePlugin::OnUpdate, this, _1));
        }

        /* This function executes upon each iteration of the Gazebo Simulation.
           It is passed as a callback function to the updateConnection event. */
        public: void OnUpdate(const common::UpdateInfo)
        {
          // Since this function loops itself on each iteration of the simulation, using a loop within this function pauses the simulation update.
          // Hence you cannot execute any loops within this function. This function in itself is an infinite loop controlled by the simulation engine iterations.

        	// Change the velocity of each wheel and see the output
          	this->LeftWheel->SetVelocity(0, -0.5);		// Control the joints as desired using their respective pointers
          	this->RightWheel->SetVelocity(0, -0.5);		// The Joint related functions can be found here: /usr/gazebo-<VERSION>/gazebo/physics/Joint.hh
        }

    };

    GZ_REGISTER_MODEL_PLUGIN(DiffDrivePlugin)         // Registering our model the Gazebo Server
}
