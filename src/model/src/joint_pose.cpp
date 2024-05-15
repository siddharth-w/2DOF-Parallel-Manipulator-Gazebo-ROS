#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

namespace gazebo
{
  class ModelJointControler : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
    {
      // Store the pointer to the model
      this->model = _parent;

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&ModelJointControler::OnUpdate, this));
      
      this->old_secs =ros::Time::now().toSec();



      if (_sdf->HasElement("kp"))
          this->kp = _sdf->Get<double>("kp");
      if (_sdf->HasElement("ki"))
          this->ki = _sdf->Get<double>("ki");
      if (_sdf->HasElement("kd"))
          this->kd = _sdf->Get<double>("kd");
      if (_sdf->HasElement("namespace_model"))
          this->namespace_model = _sdf->Get<std::string>("namespace_model");
      if (_sdf->HasElement("activate_pid_control"))
          this->activate_pid_control = (_sdf->Get<std::string>("activate_pid_control") == "yes");


      // Create a topic name


      // std::string left_wheel_pos = "/"+this->namespace_model + "/left_wheel_pos";
      // std::string right_wheel_pos = "/"+this->namespace_model + "/right_wheel_pos";      
      std::string joint_pose = "/"+this->namespace_model + "/joint_pose";
      std::string joint_pose1 = "/"+this->namespace_model + "/joint_pose1";



      // Initialize ros, if it has not already bee initialized.
      if (!ros::isInitialized())
      {
        int argc = 0;
        char **argv = NULL;
        ros::init(argc, argv, "joint_rosnode",
            ros::init_options::NoSigintHandler);
      }
         
      // Create our ROS node. This acts in a similar manner to
      // the Gazebo node
      this->rosNode.reset(new ros::NodeHandle("pose_rosnode"));
      

      if(this->activate_pid_control)
      {
          // Activated PID Speed Control
	  const auto &jointController = this->model->GetJointController();
          jointController->Reset();

          // jointController->AddJoint(model->GetJoint("right_wheel_joint"));
          // jointController->AddJoint(model->GetJoint("left_wheel_joint"));

          jointController->AddJoint(model->GetJoint("2dof_trial1::joint1"));

          jointController->AddJoint(model->GetJoint("2dof_trial1::joint2"));

          // this->right_wheel_name = model->GetJoint("right_wheel_joint")->GetScopedName();
          // this->left_wheel_name = model->GetJoint("left_wheel_joint")->GetScopedName();
          this->joint_name1 = model->GetJoint("2dof_trial1::joint1")->GetScopedName();
          this->joint_name2 = model->GetJoint("2dof_trial1::joint2")->GetScopedName();


          // jointController->SetVelocityPID(this->right_wheel_name, common::PID(this->kp, this->ki, this->kd));
          // jointController->SetVelocityPID(this->left_wheel_name, common::PID(this->kp, this->ki, this->kd));

          // jointController->SetVelocityPID(this->left_wheel_name, common::PID(this->kp, this->ki, this->kd));

          jointController->SetPositionPID(this->joint_name1, common::PID(this->kp, this->ki, this->kd));
          jointController->SetPositionPID(this->joint_name2, common::PID(this->kp, this->ki, this->kd));



          // jointController->SetVelocityTarget(this->right_wheel_name, 0.0);
          // jointController->SetVelocityTarget(this->left_wheel_name, 0.0);

          // jointController->SetVelocityTarget(this->joint_name, 0.0);
          jointController->SetPositionTarget(this->joint_name1, 0.0);
          jointController->SetPositionTarget(this->joint_name2, 0.0);

      }



      ros::SubscribeOptions so =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            joint_pose,
            1,
            boost::bind(&ModelJointControler::OnRosMsg_joint_pose, this, _1),
            ros::VoidPtr(), &this->rosQueue);
      this->rosSub = this->rosNode->subscribe(so);
      
      // Spin up the queue helper thread.
      this->rosQueueThread =
        std::thread(std::bind(&ModelJointControler::QueueThread, this));                
        


      ros::SubscribeOptions so1 =
        ros::SubscribeOptions::create<std_msgs::Float32>(
            joint_pose1,
            1,
            boost::bind(&ModelJointControler::OnRosMsg_joint_pose1, this, _1),
            ros::VoidPtr(), &this->rosQueue1);
      this->rosSub1 = this->rosNode->subscribe(so1);
      
      // Spin up the queue helper thread.
      this->rosQueueThread1 =
        std::thread(std::bind(&ModelJointControler::QueueThread1, this));                
        




      ROS_WARN("Loaded Plugin with parent...%s", this->model->GetName().c_str());
      
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      double new_secs =ros::Time::now().toSec();
      double delta = new_secs - this->old_secs;
      
      double max_delta = 0.0;
      
      if (this->freq_update != 0.0)
      {
        max_delta = 1.0 / this->freq_update;
      }
      
      if (delta > max_delta && delta != 0.0)
      {
        this->old_secs = new_secs;

	if(this->activate_pid_control)
        {
          ROS_DEBUG("Update Wheel Speed PID...");
	  const auto &jointController = this->model->GetJointController();
    // jointController->SetPositionTarget(this->right_wheel_name, this->right_wheel_pose);
    // jointController->SetPositionTarget(this->left_wheel_name, this->left_wheel_pose);

    // jointController->SetVelocityTarget(this->joint_name, this->joint_pose);
    jointController->SetPositionTarget(this->joint_name1, this->joint_pose);
    jointController->SetPositionTarget(this->joint_name2, this->joint_pose1);


	  // jointController->SetVelocityTarget(this->right_wheel_name, this->right_wheel_pose);
    //       jointController->SetVelocityTarget(this->left_wheel_name, this->left_wheel_pose);
        }else
        {
            // Apply a small linear velocity to the model.
            ROS_DEBUG("Update Wheel Speed BASIC...");
      	    // this->model->GetJoint("right_wheel_joint")->SetVelocity(0, this->right_wheel_pose);
            // this->model->GetJoint("left_wheel_joint")->SetVelocity(0, this->left_wheel_pose);

      	    // this->model->GetJoint("joint")->SetVelocity(0, this->joint_pose);  
      	    this->model->GetJoint("joint1")->SetPosition(0, this->joint_pose);          
      	    this->model->GetJoint("joint2")->SetPosition(0, this->joint_pose1);          

        }

      }

    }
    
    

    
    public: void SetJointPose(const double &_magn)
    {
      this->joint_pose = _magn;
      ROS_WARN("joint_pose >> %f", this->joint_pose);
    }

    public: void SetJointPose1(const double &_magn)
    {
      this->joint_pose1 = _magn;
      ROS_WARN("joint_pose >> %f", this->joint_pose1);
    }
     

 
 
    public: void OnRosMsg_joint_pose(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetJointPose(_msg->data);
    }



    public: void OnRosMsg_joint_pose1(const std_msgs::Float32ConstPtr &_msg)
    {
      this->SetJointPose1(_msg->data);
    }


    
    /// \brief ROS helper function that processes messages
    private: void QueueThread()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue.callAvailable(ros::WallDuration(timeout));
      }
    }
    
    /// \brief ROS helper function that processes messages
    private: void QueueThread1()
    {
      static const double timeout = 0.01;
      while (this->rosNode->ok())
      {
        this->rosQueue1.callAvailable(ros::WallDuration(timeout));
      }
    }
    

    

    

    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    // Time Memory
    double old_secs;
    
    // Frequency of earthquake
    double freq_update = 10.0;

    // double left_wheel_pose = 0.0;
    // Magnitude of the Oscilations
    // double right_wheel_pose = 0.0;


    double joint_pose = 0.0;
    double joint_pose1 = 0.0;
    double joint_pose2 = 0.0;


    
    
    /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread;
    
    
    /// \brief A ROS subscriber
    private: ros::Subscriber rosSub1;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue1;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread1;
    
    // std::string right_wheel_name;
    // std::string left_wheel_name;
    std::string joint_name;
    std::string joint_name1;
    std::string joint_name2;

    std::string namespace_model = "";
    bool activate_pid_control;

    double kp = 1.0;
    double ki = 0.0;
    double kd = 0.0;

    
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelJointControler)
}
