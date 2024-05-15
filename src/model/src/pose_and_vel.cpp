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
        public : void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            //Pointer to model
            this->model = _parent;

            //listen to the update event
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&ModelJointControler::OnUpdate, this)
            );

            this->old_secs = ros::Time::now().toSec();

            if (_sdf->HasElement("kp"))
                this->kp = _sdf->Get<double>("kp");
            if (_sdf->HasElement("ki"))
                this->ki = _sdf->Get<double>("ki");
            if (_sdf->HasElement("kd"))
                this->kd = _sdf->Get<double>("kd");


            // if (_sdf->HasElement("wheel_kp"))
            //     this->kp = _sdf->Get<double>("wheel_kp");
            // if (_sdf->HasElement("wheel_ki"))
            //     this->ki = _sdf->Get<double>("wheel_ki");
            // if (_sdf->HasElement("wheel_kd"))
            //     this->kd = _sdf->Get<double>("wheel_kd");



            if (_sdf->HasElement("namespace_model"))
                this->namespace_model = _sdf->Get<std::string>("namespace_model");
            if (_sdf->HasElement("activate_pid_control"))
                this->activate_pid_control = (_sdf->Get<std::string>("activate_pid_control") == "yes");

            //topic name
            std::string joint_pose_1 = "/"+this->namespace_model + "/joint_pose_1";
            std::string joint_pose_2 = "/"+this->namespace_model + "/joint_pose_2";

            // std::string left_wheel_vel = "/"+this->namespace_model + "/left_wheel_vel";
            // std::string right_wheel_vel = "/"+this->namespace_model + "/right_wheel_vel";
           

            //Initialize ros
            if (!ros::isInitialized())
            {
                int argc = 0;
                char **argv = NULL;
                ros::init(argc, argv, "joint_rosnode", ros::init_options::NoSigintHandler);
            }


        // if(this->activate_pid_control)
        // {

            //create our rosNode, acts to similar to the Gazebo NOde
            this->rosNode.reset(new ros::NodeHandle("pose_rosnode"));

            //Activate PID control

            const auto &jointController = this->model->GetJointController();
            jointController->Reset();

            jointController->AddJoint(model->GetJoint("2dof_trial1::joint1"));
            jointController->AddJoint(model->GetJoint("2dof_trial1::joint2"));            


            // jointController->AddJoint(model->GetJoint("back_right"));
            // jointController->AddJoint(model->GetJoint("back_left"));   
            // jointController->AddJoint(model->GetJoint("back_right"));
            // jointController->AddJoint(model->GetJoint("back_left"));                      

            this->joint_name1 = model->GetJoint("2dof_trial1::joint1")->GetScopedName();
            this->joint_name2 = model->GetJoint("2dof_trial1::joint2")->GetScopedName();

            // this->right_front_wheel_name = model->GetJoint("back_right")->GetScopedName();
            // this->left_front_wheel_name = model->GetJoint("back_left")->GetScopedName();
            // this->right_back_wheel_name = model->GetJoint("back_right")->GetScopedName();
            // this->left_back_wheel_name = model->GetJoint("back_left")->GetScopedName();

            jointController->SetPositionPID(this->joint_name1, common::PID(this->kp, this->ki, this->kd));
            jointController->SetPositionPID(this->joint_name2, common::PID(this->kp, this->ki, this->kd));


            jointController->SetPositionTarget(this->joint_name1, 0.0);
            jointController->SetPositionTarget(this->joint_name2, 0.0);



            // jointController->SetVelocityPID(this->right_front_wheel_name, common::PID(this->wheel_kp, this->wheel_ki, this->wheel_kd));
            // jointController->SetVelocityPID(this->left_front_wheel_name, common::PID(this->wheel_kp, this->wheel_ki, this->wheel_kd));
            // jointController->SetVelocityPID(this->right_back_wheel_name, common::PID(this->wheel_kp, this->wheel_ki, this->wheel_kd));
            // jointController->SetVelocityPID(this->left_back_wheel_name, common::PID(this->wheel_kp, this->wheel_ki, this->wheel_kd));



            // jointController->SetVelocityTarget(this->right_front_wheel_name, 0.0);
            // jointController->SetVelocityTarget(this->left_front_wheel_name, 0.0);
            // jointController->SetVelocityTarget(this->right_back_wheel_name, 0.0);
            // jointController->SetVelocityTarget(this->left_back_wheel_name, 0.0);





        // }

            ros::SubscribeOptions sub_joint1 = 
                ros::SubscribeOptions::create<std_msgs::Float32>(
                    joint_pose_1,
                    1,
                    boost::bind(&ModelJointControler::OnRosMsg_joint_pose_1, this, _1),
                    ros::VoidPtr(), &this->rosQueue1
                );
            this->sub_joint1 = this->rosNode->subscribe(sub_joint1);

            this->rosQueueThread1 = std::thread(std::bind(&ModelJointControler::QueueThread1, this));


            ros::SubscribeOptions sub_joint2 = 
                ros::SubscribeOptions::create<std_msgs::Float32>(
                    joint_pose_2,
                    1,
                    boost::bind(&ModelJointControler::OnRosMsg_joint_pose_2, this, _1),
                    ros::VoidPtr(), &this->rosQueue2
                );
            this->sub_joint2 = this->rosNode->subscribe(sub_joint2);

            this->rosQueueThread2 = std::thread(std::bind(&ModelJointControler::QueueThread2, this));




            // ros::SubscribeOptions right_wheel = 
            //     ros::SubscribeOptions::create<std_msgs::Float32>(
            //         right_wheel_vel,
            //         1,
            //         boost::bind(&ModelJointControler::OnRosMsg_right_wheel_vel, this, _1),
            //         ros::VoidPtr(), &this->rosQueue3
            //     );

            // this->right_wheel = this->rosNode->subscribe(right_wheel);
            // this->rosQueueThread3 = std::thread(std::bind(&ModelJointControler::QueueThread3, this));




            // ros::SubscribeOptions left_wheel = 
            //     ros::SubscribeOptions::create<std_msgs::Float32>(
            //         left_wheel_vel,
            //         1,
            //         boost::bind(&ModelJointControler::OnRosMsg_left_wheel_vel, this, _1),
            //         ros::VoidPtr(), &this->rosQueue4
            //     );

            // this->left_wheel = this->rosNode->subscribe(left_wheel);
            // this->rosQueueThread4 = std::thread(std::bind(&ModelJointControler::QueueThread4, this));

               
            // ROS_WARN("Loaded Plugin with parent...%s", this->model->GetName().c_str());







        }

        // Called by world update start event

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

            if(this->activate_pid_control){
                ROS_DEBUG("Update Wheel Speed PID...");
                const auto &jointController = this->model->GetJointController();
                    

                jointController->SetPositionTarget(this->joint_name1, this->joint_pose1);
                jointController->SetPositionTarget(this->joint_name2, this->joint_pose2);

                // jointController->SetVelocityTarget(this->right_front_wheel_name, this->right_vel);
                // jointController->SetVelocityTarget(this->left_front_wheel_name, this->left_vel);
                // jointController->SetVelocityTarget(this->right_back_wheel_name, this->right_vel);
                // jointController->SetVelocityTarget(this->left_back_wheel_name, this->left_vel);

            }     
        }

        }


        public: void SetJointPose1(const double &_magn)
        {
            this->joint_pose1 = _magn;
            ROS_WARN("Joint 1 pose >> %f", this->joint_pose1);
        }

        public: void OnRosMsg_joint_pose_1(const std_msgs::Float32ConstPtr &_msg)
        {
            this->SetJointPose1(_msg->data);
        }


        public: void SetJointPose2(const double &_magn2)
        {
            this->joint_pose2 = _magn2;
            ROS_WARN("Joint 2 pose >> %f", this->joint_pose2);
        }

        public: void OnRosMsg_joint_pose_2(const std_msgs::Float32ConstPtr &_msg)
        {
            this->SetJointPose2(_msg->data);
        }


        // brief ROS helper that process messages
        private: void QueueThread1()
        {
            static const double timeout = 0.01;
            while(this->rosNode->ok())
            {
                this->rosQueue1.callAvailable(ros::WallDuration(timeout));
            }
        }

        private: void QueueThread2()
        {
            static const double timeout = 0.01;
            while(this->rosNode->ok())
            {
                this->rosQueue2.callAvailable(ros::WallDuration(timeout));
            }
        }


        // public: void SetRightVel(const double &_magn3)
        // {
        //     this->right_vel = _magn3;
        //     ROS_WARN("Right wheel velocity >> %f", right_vel);
        // }

        // public: void OnRosMsg_right_wheel_vel(const std_msgs::Float32ConstPtr &_msg)
        // {
        //     this->SetRightVel(_msg->data);
        // }


        // // brief ROS helper that process messages
        // private: void QueueThread3()
        // {
        //     static const double timeout = 0.01;
        //     while(this->rosNode->ok())
        //     {
        //         this->rosQueue3.callAvailable(ros::WallDuration(timeout));
        //     }
        // }


        // public: void SetLeftVel(const double &_magn4)
        // {
        //     this->left_vel = _magn4;
        //     ROS_WARN("Left wheel velocity >> %f", left_vel);
        // }

        // public: void OnRosMsg_left_wheel_vel(const std_msgs::Float32ConstPtr &_msg)
        // {
        //     this->SetLeftVel(_msg->data);
        // }


        // // brief ROS helper that process messages
        // private: void QueueThread4()
        // {
        //     static const double timeout = 0.01;
        //     while(this->rosNode->ok())
        //     {
        //         this->rosQueue4.callAvailable(ros::WallDuration(timeout));
        //     }
        // }

        













    // Pointer to the model
    private: physics::ModelPtr model;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    
    // Time Memory
    double old_secs;    

    // Frequency of earthquake
    double freq_update = 10.0;


    double joint_pose1 = 0.0;
    double joint_pose2 = 0.0;
    // double right_vel = 0.0;
    // double left_vel = 0.0;

 /// \brief A node use for ROS transport
    private: std::unique_ptr<ros::NodeHandle> rosNode;
    
    /// \brief A ROS subscriber
    private: ros::Subscriber sub_joint1;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue1;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread1;
    
    
    /// \brief A ROS subscriber
    private: ros::Subscriber sub_joint2;
    /// \brief A ROS callbackqueue that helps process messages
    private: ros::CallbackQueue rosQueue2;
    /// \brief A thread the keeps running the rosQueue
    private: std::thread rosQueueThread2;

    // /// \brief A ROS subscriber
    // private: ros::Subscriber right_wheel;
    // /// \brief A ROS callbackqueue that helps process messages
    // private: ros::CallbackQueue rosQueue3;
    // /// \brief A thread the keeps running the rosQueue
    // private: std::thread rosQueueThread3;


    // /// \brief A ROS subscriber
    // private: ros::Subscriber left_wheel;
    // /// \brief A ROS callbackqueue that helps process messages
    // private: ros::CallbackQueue rosQueue4;
    // /// \brief A thread the keeps running the rosQueue
    // private: std::thread rosQueueThread4;











    std::string joint_name1;
    std::string joint_name2;
    // std::string right_front_wheel_name;
    // std::string left_front_wheel_name;
    // std::string right_back_wheel_name;
    // std::string left_back_wheel_name;


    std::string namespace_model = "";
    bool activate_pid_control;

    // double kp = 1.0;
    // double ki = 0.0;
    // double kd = 0.0;

    // double wheel_kp = 1.0;
    // double wheel_ki = 0.0;
    // double wheel_kd = 0.0;


    // double wheel_kp ;
    // double wheel_ki ;
    // double wheel_kd ;


    double kp;
    double ki;
    double kd ;
  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(ModelJointControler)
}
