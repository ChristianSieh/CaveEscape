#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <geometry_msgs/Pose2D.h>
#include <iostream>
#include <map>
#include <time.h>
#include <random>


// These need to be pulled out to parameters...
const float WHEEL_RAD = 0.05; // meters
const float WHEELBASE = 0.466725; // meters
const float TRACK = 0.28; // meters

namespace gazebo
{
  class DiffDrive : public ModelPlugin
  {
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
      // Store the pointer to the model
      this->model = _parent;
      this->mjointRight = this->model->GetJoint("right_joint");
      this->mjointLeft = this->model->GetJoint("left_joint");
      this->mGpsNoise = 0.0;

      mlastTime = this->model->GetWorld()->GetSimTime().Double();

      m_p = 0.0;
      mleft = 0;
      mright = 0;

      mRosnode.reset(new ros::NodeHandle(""));

      m_traj_sub = mRosnode->subscribe("cmd_joint_traj",1, &DiffDrive::callback, this);
      m_gps_pub = mRosnode->advertise<geometry_msgs::Pose2D>("gps", 100);

      double p = 0;
      double i = 0;
      double d = 0;
      double maxi = 0;

      ros::param::get("/gps_noise",this->mGpsNoise);
      ros::param::get("/pval",p);
      ros::param::get("/ival",i);
      ros::param::get("/dval",d);
      ros::param::get("/maxforce",this->mmaxforce);
      ros::param::get("/maxi",maxi);
      this->mleftpid.Init(p,i,d,-maxi,maxi,this->mmaxforce,-this->mmaxforce);
      this->mrightpid.Init(p,i,d,-maxi,maxi,this->mmaxforce,-this->mmaxforce);

      // Listen to the update event. This event is broadcast every
      // simulation iteration.
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
        boost::bind(&DiffDrive::UpdateChild, this, _1));

        if(this->mjointRight != NULL && this->mjointLeft != NULL)
        {
          // this->mjointRight->SetMaxForce(0,99);
          // this->mjointLeft->SetMaxForce(0,99);
        }

      }

      public: void UpdateChild(const common::UpdateInfo & /*_info*/)
      {
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<> d(0,this->mGpsNoise);

        math::Pose pose = this->model->GetWorldPose();
        float yaw = pose.rot.GetYaw();
        geometry_msgs::Pose2D msg;

        msg.x = pose.pos[0] + d(gen);
        msg.y = pose.pos[1] + d(gen);
        msg.theta = yaw + d(gen);
        this->m_gps_pub.publish(msg);



        double simtime = this->model->GetWorld()->GetSimTime().Double();
        double dt = simtime - this->mlastTime;
        this->mlastTime = simtime;

        double vel = this->mjointRight->GetVelocity(0);
        double velTarget = mright;
        double err = velTarget - vel;
        this->mrightpid.Update(err,dt);
        vel = this->mjointLeft->GetVelocity(0);
        velTarget = mleft;
        err = velTarget - vel;
        this->mleftpid.Update(err,dt);

        // std::cout << vel << "\t" << velTarget << "\t" << err << "\t" <<  this->mleftpid.GetCmd() << std::endl;
        if(this->mjointRight != NULL && this->mjointLeft != NULL)
        {
          double cmd = this->mrightpid.GetCmd();
          this->mjointRight->SetForce(0,cmd);

          cmd = this->mleftpid.GetCmd();
          this->mjointLeft->SetForce(0,cmd);
          //
          // this->mjointRight->SetVelocity(0,mright);
          // this->mjointLeft->SetVelocity(0,mleft);

          // double lastr = this->mjointRight->GetPosition(0);
          // double lastl = this->mjointLeft->GetPosition(0);
          // this->mjointRight->SetPosition(0,2);
          // this->mjointRight->SetPosition(0,2);

          // this->model->GetJointController()->SetVelocityTarget(this->mjointRight->GetName(),0);

        }
      }

        // Will pretty this up later...
        public: void callback(const trajectory_msgs::JointTrajectory::ConstPtr & cmd_msg)
        {
          std::map<std::string, double> commands;
          for (int i = 0 ; i < cmd_msg->joint_names.size(); i++)
          {
            commands[cmd_msg->joint_names[i]] = cmd_msg->points[0].velocities[i];
          }

          if(commands.find("left")!=commands.end())
          {
            mleft = commands["left"];
          }

          if(commands.find("right")!=commands.end())
          {
            mright = commands["right"];
          }

        }


      private:
        void stop()
        {
          mleft = 0;
          mright = 0;
        }

        // Pointer to the model
        physics::ModelPtr model;

        // Pointer to the update event connection
        event::ConnectionPtr updateConnection;

        boost::shared_ptr<ros::NodeHandle> mRosnode;

        physics::JointPtr mjointRight;
        physics::JointPtr mjointLeft;

        float mGpsNoise;

        float mleft;
        float mright;

        double m_p;
        double mlastTime;
        double mmaxforce;

        ros::Subscriber m_traj_sub;
        ros::Publisher m_gps_pub;

        common::PID mleftpid;
        common::PID mrightpid;
      };

      // Register this plugin with the simulator
      GZ_REGISTER_MODEL_PLUGIN(DiffDrive)
    }
