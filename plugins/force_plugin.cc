#ifndef _TESTFORCE_PLUGIN_HH_
#define _TESTFORCE_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
// #include <gazebo/math/gzmath.hh>
// #include <ignition/math/Vector3.hh>
// #include <ignition/math/Quaternion.hh>
#include <ignition/math/Pose3.hh>
// #include <math.h>
#include<cmath>
#include<geometry_msgs/Pose.h>
#include <gazebo/physics/Base.hh>
#include <gazebo/physics/Link.hh>
#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32MultiArray.h"
#include <gazebo/common/common.hh>
//#include "ros/Quaternion.h"
//#include "ros/Matrix3x3.h"
//#include "sensor_msgs/ChannelFloat32.h"

namespace gazebo
{
  /// \brief A plugin to control a Velodyne sensor.
  class TestforcePlugin : public WorldPlugin
  {
    /// \brief Constructor
    public: TestforcePlugin() {
    printf("=============================\n");
    printf("load testforce_plugin success!!!!!!!!!\n");
    printf("===========================\n");
    }

    public: virtual void Load(physics::WorldPtr _parent, sdf::ElementPtr _sdf)
    {
     this->world=_parent;
     // this->model = this->world->GetModel("testforce3");
     this->logger0_model_base = this->world->ModelByName ("logger0");
     this->logger1_model_base = this->world->ModelByName ("logger1");
     this->logger2_model_base = this->world->ModelByName ("logger2");
     this->logger3_model_base = this->world->ModelByName ("logger3");
     this->model_child = this->world->ModelByName ("payload");
     this->drone1_model = this->world->ModelByName("drone1");
     this->drone2_model = this->world->ModelByName("drone2");
     this->drone3_model = this->world->ModelByName("drone3");
     // this->model=_model;
     this->toplink=this->model_child->GetLink	(	"base_link"	);

     this->toprod0=this->logger0_model_base->GetLink	(	"logger-hat"	);
     this->toprod1=this->logger1_model_base->GetLink	(	"logger-hat"	);
     this->toprod2=this->logger2_model_base->GetLink	(	"logger-hat"	);
     this->toprod3=this->logger3_model_base->GetLink	(	"logger-hat"	);
    //UAV
     this->toprod4=this->drone1_model->GetLink	(	"iris::base_link"	);
     this->toprod5=this->drone2_model->GetLink	(	"iris::base_link"	);
     this->toprod6=this->drone3_model->GetLink	(	"iris::base_link"	);


     // Initialize ros, if it has not already bee initialized.
     if (!ros::isInitialized())
     {
       int argc = 0;
       char **argv = NULL;
       ros::init(argc, argv, "gazebo_client",
       ros::init_options::NoSigintHandler);
     }

     // Create our ROS node. This acts in a similar manner to
     // the Gazebo node
     this->rosNode.reset(new ros::NodeHandle("gazebo_client"));
     this->prevtime=this->world->SimTime();
// Create a named topic, and subscribe to it.
//++++++++++++++++++++++++++++++++++++++++++
    ros::SubscribeOptions so0 =
    ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
      "/" + this->logger0_model_base->GetName() + "/testforce",
      1,
      boost::bind(&TestforcePlugin::OnRosMsg0, this, _1),
      ros::VoidPtr(), &this->rosQueue);
      this->rosSub0 = this->rosNode->subscribe(so0);
// Spin up the queue helper thread.
//  this->rosQueueThread =
//    std::thread(std::bind(&TestforcePlugin::QueueThread, this));
//    +++++++++++++++++++++++++++++++++++++++

// Create a named topic, and subscribe to it.
//###############################################
    ros::SubscribeOptions so1 =
    ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
      "/" + this->logger1_model_base->GetName() + "/testforce",
      1,
      boost::bind(&TestforcePlugin::OnRosMsg1, this, _1),
      ros::VoidPtr(), &this->rosQueue);
      this->rosSub1 = this->rosNode->subscribe(so1);
// Spin up the queue helper thread.
//  this->rosQueueThread =
//    std::thread(std::bind(&TestforcePlugin::QueueThread, this));
//    ##############################################

// Create a named topic, and subscribe to it.
//$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$
    ros::SubscribeOptions so2 =
    ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
      "/" + this->logger2_model_base->GetName() + "/testforce",
      1,
      boost::bind(&TestforcePlugin::OnRosMsg2, this, _1),
      ros::VoidPtr(), &this->rosQueue);
      this->rosSub2 = this->rosNode->subscribe(so2);
// Spin up the queue helper thread.
//  this->rosQueueThread =
//    std::thread(std::bind(&TestforcePlugin::QueueThread, this));
//    $$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$

// Create a named topic, and subscribe to it.
//&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
    ros::SubscribeOptions so3 =
    ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
      "/" + this->logger3_model_base->GetName() + "/testforce",
      1,
      boost::bind(&TestforcePlugin::OnRosMsg3, this, _1),
      ros::VoidPtr(), &this->rosQueue);
      this->rosSub3 = this->rosNode->subscribe(so3);
//&*&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&
//uav subscribe
//################################################
ros::SubscribeOptions so4 =
ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
  "/" + this->drone1_model->GetName() + "/testforce",
  1,
  boost::bind(&TestforcePlugin::OnRosMsg4, this, _1),
  ros::VoidPtr(), &this->rosQueue);
  this->rosSub4 = this->rosNode->subscribe(so4);
//################################################
ros::SubscribeOptions so5 =
ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
  "/" + this->drone2_model->GetName() + "/testforce",
  1,
  boost::bind(&TestforcePlugin::OnRosMsg5, this, _1),
  ros::VoidPtr(), &this->rosQueue);
  this->rosSub5 = this->rosNode->subscribe(so5);
//################################################
ros::SubscribeOptions so6 =
ros::SubscribeOptions::create<std_msgs::Float32MultiArray>(
  "/" + this->drone3_model->GetName() + "/testforce",
  1,
  boost::bind(&TestforcePlugin::OnRosMsg6, this, _1),
  ros::VoidPtr(), &this->rosQueue);
  this->rosSub6 = this->rosNode->subscribe(so6);


// Spin up the queue helper thread.
  this->rosQueueThread =
    std::thread(std::bind(&TestforcePlugin::QueueThread, this));
//    &&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&&

    std::cerr <<"testcycle\n";
    }

    public: void ApplyForce0(const double &_force0)
    {
      //calculate the force
      this->bot_pos0=this->toprod0->WorldCoGPose();
      this->bot_position0=this->bot_pos0.Pos();
      this->force0=this->CalForce(_force0,this->pos,this->bot_position0);
      //display the Length of force on the terminal
//      std::cerr <<"force0 Length:"<< this->force0.Length()<<'\n';
      //set the force direction and Length, apply the force to the link
      //here a time control is used to cycle the add force process

      //count is the number of loops for force application
      this->toplink->AddForceAtRelativePosition(	this->force0,this->pos);
   }
   
   public: void ApplyForce1(const double &_force1)
    {
      //calculate the force
      this->bot_pos1=this->toprod1->WorldCoGPose();
      this->bot_position1=this->bot_pos1.Pos();
      this->force1=this->CalForce(_force1,this->pos,this->bot_position1);
      //display the Length of force on the terminal
//      std::cerr <<"force1 Length:"<< this->force1.Length()<<'\n';
      //set the force direction and Length, apply the force to the link
      //here a time control is used to cycle the add force process

      //count is the number of loops for force application
      this->toplink->AddForceAtRelativePosition(	this->force1,this->pos);
   }
   public: void ApplyForce2(const double &_force2)
    {
      //calculate the force
      this->bot_pos2=this->toprod2->WorldCoGPose();
      this->bot_position2=this->bot_pos2.Pos();
      this->force2=this->CalForce(_force2,this->pos,this->bot_position2);
      //display the Length of force on the terminal
//      std::cerr <<"force2 Length:"<< this->force2.Length()<<'\n';
      //set the force direction and Length, apply the force to the link
      //here a time control is used to cycle the add force process

      //count is the number of loops for force application
      this->toplink->AddForceAtRelativePosition(	this->force2,this->pos);
   }
   
  public: void ApplyForce3(const double &_force3)
    {
      //calculate the force
      this->bot_pos3=this->toprod3->WorldCoGPose();
      this->bot_position3=this->bot_pos3.Pos();
      this->force3=this->CalForce(_force3,this->pos,this->bot_position3);
      //display the Length of force on the terminal
//      std::cerr <<"force3 Length:"<< this->force3.Length()<<'\n';
      //set the force direction and Length, apply the force to the link
      //here a time control is used to cycle the add force process

      //count is the number of loops for force application
      this->toplink->AddForceAtRelativePosition(	this->force3,this->pos);
   }

//   ###############################
//###############UAV1####################
  public: void ApplyForce4(const double &_force4, const double &_force4p)
    {
      //calculate the force
      //返回基座连接点的世界坐标系
      this->bot_pos4=this->toprod4->WorldCoGPose();
      this->bot_position4=this->bot_pos4.Pos();
      this->force4=this->CalForce(_force4,this->pos,this->bot_position4);
//      this->cableforce = _force4/2;
      this->cableForce1 = this->CalForce(_force4p,this->pos,this->bot_position4);
//      std::cerr <<"this->cableForce1:"<< this->cableForce1.Length()<<'\n';
      //display the Length of force on the terminal
//      std::cerr <<"force_drone1 Length:"<< this->force4.Length()<<'\n';
      //set the force direction and Length, apply the force to the link
      //here a time control is used to cycle the add force process
      //count is the number of loops for force application
      //末端执行器所受的拉力
      this->toplink->AddForceAtRelativePosition(	this->cableForce1,this->pos);
      //无人机所受到的绳子拉力
      this->toprod4->AddForceAtRelativePosition(	-1.0*this->force4,this->pos_drone1);
   }
//   ###################################################################
//   ###############################
//###############UAV2####################
  public: void ApplyForce5(const double &_force5,const double &_force5p)
    {
      //calculate the force
      //返回基座连接点的世界坐标系
      this->bot_pos5=this->toprod5->WorldCoGPose();
      this->bot_position5=this->bot_pos5.Pos();

      this->force5=this->CalForce(_force5,this->pos,this->bot_position5);

      this->cableForce2 =this->CalForce(_force5p,this->pos,this->bot_position5);

      //display the Length of force on the terminal
//      std::cerr <<"force_drone1 Length:"<< this->force5.Length()<<'\n';
      //set the force direction and Length, apply the force to the link
      //here a time control is used to cycle the add force process
      //count is the number of loops for force application
      //末端执行器所受的拉力
      this->toplink->AddForceAtRelativePosition(	this->cableForce2,this->pos);
      //无人机所受到的绳子拉力
      this->toprod5->AddForceAtRelativePosition(	-1.0*this->force5,this->pos_drone2);
   }
//   ###################################################################
//   ###############################
//###############UAV3####################
  public: void ApplyForce6(const double &_force6, const double &_force6p)
    {
      //calculate the force
      //返回基座连接点的世界坐标系
      this->bot_pos6=this->toprod6->WorldCoGPose();
      this->bot_position6=this->bot_pos6.Pos();

      this->force6=this->CalForce(_force6,this->pos,this->bot_position6);
      this->cableForce3 = this->CalForce(_force6p,this->pos,this->bot_position6);
      //display the Length of force on the terminal
//      std::cerr <<"force_drone1 Length:"<< this->force6.Length()<<'\n';
      //set the force direction and Length, apply the force to the link
      //here a time control is used to cycle the add force process
      //count is the number of loops for force application
      //末端执行器所受的拉力
      this->toplink->AddForceAtRelativePosition(	this->cableForce3,this->pos);
      //无人机所受到的绳子拉力
      this->toprod6->AddForceAtRelativePosition(	-1.0*this->force6,this->pos_drone3);
   }
//   ###################################################################
     //std::cerr << "Force applied\n";
  public: ignition::math::Vector3d CalForce(const double &_force, const ignition::math::Vector3d &pos, const ignition::math::Vector3d &bot_pos)
  {
    //Calculate the force vector according to the pose of link
    //Get the absolute position of
    this->toplinkpose=this->toplink->WorldCoGPose();
    this->toplinkposition=this->toplinkpose.Pos();
//    this->toplinkattitude=this->toplinkpose.Rot();
//    std::cerr <<"toplink pos:"<< this->toplinkposition<<'\n';
//    std::cerr <<"bottlem pos:"<< bot_pos<<'\n';
//    std::cerr <<"base_link rot:"<< this->toplinkattitude<<'\n';
    this->force_dir=bot_pos- this->toplinkposition  ;
//     std::cerr <<"distance :"<< this->distance<<'\n';
//    this->force_dir=this->test  ;
//     std::cerr <<"force direction:"<< this->force_dir.Normalize()<<'\n';
    return _force * this->force_dir.Normalize();

  }

    /// \brief Handle an incoming message from ROS

public: void OnRosMsg0(const std_msgs::Float32MultiArrayConstPtr &_msg)
{

    this->ApplyForce0(_msg->data[0]);
    this->prevtime=this->world->SimTime();
//    std::cerr <<"force_logger0: "<<_msg->data[0]<<'\n'<<'\n';
    std::cerr <<"logger0: "<<_msg->data[1]<<'\n';


}
public: void OnRosMsg1(const std_msgs::Float32MultiArrayConstPtr &_msg)
{

    this->ApplyForce1(_msg->data[0]);
    this->prevtime=this->world->SimTime();
//    std::cerr <<"force_logger1:  "<<_msg->data[0]<<'\n'<<'\n';
    std::cerr <<"logger1: "<<_msg->data[1]<<'\n';


}
public: void OnRosMsg2(const std_msgs::Float32MultiArrayConstPtr &_msg)
{

    this->ApplyForce2(_msg->data[0]);
    this->prevtime=this->world->SimTime();
//    std::cerr <<"force_logger2:  "<<_msg->data[0]<<'\n'<<'\n';
    std::cerr <<"logger2: "<<_msg->data[1]<<'\n';


}
public: void OnRosMsg3(const std_msgs::Float32MultiArrayConstPtr &_msg)
{
//  if (this->world->SimTime() - this->prevtime >= this->timeinterval||this->world->SimTime() - this->prevtime<0){}

    this->ApplyForce3(_msg->data[0]);
    this->prevtime=this->world->SimTime();
//    std::cerr <<"force_logger3:  "<<_msg->data[0]<<'\n'<<'\n';
    std::cerr <<"logger3: "<<_msg->data[1]<<'\n';
    std::cerr <<"============================="<<'\n';

}

//UAV1
public: void OnRosMsg4(const std_msgs::Float32MultiArrayConstPtr &_msg)
{

    double force = 0;
    double forceCable = 0;
//    double judge = _msg->data[0]-_msg->data[1];
//    if(judge >0 ){
//    force = _msg->data[2]*(judge);
//    }
//    else{
//    force = 0;
//    }

    this->prevtime=this->world->SimTime();

    //第二种
    force = _msg->data[0];
    forceCable=_msg->data[1];

    this->ApplyForce4(force,forceCable);
//    std::cerr <<"force_drone1: "<<force<<'\t'<<"forceCable: "<<_msg->data[1]<<'\n'<<'\n';

    std::cerr <<"drone1: "<<_msg->data[2]<<'\n';

}
//UAV2
public: void OnRosMsg5(const std_msgs::Float32MultiArrayConstPtr &_msg)
{

    double force = 0;
    double forceCable = 0;
//    double judge = _msg->data[0]-_msg->data[1];
//    if(judge >0 ){
//    force = _msg->data[2]*(judge);
//    }
//    else{
//    force = 0;
//    }

    this->prevtime=this->world->SimTime();

    //第二种
    force = _msg->data[0];
    forceCable=_msg->data[1];

    this->ApplyForce5(force,forceCable);
//    std::cerr <<"force_drone2: "<<force<<'\t'<<"forceCable: "<<_msg->data[1]<<'\n'<<'\n';
    std::cerr <<"drone2: "<<_msg->data[2]<<'\n';


}
//UAV3
public: void OnRosMsg6(const std_msgs::Float32MultiArrayConstPtr &_msg)
{

    double force = 0;
    double forceCable = 0;
//    double judge = _msg->data[0]-_msg->data[1];
//    if(judge >0 ){
//    force = _msg->data[2]*(judge);
//    }
//    else{
//    force = 0;
//    }

    this->prevtime=this->world->SimTime();

    //第二种
    force = _msg->data[0];
    forceCable=_msg->data[1];

    this->ApplyForce6(force,forceCable);
//    std::cerr <<"force_drone3: "<<force<<'\t'<<"forceCable: "<<_msg->data[1]<<'\n'<<'\n';
    std::cerr <<"drone3: "<<_msg->data[2]<<'\n';

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
    //set the forces and positions
  private:
    //4 cable-drag force
    //初始化力
       double fx1=0, fy1=0, fz1=0;
    //parameters of 8 cable nodes' position
       //end-effector link nodes positions in its own coordinate
       //末端执行器链接节点在其自身坐标中的位置
//       double const posx1=0.0, posy1=0.0,posz1=0.49; //圆锥半径是0.25，高度是0.5时
//       double const posx1=0.0, posy1=0.0,posz1=0.1;//圆锥半径是0.05，高度是0.1时
       double const posx1=0.0, posy1=0.0,posz1=0.0;//球半径是0.05，圆心在中心
       //无人机端链接节点在其自身坐标中的位置
       double const bot_posx1=0.0, bot_posy1=0.0,bot_posz1=0.0;
       double const bot_posx2=0.0, bot_posy2=0.0,bot_posz2=0.0;
       double const bot_posx3=0.0, bot_posy3=0.0,bot_posz3=0.0;

       double distance = 0;
       double cableforce = 0;
       double Coeff_elasticity = 5;
    //state the class of the variable used
    private:
    physics::ModelPtr logger0_model_base;
    physics::ModelPtr logger1_model_base;
    physics::ModelPtr logger2_model_base;
    physics::ModelPtr logger3_model_base;
    physics::ModelPtr drone1_model;
    physics::ModelPtr drone2_model;
    physics::ModelPtr drone3_model;

    physics::ModelPtr model_child;
    physics::WorldPtr world;
    physics::LinkPtr toplink;
    physics::LinkPtr toprod0;
    physics::LinkPtr toprod1;
    physics::LinkPtr toprod2;
    physics::LinkPtr toprod3;
    //UAV
    physics::LinkPtr toprod4;
    physics::LinkPtr toprod5;
    physics::LinkPtr toprod6;
    //four forces on the cable
    ignition::math::Vector3d force0=ignition::math::Vector3d(fx1,fy1,fz1);
    ignition::math::Vector3d force1=ignition::math::Vector3d(fx1,fy1,fz1);
    ignition::math::Vector3d force2=ignition::math::Vector3d(fx1,fy1,fz1);
    ignition::math::Vector3d force3=ignition::math::Vector3d(fx1,fy1,fz1);
    //UAV
    ignition::math::Vector3d force4=ignition::math::Vector3d(fx1,fy1,fz1);
    ignition::math::Vector3d force5=ignition::math::Vector3d(fx1,fy1,fz1);
    ignition::math::Vector3d force6=ignition::math::Vector3d(fx1,fy1,fz1);

    //重物绳子一端
    ignition::math::Vector3d cableForce1=ignition::math::Vector3d(fx1,fy1,fz1);
    ignition::math::Vector3d cableForce2=ignition::math::Vector3d(fx1,fy1,fz1);
    ignition::math::Vector3d cableForce3=ignition::math::Vector3d(fx1,fy1,fz1);

    //relative position of nodes on the toplink
    //末端执行器的相对于自身的位置
    ignition::math::Vector3d const pos= ignition::math::Vector3d(posx1,posy1,posz1);
    //无人机的绳子接触点相对于自身坐标系的位置
    ignition::math::Vector3d const pos_drone1= ignition::math::Vector3d(bot_posx1,bot_posy1,bot_posz1);
    ignition::math::Vector3d const pos_drone2= ignition::math::Vector3d(bot_posx2,bot_posy2,bot_posz2);
    ignition::math::Vector3d const pos_drone3= ignition::math::Vector3d(bot_posx3,bot_posy3,bot_posz3);

    ignition::math::Vector3d const test= ignition::math::Vector3d(0,0,1);

    //absolute position of nodes on the bottom link
//    ignition::math::Vector3d const bot_pos0= ignition::math::Vector3d(bot_posx1,bot_posy1,bot_posz1);

    //temporaty variable for force calculation
    ignition::math::Vector3d force_dir;
    ignition::math::Pose3d toplinkpose;
    ignition::math::Pose3d bot_pos0;
    ignition::math::Pose3d bot_pos1;
    ignition::math::Pose3d bot_pos2;
    ignition::math::Pose3d bot_pos3;
    ignition::math::Vector3d bot_position0;
    ignition::math::Vector3d bot_position1;
    ignition::math::Vector3d bot_position2;
    ignition::math::Vector3d bot_position3;
    //UAV
    ignition::math::Pose3d bot_pos4;
    ignition::math::Pose3d bot_pos5;
    ignition::math::Pose3d bot_pos6;
    ignition::math::Vector3d bot_position4;
    ignition::math::Vector3d bot_position5;
    ignition::math::Vector3d bot_position6;


    ignition::math::Vector3d toplinkposition;
//    ignition::math::Quaterniond toplinkattitude;

    common::Time timeinterval=common::Time(0, common::Time::SecToNano(0.001));
    common::Time prevtime;

    /// \brief A node use for ROS transport
private: std::unique_ptr<ros::NodeHandle> rosNode;

/// \brief A ROS subscriber
private: ros::Subscriber rosSub0;
private: ros::Subscriber rosSub1;
private: ros::Subscriber rosSub2;
private: ros::Subscriber rosSub3;
//uav
private: ros::Subscriber rosSub4;
private: ros::Subscriber rosSub5;
private: ros::Subscriber rosSub6;

/// \brief A ROS callbackqueue that helps process messages
private: ros::CallbackQueue rosQueue;

/// \brief A thread the keeps running the rosQueue
private: std::thread rosQueueThread;

  };

  // Tell Gazebo about this plugin, so that Gazebo can call Load on this plugin.
  GZ_REGISTER_WORLD_PLUGIN(TestforcePlugin)
}
#endif