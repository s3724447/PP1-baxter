/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014-2017, Dataspeed Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Dataspeed Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include <mobility_base_gazebo_plugins/MobilityBasePlugin.h>
#include <ros/ros.h>

namespace gazebo
{

MobilityBasePlugin::MobilityBasePlugin() :
    nh_(NULL), spinner_thread_(NULL), tf_broadcaster_(NULL), first_update_(true), fast_(true)
{
  ROS_INFO("MobilityBasePlugin: constructor; fast: %s", (fast_ ? "true" : "false"));
  omni_a_ = 1.0 / WHEEL_RADIUS;
  omni_b_ = 1.0 / WHEEL_RADIUS;
  omni_c_ = 0.5 * (WHEEL_BASE_WIDTH + WHEEL_BASE_LENGTH) / WHEEL_RADIUS;
  frame_id_ = "/base_footprint";
  mode_ = mobility_base_core_msgs::Mode::MODE_DISABLED;
}
MobilityBasePlugin::~MobilityBasePlugin()
{
  if (nh_) {
    delete nh_;
  }
  if (spinner_thread_) {
    delete spinner_thread_;
  }
}
void MobilityBasePlugin::FiniChild()
{
  nh_->shutdown();
  spinner_thread_->join();
}

void MobilityBasePlugin::omniFromCartesian(double vx, double vy, double wz, double w[4]) const {
    w[0] = omni_a_ * vx - omni_b_ * vy - omni_c_ * wz;
    w[1] = omni_a_ * vx + omni_b_ * vy + omni_c_ * wz;
    w[2] = omni_a_ * vx + omni_b_ * vy - omni_c_ * wz;
    w[3] = omni_a_ * vx - omni_b_ * vy + omni_c_ * wz;
}

void MobilityBasePlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
  first_update_ = true;

  // Store the pointer to the model and world
  model_ = parent;
  world_ = model_->GetWorld();
  ROS_INFO("MobilityBasePlugin: Loading the kw90 mobility base plugin");

  // Get then name of the parent model
  std::string model_name = sdf->GetParent()->Get<std::string>("name");
  std::string first_level = sdf->Get<std::string>("name");
  ROS_INFO("MobilityBasePlugin: model_name of model: %s", model_name.c_str());
  gzdbg << "MobilityBasePlugin: loaded for model '" << model_name << "'\n";

  // Get parameters
  //if (sdf->HasElement("fast")) {
  //  std::string comp = "true";
  //  fast_ = comp.compare(sdf->GetElement("fast")->Get<std::string>()) ? false : true;
  //}
  if (fast_) {
    ROS_INFO("MobilityBasePlugin: fast:=true");
  } else {
    ROS_INFO("MobilityBasePlugin: fast:=false");
  }
  if (sdf->HasElement("parent_frame_id")) {
    parent_frame_id_ = sdf->GetElement("parent_frame_id")->Get<std::string>();
    if (sdf->HasElement("child_frame_id")) {
      child_frame_id_ = sdf->GetElement("child_frame_id")->Get<std::string>();
    } else {
      child_frame_id_ = frame_id_;
    }
    tf_broadcaster_ = new tf::TransformBroadcaster();
  }

  ROS_INFO("MobilityBasePlugin: Getting pointers to baselink and pushing wheels joint states back.");

  // Get the pointer to the base_link
  link_base_footprint_ = model_->GetLink("base_footprint");

  joint_state_wheels_.name.push_back("wheel_fl");
  joint_state_wheels_.name.push_back("wheel_fr");
  joint_state_wheels_.name.push_back("wheel_rl");
  joint_state_wheels_.name.push_back("wheel_rr");

  // Setup joints
  for (unsigned int i = 0; i < NUM_WHEELS; i++) {
    // Wheel joint
    const std::string &wheel = joint_state_wheels_.name[i];
    joint_wheels_[i] = parent->GetJoint(wheel);
    if (joint_wheels_[i]) {
      joint_wheels_[i]->SetDamping(0, 1.0);
      joint_state_wheels_.position.push_back(0);
      joint_state_wheels_.velocity.push_back(0);
      joint_state_wheels_.effort.push_back(0);
      ROS_INFO("MobilityBasePlugin: Wheel joint: %s is set up successfully", wheel.c_str());
    } else {
      gzerr << "MobilityBasePlugin: Failed to find joint '" << wheel << "' in the model\n";
      return;
    }
    if (!fast_) {
    // Roller joints
      for (unsigned int j = 0; j < NUM_ROLLERS; j++) {
        std::stringstream ss;
        ss << wheel << "_roller_" << j;
        std::string roller = ss.str();
        ROS_INFO("MobilityBasePlugin: Roller joints for %s", roller.c_str());
        joint_rollers_[i][j] = parent->GetJoint(roller);
        if (joint_rollers_[i][j]) {
          ROS_INFO("MobilityBasePlugin: Roller joint: %s found", roller.c_str());
          joint_rollers_[i][j]->SetDamping(0, 0.006);
          joint_state_rollers_.name.push_back(roller);
          joint_state_rollers_.position.push_back(0);
          joint_state_rollers_.velocity.push_back(0);
          joint_state_rollers_.effort.push_back(0);
          ROS_INFO("MobilityBasePlugin: Roller joint: %s is set up successfully", roller.c_str());
        } else {
          ROS_ERROR("MobilityBasePlugin: Roller joint: %s not found in the model", roller.c_str());
          gzerr << "MobilityBasePlugin: Failed to find joint '" << roller << "' in the model\n";
          return;
        }
      }
    }
  }

  // Initialize the ROS node
  ROS_INFO("Initializing the ROS Node...");
  int argc = 0;
  char** argv = NULL;
  ros::init(argc, argv, "gazebo_mobility_base", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
  nh_ = new ros::NodeHandle("/mobility_base");
  ROS_INFO("NodeHandler %s created", nh_->getNamespace().c_str());

  // Set up Publishers
  pub_imu_data_ = nh_->advertise<sensor_msgs::Imu>("imu/data_raw", 10, false);
  pub_imu_mag_ = nh_->advertise<sensor_msgs::MagneticField>("imu/mag", 10, false);
  pub_imu_is_calibrated_ = nh_->advertise<std_msgs::Bool>("imu/is_calibrated", 1, true);
  pub_twist_ = nh_->advertise<geometry_msgs::TwistStamped>("twist", 10, false);
  pub_wrench_ = nh_->advertise<geometry_msgs::WrenchStamped>("wrench", 10, false);
  pub_joint_states_ = nh_->advertise<sensor_msgs::JointState>("joint_states", 10, false);
  pub_joystick_ = nh_->advertise<geometry_msgs::TwistStamped>("joystick", 10, false);
  pub_bumper_states_ = nh_->advertise<mobility_base_core_msgs::BumperState>("bumper_states", 10, true);
  pub_mode_ = nh_->advertise<mobility_base_core_msgs::Mode>("mode", 1, true);

  // Set up Publisher Queues
  pmq_.startServiceThread();
  pmq_imu_data_ = pmq_.addPub<sensor_msgs::Imu>();
  pmq_imu_mag_ = pmq_.addPub<sensor_msgs::MagneticField>();
  pmq_imu_is_calibrated_ = pmq_.addPub<std_msgs::Bool>();
  pmq_twist_ = pmq_.addPub<geometry_msgs::TwistStamped>();
  pmq_wrench_ = pmq_.addPub<geometry_msgs::WrenchStamped>();
  pmq_joint_states_ = pmq_.addPub<sensor_msgs::JointState>();
  pmq_joystick_ = pmq_.addPub<geometry_msgs::TwistStamped>();
  pmq_bumper_states_ = pmq_.addPub<mobility_base_core_msgs::BumperState>();
  pmq_mode_ = pmq_.addPub<mobility_base_core_msgs::Mode>();
  ROS_INFO("MobilityBasePlugin: Publishers set up");

  // Set up Subscribers
  sub_cmd_vel_ = nh_->subscribe("cmd_vel", 1, &MobilityBasePlugin::recvCmdVel, this);
  sub_cmd_vel_raw_ = nh_->subscribe("cmd_vel_raw", 1, &MobilityBasePlugin::recvCmdVelRaw, this);
  ROS_INFO("MobilityBasePlugin: Subscribers set up");

  // Publish latched topics 'mode' and 'imu/is_calibrated'
  publishMode(ros::Time(0));
  std_msgs::Bool msg;
  msg.data = true;
  pmq_imu_is_calibrated_->push(msg, pub_imu_is_calibrated_);
  ROS_INFO("MobilityBasePlugin: IMU is calibrated latched topic published");

  // Listen to the update event. This event is broadcast every simulation iteration.
  spinner_thread_ = new boost::thread(boost::bind( &MobilityBasePlugin::spin, this));
  ROS_INFO("Spinner thread listens to update event of simulation iteration");
  update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&MobilityBasePlugin::UpdateChild, this, _1));
  ROS_INFO("World updates start...");
}

// Called by the world update start event
void MobilityBasePlugin::UpdateChild(const common::UpdateInfo & _info)
{
  const common::Time gstamp = world_->SimTime();
  const ros::Time rstamp(gstamp.sec, gstamp.nsec);
  const double ts = (gstamp - previous_stamp_).Double();

  // Header for all messages
  std_msgs::Header header;
  header.stamp = rstamp;
  header.frame_id = frame_id_;

  // Grab model states from Gazebo
  ignition::math::Vector3<double> linear_vel;
  ignition::math::Vector3<double> angular_vel;
  ignition::math::Vector3<double> linear_accel;
  ignition::math::Vector3<double> angular_pos;
  ignition::math::Quaternion<double> orientation;
  ignition::math::Vector3<double> position;

  linear_vel = model_->RelativeLinearVel();
  angular_vel = model_->RelativeAngularVel();
  linear_accel = model_->RelativeLinearAccel();
  angular_pos = model_->WorldPose().Rot().Euler();
  orientation = model_->WorldPose().Rot();
  position = model_->WorldPose().Pos();

  const ignition::math::Vector3<double> fb_vel(linear_vel.X(), linear_vel.Y(), angular_vel.Z());
  // ROS_INFO("fb_vel was: linear_vel.X() %f, .Y() %f and angular_vel.Z() %f", linear_vel.X(), linear_vel.Y(), angular_vel.Z());
  // Command wheel velocity and torque to meet robot velocity request
  if (first_update_) {
    stamp_vehicle_ = gstamp;
    stamp_imu_ = gstamp;
    stamp_joystick_ = gstamp;
    stamp_bumpers_ = gstamp;
    stamp_mode_ = gstamp;
    cmd_vel_history_ = ignition::math::Vector3<double>::Zero;

  } else {
    // Select command source
    ignition::math::Vector3<double> cmd = ignition::math::Vector3<double>::Zero;
    mobility_base_core_msgs::Mode::_mode_type mode = mobility_base_core_msgs::Mode::MODE_TIMEOUT;
    {
      boost::lock_guard<boost::mutex> lock1(cmd_vel_mutex_);
      boost::lock_guard<boost::mutex> lock2(cmd_vel_raw_mutex_);
      if (cmd_vel_stamp_ > cmd_vel_raw_stamp_) {
        if (gstamp - cmd_vel_stamp_ < common::Time(CMD_TIMEOUT)) {
          cmd = cmd_vel_;
          mode = mobility_base_core_msgs::Mode::MODE_VELOCITY;
        }
      } else {
        if (gstamp - cmd_vel_raw_stamp_ < common::Time(CMD_TIMEOUT)) {
          cmd = cmd_vel_raw_;
          mode = mobility_base_core_msgs::Mode::MODE_VELOCITY_RAW;
        }
      }
    }

    // Select acceleration limits
    ignition::math::Vector3<double> accel_limit;
    switch (mode) {
      case mobility_base_core_msgs::Mode::MODE_VELOCITY:
        accel_limit.X(ACCEL_LIMIT_SLOW_VXY);
        accel_limit.Y(ACCEL_LIMIT_SLOW_VXY);
        accel_limit.Z(ACCEL_LIMIT_SLOW_WZ);
        break;
      case mobility_base_core_msgs::Mode::MODE_VELOCITY_RAW:
      case mobility_base_core_msgs::Mode::MODE_TIMEOUT:
      default:
        accel_limit.X(ACCEL_LIMIT_FAST_VXY);
        accel_limit.Y(ACCEL_LIMIT_FAST_VXY);
        accel_limit.Z(ACCEL_LIMIT_FAST_WZ);
        break;
    }

    // Initialize feedback for velocity change calculation
    if (mode != mode_) {
      cmd_vel_history_ = fb_vel;
    }

    // Limit change in velocity from previous command
    ignition::math::Vector3<double> temp;
    temp.X(limitDelta(cmd.X(), cmd_vel_history_.X(), accel_limit.X() * ts)); // m/s^2
    temp.Y(limitDelta(cmd.Y(), cmd_vel_history_.Y(), accel_limit.Y() * ts)); // m/s^2
    temp.Z(limitDelta(cmd.Z(), cmd_vel_history_.Z(), accel_limit.Z() * ts)); // rad/s^2

    // Limit change in velocity from feedback
    temp.X(limitDelta(temp.X(), fb_vel.X(), ACCEL_INSTANT_VXY)); // m/s
    temp.Y(limitDelta(temp.Y(), fb_vel.Y(), ACCEL_INSTANT_VXY)); // m/s
    temp.Z(limitDelta(temp.Z(), fb_vel.Z(), ACCEL_INSTANT_WZ)); // rad/s

    // ROS_INFO("Limit changes in velocity - temp.X %f, .Y %f, .Z %f", temp.X(), temp.Y(), temp.Z());

    // Compute motor velocity commands
    double speed[4];
    omniFromCartesian(temp.X(), temp.Y(), temp.Z(), speed);
    if (omniSaturate(RADIANS_PER_SECOND_MAX, speed)) {
      cmd_vel_history_ = fb_vel;
    } else {
      cmd_vel_history_ = temp;
    }

    if (fast_) {
      // Apply force and torque to base_footprint to control mobility_base
      ignition::math::Vector3<double> linear_vel = orientation.RotateVector(ignition::math::Vector3<double>(temp.X(), temp.Y(), 0.0));
      ignition::math::Vector3<double> linear_vel_orig = link_base_footprint_->RelativeLinearVel();
      ignition::math::Vector3<double> angular_vel_orig = link_base_footprint_->RelativeAngularVel();
      link_base_footprint_->SetLinearVel(ignition::math::Vector3<double>(GAIN_X * linear_vel.X(), GAIN_Y * linear_vel.Y(), linear_vel_orig.Z() + linear_vel.Z()));
      link_base_footprint_->SetAngularVel(ignition::math::Vector3<double>(angular_vel_orig.X(), angular_vel_orig.Y(), GAIN_Z * temp.Z()));
    }

    // ROS_INFO("Commanding the wheel motors");
    // Command the wheel motors
    for (unsigned int i = 0; i < NUM_WHEELS; i++) {
      joint_wheels_[i]->SetVelocity(0, speed[i]);
      // ROS_INFO("SetVelocity(0, %f) for joint_wheels_[i] %s", speed[i], joint_wheels_[i]->GetName().c_str());
      joint_wheels_[i]->SetEffortLimit(0, TORQUE_MAX_GLOBAL);
    }

    // Publish vehicle feedback
    if (gstamp - stamp_vehicle_ >= common::Time(1.0 / PUB_FREQ_VEHICLE)) {
      stamp_vehicle_ += common::Time(1.0 / PUB_FREQ_VEHICLE);

      // ROS_INFO("Publishing twist message");
      // Publish twist
      geometry_msgs::TwistStamped msg_twist;
      msg_twist.header = header;
      msg_twist.twist.linear.x = linear_vel.X();
      msg_twist.twist.linear.y = linear_vel.Y();
      msg_twist.twist.linear.z = linear_vel.Z();
      msg_twist.twist.angular.x = angular_vel.X();
      msg_twist.twist.angular.y = angular_vel.Y();
      msg_twist.twist.angular.z = angular_vel.Z();
      pmq_twist_->push(msg_twist, pub_twist_);

      // Publish wrench
      geometry_msgs::WrenchStamped msg_wrench;
      msg_wrench.header = header;
      msg_wrench.wrench.force.x = 0.0;
      msg_wrench.wrench.force.y = 0.0;
      msg_wrench.wrench.torque.z = 0.0;
      pmq_wrench_->push(msg_wrench, pub_wrench_);
      // ROS_INFO("Pushed /twist and /wrench feedbacks");

      // Publish joint_states
      joint_state_wheels_.header = header;
      if (!fast_)
        joint_state_rollers_.header = header;
      for (unsigned int i = 0; i < NUM_WHEELS; i++) {
        joint_state_wheels_.position[i] = joint_wheels_[i]->Position();
        joint_state_wheels_.velocity[i] = joint_wheels_[i]->GetVelocity(0);

        if (!fast_) {
          for (unsigned int j = 0; j < NUM_ROLLERS; j++) {
            joint_state_rollers_.position[i * NUM_ROLLERS + j] = joint_rollers_[i][j]->Position();
            joint_state_rollers_.velocity[i * NUM_ROLLERS + j] = joint_rollers_[i][j]->GetVelocity(0);
          }
        }
      }
      pmq_joint_states_->push(joint_state_wheels_, pub_joint_states_);
      if (!fast_)
        pmq_joint_states_->push(joint_state_rollers_, pub_joint_states_);

      // Optionally publish tf transform
      if (tf_broadcaster_) {
        geometry_msgs::TransformStamped transform;
        transform.header.stamp = rstamp;
        transform.header.frame_id = parent_frame_id_;
        transform.child_frame_id = child_frame_id_;
        transform.transform.translation.x = position.X();
        transform.transform.translation.y = position.Y();
        transform.transform.translation.z = position.Z();
        transform.transform.rotation.w = orientation.W();
        transform.transform.rotation.x = orientation.X();
        transform.transform.rotation.y = orientation.Y();
        transform.transform.rotation.z = orientation.Z();
        tf_broadcaster_->sendTransform(transform);
      }
    }

    // Publish imu feedback
    if (gstamp - stamp_imu_ >= common::Time(1.0 / PUB_FREQ_IMU)) {
      stamp_imu_ += common::Time(1.0 / PUB_FREQ_IMU);

      // Publish imu_data_
      sensor_msgs::Imu msg_imu;
      msg_imu.header = header;
      msg_imu.linear_acceleration.x = linear_accel.X();
      msg_imu.linear_acceleration.y = linear_accel.Y();
      msg_imu.linear_acceleration.z = linear_accel.Z();
      msg_imu.orientation_covariance[0] = -1;
      msg_imu.angular_velocity.x = angular_vel.X();
      msg_imu.angular_velocity.y = angular_vel.Y();
      msg_imu.angular_velocity.z = angular_vel.Z();
      msg_imu.angular_velocity_covariance[0] = -1;
      pmq_imu_data_->push(msg_imu, pub_imu_data_);

      sensor_msgs::MagneticField msg_mag;
      msg_mag.header = header;
      msg_mag.magnetic_field.x = 0.0;
      msg_mag.magnetic_field.y = 0.0;
      msg_mag.magnetic_field.z = 0.0;
      msg_mag.magnetic_field_covariance[0] = -1;
      pmq_imu_mag_->push(msg_mag, pub_imu_mag_);
    }

    // Publish joystick feedback
    if (gstamp - stamp_joystick_ >= common::Time(1.0 / PUB_FREQ_JOYSTICK)) {
      stamp_joystick_ += common::Time(1.0 / PUB_FREQ_JOYSTICK);
      geometry_msgs::TwistStamped msg;
      msg.header = header;
      pmq_joystick_->push(msg, pub_joystick_);
    }

    // Publish bumper_states feedback
    if (gstamp - stamp_bumpers_ >= common::Time(1.0 / PUB_FREQ_BUMPERS)) {
      stamp_bumpers_ += common::Time(1.0 / PUB_FREQ_BUMPERS);
      mobility_base_core_msgs::BumperState msg;
      msg.front_left = 0;
      msg.front_right = 0;
      msg.rear_left = 0;
      msg.rear_right = 0;
      pmq_bumper_states_->push(msg, pub_bumper_states_);
    }

    // Publish mode feedback
    if (mode != mode_) {
      stamp_mode_ = gstamp;
      publishMode(rstamp);
    } else if (gstamp - stamp_mode_ >= common::Time(1.0 / PUB_FREQ_MODE)) {
      stamp_mode_ += common::Time(1.0 / PUB_FREQ_MODE);
      publishMode(rstamp);
    }

    mode_ = mode;
  }

  previous_stamp_ = gstamp;
  first_update_ = false;
}

void MobilityBasePlugin::recvCmdVel(const geometry_msgs::Twist::ConstPtr& msg)
{
  boost::lock_guard<boost::mutex> lock(cmd_vel_mutex_);
  cmd_vel_stamp_ = world_->SimTime();
  ROS_INFO("Received cmd_vel message!");
  cmd_vel_.X(msg->linear.x);
  cmd_vel_.Y(msg->linear.y);
  cmd_vel_.Z(msg->angular.z);
  ROS_INFO("cmd_vel_.X() set to %f", cmd_vel_.X());
}

void MobilityBasePlugin::recvCmdVelRaw(const geometry_msgs::Twist::ConstPtr& msg)
{
  boost::lock_guard<boost::mutex> lock(cmd_vel_raw_mutex_);
  cmd_vel_raw_stamp_ = world_->SimTime();
  cmd_vel_raw_.X(msg->linear.x);
  cmd_vel_raw_.Y(msg->linear.y);
  cmd_vel_raw_.Z(msg->angular.z);
}

void MobilityBasePlugin::publishMode(const ros::Time &stamp)
{
  mobility_base_core_msgs::Mode msg;
  msg.header.frame_id = frame_id_;
  msg.header.stamp = stamp;
  msg.mode = mode_;
  pmq_mode_->push(msg, pub_mode_);
}

void MobilityBasePlugin::spin()
{
  while (ros::ok()) {
    ros::spinOnce();
  }
}

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(MobilityBasePlugin)
}
