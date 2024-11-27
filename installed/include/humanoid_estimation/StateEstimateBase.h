/********************************************************************************
Modified Copyright (c) 2023-2024, BridgeDP Robotics.Co.Ltd. All rights reserved.

For further information, contact: contact@bridgedp.com or visit our website
at www.bridgedp.com.
********************************************************************************/

#pragma once

#include <ros/ros.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <realtime_tools/realtime_publisher.h>

#include <humanoid_common/hardware_interface/ContactSensorInterface.h>
#include <humanoid_common/hardware_interface/HybridJointInterface.h>
#include <ocs2_centroidal_model/CentroidalModelInfo.h>
#include <humanoid_interface/common/ModelSettings.h>
#include <humanoid_interface/common/Types.h>
#include <humanoid_interface/gait/MotionPhaseDefinition.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>

#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_core/reference/ModeSchedule.h>
#include "std_msgs/Float64MultiArray.h"
#include "humanoid_interface/common/TopicLogger.h"

namespace ocs2
{
  namespace humanoid
  {

    class StateEstimateBase
    {
    public:
      StateEstimateBase(PinocchioInterface pinocchioInterface, CentroidalModelInfo info,
                        const PinocchioEndEffectorKinematics &eeKinematics);
      virtual void updateJointStates(const vector_t &jointPos, const vector_t &jointVel);
      virtual void getJointStates(vector_t &jointPos, vector_t &jointVel)
      {
        jointPos = rbdState_.segment(6, info_.actuatedDofNum);
        jointVel = rbdState_.segment(6 + info_.generalizedCoordinatesNum, info_.actuatedDofNum);
      }
      virtual void updateContact(contact_flag_t contactFlag)
      {
        contactFlag_ = contactFlag;
        mode_ = stanceLeg2ModeNumber(contactFlag_);
      }
      virtual void updateMode(size_t mode)
      {
        mode_ = mode;
        contactFlag_ = modeNumber2StanceLeg(mode_);
      }
      virtual void updateGait(const std::string &gait)
      {
        prev_gait_ = gait_;
        gait_ = gait;
      }
      virtual void set_intial_state(const vector_t &ocs2_state) {};
      virtual void updateIntialEulerAngles(const Eigen::Quaternion<scalar_t> &quat_init);
      virtual void updateImu(const Eigen::Quaternion<scalar_t> &quat, const vector3_t &angularVelLocal,
                             const vector3_t &linearAccelLocal, const matrix3_t &orientationCovariance,
                             const matrix3_t &angularVelCovariance, const matrix3_t &linearAccelCovariance);

      virtual vector_t update(const ros::Time &time, const ros::Duration &period) = 0;

      size_t ContactDetection(size_t mode_des, const Eigen::VectorXd &joint_v, const Eigen::VectorXd &joint_current, double dt);

      size_t getMode()
      {
        return stanceLeg2ModeNumber(contactFlag_);
      }

      feet_array_t<vector3_t> &getLatestStancePos()
      {
        return latestStanceposition_;
      }

      vector_t getBodyVelWorld()
      {
        vector_t body_vel(6);
        body_vel.head(3) = rbdState_.segment<3>(info_.generalizedCoordinatesNum + 3);
        body_vel.tail(3) = rbdState_.segment<3>(info_.generalizedCoordinatesNum);
        return std::move(body_vel);
      }

      void setStartStopTime4Legs(const feet_array_t<std::array<scalar_t, 2>> &start_stop_time_4_legs)
      {
        StartStopTime4Legs_ = start_stop_time_4_legs;
      }
      void updateCmdContact(contact_flag_t cmd_contact_flag)
      {
        cmdContactflag_ = std::move(cmd_contact_flag);
      }
      void setCmdTorque(const vector_t &cmd_torque)
      {
        cmdTorque_ = cmd_torque;
      }
      void estContactForce(const ros::Duration &period);
      contact_flag_t estContactState(const scalar_t &time);
      void loadSettings(const std::string &taskFile, bool verbose);

      const vector_t &getEstContactForce()
      {
        return estContactforce_;
      }
      const vector_t &getEstDisturbanceTorque()
      {
        return estDisturbancetorque_;
      }

      const std::array<contact_flag_t, 2> &getEarlyLateContact()
      {
        return earlyLatecontact_;
      }

      vector_t getRbdState()
      {
        return rbdState_;
      }

      virtual void reset()
      {

      }

    protected:
      void earlyContactDetection(const ModeSchedule &modeSchedule, scalar_t current_time);
      void lateContactDetection(const ModeSchedule &modeSchedule, scalar_t current_time);
      void updateAngular(const vector3_t &zyx, const vector_t &angularVel);
      void updateLinear(const vector_t &pos, const vector_t &linearVel);
      void publishMsgs(const nav_msgs::Odometry &odom);

      PinocchioInterface pinocchioInterface_;
      CentroidalModelInfo info_;
      std::unique_ptr<PinocchioEndEffectorKinematics> eeKinematics_;

      vector3_t zyxOffset_ = vector3_t::Zero();
      vector_t rbdState_;
      vector3_t prev_zyx_ = vector3_t::Zero();
      contact_flag_t contactFlag_{};
      Eigen::Quaternion<scalar_t> quat_;
      Eigen::Quaternion<scalar_t> yaw_offset_quat_;
      vector3_t angle_zyx_init_;
      double stance_angle_yaw_init_ = 0.0;
      vector3_t angularVelLocal_, linearAccelLocal_;
      vector3_t angularVelWorld_, linearAccelWorld_;
      vector_t jointPos_, jointVel_;

      matrix3_t orientationCovariance_, angularVelCovariance_, linearAccelCovariance_;

      std::shared_ptr<realtime_tools::RealtimePublisher<nav_msgs::Odometry>> odomPub_;
      std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::PoseWithCovarianceStamped>> posePub_;
      ros::Time lastPub_;

      std::string gait_ = "stance";
      std::string prev_gait_ = "stance";
      feet_array_t<vector3_t> latestStanceposition_;

      vector_t pSCgZinvlast_;
      vector_t vMeasuredLast_;
      vector_t estContactforce_;
      vector_t estDisturbancetorque_;
      vector_t cmdTorque_;

      scalar_t cutoffFrequency_ = 150;
      scalar_t detectCutoffFrequency_ = 150;
      scalar_t contactThreshold_ = 23;
      contact_flag_t cmdContactflag_{};
      feet_array_t<std::array<scalar_t, 2>> StartStopTime4Legs_;

      std::array<contact_flag_t, 2> earlyLatecontact_;
      std_msgs::Float64MultiArray earlyLateContactMsg_;
      std::deque<std::pair<scalar_t, contact_flag_t>> estConHistory_;

      // contact detection
      double prve_r_EC_filter{0};
      double prve_l_EC_filter{0};
      size_t contact_state_ = ModeNumber::SS;
      size_t prev_contact_state_ = ModeNumber::SS;
      size_t mode_ = ModeNumber::SS;
      // bool contact_updata_check_{true};
      double change_contact_time_sum_{0};
      TopicLogger *ros_logger_{nullptr};
      double max_energy_threshold_ = 10, min_energy_threshold_ = -10;
      double max_energy_threshold2_ = 20, min_energy_threshold2_ = -20;
      double time_treshold_ = 0.15;
    };

    template <typename T>
    T square(T a)
    {
      return a * a;
    }

    template <typename SCALAR_T>
    Eigen::Matrix<SCALAR_T, 3, 1> quatToZyx(const Eigen::Quaternion<SCALAR_T> &q)
    {
      Eigen::Matrix<SCALAR_T, 3, 1> zyx;

      SCALAR_T as = std::min(-2. * (q.x() * q.z() - q.w() * q.y()), .99999);
      zyx(0) =
          std::atan2(2 * (q.x() * q.y() + q.w() * q.z()), square(q.w()) + square(q.x()) - square(q.y()) - square(q.z()));
      zyx(1) = std::asin(as);
      zyx(2) =
          std::atan2(2 * (q.y() * q.z() + q.w() * q.x()), square(q.w()) - square(q.x()) - square(q.y()) + square(q.z()));
      return zyx;
    }

  } // namespace humanoid
} // namespace ocs2
