#include <pinocchio/fwd.hpp> // forward declarations must be included first.
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include "std_srvs/SetBool.h"

#include "humanoid_controllers/humanoidController.h"

#include <ocs2_centroidal_model/AccessHelperFunctions.h>
#include <ocs2_centroidal_model/CentroidalModelPinocchioMapping.h>
#include <ocs2_centroidal_model/ModelHelperFunctions.h>

#include <ocs2_core/thread_support/ExecuteAndSleep.h>
#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <humanoid_interface_ros/gait/GaitReceiver.h>
#include <ocs2_msgs/mpc_observation.h>
#include <ocs2_pinocchio_interface/PinocchioEndEffectorKinematics.h>
#include <ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_ros_interfaces/synchronized_module/RosReferenceManager.h>
#include <ocs2_sqp/SqpMpc.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <std_srvs/Trigger.h>
#include <algorithm> 
#include <angles/angles.h>
#include <humanoid_estimation/FromTopiceEstimate.h>
#include <humanoid_estimation/LinearKalmanFilter.h>
#ifdef KUAVO_CONTROL_LIB_FOUND
#include <kuavo_estimation/base_filter/InEkfBaseFilter.h>
#endif
#include <humanoid_wbc/WeightedWbc.h>
#include <humanoid_wbc/HierarchicalWbc.h>
#include "humanoid_interface_drake/common/utils.h"
#include "humanoid_interface_drake/common/sensor_data.h"
#include "humanoid_interface_drake/humanoid_interface_drake.h"

namespace humanoid_controller
{
  using namespace ocs2;
  using namespace humanoid;
  using Duration = std::chrono::duration<double>;
  using Clock = std::chrono::high_resolution_clock;
  std::mutex head_mtx;


  static void mujocoSimStart(ros::NodeHandle &nh_)
  {
    std_srvs::SetBool srv;
    srv.request.data = true;

    // 等待服务可用
    bool service_available = ros::service::waitForService("sim_start", ros::Duration(5.0)); // 5秒超时

    if (service_available)
    {
      ros::ServiceClient sim_start_client = nh_.serviceClient<std_srvs::SetBool>("sim_start");
      if (sim_start_client.call(srv))
      {
        if (srv.response.success)
        {
          ROS_INFO("sim_start Service call succeeded with message: %s", srv.response.message.c_str());
          return;
        }
        else
        {
          ROS_ERROR("sim_start Service call failed");
        }
      }
      else
      {
        ROS_ERROR("Failed to call sim_start service");
      }
    }
    else
    {
      ROS_ERROR("sim_start Service not available");
    }
    exit(1);
  }
  void humanoidController::keyboard_thread_func()
  {
    usleep(100000);
    struct sched_param param;
    param.sched_priority = 0;
    auto result = pthread_setschedparam(pthread_self(), SCHED_OTHER, &param);
    if (result != 0)
    {
      std::cerr << "Failed to set keyboard_thread_func's scheduling parameters. Error: " << strerror(result) << std::endl;
    }
    stop_pub_ = controllerNh_.advertise<std_msgs::Bool>("/stop_robot", 10);

    char Walk_Command = '\0';
    while (ros::ok())
    {
      if (hardware_status_ != 1)
      {
        usleep(100000);
        continue;
      }
      if (kbhit())
      {
        Walk_Command = getchar();
        std::cout << "[keyboard command]: " << Walk_Command << std::endl;
        if (Walk_Command == 'x')
        {
          std::cout << "x" << std::endl;
          for (int i = 0; i < 5; i++)
          {
            std::cout << "publish stop message" << std::endl;
            std_msgs::Bool stop_msg;
            stop_msg.data = true;
            stop_pub_.publish(stop_msg);
            ros::Duration(0.1).sleep();
          }
        }
        else if (Walk_Command == 'f')
        {
          wbc_only_ = !wbc_only_;
          std::cout << "start using mpc: " << !wbc_only_ << std::endl;
        }
        else if (Walk_Command == 'r')
        {
          std::cerr << "reset MPC " << std::endl;
          reset_mpc_ = true;
        }
        else if (Walk_Command == 'g')
        {
          std::cout << "reset estimator" << std::endl;
          reset_estimator_ = true;
        }
        

        Walk_Command = '\0';
      }
      usleep(50000);
    }
  }

  bool humanoidController::init(HybridJointInterface *robot_hw, ros::NodeHandle &controller_nh, bool is_nodelet_node)
  {
    int robot_version_int;
    RobotVersion robot_version(3, 4);
    if (controllerNh_.hasParam("/robot_version"))
    {
        controllerNh_.getParam("/robot_version", robot_version_int);
        int major = robot_version_int / 10;
        int minor = robot_version_int % 10;
        robot_version = RobotVersion(major, minor);
    }
    is_nodelet_node_ = is_nodelet_node;
    drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(robot_version, true, 2e-3);
    kuavo_settings_ = drake_interface_->getKuavoSettings();
    auto &motor_info = kuavo_settings_.hardware_settings;
    headNum_ = motor_info.num_head_joints;
    armNumReal_ = motor_info.num_arm_joints;
    jointNumReal_ = motor_info.num_joints - headNum_ - armNumReal_;
    actuatedDofNumReal_ = jointNumReal_ + armNumReal_ + headNum_;
    ros::param::set("/armRealDof",  static_cast<int>(armNumReal_));
    ros::param::set("/legRealDof",  static_cast<int>(jointNumReal_));
    ros::param::set("/headRealDof",  static_cast<int>(headNum_));

    motor_c2t_ = Eigen::Map<Eigen::VectorXd>(motor_info.c2t_coeff.data(), motor_info.c2t_coeff.size());
    auto [plant, context] = drake_interface_->getPlantAndContext();
    ros_logger_ = new TopicLogger(controller_nh);
    controllerNh_ = controller_nh;
    // Initialize OCS2
    std::string urdfFile;
    std::string taskFile;
    std::string referenceFile;
    std::string gaitCommandFile;
    controllerNh_.getParam("/urdfFile", urdfFile);
    controllerNh_.getParam("/taskFile", taskFile);
    controllerNh_.getParam("/referenceFile", referenceFile);
    controllerNh_.getParam("/gaitCommandFile", gaitCommandFile);
    controllerNh_.getParam("/use_external_mpc", use_external_mpc_);
    double controlFrequency = 500.0; // 1000Hz
    controllerNh_.getParam("/wbc_frequency", controlFrequency);
    dt_ = 1.0 / controlFrequency;
    if (controllerNh_.hasParam("/real"))
    {
      controllerNh_.getParam("/real", is_real_);
      controllerNh_.getParam("/cali", is_cali_);
      if (is_real_)
      {
        std::cout << "real robot controller" << std::endl;
        ros::NodeHandlePtr nh_ptr = boost::make_shared<ros::NodeHandle>(controllerNh_);
        hardware_interface_ptr_ = new KuavoHardwareInterface(nh_ptr, jointNum_);
      }
    }
    if (controllerNh_.hasParam("wbc_only"))
    {
      controllerNh_.getParam("/wbc_only", wbc_only_);
    }
    if (controllerNh_.hasParam("play_back"))
    {
      controllerNh_.getParam("/play_back", is_play_back_mode_);

    }

    if (controllerNh_.hasParam("use_joint_filter"))
    {
      controllerNh_.getParam("/use_joint_filter", use_joint_filter_);
    }
    // trajectory_publisher_ = new TrajectoryPublisher(controller_nh, 0.001);
    
    size_t buffer_size = (is_play_back_mode_) ? 20 : 5;
    sensors_data_buffer_ptr_ = new KuavoDataBuffer<SensorData>("humanoid_sensors_data_buffer", buffer_size, dt_);
    gaitManagerPtr_ = new GaitManager(20);
    gaitManagerPtr_->add(0.0, "stance");
    bool verbose = false;
    loadData::loadCppDataType(taskFile, "humanoid_interface.verbose", verbose);
    loadData::loadCppDataType(taskFile, "contact_cst_st", contact_cst_st_);
    loadData::loadCppDataType(taskFile, "contact_cst_et", contact_cst_et_);

#ifdef KUAVO_CONTROL_LIB_FOUND
    joint_filter_ptr_ = new HighlyDynamic::JointFilter(&plant, &kuavo_settings_, 12, dt_, ros_logger_);
#endif
    setupHumanoidInterface(taskFile, urdfFile, referenceFile, gaitCommandFile, verbose, robot_version_int);
    setupMpc();
    setupMrt();
    // Visualization
    CentroidalModelPinocchioMapping pinocchioMapping(HumanoidInterface_->getCentroidalModelInfo());
    std::cout << "HumanoidInterface_->getCentroidalModelInfo().robotMass:" << HumanoidInterface_->getCentroidalModelInfo().robotMass << std::endl;

    eeKinematicsPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(HumanoidInterface_->getPinocchioInterface(), pinocchioMapping,
                                                                        HumanoidInterface_->modelSettings().contactNames3DoF);
    eeSpatialKinematicsPtr_ = std::make_shared<PinocchioEndEffectorSpatialKinematics>(HumanoidInterface_->getPinocchioInterface(), pinocchioMapping, 
                                                                                      HumanoidInterface_->modelSettings().contactNames6DoF);
    
    robotVisualizer_ = std::make_shared<HumanoidVisualizer>(HumanoidInterface_->getPinocchioInterface(),
                                                            HumanoidInterface_->getCentroidalModelInfo(), 
                                                            *eeKinematicsPtr_, *eeSpatialKinematicsPtr_, controllerNh_, taskFile);

    pinocchioInterface_ptr_ = new PinocchioInterface(HumanoidInterface_->getPinocchioInterface());
    centroidalModelInfo_ = HumanoidInterface_->getCentroidalModelInfo();
    eeKinematicsPtr_->setPinocchioInterface(*pinocchioInterface_ptr_);

    auto &info = HumanoidInterface_->getCentroidalModelInfo();
    jointNum_ = HumanoidInterface_->modelSettings().mpcLegsDof;
    armNum_ = info.actuatedDofNum - jointNum_;


    if (armNumReal_ + jointNumReal_ != jointNum_ + armNum_) // mpc维度和实际维度不一致，简化的模型
    {
      is_simplified_model_ = true;
      std::cout << "[HumanoidController]: using simplified mpc model" << std::endl;
      std::cout << "jointNumReal_:" << jointNumReal_ << " jointNum_:" << jointNum_ << std::endl;
      std::cout << "headNum_:" << headNum_ << std::endl;
      std::cout << "armNumReal_:" << armNumReal_ << " armNum_:" << armNum_<< std::endl;
      armDofMPC_ = armNum_ / 2;
      armDofReal_ = armNumReal_ / 2;
      armDofDiff_ = armDofReal_ - armDofMPC_;
      simplifiedJointPos_ = vector_t::Zero(armDofDiff_*2);
    }
    defalutJointPos_.resize(info.actuatedDofNum);
    sensor_data_head_.resize_joint(headNum_);
    joint_kp_.resize(actuatedDofNumReal_);
    joint_kd_.resize(actuatedDofNumReal_);
    joint_kp_walking_.resize(actuatedDofNumReal_);
    joint_kd_walking_.resize(actuatedDofNumReal_);
    head_kp_.resize(headNum_);
    head_kd_.resize(headNum_);

    joint_control_modes_ = Eigen::VectorXd::Constant(actuatedDofNumReal_, 2);
    output_tau_ = vector_t::Zero(actuatedDofNumReal_);
    output_pos_ = vector_t::Zero(actuatedDofNumReal_);
    output_vel_ = vector_t::Zero(actuatedDofNumReal_);
    Eigen::Vector3d acc_filter_params;
    Eigen::Vector3d gyro_filter_params;
    double arm_joint_pos_filter_cutoff_freq=20,arm_joint_vel_filter_cutoff_freq=20,mrt_joint_vel_filter_cutoff_freq=200;
    auto drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(robot_version, true, 2e-3);
    defalutJointPos_.head(jointNum_) = drake_interface_->getDefaultJointState();
    defalutJointPos_.tail(armNum_) = vector_t::Zero(armNum_);
    currentArmTargetTrajectories_ = {{0.0}, {vector_t::Zero(armNumReal_)}, {vector_t::Zero(info.inputDim)}};

    vector_t drake_q = drake_interface_->getDrakeState();
    vector_t mujoco_q = vector_t::Zero(drake_q.size());
    mujoco_q << drake_q.segment(4, 3), drake_q.head(4), drake_q.tail(drake_q.size() - 7);
    std::vector<double> mujoco_init_state;
    for (int i = 0; i < drake_q.size(); i++)
    {
      mujoco_init_state.push_back(mujoco_q(i));
    }
    
    ros::param::set("mujoco_init_state", mujoco_init_state);
  

    joint_state_limit_.resize(actuatedDofNumReal_, 2);

    auto robot_config = drake_interface_->getRobotConfig();
    is_swing_arm_ = robot_config->getValue<bool>("swing_arm");
    swing_arm_gain_ = robot_config->getValue<double>("swing_arm_gain");
    swing_elbow_scale_ = robot_config->getValue<double>("swing_elbow_scale");
    ruiwo_motor_velocities_factor_ = robot_config->getValue<double>("motor_velocities_factor");
    gait_map_ = HumanoidInterface_->getSwitchedModelReferenceManagerPtr()->getGaitSchedule()->getGaitMap();
    std::cout << "gait_map size: " << gait_map_.size() << std::endl;

    loadData::loadEigenMatrix(referenceFile, "joint_kp_", joint_kp_);
    loadData::loadEigenMatrix(referenceFile, "joint_kd_", joint_kd_);
    loadData::loadEigenMatrix(referenceFile, "joint_kp_walking_", joint_kp_walking_);
    loadData::loadEigenMatrix(referenceFile, "joint_kd_walking_", joint_kd_walking_);
    if (headNum_ > 0)
    {
      loadData::loadEigenMatrix(referenceFile, "head_kp_", head_kp_);
      loadData::loadEigenMatrix(referenceFile, "head_kd_", head_kd_);
    }
    loadData::loadEigenMatrix(referenceFile, "acc_filter_cutoff_freq", acc_filter_params);
    loadData::loadEigenMatrix(referenceFile, "gyro_filter_cutoff_freq", gyro_filter_params);
    loadData::loadEigenMatrix(referenceFile, "jointStateLimit", joint_state_limit_);
    loadData::loadCppDataType(referenceFile, "arm_joint_pos_filter_cutoff_freq", arm_joint_pos_filter_cutoff_freq);
    loadData::loadCppDataType(referenceFile, "arm_joint_vel_filter_cutoff_freq", arm_joint_vel_filter_cutoff_freq);
    loadData::loadCppDataType(referenceFile, "mrt_joint_vel_filter_cutoff_freq", mrt_joint_vel_filter_cutoff_freq);
    loadData::loadEigenMatrix(referenceFile, "defaultCotrolMode", joint_control_modes_);


    // Hardware interface
    // TODO: setup hardware controller interface
    // create a ROS subscriber to receive the joint pos and vel
    jointPos_ = vector_t::Zero(info.actuatedDofNum);
    jointPos_.setZero();
    jointPos_.head(jointNum_) = drake_interface_->getDefaultJointState();

    jointVel_ = vector_t::Zero(info.actuatedDofNum);
    jointAcc_ = vector_t::Zero(info.actuatedDofNum);
    jointCurrent_ = vector_t::Zero(info.actuatedDofNum);
    quat_ = Eigen::Quaternion<scalar_t>(1, 0, 0, 0);
    arm_joint_pos_cmd_prev_ = vector_t::Zero(armNumReal_);
    arm_joint_pos_filter_.setParams(dt_, Eigen::VectorXd::Constant(armNumReal_, arm_joint_pos_filter_cutoff_freq));
    arm_joint_vel_filter_.setParams(dt_, Eigen::VectorXd::Constant(armNumReal_, arm_joint_vel_filter_cutoff_freq));
    mrt_joint_vel_filter_.setParams(dt_, Eigen::VectorXd::Constant(info.actuatedDofNum-armNum_, mrt_joint_vel_filter_cutoff_freq));
    acc_filter_.setParams(dt_, acc_filter_params);
    // free_acc_filter_.setParams(dt_, acc_filter_params);
    gyro_filter_.setParams(dt_, gyro_filter_params);
    sensorsDataSub_ = controllerNh_.subscribe<kuavo_msgs::sensorsData>("/sensors_data_raw", 10, &humanoidController::sensorsDataCallback, this);
    mpcStartSub_ = controllerNh_.subscribe<std_msgs::Bool>("/start_mpc", 10, &humanoidController::startMpccallback, this);
    arm_joint_trajectory_.initialize(armNumReal_);
    arm_joint_traj_sub_ = controllerNh_.subscribe<sensor_msgs::JointState>("/kuavo_arm_traj", 10, [this](const sensor_msgs::JointState::ConstPtr &msg)
      {
        if(msg->name.size() != armNumReal_){
          std::cerr << "The dimensin of arm joint pos is NOT equal to the armNumReal_!!" << msg->name.size() << " vs " << armNumReal_ << "\n";
          return;
        }
        for(int i = 0; i < armNumReal_; i++)
        {
          // std::cout << "arm joint pos: " << msg->position[i] << std::endl;
          arm_joint_trajectory_.pos[i] = msg->position[i] * M_PI / 180.0;
          if(msg->velocity.size() == armNumReal_)
            arm_joint_trajectory_.vel[i] = msg->velocity[i] * M_PI / 180.0;
          if(msg->effort.size() == armNumReal_)
            arm_joint_trajectory_.tau[i] = msg->effort[i];
        }
        // std::cout << "arm joint pos: " << arm_joint_trajectory_.pos.size() << std::endl;
      });
     
      // Arm TargetTrajectories
      auto armTargetTrajectoriesCallback = [this](const ocs2_msgs::mpc_target_trajectories::ConstPtr &msg)
      {
        auto targetTrajectories = ros_msg_conversions::readTargetTrajectoriesMsg(*msg);

        if (targetTrajectories.stateTrajectory[0].size() != armNumReal_)
        {
          ROS_WARN_STREAM("[humanoidController]:Using simplified model, but arm targetTrajectories size : "
                          << std::to_string(targetTrajectories.stateTrajectory[0].size()) << " != "
                          << std::to_string(armNumReal_) << ", will keep the simplified arm's joints target");
          return;
        }
        currentArmTargetTrajectories_ = targetTrajectories;
      };
      if (is_simplified_model_)// 简化模型需要直接从全部target的topic中去获取被简化关节的target
        arm_target_traj_sub_ =
            controllerNh_.subscribe<ocs2_msgs::mpc_target_trajectories>(robotName_ + "_mpc_arm_commanded", 3, armTargetTrajectoriesCallback);

      gait_scheduler_sub_ = controllerNh_.subscribe<kuavo_msgs::gaitTimeName>(robotName_ + "_mpc_gait_time_name", 10, [this](const kuavo_msgs::gaitTimeName::ConstPtr &msg)
                                                                              {
                                                                              last_gait_ = current_gait_;
            current_gait_.name = msg->gait_name;
            current_gait_.startTime = msg->start_time;
            if (gaitManagerPtr_)
              gaitManagerPtr_->add(current_gait_.startTime, current_gait_.name);
            std::cout << "[controller] receive current gait name: " << current_gait_.name << " start time: " << current_gait_.startTime << std::endl; });
      head_sub_ = controllerNh_.subscribe("/robot_head_motion_data", 10, &humanoidController::headCmdCallback, this);

      enableArmCtrlSrv_ = controllerNh_.advertiseService("/enable_wbc_arm_trajectory_control", &humanoidController::enableArmTrajectoryControlCallback, this);
      jointCmdPub_ = controllerNh_.advertise<kuavo_msgs::jointCmd>("/joint_cmd", 10);
      mpcPolicyPublisher_ = controllerNh_.advertise<ocs2_msgs::mpc_flattened_controller>(robotName_ + "_mpc_policy", 1, true);
      feettargetTrajectoriesPublisher_ = controllerNh_.advertise<ocs2_msgs::mpc_target_trajectories>("/humanoid_controller/feet_target_policys", 10, true);

      wbcFrequencyPub_ = controllerNh_.advertise<std_msgs::Float64>("/monitor/frequency/wbc", 10);
      wbcTimeCostPub_ = controllerNh_.advertise<std_msgs::Float64>("/monitor/time_cost/wbc", 10);

      // State estimation
      setupStateEstimate(taskFile, verbose);
      sensors_data_buffer_ptr_->waitForReady();
      // std::cout << "waitForReady estimate ready" << std::endl;
      // Whole body control/HierarchicalWbc/WeightedWbc
      // wbc 中 eeKinematicsPtr_ 可能需要修改
      wbc_ = std::make_shared<WeightedWbc>(*pinocchioInterfaceWBCPtr_, centroidalModelInfoWBC_,
                                           *eeKinematicsWBCPtr_);
      wbc_->setArmNums(armNumReal_);
      wbc_->loadTasksSetting(taskFile, verbose, is_real_);

      // Safety Checker
      safetyChecker_ = std::make_shared<SafetyChecker>(HumanoidInterface_->getCentroidalModelInfo());
      keyboardThread_ = std::thread(&humanoidController::keyboard_thread_func, this);
      if (!keyboardThread_.joinable())
      {
        std::cerr << "Failed to start keyboard thread" << std::endl;
        exit(1);
    }

    return true;
  }
  void humanoidController::headCmdCallback(const kuavo_msgs::robotHeadMotionData::ConstPtr &msg)
  {
      if (msg->joint_data.size() ==2)
      {
          if (msg->joint_data[0] < -30 || msg->joint_data[0] > 30 || msg->joint_data[1] < -25 || msg->joint_data[1] > 25)
          {
              ROS_WARN("Invalid robot head motion data. Joint data must be in the range [-30, 30] and [-25, 25].");
              return;
          }
          head_mtx.lock();
          desire_head_pos_[0] = msg->joint_data[0]*M_PI/180.0;
          desire_head_pos_[1] = msg->joint_data[1]*M_PI/180.0;
          head_mtx.unlock();
      }
      else
      {
          ROS_WARN("Invalid robot head motion data. Expected 2 elements, but received %lu elements.", msg->joint_data.size());
      }
  }
  void humanoidController::startMpccallback(const std_msgs::Bool::ConstPtr &msg)
  {
    ROS_INFO_STREAM("start_mpc: " << msg->data);
    bool start_mpc_ = msg->data;
    wbc_only_ = !start_mpc_;
  }
  void humanoidController::publishFeetTrajectory(const TargetTrajectories &targetTrajectories)
  {
    auto &stateTrajectory = targetTrajectories.stateTrajectory;
    auto &inputTrajectory = targetTrajectories.inputTrajectory;
    auto &timeTrajectory = targetTrajectories.timeTrajectory;
    TargetTrajectories pubFeetTrajectories;
    pubFeetTrajectories.timeTrajectory = timeTrajectory;
    pubFeetTrajectories.stateTrajectory.clear();
    for (size_t j = 0; j < stateTrajectory.size(); j++)
    {
      const auto state = stateTrajectory.at(j);
      // Fill feet msgs
      const auto &model = pinocchioInterface_ptr_->getModel();
      auto &data = pinocchioInterface_ptr_->getData();
      const auto &q = centroidal_model::getGeneralizedCoordinates(state, centroidalModelInfo_);

      pinocchio::forwardKinematics(model, data, q);
      pinocchio::updateFramePlacements(model, data);

      const auto feetPositions = eeKinematicsPtr_->getPosition(state);
      vector_t feetPositions_vec(3 * centroidalModelInfo_.numThreeDofContacts);
      for (size_t i = 0; i < centroidalModelInfo_.numThreeDofContacts; i++)
      {
        feetPositions_vec.segment(3 * i, 3) = feetPositions[i];
      }
      pubFeetTrajectories.stateTrajectory.push_back(feetPositions_vec);
    }

    const auto mpcTargetTrajectoriesMsg = ros_msg_conversions::createTargetTrajectoriesMsg(pubFeetTrajectories);
    feettargetTrajectoriesPublisher_.publish(mpcTargetTrajectoriesMsg);
  }

  void humanoidController::sensorsDataCallback(const kuavo_msgs::sensorsData::ConstPtr &msg)
  {
    auto &joint_data = msg->joint_data;
    auto &imu_data = msg->imu_data;
    auto &end_effector_data = msg->end_effector_data; // TODO: add end_effector_data to the observation
    SensorData sensor_data;
    sensor_data.resize_joint(jointNumReal_+armNumReal_);
    // JOINT DATA
    for (size_t i = 0; i < jointNumReal_+armNumReal_; ++i)
    {
      sensor_data.jointPos_(i) = joint_data.joint_q[i];
      sensor_data.jointVel_(i) = joint_data.joint_v[i];
      sensor_data.jointAcc_(i) = joint_data.joint_vd[i];
      sensor_data.jointCurrent_(i) = joint_data.joint_current[i];
    }
    // std::cout << "received joint data: " << jointPos_.transpose() << std::endl;
    ros::Time ros_time = msg->header.stamp;
    sensor_data.timeStamp_ = msg->sensor_time;
    double sensor_time_diff = (ros::Time::now() - ros_time).toSec() * 1000;
    ros_logger_->publishValue("/monitor/time_cost/sensor_to_controller", sensor_time_diff);
    // IMU
    sensor_data.quat_.coeffs().w() = imu_data.quat.w;
    sensor_data.quat_.coeffs().x() = imu_data.quat.x;
    sensor_data.quat_.coeffs().y() = imu_data.quat.y;
    sensor_data.quat_.coeffs().z() = imu_data.quat.z;
    sensor_data.angularVel_ << imu_data.gyro.x, imu_data.gyro.y, imu_data.gyro.z;
    sensor_data.linearAccel_ << imu_data.acc.x, imu_data.acc.y, imu_data.acc.z;
    sensor_data.orientationCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
    sensor_data.angularVelCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();
    sensor_data.linearAccelCovariance_ << Eigen::Matrix<scalar_t, 3, 3>::Zero();

    sensor_data.linearAccel_ = acc_filter_.update(sensor_data.linearAccel_);
    // free_acc_filter_.update(sensor_data.linearAccel_);
    sensor_data.angularVel_ = gyro_filter_.update(sensor_data.angularVel_);
    ros_logger_->publishVector("/state_estimate/imu_data_filtered/linearAccel", sensor_data.linearAccel_);
    ros_logger_->publishVector("/state_estimate/imu_data_filtered/angularVel", sensor_data.angularVel_);
    // END_EFFECTOR DATA
    // sensor_data_mutex_.lock();
    // sensorDataQueue.push(sensor_data);
    // sensor_data_mutex_.unlock();
    sensors_data_buffer_ptr_->addData(sensor_data.timeStamp_.toSec(), sensor_data);

    if (headNum_ > 0 && joint_data.joint_q.size() == jointNumReal_+armNumReal_ + headNum_)
    {
      int head_start_index  = joint_data.joint_q.size() - headNum_;
      for (size_t i = 0; i < headNum_; ++i)
      {
        
        sensor_data_head_.jointPos_(i) = joint_data.joint_q[i + head_start_index];
        sensor_data_head_.jointVel_(i) = joint_data.joint_v[i + head_start_index];
        sensor_data_head_.jointAcc_(i) = joint_data.joint_vd[i + head_start_index];
        sensor_data_head_.jointCurrent_(i) = joint_data.joint_current[i + head_start_index];
      }
    }
    if (!is_initialized_)
      is_initialized_ = true;
  }
  

  bool humanoidController::enableArmTrajectoryControlCallback(kuavo_msgs::changeArmCtrlMode::Request &req, kuavo_msgs::changeArmCtrlMode::Response &res)
  {
      use_ros_arm_joint_trajectory_ = req.control_mode;
      res.result = true;
      return true;
  }

  void humanoidController::starting(const ros::Time &time)
  {
    // Initial state
    // set the initial state = {0, 0, 0, 0, 0, 0, 0, 0, 0.976, 0, 0, 0, 0, 0, 0.35, -0.90, -0.55, 0, 0, 0, 0.35, -0.90, -0.55, 0}
    // currentObservation_.state = vector_t::Zero(HumanoidInterface_->getCentroidalModelInfo().stateDim);
    // currentObservation_.state(8) = 0.78626;
    // currentObservation_.state.segment(6 + 6, jointNum_) = defalutJointPos_;
    initial_status_ = HumanoidInterface_->getInitialState();
    currentObservation_.state = initial_status_;
    std::cout << "intial state:" << currentObservation_.state.transpose() << std::endl;
    std::cout << "waitign for the first sensor data" << std::endl;
    while (!is_initialized_)
    {
      if (!is_nodelet_node_)
        ros::spinOnce();
      usleep(1000);
    }
    std::cout << "sensor data received" << std::endl;
    if (is_real_)
    {
      SensorData_t intial_sensor_data;
      hardware_interface_ptr_->init(intial_sensor_data);
      std::cout << "real robot controller starting\n";
      real_init_wait();
      std::cout << "real_init_wait done\n";
    }
    else
    {
      hardware_status_ = 1;
    }

    // applySensorsData(sensors_data_buffer_ptr_->getLastData());

    last_time_ = current_time_;
    updateStateEstimation(time, true);
    currentObservation_.input.setZero(HumanoidInterface_->getCentroidalModelInfo().inputDim);
    optimizedState2WBC_mrt_ = vector_t::Zero(centroidalModelInfoWBC_.stateDim);
    optimizedState2WBC_mrt_.head(centroidalModelInfo_.stateDim) = currentObservation_.state;
    std::cout << "initial state: " << currentObservation_.state.transpose() << std::endl;
    optimizedInput2WBC_mrt_ = vector_t::Zero(centroidalModelInfoWBC_.inputDim);
    optimizedInput2WBC_mrt_.head(centroidalModelInfo_.inputDim) = currentObservation_.input;

    currentObservation_.mode = ModeNumber::SS;
    SystemObservation initial_observation = currentObservation_;
    initial_observation.state = initial_status_;
    TargetTrajectories target_trajectories({initial_observation.time}, {initial_observation.state}, {initial_observation.input});

    // Set the first observation and command and wait for optimization to finish
    ROS_INFO_STREAM("Waiting for the initial policy ...");
    if (use_external_mpc_)
    {
      // Reset MPC node
      mrtRosInterface_->resetMpcNode(target_trajectories);
      std::cout << "reset MPC node\n";
      // Wait for the initial policy
      while (!mrtRosInterface_->initialPolicyReceived() && ros::ok() && ros::master::check())
      {
        mrtRosInterface_->spinMRT();
        mrtRosInterface_->setCurrentObservation(initial_observation);
        ros::Rate(HumanoidInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
      }
    }
    else
    {
      mpcMrtInterface_->setCurrentObservation(initial_observation);
      mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
      ROS_INFO_STREAM("Waiting for the initial policy ...");
      while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok())
      {
        mpcMrtInterface_->advanceMpc();
        ros::WallRate(HumanoidInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
      }
    }

    intail_input_ = vector_t::Zero(centroidalModelInfoWBC_.inputDim);
    for (int i = 0; i < 8; i++)
      intail_input_(3 * i + 2) = centroidalModelInfoWBC_.robotMass * 9.81 / 8; // 48.7*g/8
    optimizedInput2WBC_mrt_ = intail_input_;

    if (is_simplified_model_)
    {
      optimizedState2WBC_mrt_.head(centroidalModelInfo_.stateDim) = currentObservation_.state;
      optimizedState2WBC_mrt_.tail(armNumReal_).setZero();

      for (int i = 0; i < 2; i++)
      {
        optimizedState2WBC_mrt_.segment(12 + jointNum_ + i * armDofReal_, armDofMPC_) = optimizedState2WBC_mrt_.segment(12 + jointNum_ + i * armDofMPC_, armDofMPC_);
      }
    }

    // else
    // {
    //   mpcMrtInterface_->setCurrentObservation(currentObservation_);
    //   mpcMrtInterface_->getReferenceManager().setTargetTrajectories(target_trajectories);
    //   while (!mpcMrtInterface_->initialPolicyReceived() && ros::ok())
    //   {
    //     mpcMrtInterface_->advanceMpc();
    //     ros::WallRate(HumanoidInterface_->mpcSettings().mrtDesiredFrequency_).sleep();
    //   }
    // }
    ROS_INFO_STREAM("Initial policy has been received.");
    // usleep(1000); // wait for 1s to ensure that the initial policy is received by the MPC node
    if (!is_real_ && !is_play_back_mode_)
      mujocoSimStart(controllerNh_);
    // if (is_real_)
    // {
    //   SensorData_t intial_sensor_data;
    //   hardware_interface_ptr_->init(intial_sensor_data);
    //   std::cout << "real robot controller starting\n";
    //   real_init_wait();
    //   std::cout << "real_init_wait done\n";
    // }
    // current_time_ = ros::Time::now();
    last_time_ = current_time_;
    if (!is_play_back_mode_)
      sensors_data_buffer_ptr_->sync();

    std::cout << "starting the controller" << std::endl;
    mpcRunning_ = true;
  }
  void humanoidController::real_init_wait()
  {
    while (ros::ok())
    {
      if (ros::param::get("/hardware/is_ready", hardware_status_))
      {
        if (hardware_status_ == 1)
        {
          std::cerr << "real robot is ready\n";
          break;
        }
      }
      usleep(1000);
    }
    
  }
  void humanoidController::update(const ros::Time &time, const ros::Duration &dfd)
  {
    const auto t1 = Clock::now();

    if (reset_mpc_)// 重置mpc
    {
      currentObservation_.input.setZero(HumanoidInterface_->getCentroidalModelInfo().inputDim);
      auto target_trajectories = TargetTrajectories({currentObservation_.time}, {currentObservation_.state}, {currentObservation_.input});
      mrtRosInterface_->resetMpcNode(target_trajectories);
      reset_mpc_ = false;
      std::cout << "reset MPC node at "<<currentObservation_.time << "\n";
    }
    // kuavo_msgs::sensorsData msg = sensors_data_buffer_ptr_->getNextData();
    // // kuavo_msgs::sensorsData msg = sensors_data_buffer_ptr_->getData(ros::Time::now().toSec());
    // applySensorsData(msg);
    // State Estimate
    ros::Duration period = ros::Duration(dt_);
    updateStateEstimation(time, false);
    const auto t2 = Clock::now();

    auto& info = centroidalModelInfo_;
    auto& infoWBC = centroidalModelInfoWBC_;
    vector_t optimizedState_mrt, optimizedInput_mrt;
    bool is_mpc_updated = false;
    if (use_external_mpc_)
    {
      // Update the current state of the system
      mrtRosInterface_->setCurrentObservation(currentObservation_);
      // Trigger MRT callbacks
      mrtRosInterface_->spinMRT();
      // Update the policy if a new on was received
      if (mrtRosInterface_->updatePolicy())
      {
        is_mpc_updated = true;
        auto &policy = mrtRosInterface_->getPolicy();
        auto &state_trajectory = policy.stateTrajectory_;
        // trajectory_publisher_->publishTrajectory(state_trajectory);
        TargetTrajectories target_trajectories(policy.timeTrajectory_, policy.stateTrajectory_, policy.inputTrajectory_);

        publishFeetTrajectory(target_trajectories);
      }
      mrtRosInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState_mrt, optimizedInput_mrt, plannedMode_);
      
    }
    else
    {
      // Update the current state of the system
      mpcMrtInterface_->setCurrentObservation(currentObservation_);

      // Load the latest MPC policy
      if (mpcMrtInterface_->updatePolicy())
      {
        is_mpc_updated = true;
        auto &policy = mpcMrtInterface_->getPolicy();
        auto &command = mpcMrtInterface_->getCommand();
        auto &performance_indices = mpcMrtInterface_->getPerformanceIndices();
        auto &state_trajectory = policy.stateTrajectory_;

        ocs2_msgs::mpc_flattened_controller mpcPolicyMsg =
            createMpcPolicyMsg(policy, command, performance_indices);

        // publish the message
        mpcPolicyPublisher_.publish(mpcPolicyMsg);

      }

      // Evaluate the current policy
      mpcMrtInterface_->evaluatePolicy(currentObservation_.time, currentObservation_.state, optimizedState_mrt, optimizedInput_mrt, plannedMode_);
      observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
    }
    // std::cout << "optimizedState_mrt:" << optimizedState_mrt.transpose() << " \noptimizedInput_mrt:" << optimizedInput_mrt.transpose() << " plannedMode_:" << plannedMode_ << std::endl;
    ros_logger_->publishVector("/humanoid_controller/optimizedState_mrt_origin", optimizedState_mrt);
    ros_logger_->publishVector("/humanoid_controller/optimizedInput_mrt_origin", optimizedInput_mrt);
    if (wbc_only_)
    {
      optimizedState_mrt = initial_status_;
      optimizedInput_mrt = intail_input_;
    }

    if (is_simplified_model_)
    {
      // 躯干和腿部target
      optimizedState2WBC_mrt_.head(info.stateDim) = optimizedState_mrt;
      optimizedInput2WBC_mrt_.head(info.inputDim) = optimizedInput_mrt;
      optimizedState2WBC_mrt_.tail(armNumReal_).setZero();
      optimizedInput2WBC_mrt_.tail(armNumReal_).setZero();

      // 手臂target前半部分
      for (int i = 0; i < 2; i++)
      {
        optimizedState2WBC_mrt_.tail(armNumReal_).segment(i * armDofReal_, armDofMPC_) =
            optimizedState_mrt.tail(armNum_).segment(i * armDofMPC_, armDofMPC_);
        optimizedInput2WBC_mrt_.tail(armNumReal_).segment(i * armDofReal_, armDofMPC_) =
            optimizedInput_mrt.tail(armNum_).segment(i * armDofMPC_, armDofMPC_);
      }

      // 手臂target后半部分，从arm_joint_trajectory_获取
      auto target_arm_pos = currentArmTargetTrajectories_.getDesiredState(currentObservation_.time);
      if (target_arm_pos.size() == armNumReal_)
      {
        for (int i = 0; i < 2; i++)
        {
          optimizedState2WBC_mrt_.tail(armNumReal_).segment(i * armDofReal_ + armDofMPC_, armDofDiff_) =
              target_arm_pos.segment(i * armDofReal_ + armDofMPC_, armDofDiff_);
        }
      }
    }
    else
    {
      optimizedState2WBC_mrt_ = optimizedState_mrt;
      optimizedInput2WBC_mrt_ = optimizedInput_mrt;
      
    }
    currentObservation_.input = optimizedInput_mrt;// 传什么值都一样, MPC不使用obs.input


    // // use filter output
    optimizedState2WBC_mrt_.tail(armNumReal_) = arm_joint_pos_filter_.update(optimizedState2WBC_mrt_.tail(armNumReal_));
    optimizedInput2WBC_mrt_.tail(armNumReal_) = arm_joint_vel_filter_.update(optimizedInput2WBC_mrt_.tail(armNumReal_));
    // optimizedInput2WBC_mrt_.segment(optimizedInput_mrt.size() - info.actuatedDofNum, jointNum_) = mrt_joint_vel_filter_.update(optimizedInput_mrt.segment(optimizedInput_mrt.size() - info.actuatedDofNum, jointNum_));
    // ros_logger_->publishVector("/humanoid_controller/optimizedInput_mrt_filtered", optimizedInput2WBC_mrt_);
    
    // // use ik output 
    // vector_t filtered_arm_pose = arm_joint_pos_filter_.update(arm_joint_trajectory_.pos);
    // optimizedState2WBC_mrt_.tail(armNum_) = filtered_arm_pose;
    // vector_t filter_input_vel = (filtered_arm_pose- arm_joint_pos_cmd_prev_)/dt_;
    // optimizedInput2WBC_mrt_.tail(armNum_) = arm_joint_vel_filter_.update(filter_input_vel);
    // arm_joint_pos_cmd_prev_ = filtered_arm_pose;
  
    
    
    if(use_ros_arm_joint_trajectory_)
    {
      // TODO: feedback in planner
      // auto arm_pos = currentObservation_.state.tail(armNum_); 
      // optimizedInput2WBC_mrt_.tail(armNum_) = 0.05 * (arm_joint_trajectory_.pos - arm_pos)/dt_;
      // optimizedState2WBC_mrt_.tail(armNum_) = arm_pos + optimizedInput2WBC_mrt_.tail(armNum_) * dt_;
      //直接覆盖mpc每只手臂末3位角度
      optimizedState2WBC_mrt_.segment<3>(24+4) = arm_joint_trajectory_.pos.segment<3>(4);
      optimizedState2WBC_mrt_.segment<3>(24+7+4) = arm_joint_trajectory_.pos.segment<3>(7+4);
      // std::cout << "target_arm_joint_pos[0]: " << arm_joint_trajectory_.pos[0] << std::endl;
    }
    // for(int i=0;i<info.actuatedDofNum;i++)
    // {
    //   optimizedState2WBC_mrt_(12+i) = std::max(joint_state_limit_(i, 0), std::min(optimizedState2WBC_mrt_[12+i], joint_state_limit_(i, 1)));
    // }
     
    optimized_mode_ = plannedMode_;
    // currentObservation_.input.tail(info.actuatedDofNum) = measuredRbdState_.tail(info.actuatedDofNum);

    // Whole body control
    // wbc_->setStanceMode(currentObservation_.mode == ModeNumber::SS);

    auto contactFlag_ = modeNumber2StanceLeg(currentObservation_.mode);
    bool lf_contact = std::any_of(contactFlag_.begin(), contactFlag_.begin() + 4, [](int flag)
                                  { return flag; });
    bool rf_contact = std::any_of(contactFlag_.begin() + 4, contactFlag_.end(), [](int flag)
                                  { return flag; });
    if (lf_contact && rf_contact)
    {
      wbc_->setStanceMode(true);
    }
    else
    {
      wbc_->setStanceMode(false);
    }
    wbcTimer_.startTimer();
    const auto t3 = Clock::now();
    for(int i=0;i<infoWBC.numThreeDofContacts;i++)
    {
      ros_logger_->publishVector("/humanoid_controller/optimizedInput_mrt/force_" + std::to_string(i+1), optimizedInput2WBC_mrt_.segment(3 * i, 3));
    }
    ros_logger_->publishVector("/humanoid_controller/optimizedInput_mrt/joint_vel", optimizedInput2WBC_mrt_.segment(3 * infoWBC.numThreeDofContacts, infoWBC.actuatedDofNum));
    
    ros_logger_->publishVector("/humanoid_controller/optimizedState_mrt/com/linear_vel_xyz", optimizedState2WBC_mrt_.head<3>());
    ros_logger_->publishVector("/humanoid_controller/optimizedState_mrt/com/angular_vel_xyz", optimizedState2WBC_mrt_.segment<3>(3));
    ros_logger_->publishVector("/humanoid_controller/optimizedState_mrt/base/pos_xyz", optimizedState2WBC_mrt_.segment<3>(6));
    ros_logger_->publishVector("/humanoid_controller/optimizedState_mrt/base/angular_zyx", optimizedState2WBC_mrt_.segment<3>(9));
    ros_logger_->publishVector("/humanoid_controller/optimizedState_mrt/joint_pos", optimizedState2WBC_mrt_.segment(12, infoWBC.actuatedDofNum));
    ros_logger_->publishValue("/humanoid_controller/optimized_mode", static_cast<double>(optimized_mode_));


    
    // *************************** WBC **********************************
    vector_t x = wbc_->update(optimizedState2WBC_mrt_, optimizedInput2WBC_mrt_, measuredRbdStateReal_, plannedMode_, period.toSec(), is_mpc_updated);
    // wbc_->updateVd(jointAcc_);
    const auto t4 = Clock::now();
    wbcTimer_.endTimer();

    // 决策变量, 6*body_acc + 12*joint_acc + 3x4*contact_force + 12*torque = 42
    vector_t torque = x.tail(infoWBC.actuatedDofNum);
    const vector_t &wbc_planned_joint_acc = x.segment(6, infoWBC.actuatedDofNum);
    const vector_t &wbc_planned_body_acc = x.head(6);
    // std::cout << "wbc_planned_joint_acc:" << wbc_planned_joint_acc.transpose() << std::endl;
    // std::cout << "wbc_planned_body_acc:" << wbc_planned_body_acc.transpose() << std::endl;
    const vector_t &wbc_planned_contact_force = x.segment(6 + infoWBC.actuatedDofNum, wbc_->getContactForceSize());
    // std::cout << "wbc_planned_contact_force:" << wbc_planned_contact_force.transpose() << std::endl;
    // std::cout << "torque:" << torque.transpose() << std::endl;
    ros_logger_->publishVector("/humanoid_controller/torque", torque);
    ros_logger_->publishVector("/humanoid_controller/wbc_planned_joint_acc", wbc_planned_joint_acc);
    ros_logger_->publishVector("/humanoid_controller/wbc_planned_body_acc/linear", wbc_planned_body_acc.head<3>());
    ros_logger_->publishVector("/humanoid_controller/wbc_planned_body_acc/angular", wbc_planned_body_acc.tail<3>());
    ros_logger_->publishVector("/humanoid_controller/wbc_planned_contact_force/left_foot", wbc_planned_contact_force.head<12>());
    ros_logger_->publishVector("/humanoid_controller/wbc_planned_contact_force/right_foot", wbc_planned_contact_force.tail<12>());
    // std::cout << "wbc_planned_contact_force:" << wbc_planned_contact_force.transpose() << std::endl;

    vector_t posDes = centroidal_model::getJointAngles(optimizedState2WBC_mrt_, infoWBC);
    vector_t velDes = centroidal_model::getJointVelocities(optimizedInput2WBC_mrt_, infoWBC);

    scalar_t dt = period.toSec();
    // bool is_joint_acc_out_of_range = wbc_planned_joint_acc.array().abs().maxCoeff() > 2000;
    // if (is_joint_acc_out_of_range)
    // {
    //   std::cerr << "wbc_planned_joint_acc is out of range, reset it to zero." << std::endl;
    //   std::cerr << "wbc_planned_joint_acc: " << wbc_planned_joint_acc.transpose() << std::endl;
    //   torque = output_tau_;
    // }
    // else
    {
      posDes = posDes + 0.5 * wbc_planned_joint_acc * dt * dt;
      velDes = velDes + wbc_planned_joint_acc * dt;
    }
    // ros_logger_->publishVector("/humanoid_controller/posDes", posDes);
    // ros_logger_->publishVector("/humanoid_controller/velDes", velDes);
    // ***************************** WBC END **********************************


    // Safety check, if failed, stop the controller
    if (!safetyChecker_->check(currentObservation_, optimizedState_mrt, optimizedInput_mrt))
    {
      ROS_ERROR_STREAM("[humanoid Controller] Safety check failed, stopping the controller.");
      std_msgs::Bool stop_msg;
      stop_msg.data = true;
      stop_pub_.publish(stop_msg);
      usleep(100000);
      // TODO: send the stop command to hardware interface
      return;
    }


    {
      output_pos_ = posDes;
      output_vel_ = velDes;
      output_tau_ = torque;
    }
    vector_t kp_ = joint_kp_, kd_ = joint_kd_;
    if (currentObservation_.mode != ModeNumber::SS)
    {
      kp_ = joint_kp_walking_;
      kd_ = joint_kd_walking_;
    }


    const auto t5 = Clock::now();

    kuavo_msgs::jointCmd jointCmdMsg;
    jointCmdMsg.header.stamp = time;
    // std::cout << "jointNum_:  " << jointNum_+armNum_ << "\n\n";
    for (int i1 = 0; i1 < jointNumReal_; ++i1)
    {
      jointCmdMsg.joint_q.push_back(output_pos_(i1));
      jointCmdMsg.joint_v.push_back(output_vel_(i1));
      jointCmdMsg.tau.push_back(output_tau_(i1));
      jointCmdMsg.tau_ratio.push_back(1);
      jointCmdMsg.joint_kp.push_back(joint_kp_[i1]);
      jointCmdMsg.joint_kd.push_back(joint_kd_[i1]);
      jointCmdMsg.tau_max.push_back(kuavo_settings_.hardware_settings.max_current[i1]);
      jointCmdMsg.control_modes.push_back(joint_control_modes_[i1]);
    }

    ModeSchedule current_mode_schedule = mrtRosInterface_->getCurrentModeSchedule();
    auto is_SS_mode_after = [&](const ModeSchedule &mode_schedule) { // 后续都是SS mode
      int start_index = mode_schedule.modeBeforeId(currentObservation_.time);
      for (int i1 = start_index + 1; i1 < mode_schedule.modeSequence.size(); ++i1)
      {
        if (mode_schedule.modeSequence[i1] != ModeNumber::SS)
        {
          return false;
        }
      }
      return true;
    };
    auto is_walking_gait = [&](const std::string &gait_name)
    {
      return gait_name == "walk" || gait_name == "trot";
    };

    bool is_stance_mode_ = is_SS_mode_after(current_mode_schedule);

    // 膝关节全程力控

    const auto &current_time = currentObservation_.time - dt_;
    size_t current_mode = currentObservation_.mode;
    size_t before_mode = current_mode_schedule.modeBefore(current_time);
    size_t next_mode = current_mode_schedule.modeNext(current_time);
    double switch_time = current_mode_schedule.timeSwitch(current_time);
    double start_time = current_mode_schedule.timeBefore(current_time);
    size_t be_before_mode = current_mode_schedule.modeBefore(start_time - dt_); // 前前一个mode

    bool to_double_contact = current_mode == ModeNumber::SS && before_mode != ModeNumber::SS;
    bool lf_heel_off_contact = current_mode == ModeNumber::TS && current_time < start_time + contact_cst_et_ && be_before_mode != ModeNumber::SS;
    bool rf_heel_off_contact = current_mode == ModeNumber::ST && current_time < start_time + contact_cst_et_ && be_before_mode != ModeNumber::SS;

    if (((current_mode == ModeNumber::SF || current_mode == ModeNumber::FS) && current_time >= switch_time - contact_cst_st_) ||
        ((current_mode == ModeNumber::SF || current_mode == ModeNumber::FS) && current_time <= start_time + contact_cst_et_) ||
        current_mode == ModeNumber::SH || current_mode == ModeNumber::TS ||
        current_mode == ModeNumber::HS || current_mode == ModeNumber::ST || to_double_contact)
    {
      jointCmdMsg.joint_kp[3] = joint_kp_walking_[3];
      jointCmdMsg.joint_kp[9] = joint_kp_walking_[9];
      jointCmdMsg.joint_kd[3] = joint_kd_walking_[3];
      jointCmdMsg.joint_kd[9] = joint_kd_walking_[9];
    }

    // 踝关节全程力控+pd
    jointCmdMsg.control_modes[4] = 0;
    jointCmdMsg.control_modes[5] = 0;
    jointCmdMsg.control_modes[10] = 0;
    jointCmdMsg.control_modes[11] = 0;
    if (!is_stance_mode_)
    {
      if (std::any_of(contactFlag_.begin(), contactFlag_.begin() + 4, [](int flag)
                      { return !flag; }))
      {
        jointCmdMsg.joint_kp[4] = joint_kp_walking_[4];
        jointCmdMsg.joint_kp[5] = joint_kp_walking_[5];
        jointCmdMsg.joint_kd[4] = joint_kd_walking_[4];
        jointCmdMsg.joint_kd[5] = joint_kd_walking_[5];
      }

      if (std::any_of(contactFlag_.begin() + 4, contactFlag_.end(), [](int flag)
                      { return !flag; }))
      {
        jointCmdMsg.joint_kp[10] = joint_kp_walking_[10];
        jointCmdMsg.joint_kp[11] = joint_kp_walking_[11];
        jointCmdMsg.joint_kd[10] = joint_kd_walking_[10];
        jointCmdMsg.joint_kd[11] = joint_kd_walking_[11];
      }
    }
    else
    {
      jointCmdMsg.joint_kp[4] = 0.0;
      jointCmdMsg.joint_kp[5] = 0.0;
      jointCmdMsg.joint_kd[4] = 0.0;
      jointCmdMsg.joint_kd[5] = 0.0;
      jointCmdMsg.joint_kp[10] = 0.0;
      jointCmdMsg.joint_kp[11] = 0.0;
      jointCmdMsg.joint_kd[10] = 0.0;
      jointCmdMsg.joint_kd[11] = 0.0;
    }

    // 补全手臂的Cmd维度
    for(int i2 = 0; i2 < armNumReal_; ++i2)
    {
      jointCmdMsg.joint_q.push_back(output_pos_(jointNum_+i2));
      jointCmdMsg.joint_v.push_back(output_vel_(jointNum_+i2));
      jointCmdMsg.tau.push_back(output_tau_(jointNum_+i2));
      jointCmdMsg.tau_ratio.push_back(1);
      jointCmdMsg.tau_max.push_back(kuavo_settings_.hardware_settings.max_current[jointNum_+i2]);
      jointCmdMsg.control_modes.push_back(joint_control_modes_[jointNum_+i2]);
    }

    // 补充头部维度
    // 计算头部反馈力
    if (headNum_ > 0)
    {
      vector_t get_head_pos = vector_t::Zero(headNum_);
      head_mtx.lock();
      get_head_pos = desire_head_pos_;
      head_mtx.unlock();
      auto &hardware_settings = kuavo_settings_.hardware_settings;
      vector_t head_feedback_tau = vector_t::Zero(headNum_);
      vector_t head_feedback_vel = vector_t::Zero(headNum_);
      if (!is_real_) // 实物不需要头部反馈力，来自kuavo仓库的移植
        head_feedback_tau = head_kp_.cwiseProduct(get_head_pos - sensor_data_head_.jointPos_) + head_kd_.cwiseProduct(-sensor_data_head_.jointVel_);
      for (int i3 = 0; i3 < headNum_; ++i3)
      {
        auto cur_head_pos = sensor_data_head_.jointPos_ * TO_DEGREE;
        auto vel = (get_head_pos[i3] - sensor_data_head_.jointPos_[i3]) * TO_DEGREE / dt_ * ruiwo_motor_velocities_factor_;
        double head_limit_vel = hardware_settings.joint_velocity_limits[jointNum_ + armNumReal_ + i3];

        vel = std::clamp(vel, -head_limit_vel, head_limit_vel) * TO_RADIAN;
        jointCmdMsg.joint_q.push_back(get_head_pos(i3));
        jointCmdMsg.joint_v.push_back(0);
        jointCmdMsg.tau.push_back(head_feedback_tau(i3));
        jointCmdMsg.tau_ratio.push_back(1);
        jointCmdMsg.tau_max.push_back(10);
        jointCmdMsg.control_modes.push_back(2);
      }
      robotVisualizer_->updateHeadJointPositions(sensor_data_head_.jointPos_);
    }

    jointCmdPub_.publish(jointCmdMsg);
    
    // Visualization
    robotVisualizer_->updateSimplifiedArmPositions(simplifiedJointPos_);
    if (use_external_mpc_)
      robotVisualizer_->update(currentObservation_, mrtRosInterface_->getPolicy(), mrtRosInterface_->getCommand());
    else
      robotVisualizer_->update(currentObservation_, mpcMrtInterface_->getPolicy(), mpcMrtInterface_->getCommand());

    // Publish the observation. Only needed for the command interface
    // observationPublisher_.publish(ros_msg_conversions::createObservationMsg(currentObservation_));
    const auto t6 = Clock::now();
    if (std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t1).count() > 1000)
    {
      std::cout << "t1-t2: " << std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " ms" << std::endl;
      std::cout << "t2-t3: " << std::chrono::duration_cast<std::chrono::milliseconds>(t3 - t2).count() << " ms" << std::endl;
      std::cout << "t3-t4: " << std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count() << " ms" << std::endl;
      std::cout << "t4-t5: " << std::chrono::duration_cast<std::chrono::milliseconds>(t5 - t4).count() << " ms" << std::endl;
      std::cout << "t5-t6: " << std::chrono::duration_cast<std::chrono::milliseconds>(t6 - t5).count() << " ms" << std::endl;
    }
    // publish time cost
    std_msgs::Float64 msg;
    msg.data = wbcTimer_.getFrequencyInHz();
    wbcFrequencyPub_.publish(msg);
    msg.data = wbcTimer_.getLastIntervalInMilliseconds();
    wbcTimeCostPub_.publish(msg);

    static double last_ros_time = ros::Time::now().toSec();
    ros_logger_->publishValue("/monitor/time_cost/controller_loop_time", (ros::Time::now().toSec() - last_ros_time) * 1000);
    last_ros_time = ros::Time::now().toSec();
    lastObservation_ = currentObservation_;
  }
  void humanoidController::applySensorData()
  {
    if (!sensorDataQueue.empty())
    {
      sensor_data_mutex_.lock();
      while (sensorDataQueue.size() > 10)
      {
        sensorDataQueue.pop();
        // ROS_WARN_STREAM("Sensor data queue size exceeds 10, pop one element");
      }
      SensorData data = sensorDataQueue.front();
      sensorDataQueue.pop();
      sensor_data_mutex_.unlock();

      applySensorData(data);
    }
  }
  
  void humanoidController::applySensorData(const SensorData &data)
  {
    if (is_simplified_model_)// 简化模型, 需要将实物维度转为MPC维度
    {
      jointPos_.head(jointNum_) = data.jointPos_.head(jointNum_);
      jointVel_.head(jointNum_) = data.jointVel_.head(jointNum_);
      jointAcc_.head(jointNum_) = data.jointAcc_.head(jointNum_);
      jointCurrent_.head(jointNum_) = data.jointCurrent_.head(jointNum_);

      for (int i = 0; i < 2; i++)
      {

        jointPos_.segment(jointNum_ + armDofMPC_ * i, armDofMPC_) = data.jointPos_.segment(jointNum_ + armDofReal_ * i, armDofMPC_);
        jointVel_.segment(jointNum_ + armDofMPC_ * i, armDofMPC_) = data.jointVel_.segment(jointNum_ + armDofReal_ * i, armDofMPC_);
        jointAcc_.segment(jointNum_ + armDofMPC_ * i, armDofMPC_) = data.jointAcc_.segment(jointNum_ + armDofReal_ * i, armDofMPC_);
        jointCurrent_.segment(jointNum_ + armDofMPC_ * i, armDofMPC_) = data.jointCurrent_.segment(jointNum_ + armDofReal_ * i, armDofMPC_);
        simplifiedJointPos_.segment(armDofDiff_ * i, armDofDiff_) = data.jointPos_.segment(jointNum_ + armDofReal_ * i + armDofMPC_, armDofDiff_);
      }
    }
    else
    {
      jointPos_ = data.jointPos_;
      jointVel_ = data.jointVel_;
      jointAcc_ = data.jointAcc_;
      jointCurrent_ = data.jointCurrent_;
    }

    quat_ = data.quat_;
    angularVel_ = data.angularVel_;
    linearAccel_ = data.linearAccel_;
    orientationCovariance_ = data.orientationCovariance_;
    angularVelCovariance_ = data.angularVelCovariance_;
    linearAccelCovariance_ = data.linearAccelCovariance_;
    current_time_ = data.timeStamp_;
    // stateEstimate_->updateJointStates(jointPos_, jointVel_);
    stateEstimate_->updateImu(quat_, angularVel_, linearAccel_, orientationCovariance_, angularVelCovariance_, linearAccelCovariance_);
  }
  void humanoidController::updateStateEstimation(const ros::Time &time, bool is_init)
  {
    if (reset_estimator_)
    {
      stateEstimate_->reset();
      reset_estimator_ = false;
    }
    // vector_t jointPos(jointNum_+armNum_), jointVel(jointNum_+armNum_), jointCurrent(jointNum_+armNum_);
    // contact_flag_t contacts;
    // Eigen::Quaternion<scalar_t> quat;
    contact_flag_t contactFlag;
    // vector3_t angularVel, linearAccel;
    // matrix3_t orientationCovariance, angularVelCovariance, linearAccelCovariance;
    SensorData sensors_data;
    if (is_init)
      sensors_data = sensors_data_buffer_ptr_->getLastData();
    else
      sensors_data = sensors_data_buffer_ptr_->getLastData();
    // SensorData &sensors_data = sensors_data_buffer_ptr_->getData(ros::Time::now().toSec());
    applySensorData(sensors_data);

    // TODO: get contactFlag from hardware interface
    // 暂时用plannedMode_代替，需要在接触传感器可靠之后修改为stateEstimate_->getMode()
    // contactFlag = modeNumber2StanceLeg(plannedMode_);
    if (is_init)
    {
      last_time_ = current_time_ - ros::Duration(0.001);
      stateEstimate_->updateJointStates(jointPos_, jointVel_);
      stateEstimate_->updateIntialEulerAngles(quat_);
      applySensorData(sensors_data);
      stateEstimate_->set_intial_state(currentObservation_.state);
      measuredRbdState_ = stateEstimate_->getRbdState();
      // std::cout << "initial measuredRbdState_:" << measuredRbdState_.transpose() << std::endl;
      // clear the sensor data queue
      // sensor_data_mutex_.lock();
      // while (!sensorDataQueue.empty())
      //   sensorDataQueue.pop();
      // sensor_data_mutex_.unlock();
    }
    // last_time_ = current_time_ - ros::Duration(0.002);
    double diff_time = (current_time_ - last_time_).toSec();
    auto est_mode = stateEstimate_->ContactDetection(plannedMode_, jointVel_, jointCurrent_, diff_time);
    ros_logger_->publishValue("/state_estimate/mode", static_cast<double>(est_mode));
    est_mode = plannedMode_;
    // contactFlag = modeNumber2StanceLeg(est_mode);
    // std::cout << "mode: " << modeNumber2String(est_mode) << std::endl;
    last_time_ = current_time_;
    ros::Duration period = ros::Duration(diff_time);

    vector_t activeTorque_ = (is_real_) ? jointCurrent_.cwiseProduct(motor_c2t_) : jointCurrent_;
    stateEstimate_->setCmdTorque(activeTorque_);
    stateEstimate_->estContactForce(period);
    auto est_contact_force = stateEstimate_->getEstContactForce();
    ros_logger_->publishVector("/state_estimate/contact_force", est_contact_force);

    stateEstimate_->updateMode(est_mode);
    stateEstimate_->updateGait(gaitManagerPtr_->getGaitName(currentObservation_.time));
    // rbdState_: Angular(zyx),pos(xyz),jointPos[info_.actuatedDofNum],angularVel(zyx),linervel(xyz),jointVel[info_.actuatedDofNum]
    if (diff_time > 0.00005 || is_init)
    {
      Eigen::VectorXd updated_joint_pos = jointPos_;
      Eigen::VectorXd updated_joint_vel = jointVel_;
      Eigen::VectorXd updated_joint_current = jointCurrent_;
#ifdef KUAVO_CONTROL_LIB_FOUND
      if (use_joint_filter_)
      {
        joint_filter_ptr_->update(measuredRbdState_, updated_joint_pos, updated_joint_vel, updated_joint_current, output_tau_, est_mode);
      }
#endif
      stateEstimate_->updateJointStates(updated_joint_pos, updated_joint_vel); // 使用关节滤波之后的jointPos和jointVel更新状态估计器
      measuredRbdState_ = stateEstimate_->update(time, period);                // angle(zyx),pos(xyz),jointPos[info_.actuatedDofNum],angularVel(zyx),linervel(xyz),jointVel[info_.actuatedDofNum]
      currentObservation_.time += period.toSec();
    }
    ros_logger_->publishVector("/state_estimate/measuredRbdState", measuredRbdState_);
    auto &info = HumanoidInterface_->getCentroidalModelInfo();


    scalar_t yawLast = currentObservation_.state(9);
    currentObservation_.state = rbdConversions_->computeCentroidalStateFromRbdModel(measuredRbdState_);
    currentObservation_.state(9) = yawLast + angles::shortest_angular_distance(yawLast, currentObservation_.state(9));
    std_msgs::Float32MultiArray state;
    for (int i1 = 0; i1 < currentObservation_.state.rows(); ++i1)
    {
      state.data.push_back(currentObservation_.state(i1));
    }
    // RbdStatePub_.publish(state);
    // std::cout << "currentObservation_.state:" << currentObservation_.state.transpose() << std::endl;
    // currentObservation_.mode = stateEstimate_->getMode();
    // std::cout << "currentObservation_.mode:" << currentObservation_.mode << std::endl;
    // TODO: 暂时用plannedMode_代替，需要在接触传感器可靠之后修改为stateEstimate_->getMode()
    // currentObservation_.mode = plannedMode_;
    currentObservation_.mode = est_mode;
    if (is_simplified_model_)
    {
      measuredRbdStateReal_.resize(centroidalModelInfoWBC_.generalizedCoordinatesNum*2);

      for (int i = 0; i < 2; i++)// qv
      {
        // 躯干+腿部自由度
        measuredRbdStateReal_.segment(centroidalModelInfoWBC_.generalizedCoordinatesNum * i, 6 + jointNum_) =
            measuredRbdState_.segment(info.generalizedCoordinatesNum * i, 6 + jointNum_);

        // 共有的手臂关节
        int arm_start_index = centroidalModelInfoWBC_.generalizedCoordinatesNum * i + 6 + jointNum_;
        int arm_start_index_mpc = info.generalizedCoordinatesNum * i + 6 + jointNum_ ;
        for (int j = 0; j < 2; j++) // 左右手
        {
          measuredRbdStateReal_.segment(arm_start_index + armDofReal_ * j, armDofMPC_) =
              measuredRbdState_.segment(arm_start_index_mpc + armDofMPC_ * j, armDofMPC_);

          // 简化的手臂关节部分从传感器数据获取
          vector_t joint_qv(sensors_data.jointPos_.size() * 2);
          joint_qv << sensors_data.jointPos_, sensors_data.jointVel_;
          int sensors_joint_num = sensors_data.jointPos_.size();
          measuredRbdStateReal_.segment(arm_start_index + armDofReal_ * j + armDofMPC_, armDofDiff_) =
              joint_qv.segment(sensors_joint_num * i + jointNum_ + armDofReal_ * j + armDofMPC_, armDofDiff_);
        }
      }
    }
    else
    {
      measuredRbdStateReal_ = measuredRbdState_;
    }
  }

  humanoidController::~humanoidController()
  {
    controllerRunning_ = false;
    if (mpcThread_.joinable())
    {
      mpcThread_.join();
    }
    std::cerr << "########################################################################";
    std::cerr << "\n### MPC Benchmarking";
    std::cerr << "\n###   Maximum : " << mpcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << mpcTimer_.getAverageInMilliseconds() << "[ms]." << std::endl;
    std::cerr << "########################################################################";
    std::cerr << "\n### WBC Benchmarking";
    std::cerr << "\n###   Maximum : " << wbcTimer_.getMaxIntervalInMilliseconds() << "[ms].";
    std::cerr << "\n###   Average : " << wbcTimer_.getAverageInMilliseconds() << "[ms].";
  }

  void humanoidController::setupHumanoidInterface(const std::string &taskFile, const std::string &urdfFile, const std::string &referenceFile, const std::string &gaitCommandFile,
                                                  bool verbose, int robot_version_int)
  {
    HumanoidInterface_ = std::make_shared<HumanoidInterface>(taskFile, urdfFile, referenceFile, gaitCommandFile, robot_version_int);
    rbdConversions_ = std::make_shared<CentroidalModelRbdConversions>(HumanoidInterface_->getPinocchioInterface(),
                                                                      HumanoidInterface_->getCentroidalModelInfo());
    // **************** create the centroidal model for WBC ***********
    // PinocchioInterface
    auto &modelSettings_ = HumanoidInterface_->modelSettings();
    pinocchioInterfaceWBCPtr_.reset(new PinocchioInterface(centroidal_model::createPinocchioInterface(urdfFile, modelSettings_.jointNamesReal)));

    vector_t defaultJointState(pinocchioInterfaceWBCPtr_->getModel().nq - 6);
    defaultJointState.setZero();
    auto drake_interface_ = HighlyDynamic::HumanoidInterfaceDrake::getInstancePtr(RobotVersion(robot_version_int / 10, robot_version_int % 10), true, 2e-3);
    defaultJointState.head(jointNum_) = drake_interface_->getDefaultJointState();

    // CentroidalModelInfo
    centroidalModelInfoWBC_ = centroidal_model::createCentroidalModelInfo(
        *pinocchioInterfaceWBCPtr_, centroidal_model::loadCentroidalType(taskFile), defaultJointState, modelSettings_.contactNames3DoF,
        modelSettings_.contactNames6DoF);
    CentroidalModelPinocchioMapping pinocchioMapping(centroidalModelInfoWBC_);

    eeKinematicsWBCPtr_ = std::make_shared<PinocchioEndEffectorKinematics>(*pinocchioInterfaceWBCPtr_, pinocchioMapping,
                                                                    modelSettings_.contactNames3DoF);
    eeKinematicsWBCPtr_->setPinocchioInterface(*pinocchioInterfaceWBCPtr_);
  
  }

  void humanoidController::setupMpc()
  {
    std::cout << "use_external_mpc_:" << use_external_mpc_ << std::endl;
    if (use_external_mpc_)
      return;
    // mpc_ = std::make_shared<SqpMpc>(HumanoidInterface_->mpcSettings(), HumanoidInterface_->sqpSettings(),
    //                                 HumanoidInterface_->getOptimalControlProblem(), HumanoidInterface_->getInitializer());
    mpc_ = std::make_shared<GaussNewtonDDP_MPC>(HumanoidInterface_->mpcSettings(), HumanoidInterface_->ddpSettings(), HumanoidInterface_->getRollout(),
                                                HumanoidInterface_->getOptimalControlProblem(), HumanoidInterface_->getInitializer());


    // Gait receiver
    auto gaitReceiverPtr =
        std::make_shared<GaitReceiver>(controllerNh_, HumanoidInterface_->getSwitchedModelReferenceManagerPtr(), robotName_);
    // ROS ReferenceManager
    auto rosReferenceManagerPtr = std::make_shared<RosReferenceManager>(robotName_, HumanoidInterface_->getReferenceManagerPtr());
    rosReferenceManagerPtr->subscribe(controllerNh_);
    mpc_->getSolverPtr()->addSynchronizedModule(gaitReceiverPtr);
    mpc_->getSolverPtr()->setReferenceManager(rosReferenceManagerPtr);
    observationPublisher_ = controllerNh_.advertise<ocs2_msgs::mpc_observation>(robotName_ + "_mpc_observation", 1);
  }

  void humanoidController::setupMrt()
  {
    if (use_external_mpc_)
    {
      mrtRosInterface_ = std::make_shared<MRT_ROS_Interface>(robotName_);
      mrtRosInterface_->launchNodes(controllerNh_);
      return;
    }
    mpcMrtInterface_ = std::make_shared<MPC_MRT_Interface>(*mpc_);
    mpcMrtInterface_->initRollout(&HumanoidInterface_->getRollout());
    mpcTimer_.reset();

    controllerRunning_ = true;
    mpcThread_ = std::thread([&]()
                             {
    while (controllerRunning_) {
      try {
        executeAndSleep(
            [&]() {
              if (mpcRunning_) {
                mpcTimer_.startTimer();
                mpcMrtInterface_->advanceMpc();
                mpcTimer_.endTimer();
              }
            },
            HumanoidInterface_->mpcSettings().mpcDesiredFrequency_);
      } catch (const std::exception& e) {
        controllerRunning_ = false;
        ROS_ERROR_STREAM("[Ocs2 MPC thread] Error : " << e.what());
        //TODO: send the stop command to hardware interface
      }
    } });
    setThreadPriority(HumanoidInterface_->sqpSettings().threadPriority, mpcThread_);
  }

  ocs2_msgs::mpc_flattened_controller humanoidController::createMpcPolicyMsg(const PrimalSolution &primalSolution,
                                                                             const CommandData &commandData,
                                                                             const PerformanceIndex &performanceIndices)
  {
    ocs2_msgs::mpc_flattened_controller mpcPolicyMsg;

    mpcPolicyMsg.initObservation = ros_msg_conversions::createObservationMsg(commandData.mpcInitObservation_);
    mpcPolicyMsg.planTargetTrajectories = ros_msg_conversions::createTargetTrajectoriesMsg(commandData.mpcTargetTrajectories_);
    mpcPolicyMsg.modeSchedule = ros_msg_conversions::createModeScheduleMsg(primalSolution.modeSchedule_);
    mpcPolicyMsg.performanceIndices =
        ros_msg_conversions::createPerformanceIndicesMsg(commandData.mpcInitObservation_.time, performanceIndices);

    switch (primalSolution.controllerPtr_->getType())
    {
    case ControllerType::FEEDFORWARD:
      mpcPolicyMsg.controllerType = ocs2_msgs::mpc_flattened_controller::CONTROLLER_FEEDFORWARD;
      break;
    case ControllerType::LINEAR:
      mpcPolicyMsg.controllerType = ocs2_msgs::mpc_flattened_controller::CONTROLLER_LINEAR;
      break;
    default:
      throw std::runtime_error("MPC_ROS_Interface::createMpcPolicyMsg: Unknown ControllerType");
    }

    // maximum length of the message
    const size_t N = primalSolution.timeTrajectory_.size();

    mpcPolicyMsg.timeTrajectory.clear();
    mpcPolicyMsg.timeTrajectory.reserve(N);
    mpcPolicyMsg.stateTrajectory.clear();
    mpcPolicyMsg.stateTrajectory.reserve(N);
    mpcPolicyMsg.data.clear();
    mpcPolicyMsg.data.reserve(N);
    mpcPolicyMsg.postEventIndices.clear();
    mpcPolicyMsg.postEventIndices.reserve(primalSolution.postEventIndices_.size());

    // time
    for (auto t : primalSolution.timeTrajectory_)
    {
      mpcPolicyMsg.timeTrajectory.emplace_back(t);
    }

    // post-event indices
    for (auto ind : primalSolution.postEventIndices_)
    {
      mpcPolicyMsg.postEventIndices.emplace_back(static_cast<uint16_t>(ind));
    }

    // state
    for (size_t k = 0; k < N; k++)
    {
      ocs2_msgs::mpc_state mpcState;
      mpcState.value.resize(primalSolution.stateTrajectory_[k].rows());
      for (size_t j = 0; j < primalSolution.stateTrajectory_[k].rows(); j++)
      {
        mpcState.value[j] = primalSolution.stateTrajectory_[k](j);
      }
      mpcPolicyMsg.stateTrajectory.emplace_back(mpcState);
    } // end of k loop

    // input
    for (size_t k = 0; k < N; k++)
    {
      ocs2_msgs::mpc_input mpcInput;
      mpcInput.value.resize(primalSolution.inputTrajectory_[k].rows());
      for (size_t j = 0; j < primalSolution.inputTrajectory_[k].rows(); j++)
      {
        mpcInput.value[j] = primalSolution.inputTrajectory_[k](j);
      }
      mpcPolicyMsg.inputTrajectory.emplace_back(mpcInput);
    } // end of k loop

    // controller
    scalar_array_t timeTrajectoryTruncated;
    std::vector<std::vector<float> *> policyMsgDataPointers;
    policyMsgDataPointers.reserve(N);
    for (auto t : primalSolution.timeTrajectory_)
    {
      mpcPolicyMsg.data.emplace_back(ocs2_msgs::controller_data());

      policyMsgDataPointers.push_back(&mpcPolicyMsg.data.back().data);
      timeTrajectoryTruncated.push_back(t);
    } // end of k loop

    // serialize controller into data buffer
    primalSolution.controllerPtr_->flatten(timeTrajectoryTruncated, policyMsgDataPointers);

    return mpcPolicyMsg;
  }

  void humanoidController::setupStateEstimate(const std::string &taskFile, bool verbose)
  {
    // 这部分只有下肢，可能需要修改。
    stateEstimate_ = std::make_shared<KalmanFilterEstimate>(HumanoidInterface_->getPinocchioInterface(),
                                                            HumanoidInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_);
    dynamic_cast<KalmanFilterEstimate &>(*stateEstimate_).loadSettings(taskFile, verbose);
    currentObservation_.time = 0;
  }

  void humanoidCheaterController::setupStateEstimate(const std::string & /*taskFile*/, bool /*verbose*/)
  {
    stateEstimate_ = std::make_shared<FromTopicStateEstimate>(HumanoidInterface_->getPinocchioInterface(),
                                                              HumanoidInterface_->getCentroidalModelInfo(), *eeKinematicsPtr_, controllerNh_);
  }

  void humanoidKuavoController::setupStateEstimate(const std::string &taskFile, bool verbose)
  {
#ifdef KUAVO_CONTROL_LIB_FOUND
    // auto [plant, context] = drake_interface_->getPlantAndContext();
    stateEstimate_ = std::make_shared<InEkfBaseFilter>(HumanoidInterface_->getPinocchioInterface(),
                                                       HumanoidInterface_->getCentroidalModelInfo(),
                                                       *eeKinematicsPtr_,
                                                       drake_interface_,
                                                       dt_,
                                                       ros_logger_);
    std::cout << "InEkfBaseFilter stateEstimate_ initialized" << std::endl;
#endif
  }

} // namespace humanoid_controller
// PLUGINLIB_EXPORT_CLASS(humanoid_controller::humanoidController)
// PLUGINLIB_EXPORT_CLASS(humanoid_controller::humanoidCheaterController)
