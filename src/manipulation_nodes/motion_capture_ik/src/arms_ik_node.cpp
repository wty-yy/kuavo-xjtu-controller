#include <iostream>
#include <Eigen/Core>
#include <chrono>
#include <unistd.h>
#include <iomanip> 
#include <algorithm>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <ros/package.h>

#include "plantIK.h"
#include "motion_capture_ik/package_path.h"

#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/lcm/drake_lcm.h"
#include <drake/systems/lcm/lcm_interface_system.h>
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/common/eigen_types.h"

// msgs
#include "motion_capture_ik/twoArmHandPoseCmd.h"
#include "motion_capture_ik/twoArmHandPose.h"
#include "motion_capture_ik/headBodyPose.h"
// srv
#include "motion_capture_ik/twoArmHandPoseCmdSrv.h"
#include "motion_capture_ik/fkSrv.h"


namespace
{
    const double TO_RADIAN = M_PI / 180.0;
    const double TO_DEGREE = 180.0 / M_PI;
};

namespace HighlyDynamic{
struct IkCmd
{
    Eigen::Vector3d pos_xyz; // hand pos
    Eigen::Quaterniond quat; // hand quaternion
    Eigen::Vector3d elbow_pos_xyz; // elbow pos, only for motion capture
    Eigen::VectorXd joint_angles; // joint angles, could as initial guess
    // IKParams ik_params; // solver params

    bool is_elbow_pos_valid() const
    {
        bool invalid = true;
        for(int i=0; i<3; ++i)// if all three values are close to zero, it's invalid
            invalid &= (elbow_pos_xyz[i] >= -1e3 && elbow_pos_xyz[i] <= 1e3);
        return !invalid;
    }
};

class ArmsIKNode
{
    public:
        ArmsIKNode(ros::NodeHandle& nh, const std::string& model_path
            , std::vector<std::string> end_frames_name, Eigen::Vector3d custom_eef_frame_pos=Eigen::Vector3d::Zero()
            , HighlyDynamic::HandSide hand_side=HighlyDynamic::HandSide::LEFT
            , int single_arm_num=7
        )
        : nh_(nh)
        , single_arm_num_(single_arm_num)
        , hand_side_(hand_side)
        {
            const double dt = 0.001;
            const std::vector<std::string> custom_eef_frame_names{"eef_left", "eef_right"};
            
            drake::systems::DiagramBuilder<double> builder;
            plant_ptr_ = builder.AddSystem<drake::multibody::MultibodyPlant>(dt);
            drake::multibody::Parser(plant_ptr_).AddModelFromFile(model_path);
            plant_ptr_->WeldFrames(plant_ptr_->world_frame(), plant_ptr_->GetFrameByName(end_frames_name[0]));
            // custom eef frame
            plant_ptr_->AddFrame(std::make_unique<drake::multibody::FixedOffsetFrame<double>>(
                custom_eef_frame_names[0], plant_ptr_->GetFrameByName(end_frames_name[1]),
                drake::math::RigidTransform<double>(drake::math::RotationMatrix<double>::Identity(), custom_eef_frame_pos)));
            plant_ptr_->AddFrame(std::make_unique<drake::multibody::FixedOffsetFrame<double>>(
                custom_eef_frame_names[1], plant_ptr_->GetFrameByName(end_frames_name[2]),
                drake::math::RigidTransform<double>(drake::math::RotationMatrix<double>::Identity(), custom_eef_frame_pos)));
            // 替换原始eef name
            end_frames_name[1] = custom_eef_frame_names[0];
            end_frames_name[2] = custom_eef_frame_names[1];
            plant_ptr_->Finalize();
            std::cout << "plant nq: " << plant_ptr_->num_positions() << ", nv: " << plant_ptr_->num_velocities() << std::endl;

            diagram_ptr_ = builder.Build();
            diagram_context_ptr_ = diagram_ptr_->CreateDefaultContext();
            plant_context_ptr_ = &diagram_ptr_->GetMutableSubsystemContext(*plant_ptr_, diagram_context_ptr_.get());

            q0_ = plant_ptr_->GetPositions(*plant_context_ptr_);

            ik_ = HighlyDynamic::CoMIK(plant_ptr_, end_frames_name);

            // ros
            sub_ik_cmd_ = nh_.subscribe<motion_capture_ik::twoArmHandPoseCmd>("/ik/two_arm_hand_pose_cmd", 10, &ArmsIKNode::ik_cmd_callback, this);

            joint_pub_ = nh_.advertise<sensor_msgs::JointState>("/kuavo_arm_traj", 10);
            time_cost_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("/ik/debug/time_cost", 10);
            ik_result_pub_ = nh_.advertise<motion_capture_ik::twoArmHandPose>("/ik/result", 10);
            head_body_pose_pub_ = nh_.advertise<motion_capture_ik::headBodyPose>("/kuavo_head_body_orientation", 10);
            // srv
            // 初始化服务服务器
            ik_server_ = nh_.advertiseService("/ik/two_arm_hand_pose_cmd_srv", &ArmsIKNode::handleServiceRequest, this);
            fk_server_ = nh_.advertiseService("/ik/fk_srv", &ArmsIKNode::handleFKServiceRequest, this);
            // solver params
            ik_solve_params_.major_optimality_tol = 9e-3;
            ik_solve_params_.major_feasibility_tol = 9e-3;
            ik_solve_params_.minor_feasibility_tol = 9e-3;

            ik_solve_params_.major_iterations_limit = 50;
            ik_solve_params_.oritation_constraint_tol = 19e-3;
            ik_solve_params_.pos_constraint_tol = 9e-3;
            ik_solve_params_.pos_cost_weight = 10;
            // q0 for ik
            ik_cmd_left_.joint_angles = Eigen::VectorXd::Zero(single_arm_num_);
            ik_cmd_right_.joint_angles = Eigen::VectorXd::Zero(single_arm_num_);
        }

        void run()
        {
            ros::Rate rate(1000);
            while(ros::ok()) 
            {
                ros::spinOnce();
                if(!recived_cmd_)
                {
                    rate.sleep();
                    // std::cout << "Waiting for command..." << std::endl;
                    continue;
                }
                auto loop_start = std::chrono::high_resolution_clock::now();
                std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>> pose_vec{
                    {Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(0, 0, 0)},
                    {ik_cmd_left_.quat, ik_cmd_left_.pos_xyz},
                    {ik_cmd_right_.quat, ik_cmd_right_.pos_xyz},
                    {Eigen::Quaterniond(1, 0, 0, 0), ik_cmd_left_.elbow_pos_xyz},
                    {Eigen::Quaterniond(1, 0, 0, 0), ik_cmd_right_.elbow_pos_xyz}
                    };

                Eigen::VectorXd q;
                // std::cout << std::fixed << std::setprecision(5) << "q0: " << q0_.head(single_arm_num_).transpose() << std::endl;
                if(use_ik_cmd_q0_)
                {
                    q0_ << ik_cmd_left_.joint_angles, ik_cmd_right_.joint_angles;
                    std::cout << std::fixed << std::setprecision(3) << "Left: " << q0_.head(single_arm_num_).transpose()
                                            << ", Left: " << q0_.tail(single_arm_num_).transpose() << std::endl;
                }
                auto start = std::chrono::high_resolution_clock::now();
                bool result = ik_.solve(pose_vec, q0_, q, ik_solve_params_);
                std::chrono::duration<double, std::milli> elapsed = std::chrono::high_resolution_clock::now() - start;
                // std::cout << "Time elapsed: " << elapsed.count() << " ms" << std::endl;

                if(result && recived__new_cmd_)//接收到新的命令时才发布手臂控制话题
                {
                    ++ik_success_count_;
                    // std::cout << "IK success" << std::endl;
                    // std::cout << std::fixed << std::setprecision(5) << "q:  " << q.head(single_arm_num_).transpose() << std::endl;
                    q0_ = Eigen::VectorXd(q);
                    // 发布JointState消息
                    sensor_msgs::JointState joint_state;
                    joint_state.header.stamp = ros::Time::now();
                    const int start_idx = q.size() - 2*single_arm_num_;
                    for (int j = 0; j < 14; ++j)
                    {
                        joint_state.name.push_back("joint_" + std::to_string(j+1));
                        joint_state.position.push_back(TO_DEGREE*q[start_idx+j]);
                    }
                    // 如果只控制单手，则将其他手臂的关节位置设置为0
                    if (hand_side_ == HighlyDynamic::HandSide::LEFT)
                        std::fill(joint_state.position.begin() + 7, joint_state.position.begin() + 14, 0);
                    else if (hand_side_ == HighlyDynamic::HandSide::RIGHT)
                        std::fill(joint_state.position.begin() + 0, joint_state.position.begin() + 7, 0);
                    joint_pub_.publish(joint_state);
                    if(start_idx > 0)//包含躯干
                    {
                        motion_capture_ik::headBodyPose head_body_pose_msg;
                        head_body_pose_msg.body_height = q[0];
                        head_body_pose_msg.body_yaw = q[1];
                        head_body_pose_msg.body_pitch = q[2];
                        head_body_pose_msg.body_roll = q[3];
                        // TODO: body height
                        head_body_pose_pub_.publish(head_body_pose_msg);
                    }
                    publish_ik_result_info(q);
                }
                // else
                //     std::cout << "IK failed" << std::endl;
                rate.sleep();

                std::chrono::duration<double, std::milli> loop_elapsed = std::chrono::high_resolution_clock::now() - loop_start;
                // time cost
                std_msgs::Float32MultiArray time_cost_msg;
                time_cost_msg.data.push_back(loop_elapsed.count());
                time_cost_msg.data.push_back(elapsed.count());
                time_cost_pub_.publish(time_cost_msg);
                if(recived__new_cmd_)
                {
                    ++ik_count_;
                    // print status
                    std::cout << "\rLoop count: " << ik_count_ 
                            << ", sucess count: " << ik_success_count_
                            << ", IK success rate: " << std::fixed << std::setprecision(1) << 100 * ik_success_count_ / static_cast<double>(ik_count_)
                            << "%, time-cost: " << std::fixed << std::setprecision(1) << (elapsed.count()) << " ms.";
                    std::cout.flush();
                    if(ik_count_ >= std::numeric_limits<long long int>::max())
                    {
                        ik_count_ = 0;
                        ik_success_count_ = 0;
                    }
                }
                recived__new_cmd_ = false;
            }
        }

    private:
        void ik_cmd_callback(const motion_capture_ik::twoArmHandPoseCmd::ConstPtr& msg)
        {
            auto &hand_poses = msg->hand_poses;
            ik_cmd_left_.pos_xyz << hand_poses.left_pose.pos_xyz[0], hand_poses.left_pose.pos_xyz[1], hand_poses.left_pose.pos_xyz[2];
            ik_cmd_left_.quat = Eigen::Quaterniond(hand_poses.left_pose.quat_xyzw[3],
                                    hand_poses.left_pose.quat_xyzw[0], hand_poses.left_pose.quat_xyzw[1], hand_poses.left_pose.quat_xyzw[2]);
            ik_cmd_left_.elbow_pos_xyz << hand_poses.left_pose.elbow_pos_xyz[0], hand_poses.left_pose.elbow_pos_xyz[1], hand_poses.left_pose.elbow_pos_xyz[2];
            // right
            ik_cmd_right_.pos_xyz << hand_poses.right_pose.pos_xyz[0], hand_poses.right_pose.pos_xyz[1], hand_poses.right_pose.pos_xyz[2];
            ik_cmd_right_.quat = Eigen::Quaterniond(hand_poses.right_pose.quat_xyzw[3],
                                    hand_poses.right_pose.quat_xyzw[0], hand_poses.right_pose.quat_xyzw[1], hand_poses.right_pose.quat_xyzw[2]);
            ik_cmd_right_.elbow_pos_xyz << hand_poses.right_pose.elbow_pos_xyz[0], hand_poses.right_pose.elbow_pos_xyz[1], hand_poses.right_pose.elbow_pos_xyz[2];
            // ik_cmd_left_.ik_params; // TO-DO: set solver params
            use_ik_cmd_q0_ = msg->joint_angles_as_q0;
            if(use_ik_cmd_q0_)
                for(int i=0; i<single_arm_num_; ++i)
                {
                    ik_cmd_left_.joint_angles[i] = hand_poses.left_pose.joint_angles[i];
                    ik_cmd_right_.joint_angles[i] = hand_poses.right_pose.joint_angles[i];
                }
            if (msg->use_custom_ik_param)
            {
                ik_solve_params_.major_optimality_tol = msg->ik_param.major_optimality_tol;
                ik_solve_params_.major_feasibility_tol = msg->ik_param.major_feasibility_tol;
                ik_solve_params_.minor_feasibility_tol = msg->ik_param.minor_feasibility_tol;
                ik_solve_params_.major_iterations_limit = msg->ik_param.major_iterations_limit;

                ik_solve_params_.oritation_constraint_tol = msg->ik_param.oritation_constraint_tol;
                ik_solve_params_.pos_constraint_tol = msg->ik_param.pos_constraint_tol;
                ik_solve_params_.pos_cost_weight = msg->ik_param.pos_cost_weight;
            }
            if (!recived_cmd_)
                recived_cmd_ = true;
            recived__new_cmd_ = true;
        }

        motion_capture_ik::twoArmHandPose publish_ik_result_info(const Eigen::VectorXd& q)
        {
            motion_capture_ik::twoArmHandPose msg;
            const int start_idx = q.size() - 2*single_arm_num_;
            for (size_t i = 0; i < single_arm_num_; i++)
            {
                msg.left_pose.joint_angles[i] = q[i + start_idx];
                msg.right_pose.joint_angles[i] = q[i + single_arm_num_ + start_idx];
            }
            auto [left_pos, left_quat] = ik_.FK(q, HighlyDynamic::HandSide::LEFT);
            auto [right_pos, right_quat] = ik_.FK(q, HighlyDynamic::HandSide::RIGHT);

            for(int i = 0; i < 3; i++)
            {
                msg.left_pose.pos_xyz[i] = left_pos[i];
                msg.right_pose.pos_xyz[i] = right_pos[i];
            }
            for(int i = 0; i < 4; i++)
            {
                msg.left_pose.quat_xyzw[i] = left_quat.coeffs()[i]; //coeffs() return order: xyzw
                msg.right_pose.quat_xyzw[i] = right_quat.coeffs()[i];
            }
            ik_result_pub_.publish(msg);
            return msg;
        }

        // 处理服务请求的回调函数
        bool handleServiceRequest(motion_capture_ik::twoArmHandPoseCmdSrv::Request &req,
                                motion_capture_ik::twoArmHandPoseCmdSrv::Response &res) 
        {
            const auto &cmd = req.twoArmHandPoseCmdRequest;
            auto &hand_poses = cmd.hand_poses;
            ik_cmd_left_.pos_xyz << hand_poses.left_pose.pos_xyz[0], hand_poses.left_pose.pos_xyz[1], hand_poses.left_pose.pos_xyz[2];
            ik_cmd_left_.quat = Eigen::Quaterniond(hand_poses.left_pose.quat_xyzw[3],
                                    hand_poses.left_pose.quat_xyzw[0], hand_poses.left_pose.quat_xyzw[1], hand_poses.left_pose.quat_xyzw[2]);
            ik_cmd_left_.elbow_pos_xyz << hand_poses.left_pose.elbow_pos_xyz[0], hand_poses.left_pose.elbow_pos_xyz[1], hand_poses.left_pose.elbow_pos_xyz[2];
            // right
            ik_cmd_right_.pos_xyz << hand_poses.right_pose.pos_xyz[0], hand_poses.right_pose.pos_xyz[1], hand_poses.right_pose.pos_xyz[2];
            ik_cmd_right_.quat = Eigen::Quaterniond(hand_poses.right_pose.quat_xyzw[3],
                                    hand_poses.right_pose.quat_xyzw[0], hand_poses.right_pose.quat_xyzw[1], hand_poses.right_pose.quat_xyzw[2]);
            ik_cmd_right_.elbow_pos_xyz << hand_poses.right_pose.elbow_pos_xyz[0], hand_poses.right_pose.elbow_pos_xyz[1], hand_poses.right_pose.elbow_pos_xyz[2];
            // ik_cmd_left_.ik_params; // TO-DO: set solver params
            use_ik_cmd_q0_ = cmd.joint_angles_as_q0;
            if(use_ik_cmd_q0_)
                for(int i=0; i<single_arm_num_; ++i)
                {
                    ik_cmd_left_.joint_angles[i] = hand_poses.left_pose.joint_angles[i];
                    ik_cmd_right_.joint_angles[i] = hand_poses.right_pose.joint_angles[i];
                }
            if (cmd.use_custom_ik_param)
            {
                ik_solve_params_.major_optimality_tol = cmd.ik_param.major_optimality_tol;
                ik_solve_params_.major_feasibility_tol = cmd.ik_param.major_feasibility_tol;
                ik_solve_params_.minor_feasibility_tol = cmd.ik_param.minor_feasibility_tol;
                ik_solve_params_.major_iterations_limit = cmd.ik_param.major_iterations_limit;

                ik_solve_params_.oritation_constraint_tol = cmd.ik_param.oritation_constraint_tol;
                ik_solve_params_.pos_constraint_tol = cmd.ik_param.pos_constraint_tol;
                ik_solve_params_.pos_cost_weight = cmd.ik_param.pos_cost_weight;
            }
            //
            if(use_ik_cmd_q0_)
            {
                q0_ << ik_cmd_left_.joint_angles, ik_cmd_right_.joint_angles;
                std::cout << std::fixed << std::setprecision(3) << "Left: " << q0_.head(single_arm_num_).transpose()
                                        << ", Left: " << q0_.tail(single_arm_num_).transpose() << std::endl;
            }
            auto start = std::chrono::high_resolution_clock::now();
            std::vector<std::pair<Eigen::Quaterniond, Eigen::Vector3d>> pose_vec{
                    {Eigen::Quaterniond(1, 0, 0, 0), Eigen::Vector3d(0, 0, 0)},
                    {ik_cmd_left_.quat, ik_cmd_left_.pos_xyz},
                    {ik_cmd_right_.quat, ik_cmd_right_.pos_xyz},
                    {Eigen::Quaterniond(1, 0, 0, 0), ik_cmd_left_.elbow_pos_xyz},
                    {Eigen::Quaterniond(1, 0, 0, 0), ik_cmd_right_.elbow_pos_xyz}
                    };
            Eigen::VectorXd q;
            bool result = ik_.solve(pose_vec, q0_, q, ik_solve_params_);
            std::chrono::duration<double, std::milli> elapsed = std::chrono::high_resolution_clock::now() - start;
            // std::cout << "Time elapsed: " << elapsed.count() << " ms" << std::endl;
            res.success = false;
            res.with_torso = false;
            res.time_cost = elapsed.count();
            if(result)
            {
                res.success = true;
                q0_ = Eigen::VectorXd(q);
                //
                const int start_idx = q.size() - 2*single_arm_num_;
                for (int j = 0; j < 2*single_arm_num_; ++j)
                {
                    res.q_arm.push_back(q[start_idx+j]);
                }
                // 如果只控制单手，则将其他手臂的关节位置设置为0
                if (hand_side_ == HighlyDynamic::HandSide::LEFT)
                    std::fill(res.q_arm.begin() + 7, res.q_arm.begin() + 14, 0);
                else if (hand_side_ == HighlyDynamic::HandSide::RIGHT)
                    std::fill(res.q_arm.begin() + 0, res.q_arm.begin() + 7, 0);
                if(start_idx > 0)//包含躯干
                {
                    res.with_torso = true;
                    res.q_torso = std::vector<double>(q.data(), q.data() + start_idx);
                }
                motion_capture_ik::twoArmHandPose msg = publish_ik_result_info(q);
                res.hand_poses = msg;
            }
            // 返回响应
            return true;
        }

        bool handleFKServiceRequest(motion_capture_ik::fkSrv::Request &req, motion_capture_ik::fkSrv::Response &res) 
        {
            const int num_dof = q0_.size(); 
            if(req.q.size() != num_dof)
            {
                ROS_ERROR_STREAM("The size of the request q ("<< req.q.size() << ") is not equal to the number of dof in ik_node(" << num_dof << ")");
                res.success = false;
                return false;
            }
            Eigen::VectorXd q = Eigen::VectorXd::Zero(num_dof);
            for(int i = 0; i < num_dof; i++)
                q(i) = req.q[i];
            auto msg = publish_ik_result_info(q);
            res.hand_poses = msg;
            res.success = true;
            return true;
        }

    private:
        HighlyDynamic::CoMIK ik_;
        drake::multibody::MultibodyPlant<double>* plant_ptr_;
        std::unique_ptr<drake::systems::Diagram<double>> diagram_ptr_;
        std::unique_ptr<drake::systems::Context<double>> diagram_context_ptr_;
        drake::systems::Context<double> *plant_context_ptr_;
        Eigen::VectorXd q0_;
        IkCmd ik_cmd_left_;
        IkCmd ik_cmd_right_;
        const int single_arm_num_{7}; 
        HighlyDynamic::HandSide hand_side_;
        // ros
        ros::NodeHandle nh_;
        ros::Subscriber sub_ik_cmd_;
        ros::Publisher joint_pub_;
        ros::Publisher time_cost_pub_;
        ros::Publisher ik_result_pub_;
        ros::Publisher head_body_pose_pub_;
        ros::ServiceServer ik_server_;
        ros::ServiceServer fk_server_;
        bool recived_cmd_ = false;
        bool recived__new_cmd_ = false;
        HighlyDynamic::IKParams ik_solve_params_;
        bool use_ik_cmd_q0_{false};
        long long int ik_count_{0};
        long long int ik_success_count_{0};
};
}
int main(int argc, char* argv[])
{
    // 初始化ROS节点
    ros::init(argc, argv, "ik_publisher");
    ros::NodeHandle nh;
    double eef_z_bias = 0.0;
    std::string model_path = "models/biped_gen4.0/urdf/biped_v3_arm.urdf";
    int control_hand_side = 2; // 0: left, 1: right, 2: both
    if(ros::param::has("model_path"))
    {
        ros::param::get("model_path", model_path);
        
        std::cout << "model_path: " << model_path << std::endl;
    }else
    {
        std::string package_path = ros::package::getPath("kuavo_assets");
        model_path = package_path + "/models/biped_s40/urdf/drake/biped_v3_arm.urdf";
    }
    if(ros::param::has("eef_z_bias"))
    {
        ros::param::get("eef_z_bias", eef_z_bias);
        std::cout << "eef_z_bias: " << eef_z_bias << std::endl;
    }
    if(ros::param::has("control_hand_side"))
    {
        ros::param::get("control_hand_side", control_hand_side);
        std::cout << "control_hand_side: " << control_hand_side << std::endl;
    }
    
    std::cout << "model_path: " << model_path << std::endl;
    std::vector<std::string> end_frames_name = {"torso", "l_hand_roll", "r_hand_roll", "l_forearm_pitch", "r_forearm_pitch"};
    Eigen::Vector3d custom_eef_frame_pos = Eigen::Vector3d(0, 0, eef_z_bias);
    HighlyDynamic::ArmsIKNode arm_ik_node(nh, model_path, end_frames_name, custom_eef_frame_pos, HighlyDynamic::intToHandSide(control_hand_side));
    arm_ik_node.run();

    return 0;
}
