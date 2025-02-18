import subprocess
import rospy
import os
from rich import console
from humanoid_plan_arm_trajectory.srv import planArmTrajectoryBezierCurve, planArmTrajectoryBezierCurveRequest
from humanoid_plan_arm_trajectory.msg import jointBezierTrajectory, bezierCurveCubicPoint
from kuavo_msgs.srv import changeArmCtrlMode
from utils.utils import get_start_end_frame_time, frames_to_custom_action_data_ocs2
import time
import signal
import datetime
import json
import threading
from h12pro_controller_node.srv import playmusic, playmusicRequest, playmusicResponse
from h12pro_controller_node.srv import ExecuteArmAction, ExecuteArmActionRequest, ExecuteArmActionResponse
from h12pro_controller_node.msg import RobotActionState

console = console.Console()
current_dir = os.path.dirname(os.path.abspath(__file__))
config_dir = os.path.join(os.path.dirname(current_dir), "config")
ACTION_FILE_FOLDER = "~/.config/lejuconfig/action_files"
ROS_BAG_LOG_SAVE_PATH = "~/.log/vr_remote_control/rosbag"
HUMANOID_ROBOT_SESSION_NAME = "humanoid_robot"
VR_REMOTE_CONTROL_SESSION_NAME = "vr_remote_control"
LAUNCH_HUMANOID_ROBOT_SIM_CMD = "roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch joystick_type:=h12 start_way:=auto"
# LAUNCH_HUMANOID_ROBOT_SIM_CMD = "roslaunch humanoid_controllers load_kuavo_mujoco_sim.launch joystick_type:=h12"
LAUNCH_HUMANOID_ROBOT_REAL_CMD = "roslaunch humanoid_controllers load_kuavo_real.launch joystick_type:=h12 start_way:=auto"
LAUNCH_VR_REMOTE_CONTROL_CMD = "roslaunch noitom_hi5_hand_udp_python launch_quest3_ik.launch"
kuavo_ros_control_ws_path = os.getenv("KUAVO_ROS_CONTROL_WS_PATH")
# 录制话题的格式
record_topics_path = os.path.join(config_dir, "record_topics.json")
with open(record_topics_path, "r") as f:
    record_topics = json.load(f)["record_topics"]
record_vr_rosbag_pid = None
# 自定义动作json文件
customize_config_path = os.path.join(config_dir, "customize_config.json")
with open(customize_config_path, "r") as f:
    customize_config_data = json.load(f)

ROBOT_ACTION_STATUS = 0 # 手臂完成状态 | 0 没开始 | 1 执行中 |  2 完成
def robot_action_state_callback(msg):
    global ROBOT_ACTION_STATUS
    ROBOT_ACTION_STATUS = msg.state
    # rospy.loginfo(f" ---------- ROBOT_ACTION_STATUS ---------- : {ROBOT_ACTION_STATUS}")
rospy.Subscriber('/robot_action_state', RobotActionState, robot_action_state_callback)

import os
import netifaces
import json
import hashlib
import math
import numpy as np
def get_wifi_ip():
    try:
        # Get all network interfaces
        interfaces = netifaces.interfaces()

        # Find the WiFi interface (usually starts with 'wl')
        wifi_interface = next(
            (iface for iface in interfaces if iface.startswith("enx00e04c686129")), None
        )

        if wifi_interface:
            # Get the IPv4 address of the WiFi interface
            addresses = netifaces.ifaddresses(wifi_interface)
            if netifaces.AF_INET in addresses:
                return addresses[netifaces.AF_INET][0]["addr"]

        return "WiFi not connected"
    except Exception as e:
        return f"Error: {e}"

def call_execute_arm_action(action_name):
    """Call the /execute_arm_action service
    :param action_name 动作名字
    :return: bool， 服务调用结果
    """
    try:
        _execute_arm_action_client = rospy.ServiceProxy('/execute_arm_action', ExecuteArmAction)
        request = ExecuteArmActionRequest()
        request.action_name = action_name

        response = _execute_arm_action_client(request)
        rospy.loginfo(f"ExecuteArmAction service response:\nsuccess: {response.success}\nmessage: {response.message}")
        return response.success, response.message
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call to '/execute_arm_action' failed: {e}")
        return False, f"Service exception: {e}"

def set_robot_play_music(music_file_name:str, music_volume:int)->bool:
    """机器人播放指定文件的音乐
    :param music_file_name, 音乐文件名字
    :param music_volume, 音乐音量
    :return: bool, 服务调用结果 
    """
    try:
        _robot_music_play_client = rospy.ServiceProxy("/play_music", playmusic)
        request = playmusicRequest()
        request.music_number = music_file_name
        request.volume = music_volume
        # 客户端接收
        response = _robot_music_play_client(request)
        rospy.loginfo(f"Service call /play_music call: {response.success_flag}")
        return response.success_flag
    except Exception as e:
        print(f"An error occurred: {e}")
        rospy.loginfo("Service /play_music call: fail!...please check again!")
        return False

def call_change_arm_ctrl_mode_service(arm_ctrl_mode):
    result = True
    service_name = "humanoid_change_arm_ctrl_mode"
    try:
        rospy.wait_for_service(service_name, timeout=0.5)
        change_arm_ctrl_mode = rospy.ServiceProxy(
            "humanoid_change_arm_ctrl_mode", changeArmCtrlMode
        )
        change_arm_ctrl_mode(control_mode=arm_ctrl_mode)
        rospy.loginfo("Change arm ctrl mode Service call successful")
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed: %s", e)
        result = False
    except rospy.ROSException:
        rospy.logerr(f"Service {service_name} not available")
        result = False
    finally:
        return result

def create_bezier_request(action_data, start_frame_time, end_frame_time):
    req = planArmTrajectoryBezierCurveRequest()
    for key, value in action_data.items():
        msg = jointBezierTrajectory()
        for frame in value:
            point = bezierCurveCubicPoint()
            point.end_point, point.left_control_point, point.right_control_point = frame
            msg.bezier_curve_points.append(point)
        req.multi_joint_bezier_trajectory.append(msg)
    req.start_frame_time = start_frame_time
    req.end_frame_time = end_frame_time
    req.joint_names = [
        "l_arm_pitch", 
        "l_arm_roll", 
        "l_arm_yaw", 
        "l_forearm_pitch", 
        "l_hand_yaw", 
        "l_hand_pitch", 
        "l_hand_roll", 
        "r_arm_pitch", 
        "r_arm_roll", 
        "r_arm_yaw", 
        "r_forearm_pitch", 
        "r_hand_yaw", 
        "r_hand_pitch", 
        "r_hand_roll",
        "thumb1",
        "thumb2",
        "index1",
        "middle1",
        "ring1",
        "pinky1",
        "head_yaw",
        "head_pitch",
    ]
    return req

def plan_arm_trajectory_bezier_curve_client(req):
    service_name = '/bezier/plan_arm_trajectory'
    rospy.wait_for_service(service_name)
    try:
        plan_service = rospy.ServiceProxy(service_name, planArmTrajectoryBezierCurve)
        res = plan_service(req)
        return res.success
    except rospy.ServiceException as e:
        rospy.logerr(f"PlService call failed: {e}")
        return False

def print_state_transition(trigger, source, target) -> None:
    console.print(
        f"Trigger: [bold blue]{trigger}[/bold blue] From [bold green]{source}[/bold green] to [bold green]{target}[/bold green]"
    )


def launch_humanoid_robot(real_robot=True,calibrate=False):
    
    robot_version = os.getenv('ROBOT_VERSION')
    print(f"current robot version: {robot_version}")
    subprocess.run(["tmux", "kill-session", "-t", HUMANOID_ROBOT_SESSION_NAME], 
                    stderr=subprocess.DEVNULL) 
    
    if real_robot:
        launch_cmd = LAUNCH_HUMANOID_ROBOT_REAL_CMD
    else:
        launch_cmd = LAUNCH_HUMANOID_ROBOT_SIM_CMD
    
    if calibrate:
        launch_cmd += " cali:=true cali_arm:=true"
        
    print(f"launch_cmd: {launch_cmd}")
    print("If you want to check the session, please run 'tmux attach -t humanoid_robot'")
    ip = get_wifi_ip()
    tmux_cmd = [
        "sudo", "tmux", "new-session",
        "-s", HUMANOID_ROBOT_SESSION_NAME, 
        "-d",  
        f"source ~/.bashrc && \
            source {kuavo_ros_control_ws_path}/devel/setup.bash && \
            export ROBOT_VERSION={robot_version} && \
            export ROS_MASTER_URI=http://{ip}:11311 && \
            export ROS_IP={ip} && \
            {launch_cmd}; exec bash"
    ]
    
    process = subprocess.Popen(
        tmux_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )
    
    rospy.sleep(5.0)
    
    result = subprocess.run(["tmux", "has-session", "-t", HUMANOID_ROBOT_SESSION_NAME], 
                            capture_output=True)
    if result.returncode == 0:
        print(f"Started humanoid_robot in tmux session: {HUMANOID_ROBOT_SESSION_NAME}")
    else:
        print("Failed to start humanoid_robot")
        raise Exception("Failed to start humanoid_robot")
        


def start_vr_remote_control_callback(event):
    source = event.kwargs.get("source")
    trigger = event.kwargs.get("trigger")
    print(f"launch_cmd: {LAUNCH_VR_REMOTE_CONTROL_CMD}")
    tmux_cmd = [
        "tmux", "new-session",
        "-s", VR_REMOTE_CONTROL_SESSION_NAME, 
        "-d",  
        f"bash -c -i 'source ~/.bashrc && \
          source {kuavo_ros_control_ws_path}/devel/setup.bash && \
          {LAUNCH_VR_REMOTE_CONTROL_CMD}; exec bash'"
    ]
    subprocess.run(["tmux", "kill-session", "-t", VR_REMOTE_CONTROL_SESSION_NAME], 
                  stderr=subprocess.DEVNULL) 
    process = subprocess.Popen(
        tmux_cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

    time.sleep(3)
    result = subprocess.run(["tmux", "has-session", "-t", VR_REMOTE_CONTROL_SESSION_NAME], 
                              capture_output=True)
    if result.returncode == 0:
        print(f"Started vr_remote_control in tmux session: {VR_REMOTE_CONTROL_SESSION_NAME}")
        print_state_transition(trigger, source, "vr_remote_control")
    else:
        print("Failed to start vr_remote_control")
        raise Exception("Failed to create tmux session")
    

def stop_vr_remote_control_callback(event):
    source = event.kwargs.get("source")
    trigger = event.kwargs.get("trigger")
    subprocess.run(["tmux", "kill-session", "-t", VR_REMOTE_CONTROL_SESSION_NAME], 
                  stderr=subprocess.DEVNULL) 
    print(f"Stopped {VR_REMOTE_CONTROL_SESSION_NAME} in tmux session")
    kill_record_vr_rosbag()
    time.sleep(3)
    print_state_transition(trigger, source, "stance")

def initial_pre_callback(event):
    source = event.kwargs.get("source")
    print_state_transition("initial_pre", source, "ready_stance")
    launch_humanoid_robot(event.kwargs.get("real_robot"))
    
def calibrate_callback(event):
    source = event.kwargs.get("source")
    print_state_transition("calibrate", source, "calibrate")
    launch_humanoid_robot(event.kwargs.get("real_robot"), calibrate=True)
    
def ready_stance_callback(event):
    source = event.kwargs.get("source")
    print_state_transition("ready_stance", source, "stance")

def stance_callback(event):
    source = event.kwargs.get("source")
    call_change_arm_ctrl_mode_service(1)
    print_state_transition("stance", source, "stance")

def walk_callback(event):
    source = event.kwargs.get("source")
    call_change_arm_ctrl_mode_service(1)
    print_state_transition("walk", source, "walk")

def trot_callback(event):
    source = event.kwargs.get("source")
    call_change_arm_ctrl_mode_service(1)
    print_state_transition("trot", source, "trot")

def stop_callback(event):
    source = event.kwargs.get("source")
    print_state_transition("stop", source, "initial")
    # kill humanoid_robot and vr_remote_control
    subprocess.run(["tmux", "kill-session", "-t", HUMANOID_ROBOT_SESSION_NAME], 
                  stderr=subprocess.DEVNULL) 
    subprocess.run(["tmux", "kill-session", "-t", VR_REMOTE_CONTROL_SESSION_NAME], 
                  stderr=subprocess.DEVNULL) 
    kill_record_vr_rosbag()

def arm_pose_callback(event):
    source = event.kwargs.get("source")
    trigger = event.kwargs.get("trigger")
    current_arm_joint_state = event.kwargs.get("current_arm_joint_state")
    print_state_transition(trigger, source, "stance")
    try:
        call_change_arm_ctrl_mode_service(2)
        action_file_path = os.path.expanduser(f"{ACTION_FILE_FOLDER}/{trigger}.tact")
        start_frame_time, end_frame_time = get_start_end_frame_time(action_file_path)
        action_frames = frames_to_custom_action_data_ocs2(action_file_path, start_frame_time, current_arm_joint_state)
        req = create_bezier_request(action_frames, start_frame_time, end_frame_time+1)
        if plan_arm_trajectory_bezier_curve_client(req):
            rospy.loginfo("Plan arm trajectory bezier curve client call successful")
    except Exception as e:
        rospy.logerr(f"Error in arm_pose_callback: {e}")
    pass

# 等待动作完成的函数，增加超时机制
def wait_for_action_completion(timeout=10.0):
    """
    等待动作完成，直到 ROBOT_ACTION_STATUS 为 2 或超时。
    
    :param timeout: 超时时间（秒）
    """
    global ROBOT_ACTION_STATUS
    start_time = time.time()  # 记录开始时间
    while ROBOT_ACTION_STATUS != 2:
        if time.time() - start_time > timeout:  # 检查是否超时
            rospy.logwarn("等待动作完成超时，自动退出等待循环")
            break
        rospy.sleep(0.1)  # 等待0.1秒后再检查状态

# 执行动作的线程函数
def execute_arm_poses(arm_pose_names):
    for arm_pose in arm_pose_names:
        rospy.loginfo(f"Executing arm pose: {arm_pose}")
        call_execute_arm_action(arm_pose)
        wait_for_action_completion(timeout=10.0)  # 设置超时时间为10秒 | 等待动作完成

# 播放音乐的线程函数
def play_music(music_names):
    for music in music_names:
        rospy.loginfo(f"Playing music: {music}")
        set_robot_play_music(music, 100)

def customize_action_callback(event):
    global customize_config_data
    # 打印动作类型
    source = event.kwargs.get("source")
    trigger = event.kwargs.get("trigger")
    print_state_transition(trigger, source, "stance")
    try:
        # 根据 trigger 查找对应的配置
        if trigger in customize_config_data:
            action_config = customize_config_data[trigger]
            arm_pose_names = action_config.get("arm_pose_name", [])
            music_names = action_config.get("music_name", [])
            
            # 打印匹配到的动作和音乐信息
            rospy.loginfo(f"Trigger: {trigger}")
            rospy.loginfo(f"Arm Pose Names: {arm_pose_names}")
            rospy.loginfo(f"Music Names: {music_names}") 

            # 创建线程
            if arm_pose_names:
                arm_pose_thread = threading.Thread(target=execute_arm_poses, args=(arm_pose_names,))
                arm_pose_thread.start()
            if music_names:
                music_thread = threading.Thread(target=play_music, args=(music_names,))
                music_thread.start()

            # 等待线程完成
            if arm_pose_names:
                arm_pose_thread.join()
            if music_names:
                music_thread.join()
                
        else:
            rospy.logwarn(f"No configuration found for trigger: {trigger}")
    except Exception as e:
        rospy.logerr(f"Error in customize_action_callback: {e}")

def record_vr_rosbag_callback(event):
    global record_vr_rosbag_pid
    source = event.kwargs.get("source")
    trigger = event.kwargs.get("trigger")
    print_state_transition(trigger, source, "vr_remote_control")
    try:
        base_dir = os.path.expanduser(ROS_BAG_LOG_SAVE_PATH)
        current_date = datetime.datetime.now().strftime("%Y-%m-%d")
        date_folder = os.path.join(base_dir, current_date)
        if not os.path.exists(date_folder):
            os.makedirs(date_folder)
        
        base_filename = "vr_record"
        bag_file_base = os.path.join(date_folder, base_filename)
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
        actual_bag_file = f"{bag_file_base}_{timestamp}.bag"  # 实际会被创建的文件路径
        
        command = [
            "rosbag",
            "record",
            "-o",
            bag_file_base
        ]

        for topic in record_topics:
            command.append(topic)

        process = subprocess.Popen(
            command,
            start_new_session=True,
        )
        record_vr_rosbag_pid = process.pid
    except Exception as e:
        rospy.logerr(f"Error in record_vr_rosbag_callback: {e}")


def kill_record_vr_rosbag():
    global record_vr_rosbag_pid
    if record_vr_rosbag_pid:
        os.kill(record_vr_rosbag_pid, signal.SIGINT)
        record_vr_rosbag_pid = None

def stop_record_vr_rosbag_callback(event):
    source = event.kwargs.get("source")
    trigger = event.kwargs.get("trigger")
    print_state_transition(trigger, source, "vr_remote_control")
    kill_record_vr_rosbag()
