#!/usr/bin/env python3

import rospy
from kuavo_msgs.msg import armTargetPoses
from kuavo_msgs.srv import changeArmCtrlModeRequest, changeArmCtrlModeResponse, changeArmCtrlMode
import os

def read_csv_file(file_path):
    times = []
    values = []
    with open(file_path, 'r') as f:
        for line in f:
            tokens = line.strip().split()
            if not tokens:
                continue  # 跳过空行
            # 跳过第一列文字
            tokens = tokens[1:]
            if len(tokens) < 15:  # 需要至少1个时间和28个关节角度
                rospy.logwarn("行数据不足，已跳过: %s", line)
                continue
            time = float(tokens[0])
            joint_angles = [float(x) for x in tokens[1:15]]  # 28个关节角度
            times.append(time)
            values.extend(joint_angles)  # 将关节角度展平为一个列表
    return times, values

def change_arm_ctrl_mode(control_mode):
    rospy.wait_for_service('/arm_traj_change_mode')
    try:
        change_mode = rospy.ServiceProxy('/arm_traj_change_mode', changeArmCtrlMode)
        req = changeArmCtrlModeRequest()
        req.control_mode = control_mode
        res = change_mode(req)
        if res.result:
            rospy.loginfo("手臂控制模式已更改为 %d", control_mode)
        else:
            rospy.logerr("无法将手臂控制模式更改为 %d", control_mode)
    except rospy.ServiceException as e:
        rospy.logerr("服务调用失败: %s", e)

def main():
    rospy.init_node('csv_to_topic_publisher')
    pub = rospy.Publisher('kuavo_arm_target_poses', armTargetPoses, queue_size=10)

    # 使用您指定的CSV文件路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_file_name = 'actions/poses_0.csv'  # 请替换为您的CSV文件名
    file_path = os.path.join(script_dir, csv_file_name)

    if not os.path.isfile(file_path):
        rospy.logerr("文件不存在: %s", file_path)
        return

    times, values = read_csv_file(file_path)

    if not times or not values:
        rospy.logerr("读取CSV文件失败，数据为空。")
        return
    # 在发布之前，调用服务将控制模式设置为1
    change_arm_ctrl_mode(2)

    msg = armTargetPoses()
    msg.times = times
    msg.values = values

    rospy.loginfo("正在将手臂目标姿态发布到话题 'kuavo_arm_target_poses'")
    # 等待订阅者连接
    rate = rospy.Rate(10)  # 10Hz
    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.loginfo("等待订阅者连接...")
        rate.sleep()
    pub.publish(msg)
    rospy.loginfo("消息已发布。")

    # 在发布完成后，调用服务将控制模式设置为0
    # change_arm_ctrl_mode(0)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
