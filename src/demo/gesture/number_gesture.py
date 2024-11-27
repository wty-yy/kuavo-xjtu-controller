#!/usr/bin/env python3
# coding: utf-8

import rospy
from kuavo_msgs.msg import gestureTask
from kuavo_msgs.srv import gestureExecute, gestureExecuteRequest

def execute_number_gesture():
    # 服务的名称，需要与服务器端注册的服务名一致
    service_name = 'gesture/execute'
    
    # 等待服务可用
    rospy.wait_for_service(service_name)
    
    try:
        # 创建服务代理
        gesture_service = rospy.ServiceProxy(service_name, gestureExecute)
        
        # 创建请求对象
        request = gestureExecuteRequest()

        for i in range(1, 9):
            gesture = gestureTask(gesture_name='number_' + str(i), hand_side=0)
            request.gestures.append(gesture)

        # reset hand gesture.
        request.gestures.append(gestureTask(gesture_name='empty', hand_side=2))

        print(request)    
           
        # 调用服务
        response = gesture_service(request)
        
        # 处理响应
        if response.success:
            print(f"Gesture number_1 ~ number_8 executed successfully.")
        else:
            print(f"Failed to execute gesture 'number_1 ~ number_8': {response.message}")
    
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")

if __name__ == "__main__":
    # 初始化节点
    rospy.init_node('gesture_client')

    # 执行数字手势: 1 ~8
    execute_number_gesture()