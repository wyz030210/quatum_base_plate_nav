#!/usr/bin/env python3

import rclpy
import sys, select, termios, tty
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

msg = """
控制车辆移动:
---------------------------
   q    w    e
   a    s    d
        x

w/x : 前进/后退
a/d : 左转/右转
q   : 前进+左转
e   : 前进+右转
s   : 停止
+/- : 增加/减少速度
l   : 切换头灯/滚刷（开/关）
CTRL-C 退出

组合控制说明:
- 可以同时按住多个键实现组合动作
- 例如：w+a = 前进+左转
- 按+/-键可以实时调整速度
- l 是开关切换键（不是按住生效），每按一次翻转一次头灯/滚刷的状态
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(speed, turn):
    return "当前速度:\t线速度 %s\t角速度 %s " % (speed, turn)

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node('keyboard_control')
    pub = node.create_publisher(Twist, '/cmd_vel', 1)
    # 2026-07-13 新增：头灯/滚刷合并继电器开关（PF0 同一路输出同时接工作
    # 头灯和滚刷电机，硬件上捆绑在一起的单个开关量），发布到
    # can_bridge_node 新增的 /vehicle/headlight_brush_cmd（Bool），由
    # can_bridge_node 转成 CAN ID 0x06 command_type=0x05 发给STM32。这里
    # 只负责发指令，不订阅 /vehicle/headlight_brush_state 回读——按键脚本
    # 本身没有反馈显示需求，状态仅在本地 headlight_brush_on 变量里跟踪
    # 用于下一次切换。
    headlight_brush_pub = node.create_publisher(Bool, '/vehicle/headlight_brush_cmd', 1)
    headlight_brush_on = False

    # 直接设置默认速度值，不依赖ROS参数
    # (rclpy: 默认值直接作为 declare_parameter 的第二个参数，效果与原 rospy
    # 版本先赋本地默认值再用 get_param 覆盖一致)
    node.declare_parameter('speed', 3.0)
    node.declare_parameter('turn', 1.0)

    speed = float(node.get_parameter('speed').value)
    turn = float(node.get_parameter('turn').value)

    # 如果发现线速度被缩放，可以在这里预先补偿
    # speed = speed * 4.0  # 如果需要的话取消注释这行
    x = 0
    y = 0
    z = 0
    th = 0
    status = 0

    try:
        print(msg)
        print(vels(speed, turn))
        print("实际读取的参数: speed=%.2f, turn=%.2f" % (speed, turn))
        while(1):
            key = getKey()
            if key in ['w', 'W']:
                x = speed
                y = 0
                z = 0
                th = 0
            elif key in ['s', 'S']:
                x = 0
                y = 0
                z = 0
                th = 0
            elif key in ['a', 'A']:
                x = 0
                y = 0
                z = 0
                th = turn
            elif key in ['d', 'D']:
                x = 0
                y = 0
                z = 0
                th = -turn
            elif key in ['q', 'Q']:
                # 前进+左转
                x = speed
                y = 0
                z = 0
                th = turn
            elif key in ['e', 'E']:
                # 前进+右转
                x = speed
                y = 0
                z = 0
                th = -turn
            elif key in ['x', 'X']:
                x = -speed
                y = 0
                z = 0
                th = 0
            elif key == '+' or key == '=':
                speed = min(speed + 0.2, 50.0)  # 每次增加0.2 m/s，最大6.0 m/s
                print("速度增加到: %.1f m/s" % speed)
                continue
            elif key == '-' or key == '_':
                speed = max(speed - 0.2, 0.1)  # 每次减少0.2 m/s，最小0.1 m/s
                print("速度减少到: %.1f m/s" % speed)
                continue
            elif key in ['l', 'L']:
                # 开关切换（不是按住生效）：每按一次翻转一次本地记录的状态，
                # 发布给 can_bridge_node 的 /vehicle/headlight_brush_cmd
                headlight_brush_on = not headlight_brush_on
                headlight_brush_msg = Bool()
                headlight_brush_msg.data = headlight_brush_on
                headlight_brush_pub.publish(headlight_brush_msg)
                print("头灯/滚刷: %s" % ("开" if headlight_brush_on else "关"))
                continue
            else:
                x = 0
                y = 0
                z = 0
                th = 0
                if (key == '\x03'):
                    break

            twist = Twist()
            twist.linear.x = float(x)
            twist.linear.y = float(y)
            twist.linear.z = float(z)
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = float(th)
            pub.publish(twist)

            # 显示当前发送的控制指令
            if x != 0 or th != 0:
                print("发送控制指令: 线速度=%.2f, 角速度=%.2f" % (x, th))

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
