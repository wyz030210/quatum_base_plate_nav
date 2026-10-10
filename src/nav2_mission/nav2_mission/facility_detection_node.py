#!/usr/bin/env python3   告诉 Linux 系统用 python3 解释器运行这个文件
"""
设备设施图像检测 ROS2 节点
到达巡检点后由 patrol_mission_node 调用
消防栓和路灯使用同一个模型 ludeng.pt，按类别 ID 区分
"""
import rclpy                       #ROS2 Python 客户端库
from rclpy.node import Node        #所有 ROS2 节点的基类，你的节点要继承它
from std_srvs.srv import Trigger   #ROS2 标准 Service 消息类型，只有 success 和 message 两个字段。这就是 patrol_mission_node 调用检测的"接口"
import cv2                         #OpenCV 用于从相机拍照、保存结果图片
from ultralytics import YOLO       #YOLO 目标检测框架，用来加载和运行你的路灯/消防栓模型
import os                          #用于创建保存目录、拼接文件路径
from datetime import datetime      #给保存的结果图片加时间戳，防止重名覆盖


class FacilityDetectionNode(Node):                       #定义一个 ROS2 节点类，继承 Node
    def __init__(self):
        super().__init__('facility_detection_node')          #调用父类构造函数，给节点注册一个名字叫 facility_detection_node。启动后在 ROS2 网络中可以用这个名字找到它

        # 读取 launch 文件传入的参数
        self.declare_parameter('model_path', r'/path/to/ludeng.pt')
        self.declare_parameter('save_dir', r'/path/to/results')
        self.declare_parameter('conf_threshold', 0.5)
        self.declare_parameter('detection_service_name', '/detect_facility')

        self.model_path = self.get_parameter('model_path').value
        self.save_dir = self.get_parameter('save_dir').value
        self.conf_threshold = self.get_parameter('conf_threshold').value
        detection_service_name = self.get_parameter('detection_service_name').value

        # --- 配置区域 ---
        #self.model_path = r'/path/to/ludeng.pt'      #YOLO 需要从 .pt 文件加载训练好的权重

        # 类别映射：根据模型训练时的 class id 对应
        # 请根据你的模型实际训练时的类别 ID 修改下面的映射
        self.class_names = {
            0: 'garbage can',    # class 0 = 路灯
            1: 'street light',  # class 1 = 消防栓
        }
        #self.save_dir = r'/path/to/results'               #每次检测完把带标注框的图片保存到这里
        self.camera_source = 0                            # 相机来源
        #self.conf_threshold = 0.5                         # 置信度阈值 只有模型 50% 以上确信的检测结果才会被保留，低于这个值的会被过滤掉，减少误报
        # --- 配置结束 ---

        # 加载模型
        self.get_logger().info('Loading detection models...')
        self.model = YOLO(self.model_path)
        self.get_logger().info(f'Model loaded: {self.model_path}')

        # 创建 Service（接收巡检节点的检测请求）
        self.create_service(Trigger, detection_service_name, self.detect_callback)   
        #Trigger：Service 消息类型（只有 success + message 两个字段）
        #'/detect_facility'：Service 名称，其他节点通过这个名字来调用你
        #self.detect_callback：收到请求后执行的回调函数

        # 创建结果保存目录
        os.makedirs(self.save_dir, exist_ok=True)

        self.get_logger().info('Facility detection node ready, waiting for requests...')

    #检测回调函数
    def detect_callback(self, request, response):
        """收到巡检节点请求后执行检测"""
        self.get_logger().info('🔍 Starting facility detection...')

        # 1. 从相机获取图像
        cap = cv2.VideoCapture(self.camera_source)          #cv2.VideoCapture(self.camera_source)：打开相机设备。0 是 USB 摄像头，
        ret, frame = cap.read()                             #cap.read()：从相机读取一帧图像。ret 是布尔值（是否成功），frame 是图像数组
        cap.release()                                       #用完立即释放相机资源，防止相机被占用

        if not ret:                                         #如果拍照失败，返回失败响应，巡检节点会记录错误
            response.success = False
            response.message = "Failed to capture image"
            self.get_logger().error('Cannot capture image from camera')
            return response

        # 2. 运行检测（只加载一个模型，只推理一次）
        results = self.model.predict(
            frame, conf=self.conf_threshold, verbose=False
        )
        result = results[0]  # 取第一张图的结果

        # 3. 按类别 ID 分别统计
        class_counts = {cls_id: 0 for cls_id in self.class_names}
        # YOLO 自动在图上绘制所有检测框
        annotated_frame = result.plot()

        for box in result.boxes:
            cls_id = int(box.cls[0])
            if cls_id in class_counts:
                class_counts[cls_id] += 1

        # 4. 保存结果图像
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')  #datetime.now().strftime('%Y%m%d_%H%M%S')：生成时间戳
        result_path = os.path.join(self.save_dir, f'detection_{timestamp}.jpg')      #os.path.join：拼接完整保存路径
        cv2.imwrite(result_path, annotated_frame)                                    #把标注后的图像写入文件

        # 5. 返回结果
        total = sum(class_counts.values())
        count_str = ', '.join(
            [f'{self.class_names[cid]}:{cnt}' for cid, cnt in class_counts.items() if cnt > 0]
        )
        if not count_str:
            count_str = 'No detections'
        response.success = True
        response.message = (
            f"{count_str}, Total:{total}, "
            f"Results saved to {result_path}"
        )
        self.get_logger().info(f'✅ Detection done — {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)                                                         #初始化 ROS2 系统，建立和 ROS2 通信层的连接
    node = FacilityDetectionNode()                                                #创建节点实例
    try:
        rclpy.spin(node)                                                          #让节点持续运行，监听 Service 请求。没有这行，节点启动就退出了
    except KeyboardInterrupt:                                                     #用户按 Ctrl+C 时优雅退出
        pass
    finally:
        node.destroy_node()                                                       #销毁节点，释放 ROS2 资源
        rclpy.shutdown()                                                          #关闭 ROS2 系统


if __name__ == '__main__':                                                              #判断文件是否被直接运行（而不是被 import）
    main()