
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import subprocess
import re
import time
from threading import Thread

class CameraReaderNode(Node):
    def __init__(self):
        super().__init__('camera_reader_node')
        self.declare_parameter('program_path', '/home/fins/myrobot_move/src/mpc_python/Exp_Homo_P2')
        self.program_path = self.get_parameter('program_path').get_parameter_value().string_value
        
        self.red_pub = self.create_publisher(Float32MultiArray, 'circle_red', 10)
        self.green_pub = self.create_publisher(Float32MultiArray, 'circle_green', 10)
        self.blue_pub = self.create_publisher(Float32MultiArray, 'circle_blue', 10)
        
        self.get_logger().info(f"相机读取节点已启动，程序路径: {self.program_path}")
        
        # 在独立线程中运行相机读取，避免阻塞主线程
        self.camera_thread = Thread(target=self.read_camera_output, daemon=True)
        self.camera_thread.start()
    
    def read_camera_output(self):
        try:
            process = subprocess.Popen(
                [self.program_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            # 正则表达式匹配输出的 Circle 信息
            patterns = {
                "red": re.compile(r"Circle_Red: *\(([-\d.]+), ([-\d.]+), ([-\d.]+)\)"),
                "green": re.compile(r"Circle_Green: *\(([-\d.]+), ([-\d.]+), ([-\d.]+)\)"),
                "blue": re.compile(r"Circle_Blue: *\(([-\d.]+), ([-\d.]+), ([-\d.]+)\)"),
            }

            
            while rclpy.ok():
                output = process.stdout.readline()
                if output == '' and process.poll() is not None:
                    break
                    
                for color, pattern in patterns.items():
                    match = pattern.search(output)
                    if match:
                        coord = tuple(map(float, match.groups()))
                        self.publish_coordinate(color, coord)

                time.sleep(0.01)  # 避免CPU占用过高

        except Exception as e:
            self.get_logger().error(f"读取相机输出时发生错误: {str(e)}")
        finally:
            if 'process' in locals():
                process.terminate()
                process.wait()
                self.get_logger().info("相机程序已终止")

    def publish_coordinate(self, color, coord):
        """发布坐标到ROS话题"""
        msg = Float32MultiArray()
        msg.data = list(coord)
        
        if color == 'red':
            self.red_pub.publish(msg)
        elif color == 'green':
            self.green_pub.publish(msg)
        elif color == 'blue':
            self.blue_pub.publish(msg)
            
        self.get_logger().info(f"发布 {color} 坐标: {coord}")

def main(args=None):
    rclpy.init(args=args)
    node = CameraReaderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()