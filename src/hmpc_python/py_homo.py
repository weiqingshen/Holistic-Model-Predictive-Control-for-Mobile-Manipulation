#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import subprocess
import re
import time
import threading


class CircleCoordinatesPublisher(Node):
    """
    简单的ROS2节点：从C++程序捕获坐标并发布到话题
    """

    def __init__(self):
        super().__init__('circle_coordinates_publisher')

        # 配置参数
        self.program_path = "/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Exp_Homo_P2"
        self.publish_rate = 10.0  # Hz

        # 创建发布者
        self.red_publisher = self.create_publisher(Point, 'circle_coordinates/red', 10)
        self.green_publisher = self.create_publisher(Point, 'circle_coordinates/green', 10)
        self.blue_publisher = self.create_publisher(Point, 'circle_coordinates/blue', 10)

        # 正则表达式模式
        self.patterns = {
            "red": re.compile(r"Circle_Red: *\(([-\d.]+), ([-\d.]+), ([-\d.]+)\)"),
            "green": re.compile(r"Circle_Green: *\(([-\d.]+), ([-\d.]+), ([-\d.]+)\)"),
            "blue": re.compile(r"Circle_Blue: *\(([-\d.]+), ([-\d.]+), ([-\d.]+)\)"),
        }

        # 坐标存储
        self.coords = {"red": None, "green": None, "blue": None}
        self.coords_lock = threading.Lock()

        # 创建定时器用于定期发布坐标
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_coordinates)

        # 启动坐标捕获线程
        self.capture_thread = threading.Thread(target=self.capture_coordinates_loop, daemon=True)
        self.capture_thread.start()

        self.get_logger().info('Circle Coordinates Publisher started')

    def start_capture_process(self):
        """启动并持续监听C++程序"""
        try:
            self.process = subprocess.Popen(
                [self.program_path],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,  # 行缓冲
                universal_newlines=True
            )
            self.get_logger().info(f'Started C++ program: {self.program_path}')
            return True
        except Exception as e:
            self.get_logger().error(f'Failed to start program: {e}')
            return False

    def capture_coordinates_continuously(self):
        """持续从C++程序输出中读取坐标"""
        while rclpy.ok():
            try:
                if not hasattr(self, 'process') or self.process.poll() is not None:
                    # 程序未启动或已结束，重新启动
                    if not self.start_capture_process():
                        time.sleep(5)
                        continue

                # 读取一行输出
                output = self.process.stdout.readline()
                if output:
                    # 检查每种颜色的坐标
                    for color, pattern in self.patterns.items():
                        match = pattern.search(output)
                        if match:
                            new_coord = tuple(map(float, match.groups()))
                            with self.coords_lock:
                                self.coords[color] = new_coord
                            #self.get_logger().info(f"Updated {color.capitalize()}: {new_coord}")
                else:
                    # 没有输出，短暂等待
                    time.sleep(0.01)

            except Exception as e:
                self.get_logger().error(f'Error in capture loop: {e}')
                # 清理并重启进程
                if hasattr(self, 'process'):
                    self.process.terminate()
                time.sleep(5)

    def capture_coordinates_loop(self):
        """持续捕获坐标的线程"""
        self.capture_coordinates_continuously()

    def publish_coordinates(self):
        """发布坐标到ROS话题"""
        with self.coords_lock:
            # 发布红色圆坐标
            if self.coords["red"] is not None:
                red_point = Point()
                red_point.x = float(self.coords["red"][0])
                red_point.y = float(self.coords["red"][1])
                red_point.z = float(self.coords["red"][2])
                self.red_publisher.publish(red_point)
                # print(f"Published Red: ({red_point.x:.3f}, {red_point.y:.3f}, {red_point.z:.3f})")

            # 发布绿色圆坐标
            if self.coords["green"] is not None:
                green_point = Point()
                green_point.x = float(self.coords["green"][0])
                green_point.y = float(self.coords["green"][1])
                green_point.z = float(self.coords["green"][2])
                self.green_publisher.publish(green_point)
                # print(f"Published Green: ({green_point.x:.3f}, {green_point.y:.3f}, {green_point.z:.3f})")

            # 发布蓝色圆坐标
            if self.coords["blue"] is not None:
                blue_point = Point()
                blue_point.x = float(self.coords["blue"][0])
                blue_point.y = float(self.coords["blue"][1])
                blue_point.z = float(self.coords["blue"][2])
                self.blue_publisher.publish(blue_point)
                # print(f"Published Blue: ({blue_point.x:.3f}, {blue_point.y:.3f}, {blue_point.z:.3f})")


def main():
    """主函数"""
    rclpy.init()

    try:
        node = CircleCoordinatesPublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()