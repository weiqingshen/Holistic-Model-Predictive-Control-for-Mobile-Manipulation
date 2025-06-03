import subprocess
import re
import time
from threading import Thread, Event


def read_camera_output(
        program_path='/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Exp_Homo_P2',
        update_callback=None,
        stop_event=None):
    """
    读取相机程序输出并解析圆形坐标信息的独立函数

    参数:
        program_path: 外部相机程序的路径
        update_callback: 坐标更新时调用的回调函数
        stop_event: 用于停止线程的事件对象
    """
    if stop_event is None:
        stop_event = Event()

    def parse_output():
        try:
            # 启动外部程序
            process = subprocess.Popen(
                [program_path],
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

            print(f"相机读取已启动，程序路径: {program_path}")

            while not stop_event.is_set():
                # 读取程序输出
                output = process.stdout.readline()
                if output == '' and process.poll() is not None:
                    break

                # 解析坐标信息
                for color, pattern in patterns.items():
                    match = pattern.search(output)
                    if match:
                        coord = tuple(map(float, match.groups()))

                        # 打印坐标信息（替代ROS发布）
                        print(f"检测到 {color} 坐标: {coord}")

                        # 调用回调函数（如果提供）
                        if update_callback:
                            update_callback(color, coord)

                time.sleep(0.01)  # 避免CPU占用过高

        except Exception as e:
            print(f"读取相机输出时发生错误: {str(e)}")
        finally:
            if 'process' in locals():
                process.terminate()
                process.wait()
                print("相机程序已终止")

    # 在独立线程中运行解析函数
    thread = Thread(target=parse_output, daemon=True)
    thread.start()

    return thread, stop_event


# 使用示例
if __name__ == "__main__":
    # 创建停止事件
    stop_event = Event()

    # 启动相机读取
    thread, event = read_camera_output(
        program_path='/home/fins/dev_ws/src/state_machine/state_machine/FineVision/Build/Bin/Exp_Homo_P2',
        stop_event=stop_event
    )

    try:
        print("按 Ctrl+C 停止程序")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n程序正在停止...")
        stop_event.set()  # 发出停止信号
        thread.join(timeout=2.0)  # 等待线程结束
        print("程序已停止")
