import distance_count_bind

def test_compute_joint_distances():
    """
    测试 compute_joint_distances 函数，确保它能够正确计算机械臂关节之间的最小距离，并显示对应的关节名称。
    """
    # ✅ 关节值：确保匹配 MoveIt!
    #对应join
    joint_values = [0.0, -1.2, 0.0, 0.0, 0.0, 0.0,0.001, 0.5, 0.0, 0.6]

    print(f"\n🔢 传入的关节值: {joint_values}")

    # 计算连杆之间的最短距离
    try:
        distances = distance_count_bind.compute_joint_distances(joint_values)
        print("\n✅ 机械臂各连杆之间的最短距离:")
        for result in distances:
            print(f"  [{result.link1}] ↔ [{result.link2}]: {result.distance:.6f} m")
    except Exception as e:
        print("\n❌ 测试失败，错误信息:")
        print(str(e))

if __name__ == "__main__":
    print("\n🔍 测试 `distance_count_bind` Python 绑定...")

    # **测试 Python 模块是否正确导入**
    try:
        print(f"✅ 模块导入成功: {distance_count_bind}")
    except ImportError as e:
        print(f"\n❌ 无法导入 `distance_count_bind`，错误: {e}")
        exit(1)

    # **测试 compute_joint_distances**
    test_compute_joint_distances()
