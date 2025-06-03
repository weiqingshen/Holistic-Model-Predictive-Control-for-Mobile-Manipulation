import json
import sympy as sp
import numpy as np
from sympy.parsing.sympy_parser import parse_expr
# 定义符号变量
x, y, theta = sp.symbols('x y theta')
q1, q2, q3, q4, q5, q6 = sp.symbols('q1 q2 q3 q4 q5 q6')
g = sp.symbols('g')

# 所有变量按顺序组成一个字典
symbol_vars = [x, y, theta, q1, q2, q3, q4, q5, q6, g]

def parse_expr(expr_str):
    """将字符串表达式转换为 sympy 表达式"""
    return sp.sympify(expr_str)

def input_joint_values():
    print("请输入10个浮点数（x y theta q1 q2 q3 q4 q5 q6 g），用空格分隔：")
    while True:
        try:
            values = list(map(float, input(">>> ").strip().split()))
            if len(values) != 10:
                raise ValueError("应输入10个浮点数")
            return values
        except Exception as e:
            print(f"输入错误：{e}，请重试")

def evaluate_distances(data, value_dict):
    results = []
    for item in data:
        # 若 item 是列表，取其第一个元素
        pair = item[0] if isinstance(item, list) else item

        a_pos = pair["sphere_a_world_position"]
        b_pos = pair["sphere_b_world_position"]

        # inside your evaluate_distances function:
        ax = float(parse_expr(a_pos[0]).evalf(subs=value_dict))
        ay = float(parse_expr(a_pos[1]).evalf(subs=value_dict))
        az = float(parse_expr(a_pos[2]).evalf(subs=value_dict))

        bx = float(parse_expr(b_pos[0]).evalf(subs=value_dict))
        by = float(parse_expr(b_pos[1]).evalf(subs=value_dict))
        bz = float(parse_expr(b_pos[2]).evalf(subs=value_dict))

        dist = np.linalg.norm([ax - bx, ay - by, az - bz])
        results.append((pair["link_a"], pair["link_b"], dist))
    return results

if __name__ == "__main__":
    with open("symbolic_positions.json", "r") as f:
        data = json.load(f)

    values = input_joint_values()
    value_dict = dict(zip(symbol_vars, values))

    print("\n各小球对之间的实际距离（单位：米）：")
    results = evaluate_distances(data, value_dict)
    for link_a, link_b, d in results:
        print(f"{link_a} 与 {link_b} 的距离为 {d:.6f}")
