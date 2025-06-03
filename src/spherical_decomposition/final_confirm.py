import json
import sympy as sp
from final_symbolic_transform import get_symbolic_transforms_of_closest_spheres

# 定义符号变量（必须与主程序一致）
x, y, theta = sp.symbols('x y theta')
q1, q2, q3, q4, q5, q6 = sp.symbols('q1 q2 q3 q4 q5 q6')
g = sp.symbols('g')
joint_symbols = [x, y, theta, q1, q2, q3, q4, q5, q6, g]

# 用实际值替换符号并计算表达式
def evaluate_symbolic_distances(symbolic_results, joint_values):
    subs_dict = dict(zip(joint_symbols, joint_values))

    print("\n符号表达式与数值最短距离差值：\n")
    for pair, min_dist_val, dist_expr in symbolic_results:
        evaluated = dist_expr.evalf(subs=subs_dict)
        error = abs(evaluated - min_dist_val)
        print(f"{pair}:")
        print(f"  符号表达式计算值 = {evaluated:.6f}")
        print(f"  原最短值 = {min_dist_val:.6f}")
        print(f"  差值 = {error:.6e}\n")

if __name__ == "__main__":
    symbolic_results = get_symbolic_transforms_of_closest_spheres("output_results.json")
    joint_values = [0.5, 0.5, 0.5,  0, -1, 0 ,0, -1, 1, 0.01]
    evaluate_symbolic_distances(symbolic_results, joint_values)
