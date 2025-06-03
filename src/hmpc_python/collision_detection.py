import os
import json
import numpy as np
from scipy.spatial.distance import cdist
from collections import defaultdict

IGNORE_PAIRS = {
    ("base_link", "link1"), ("link1", "base_link"),
    ("link1", "link2"), ("link2", "link1"),
    ("link1", "link3"), ("link3", "link1"),
    ("link2", "link3"), ("link3", "link2"),
    ("link3", "link4"), ("link4", "link3"),
    ("link3", "link5"), ("link5", "link3"),
    ("link3", "link6"), ("link6", "link3"),
    ("link4", "link5"), ("link5", "link4"),
    ("link4", "link6"), ("link6", "link4"),
    ("link5", "link6"), ("link6", "link5"),
    ("link5", "link_hand"), ("link_hand", "link5"),
    ("link5", "link_left"), ("link_left", "link5"),
    ("link5", "link_right"), ("link_right", "link5"),
    ("link6", "link_hand"), ("link_hand", "link6"),
    ("link6", "link_left"), ("link_left", "link6"),
    ("link6", "link_right"), ("link_right", "link6"),
    ("link_hand", "link_left"), ("link_left", "link_hand"),
    ("link_hand", "link_right"), ("link_right", "link_hand"),
    ("link_left", "link_right"), ("link_right", "link_left")
}

def euler_to_rotation_matrix(rpy):
    roll, pitch, yaw = rpy
    cr, sr = np.cos(roll), np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ])

def compute_transforms(joint_angles):
    def make_transform(rpy, xyz):
        R = euler_to_rotation_matrix(rpy)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = xyz
        return T

    T = {}
    base_x, base_y, base_theta = joint_angles[0:3]
    T["base_link"] = make_transform([0, 0, base_theta], [base_x, base_y, 0])
    T["obstacle"] = make_transform([0, 0, 0], [0, 0, 0])
    T["link1"] = T["base_link"] @ make_transform([0, 0, joint_angles[3]], [0, 0, 0.1284])
    T["link2"] = T["link1"] @ make_transform([1.5708, -joint_angles[4], 1.5708], [0, 0, 0.0927])
    T["link3"] = T["link2"] @ make_transform([-3.1416, 0, -1.5708 + joint_angles[5]], [0, 0.22, 0])
    T["link4"] = T["link3"] @ make_transform([1.5708, -joint_angles[6], 0], [0, 0, 0])
    T["link5"] = T["link4"] @ make_transform([1.5708, -1.5708 + joint_angles[7], 0], [0, 0, 0.1685])
    T["link6"] = T["link5"] @ make_transform([1.5708, -joint_angles[8], 0], [0, 0, 0])
    T["link_hand"] = T["link6"] @ make_transform([0, 0, 0], [0, 0, 0])
    g = joint_angles[9] if len(joint_angles) > 9 else 0.001

    T["link_right"] = T["link6"] @ make_transform([-1.5708, 0, 0], [0, -g/2, 0.1465])
    T["link_left"] = T["link6"] @ make_transform([-1.5708, 0, 0], [0, g/2, 0.1465])
    return T

def compute_min_distances(transforms, rel_spheres):
    world_positions = {}
    for link, spheres in rel_spheres.items():
        if link not in transforms:
            continue
        T_link = transforms[link]
        world_positions[link] = [ (T_link @ np.array(pos + [1.0]))[:3] for pos in spheres ]
        world_positions[link] = np.array(world_positions[link])

    pairwise_candidates = defaultdict(list)
    links = list(world_positions.keys())
    for i in range(len(links)):
        for j in range(i + 1, len(links)):
            a, b = links[i], links[j]
            if (a, b) in IGNORE_PAIRS:
                continue
            pos_a, pos_b = world_positions[a], world_positions[b]
            dists = cdist(pos_a, pos_b)
            min_dist = np.min(dists)
            pairwise_candidates[(a, b)].append(min_dist)

    results = []
    for pair_key, distances in pairwise_candidates.items():
        results.append((pair_key[0], pair_key[1], min(distances)))
    return results

def detect_collisions(joint_angles, sphere_json_path="link_spheres_relative_v2.json"):
    with open(sphere_json_path, "r") as f:
        rel_spheres = json.load(f)
    transforms = compute_transforms(joint_angles)
    collisions = compute_min_distances(transforms, rel_spheres)
    return collisions
