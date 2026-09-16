"""
 DWA算法
author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı
Source: https://github.com/AtsushiSakai/PythonRobotics

License: MIT
"""
import sys
from pathlib import Path

# 当前脚本文件
FILE = Path(__file__).resolve()

# 往上两层，拿到 code_python 根目录
PROJECT_ROOT = FILE.parent.parent
sys.path.append(str(PROJECT_ROOT))

import utils.grid_map_data as data

import utils.plt_dynamic as dynamic

import math
from enum import Enum
import matplotlib.pyplot as plt
import numpy as np
from typing import List

show_animation = True

class RobotType(Enum):
    circle = 0
    rectangle = 1

class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 1.0  # [m/s] 容许最大线速度
        self.min_speed = -0.5  # [m/s] 容许最小线速度（负值代表后退）
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # [rad/s] 最大航向角速度（最大旋转速度，40 deg/s转弧度）
        self.max_accel = 0.2  # [m/s²] 容许最大线加速度
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/s²] 最大角加速度（角速度变化上限）

        self.v_resolution = 0.01  # [m/s] 线速度采样步长，DWA窗口内v每隔0.01m/s采一个样本
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s] 角速度采样步长

        self.dt = 0.1  # [s] 运动学模型预测的时间步长
        self.predict_time = 3.0  # [s] 向前预测轨迹总时长，每条候选轨迹预测未来3秒

        self.to_goal_cost_gain = 0.15  # 目标航向代价权重 heading cost
        self.speed_cost_gain = 1.0  # 速度代价权重（倾向高速）
        self.obstacle_cost_gain = 1.0  # 障碍物避障代价权重

        self.robot_stuck_flag_cons = 0.001  # 防止机器人卡死的阈值常量

        self.robot_type = RobotType.circle  # 机器人碰撞模型：圆形包络

        # 圆形碰撞模型参数，也用于矩形模型判断是否到达目标点
        self.robot_radius = 1.0  # [m] 机器人外接圆半径，碰撞检测用
        # 矩形机器人参数
        self.robot_width = 0.5  # [m] 机器人宽度
        self.robot_length = 1.2  # [m] 机器人长度

        # obstacles [x(m) y(m), ....]
        self.obstacle = 5

    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value
config = Config()

def dwa_control(state, config, goal, obstacle):
    """
    Dynamic Window Approach control
    返回 预测点轨迹，及 预测轨迹
    """
    # 由当前状态 得到 动态窗口
    # dw = #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
    dw = calc_dynamic_window(state, config)
    # 代入当前状态、动态窗口 得到 预测 后 点的运动速度 、预测的轨迹
    twist, trajectory = calc_control_and_trajectory(state, dw, config, goal, obstacle)
    return twist, trajectory

# 返回 更新后的state[x,y,yaw,v,]
def motion(state, twist, dt):
    """
    motion model
    state[pos_x,pos_y,yaw,v,y]：【x坐标，y坐标，航向角、线速度、角速度】
    twist[v,y]
    """
    # 更新 航向角（ 更新后航向角 = 原航向角 + 角度度 * dt时间  ）
    state[2] += twist[1] * dt
    # 更新 x坐标 （更新后的x坐标 = 原x坐标 + 线速度 * dt时间 * 更新后的航向角）
    state[0] += twist[0] * dt * math.cos(state[2])
    # 更新 y坐标 （更新后的y坐标 = 原y坐标 + 线速度 * dt时间 * 更新后的航向角）
    state[1] += twist[0] * dt * math.sin(state[2])
    # 更新 线速度
    state[3] = twist[0]
    # 更新 角速度
    state[4] = twist[1]
    # 返回 一个dt 时间 更新后状态
    return state

# 返回动态窗口dw
def calc_dynamic_window(state, config):
    """
    动态窗口 基于当前状态 的 线速度和角速度 计算动态窗口 返回 允许的动态窗口
    """
    # Dynamic window from robot specification 由机器人自身定义的 线速度域 和 角速度域 确定 （角速度也有方向）
    Vs = [config.min_speed, config.max_speed, -config.max_yaw_rate, config.max_yaw_rate]
    # Dynamic window from motion model
    Vd = [state[3] - config.max_accel * config.dt,
          state[3] + config.max_accel * config.dt,
          state[4] - config.max_delta_yaw_rate * config.dt,
          state[4] + config.max_delta_yaw_rate * config.dt]
    #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
    dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]
    return dw

def predict_trajectory(current_state_init, v, y, config):
    """
    predict trajectory with an input
    """
    state = np.array(current_state_init)
    trajectory = np.array(state)
    time = 0
    # predict_time 3s,dt 0.1s,由0s开始，小步长更新状态，直到满足预测时间
    while time <= config.predict_time:
        # 预测 dt 后的状态
        state = motion(state, [v, y], config.dt)
        # 保留状态轨迹
        trajectory = np.vstack((trajectory, state))
        # 更新已预测时间
        time += config.dt
    # 返回 当前状态 的 的预测轨迹
    return trajectory

def calc_control_and_trajectory(state, dw, config, goal, obstacle):
    """
    返回 最优 预测点 和 最优 预测轨迹
    dw 允许的动态窗口
    calculation final input with dynamic window
    """
    # 当前状态 初始化
    current_state_init = state[:]
    min_cost = float("inf")
    # 预测后的 机器人运动速度 【线速度 、角速度】
    best_predict_twist = [0.0, 0.0]
    # 预测轨迹初始化
    best_trajectory = np.array([state])
    # evaluate all trajectory with sampled input in dynamic window
    # dw [v_min, v_max, y_min, y_max]
    # 离散 线速度空间 v、 角速度空间 y
    for v in np.arange(dw[0], dw[1], config.v_resolution):
        for y in np.arange(dw[2], dw[3], config.yaw_rate_resolution):
            # 对于不同 采样后的 线速度、角速度 预测 轨迹（一个预测时间）
            trajectory = predict_trajectory(current_state_init, v, y, config)
            # 分别计算三种代价
            to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(trajectory, goal)
            speed_cost = config.speed_cost_gain * (config.max_speed - trajectory[-1, 3])
            ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajectory, obstacle, config)
            # 计算总代价
            final_cost = to_goal_cost + speed_cost + ob_cost
            # search minimum trajectory
            if min_cost >= final_cost:
                min_cost = final_cost
                best_predict_twist = [v, y]
                best_trajectory = trajectory
                if abs(best_predict_twist[0]) < config.robot_stuck_flag_cons \
                        and abs(state[3]) < config.robot_stuck_flag_cons:
                    # to ensure the robot do not get stuck in
                    # best v=0 m/s (in front of an obstacle) and
                    # best omega=0 rad/s (heading to the goal with
                    # angle difference of 0)
                    best_predict_twist[1] = -config.max_delta_yaw_rate
    return best_predict_twist, best_trajectory

def calc_obstacle_cost(trajectory, ob, config):
    """
    calc obstacle cost inf: collision
    """
    ox = ob[:, 0]
    oy = ob[:, 1]
    dx = trajectory[:, 0] - ox[:, None]
    dy = trajectory[:, 1] - oy[:, None]
    r = np.hypot(dx, dy)
    if config.robot_type == RobotType.rectangle:

        yaw = trajectory[:, 2]
        rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
        rot = np.transpose(rot, [2, 0, 1])
        local_ob = ob[:, None] - trajectory[:, 0:2]
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        local_ob = np.array([local_ob @ x for x in rot])
        local_ob = local_ob.reshape(-1, local_ob.shape[-1])
        upper_check = local_ob[:, 0] <= config.robot_length / 2
        right_check = local_ob[:, 1] <= config.robot_width / 2
        bottom_check = local_ob[:, 0] >= -config.robot_length / 2
        left_check = local_ob[:, 1] >= -config.robot_width / 2
        if (np.logical_and(np.logical_and(upper_check, right_check),
                           np.logical_and(bottom_check, left_check))).any():
            return float("Inf")
    elif config.robot_type == RobotType.circle:
        if np.array(r <= config.robot_radius).any():
            return float("Inf")
    min_r = np.min(r)
    return 1.0 / min_r  # OK

def calc_to_goal_cost(trajectory, goal):
    """
        calc to goal cost with angle difference
        trajectory 最后一行是 轨迹的最后状态，0 是 x坐标 ，1 是 y坐标
    """
    # 计算预测后位置 与 目标点位置的 差值
    dx = goal[0] - trajectory[-1, 0]
    dy = goal[1] - trajectory[-1, 1]
    # 目标航向角差值 计算 目标点航向角由差值计算方法
    error_angle = math.atan2(dy, dx)
    # 计算 当前航向角 与 目标航向角差值
    cost_angle = error_angle - trajectory[-1, 2]
    #
    cost = abs( math.atan2(math.sin(cost_angle), math.cos(cost_angle)) )
    return cost

def plot_arrow(x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
    plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
              head_length=width, head_width=width)
    plt.plot(x, y)

def plot_robot(x, y, yaw, config):  # pragma: no cover
    if config.robot_type == RobotType.rectangle:
        outline = np.array([[-config.robot_length / 2, config.robot_length / 2,
                             (config.robot_length / 2), -config.robot_length / 2,
                             -config.robot_length / 2],
                            [config.robot_width / 2, config.robot_width / 2,
                             - config.robot_width / 2, - config.robot_width / 2,
                             config.robot_width / 2]])
        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                         [-math.sin(yaw), math.cos(yaw)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x
        outline[1, :] += y
        plt.plot(np.array(outline[0, :]).flatten(),
                 np.array(outline[1, :]).flatten(), "-k")
    elif config.robot_type == RobotType.circle:
        circle = plt.Circle((x, y), config.robot_radius, color="b")
        plt.gcf().gca().add_artist(circle)
        out_x, out_y = (np.array([x, y]) +
                        np.array([np.cos(yaw), np.sin(yaw)]) * config.robot_radius)
        plt.plot([x, out_x], [y, out_y], "-k")

def main(target_x=10.0, target_y=10.0, robot_type=RobotType.circle):
    print(__file__ + " start!!")
    # 初始化 机器人状态 x坐标、y坐标、航向角、线速度、角速度
    current_state = np.array([0.0, 0.0, math.pi / 8.0, 0.0, 0.0])
    # 目标点 坐标 单位 m
    target = np.array([target_x, target_y])
    # input [forward speed, yaw_rate]
    config.robot_type = robot_type
    # 初始化 轨迹 存在当前状态
    trajectory = np.array(current_state)
    # 初始化 障碍物 坐标
    obstacle = config.obstacle
    while True:
        # 从当前状态 在动态窗口 一个 预测时间 更新后的 新运动速度、预测轨迹。这里返回的已经最优预测轨迹，所以此时的twist表现最优
        twist, predicted_trajectory = dwa_control(current_state, config, target, obstacle)
        # 从 当前状态 走 dt 时间的 状态
        current_state = motion(current_state, twist, config.dt)  # simulate robot
        # 累计轨迹
        trajectory = np.vstack((trajectory, current_state))  # store state history

        if show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            plt.plot(current_state[0], current_state[1], "xr")
            plt.plot(target[0], target[1], "xb")
            plt.plot(obstacle[:, 0], obstacle[:, 1], "ok")
            plot_robot(current_state[0], current_state[1], current_state[2], config)
            plot_arrow(current_state[0], current_state[1], current_state[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)
        # check reaching target
        dist_to_goal = math.hypot(current_state[0] - target[0], current_state[1] - target[1])
        if dist_to_goal <= config.robot_radius:
            print("Goal!!")
            break
    print("Done")
    if show_animation:
        plt.plot(trajectory[:, 0], trajectory[:, 1], "-r")
        plt.pause(0.0001)
        plt.show()


if __name__ == '__main__':
    main(robot_type=RobotType.rectangle)
    # main(robot_type=RobotType.circle)
