# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import SceneEntityCfg
from isaaclab.utils import configclass

import locolab.tasks.manager_based.locomotion.velocity.mdp as mdp

from . import BASE_LINK_NAME, FOOT_LINK_NAMES, JOINT_NAMES, OTHER_BODY_LINK_NAMES


@configclass
class EventCfg:
    """Configuration for events."""

    # ===== startup ===== (6 events)
    # 1.
    randomize_feet_material = EventTerm(
        func=mdp.randomize_rigid_body_material,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=FOOT_LINK_NAMES),
            "static_friction_range": (0.2, 1.5),
            "dynamic_friction_range": (0.2, 1.5),
            "restitution_range": (0.0, 0.5),
            "num_buckets": 64,
            "make_consistent": (
                True
            ),  # Ensure dynamic friction is less than or equal to static friction. This obeys the physics constraint on friction values.
        },
    )
    # 2.
    randomize_base_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=BASE_LINK_NAME),
            "mass_distribution_params": (-1.0, 3.0),
            "operation": "add",
            "recompute_inertia": True,
        },
    )
    # 3.
    randomize_other_bodies_mass = EventTerm(
        func=mdp.randomize_rigid_body_mass,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=OTHER_BODY_LINK_NAMES),
            "mass_distribution_params": (0.9, 1.1),
            "operation": "scale",
            "recompute_inertia": True,
        },
    )
    # 4.
    randomize_base_com = EventTerm(
        func=mdp.randomize_rigid_body_com,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=BASE_LINK_NAME),
            "com_range": {"x": (-0.05, 0.05), "y": (-0.05, 0.05), "z": (-0.05, 0.05)},
        },
    )
    # 5.
    randomize_actuator_gains = EventTerm(
        func=mdp.randomize_actuator_gains,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES),
            "stiffness_distribution_params": (0.8, 1.2),
            "damping_distribution_params": (0.8, 1.2),
            "operation": "scale",
            "distribution": "uniform",
        },
    )
    # 6.
    # this term has bug!!!
    #  randomize_joint_offsets = EventTerm(
    #      func=mdp.randomize_joint_offsets,
    #      mode="startup",
    #      params={
    #          "asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES),
    #          "offsets_distribution_params": (-0.02, 0.02),
    #          "operation": "add",
    #          "distribution": "gaussian",
    #      },
    #  )
    # 7.
    randomize_joint_parameters = EventTerm(
        func=mdp.randomize_joint_parameters,
        mode="startup",
        params={
            "asset_cfg": SceneEntityCfg("robot", joint_names=JOINT_NAMES),
            "friction_distribution_params": (0.0, 0.1),
            "armature_distribution_params": (0.0, 0.01),
            "operation": "add",
            "distribution": "uniform",
        },
    )

    # ===== reset ===== (2 events)
    # 1.
    reset_joints = EventTerm(
        func=mdp.reset_joints_by_scale,
        mode="reset",
        params={
            "position_range": (0.75, 1.25),
            "velocity_range": (0.0, 0.0),
        },
    )
    # 2.
    reset_base = EventTerm(
        func=mdp.reset_root_state_uniform,
        mode="reset",
        params={
            "pose_range": {
                "x": (-0.5, 0.5),
                "y": (-0.5, 0.5),
                "z": (0.0, 0.2),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (-3.14, 3.14),
            },
            "velocity_range": {
                "x": (0.0, 0.0),
                "y": (0.0, 0.0),
                "z": (0.0, 0.0),
                "roll": (0.0, 0.0),
                "pitch": (0.0, 0.0),
                "yaw": (0.0, 0.0),
            },
        },
    )

    # ===== interval ===== (1 events)
    # 1.
    horizontal_force_pulse = EventTerm(
        func=mdp.HorizontalForcePulse,
        mode="interval",
        # Go2FlatEnvCfg: dt=0.005, decimation=4，因此控制周期是 0.02 s。
        # 该 term 每个控制步运行一次，用于计时和清零。
        interval_range_s=(0.02, 0.02),
        params={
            "asset_cfg": SceneEntityCfg("robot", body_names=BASE_LINK_NAME),
            # 水平随机方向，大小范围单位为 N；建议从较小值开始。
            "force_magnitude_range": (30.0, 60.0),
            "pulse_duration_s": 1.0,
            # 每次推力结束后，等待 5--10 秒再施加下一次。
            "wait_range_s": (5,10 ),
            # False: x/y 相对机器人 base 坐标系；True: 相对世界坐标系。
            "is_global": False,
        },
    )