# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from __future__ import annotations

from typing import TYPE_CHECKING, Literal

import torch

from isaaclab.envs.mdp.events import _randomize_prop_by_op
from isaaclab.managers import EventTermCfg, ManagerTermBase, SceneEntityCfg

if TYPE_CHECKING:
    from isaaclab.assets import Articulation
    from isaaclab.envs import ManagerBasedEnv


#  class randomize_joint_offsets(ManagerTermBase):
    # TODO
class HorizontalForcePulse(ManagerTermBase):
    """Apply a random horizontal force pulse to the robot base."""

    def __init__(self, cfg: EventTermCfg, env: ManagerBasedEnv):
        super().__init__(cfg, env)

        self._asset_cfg: SceneEntityCfg = cfg.params["asset_cfg"]
        self._asset: Articulation = env.scene[self._asset_cfg.name]

        if not isinstance(self._asset_cfg.body_ids, list):
            raise ValueError("HorizontalForcePulse requires explicitly resolved body_ids.")

        self._num_bodies = len(self._asset_cfg.body_ids)
        self._pulse_time_left = torch.zeros(env.num_envs, device=env.device)
        self._wait_time_left = torch.zeros(env.num_envs, device=env.device)

        # Needed before the first reset call.
        self._sample_wait(torch.arange(env.num_envs, device=env.device), cfg.params["wait_range_s"])

    def reset(self, env_ids=None):
        if env_ids is None:
            env_ids = torch.arange(self.num_envs, device=self.device)
        elif not isinstance(env_ids, torch.Tensor):
            env_ids = torch.tensor(env_ids, device=self.device, dtype=torch.long)
        else:
            env_ids = env_ids.to(device=self.device, dtype=torch.long)

        self._pulse_time_left[env_ids] = 0.0
        self._sample_wait(env_ids, self.cfg.params["wait_range_s"])
        self._clear_force(env_ids, self.cfg.params["is_global"])

    def __call__(
        self,
        env: ManagerBasedEnv,
        env_ids: torch.Tensor,
        asset_cfg: SceneEntityCfg,
        force_magnitude_range: tuple[float, float],
        pulse_duration_s: float,
        wait_range_s: tuple[float, float],
        is_global: bool = False,
    ):
        """Ticked once per control step; starts/ends force pulses independently per environment."""
        del asset_cfg  # The resolved config is cached in __init__.

        env_ids = env_ids.to(device=self.device, dtype=torch.long)
        dt = env.step_dt

        # Continue active pulses. Clear force immediately after the requested duration.
        active_mask = self._pulse_time_left[env_ids] > 0.0
        active_ids = env_ids[active_mask]
        if active_ids.numel() > 0:
            self._pulse_time_left[active_ids] -= dt
            ended_ids = active_ids[self._pulse_time_left[active_ids] <= 0.0]
            if ended_ids.numel() > 0:
                self._clear_force(ended_ids, is_global)
                self._sample_wait(ended_ids, wait_range_s)

        # Count down environments that are waiting for their next push.
        waiting_ids = env_ids[~active_mask]
        if waiting_ids.numel() > 0:
            self._wait_time_left[waiting_ids] -= dt
            start_ids = waiting_ids[self._wait_time_left[waiting_ids] <= 0.0]
            if start_ids.numel() > 0:
                self._apply_random_horizontal_force(start_ids, force_magnitude_range, is_global)
                self._pulse_time_left[start_ids] = pulse_duration_s

    def _sample_wait(self, env_ids: torch.Tensor, wait_range_s: tuple[float, float]):
        low, high = wait_range_s
        self._wait_time_left[env_ids] = torch.empty(len(env_ids), device=self.device).uniform_(low, high)

    def _apply_random_horizontal_force(
        self,
        env_ids: torch.Tensor,
        force_magnitude_range: tuple[float, float],
        is_global: bool,
    ):
        magnitude = torch.empty(len(env_ids), 1, device=self.device).uniform_(*force_magnitude_range)
        heading = torch.empty(len(env_ids), 1, device=self.device).uniform_(-torch.pi, torch.pi)

        forces = torch.zeros(len(env_ids), self._num_bodies, 3, device=self.device)
        forces[:, :, 0] = magnitude * torch.cos(heading)
        forces[:, :, 1] = magnitude * torch.sin(heading)
        # forces[:, :, 2] remains exactly zero

        torques = torch.zeros_like(forces)
        self._asset.permanent_wrench_composer.set_forces_and_torques(
            forces=forces,
            torques=torques,
            body_ids=self._asset_cfg.body_ids,
            env_ids=env_ids,
            is_global=is_global,
        )

    def _clear_force(self, env_ids: torch.Tensor, is_global: bool):
        zeros = torch.zeros(len(env_ids), self._num_bodies, 3, device=self.device)
        self._asset.permanent_wrench_composer.set_forces_and_torques(
            forces=zeros,
            torques=zeros,
            body_ids=self._asset_cfg.body_ids,
            env_ids=env_ids,
            is_global=is_global,
        )