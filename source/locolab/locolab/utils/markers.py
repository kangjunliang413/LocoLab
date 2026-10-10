# Copyright (c) 2022-2026, The Isaac Lab Project Developers.
# All rights reserved.
# Original code is licensed under BSD-3-Clause.
#
# Copyright (c) 2025-2026, The Loco Lab Project Developers.
# All rights reserved.
# Modifications are licensed under BSD-3-Clause.

from pathlib import Path

import isaaclab.sim as sim_utils
from isaaclab.markers.visualization_markers import VisualizationMarkersCfg

ARROW_X_USD_PATH = str(
    Path(__file__).resolve().parents[1] / "assets" / "arrow_x.usd"
)

RED_ARROW_X_MARKER_CFG = VisualizationMarkersCfg(
    markers={
        "arrow": sim_utils.UsdFileCfg(
            usd_path=ARROW_X_USD_PATH,
            scale=(1.0, 0.1, 0.1),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(1.0, 0.0, 0.0)),
        )
    }
)
"""Configuration for the red arrow marker (along x-direction)."""


BLUE_ARROW_X_MARKER_CFG = VisualizationMarkersCfg(
    markers={
        "arrow": sim_utils.UsdFileCfg(
            usd_path=ARROW_X_USD_PATH,
            scale=(1.0, 0.1, 0.1),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.2, 0.9)),
        )
    }
)
"""Configuration for the blue arrow marker (along x-direction)."""

GREEN_ARROW_X_MARKER_CFG = VisualizationMarkersCfg(
    markers={
        "arrow": sim_utils.UsdFileCfg(
            usd_path=ARROW_X_USD_PATH,
            scale=(1.0, 0.1, 0.1),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.9, 0.2)),
        )
    }
)
"""Configuration for the green arrow marker (along x-direction)."""


RED_ARROW_X_MARKER_CFG = VisualizationMarkersCfg(
    markers={
        "arrow": sim_utils.UsdFileCfg(
            usd_path=ARROW_X_USD_PATH,
            scale=(1.0, 0.1, 0.1),
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.2, 0.2)),
        )
    }
)
"""Configuration for the red arrow marker (along x-direction)."""


BLUE_RAY_CASTER_MARKER_CFG = VisualizationMarkersCfg(
    prim_path="/Visuals/RayCaster",
    markers={
        "hit": sim_utils.SphereCfg(
            radius=0.02,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.2, 0.2, 0.9)),
        ),
    },
)
"""Configuration for the blue ray-caster marker."""

PURPLE_RAY_CASTER_MARKER_CFG = VisualizationMarkersCfg(
    prim_path="/Visuals/RayCaster",
    markers={
        "hit": sim_utils.SphereCfg(
            radius=0.02,
            visual_material=sim_utils.PreviewSurfaceCfg(diffuse_color=(0.9, 0.2, 0.5)),
        ),
    },
)
"""Configuration for the purple ray-caster marker."""

