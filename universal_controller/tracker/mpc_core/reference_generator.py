import numpy as np
import logging
from typing import Tuple, Optional

from ...core.data_types import Trajectory, ConsistencyResult
from ...core.indices import MPCStateIdx, MPCSlices
from ...core.ros_compat import normalize_angle
from ...core.constants import EPSILON

# =============================================================================
# Velocity Channel Mapping Constants
# =============================================================================
# For 6D velocity [vx, vy, vz, wx, wy, wz], we select [vx, vy, vz, wz]
VEL_INDICES_6D = [0, 1, 2, 5]
# For 4D velocity [vx, vy, vz, wz], we select all
VEL_INDICES_4D = [0, 1, 2, 3]

# Minimum channel count to use 6D mapping (self-documenting constant)
MIN_CHANNELS_FOR_6D = len(VEL_INDICES_6D) + 2  # 6 channels: vx, vy, vz, wx, wy, wz

logger = logging.getLogger(__name__)

class MPCReferenceGenerator:
    """Handles trajectory resampling, heating calculation, and reference buffer construction for MPC."""

    def __init__(self, horizon: int, dt: float):
        self.horizon = horizon
        self.dt = dt
        self._yref_buffer: Optional[np.ndarray] = None

    def resize_buffer_if_needed(self, horizon: int):
        if self._yref_buffer is None or self._yref_buffer.shape[0] != horizon:
            self._yref_buffer = np.zeros((horizon, 12), dtype=np.float64)
            self.horizon = horizon

    def generate_references(self, 
                          state: np.ndarray, 
                          trajectory: Trajectory, 
                          consistency: ConsistencyResult) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Generates yref (reference matrix) and p (parameters) for ACADOS solver.
        
        Returns:
            yrefs: [H, 12] matrix
            ps: [H, 7] matrix (view of yrefs)
            yref_e: [8] terminal reference
        """
        self.resize_buffer_if_needed(self.horizon)
        
        # Unpack consistency/trajectory info
        alpha = consistency.alpha
        traj_dt = trajectory.dt_sec
        mpc_dt = self.dt
        
        # Resampling Logic
        needs_resampling = abs(traj_dt - mpc_dt) > 1e-4 and traj_dt > 0
        src_len = len(trajectory.points)

        target_times = np.arange(self.horizon, dtype=np.float64) * mpc_dt
        
        if needs_resampling:
            # High performance resampling
            if src_len < 2:
                # Fallback
                blended_vels = np.zeros((self.horizon, 4))
                points_slice = np.zeros((self.horizon, 3))
            else:
                # Interpolation
                src_indices = np.arange(src_len, dtype=np.float64)
                target_indices = target_times / traj_dt
                
                # Resample velocities
                src_vels = trajectory.get_blended_velocities_slice(0, src_len, alpha)
                src_channels = src_vels.shape[1]
                
                # Determine indices for [vx, vy, vz, omega]
                # If 6D: [vx, vy, vz, wx, wy, wz] -> use [0, 1, 2, 5]
                # If 4D: [vx, vy, vz, wz]         -> use [0, 1, 2, 3]
                if src_channels >= MIN_CHANNELS_FOR_6D:
                     vel_indices = VEL_INDICES_6D
                else:
                     vel_indices = VEL_INDICES_4D

                blended_vels = np.zeros((self.horizon, 4))
                for k, src_idx in enumerate(vel_indices):
                    # Robust check for index existence (e.g. if src is 3D)
                    if src_idx < src_channels:
                         blended_vels[:, k] = np.interp(target_indices, src_indices, src_vels[:, src_idx])

                # Resample points
                src_points = trajectory.get_points_matrix()
                points_slice = np.zeros((self.horizon, 3))
                for k in range(3):
                    points_slice[:, k] = np.interp(target_indices, src_indices, src_points[:, k])
        else:
            # Fast slice
            raw_vels = trajectory.get_blended_velocities_slice(0, self.horizon, alpha)
            # Map raw channels to MPC standard [vx, vy, vz, omega]
            if raw_vels.shape[1] >= MIN_CHANNELS_FOR_6D:
                # Select [0,1,2] and [5] (wz)
                blended_vels = np.hstack([raw_vels[:, 0:3], raw_vels[:, 5:6]])
            else:
                # Standard 4D: take first 4 (or pad if only 3 provided)
                if raw_vels.shape[1] >= len(VEL_INDICES_4D):
                     blended_vels = raw_vels[:, :4]
                else:
                     # Fallback for 3D velocity (no omega)
                     blended_vels = np.zeros((len(raw_vels), 4))
                     blended_vels[:, :raw_vels.shape[1]] = raw_vels

            points_all = trajectory.get_points_matrix()
            points_slice = points_all[:self.horizon]

            # Padding Logic (Common for both vels and points)
            # Pad Velocities
            v_len = len(blended_vels)
            if v_len < self.horizon:
                pad_len = self.horizon - v_len
                if v_len > 0:
                    last_vel = blended_vels[-1]
                    padding = np.tile(last_vel, (pad_len, 1))
                    blended_vels = np.vstack([blended_vels, padding])
                else:
                    blended_vels = np.zeros((self.horizon, 4))
            
            # Pad Points
            p_len = len(points_slice)
            if p_len < self.horizon:
                pad_len = self.horizon - p_len
                if p_len > 0:
                    last_pt = points_slice[-1]
                    pad_pts = np.tile(last_pt, (pad_len, 1))
                    points_slice = np.vstack([points_slice, pad_pts])
                else:
                    points_slice = np.zeros((self.horizon, 3))

        # Heading Calculation (Geometric Consistency)
        diffs = points_slice[1:] - points_slice[:-1]
        last_diff = np.zeros(3)
        if len(diffs) > 0:
            last_diff = diffs[-1]
        diffs = np.vstack([diffs, last_diff]) # [H, 3]

        vel_refs = blended_vels[:, 0:2]
        motion_dot_prod = diffs[:, 0] * vel_refs[:, 0] + diffs[:, 1] * vel_refs[:, 1]
        geom_thetas = np.arctan2(diffs[:, 1], diffs[:, 0])
        
        # Reversing detection
        reversing_mask = motion_dot_prod < -EPSILON
        if np.any(reversing_mask):
            geom_thetas[reversing_mask] = normalize_angle(geom_thetas[reversing_mask] + np.pi)

        dist_sq = diffs[:, 0]**2 + diffs[:, 1]**2
        valid_mask = dist_sq > EPSILON

        theta_refs = np.zeros(self.horizon)
        # Use current state heading for continuity at step 0
        theta_refs[0] = state[MPCStateIdx.THETA]
        
        last_valid_theta = theta_refs[0]
        for i in range(1, self.horizon):
            if valid_mask[i-1]:
                last_valid_theta = geom_thetas[i-1]
            theta_refs[i] = last_valid_theta
            
        # Unwrapping / Cumulative Theta for solver continuity
        for i in range(1, self.horizon):
            delta = normalize_angle(theta_refs[i] - theta_refs[i-1])
            theta_refs[i] = theta_refs[i-1] + delta

        # Construct yref buffer
        yrefs = self._yref_buffer
        yrefs[:, MPCSlices.REF_POS] = points_slice
        yrefs[:, MPCSlices.REF_VEL] = blended_vels[:, 0:3]
        yrefs[:, MPCSlices.REF_THETA] = theta_refs
        yrefs[:, MPCSlices.REF_OMEGA] = blended_vels[:, 3]
        yrefs[:, MPCSlices.REF_CONTROLS] = 0.0

        # Ensure Contiguous
        if not yrefs.flags['C_CONTIGUOUS']:
            yrefs = np.ascontiguousarray(yrefs, dtype=np.float64)
            self._yref_buffer = yrefs # Update ref

        ps = yrefs[:, MPCSlices.POSE_VEL]
        
        # Terminal Cost Reference
        final_pt = points_slice[-1]
        final_theta = theta_refs[-1]
        
        yref_e = np.array([
            final_pt[0], final_pt[1], final_pt[2],
            0.0, 0.0, 0.0, final_theta, 0.0
        ])
        
        return yrefs, ps, yref_e
