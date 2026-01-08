
import unittest
import numpy as np
import time
from unittest.mock import MagicMock, patch

from universal_controller.core.data_types import Trajectory, Header, Point3D, ControlOutput, ConsistencyResult, EstimatorOutput, Imu
from universal_controller.tracker.mpc_controller import MPCController
from universal_controller.estimator.adaptive_ekf import AdaptiveEKFEstimator
from universal_controller.transform.robust_transformer import RobustCoordinateTransformer
from universal_controller.core.enums import TransformStatus, ControllerState
from universal_controller.config.default_config import DEFAULT_CONFIG, PLATFORM_CONFIG

class TestNewCriticalFixes(unittest.TestCase):

    def setUp(self):
        self.config = DEFAULT_CONFIG.copy()
        self.platform_config = PLATFORM_CONFIG['differential']

    def test_mpc_resampling_logic(self):
        """Test that MPC correctly resamples trajectory when dt mismatches.
        
        This test directly tests the MPCReferenceGenerator to avoid ACADOS dependency.
        """
        from universal_controller.tracker.mpc_core.reference_generator import MPCReferenceGenerator
        
        # Create reference generator with MPC dt=0.1
        ref_gen = MPCReferenceGenerator(horizon=10, dt=0.1)
        
        # Create a trajectory with dt=0.2 (2x slower)
        # Points: x moves 2m every step => v=10m/s
        traj_dt = 0.2
        # Create 10 points
        points = [Point3D(x=i*2.0, y=0.0, z=0.0) for i in range(10)]
        # Velocities: Constant 10m/s x
        velocities = np.zeros((10, 6))
        velocities[:, 0] = 10.0 
        
        trajectory = Trajectory(
            header=Header(stamp=0.0, frame_id='odom'),
            points=points,
            velocities=velocities,
            dt_sec=traj_dt
        )
        
        # Generate references
        state = np.zeros(8)
        consistency = ConsistencyResult(
            alpha=1.0, kappa_consistency=1.0, v_dir_consistency=1.0, 
            temporal_smooth=1.0, should_disable_soft=False, data_valid=True
        )
        
        yrefs, ps, yref_e = ref_gen.generate_references(state, trajectory, consistency)
        
        # Verify resampling:
        # Input Trajectory: t=0, x=0; t=0.2, x=2; t=0.4, x=4 ... (v=10)
        # MPC wants: t=0, 0.1, 0.2, 0.3, 0.4, 0.5 ...
        # Expected Resampled Points at MPC grid:
        # i=0 (t=0): x=0
        # i=1 (t=0.1): x=1 (Interpolated between 0 and 2)
        # i=2 (t=0.2): x=2
        # i=3 (t=0.3): x=3 (Interpolated)
        # i=4 (t=0.4): x=4
        
        # Check index 1 (t=0.1s). 
        # yrefs structure: [x, y, z, vx, vy, vz, theta, omega, ...]
        # Based on MPCSlices: POS=0:3, VEL=3:6
        
        x_pos_at_t1 = yrefs[1, 0]  # x position at step 1
        vx_at_t1 = yrefs[1, 3]     # vx at step 1
        
        print(f"MPC Resampling Test: Step 1 x_pos = {x_pos_at_t1}")
        
        # Allow slight floating point error
        self.assertAlmostEqual(x_pos_at_t1, 1.0, delta=0.1, 
            msg=f"Resampling failed! Got x={x_pos_at_t1} instead of ~1.0 for t=0.1s")
        self.assertAlmostEqual(vx_at_t1, 10.0, delta=0.1,
            msg=f"Velocity resampling failed! Got vx={vx_at_t1} instead of ~10.0")

    def test_ekf_stale_data_logic(self):
        """Test that EKF ignores stale Odom data for stationarity checks."""
        ekf = AdaptiveEKFEstimator(self.config, self.platform_config)
        
        # Hard init
        class MockOdom:
            pose_position = Point3D(0,0,0)
            pose_orientation = [0,0,0,1]
            twist_linear = [0,0,0]
            twist_angular = [0,0,0]
        
        ekf.update_odom(MockOdom(), current_time=0.0)
        self.assertTrue(ekf._initialized)
        self.assertEqual(ekf._time_since_last_odom, 0.0)
        
        # 1. Fresh Data Test
        # Set raw odom to be stationary (0 velocity)
        ekf._raw_odom_twist_norm = 0.0
        ekf._raw_odom_angular_velocity = 0.0
        # Set Gyro to be DRIFTING (rotating)
        ekf.gyro_z = 0.5 # > default threshold (0.02)
        
        # Predict a small step (fresh)
        ekf.predict(dt=0.1) # _time_since_last_odom becomes 0.1
        
        anomalies = ekf._detect_anomalies_unlocked()
        
        # Should detect IMU_DRIFT because Odom says Stop, IMU says Move, and Data is Fresh
        self.assertIn("IMU_DRIFT", anomalies, "Fresh data should detect drift")
        
        # 2. Stale Data Test
        # Clear anomalies
        # Predict a LARGE step (simulating timeout)
        ekf.predict(dt=5.0) # _time_since_last_odom becomes 5.1
        
        # Internal state _raw_odom_twist_norm is STILL 0.0 (Stationary)
        # Gyro is STILL 0.5 (Drifting)
        
        anomalies_stale = ekf._detect_anomalies_unlocked()
        
        # Should NOT detect IMU_DRIFT because Odom is stale, so we don't trust "Stationary"
        self.assertNotIn("IMU_DRIFT", anomalies_stale, "Stale data should NOT detect drift")

    def test_3d_fallback_transform(self):
        """Test that RobustCoordinateTransformer recovers 3D rotation from EstimatorOutput."""
        transformer = RobustCoordinateTransformer(self.config)
        transformer.is_3d = True # Assume 3D platform settings if relevant, though logic is generic
        
        # Create a trajectory along X axis in Body Frame
        # Point (1, 0, 0)
        traj = Trajectory(
            header=Header(0, 'base_link'),
            points=[Point3D(1.0, 0.0, 0.0)],
            velocities=None,
            dt_sec=0.1
        )
        
        # Create Fallback State
        # Position: 0,0,0
        # Orientation: Pitch down 90 degrees (Face down)
        # Quaternion for Pitch=90 deg (around Y) -> w=cos(45), y=sin(45) -> 0.707, 0.707
        # Rotate point (1,0,0) by Pitch 90 -> (0, 0, -1) in World? 
        # Wait, Pitch +90 around Y (Right Hand Rule)? X -> -Z.
        q = np.array([0.0, 0.70710678, 0.0, 0.70710678]) 
        
        fallback_state = EstimatorOutput(
            state=np.zeros(8),
            covariance=np.eye(8),
            covariance_norm=0.0,
            innovation_norm=0.0,
            imu_bias=np.zeros(3),
            slip_probability=0.0,
            anomalies=[],
            orientation_quat=q # NEW FIELD
        )
        
        # Trigger Fallback
        res_traj, status = transformer._handle_fallback(traj, 0.0, 'odom', fallback_state)
        
        p = res_traj.points[0]
        
        print(f"3D Fallback Result: x={p[0]:.3f}, y={p[1]:.3f}, z={p[2]:.3f}")
        
        # Expect X to be near 0, Z to be near -1 or 1 depending on rotation direction
        # Q(0, 0.707, 0, 0.707) is +90 deg around Y.
        # X vector (1,0,0) rotated +90 around Y -> Z vector (-1)? 
        # Check standard rotation: R = [ [c, 0, s], [0, 1, 0], [-s, 0, c] ]
        # x_new = c*x + s*z = 0*1 + 1*0 = 0
        # z_new = -s*x + c*z = -1*1 = -1.
        
        self.assertAlmostEqual(p[0], 0.0, delta=0.1)
        self.assertAlmostEqual(p[2], -1.0, delta=0.1)
        
        # Verify OLD 2D Fallback would fail this
        # If we removed orientation_quat, it would use Yaw=0 (from state zeros)
        # Result would be (1, 0, 0)
        
        fallback_state_no_3d = EstimatorOutput(
            state=np.zeros(8),
            covariance=np.eye(8),
            covariance_norm=0.0,
            innovation_norm=0.0,
            imu_bias=np.zeros(3),
            slip_probability=0.0,
            anomalies=[],
            orientation_quat=None # Simulate old data
        )
        
        res_traj_2d, _ = transformer._handle_fallback(traj, 0.0, 'odom', fallback_state_no_3d)
        p2d = res_traj_2d.points[0]
        # Should be flat
        self.assertAlmostEqual(p2d[0], 1.0, delta=0.1)
        self.assertAlmostEqual(p2d[2], 0.0, delta=0.1)

if __name__ == '__main__':
    unittest.main()
