
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
        """Test that MPC correctly resamples trajectory when dt mismatches."""
        # Mock MPCController to avoid loading actual ACADOS solver
        with patch('universal_controller.tracker.mpc_controller.AcadosOcpSolver') as MockSolver, \
             patch('universal_controller.tracker.mpc_controller.AcadosOcp') as MockOcp, \
             patch('universal_controller.tracker.mpc_controller.AcadosModel') as MockModel, \
             patch('universal_controller.tracker.mpc_controller.ca') as MockCa, \
             patch('universal_controller.tracker.mpc_controller.ACADOS_AVAILABLE', True):
            
            controller = MPCController(self.config, self.platform_config)
            # Force self.dt to 0.1
            controller.dt = 0.1
            controller.horizon = 10 
            # Create Mock Solver instance
            controller._get_solver = MagicMock()
            mock_solver_instance = MagicMock()
            controller._get_solver.return_value = mock_solver_instance
            mock_solver_instance.solve.return_value = 0 # Success status
            mock_solver_instance.get.return_value = np.zeros(8) # Control output - State vector size 8
            
            # DIRECTLY inject the mock solver so _solve_with_acados uses it
            controller._solver = mock_solver_instance
            controller._is_initialized = True

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

            # We need to inspect what is passed to the solver
            # We can mock the solver's 'set' method
            # solver.set(i, "yref", ...)
            
            # Mock _update_constraints to avoid issues
            controller._update_constraints = MagicMock()

            # Call compute
            state = np.zeros(8)
            consistency = ConsistencyResult(alpha=1.0, kappa_consistency=1.0, v_dir_consistency=1.0, temporal_smooth=1.0, should_disable_soft=False, data_valid=True)
            
            # We need to spy on the internal logic. Ideally, we unit test the resampling, 
            # but it's embedded in _solve_with_acados.
            # Let's override _solve_with_acados to just run the prep part? No, that's messy.
            # Let's inspect the mock_solver calls.
            
            controller.compute(state, trajectory, consistency)
            
            # Verify calls to solver.set(i, "yref", ...)
            # Input Trajectory: t=0, x=0; t=0.2, x=2; t=0.4, x=4 ... (v=10)
            # MPC wants: t=0, 0.1, 0.2, 0.3 ...
            # Expected Resampled Points/Vels at MPC grid:
            # i=0 (t=0): x=0, v=10
            # i=1 (t=0.1): x=1, v=10 (Interpolated!)
            # i=2 (t=0.2): x=2, v=10
            
            # Check call args
            # We look for "yref" setting
            yref_calls = []
            for call in mock_solver_instance.set.call_args_list:
                args = call.args
                if args[1] == "yref":
                    # args[0] is index, args[2] is value
                    yref_calls.append((args[0], args[2]))
            
            yref_calls.sort(key=lambda x: x[0])
            
            self.assertGreater(len(yref_calls), 1, "Mock Solver 'set' was not called enough times (expected >1 for horizon>1)")
            
            # Check index 1 (t=0.1s). 
            # IMPORTANT: yref structure depends on implementation. 
            # Usually [x, y, z, vx, vy, vz, theta, omega, ax, ay, az, alpha] 
            # or similar cost reference.
            # Based on MPCSlices: POS=0:3, VEL=3:6
            
            if len(yref_calls) > 1:
                idx, val = yref_calls[1] # Step 1
                # Check X position
                x_pos = val[0]
                vx = val[3]
                
                # Original code (Buggy) would take index 1 directly -> Traj point 1 -> x=2.0
                # Correct code (Fixed) should interpolate t=0.1 between t=0(x=0) and t=0.2(x=2) -> x=1.0
                
                print(f"MPC Resampling Test: Step 1 x_pos = {x_pos}")
                # Allow slight floating point error
                self.assertAlmostEqual(x_pos, 1.0, delta=0.01, 
                    msg="Resampling failed! Got x={} instead of 1.0 for t=0.1s (Linear Interp of 0 and 2)".format(x_pos))
                self.assertAlmostEqual(vx, 10.0, delta=0.01)

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
