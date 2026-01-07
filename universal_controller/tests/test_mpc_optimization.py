"""
MPC Optimization Tests

Verifies:
1. Solver caching (pre-warming and reuse)
2. Predicted trajectory output
"""
import unittest
from unittest.mock import MagicMock, patch
import numpy as np
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from universal_controller.tracker.mpc_controller import MPCController
from universal_controller.config.default_config import DEFAULT_CONFIG, PLATFORM_CONFIG
from universal_controller.core.data_types import Trajectory, ConsistencyResult
from universal_controller.tests.fixtures import create_test_trajectory

class TestMPCOptimization(unittest.TestCase):
    
    def setUp(self):
        self.config = DEFAULT_CONFIG.copy()
        self.platform_config = PLATFORM_CONFIG['differential']
        
        # Mock ACADOS classes to simulate availability and tracking creation
        self.acados_patcher = patch.dict('sys.modules', {
            'acados_template': MagicMock(),
            'casadi': MagicMock()
        })
        self.acados_patcher.start()
        
        # We need to force reload MPCController to pick up the mocked modules if it was already imported
        # But for simplicity, we mock the internal availability flag and classes in the test
        
    def tearDown(self):
        self.acados_patcher.stop()

    @patch('universal_controller.tracker.mpc_controller.ACADOS_AVAILABLE', True)
    @patch('universal_controller.tracker.mpc_controller.AcadosOcpSolver')
    @patch('universal_controller.tracker.mpc_controller.AcadosOcp')
    @patch('universal_controller.tracker.mpc_controller.AcadosModel')
    @patch('universal_controller.tracker.mpc_controller.ca')
    def test_solver_caching(self, mock_ca, mock_model, mock_ocp, mock_solver_cls):
        """Test that solvers are cached and reused"""
        
        # Setup mock solver to return a mock object
        mock_solver_instance_10 = MagicMock()
        mock_solver_instance_20 = MagicMock()
        
        # Use side_effect to return different mocks based on calls or just unique mocks per creation
        mock_solver_cls.side_effect = [mock_solver_instance_20, mock_solver_instance_10, MagicMock()]
        
        # Configure pre-warming
        self.config['mpc']['prewarm_horizons'] = [10, 20]
        self.config['mpc']['horizon'] = 20
        
        # Initialize controller
        mpc = MPCController(self.config, self.platform_config)
        
        # Verify initialization created solvers for 20 (current) and 10 (pre-warm)
        # Note: 20 is created first as current horizon
        self.assertTrue(20 in mpc._solver_cache)
        self.assertTrue(10 in mpc._solver_cache)
        self.assertIs(mpc._solver_cache[20], mock_solver_instance_20)
        self.assertIs(mpc._solver_cache[10], mock_solver_instance_10)
        
        # Record usage
        mock_solver_cls.reset_mock()
        
        # Switch to 10 - should reuse cache
        mpc.set_horizon(10)
        self.assertEqual(mpc.horizon, 10)
        self.assertIs(mpc._solver, mock_solver_instance_10)
        mock_solver_cls.assert_not_called()  # Should NOT create new solver
        
        # Switch back to 20 - should reuse cache
        mpc.set_horizon(20)
        self.assertEqual(mpc.horizon, 20)
        self.assertIs(mpc._solver, mock_solver_instance_20)
        mock_solver_cls.assert_not_called()
        
        mpc.shutdown()

    @patch('universal_controller.tracker.mpc_controller.ACADOS_AVAILABLE', True)
    @patch('universal_controller.tracker.mpc_controller.AcadosOcpSolver')
    @patch('universal_controller.tracker.mpc_controller.AcadosOcp')
    @patch('universal_controller.tracker.mpc_controller.AcadosModel')
    @patch('universal_controller.tracker.mpc_controller.ca')
    def test_predicted_trajectory_output(self, mock_ca, mock_model, mock_ocp, mock_solver_cls):
        """Test that predicted trajectory is populated in output extras when visualize_prediction is enabled"""
        import logging
        
        # Enable DEBUG logging for the mpc_controller module
        mpc_logger = logging.getLogger('universal_controller.tracker.mpc_controller')
        original_level = mpc_logger.level
        mpc_logger.setLevel(logging.DEBUG)
        
        try:
            # Setup mock solver
            mock_solver = MagicMock()
            mock_solver_cls.return_value = mock_solver
            mock_solver.solve.return_value = 0 # Success
            
            # Mock .get() to return state based on index
            # Return dummy state [px, py, pz, vx, vy, vz, theta, omega]
            def get_side_effect(idx, key):
                if key == "x":
                    return np.array([float(idx), 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
                return None
            mock_solver.get.side_effect = get_side_effect
            
            # Enable visualize_prediction to trigger predicted_trajectory extraction
            config_with_viz = self.config.copy()
            config_with_viz['mpc'] = config_with_viz.get('mpc', {}).copy()
            config_with_viz['mpc']['visualize_prediction'] = True
            
            mpc = MPCController(config_with_viz, self.platform_config)
            mpc.set_horizon(20)
            
            state = np.array([0]*8)
            trajectory = create_test_trajectory(soft_enabled=True)
            consistency = ConsistencyResult(
                alpha=1.0, kappa_consistency=1.0, v_dir_consistency=1.0,
                temporal_smooth=1.0, should_disable_soft=False, data_valid=True
            )
            
            # Compute
            cmd = mpc.compute(state, trajectory, consistency)
            
            # Verify extras
            self.assertTrue(cmd.success)
            self.assertIn('predicted_trajectory', cmd.extras)
            pred_traj = cmd.extras['predicted_trajectory']
            
            # Should have horizon + 1 points (0 to N)
            self.assertEqual(len(pred_traj), 21)
            # Check first point (x=0) and last point (x=20) logic from our mock
            self.assertEqual(pred_traj[0][0], 0.0)
            self.assertEqual(pred_traj[20][0], 20.0)
            
            mpc.shutdown()
        finally:
            # Restore original log level
            mpc_logger.setLevel(original_level)

if __name__ == '__main__':
    unittest.main()
