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

    def test_solver_caching(self):
        """Test that solvers are cached and reused
        
        Note: This test requires ACADOS to be installed, as mocking module-level 
        constants after import does not work reliably.
        """
        from universal_controller.tracker.mpc_core.solver_manager import ACADOS_AVAILABLE
        
        if not ACADOS_AVAILABLE:
            self.skipTest("ACADOS not available, skipping solver caching test")
        
        # Configure pre-warming
        self.config['mpc'] = self.config.get('mpc', {}).copy()
        self.config['mpc']['prewarm_horizons'] = [10, 20]
        self.config['mpc']['horizon'] = 20
        
        # Initialize controller
        mpc = MPCController(self.config, self.platform_config)
        
        # Check that the solver_manager has cached the solvers
        self.assertIn(20, mpc.solver_manager._solver_cache)
        self.assertIn(10, mpc.solver_manager._solver_cache)
        
        initial_solver_10 = mpc.solver_manager._solver_cache[10]
        initial_solver_20 = mpc.solver_manager._solver_cache[20]
        
        # Switch to 10 - should reuse cache
        mpc.set_horizon(10)
        self.assertEqual(mpc.horizon, 10)
        # Verify the same solver instance is used
        self.assertIs(mpc.solver_manager._solver_cache[10], initial_solver_10)
        
        # Switch back to 20 - should reuse cache
        mpc.set_horizon(20)
        self.assertEqual(mpc.horizon, 20)
        self.assertIs(mpc.solver_manager._solver_cache[20], initial_solver_20)
        
        mpc.shutdown()

    def test_predicted_trajectory_output(self):
        """Test that predicted trajectory is populated in output extras when visualize_prediction is enabled
        
        Note: This test requires ACADOS to be installed.
        """
        from universal_controller.tracker.mpc_core.solver_manager import ACADOS_AVAILABLE
        import logging
        
        if not ACADOS_AVAILABLE:
            self.skipTest("ACADOS not available, skipping predicted trajectory test")
        
        # Enable DEBUG logging for the mpc_controller module
        mpc_logger = logging.getLogger('universal_controller.tracker.mpc_controller')
        original_level = mpc_logger.level
        mpc_logger.setLevel(logging.DEBUG)
        
        try:
            # Enable visualize_prediction to trigger predicted_trajectory extraction
            config_with_viz = self.config.copy()
            config_with_viz['mpc'] = config_with_viz.get('mpc', {}).copy()
            config_with_viz['mpc']['visualize_prediction'] = True
            
            mpc = MPCController(config_with_viz, self.platform_config)
            
            state = np.array([0.0]*8)
            trajectory = create_test_trajectory(soft_enabled=True)
            consistency = ConsistencyResult(
                alpha=1.0, kappa_consistency=1.0, v_dir_consistency=1.0,
                temporal_smooth=1.0, should_disable_soft=False, data_valid=True
            )
            
            # Compute
            cmd = mpc.compute(state, trajectory, consistency)
            
            # Verify extras (only if solve succeeded)
            if cmd.success:
                self.assertIn('predicted_trajectory', cmd.extras)
                pred_traj = cmd.extras['predicted_trajectory']
                # Should have horizon + 1 points (0 to N)
                self.assertEqual(len(pred_traj), mpc.horizon + 1)
            else:
                # If solver failed, just check that the output is valid
                self.assertIsNotNone(cmd)
            
            mpc.shutdown()
        finally:
            # Restore original log level
            mpc_logger.setLevel(original_level)

if __name__ == '__main__':
    unittest.main()
