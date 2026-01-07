
import unittest
import numpy as np
import sys
import os

# Ensure we can import universal_controller
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from universal_controller.tracker.pure_pursuit import PurePursuitController
from universal_controller.tracker.mpc_controller import MPCController, ACADOS_AVAILABLE
from universal_controller.core.data_types import ConsistencyResult, Trajectory, Point3D, Header, ControlOutput
from universal_controller.config.default_config import DEFAULT_CONFIG, PLATFORM_CONFIG
from universal_controller.core.enums import PlatformType

class TestAckermannConstraints(unittest.TestCase):
    def setUp(self):
        self.config = DEFAULT_CONFIG.copy()
        
        # Create an Ackermann-like config (cannot rotate in place)
        self.ackermann_config = PLATFORM_CONFIG['differential'].copy()
        self.ackermann_config['type'] = PlatformType.ACKERMANN
        self.ackermann_config['can_rotate_in_place'] = False # Critical flag
        
        self.consistency = ConsistencyResult(0.8, 1, 1, 1, True, True)

    def test_pure_pursuit_ackermann_reversing(self):
        """Test that Pure Pursuit uses reversing strategy for Ackermann vehicles when target is behind
        
        Ackermann vehicles cannot rotate in place, so when the target is behind,
        the controller should use a reversing (backward driving) strategy instead
        of attempting in-place rotation.
        """
        pp = PurePursuitController(self.config, self.ackermann_config)
        
        # State: Stopped at origin, facing East (0)
        state = np.array([0, 0, 0, 0.0, 0, 0, 0, 0])
        
        # Trajectory: A point strictly behind the robot (-1, 0)
        # This forces the controller to attempt reversing logic
        trajectory = Trajectory(
            header=Header(0, 'world'),
            points=[Point3D(0, 0, 0), Point3D(-1, 0, 0)],
            velocities=None
        )
        
        cmd = pp.compute(state, trajectory, self.consistency)
        
        # Expectation: 
        # Since can_rotate_in_place is False, it should use reversing strategy
        # - vx should be negative (reversing)
        # - Command should be successful (reversing is a valid strategy)
        self.assertLess(cmd.vx, 0.0, "Velocity should be negative (reversing)")
        self.assertTrue(cmd.success, "Command should be successful (reversing strategy)")
        
        pp.shutdown()

    def test_pure_pursuit_differential_reversing(self):
        """Test that Pure Pursuit ALLOWS in-place rotation for Differential vehicles"""
        diff_config = PLATFORM_CONFIG['differential'].copy()
        diff_config['can_rotate_in_place'] = True
        
        pp = PurePursuitController(self.config, diff_config)
        
        state = np.array([0, 0, 0, 0.0, 0, 0, 0, 0])
        trajectory = Trajectory(
            header=Header(0, 'world'),
            points=[Point3D(0, 0, 0), Point3D(-1, 0, 0)],
            velocities=None
        )
        
        cmd = pp.compute(state, trajectory, self.consistency)
        
        # Expectation: Should output rotation
        self.assertTrue(abs(cmd.omega) > 0.0, "Should attempt to rotate")
        self.assertTrue(cmd.success, "Command should be successful")
        
        pp.shutdown()

    def test_mpc_model_naming_ackermann(self):
        """Test that MPC generates distinct model names for Ackermann"""
        if not ACADOS_AVAILABLE:
            print("Skipping MPC Ackermann test (ACADOS not available)")
            return

        mpc_ack = MPCController(self.config, self.ackermann_config)
        self.assertIn("ACKERMANN", mpc_ack._model_name_base, 
                      "Model name should contain platform type")
        
        mpc_diff = MPCController(self.config, PLATFORM_CONFIG['differential'])
        self.assertNotEqual(mpc_ack._model_name_base, mpc_diff._model_name_base,
                            "Model names should differ between platforms")
        
        mpc_ack.shutdown()
        mpc_diff.shutdown()

if __name__ == '__main__':
    unittest.main()
