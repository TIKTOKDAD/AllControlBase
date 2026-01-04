"""
Configuration Validator

Provides schema validation for the universal_controller configuration dictionary.
Enforces type safety and existence of critical keys to fail-early on configuration errors.
"""
from typing import Dict, Any, List, Optional
import logging

logger = logging.getLogger(__name__)

class ConfigValidator:
    """configuration Validator class"""
    
    @staticmethod
    def validate(config: Dict[str, Any]) -> None:
        """
        Validate the configuration dictionary.
        Raises ValueError if validation fails.
        """
        if not isinstance(config, dict):
            raise ValueError(f"Configuration must be a dictionary, got {type(config)}")
            
        # 1. System Config
        system = config.get('system', {})
        if not isinstance(system, dict):
             raise ValueError("Config 'system' section must be a dictionary")
        
        platform = system.get('platform', 'differential')
        valid_platforms = ['differential', 'omni', 'ackermann', 'quadrotor']
        if platform not in valid_platforms:
            raise ValueError(f"Invalid platform '{platform}'. Must be one of {valid_platforms}")
            
        # 2. MPC Config
        mpc = config.get('mpc', {})
        if not isinstance(mpc, dict):
            raise ValueError("Config 'mpc' section must be a dictionary")
            
        if 'dt' in mpc:
            dt = mpc['dt']
            if not isinstance(dt, (int, float)) or dt <= 0:
                raise ValueError(f"Invalid mpc.dt '{dt}'. Must be distinct positive float.")
        
        # 3. Transform Config
        transform = config.get('transform', {})
        if not isinstance(transform, dict):
            raise ValueError("Config 'transform' section must be a dictionary")
            
        source_frame = transform.get('source_frame')
        target_frame = transform.get('target_frame')
        
        # Explicitly check for empty strings if keys exist
        if 'source_frame' in transform and not source_frame:
             raise ValueError("transform.source_frame cannot be empty if specified")
        if 'target_frame' in transform and not target_frame:
             raise ValueError("transform.target_frame cannot be empty if specified")

        logger.info("Configuration validation passed.")
