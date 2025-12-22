"""配置模块"""
from .default_config import (
    DEFAULT_CONFIG, 
    PLATFORM_CONFIG, 
    get_config_value,
    validate_config,
    ConfigValidationError,
    CONFIG_VALIDATION_RULES
)
