"""
Configuration Validator

Legacy wrapper that delegates to config.validation.ConfigValidator.
"""
from typing import Dict, Any

from ..config.validation import ConfigValidator as CoreConfigValidator


class ConfigValidator:
    """Compatibility wrapper for config.validation.ConfigValidator."""

    @staticmethod
    def validate(config: Dict[str, Any]) -> None:
        CoreConfigValidator.validate(config)
