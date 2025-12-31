"""模块配置兼容性导入

此文件保留用于向后兼容。
配置已拆分到独立文件：
- consistency_config.py: 一致性检查配置
- transform_config.py: 坐标变换配置
- transition_config.py: 平滑过渡配置
- backup_config.py: 备份控制器配置

请直接从各自的模块导入，或从 default_config.py 导入。
"""

# 从拆分后的模块导入，保持向后兼容
from .consistency_config import CONSISTENCY_CONFIG, CONSISTENCY_VALIDATION_RULES
from .transform_config import TRANSFORM_CONFIG, TRANSFORM_VALIDATION_RULES
from .transition_config import TRANSITION_CONFIG, TRANSITION_VALIDATION_RULES
from .backup_config import BACKUP_CONFIG, BACKUP_VALIDATION_RULES

# 合并验证规则（向后兼容）
MODULES_VALIDATION_RULES = {}
MODULES_VALIDATION_RULES.update(CONSISTENCY_VALIDATION_RULES)
MODULES_VALIDATION_RULES.update(TRANSFORM_VALIDATION_RULES)
MODULES_VALIDATION_RULES.update(TRANSITION_VALIDATION_RULES)
MODULES_VALIDATION_RULES.update(BACKUP_VALIDATION_RULES)

__all__ = [
    'CONSISTENCY_CONFIG',
    'TRANSFORM_CONFIG',
    'TRANSITION_CONFIG',
    'BACKUP_CONFIG',
    'MODULES_VALIDATION_RULES',
    # 单独的验证规则（新增）
    'CONSISTENCY_VALIDATION_RULES',
    'TRANSFORM_VALIDATION_RULES',
    'TRANSITION_VALIDATION_RULES',
    'BACKUP_VALIDATION_RULES',
]
