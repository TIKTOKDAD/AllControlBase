#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
配置生成器

根据分析结果生成优化后的 YAML 配置文件。
"""

import yaml
import copy
from datetime import datetime
from typing import Dict, List, Any, Optional
from pathlib import Path

from .diagnostics_analyzer import AnalysisResult


class ConfigGenerator:
    """配置生成器"""
    
    # YAML 配置文件模板头部
    HEADER_TEMPLATE = """# {filename}
# 自动生成的优化配置
# 生成时间: {timestamp}
# 基于: {base_config}
#
# 本配置由诊断调优工具自动生成
# 请在部署前仔细检查各项参数
#
# =============================================================================
# 优化摘要
# =============================================================================
{summary}
#
"""

    def __init__(self, base_config: Dict[str, Any], base_config_path: str = ""):
        """
        初始化配置生成器
        
        Args:
            base_config: 基础配置字典
            base_config_path: 基础配置文件路径
        """
        self.base_config = copy.deepcopy(base_config)
        self.base_config_path = base_config_path
        self.applied_changes: List[Dict[str, Any]] = []
    
    def apply_results(self, results: List[AnalysisResult], 
                      min_confidence: float = 0.6,
                      severity_filter: Optional[List[str]] = None) -> Dict[str, Any]:
        """
        应用分析结果到配置
        
        Args:
            results: 分析结果列表
            min_confidence: 最小置信度阈值
            severity_filter: 严重程度过滤器 (None 表示全部)
        
        Returns:
            优化后的配置字典
        """
        config = copy.deepcopy(self.base_config)
        self.applied_changes = []
        
        for result in results:
            # 过滤置信度
            if result.confidence < min_confidence:
                continue
            
            # 过滤严重程度
            if severity_filter and result.severity not in severity_filter:
                continue
            
            # 跳过仅信息性的建议（除非明确包含）
            if result.severity == 'info' and severity_filter is None:
                continue
            
            # 应用更改
            if self._apply_change(config, result):
                self.applied_changes.append({
                    'parameter': result.parameter,
                    'old_value': result.current_value,
                    'new_value': result.suggested_value,
                    'reason': result.reason,
                    'severity': result.severity,
                    'confidence': result.confidence
                })
        
        return config
    
    def _apply_change(self, config: Dict[str, Any], result: AnalysisResult) -> bool:
        """应用单个更改"""
        try:
            # 解析参数路径 (如 "mpc.weights.position")
            parts = result.parameter.split('.')
            
            # 导航到父节点
            current = config
            for part in parts[:-1]:
                if part not in current:
                    current[part] = {}
                current = current[part]
            
            # 设置值 (转换 numpy 类型为 Python 原生类型)
            key = parts[-1]
            value = result.suggested_value
            
            # 处理 numpy 类型
            if hasattr(value, 'item'):
                value = value.item()
            elif isinstance(value, (list, tuple)):
                value = [v.item() if hasattr(v, 'item') else v for v in value]
            
            current[key] = value
            return True
            
        except Exception as e:
            print(f"警告: 无法应用更改 {result.parameter}: {e}")
            return False

    
    def generate_yaml(self, config: Dict[str, Any], 
                      output_path: str,
                      summary: Dict[str, Any] = None) -> str:
        """
        生成 YAML 配置文件
        
        Args:
            config: 配置字典
            output_path: 输出文件路径
            summary: 分析摘要
        
        Returns:
            生成的文件路径
        """
        # 生成摘要注释
        summary_lines = []
        if summary:
            summary_lines.append(f"# 样本数: {summary.get('total_samples', 'N/A')}")
            
            mpc = summary.get('mpc', {})
            if mpc:
                summary_lines.append(f"# MPC 成功率: {mpc.get('success_rate', 'N/A')}%")
                summary_lines.append(f"# MPC 平均求解时间: {mpc.get('avg_solve_time_ms', 'N/A')}ms")
            
            tracking = summary.get('tracking', {})
            if 'lateral' in tracking:
                summary_lines.append(f"# 横向误差(avg): {tracking['lateral'].get('avg_cm', 'N/A')}cm")
            if 'longitudinal' in tracking:
                summary_lines.append(f"# 纵向误差(avg): {tracking['longitudinal'].get('avg_cm', 'N/A')}cm")
        
        if self.applied_changes:
            summary_lines.append("#")
            summary_lines.append("# 应用的优化:")
            for change in self.applied_changes:
                summary_lines.append(f"#   - {change['parameter']}: {change['old_value']} → {change['new_value']}")
                summary_lines.append(f"#     原因: {change['reason']}")
        
        summary_text = '\n'.join(summary_lines) if summary_lines else "# 无优化建议"
        
        # 生成头部
        filename = Path(output_path).name
        header = self.HEADER_TEMPLATE.format(
            filename=filename,
            timestamp=datetime.now().strftime("%Y-%m-%d %H:%M:%S"),
            base_config=self.base_config_path or "unknown",
            summary=summary_text
        )
        
        # 生成 YAML 内容
        yaml_content = self._dict_to_yaml_with_comments(config)
        
        # 写入文件
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        
        with open(output_path, 'w', encoding='utf-8') as f:
            f.write(header)
            f.write(yaml_content)
        
        return str(output_path)
    
    def _dict_to_yaml_with_comments(self, config: Dict[str, Any]) -> str:
        """将字典转换为带注释的 YAML"""
        # 使用自定义 Dumper 保持顺序和格式
        class CustomDumper(yaml.SafeDumper):
            pass
        
        def str_representer(dumper, data):
            if '\n' in data:
                return dumper.represent_scalar('tag:yaml.org,2002:str', data, style='|')
            return dumper.represent_scalar('tag:yaml.org,2002:str', data)
        
        CustomDumper.add_representer(str, str_representer)
        
        # 添加分节注释
        sections = {
            'system': '系统配置',
            'node': 'ROS 节点配置',
            'topics': '话题配置',
            'tf': 'TF 配置',
            'watchdog': '超时配置',
            'diagnostics': '诊断配置',
            'mpc': 'MPC 配置',
            'constraints': '速度约束',
            'trajectory': '轨迹配置',
            'consistency': '一致性检查配置',
            'tracking': '跟踪质量评估配置',
            'safety': '安全配置',
            'backup': '备份控制器配置',
            'transform': '坐标变换配置',
            'transition': '平滑过渡配置',
            'ekf': 'EKF 状态估计配置',
            'cmd_vel_adapter': 'cmd_vel 适配器配置'
        }
        
        lines = []
        for key, value in config.items():
            # 添加分节注释
            if key in sections:
                lines.append("")
                lines.append(f"# =============================================================================")
                lines.append(f"# {sections[key]}")
                lines.append(f"# =============================================================================")
            
            # 转换该节
            section_yaml = yaml.dump({key: value}, Dumper=CustomDumper, 
                                     default_flow_style=False, 
                                     allow_unicode=True,
                                     sort_keys=False)
            lines.append(section_yaml.rstrip())
        
        return '\n'.join(lines)
    
    def get_change_report(self) -> str:
        """获取更改报告"""
        if not self.applied_changes:
            return "无更改"
        
        report_lines = ["=" * 60, "配置更改报告", "=" * 60, ""]
        
        # 按类别分组
        by_category = {}
        for change in self.applied_changes:
            category = change['parameter'].split('.')[0]
            if category not in by_category:
                by_category[category] = []
            by_category[category].append(change)
        
        for category, changes in by_category.items():
            report_lines.append(f"\n[{category.upper()}]")
            for change in changes:
                report_lines.append(f"  {change['parameter']}:")
                report_lines.append(f"    旧值: {change['old_value']}")
                report_lines.append(f"    新值: {change['new_value']}")
                report_lines.append(f"    原因: {change['reason']}")
                report_lines.append(f"    置信度: {change['confidence']*100:.0f}%")
                report_lines.append(f"    严重程度: {change['severity']}")
        
        report_lines.append("")
        report_lines.append("=" * 60)
        report_lines.append(f"共 {len(self.applied_changes)} 项更改")
        
        return '\n'.join(report_lines)
