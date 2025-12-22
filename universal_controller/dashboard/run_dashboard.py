#!/usr/bin/env python3
"""
启动 Dashboard 可视化界面

用法:
    python -m universal_controller.dashboard.run_dashboard
    
或者:
    python universal_controller/dashboard/run_dashboard.py
"""

import sys
import os

# 添加项目根目录到路径
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from PyQt5.QtWidgets import QApplication
from universal_controller.dashboard.main_window import DashboardWindow
from universal_controller.dashboard.data_source import DashboardDataSource
from universal_controller.config.default_config import DEFAULT_CONFIG


def main():
    """主函数"""
    print("=" * 60)
    print("  Universal Controller Dashboard")
    print("  版本: v3.17.7")
    print("=" * 60)
    print()
    print("正在启动可视化界面...")
    print("提示: 当前使用模拟数据，如需连接真实控制器，请传入 ControllerManager 实例")
    print()
    
    app = QApplication(sys.argv)
    
    # 创建数据源 (使用模拟数据)
    data_source = DashboardDataSource(
        controller_manager=None,  # 可以传入真实的 ControllerManager
        config=DEFAULT_CONFIG
    )
    
    # 创建主窗口
    window = DashboardWindow(data_source)
    window.show()
    
    print("Dashboard 已启动!")
    print("按 Ctrl+C 或关闭窗口退出")
    print()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
