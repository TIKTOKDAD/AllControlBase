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
    print("  版本: v3.17.12")
    print("=" * 60)
    print()
    print("正在启动可视化界面...")
    print("提示: 当前使用模拟数据模式")
    print("      如需连接真实控制器，请使用 roslaunch controller_ros controller.launch dashboard:=true")
    print()
    
    app = QApplication(sys.argv)
    
    # 创建数据源 (显式启用模拟数据模式)
    data_source = DashboardDataSource(
        controller_manager=None,
        config=DEFAULT_CONFIG,
        use_mock=True  # 显式启用模拟模式
    )
    
    # 创建主窗口
    window = DashboardWindow(data_source)
    window.setWindowTitle('Universal Controller Dashboard [调试模式]')
    window.show()
    
    print("Dashboard 已启动!")
    print("按 Ctrl+C 或关闭窗口退出")
    print()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
