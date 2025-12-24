"""
TF2InjectionManager 单元测试
"""
import pytest
import sys
import os

# 添加 src 目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from controller_ros.utils.tf2_injection_manager import TF2InjectionManager


class MockTFBridge:
    """模拟 TF Bridge"""
    
    def __init__(self, initialized: bool = True, can_transform_result: bool = True):
        self.is_initialized = initialized
        self._can_transform_result = can_transform_result
        self._lookup_transform_calls = []
        self._can_transform_calls = []
    
    def can_transform(self, target_frame: str, source_frame: str, timeout_sec: float = 0.01) -> bool:
        self._can_transform_calls.append((target_frame, source_frame, timeout_sec))
        return self._can_transform_result
    
    def lookup_transform(self, target_frame: str, source_frame: str, 
                        time=None, timeout_sec: float = 0.01) -> dict:
        self._lookup_transform_calls.append((target_frame, source_frame, time, timeout_sec))
        return {
            'translation': (0.0, 0.0, 0.0),
            'rotation': (0.0, 0.0, 0.0, 1.0)
        }


class MockCoordTransformer:
    """模拟坐标变换器"""
    
    def __init__(self):
        self._tf2_callback = None
    
    def set_tf2_lookup_callback(self, callback):
        self._tf2_callback = callback


class MockControllerManager:
    """模拟 ControllerManager"""
    
    def __init__(self, has_coord_transformer: bool = True):
        if has_coord_transformer:
            self.coord_transformer = MockCoordTransformer()
        else:
            self.coord_transformer = None


class TestTF2InjectionManager:
    """测试 TF2InjectionManager"""
    
    def test_initialization(self):
        """测试初始化"""
        tf_bridge = MockTFBridge()
        manager = MockControllerManager()
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
        )
        
        assert injection_manager.is_injected == False
        assert injection_manager.injection_attempted == False
        assert injection_manager.retry_count == 0
    
    def test_inject_success(self):
        """测试成功注入"""
        tf_bridge = MockTFBridge(initialized=True, can_transform_result=True)
        manager = MockControllerManager()
        
        log_messages = []
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
            log_info=lambda msg: log_messages.append(('info', msg)),
            log_warn=lambda msg: log_messages.append(('warn', msg)),
        )
        
        result = injection_manager.inject(blocking=False)
        
        assert result == True
        assert injection_manager.is_injected == True
        assert injection_manager.injection_attempted == True
        assert manager.coord_transformer._tf2_callback is not None
    
    def test_inject_tf_bridge_none(self):
        """测试 TF bridge 为 None"""
        manager = MockControllerManager()
        
        injection_manager = TF2InjectionManager(
            tf_bridge=None,
            controller_manager=manager,
        )
        
        result = injection_manager.inject(blocking=False)
        
        assert result == False
        assert injection_manager.is_injected == False
    
    def test_inject_tf_not_initialized(self):
        """测试 TF2 未初始化"""
        tf_bridge = MockTFBridge(initialized=False)
        manager = MockControllerManager()
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
        )
        
        result = injection_manager.inject(blocking=False)
        
        assert result == False
        assert injection_manager.is_injected == False
    
    def test_inject_no_coord_transformer(self):
        """测试没有坐标变换器"""
        tf_bridge = MockTFBridge()
        manager = MockControllerManager(has_coord_transformer=False)
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
        )
        
        result = injection_manager.inject(blocking=False)
        
        assert result == False
        assert injection_manager.is_injected == False
    
    def test_inject_controller_manager_none(self):
        """测试 ControllerManager 为 None"""
        tf_bridge = MockTFBridge()
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=None,
        )
        
        result = injection_manager.inject(blocking=False)
        
        assert result == False
        assert injection_manager.is_injected == False
    
    def test_try_reinjection_already_injected(self):
        """测试已注入时不重试"""
        tf_bridge = MockTFBridge()
        manager = MockControllerManager()
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
        )
        
        # 先注入
        injection_manager.inject(blocking=False)
        assert injection_manager.is_injected == True
        
        # 尝试重新注入，应该返回 False（不需要重试）
        result = injection_manager.try_reinjection_if_needed()
        assert result == False
    
    def test_try_reinjection_not_attempted(self):
        """测试未尝试过注入时不重试"""
        tf_bridge = MockTFBridge()
        manager = MockControllerManager()
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
        )
        
        # 未尝试过注入
        assert injection_manager.injection_attempted == False
        
        # 尝试重新注入，应该返回 False
        result = injection_manager.try_reinjection_if_needed()
        assert result == False
    
    def test_try_reinjection_interval(self):
        """测试重试间隔"""
        tf_bridge = MockTFBridge(initialized=True, can_transform_result=False)
        manager = MockControllerManager()
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
            config={'retry_interval_cycles': 5},
        )
        
        # 先尝试注入（会失败因为 can_transform 返回 False，但回调仍会注入）
        # 为了测试重试逻辑，我们需要模拟注入失败的情况
        # 这里我们手动设置状态
        injection_manager._injection_attempted = True
        injection_manager._injected = False
        
        # 前 4 次调用不应该触发重试
        for i in range(4):
            result = injection_manager.try_reinjection_if_needed()
            assert result == False, f"Should not retry at iteration {i}"
        
        # 第 5 次应该触发重试
        result = injection_manager.try_reinjection_if_needed()
        assert result == True
        assert injection_manager.retry_count == 1
    
    def test_try_reinjection_max_retries(self):
        """测试最大重试次数"""
        # 使用未初始化的 TF bridge，这样注入会失败
        tf_bridge = MockTFBridge(initialized=False)
        manager = MockControllerManager()
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
            config={
                'retry_interval_cycles': 1,  # 每次都重试
                'max_retries': 3,
            },
        )
        
        # 手动设置状态（模拟初始注入失败后的状态）
        injection_manager._injection_attempted = True
        injection_manager._injected = False
        
        # 现在让 TF bridge 变为已初始化，但注入仍会失败（因为 coord_transformer 会被注入）
        # 为了测试最大重试次数，我们需要让注入一直失败
        # 移除 coord_transformer
        manager.coord_transformer = None
        
        # 前 3 次应该重试
        for i in range(3):
            result = injection_manager.try_reinjection_if_needed()
            # 由于 TF bridge 未初始化，不会触发重试
            # 我们需要让它初始化
        
        # 重新设置：让 TF bridge 初始化，但没有 coord_transformer
        tf_bridge.is_initialized = True
        injection_manager._retry_count = 0
        
        # 前 3 次应该重试
        for i in range(3):
            result = injection_manager.try_reinjection_if_needed()
            assert result == True, f"Should retry at iteration {i}"
            # 注入会失败因为没有 coord_transformer，所以 _injected 保持 False
            assert injection_manager.is_injected == False
        
        # 第 4 次不应该重试（超过最大次数）
        result = injection_manager.try_reinjection_if_needed()
        assert result == False
    
    def test_try_reinjection_unlimited(self):
        """测试无限重试"""
        # 使用没有 coord_transformer 的 manager，这样注入会失败
        tf_bridge = MockTFBridge(initialized=True, can_transform_result=False)
        manager = MockControllerManager(has_coord_transformer=False)
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
            config={
                'retry_interval_cycles': 1,
                'max_retries': -1,  # 无限重试
            },
        )
        
        # 手动设置状态
        injection_manager._injection_attempted = True
        injection_manager._injected = False
        
        # 应该一直重试（测试 10 次）
        for i in range(10):
            result = injection_manager.try_reinjection_if_needed()
            assert result == True, f"Should retry at iteration {i}"
            # 注入会失败因为没有 coord_transformer
            assert injection_manager.is_injected == False
    
    def test_reset(self):
        """测试重置"""
        tf_bridge = MockTFBridge()
        manager = MockControllerManager()
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
            config={'retry_interval_cycles': 5},
        )
        
        # 模拟一些状态
        injection_manager._injection_attempted = True
        injection_manager._retry_counter = 3
        
        # 重置
        injection_manager.reset()
        
        # 计数器应该重置
        assert injection_manager._retry_counter == 0
        # 注入状态不应该重置
        assert injection_manager._injection_attempted == True
    
    def test_get_status(self):
        """测试获取状态"""
        tf_bridge = MockTFBridge()
        manager = MockControllerManager()
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
            config={
                'source_frame': 'base_link',
                'target_frame': 'odom',
            },
        )
        
        injection_manager.inject(blocking=False)
        
        status = injection_manager.get_status()
        
        assert status['injected'] == True
        assert status['injection_attempted'] == True
        assert status['source_frame'] == 'base_link'
        assert status['target_frame'] == 'odom'
    
    def test_custom_config(self):
        """测试自定义配置"""
        tf_bridge = MockTFBridge()
        manager = MockControllerManager()
        
        injection_manager = TF2InjectionManager(
            tf_bridge=tf_bridge,
            controller_manager=manager,
            config={
                'source_frame': 'custom_base',
                'target_frame': 'custom_odom',
                'buffer_warmup_timeout_sec': 5.0,
                'buffer_warmup_interval_sec': 0.2,
                'retry_interval_cycles': 100,
                'max_retries': 10,
            },
        )
        
        assert injection_manager._source_frame == 'custom_base'
        assert injection_manager._target_frame == 'custom_odom'
        assert injection_manager._buffer_warmup_timeout_sec == 5.0
        assert injection_manager._buffer_warmup_interval_sec == 0.2
        assert injection_manager._retry_interval_cycles == 100
        assert injection_manager._max_retries == 10


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
