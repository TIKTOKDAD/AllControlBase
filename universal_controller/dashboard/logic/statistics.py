"""
Statistics management logic for Dashboard
"""

import time
from collections import deque
from typing import Dict, Any

from ..models import StatisticsData

class StatisticsManager:
    def __init__(self):
        self._start_time = time.time()
        
        # Statistics counters
        self._total_cycles = 0
        self._state_counts = {i: 0 for i in range(7)}
        self._mpc_success_count = 0
        self._backup_switch_count = 0
        self._safety_limit_count = 0
        self._tf2_fallback_count = 0
        self._soft_disable_count = 0
        
        # State change tracking
        self._last_state: int = 0
        self._last_safety_check_passed: bool = True
        self._last_tf2_fallback_active: bool = False

        # History buffers
        self._history_size = 500
        self._solve_time_history = deque(maxlen=self._history_size)
        self._lateral_error_history = deque(maxlen=self._history_size)
        self._alpha_history = deque(maxlen=self._history_size)

        # Cycle time statistics
        self._cycle_times = deque(maxlen=100)
        self._last_update_time = time.time()

    def update(self, diagnostics: Dict[str, Any]):
        """Update statistics with new diagnostics data"""
        current_time = time.time()

        # Update cycle time
        cycle_time = current_time - self._last_update_time
        self._cycle_times.append(cycle_time * 1000)
        self._last_update_time = current_time

        self._total_cycles += 1

        if diagnostics:
            # State counts
            state = diagnostics.get('state', 0)
            if 0 <= state < 7:
                self._state_counts[state] += 1
            
            # Backup switch count
            if state == 4 and self._last_state != 4:
                self._backup_switch_count += 1
            
            # Soft disable count
            if state == 2 and self._last_state != 2:
                self._soft_disable_count += 1
            
            self._last_state = state
            
            # Safety limit count
            safety_check_passed = diagnostics.get('safety_check_passed', True)
            if not safety_check_passed and self._last_safety_check_passed:
                self._safety_limit_count += 1
            self._last_safety_check_passed = safety_check_passed
            
            # TF2 fallback count
            transform = diagnostics.get('transform', {})
            if isinstance(transform, dict):
                fallback_ms = transform.get('fallback_duration_ms', 0)
                tf2_fallback_active = fallback_ms > 0
                if tf2_fallback_active and not self._last_tf2_fallback_active:
                    self._tf2_fallback_count += 1
                self._last_tf2_fallback_active = tf2_fallback_active

            # MPC success count
            if diagnostics.get('mpc_success', False):
                self._mpc_success_count += 1

            # History data
            self._solve_time_history.append(diagnostics.get('mpc_solve_time_ms', 0))
            
            tracking = diagnostics.get('tracking', {})
            if isinstance(tracking, dict):
                self._lateral_error_history.append(tracking.get('lateral_error', 0))

            consistency = diagnostics.get('consistency', {})
            if isinstance(consistency, dict):
                self._alpha_history.append(consistency.get('alpha_soft', 0))

    def get_statistics_data(self) -> StatisticsData:
        """Build StatisticsData object"""
        elapsed = time.time() - self._start_time
        hours = int(elapsed // 3600)
        minutes = int((elapsed % 3600) // 60)
        seconds = int(elapsed % 60)

        avg_cycle = sum(self._cycle_times) / len(self._cycle_times) if self._cycle_times else 0
        actual_freq = 1000 / avg_cycle if avg_cycle > 0 else 0

        return StatisticsData(
            elapsed_time=elapsed,
            elapsed_time_str=f'{hours:02d}:{minutes:02d}:{seconds:02d}',
            total_cycles=self._total_cycles,
            actual_freq=actual_freq,
            avg_cycle_ms=avg_cycle,
            max_cycle_ms=max(self._cycle_times) if self._cycle_times else 0,
            min_cycle_ms=min(self._cycle_times) if self._cycle_times else 0,
            mpc_success_rate=(self._mpc_success_count / self._total_cycles * 100
                             if self._total_cycles > 0 else 0),
            state_counts=self._state_counts.copy(),
            backup_switch_count=self._backup_switch_count,
            safety_limit_count=self._safety_limit_count,
            tf2_fallback_count=self._tf2_fallback_count,
            soft_disable_count=self._soft_disable_count,
        )

    def get_history(self) -> Dict[str, Any]:
        """Get history data for plots"""
        return {
            'solve_time': list(self._solve_time_history),
            'lateral_error': list(self._lateral_error_history),
            'alpha': list(self._alpha_history),
        }
