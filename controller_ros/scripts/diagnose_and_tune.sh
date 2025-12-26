#!/bin/bash
# ============================================================================
# Controller Diagnostics and Tuning Script
# 
# Features:
#   1. Detect chassis and sensor topic publish rates
#   2. Analyze trajectory input characteristics
#   3. Generate optimized configuration recommendations
#   4. Optional: Auto-generate tuned YAML config
#
# Usage:
#   chmod +x diagnose_and_tune.sh
#   ./diagnose_and_tune.sh                    # Interactive diagnostics
#   ./diagnose_and_tune.sh --auto             # Auto-generate config
#   ./diagnose_and_tune.sh --output my.yaml   # Specify output file
#
# Author: Auto-generated
# Date: 2024-12-26
# ============================================================================

set -e

# ============================================================================
# Color definitions
# ============================================================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# ============================================================================
# Default configuration
# ============================================================================
SAMPLE_DURATION=5
OUTPUT_FILE=""
AUTO_MODE=false
VERBOSE=false

# Topic names (can be overridden via parameters)
ODOM_TOPIC="/odom"
TRAJ_TOPIC="/nn/local_trajectory"
IMU_TOPIC="/imu"
CMD_VEL_TOPIC="/mobile_base/commands/velocity"

# ============================================================================
# Parse command line arguments
# ============================================================================
while [[ $# -gt 0 ]]; do
    case $1 in
        --auto|-a) AUTO_MODE=true; shift ;;
        --output|-o) OUTPUT_FILE="$2"; shift 2 ;;
        --duration|-d) SAMPLE_DURATION="$2"; shift 2 ;;
        --odom) ODOM_TOPIC="$2"; shift 2 ;;
        --traj) TRAJ_TOPIC="$2"; shift 2 ;;
        --imu) IMU_TOPIC="$2"; shift 2 ;;
        --cmd-vel) CMD_VEL_TOPIC="$2"; shift 2 ;;
        --verbose|-v) VERBOSE=true; shift ;;
        --help|-h)
            echo "Usage: $0 [options]"
            echo ""
            echo "Options:"
            echo "  -a, --auto              Auto mode, generate optimized config"
            echo "  -o, --output FILE       Specify output config file"
            echo "  -d, --duration SEC      Sample duration (default: 5s)"
            echo "  --odom TOPIC            Odometry topic (default: /odom)"
            echo "  --traj TOPIC            Trajectory topic (default: /nn/local_trajectory)"
            echo "  --imu TOPIC             IMU topic (default: /imu)"
            echo "  --cmd-vel TOPIC         Chassis command topic"
            echo "  -v, --verbose           Verbose output"
            echo "  -h, --help              Show help"
            exit 0
            ;;
        *) echo "Unknown parameter: $1"; exit 1 ;;
    esac
done

# ============================================================================
# Helper functions
# ============================================================================
print_header() {
    echo ""
    echo -e "${BLUE}============================================================================${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}============================================================================${NC}"
    echo ""
}

print_success() { echo -e "${GREEN}[OK] $1${NC}"; }
print_warning() { echo -e "${YELLOW}[WARN] $1${NC}"; }
print_error() { echo -e "${RED}[ERROR] $1${NC}"; }
print_info() { echo -e "${CYAN}[INFO] $1${NC}"; }

# ============================================================================
# Check ROS environment
# ============================================================================
check_ros_env() {
    print_header "Step 1: Check ROS Environment"
    
    if [ -f /opt/ros/noetic/setup.bash ]; then
        source /opt/ros/noetic/setup.bash
        print_success "ROS Noetic environment loaded"
    elif [ -f /opt/ros/melodic/setup.bash ]; then
        source /opt/ros/melodic/setup.bash
        print_success "ROS Melodic environment loaded"
    else
        print_error "ROS environment not found!"
        exit 1
    fi
    
    if ! rostopic list &> /dev/null; then
        print_error "roscore not running! Please start roscore first"
        exit 1
    fi
    print_success "roscore is running"
}

# ============================================================================
# Get topic frequency
# ============================================================================
get_topic_hz() {
    local topic=$1
    local duration=${2:-3}
    
    if ! rostopic list | grep -q "^${topic}$"; then
        echo "0"
        return
    fi
    
    local hz_output=$(timeout ${duration}s rostopic hz "$topic" 2>&1 | tail -1)
    local hz=$(echo "$hz_output" | grep -oP 'average rate: \K[0-9.]+' || echo "0")
    
    if [ -z "$hz" ] || [ "$hz" == "0" ]; then
        echo "0"
    else
        echo "$hz"
    fi
}

# ============================================================================
# Analyze odometry
# ============================================================================
analyze_odom() {
    print_header "Step 2: Analyze Odometry ($ODOM_TOPIC)"
    
    if ! rostopic list | grep -q "^${ODOM_TOPIC}$"; then
        print_error "Odometry topic $ODOM_TOPIC does not exist!"
        ODOM_HZ=0
        return
    fi
    
    print_info "Sampling for ${SAMPLE_DURATION} seconds..."
    ODOM_HZ=$(get_topic_hz "$ODOM_TOPIC" "$SAMPLE_DURATION")
    
    if [ "$ODOM_HZ" == "0" ]; then
        print_warning "No odometry data or frequency too low"
    else
        print_success "Odometry frequency: ${ODOM_HZ} Hz"
        
        if (( $(echo "$ODOM_HZ < 10" | bc -l) )); then
            print_warning "Odometry frequency is low, consider increasing odom_timeout_ms"
        elif (( $(echo "$ODOM_HZ > 50" | bc -l) )); then
            print_success "Odometry frequency is good, can use higher control frequency"
        fi
    fi
}

# ============================================================================
# Analyze trajectory input
# ============================================================================
analyze_trajectory() {
    print_header "Step 3: Analyze Trajectory ($TRAJ_TOPIC)"
    
    if ! rostopic list | grep -q "^${TRAJ_TOPIC}$"; then
        print_warning "Trajectory topic $TRAJ_TOPIC does not exist (network may not be running)"
        TRAJ_HZ=0
        TRAJ_POINTS=8
        TRAJ_DT=0.1
        HAS_VELOCITY=false
        return
    fi
    
    print_info "Sampling for ${SAMPLE_DURATION} seconds..."
    TRAJ_HZ=$(get_topic_hz "$TRAJ_TOPIC" "$SAMPLE_DURATION")
    
    if [ "$TRAJ_HZ" == "0" ]; then
        print_warning "No trajectory data"
        TRAJ_POINTS=8
        TRAJ_DT=0.1
        HAS_VELOCITY=false
    else
        print_success "Trajectory publish rate: ${TRAJ_HZ} Hz"
        
        print_info "Getting trajectory details..."
        local traj_msg=$(timeout 2s rostopic echo -n 1 "$TRAJ_TOPIC" 2>/dev/null || true)
        
        if [ -n "$traj_msg" ]; then
            TRAJ_POINTS=$(echo "$traj_msg" | grep -c "x:" 2>/dev/null || echo "8")
            TRAJ_DT=$(echo "$traj_msg" | grep -oP 'dt_sec: \K[0-9.]+' | head -1 || echo "0.1")
            
            [ -z "$TRAJ_DT" ] && TRAJ_DT="0.1"
            [ "$TRAJ_POINTS" -eq 0 ] && TRAJ_POINTS=8
            
            print_success "Trajectory points: $TRAJ_POINTS"
            print_success "Trajectory dt: ${TRAJ_DT} sec"
            
            if echo "$traj_msg" | grep -q "velocities"; then
                print_success "Trajectory contains velocity info (soft head)"
                HAS_VELOCITY=true
            else
                print_warning "Trajectory has no velocity info (hard head only)"
                HAS_VELOCITY=false
            fi
        else
            TRAJ_POINTS=8
            TRAJ_DT=0.1
            HAS_VELOCITY=false
        fi
    fi
}

# ============================================================================
# Analyze IMU
# ============================================================================
analyze_imu() {
    print_header "Step 4: Analyze IMU ($IMU_TOPIC)"
    
    if ! rostopic list | grep -q "^${IMU_TOPIC}$"; then
        print_warning "IMU topic $IMU_TOPIC does not exist"
        IMU_HZ=0
        HAS_IMU=false
        return
    fi
    
    print_info "Sampling for ${SAMPLE_DURATION} seconds..."
    IMU_HZ=$(get_topic_hz "$IMU_TOPIC" "$SAMPLE_DURATION")
    
    if [ "$IMU_HZ" == "0" ]; then
        print_warning "No IMU data"
        HAS_IMU=false
    else
        print_success "IMU frequency: ${IMU_HZ} Hz"
        HAS_IMU=true
    fi
}

# ============================================================================
# Analyze chassis characteristics from odometry
# ============================================================================
analyze_chassis() {
    print_header "Step 5: Analyze Chassis Characteristics"
    
    # Initialize defaults
    MAX_VX=0.5
    MAX_WZ=1.0
    MAX_AX=0.5
    MAX_ALPHA=1.0
    ODOM_JITTER_MS=0
    ODOM_LATENCY_MS=0
    
    if [ "$ODOM_HZ" == "0" ]; then
        print_warning "No odometry data, using default chassis parameters"
        return
    fi
    
    print_info "Analyzing chassis from odometry data..."
    
    # Get velocity statistics from odometry (collect more samples)
    local odom_data=$(timeout 5s rostopic echo -n 100 "$ODOM_TOPIC" 2>/dev/null)
    
    if [ -n "$odom_data" ]; then
        # Extract linear velocities
        local vx_values=$(echo "$odom_data" | grep -A 3 "linear:" | grep "x:" | awk '{print $2}')
        local wz_values=$(echo "$odom_data" | grep -A 3 "angular:" | grep "z:" | awk '{print $2}')
        
        if [ -n "$vx_values" ]; then
            MAX_VX=$(echo "$vx_values" | awk 'BEGIN{max=0} {v=($1>0?$1:-$1); if(v>max)max=v} END{print max}')
            [ -z "$MAX_VX" ] || [ "$MAX_VX" == "0" ] && MAX_VX=0.5
            print_success "Observed max linear velocity: ${MAX_VX} m/s"
        fi
        
        if [ -n "$wz_values" ]; then
            MAX_WZ=$(echo "$wz_values" | awk 'BEGIN{max=0} {v=($1>0?$1:-$1); if(v>max)max=v} END{print max}')
            [ -z "$MAX_WZ" ] || [ "$MAX_WZ" == "0" ] && MAX_WZ=1.0
            print_success "Observed max angular velocity: ${MAX_WZ} rad/s"
        fi
    else
        print_warning "Could not analyze chassis, using defaults"
    fi
    
    # Estimate acceleration and alpha from velocity changes (simplified)
    # In practice, these should be measured with chassis tests
    MAX_AX=$(echo "scale=2; $MAX_VX * 1.0" | bc 2>/dev/null || echo "0.5")
    MAX_ALPHA=$(echo "scale=2; $MAX_WZ * 1.0" | bc 2>/dev/null || echo "1.0")
    print_info "Estimated max acceleration: ${MAX_AX} m/s^2"
    print_info "Estimated max alpha: ${MAX_ALPHA} rad/s^2"
    
    # Analyze odom jitter and latency
    print_info "Analyzing odometry timing..."
    local hz_output=$(timeout 3s rostopic hz "$ODOM_TOPIC" 2>&1)
    
    # Extract jitter (standard deviation)
    ODOM_JITTER_MS=$(echo "$hz_output" | grep -oP 'std dev: \K[0-9.]+' | head -1 || echo "0")
    if [ -n "$ODOM_JITTER_MS" ] && [ "$ODOM_JITTER_MS" != "0" ]; then
        # Convert to ms
        ODOM_JITTER_MS=$(echo "scale=1; $ODOM_JITTER_MS * 1000" | bc 2>/dev/null || echo "0")
        print_info "Odometry jitter: ${ODOM_JITTER_MS} ms"
    else
        ODOM_JITTER_MS=0
    fi
    
    # Estimate latency from delay tool (if available)
    local delay_output=$(timeout 2s rostopic delay "$ODOM_TOPIC" 2>&1 | head -5)
    ODOM_LATENCY_MS=$(echo "$delay_output" | grep -oP 'average delay: \K[0-9.]+' | head -1 || echo "0")
    if [ -n "$ODOM_LATENCY_MS" ] && [ "$ODOM_LATENCY_MS" != "0" ]; then
        ODOM_LATENCY_MS=$(echo "scale=1; $ODOM_LATENCY_MS * 1000" | bc 2>/dev/null || echo "0")
        print_info "Odometry latency: ${ODOM_LATENCY_MS} ms"
    else
        ODOM_LATENCY_MS=0
    fi
}

# ============================================================================
# Calculate recommended parameters
# ============================================================================
calculate_recommendations() {
    print_header "Step 6: Calculate Recommended Parameters"
    
    # Default values
    RECOMMENDED_CTRL_FREQ=20
    RECOMMENDED_ODOM_TIMEOUT=500
    RECOMMENDED_TRAJ_TIMEOUT=1000
    RECOMMENDED_TRAJ_GRACE=500
    RECOMMENDED_IMU_TIMEOUT=-1
    RECOMMENDED_MPC_HORIZON=7
    RECOMMENDED_MPC_DT=0.1
    RECOMMENDED_V_MAX=0.5
    RECOMMENDED_OMEGA_MAX=1.0
    RECOMMENDED_A_MAX=0.5
    RECOMMENDED_ALPHA_MAX=1.0
    RECOMMENDED_LOOKAHEAD=0.5
    
    # Adjust control frequency based on odometry rate
    if [ "$ODOM_HZ" != "0" ]; then
        local odom_hz_int=$(printf "%.0f" "$ODOM_HZ")
        
        if [ "$odom_hz_int" -ge 100 ]; then
            RECOMMENDED_CTRL_FREQ=50
            RECOMMENDED_ODOM_TIMEOUT=100
        elif [ "$odom_hz_int" -ge 50 ]; then
            RECOMMENDED_CTRL_FREQ=40
            RECOMMENDED_ODOM_TIMEOUT=150
        elif [ "$odom_hz_int" -ge 20 ]; then
            RECOMMENDED_CTRL_FREQ=20
            RECOMMENDED_ODOM_TIMEOUT=300
        else
            RECOMMENDED_CTRL_FREQ=10
            RECOMMENDED_ODOM_TIMEOUT=500
        fi
        
        print_info "Based on odometry rate ${ODOM_HZ} Hz:"
        print_info "  Recommended ctrl_freq: ${RECOMMENDED_CTRL_FREQ} Hz"
        print_info "  Recommended odom_timeout: ${RECOMMENDED_ODOM_TIMEOUT} ms"
    fi
    
    # Adjust timeout based on trajectory rate
    if [ "$TRAJ_HZ" != "0" ] && [ "$TRAJ_HZ" != "" ]; then
        local traj_period_ms=$(echo "scale=0; 1000 / $TRAJ_HZ" | bc 2>/dev/null || echo "500")
        RECOMMENDED_TRAJ_TIMEOUT=$((traj_period_ms * 2))
        RECOMMENDED_TRAJ_GRACE=$((traj_period_ms))
        
        print_info "Based on trajectory rate ${TRAJ_HZ} Hz:"
        print_info "  Recommended traj_timeout: ${RECOMMENDED_TRAJ_TIMEOUT} ms"
        print_info "  Recommended traj_grace: ${RECOMMENDED_TRAJ_GRACE} ms"
    fi
    
    # Adjust MPC horizon based on trajectory points
    if [ "$TRAJ_POINTS" -gt 0 ]; then
        RECOMMENDED_MPC_HORIZON=$((TRAJ_POINTS - 1))
        [ "$RECOMMENDED_MPC_HORIZON" -lt 3 ] && RECOMMENDED_MPC_HORIZON=3
        [ "$RECOMMENDED_MPC_HORIZON" -gt 30 ] && RECOMMENDED_MPC_HORIZON=30
        
        print_info "Based on trajectory points ${TRAJ_POINTS}:"
        print_info "  Recommended MPC horizon: ${RECOMMENDED_MPC_HORIZON}"
    fi
    
    # Adjust MPC dt based on trajectory dt
    if [ -n "$TRAJ_DT" ] && [ "$TRAJ_DT" != "0" ]; then
        RECOMMENDED_MPC_DT="$TRAJ_DT"
        print_info "  Recommended MPC dt: ${RECOMMENDED_MPC_DT} sec"
    fi
    
    # IMU configuration
    if [ "$HAS_IMU" = true ]; then
        local imu_hz_int=$(printf "%.0f" "$IMU_HZ")
        [ "$imu_hz_int" -gt 0 ] && RECOMMENDED_IMU_TIMEOUT=$((1000 / imu_hz_int * 3))
        print_info "Based on IMU rate ${IMU_HZ} Hz:"
        print_info "  Recommended IMU timeout: ${RECOMMENDED_IMU_TIMEOUT} ms"
    else
        RECOMMENDED_IMU_TIMEOUT=-1
        print_info "No IMU, disabling IMU timeout detection"
    fi
    
    # Chassis constraints (with 80% safety margin)
    if [ -n "$MAX_VX" ] && [ "$MAX_VX" != "0" ]; then
        RECOMMENDED_V_MAX=$(echo "scale=2; $MAX_VX * 0.8" | bc 2>/dev/null || echo "0.5")
        print_info "Based on observed max velocity ${MAX_VX} m/s:"
        print_info "  Recommended v_max: ${RECOMMENDED_V_MAX} m/s (80% safety margin)"
    fi
    
    if [ -n "$MAX_WZ" ] && [ "$MAX_WZ" != "0" ]; then
        RECOMMENDED_OMEGA_MAX=$(echo "scale=2; $MAX_WZ * 0.8" | bc 2>/dev/null || echo "1.0")
        print_info "Based on observed max angular ${MAX_WZ} rad/s:"
        print_info "  Recommended omega_max: ${RECOMMENDED_OMEGA_MAX} rad/s (80% safety margin)"
    fi
    
    if [ -n "$MAX_AX" ] && [ "$MAX_AX" != "0" ]; then
        RECOMMENDED_A_MAX=$(echo "scale=2; $MAX_AX * 0.8" | bc 2>/dev/null || echo "0.5")
        print_info "Based on observed max accel ${MAX_AX} m/s^2:"
        print_info "  Recommended a_max: ${RECOMMENDED_A_MAX} m/s^2 (80% safety margin)"
    fi
    
    if [ -n "$MAX_ALPHA" ] && [ "$MAX_ALPHA" != "0" ]; then
        RECOMMENDED_ALPHA_MAX=$(echo "scale=2; $MAX_ALPHA * 0.8" | bc 2>/dev/null || echo "1.0")
        print_info "Based on observed max alpha ${MAX_ALPHA} rad/s^2:"
        print_info "  Recommended alpha_max: ${RECOMMENDED_ALPHA_MAX} rad/s^2 (80% safety margin)"
    fi
    
    # Lookahead based on velocity
    RECOMMENDED_LOOKAHEAD=$(echo "scale=2; $RECOMMENDED_V_MAX * 1.0" | bc 2>/dev/null || echo "0.5")
    [ "$(echo "$RECOMMENDED_LOOKAHEAD < 0.3" | bc)" -eq 1 ] && RECOMMENDED_LOOKAHEAD=0.3
    print_info "  Recommended lookahead: ${RECOMMENDED_LOOKAHEAD} m"
    
    # MPC timing
    local ctrl_period_ms=$((1000 / RECOMMENDED_CTRL_FREQ))
    RECOMMENDED_MPC_WARN=$((ctrl_period_ms * 40 / 100))
    RECOMMENDED_MPC_CRIT=$((ctrl_period_ms * 80 / 100))
    print_info "  MPC time warning: ${RECOMMENDED_MPC_WARN} ms"
    print_info "  MPC time critical: ${RECOMMENDED_MPC_CRIT} ms"
    
    # EKF noise adjustment based on odom jitter
    if [ -n "$ODOM_JITTER_MS" ] && [ "$ODOM_JITTER_MS" != "0" ]; then
        local noise_factor=$(echo "scale=2; 1.0 + $ODOM_JITTER_MS / 50.0" | bc 2>/dev/null || echo "1.0")
        print_info "Based on odom jitter ${ODOM_JITTER_MS} ms:"
        print_info "  EKF noise factor: ${noise_factor}x"
    fi
}

# ============================================================================
# Generate optimized configuration
# ============================================================================
generate_config() {
    print_header "Step 7: Generate Optimized Configuration"
    
    local output_file="${OUTPUT_FILE:-optimized_config.yaml}"
    local mpc_horizon_deg=$((RECOMMENDED_MPC_HORIZON / 2))
    [ "$mpc_horizon_deg" -lt 3 ] && mpc_horizon_deg=3
    
    # Determine alpha settings based on velocity info
    local alpha_disable="0.0"
    local alpha_recovery="0.0"
    local alpha_thresh="1"
    if [ "$HAS_VELOCITY" = true ]; then
        alpha_disable="0.1"
        alpha_recovery="0.3"
        alpha_thresh="5"
    fi
    
    # Calculate EKF noise based on odom jitter (if available)
    local odom_noise_factor="1.0"
    if [ -n "$ODOM_JITTER_MS" ] && [ "$ODOM_JITTER_MS" != "0" ]; then
        odom_noise_factor=$(echo "scale=2; 1.0 + $ODOM_JITTER_MS / 50.0" | bc 2>/dev/null || echo "1.0")
    fi
    local ekf_pos_noise=$(echo "scale=4; 0.01 * $odom_noise_factor" | bc 2>/dev/null || echo "0.01")
    local ekf_vel_noise=$(echo "scale=3; 0.1 * $odom_noise_factor" | bc 2>/dev/null || echo "0.1")
    local ekf_orient_noise=$(echo "scale=4; 0.05 * $odom_noise_factor" | bc 2>/dev/null || echo "0.05")
    
    # Calculate omega_max_low (50% of omega_max)
    local omega_max_low=$(echo "scale=2; ${RECOMMENDED_OMEGA_MAX:-1.0} * 0.5" | bc 2>/dev/null || echo "0.5")
    
    # Calculate alpha_max based on chassis (if available)
    local alpha_max="${RECOMMENDED_ALPHA_MAX:-1.0}"
    
    # Calculate emergency decel (1.5x of a_max)
    local emergency_decel=$(echo "scale=2; ${RECOMMENDED_A_MAX:-0.5} * 1.5" | bc 2>/dev/null || echo "1.0")
    
    # Calculate MPC recovery threshold
    local mpc_recovery_thresh=$((RECOMMENDED_MPC_WARN - 5))
    [ "$mpc_recovery_thresh" -lt 5 ] && mpc_recovery_thresh=5
    
    # Calculate TF timeout based on odom latency
    local tf_timeout=50
    if [ -n "$ODOM_LATENCY_MS" ] && [ "$ODOM_LATENCY_MS" != "0" ]; then
        tf_timeout=$(echo "scale=0; $ODOM_LATENCY_MS * 2 + 20" | bc 2>/dev/null || echo "50")
        [ "$tf_timeout" -lt 20 ] && tf_timeout=20
        [ "$tf_timeout" -gt 200 ] && tf_timeout=200
    fi
    
    # Calculate transition tau based on control frequency
    local transition_tau=$(echo "scale=3; 1.0 / ${RECOMMENDED_CTRL_FREQ} * 2" | bc 2>/dev/null || echo "0.1")
    
    cat > "$output_file" << EOF
# ============================================================================
# Auto-generated Optimized Configuration
# Generated: $(date)
# 
# Detection Results:
#   Odometry rate: ${ODOM_HZ:-0} Hz (jitter: ${ODOM_JITTER_MS:-0}ms, latency: ${ODOM_LATENCY_MS:-0}ms)
#   Trajectory rate: ${TRAJ_HZ:-0} Hz
#   Trajectory points: ${TRAJ_POINTS:-8}
#   Trajectory dt: ${TRAJ_DT:-0.1} sec
#   IMU available: ${HAS_IMU:-false}
#   Trajectory has velocity: ${HAS_VELOCITY:-false}
#   Observed max velocity: ${MAX_VX:-0.5} m/s
#   Observed max angular: ${MAX_WZ:-1.0} rad/s
#   Observed max accel: ${MAX_AX:-0.5} m/s^2
#   Observed max alpha: ${MAX_ALPHA:-1.0} rad/s^2
# ============================================================================

# System configuration
system:
  ctrl_freq: ${RECOMMENDED_CTRL_FREQ}
  platform: "differential"

# Topic configuration
topics:
  odom: "${ODOM_TOPIC}"
  imu: "${IMU_TOPIC}"
  trajectory: "${TRAJ_TOPIC}"
  emergency_stop: "/controller/emergency_stop"
  cmd_unified: "/cmd_unified"
  diagnostics: "/controller/diagnostics"
  state: "/controller/state"

# TF configuration
tf:
  source_frame: "base_link"
  target_frame: "odom"
  timeout_ms: ${tf_timeout}
  buffer_warmup_timeout_sec: 5.0
  buffer_warmup_interval_sec: 0.2
  expected_source_frames:
    - "base_link"
    - "base_footprint"
    - ""

# Timeout configuration
watchdog:
  odom_timeout_ms: ${RECOMMENDED_ODOM_TIMEOUT}
  traj_timeout_ms: ${RECOMMENDED_TRAJ_TIMEOUT}
  traj_grace_ms: ${RECOMMENDED_TRAJ_GRACE}
  imu_timeout_ms: ${RECOMMENDED_IMU_TIMEOUT}
  startup_grace_ms: 5000

# MPC configuration
mpc:
  horizon: ${RECOMMENDED_MPC_HORIZON}
  horizon_degraded: ${mpc_horizon_deg}
  dt: ${RECOMMENDED_MPC_DT}
  
  # MPC weights - tune based on tracking performance
  # Higher position weight = tighter tracking but may cause oscillation
  # Higher control weights = smoother but slower response
  weights:
    position: 10.0
    velocity: 1.0
    heading: 5.0
    control_accel: 0.2
    control_alpha: 0.2
  
  solver:
    nlp_max_iter: 50
    qp_solver: "PARTIAL_CONDENSING_HPIPM"
    integrator_type: "ERK"
    nlp_solver_type: "SQP_RTI"
  
  health_monitor:
    time_warning_thresh_ms: ${RECOMMENDED_MPC_WARN:-20}
    time_critical_thresh_ms: ${RECOMMENDED_MPC_CRIT:-40}
    time_recovery_thresh_ms: ${mpc_recovery_thresh}
    consecutive_warning_limit: 10
  
  fallback:
    lookahead_steps: 3
    heading_kp: 1.5
    default_speed_ratio: 0.5

# Velocity constraints (based on chassis analysis with 80% safety margin)
constraints:
  v_max: ${RECOMMENDED_V_MAX:-0.5}
  v_min: -0.2
  omega_max: ${RECOMMENDED_OMEGA_MAX:-1.0}
  omega_max_low: ${omega_max_low}
  a_max: ${RECOMMENDED_A_MAX:-0.5}
  alpha_max: ${alpha_max}
  v_low_thresh: 0.1

# Consistency configuration
consistency:
  kappa_thresh: 0.5
  v_dir_thresh: 0.8
  temporal_smooth_thresh: 0.5
  alpha_min: 0.1
  max_curvature: 10.0
  temporal_window_size: 10
  weights:
    kappa: 0.3
    velocity: 0.3
    temporal: 0.4

# Safety configuration
safety:
  v_stop_thresh: 0.05
  stopping_timeout: 5.0
  emergency_decel: ${emergency_decel}
  
  low_speed:
    threshold: 0.1
    omega_limit: ${omega_max_low}
  
  margins:
    velocity: 1.1
    acceleration: 1.5
  
  # Acceleration filter parameters
  accel_filter_window: 3
  accel_filter_alpha: 0.3
  accel_filter_warmup_alpha: 0.5
  accel_filter_warmup_period: 3
  
  state_machine:
    alpha_disable_thresh: ${alpha_disable}
    alpha_recovery_value: ${alpha_recovery}
    alpha_recovery_thresh: ${alpha_thresh}
    mpc_recovery_thresh: 5
    mpc_fail_window_size: 10
    mpc_fail_thresh: 3

# EKF configuration (noise adjusted based on odom jitter: ${ODOM_JITTER_MS:-0}ms)
ekf:
  use_odom_orientation_fallback: true
  imu_motion_compensation: ${HAS_IMU:-false}
  
  process_noise:
    position: ${ekf_pos_noise}
    velocity: ${ekf_vel_noise}
    orientation: ${ekf_orient_noise}
    angular_velocity: 0.1
  
  measurement_noise:
    odom_position: ${ekf_pos_noise}
    odom_velocity: ${ekf_vel_noise}
  
  adaptive:
    base_slip_thresh: 2.0
    slip_velocity_factor: 0.5
    slip_covariance_scale: 10.0
    stationary_covariance_scale: 0.1
    stationary_thresh: 0.05
  
  anomaly_detection:
    drift_thresh: 0.1
    jump_thresh: 0.5
    covariance_explosion_thresh: 1000.0
    innovation_anomaly_thresh: 10.0

# Transform configuration
transform:
  target_frame: "odom"
  source_frame: "base_link"
  timeout_ms: ${tf_timeout}
  fallback_duration_limit_ms: 500
  fallback_critical_limit_ms: 1000
  drift_estimation_enabled: false
  recovery_correction_enabled: true

# Transition configuration
transition:
  type: "exponential"
  tau: ${transition_tau}
  max_duration: 0.5
  completion_threshold: 0.95
  duration: 0.2

# Backup controller configuration (Pure Pursuit)
backup:
  lookahead_dist: ${RECOMMENDED_LOOKAHEAD:-0.5}
  min_lookahead: 0.3
  max_lookahead: 1.5
  lookahead_ratio: 0.5
  kp_heading: 1.5
  heading_error_thresh: 1.047
  pure_pursuit_angle_thresh: 1.047
  heading_control_angle_thresh: 1.571
  max_curvature: 5.0
  min_turn_speed: 0.1
  default_speed_ratio: 0.5
  min_distance_thresh: 0.1

# Tracking quality thresholds
tracking:
  lateral_thresh: 0.3
  longitudinal_thresh: 0.5
  heading_thresh: 0.5
  prediction_thresh: 0.5
  weights:
    lateral: 0.4
    longitudinal: 0.4
    heading: 0.2
  rating:
    excellent: 90
    good: 70
    fair: 50

# cmd_vel adapter configuration
cmd_vel_adapter:
  publish_rate: ${RECOMMENDED_CTRL_FREQ}.0
  joy_timeout: 0.5
  max_linear: ${RECOMMENDED_V_MAX:-0.5}
  max_angular: ${RECOMMENDED_OMEGA_MAX:-1.0}
  max_linear_accel: 0.0
  max_angular_accel: 0.0
  output_topic: "${CMD_VEL_TOPIC}"

# Diagnostics configuration
diagnostics:
  publish_rate: 10
EOF

    print_success "Configuration saved to: $output_file"
}

# ============================================================================
# Show diagnostics summary
# ============================================================================
show_summary() {
    print_header "Diagnostics Summary"
    
    echo -e "${CYAN}Sensor Status:${NC}"
    echo "  Odometry ($ODOM_TOPIC): ${ODOM_HZ:-0} Hz"
    echo "    Jitter: ${ODOM_JITTER_MS:-0} ms"
    echo "    Latency: ${ODOM_LATENCY_MS:-0} ms"
    echo "  Trajectory ($TRAJ_TOPIC): ${TRAJ_HZ:-0} Hz"
    echo "  IMU ($IMU_TOPIC): ${IMU_HZ:-0} Hz"
    echo ""
    
    echo -e "${CYAN}Trajectory Characteristics:${NC}"
    echo "  Points: ${TRAJ_POINTS:-unknown}"
    echo "  Time step: ${TRAJ_DT:-0.1} sec"
    echo "  Has velocity: ${HAS_VELOCITY:-unknown}"
    echo ""
    
    echo -e "${CYAN}Chassis Characteristics:${NC}"
    echo "  Observed max velocity: ${MAX_VX:-0.5} m/s"
    echo "  Observed max angular: ${MAX_WZ:-1.0} rad/s"
    echo "  Observed max accel: ${MAX_AX:-0.5} m/s^2"
    echo "  Observed max alpha: ${MAX_ALPHA:-1.0} rad/s^2"
    echo ""
    
    echo -e "${CYAN}Recommended Parameters:${NC}"
    echo "  Control frequency: ${RECOMMENDED_CTRL_FREQ} Hz"
    echo "  MPC horizon: ${RECOMMENDED_MPC_HORIZON}"
    echo "  MPC dt: ${RECOMMENDED_MPC_DT} sec"
    echo "  v_max: ${RECOMMENDED_V_MAX:-0.5} m/s"
    echo "  omega_max: ${RECOMMENDED_OMEGA_MAX:-1.0} rad/s"
    echo "  a_max: ${RECOMMENDED_A_MAX:-0.5} m/s^2"
    echo "  alpha_max: ${RECOMMENDED_ALPHA_MAX:-1.0} rad/s^2"
    echo "  Odom timeout: ${RECOMMENDED_ODOM_TIMEOUT} ms"
    echo "  Traj timeout: ${RECOMMENDED_TRAJ_TIMEOUT} ms"
    echo "  Traj grace: ${RECOMMENDED_TRAJ_GRACE} ms"
    echo "  Lookahead: ${RECOMMENDED_LOOKAHEAD:-0.5} m"
    echo "  MPC time warning: ${RECOMMENDED_MPC_WARN:-20} ms"
    echo "  MPC time critical: ${RECOMMENDED_MPC_CRIT:-40} ms"
    echo ""
}

# ============================================================================
# Show tuning guide
# ============================================================================
show_tuning_guide() {
    print_header "Tuning Guide"
    
    cat << 'EOF'
=== Key Parameters ===

1. Control Frequency (system.ctrl_freq)
   - Should be <= odometry rate
   - Typical: 20-50 Hz
   - Too high may cause MPC solver timeout

2. MPC Prediction Horizon (mpc.horizon)
   - Must be < trajectory points
   - Typical: 5-20
   - Larger = more computation time

3. MPC Time Step (mpc.dt)
   - Must match trajectory dt_sec
   - Typical: 0.05-0.2 sec

4. Velocity Constraints (constraints)
   - v_max: 80% of chassis max velocity
   - omega_max: 80% of chassis max angular velocity
   - omega_max_low: 50% of omega_max (for low speed)
   - a_max: 80% of chassis max acceleration
   - alpha_max: 80% of chassis max angular acceleration

5. MPC Weights (mpc.weights)
   - position: Position tracking accuracy (10-50)
   - velocity: Velocity tracking accuracy (1-10)
   - heading: Heading tracking accuracy (5-20)
   - control_accel: Control smoothness (0.1-1.0)
     Higher = smoother but slower response
   - control_alpha: Angular control smoothness (0.1-1.0)

6. Timeout Configuration
   - odom_timeout_ms: 2-3x odometry period
   - traj_timeout_ms: 2x trajectory period
   - traj_grace_ms: 1x trajectory period

7. EKF Noise Parameters
   - Higher odom jitter -> higher measurement noise
   - process_noise.velocity: affects state prediction
   - measurement_noise.odom_*: affects sensor trust
   - Noise factor = 1.0 + (jitter_ms / 50)

8. Backup Controller (Pure Pursuit)
   - lookahead_dist: ~1x max_velocity (in meters)
   - kp_heading: 1.0-2.0 for responsive turning
   - heading_error_thresh: ~60° (1.047 rad)

9. Consistency Check (consistency)
   - kappa_thresh: Curvature consistency (0.3-0.8)
   - v_dir_thresh: Velocity direction (0.6-0.9)
   - temporal_smooth_thresh: Temporal smoothness (0.3-0.7)
   - weights: Adjust based on trajectory quality

10. Safety Configuration (safety)
    - emergency_decel: 1.5x of a_max
    - low_speed.omega_limit: 50% of omega_max
    - state_machine.alpha_disable_thresh: 0.0 if no velocity info

11. Transform Configuration (transform)
    - timeout_ms: 2x odom_latency + 20ms
    - fallback_duration_limit_ms: 500ms typical
    - fallback_critical_limit_ms: 1000ms typical

12. Transition Configuration (transition)
    - tau: 2x control period for smooth transitions
    - max_duration: 0.5s typical

=== Tuning Steps ===

1. Run diagnostics to get baseline parameters
2. Start with conservative speed limits (50% of max)
3. Test trajectory tracking at low speed
4. Gradually increase speed limits
5. If tracking error is high:
   - Increase mpc.weights.position
   - Decrease mpc.dt (if trajectory allows)
6. If control is jittery:
   - Increase mpc.weights.control_accel
   - Increase mpc.weights.control_alpha
7. If response is slow:
   - Decrease mpc.weights.control_accel
   - Increase constraints.a_max

=== Performance Indicators ===

Good tracking:
  - Lateral error < 0.1m
  - Heading error < 0.2 rad
  - MPC solve time < 50% of control period

Warning signs:
  - Frequent MPC fallback to Pure Pursuit
  - High tracking error at turns
  - Oscillation or jitter in velocity

=== Configuration Coverage ===

This script generates ALL tunable parameters:
  ✓ System: ctrl_freq, platform
  ✓ Topics: odom, imu, trajectory, cmd_vel
  ✓ TF: source_frame, target_frame, timeout_ms
  ✓ Watchdog: all timeout values
  ✓ MPC: horizon, dt, weights, solver, health_monitor, fallback
  ✓ Constraints: v_max, omega_max, a_max, alpha_max, etc.
  ✓ Consistency: thresholds and weights
  ✓ Safety: thresholds, margins, state_machine, accel_filter
  ✓ EKF: noise parameters, adaptive, anomaly_detection
  ✓ Transform: timeout, fallback limits
  ✓ Transition: tau, duration, thresholds
  ✓ Backup: lookahead, kp_heading, angle thresholds
  ✓ Tracking: error thresholds, weights, rating
  ✓ cmd_vel_adapter: rate, limits, topics
EOF
}

# ============================================================================
# Main function
# ============================================================================
main() {
    echo ""
    echo -e "${GREEN}============================================================================${NC}"
    echo -e "${GREEN}  Controller Diagnostics and Tuning Tool${NC}"
    echo -e "${GREEN}============================================================================${NC}"
    echo ""
    
    check_ros_env
    analyze_odom
    analyze_trajectory
    analyze_imu
    analyze_chassis
    calculate_recommendations
    show_summary
    
    if [ "$AUTO_MODE" = true ]; then
        generate_config
    else
        echo ""
        read -p "Generate optimized config file? [y/N] " -n 1 -r
        echo ""
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            if [ -z "$OUTPUT_FILE" ]; then
                read -p "Output filename (default: optimized_config.yaml): " OUTPUT_FILE
                OUTPUT_FILE="${OUTPUT_FILE:-optimized_config.yaml}"
            fi
            generate_config
        fi
    fi
    
    show_tuning_guide
    
    echo ""
    print_success "Diagnostics complete!"
    echo ""
    echo "To use the generated config:"
    echo "  roslaunch controller_ros controller.launch config:=\$(pwd)/${OUTPUT_FILE:-optimized_config.yaml}"
    echo ""
    echo "For more detailed diagnostics (including chassis tests), use:"
    echo "  rosrun controller_ros full_diagnostics.py --test-chassis --output my_config.yaml"
    echo ""
}

main
