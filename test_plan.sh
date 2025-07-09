#!/bin/bash

# 12-Hour Progressive Testing Plan for Dynamic Pathfinding - EXPONENTIAL SCALING VERSION
# This script runs increasingly complex tests with proper density scaling

# Remove the strict error handling that causes immediate exit
set -uo pipefail

# Configuration
OUTPUT_DIR="test_results_$(date +%Y%m%d_%H%M%S)"
FINAL_CSV="combined_results_$(date +%Y%m%d_%H%M%S).csv"
LOG_FILE="test_log_$(date +%Y%m%d_%H%M%S).log"
START_TIME=$(date +%s)

# Test tracking
TOTAL_PHASES=4
CURRENT_PHASE=0
COMPLETED_TESTS=0
FAILED_TESTS=0
CSV_FILES=()

# Create output directory
mkdir -p "$OUTPUT_DIR"
cd "$OUTPUT_DIR" || exit

# Initialize CSV header file
echo "simulation_id,algorithm,grid_size,num_walls,num_obstacles,success,total_moves,optimal_path_length,route_efficiency,execution_time_ms,a_star_calls,d_star_calls,average_find_path_time_ns,total_pathfinding_calls" > "$FINAL_CSV"

# Logging function
log() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG_FILE"
}

# Progress tracking
update_progress() {
    local phase=$1
    local description=$2
    CURRENT_PHASE=$phase
    log "=== PHASE $phase/$TOTAL_PHASES: $description ==="
    log "Elapsed time: $((($(date +%s) - START_TIME) / 3600))h $((($(date +%s) - START_TIME) % 3600 / 60))m"
}

# Test execution function with better error handling
run_test() {
    local test_name=$1
    local output_file=$2
    shift 2
    local args=("$@")

    log "Starting test: $test_name"
    log "Command: cargo run --release -- ${args[*]}"

    # Run the test and capture exit code
    cargo run --release -- "${args[@]}" 2>&1 | tee -a "$LOG_FILE"
    local exit_code=$?

    if [[ $exit_code -eq 0 ]]; then
        if [[ -f "$output_file" ]]; then
            CSV_FILES+=("$output_file")
            local lines
            lines=$(wc -l < "$output_file")
            log "✓ Test completed: $test_name ($lines lines)"
            ((COMPLETED_TESTS++))
        else
            log "✗ Test failed: $test_name (no output file generated)"
            ((FAILED_TESTS++))
        fi
    else
        log "✗ Test failed with exit code $exit_code: $test_name"
        ((FAILED_TESTS++))
    fi

    return 0
}

# Cleanup and combine function
# shellcheck disable=SC2317  # This function is called via trap
cleanup_and_combine() {
    log "=== CLEANUP AND COMBINATION PHASE ==="

    # Count total lines across all CSV files
    local total_lines=0
    local valid_files=0

    for file in "${CSV_FILES[@]}"; do
        if [[ -f "$file" ]]; then
            local lines
            lines=$(wc -l < "$file")
            total_lines=$((total_lines + lines))
            ((valid_files++))
            log "File: $file ($lines lines)"
        fi
    done

    log "Found $valid_files valid CSV files with $total_lines total lines"

    # Combine CSV files
    if [[ ${#CSV_FILES[@]} -gt 0 ]]; then
        log "Combining CSV files into $FINAL_CSV"

        # Add data from all files (skip headers)
        for file in "${CSV_FILES[@]}"; do
            if [[ -f "$file" ]]; then
                tail -n +2 "$file" >> "$FINAL_CSV" 2>/dev/null || true
            fi
        done

        local final_lines
        final_lines=$(wc -l < "$FINAL_CSV")
        log "✓ Combined file created: $FINAL_CSV ($final_lines lines)"
    else
        log "✗ No CSV files to combine"
    fi

    # Generate summary report
    local end_time
    end_time=$(date +%s)
    local duration=$((end_time - START_TIME))
    local hours=$((duration / 3600))
    local minutes=$(((duration % 3600) / 60))

    log "=== FINAL SUMMARY ==="
    log "Total runtime: ${hours}h ${minutes}m"
    log "Completed phases: $CURRENT_PHASE/$TOTAL_PHASES"
    log "Successful tests: $COMPLETED_TESTS"
    log "Failed tests: $FAILED_TESTS"
    log "CSV files generated: ${#CSV_FILES[@]}"
    log "Total data lines: $total_lines"
    log "Final combined file: $FINAL_CSV"

    # Create summary file
    cat > "test_summary.txt" << EOF
Dynamic Pathfinding Test Summary - EXPONENTIAL SCALING
======================================================
Date: $(date)
Total Runtime: ${hours}h ${minutes}m
Completed Phases: $CURRENT_PHASE/$TOTAL_PHASES
Successful Tests: $COMPLETED_TESTS
Failed Tests: $FAILED_TESTS
CSV Files Generated: ${#CSV_FILES[@]}
Total Data Lines: $total_lines
Final Combined File: $FINAL_CSV

Test Files Generated:
$(for file in "${CSV_FILES[@]}"; do echo "  - $file"; done)
EOF

    log "Test summary written to test_summary.txt"
}

# Set up signal handling for cleanup
trap cleanup_and_combine EXIT INT TERM

# Start testing
log "Starting 12-hour EXPONENTIAL SCALING progressive testing plan"
log "Working directory: $(pwd)"
log "Target duration: 12 hours"

# PHASE 1: 60x60 Grids - ~2 hours
update_progress 1 "60x60 Grids with Medium and High Obstacle Scaling"

# 60x60 grid (3,600 squares) - Medium obstacles: 10-20% = 360-720
run_test "Grid_60x60_Medium_Obstacles" "phase1_60x60_med.csv" \
    --batch-mode --algorithm all --grid-size 60 \
    --min-walls 180 --max-walls 360 --min-obstacles 360 --max-obstacles 720 \
    --num-simulations 40 --timeout-seconds 1800 --output-file "phase1_60x60_med.csv" --quiet --no-visualization

# 60x60 grid (3,600 squares) - High obstacles: 40-50% = 1,440-1,800
run_test "Grid_60x60_High_Obstacles" "phase1_60x60_high.csv" \
    --batch-mode --algorithm all --grid-size 60 \
    --min-walls 360 --max-walls 720 --min-obstacles 1440 --max-obstacles 1800 \
    --num-simulations 40 --timeout-seconds 1800 --output-file "phase1_60x60_high.csv" --quiet --no-visualization

update_progress 2 "Large Grids with Exponential Obstacle Scaling"

# 120x120 grid (14,400 squares) - Low obstacles: 5-10% = 720-1,440
run_test "Grid_120x120_Low_Obstacles" "phase2_120x120_low.csv" \
    --batch-mode --algorithm all --grid-size 120 \
    --min-walls 360 --max-walls 720 --min-obstacles 720 --max-obstacles 1440 \
    --num-simulations 50 --timeout-seconds 3600 --output-file "phase2_120x120_low.csv" --quiet --no-visualization

# 120x120 grid (14,400 squares) - Medium obstacles: 10-20% = 1,440-2,880
run_test "Grid_120x120_Medium_Obstacles" "phase2_120x120_med.csv" \
    --batch-mode --algorithm all --grid-size 120 \
    --min-walls 720 --max-walls 1440 --min-obstacles 1440 --max-obstacles 2880 \
    --num-simulations 50 --timeout-seconds 3600 --output-file "phase2_120x120_med.csv" --quiet --no-visualization

# 120x120 grid (14,400 squares) - High obstacles: 20-30% = 2,880-4,320
run_test "Grid_120x120_High_Obstacles" "phase2_120x120_high.csv" \
    --batch-mode --algorithm all --grid-size 120 \
    --min-walls 1440 --max-walls 2160 --min-obstacles 2880 --max-obstacles 4320 \
    --num-simulations 50 --timeout-seconds 3600 --output-file "phase2_120x120_high.csv" --quiet --no-visualization

# 120x120 grid (14,400 squares) - Very high obstacles: 30-40% = 4,320-5,760
run_test "Grid_120x120_Very_High_Obstacles" "phase2_120x120_veryhigh.csv" \
    --batch-mode --algorithm all --grid-size 120 \
    --min-walls 2160 --max-walls 2880 --min-obstacles 4320 --max-obstacles 5760 \
    --num-simulations 40 --timeout-seconds 3600 --output-file "phase2_120x120_veryhigh.csv" --quiet --no-visualization

# PHASE 3: Massive Grids (180x180 to 240x240) - ~5 hours
update_progress 3 "Massive Grids with Exponential Obstacle Scaling"

# 180x180 grid (32,400 squares) - Moderate obstacles: 10-20% = 3,240-6,480
run_test "Grid_180x180_Moderate_Obstacles" "phase3_180x180_mod.csv" \
    --batch-mode --algorithm all --grid-size 180 \
    --min-walls 1620 --max-walls 3240 --min-obstacles 3240 --max-obstacles 6480 \
    --num-simulations 40 --timeout-seconds 5400 --output-file "phase3_180x180_mod.csv" --quiet --no-visualization

# 200x200 grid (40,000 squares) - High obstacles: 30-40% = 12,000-16,000
run_test "Grid_200x200_High_Obstacles" "phase3_200x200_high.csv" \
    --batch-mode --algorithm all --grid-size 200 \
    --min-walls 2000 --max-walls 4000 --min-obstacles 12000 --max-obstacles 16000 \
    --num-simulations 32 --timeout-seconds 5400 --output-file "phase3_200x200_high.csv" --quiet --no-visualization

# 240x240 grid (57,600 squares) - Very high obstacles: 50-60% = 28,800-34,560
run_test "Grid_240x240_Very_High_Obstacles" "phase3_240x240_veryhigh.csv" \
    --batch-mode --algorithm all --grid-size 240 \
    --min-walls 5760 --max-walls 11520 --min-obstacles 28800 --max-obstacles 34560 \
    --num-simulations 24 --timeout-seconds 5400 --output-file "phase3_240x240_veryhigh.csv" --quiet --no-visualization

# PHASE 4: Extreme Stress Tests - ~1 hour
update_progress 4 "Extreme Stress Tests with Ultra-High Density"

# 150x150 grid (22,500 squares) - Extreme density: 50-60% = 11,250-13,500
run_test "Stress_150x150_Extreme_Density" "phase4_stress_150x150.csv" \
    --batch-mode --algorithm all --grid-size 150 \
    --min-walls 2250 --max-walls 3375 --min-obstacles 11250 --max-obstacles 13500 \
    --num-simulations 60 --timeout-seconds 3600 --output-file "phase4_stress_150x150.csv" --quiet --no-visualization

# 180x180 grid (32,400 squares) - Extreme density: 50-60% = 16,200-19,440
run_test "Stress_180x180_Extreme_Density" "phase4_stress_180x180.csv" \
    --batch-mode --algorithm all --grid-size 180 \
    --min-walls 3240 --max-walls 4860 --min-obstacles 16200 --max-obstacles 19440 \
    --num-simulations 40 --timeout-seconds 5400 --output-file "phase4_stress_180x180.csv" --quiet --no-visualization

# 220x220 grid (48,400 squares) - Ultra extreme density: 65% = 31,460 obstacles
run_test "Stress_220x220_Ultra_Extreme" "phase4_stress_220x220.csv" \
    --batch-mode --algorithm all --grid-size 220 \
    --min-walls 4840 --max-walls 7260 --min-obstacles 31460 --max-obstacles 31460 \
    --num-simulations 30 --timeout-seconds 5400 --output-file "phase4_stress_220x220.csv" --quiet --no-visualization

# Algorithm-specific ultra-dense tests
# D* Lite specific test - 160x160 (25,600 squares) - 70% density = 17,920 obstacles
run_test "Stress_DStar_Ultra_Dense" "phase4_stress_dstar_ultra.csv" \
    --batch-mode --algorithm d_star_lite --grid-size 160 \
    --min-walls 2560 --max-walls 3840 --min-obstacles 17920 --max-obstacles 17920 \
    --num-simulations 80 --timeout-seconds 3600 --output-file "phase4_stress_dstar_ultra.csv" --quiet --no-visualization

# A* specific test - 160x160 (25,600 squares) - 70% density = 17,920 obstacles
run_test "Stress_AStar_Ultra_Dense" "phase4_stress_astar_ultra.csv" \
    --batch-mode --algorithm a_star --grid-size 160 \
    --min-walls 2560 --max-walls 3840 --min-obstacles 17920 --max-obstacles 17920 \
    --num-simulations 80 --timeout-seconds 3600 --output-file "phase4_stress_astar_ultra.csv" --quiet --no-visualization


log "=== ALL PHASES COMPLETED ==="
log "Proceeding to cleanup and combination..."

# The cleanup_and_combine function will be called automatically via trap
exit 0
