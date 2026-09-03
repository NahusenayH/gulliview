#!/bin/bash
#
# GulliView thermal stress test.
# Runs four sequential 30-minute tests with 1, 2, 3, and 4 cameras.
# Samples CPU package temperature and fan speed every 10 seconds.
# Cools down 10 minutes between tests.
#
# Run from the build directory:
#   cd ~/Gulliview_Intern/gulliview_multithreading_as_is/build
#   sudo -E bash thermal_test.sh
#
# All output goes to logs/thermal_YYYYMMDD_HHMMSS/
# The final summary.txt in that directory is what you look at first when back.

set -u

# --- Config -----------------------------------------------------------------
RUN_MINUTES=30            # duration of each camera-count test
COOLDOWN_MINUTES=10       # idle wait between tests (lets CPU cool back down)
SAMPLE_INTERVAL_SEC=10    # temperature sampling frequency
CAMERA_COUNTS=(1 2 3 4)   # tests to run, in order

# --- Setup ------------------------------------------------------------------
if [ "$(id -u)" -ne 0 ]; then
    echo "ERROR: this script needs sudo (GulliView requires it). Run with: sudo -E bash $0"
    exit 1
fi

if [ ! -x "./GulliView" ]; then
    echo "ERROR: ./GulliView not found. Run this script from the build directory."
    exit 1
fi

# Make sure sensors work; if not, warn and continue anyway (test still runs).
if ! command -v sensors >/dev/null 2>&1; then
    echo "WARNING: 'sensors' not installed. Install with: sudo apt install lm-sensors"
    echo "Continuing anyway; temperature logs will be empty."
    HAVE_SENSORS=0
else
    modprobe coretemp 2>/dev/null || true
    HAVE_SENSORS=1
fi

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTDIR="logs/thermal_${TIMESTAMP}"
mkdir -p "$OUTDIR"

SUMMARY="$OUTDIR/summary.txt"

echo "GulliView thermal test starting at $(date)"                | tee "$SUMMARY"
echo "Output directory: $OUTDIR"                                 | tee -a "$SUMMARY"
echo "Runs: ${CAMERA_COUNTS[*]} cameras, ${RUN_MINUTES} min each"| tee -a "$SUMMARY"
echo "Cooldown: ${COOLDOWN_MINUTES} min between runs"            | tee -a "$SUMMARY"
echo ""                                                          | tee -a "$SUMMARY"

# --- Helpers ----------------------------------------------------------------

# Extract just the package temperature number (in whole degrees C).
get_pkg_temp() {
    if [ "$HAVE_SENSORS" = "1" ]; then
        sensors 2>/dev/null | awk '/Package id 0:/ {gsub(/[+°C]/,"",$4); print int($4); exit}'
    else
        echo ""
    fi
}

# Extract fan RPM.
get_fan_rpm() {
    if [ "$HAVE_SENSORS" = "1" ]; then
        sensors 2>/dev/null | awk '/cpu_fan:/ {print $2; exit}'
    else
        echo ""
    fi
}

# Clean stale shared memory / semaphores from previous runs (prevents startup errors).
clean_ipc() {
    rm -f /dev/shm/shared_memory* /dev/shm/sem.my_semaphore* 2>/dev/null
}

# Sample sensors every N seconds into a CSV file until stop file exists.
# Runs in the background alongside GulliView.
sensor_sampler() {
    local outfile="$1"
    local stopfile="$2"
    echo "timestamp,elapsed_sec,pkg_temp_c,fan_rpm" > "$outfile"
    local start=$(date +%s)
    while [ ! -f "$stopfile" ]; do
        local now=$(date +%s)
        local elapsed=$((now - start))
        local t=$(get_pkg_temp)
        local f=$(get_fan_rpm)
        echo "$(date +%H:%M:%S),$elapsed,$t,$f" >> "$outfile"
        sleep "$SAMPLE_INTERVAL_SEC"
    done
}

# Analyse a completed run's temperature log and append stats to summary.
analyse_run() {
    local ncam="$1"
    local gulliview_log="$2"
    local sensor_log="$3"
    local duration_sec="$4"

    echo "----- Results for ${ncam} camera(s) --------------------------------" | tee -a "$SUMMARY"
    echo "Actual duration: $((duration_sec / 60)) min $((duration_sec % 60)) sec" | tee -a "$SUMMARY"

    # Temperature statistics
    if [ -s "$sensor_log" ] && [ "$HAVE_SENSORS" = "1" ]; then
        awk -F, 'NR>1 && $3 != "" {
            t=$3+0
            if (min=="" || t<min) min=t
            if (t>max) max=t
            sum+=t; n++
        } END {
            if (n>0) printf "Package temp:  min=%d°C  max=%d°C  avg=%d°C  samples=%d\n", min, max, sum/n, n
            else print "Package temp:  no samples captured"
        }' "$sensor_log" | tee -a "$SUMMARY"

        awk -F, 'NR>1 && $4 != "" {
            f=$4+0
            if (min=="" || f<min) min=f
            if (f>max) max=f
            sum+=f; n++
        } END {
            if (n>0) printf "Fan speed:     min=%d RPM  max=%d RPM  avg=%d RPM\n", min, max, sum/n
        }' "$sensor_log" | tee -a "$SUMMARY"
    else
        echo "Temperature data unavailable" | tee -a "$SUMMARY"
    fi

    # Application-level warnings
    local timeouts=0
    local corrupt=0
    if [ -f "$gulliview_log" ]; then
        timeouts=$(grep -c "select() timeout" "$gulliview_log" 2>/dev/null || echo 0)
        corrupt=$(grep -c "Corrupt JPEG" "$gulliview_log" 2>/dev/null || echo 0)
    fi
    echo "select() timeouts in log: $timeouts" | tee -a "$SUMMARY"
    echo "Corrupt JPEG warnings:    $corrupt" | tee -a "$SUMMARY"

    # Kernel-level errors during this run (approximate — dmesg is global)
    local thr=$(dmesg -T 2>/dev/null | tail -500 | grep -c "temperature is above threshold" || echo 0)
    local usb=$(dmesg -T 2>/dev/null | tail -500 | grep -c "uvcvideo.*status (-71)" || echo 0)
    local seg=$(dmesg -T 2>/dev/null | tail -500 | grep -c "GulliView.*segfault" || echo 0)
    echo "Recent dmesg throttle events (approx): $thr" | tee -a "$SUMMARY"
    echo "Recent dmesg USB -71 errors (approx):  $usb" | tee -a "$SUMMARY"
    echo "Recent dmesg GulliView segfaults:      $seg" | tee -a "$SUMMARY"
    echo "" | tee -a "$SUMMARY"
}

# --- Main loop --------------------------------------------------------------

RUN_SECONDS=$((RUN_MINUTES * 60))
COOLDOWN_SECONDS=$((COOLDOWN_MINUTES * 60))

# Record initial idle temperature as a baseline
idle_temp=$(get_pkg_temp)
idle_fan=$(get_fan_rpm)
echo "Baseline idle: pkg=${idle_temp}°C  fan=${idle_fan} RPM"     | tee -a "$SUMMARY"
echo ""                                                            | tee -a "$SUMMARY"

for ncam in "${CAMERA_COUNTS[@]}"; do
    echo "===================================================================" | tee -a "$SUMMARY"
    echo "TEST: ${ncam} camera(s) — starting at $(date)"                        | tee -a "$SUMMARY"
    echo "===================================================================" | tee -a "$SUMMARY"

    clean_ipc

    gulliview_log="$OUTDIR/gulliview_${ncam}cam.log"
    sensor_log="$OUTDIR/sensors_${ncam}cam.csv"
    stopfile="$OUTDIR/.stop_${ncam}"
    rm -f "$stopfile"

    # Start background sensor sampler
    sensor_sampler "$sensor_log" "$stopfile" &
    sampler_pid=$!

    # Launch GulliView in background so we can enforce the time limit
    ./GulliView -d "$ncam" -f tag36h11 -W 3840 -H 2160 \
                -V 192.168.50.255 -B -N my_semaphore -T shared_memory -n \
                > "$gulliview_log" 2>&1 &
    gv_pid=$!

    run_start=$(date +%s)
    echo "GulliView started (PID $gv_pid), running for ${RUN_MINUTES} minutes..."

    # Wait either the full duration OR until GulliView exits early
    elapsed=0
    while [ "$elapsed" -lt "$RUN_SECONDS" ]; do
        if ! kill -0 "$gv_pid" 2>/dev/null; then
            echo "WARNING: GulliView exited early after $elapsed sec" | tee -a "$SUMMARY"
            break
        fi
        sleep 5
        elapsed=$(( $(date +%s) - run_start ))
    done

    # Stop GulliView cleanly
    if kill -0 "$gv_pid" 2>/dev/null; then
        echo "Stopping GulliView..."
        kill -INT "$gv_pid" 2>/dev/null || true
        sleep 3
        kill -TERM "$gv_pid" 2>/dev/null || true
        sleep 2
        kill -KILL "$gv_pid" 2>/dev/null || true
    fi
    wait "$gv_pid" 2>/dev/null || true

    run_end=$(date +%s)
    duration=$((run_end - run_start))

    # Stop the sensor sampler
    touch "$stopfile"
    wait "$sampler_pid" 2>/dev/null || true

    clean_ipc

    # Summarise this run
    analyse_run "$ncam" "$gulliview_log" "$sensor_log" "$duration"

    # Cooldown (skip after the last run)
    if [ "$ncam" != "${CAMERA_COUNTS[-1]}" ]; then
        echo "Cooling down for ${COOLDOWN_MINUTES} minutes..." | tee -a "$SUMMARY"
        cool_start=$(date +%s)
        cool_log="$OUTDIR/cooldown_after_${ncam}cam.csv"
        stopfile_cool="$OUTDIR/.stop_cool_${ncam}"
        rm -f "$stopfile_cool"
        sensor_sampler "$cool_log" "$stopfile_cool" &
        cool_pid=$!
        sleep "$COOLDOWN_SECONDS"
        touch "$stopfile_cool"
        wait "$cool_pid" 2>/dev/null || true
        end_temp=$(get_pkg_temp)
        echo "End of cooldown: pkg=${end_temp}°C" | tee -a "$SUMMARY"
        echo "" | tee -a "$SUMMARY"
    fi
done

echo "===================================================================" | tee -a "$SUMMARY"
echo "All tests complete at $(date)"                                        | tee -a "$SUMMARY"
echo "===================================================================" | tee -a "$SUMMARY"
echo ""                                                                     | tee -a "$SUMMARY"
echo "Per-test files in $OUTDIR:"                                           | tee -a "$SUMMARY"
echo "  gulliview_Ncam.log     - full GulliView stdout/stderr"              | tee -a "$SUMMARY"
echo "  sensors_Ncam.csv       - temperature samples during the run"        | tee -a "$SUMMARY"
echo "  cooldown_after_Ncam.csv - temperature samples during cooldown"      | tee -a "$SUMMARY"
echo ""                                                                     | tee -a "$SUMMARY"
echo "Read summary.txt first — it has the comparative numbers."             | tee -a "$SUMMARY"
