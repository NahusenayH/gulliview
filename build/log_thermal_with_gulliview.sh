#!/bin/bash
#
# Log CPU thermal state at 1 Hz to a CSV, plus track GulliView issues.
# When GulliView exits, record the exit time and final temperature.
#
# Issues tracked (via dmesg and GulliView log if provided):
#   - select() timeouts (per camera)
#   - uvcvideo USB errors (status -71)
#   - CPU thermal throttling events
#   - Corrupt JPEG data
#   - GulliView segfaults / GP faults
#   - GulliView exit (clean or crash)
#
# Usage:
#   sudo ./log_thermal_with_gulliview.sh [run_label] [gulliview_log_path]
#
# Example:
#   # In one terminal:
#   sudo -E ./GulliView -d 4 ... 2>&1 | tee logs/live.log
#   # In another:
#   sudo ./log_thermal_with_gulliview.sh baseline logs/live.log
#
# Output:
#   ~/thermal_<label>_<timestamp>.csv          — 1 Hz thermal + issue-count trace
#   ~/thermal_<label>_<timestamp>_events.txt   — human-readable event log
#

RUN_LABEL="${1:-run}"
GV_LOG="${2:-}"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

# Detect the invoking user's home even under sudo
REAL_HOME="${SUDO_USER:+$(getent passwd "$SUDO_USER" | cut -d: -f6)}"
REAL_HOME="${REAL_HOME:-$HOME}"

LOG="${REAL_HOME}/thermal_${RUN_LABEL}_${TIMESTAMP}.csv"
EVENTS="${REAL_HOME}/thermal_${RUN_LABEL}_${TIMESTAMP}_events.txt"

modprobe coretemp 2>/dev/null || true

if ! command -v sensors >/dev/null 2>&1; then
    echo "ERROR: 'sensors' not found. Install with: sudo apt install lm-sensors"
    exit 1
fi

# --- CSV header (dynamic core count) --------------------------------------

CORE_HEADERS=$(sensors 2>/dev/null | awk '/^Core [0-9]+:/ {
    match($0, /Core ([0-9]+):/, arr)
    printf "core%s_temp_c,", arr[1]
} END {print ""}' | sed 's/,$//')

HEADER="timestamp,elapsed_sec,pkg_temp_c,${CORE_HEADERS},fan_rpm"
HEADER="${HEADER},throttle_events_total,uvcvideo_errors_total"
HEADER="${HEADER},select_timeouts_delta,corrupt_jpeg_delta,gv_segfaults_delta"
HEADER="${HEADER},dmesg_throttle_delta,dmesg_uvc_delta"
echo "$HEADER" > "$LOG"

# --- Baseline counts (so we log deltas per sample) ------------------------

BASE_THROTTLE=$(dmesg 2>/dev/null | grep -c "temperature is above threshold" || echo 0)
BASE_UVC=$(dmesg 2>/dev/null | grep -c "uvcvideo.*Non-zero status" || echo 0)
BASE_SEGV=$(dmesg 2>/dev/null | grep -c "GulliView.*segfault\|GulliView.*general protection" || echo 0)

BASE_SELECT_TO=0
BASE_CORRUPT_JPEG=0
if [ -n "$GV_LOG" ] && [ -f "$GV_LOG" ]; then
    BASE_SELECT_TO=$(grep -c "select() timeout" "$GV_LOG" 2>/dev/null || echo 0)
    BASE_CORRUPT_JPEG=$(grep -c "Corrupt JPEG" "$GV_LOG" 2>/dev/null || echo 0)
fi

PREV_THROTTLE=$BASE_THROTTLE
PREV_UVC=$BASE_UVC
PREV_SEGV=$BASE_SEGV
PREV_SELECT_TO=$BASE_SELECT_TO
PREV_CORRUPT=$BASE_CORRUPT_JPEG

# --- Event log helper -----------------------------------------------------

log_event() {
    local msg="$1"
    local now=$(date '+%Y-%m-%d %H:%M:%S')
    local pkg=$(sensors 2>/dev/null | awk '/Package id 0:/ {gsub(/[+°C]/,"",$4); print $4; exit}')
    echo "[$now] pkg=${pkg}°C  $msg" | tee -a "$EVENTS"
}

log_event "Logger started."
log_event "CSV: $LOG"
if [ -n "$GV_LOG" ]; then
    log_event "Watching GulliView log: $GV_LOG"
fi
log_event "Baseline dmesg counts: throttle=$BASE_THROTTLE uvc=$BASE_UVC segv=$BASE_SEGV"

# --- Find GulliView process -----------------------------------------------

GV_PID=$(pgrep -x GulliView | head -1)
if [ -n "$GV_PID" ]; then
    log_event "GulliView detected running (PID $GV_PID)."
    WATCH_GV=1
else
    log_event "GulliView not currently running. Waiting for it to start."
    WATCH_GV=0
fi

# --- Ctrl+C handler -------------------------------------------------------

STOP=0
trap 'STOP=1' INT TERM

# --- Main loop ------------------------------------------------------------

START_TIME=$(date +%s)
SAMPLE_COUNT=0

while [ "$STOP" -eq 0 ]; do
    NOW_EPOCH=$(date +%s)
    NOW_ISO=$(date '+%Y-%m-%d %H:%M:%S')
    ELAPSED=$((NOW_EPOCH - START_TIME))

    SNAPSHOT=$(sensors 2>/dev/null)
    PKG=$(echo "$SNAPSHOT" | awk '/Package id 0:/ {gsub(/[+°C]/,"",$4); print $4; exit}')
    CORE_TEMPS=$(echo "$SNAPSHOT" | awk '/^Core [0-9]+:/ {gsub(/[+°C]/,"",$3); printf "%s,", $3} END {print ""}' | sed 's/,$//')
    FAN=$(echo "$SNAPSHOT" | awk '/cpu_fan:/ {print $2; exit}')

    CUR_THROTTLE=$(dmesg 2>/dev/null | grep -c "temperature is above threshold")
    CUR_UVC=$(dmesg 2>/dev/null | grep -c "uvcvideo.*Non-zero status")
    CUR_SEGV=$(dmesg 2>/dev/null | grep -c "GulliView.*segfault\|GulliView.*general protection")

    DELTA_THROTTLE=$((CUR_THROTTLE - PREV_THROTTLE))
    DELTA_UVC=$((CUR_UVC - PREV_UVC))
    DELTA_SEGV=$((CUR_SEGV - PREV_SEGV))

    CUR_SELECT_TO=0
    CUR_CORRUPT=0
    if [ -n "$GV_LOG" ] && [ -f "$GV_LOG" ]; then
        CUR_SELECT_TO=$(grep -c "select() timeout" "$GV_LOG" 2>/dev/null || echo 0)
        CUR_CORRUPT=$(grep -c "Corrupt JPEG" "$GV_LOG" 2>/dev/null || echo 0)
    fi
    DELTA_SELECT_TO=$((CUR_SELECT_TO - PREV_SELECT_TO))
    DELTA_CORRUPT=$((CUR_CORRUPT - PREV_CORRUPT))

    TOTAL_THROTTLE=$((CUR_THROTTLE - BASE_THROTTLE))
    TOTAL_UVC=$((CUR_UVC - BASE_UVC))

    echo "${NOW_ISO},${ELAPSED},${PKG},${CORE_TEMPS},${FAN},${TOTAL_THROTTLE},${TOTAL_UVC},${DELTA_SELECT_TO},${DELTA_CORRUPT},${DELTA_SEGV},${DELTA_THROTTLE},${DELTA_UVC}" >> "$LOG"
    SAMPLE_COUNT=$((SAMPLE_COUNT + 1))

    if [ "$DELTA_SEGV" -gt 0 ]; then
        log_event "ISSUE: $DELTA_SEGV new GulliView segfault(s)/GP fault(s) in dmesg"
        dmesg 2>/dev/null | grep -E "GulliView.*(segfault|general protection)" | tail -"$DELTA_SEGV" | while read -r line; do
            echo "    $line" >> "$EVENTS"
        done
    fi
    if [ "$DELTA_THROTTLE" -gt 0 ]; then
        log_event "ISSUE: $DELTA_THROTTLE new thermal throttle event(s) (total this run: $TOTAL_THROTTLE)"
    fi
    if [ "$DELTA_UVC" -gt 0 ]; then
        log_event "ISSUE: $DELTA_UVC new uvcvideo USB error(s) (total this run: $TOTAL_UVC)"
    fi
    if [ "$DELTA_SELECT_TO" -gt 0 ] && [ -n "$GV_LOG" ]; then
        log_event "ISSUE: $DELTA_SELECT_TO new select() timeout(s) in GulliView log"
    fi
    if [ "$DELTA_CORRUPT" -gt 0 ] && [ -n "$GV_LOG" ]; then
        log_event "ISSUE: $DELTA_CORRUPT new Corrupt JPEG warning(s) in GulliView log"
    fi

    PREV_THROTTLE=$CUR_THROTTLE
    PREV_UVC=$CUR_UVC
    PREV_SEGV=$CUR_SEGV
    PREV_SELECT_TO=$CUR_SELECT_TO
    PREV_CORRUPT=$CUR_CORRUPT

    if [ "$WATCH_GV" -eq 1 ] && ! kill -0 "$GV_PID" 2>/dev/null; then
        log_event "GULLIVIEW EXITED (PID $GV_PID). Stopping logger."
        break
    fi

    if [ "$WATCH_GV" -eq 0 ] && [ $((SAMPLE_COUNT % 5)) -eq 0 ]; then
        NEW_PID=$(pgrep -x GulliView | head -1)
        if [ -n "$NEW_PID" ]; then
            log_event "GulliView started (PID $NEW_PID). Now watching for exit."
            GV_PID="$NEW_PID"
            WATCH_GV=1
        fi
    fi

    sleep 1
done

if [ "$STOP" -eq 1 ]; then
    log_event "Logger stopped by user (Ctrl+C). Samples collected: $SAMPLE_COUNT"
fi

echo ""
echo "=============================================================="
echo "Logging complete."
echo "  CSV:         $LOG"
echo "  Event log:   $EVENTS"
echo "  Samples:     $SAMPLE_COUNT"
DURATION=$((NOW_EPOCH - START_TIME))
echo "  Duration:    ${DURATION} sec ($((DURATION / 60)) min $((DURATION % 60)) sec)"
echo ""

PEAK=$(awk -F, 'NR>1 && $3 != "" {t=$3+0; if(t>max) max=t} END {print max}' "$LOG")
FIRST=$(awk -F, 'NR==2 && $3 != "" {print $3+0}' "$LOG")
LAST=$(awk -F, 'END {print $3+0}' "$LOG")
echo "  Start pkg temp:   ${FIRST}°C"
echo "  Peak pkg temp:    ${PEAK}°C"
echo "  Final pkg temp:   ${LAST}°C"
echo ""

SUM_SELECT=$(awk -F, 'NR>1 {s+=$(NF-4)} END {print s+0}' "$LOG")
SUM_CORRUPT=$(awk -F, 'NR>1 {s+=$(NF-3)} END {print s+0}' "$LOG")
SUM_SEGV=$(awk -F, 'NR>1 {s+=$(NF-2)} END {print s+0}' "$LOG")
SUM_THROT=$(awk -F, 'NR>1 {s+=$(NF-1)} END {print s+0}' "$LOG")
SUM_UVC=$(awk -F, 'NR>1 {s+=$NF} END {print s+0}' "$LOG")
echo "  Issue totals during run:"
echo "    select() timeouts:      ${SUM_SELECT}"
echo "    corrupt JPEGs:          ${SUM_CORRUPT}"
echo "    GulliView segfaults:    ${SUM_SEGV}"
echo "    thermal throttles:      ${SUM_THROT}"
echo "    uvcvideo USB errors:    ${SUM_UVC}"
echo "=============================================================="
