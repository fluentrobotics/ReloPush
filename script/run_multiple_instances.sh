#!/bin/bash

set -u

usage() {
    cat <<'EOF'
Usage:
  ./script/run_all_indices_integrated.sh <input_file> [--mode=f|d|u|o] [--lns-iters=N] [--num-robots=N] [--lns-reassign-only|--lns-task-reassign] [--visualize|--no-visualization] [--start-index=N] [--end-index=N] [--base-port=N] [--continue-on-error] [--mars-arg=ARG]...

Description:
  Runs every non-empty instance index in input/<input_file> using the integrated
  ReloPush-BOSS -> MARS workflow. For each index, the script:
  1. starts phastar_push_demo in integrated mode as a ZeroMQ server
  2. runs ReloPush-BOSS in integrated mode for that instance index
  3. waits for MARS to finish and stores logs

Arguments:
  <input_file>     File name relative to input/

Options:
  --mode=MODE            ReloPush mode: f, d, u, or o (default: f)
  --lns-iters=N          Default MARS LNS iterations (default: 10)
  --num-robots=N         MARS robot count to use, capped by predefined robots (default: 2)
  --lns-task-reassign    During LNS, allow task-order repair and robot reassignment (default)
  --lns-reassign-only    During LNS, preserve task order and reassign only robots
  --visualize            Enable MARS visualization
  --no-visualization     Disable MARS visualization (default)
  --start-index=N        First instance index to run
  --end-index=N          Last instance index to run, inclusive
  --base-port=N          Base TCP port for ZeroMQ endpoint (default: 5566)
  --continue-on-error    Continue to later indices if one run fails
  --mars-arg=ARG         Extra argument forwarded to phastar_push_demo

Example:
  ./script/run_all_indices_integrated.sh ReloPush-BOSS_13_objects.txt
EOF
}

if [ "$#" -lt 1 ]; then
    usage
    exit 1
fi

INPUT_FILE="$1"
shift 1

MODE="f"
LNS_ITERS=20
NUM_ROBOTS=2
LNS_MODE="lns-task-reassign"
ENABLE_VISUALIZATION=0

START_INDEX=""
END_INDEX=""
BASE_PORT=5566
CONTINUE_ON_ERROR=1
MARS_ARGS=()

for arg in "$@"; do
    case "$arg" in
        --mode=*)
            MODE="${arg#--mode=}"
            ;;
        --lns-iters=*)
            LNS_ITERS="${arg#--lns-iters=}"
            ;;
        --num-robots=*|--robot-count=*)
            NUM_ROBOTS="${arg#*=}"
            ;;
        --lns-reassign-only|--lns-preserve-task-sequence)
            LNS_MODE="lns-reassign-only"
            ;;
        --lns-task-reassign|--lns-allow-sequence-edits|--no-lns-reassign-only)
            LNS_MODE="lns-task-reassign"
            ;;
        --lns-mode=*)
            LNS_MODE="${arg#--lns-mode=}"
            ;;
        --visualize|--visualization)
            ENABLE_VISUALIZATION=1
            ;;
        --no-visualization)
            ENABLE_VISUALIZATION=0
            ;;
        --start-index=*)
            START_INDEX="${arg#--start-index=}"
            ;;
        --end-index=*)
            END_INDEX="${arg#--end-index=}"
            ;;
        --base-port=*)
            BASE_PORT="${arg#--base-port=}"
            ;;
        --continue-on-error)
            CONTINUE_ON_ERROR=1
            ;;
        --mars-arg=*)
            MARS_ARGS+=("${arg#--mars-arg=}")
            ;;
        --help|-h)
            usage
            exit 0
            ;;
        *)
            echo "[Error] Unknown option: $arg" >&2
            usage
            exit 1
            ;;
    esac
done

case "$MODE" in
    f|d|u|o)
        ;;
    *)
        echo "[Error] Invalid mode '$MODE'. Expected one of: f, d, u, o." >&2
        exit 1
        ;;
esac

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
INPUT_PATH="$REPO_ROOT/input/$INPUT_FILE"
RELOPUSH_BIN="$REPO_ROOT/build/ReloPush-BOSS"
MARS_BIN="$REPO_ROOT/build/MARS/phastar_push_demo"

if [ ! -f "$INPUT_PATH" ]; then
    echo "[Error] Input file not found: $INPUT_PATH" >&2
    exit 1
fi

if [ ! -x "$RELOPUSH_BIN" ]; then
    echo "[Error] Missing executable: $RELOPUSH_BIN" >&2
    exit 1
fi

if [ ! -x "$MARS_BIN" ]; then
    echo "[Error] Missing executable: $MARS_BIN" >&2
    exit 1
fi

sanitize_name() {
    local value="$1"
    value="${value// /_}"
    value="${value//\//_}"
    value="${value//:/_}"
    echo "$value"
}

ACTIVE_RUN_PIDS=()

cleanup() {
    if [ "${#ACTIVE_RUN_PIDS[@]}" -gt 0 ]; then
        for pid in "${ACTIVE_RUN_PIDS[@]}"; do
            if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
                kill "$pid" 2>/dev/null || true
                kill -- "-$pid" 2>/dev/null || true
            fi
        done
    fi
}

on_interrupt() {
    cleanup
    exit 130
}

trap cleanup EXIT
trap on_interrupt INT TERM

LOG_DIR="$REPO_ROOT/results/integrated_logs/$(sanitize_name "$INPUT_FILE")"
mkdir -p "$LOG_DIR"
ACTIVE_RUN_DIR=""

TOTAL_RUNS=0
PASSED_RUNS=0

VISUALIZATION_ARG="--no-visualization"
if [ "$ENABLE_VISUALIZATION" -eq 1 ]; then
    VISUALIZATION_ARG="--visualize"
fi

echo "[Config] input file: $INPUT_FILE"
echo "[Config] mode: $MODE"
echo "[Config] visualization: ${VISUALIZATION_ARG#--}"
echo "[Config] lns iterations: $LNS_ITERS"
echo "[Config] MARS robots: $NUM_ROBOTS"
echo "[Config] LNS mode: $LNS_MODE"
echo "[Config] base port: $BASE_PORT"
if [ -n "$START_INDEX" ]; then
    echo "[Config] start index: $START_INDEX"
fi
if [ -n "$END_INDEX" ]; then
    echo "[Config] end index: $END_INDEX"
fi
if [ "$CONTINUE_ON_ERROR" -eq 1 ]; then
    echo "[Config] continue on error: enabled"
else
    echo "[Config] continue on error: disabled"
fi
if [ "${#MARS_ARGS[@]}" -gt 0 ]; then
    echo "[Config] extra MARS args: ${MARS_ARGS[*]}"
fi
echo

monitor_run() {
    local mars_pid="$1"
    local relopush_pid="$2"
    local mars_log="$3"
    local relopush_log="$4"
    local last_mars_tail=""
    local last_relopush_tail=""

    while true; do
        local mars_alive=0
        local relopush_alive=0
        if kill -0 "$mars_pid" 2>/dev/null; then
            mars_alive=1
        fi
        if kill -0 "$relopush_pid" 2>/dev/null; then
            relopush_alive=1
        fi

        #printf '[Monitor] MARS: %s | ReloPush: %s\n' \
        #    "$([ "$mars_alive" -eq 1 ] && echo running || echo finished)" \
        #    "$([ "$relopush_alive" -eq 1 ] && echo running || echo finished)"

        local mars_tail=""
        local relopush_tail=""
        mars_tail="$(tail -n 1 "$mars_log" 2>/dev/null || true)"
        relopush_tail="$(tail -n 1 "$relopush_log" 2>/dev/null || true)"

        if [ -n "$mars_tail" ] && [ "$mars_tail" != "$last_mars_tail" ]; then
            printf '  [MARS] %s\n' "$mars_tail"
            last_mars_tail="$mars_tail"
        fi
        if [ -n "$relopush_tail" ] && [ "$relopush_tail" != "$last_relopush_tail" ]; then
            printf '  [ReloPush] %s\n' "$relopush_tail"
            last_relopush_tail="$relopush_tail"
        fi

        if [ "$mars_alive" -eq 0 ] && [ "$relopush_alive" -eq 0 ]; then
            break
        fi

        sleep 2
    done
}

cd "$REPO_ROOT"

INDEX=0
while IFS= read -r line || [ -n "$line" ]; do
    if [[ -z "$line" ]]; then
        INDEX=$((INDEX + 1))
        continue
    fi

    if [ -n "$START_INDEX" ] && [ "$INDEX" -lt "$START_INDEX" ]; then
        INDEX=$((INDEX + 1))
        continue
    fi

    if [ -n "$END_INDEX" ] && [ "$INDEX" -gt "$END_INDEX" ]; then
        break
    fi

    TOTAL_RUNS=$((TOTAL_RUNS + 1))

    ENDPOINT_PORT=$((BASE_PORT + INDEX))
    ENDPOINT="tcp://127.0.0.1:$ENDPOINT_PORT"
    PREFIX="$LOG_DIR/index_${INDEX}"
    MARS_LOG="${PREFIX}_mars.log"
    RELOPUSH_LOG="${PREFIX}_relopush.log"
    RUN_DIR="${PREFIX}_run"
    ACTIVE_RUN_DIR="$RUN_DIR"
    ACTIVE_RUN_PIDS=()
    mkdir -p "$RUN_DIR"
    MARS_CMD=(
        "$MARS_BIN"
        --integrated-mode
        "--handoff-endpoint=$ENDPOINT"
        "$VISUALIZATION_ARG"
        "--lns-iters=$LNS_ITERS"
        "--num-robots=$NUM_ROBOTS"
        "--no-visualize-relopush-plan"
        "--no-debug-vis"
    )
    case "$LNS_MODE" in
        lns-reassign-only|reassign-only|preserve-task-sequence)
            MARS_CMD+=(--lns-reassign-only)
            ;;
        lns-task-reassign|task-reassign|allow-sequence-edits)
            MARS_CMD+=(--lns-task-reassign)
            ;;
        *)
            echo "[Error] Invalid LNS mode '$LNS_MODE'. Expected lns-task-reassign or lns-reassign-only." >&2
            cleanup
            exit 1
            ;;
    esac
    if [ "${#MARS_ARGS[@]}" -gt 0 ]; then
        MARS_CMD+=("${MARS_ARGS[@]}")
    fi

    echo
    echo "[Run] file=$INPUT_FILE index=$INDEX mode=$MODE endpoint=$ENDPOINT"

    (
        "${MARS_CMD[@]}"
    ) > "$MARS_LOG" 2>&1 &
    MARS_PID=$!

    sleep 1

    (
        "$RELOPUSH_BIN" \
            "$INPUT_FILE" \
            "$INDEX" \
            "$MODE" \
            --integrated-mode \
            "--mars-endpoint=$ENDPOINT"
    ) > "$RELOPUSH_LOG" 2>&1 &
    RELOPUSH_PID=$!

    ACTIVE_RUN_PIDS=("$MARS_PID" "$RELOPUSH_PID")

    monitor_run "$MARS_PID" "$RELOPUSH_PID" "$MARS_LOG" "$RELOPUSH_LOG"

    wait "$RELOPUSH_PID"
    RELOPUSH_STATUS=$?
    wait "$MARS_PID"
    MARS_STATUS=$?
    ACTIVE_RUN_DIR=""
    ACTIVE_RUN_PIDS=()

    if [ "$RELOPUSH_STATUS" -eq 0 ] && [ "$MARS_STATUS" -eq 0 ]; then
        PASSED_RUNS=$((PASSED_RUNS + 1))
        echo "[OK] index=$INDEX completed"
    else
        echo "[Error] index=$INDEX failed (ReloPush=$RELOPUSH_STATUS, MARS=$MARS_STATUS)" >&2
        echo "[Error] Logs:"
        echo "  $RELOPUSH_LOG"
        echo "  $MARS_LOG"
        if [ "$CONTINUE_ON_ERROR" -ne 1 ]; then
            exit 1
        fi
    fi

    INDEX=$((INDEX + 1))
done < "$INPUT_PATH"

echo
echo "[Summary] completed $PASSED_RUNS / $TOTAL_RUNS requested runs"
echo "[Summary] logs saved in $LOG_DIR"
