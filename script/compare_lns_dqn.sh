#!/usr/bin/env bash
#
# LNS vs DQN allocation-search comparison over a range of instance indices.
#
# For each instance index it runs the DQN online search first (which also
# computes the greedy baseline). If the greedy allocation is infeasible the
# instance is skipped. Otherwise it also runs the LNS search, then records:
#   - the full stdout/stderr log per run (with a metadata header),
#   - a copy of the executed best plan (task_execution_log.csv -> *_plan.csv)
#     so the result can be visually replayed later,
#   - a row in summary.csv with all parameters + metrics.
#
# Runs are sequential on purpose: the planner writes shared files under
# results/index-logs, so concurrent outer runs would clash. The inner planning
# already uses multiple threads (--lns-threads).
#
# Usage: script/compare_lns_dqn.sh [start_index] [end_index]
set -u

# --- Configuration / parameters (recorded as metadata) ----------------------
START_INDEX="${1:-0}"
END_INDEX="${2:-22}"
SEED="${SEED:-1}"
ITERS="${ITERS:-10}"
THREADS="${THREADS:-5}"
INSTANCE="${INSTANCE:-ReloPush-BOSS_12_objects.txt}"
# DQN hyperparameters left at program defaults; recorded for provenance.
DQN_EPS_START="0.9"
DQN_EPS_END="0.1"
DQN_LR="0.05"

# --- Paths ------------------------------------------------------------------
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="$REPO_ROOT/build-release/MARS/phastar_push_demo"
SEQ_DIR="$REPO_ROOT/results/relopush-out"
PLAN_SRC="$REPO_ROOT/results/index-logs/task_execution_log.csv"
OUT_DIR="$REPO_ROOT/results/lns_vs_dqn"
SUMMARY="$OUT_DIR/summary.csv"

mkdir -p "$OUT_DIR"

if [[ ! -x "$BIN" ]]; then
  echo "[ERROR] Binary not found: $BIN" >&2
  echo "        Build it: cmake --build build-release --target phastar_push_demo" >&2
  exit 1
fi

# Fresh summary with header.
echo "timestamp,instance_file,index,mode,iterations,threads,seed,dqn_eps_start,dqn_eps_end,dqn_lr,greedy_makespan,best_makespan,delta_vs_greedy,feasible,failed_iters,wall_clock_s" > "$SUMMARY"

now_utc() { date -u +%Y-%m-%dT%H:%M:%SZ; }

# Extract greedy makespan from a log. Echoes the number, or "INFEASIBLE", or "NA".
parse_greedy() {
  local log="$1" line
  line="$(grep -aE '^\[Compare\] greedy:' "$log" | tail -1)"
  if [[ -z "$line" ]]; then echo "NA"; return; fi
  if [[ "$line" == *infeasible* ]]; then echo "INFEASIBLE"; return; fi
  echo "$line" | sed -E 's/.*makespan=([0-9.]+)s.*/\1/'
}

# Extract best makespan for a mode label. Echoes number or "INFEASIBLE"/"NA".
parse_best() {
  local log="$1" label="$2" line
  line="$(grep -aE "^\[Compare\] ${label}:" "$log" | tail -1)"
  if [[ -z "$line" ]]; then echo "NA"; return; fi
  if [[ "$line" == *infeasible* || "$line" == *"no candidates"* ]]; then echo "INFEASIBLE"; return; fi
  echo "$line" | sed -E 's/.*makespan=([0-9.]+)s.*/\1/'
}

parse_delta() {
  local log="$1" label="$2" line
  line="$(grep -aE "^\[Compare\] ${label}:" "$log" | tail -1)"
  echo "$line" | sed -nE 's/.*delta vs greedy=([+-][0-9.]+)s.*/\1/p'
}

parse_wall() {
  grep -aE '^real ' "$1" | tail -1 | awk '{print $2}'
}

# Failed-iteration count. DQN prints "failed=N/M"; LNS result lines carry feas=.
parse_failed() {
  local log="$1" mode="$2"
  if [[ "$mode" == "dqn" ]]; then
    grep -aoE 'failed=[0-9]+/[0-9]+' "$log" | tail -1 | sed -E 's@failed=([0-9]+)/.*@\1@'
  else
    grep -aE '^\[LNS [0-9]+/[0-9]+\] ' "$log" | grep -a 'feas=' | grep -ac 'feas=0'
  fi
}

run_one() {
  local idx="$1" mode="$2" seqfile="$3"
  local log="$OUT_DIR/idx${idx}_${mode}.log"
  local plan="$OUT_DIR/idx${idx}_${mode}_plan.csv"
  local meta="# META mode=${mode} index=${idx} instance=${INSTANCE} iterations=${ITERS} threads=${THREADS} seed=${SEED} dqn_eps_start=${DQN_EPS_START} dqn_eps_end=${DQN_EPS_END} dqn_lr=${DQN_LR} timestamp=$(now_utc)"

  echo "$meta" > "$log"
  /usr/bin/time -p "$BIN" "--${mode}" --no-visualization \
    --random-seed="$SEED" --lns-iters="$ITERS" --lns-threads="$THREADS" \
    --input-sequence="$seqfile" >> "$log" 2>&1

  # Preserve the executed best plan for later visual replay.
  if [[ -f "$PLAN_SRC" ]]; then
    { echo "$meta"; cat "$PLAN_SRC"; } > "$plan"
  fi
}

for ((i = START_INDEX; i <= END_INDEX; i++)); do
  seqfile="$SEQ_DIR/result_seq_${INSTANCE}_ind${i}.b64"
  if [[ ! -f "$seqfile" ]]; then
    echo "[skip] index $i: no sequence file"
    echo "$(now_utc),$INSTANCE,$i,none,$ITERS,$THREADS,$SEED,$DQN_EPS_START,$DQN_EPS_END,$DQN_LR,NA,NA,NA,no_sequence_file,NA,NA" >> "$SUMMARY"
    continue
  fi

  echo "[run ] index $i: DQN..."
  run_one "$i" "dqn" "$seqfile"
  dqn_log="$OUT_DIR/idx${i}_dqn.log"
  greedy="$(parse_greedy "$dqn_log")"

  if [[ "$greedy" == "INFEASIBLE" || "$greedy" == "NA" ]]; then
    echo "[skip] index $i: greedy infeasible (greedy=$greedy) -> skipping comparison"
    echo "$(now_utc),$INSTANCE,$i,greedy,$ITERS,$THREADS,$SEED,$DQN_EPS_START,$DQN_EPS_END,$DQN_LR,$greedy,NA,NA,greedy_failed,NA,NA" >> "$SUMMARY"
    continue
  fi

  # Record DQN row.
  dqn_best="$(parse_best "$dqn_log" 'dqn-online')"
  dqn_delta="$(parse_delta "$dqn_log" 'dqn-online')"
  dqn_wall="$(parse_wall "$dqn_log")"
  dqn_failed="$(parse_failed "$dqn_log" 'dqn')"
  dqn_feas="yes"; [[ "$dqn_best" == "INFEASIBLE" || "$dqn_best" == "NA" ]] && dqn_feas="no"
  echo "$(now_utc),$INSTANCE,$i,dqn,$ITERS,$THREADS,$SEED,$DQN_EPS_START,$DQN_EPS_END,$DQN_LR,$greedy,$dqn_best,$dqn_delta,$dqn_feas,$dqn_failed,$dqn_wall" >> "$SUMMARY"

  echo "[run ] index $i: LNS..."
  run_one "$i" "lns" "$seqfile"
  lns_log="$OUT_DIR/idx${i}_lns.log"
  lns_best="$(parse_best "$lns_log" 'lns-adaptive')"
  lns_delta="$(parse_delta "$lns_log" 'lns-adaptive')"
  lns_wall="$(parse_wall "$lns_log")"
  lns_failed="$(parse_failed "$lns_log" 'lns')"
  lns_feas="yes"; [[ "$lns_best" == "INFEASIBLE" || "$lns_best" == "NA" ]] && lns_feas="no"
  echo "$(now_utc),$INSTANCE,$i,lns,$ITERS,$THREADS,$SEED,$DQN_EPS_START,$DQN_EPS_END,$DQN_LR,$greedy,$lns_best,$lns_delta,$lns_feas,$lns_failed,$lns_wall" >> "$SUMMARY"

  echo "[done] index $i: greedy=$greedy  dqn=$dqn_best (${dqn_delta:-NA})  lns=$lns_best (${lns_delta:-NA})"
done

echo "[ALL DONE] summary -> $SUMMARY"
