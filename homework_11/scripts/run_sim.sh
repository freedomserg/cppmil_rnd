#!/usr/bin/env bash
# Прогін homework_11 у режимі симуляції (socat + gpio-sim), без дротів.
# Запускати на Linux/Pi 5 з root (gpio-sim потребує ядро >=5.17).
#
# Використання:  sudo ./scripts/run_sim.sh <mission 1..10> [checker_binary]
# Змінні:
#   GPIOCHIP=gpiochipN          — задати чип вручну (інакше беремо з виводу чекера)
#   START_LINE=24 DROP_LINE=23  — номери GPIO-ліній
#   SOLVER=analytical|table     — балістичний solver студента
set -euo pipefail

MISSION="${1:-1}"
CHECKER="${2:-./checker_pi_arm64}"
START_LINE="${START_LINE:-24}"
DROP_LINE="${DROP_LINE:-23}"
SOLVER="${SOLVER:-analytical}"
TTY_A=/tmp/ttyA
TTY_B=/tmp/ttyB

HERE="$(cd "$(dirname "$0")/.." && pwd)"
STUDENT="$HERE/../build/debug/homework_11/drone_autopilot"

if [[ $EUID -ne 0 ]]; then echo "Потрібен root (gpio-sim + serial). Запусти через sudo."; exit 1; fi
if [[ ! -x "$CHECKER" ]]; then echo "Не знайдено чекер: $CHECKER"; exit 1; fi
if [[ ! -x "$STUDENT" ]]; then
  echo "Спершу збери студента:"
  echo "  cmake --preset debug && cmake --build --preset debug --target drone_autopilot"
  exit 1
fi

SOCAT_PID=""; CHECKER_PID=""
cleanup() { kill "$SOCAT_PID" "$CHECKER_PID" 2>/dev/null || true; }
trap cleanup EXIT

echo "[sim] socat PTY pair $TTY_A <-> $TTY_B"
socat -d -d pty,raw,echo=0,link=$TTY_A pty,raw,echo=0,link=$TTY_B &
SOCAT_PID=$!
for _ in $(seq 1 50); do [[ -e $TTY_A && -e $TTY_B ]] && break; sleep 0.1; done

CHECKER_LOG="$(mktemp)"
echo "[sim] checker mission $MISSION on $TTY_B (log: $CHECKER_LOG)"
"$CHECKER" "$MISSION" --uart "$TTY_B" --start-line "$START_LINE" --drop-line "$DROP_LINE" \
    >"$CHECKER_LOG" 2>&1 &
CHECKER_PID=$!

CHIP="${GPIOCHIP:-}"
if [[ -z "$CHIP" ]]; then
  for _ in $(seq 1 50); do
    CHIP="$(grep -oE 'gpiochip[0-9]+' "$CHECKER_LOG" | head -n1 || true)"
    [[ -n "$CHIP" ]] && break
    sleep 0.1
  done
fi
[[ -z "$CHIP" ]] && CHIP=gpiochip1
echo "[sim] student on $TTY_A, gpiochip=$CHIP, solver=$SOLVER"

cd "$HERE"
"$STUDENT" --uart "$TTY_A" --gpiochip "$CHIP" \
    --start-line "$START_LINE" --drop-line "$DROP_LINE" --solver "$SOLVER" || true

echo "[sim] --- checker output ---"
cat "$CHECKER_LOG"
rm -f "$CHECKER_LOG"
