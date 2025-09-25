#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")" && pwd)"


API_MODULE="api:app"
API_PORT=8000
API_HOST="0.0.0.0"
API_CMD="uvicorn ${API_MODULE} --host ${API_HOST} --port ${API_PORT}"
FRONT_CMD="npm run dev -- --host"

# tableau des PIDs
pids=()

# Fonction d'arrêt propre
stop_all() {
  echo
  echo "Arrêt des services..."
  for pid in "${pids[@]}"; do
    if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
      echo " -> stopping pid ${pid}"
      kill "$pid" 2>/dev/null || true
      sleep 1
      if kill -0 "$pid" 2>/dev/null; then
        echo " -> force kill ${pid}"
        kill -9 "$pid" 2>/dev/null || true
      fi
    fi
  done
  echo "Stopped."
  exit 0
}

trap stop_all SIGINT SIGTERM



# --- Lancer l'API ---
# on lance dans un sous-shell et on récupère le PID du process démarré
API_PID="$(bash -lc "cd \"${ROOT_DIR}\" && ${API_CMD} & echo \$!")" || API_PID=""
sleep 0.2

# fallback si on n'a pas récupéré le PID correctement
if [ -z "${API_PID}" ] || ! kill -0 "${API_PID}" 2>/dev/null; then
  API_PID="$(pgrep -f "uvicorn ${API_MODULE}" | head -n1 || true)"
fi

if [ -n "${API_PID}" ]; then
  echo "API PID=${API_PID}"
  pids+=("${API_PID}")
else
  echo "Impossible de récupérer le PID de l'API. Vérifie que uvicorn est disponible."
fi

# --- Lancer le frontend ---
FRONT_PID="$(bash -lc "cd \"${ROOT_DIR}\" && ${FRONT_CMD} & echo \$!")" || FRONT_PID=""
sleep 0.2

if [ -z "${FRONT_PID}" ] || ! kill -0 "${FRONT_PID}" 2>/dev/null; then
  FRONT_PID="$(pgrep -f "npm run dev" | head -n1 || true)"
fi

if [ -n "${FRONT_PID}" ]; then
  echo "Frontend PID=${FRONT_PID}"
  pids+=("${FRONT_PID}")
else
  echo "Impossible de récupérer le PID du frontend. Vérifie que npm est disponible."
fi

echo ""
echo "Services démarrés. Appuie sur CTRL+C pour arrêter les deux."
echo ""

# boucle de supervision : si un process meurt, on arrête l'autre
while true; do
  sleep 1
  for pid in "${pids[@]}"; do
    if [ -n "${pid}" ] && ! kill -0 "${pid}" 2>/dev/null; then
      echo "Process ${pid} terminé — arrêt des autres."
      stop_all
    fi
  done
done
