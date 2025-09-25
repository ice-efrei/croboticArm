# bras_api.py (version robuste)
import json
import threading
import time
from fastapi import FastAPI, WebSocket, WebSocketDisconnect, HTTPException

from contextlib import asynccontextmanager

@asynccontextmanager
async def lifespan(app):
    # démarre l'initialisation du Bras en thread pour ne pas bloquer le serveur
    init_thread = threading.Thread(target=_safe_init_bras, daemon=True)
    init_thread.start()
    try:
        yield
    finally:
        # cleanup au shutdown : arrêter les moteurs si le controller existe
        try:
            if controller:
                controller.stop_all()
        except Exception as e:
            print("Erreur lors du cleanup au shutdown :", e)

# crée l'app avec le gestionnaire de lifespan
app = FastAPI(lifespan=lifespan)

# Adapter le nom du module au nom réel sur ta Pi (Bras.py ou bras.py)
try:
    from Bras import Bras
except Exception as e:
    print("Import Bras failed:", e)
    Bras = None

# contrôle global
controller = None
_controller_lock = threading.Lock()

class BrasController:
    def __init__(self, bras_instance):
        self.bras = bras_instance
        self.threads = {}
        self.stop_events = {}

    def _motor_obj(self, name):
        if name == "axe":
            return self.bras.moteurAxe
        if name == "1":
            return self.bras.moteur1
        if name == "2":
            return self.bras.moteur2
        raise KeyError("motor unknown")

    def start(self, name: str, direction: str):
        if name in self.threads and self.threads[name].is_alive():
            return {"status": "already_running"}

        motor = self._motor_obj(name)
        dir_int = 1 if direction in ("pos", "1", "up", "+", "cw") else 0
        speed = getattr(motor, "VITESSE_AXE", None) if name == "axe" else getattr(motor, "VITESSE_ANGLE", None)
        if speed is None:
            speed = 0.001

        stop_event = threading.Event()

        def run_loop():
            try:
                while not stop_event.is_set():
                    try:
                        motor.one_step(dir_int, speed)
                    except Exception as e:
                        print(f"Erreur moteur {name}: {e}")
                        break
            finally:
                print(f"Thread moteur {name} terminé")

        th = threading.Thread(target=run_loop, daemon=True)
        self.threads[name] = th
        self.stop_events[name] = stop_event
        th.start()
        return {"status": "started"}

    def stop(self, name: str):
        ev = self.stop_events.get(name)
        if ev:
            ev.set()
        th = self.threads.get(name)
        if th:
            th.join(timeout=1.0)
        self.threads.pop(name, None)
        self.stop_events.pop(name, None)
        return {"status": "stopped"}

    def stop_all(self):
        for n in list(self.threads.keys()):
            self.stop(n)

# état d'init : "not_started", "initializing", "ready", "error"
_init_state = {"status": "not_started", "error": None}

def _safe_init_bras():
    global controller, _init_state
    with _controller_lock:
        if _init_state["status"] == "ready":
            print("Bras déjà initialisé.")
            return
        _init_state["status"] = "initializing"
        _init_state["error"] = None
        try:
            if Bras is None:
                raise RuntimeError("Classe Bras introuvable (import failed)")
            print("Initialisation du Bras (en thread)...")
            # instantiate (attention : si Bras.__init__ bloque, ce thread restera occupé mais ne tue pas le serveur)
            bras_instance = Bras()
            controller = BrasController(bras_instance)
            _init_state["status"] = "ready"
            print("Bras initialisé avec succès.")
        except Exception as e:
            _init_state["status"] = "error"
            _init_state["error"] = str(e)
            print("Erreur à l'initialisation du Bras:", e)

@app.on_event("startup")
def startup_event():
    # Lancer l'init en thread pour ne pas bloquer uvicorn si Bras.__init__ est lent ou bloquant
    t = threading.Thread(target=_safe_init_bras, daemon=True)
    t.start()

@app.get("/health")
def health():
    return {"server": "ok", "bras": _init_state}

@app.post("/init_bras")
def init_bras_manual():
    # endpoint pour forcer (ré)initialisation manuelle
    if _init_state["status"] == "initializing":
        raise HTTPException(status_code=409, detail="Already initializing")
    t = threading.Thread(target=_safe_init_bras, daemon=True)
    t.start()
    return {"result": "initialization_started"}

@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    print("Client WebSocket connecté")
    try:
        while True:
            msg = await ws.receive_text()
            try:
                data = json.loads(msg)
            except json.JSONDecodeError:
                await ws.send_text(json.dumps({"error": "invalid json"}))
                continue

            action = data.get("action")
            motor = data.get("motor")
            direction = data.get("dir", "pos")

            if controller is None:
                await ws.send_text(json.dumps({"error": "controller not initialized", "state": _init_state}))
                continue

            if action == "start" and motor in ("axe", "1", "2"):
                res = controller.start(motor, direction)
                await ws.send_text(json.dumps({"result": res}))
            elif action == "stop" and motor in ("axe", "1", "2"):
                res = controller.stop(motor)
                await ws.send_text(json.dumps({"result": res}))
            elif action == "stop_all":
                controller.stop_all()
                await ws.send_text(json.dumps({"result": "all stopped"}))
            else:
                await ws.send_text(json.dumps({"error": "unknown action or motor"}))

    except WebSocketDisconnect:
        print("Client déconnecté, arrêt des moteurs")
        if controller:
            controller.stop_all()
    except Exception as e:
        print("WebSocket erreur:", e)
        if controller:
            controller.stop_all()
