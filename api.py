# api.py
from flask import Flask, request, jsonify
from flask_cors import CORS
import threading
import traceback
from flask_socketio import SocketIO, emit
import eventlet

from Bras import Bras

app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*", async_mode="eventlet")

bras = Bras()

def run_in_thread(fn, *args, **kwargs):
    t = threading.Thread(target=fn, args=args, kwargs=kwargs, daemon=True)
    t.start()
    return t

@socketio.on('connect')
def on_connect():
    try:
        print(f"[socket] client connected sid={request.sid}")
    except Exception:
        print("[socket] client connected")

@app.get("/health")
def health():
    return {"ok": True}, 200

@app.post("/api/magnet")
def api_magnet():
    try:
        data = request.get_json(force=True)
        state = int(data.get("state", 0))
        bras.activerAimant(state)
        return jsonify({"message": f"magnet set to {state}"}), 200
    except Exception as e:
        return jsonify({"error": str(e), "trace": traceback.format_exc()}), 500

@app.post("/api/motor/angle")
def api_motor_angle():
    try:
        data = request.get_json(force=True)
        motor = int(data.get("motor"))
        angle = float(data.get("angle"))
        if motor == 1:
            run_in_thread(bras.moteur1.positionAngle, angle)
        elif motor == 2:
            run_in_thread(bras.moteur2.positionAngle, angle)
        else:
            return jsonify({"error": "motor must be 1 or 2"}), 400
        return jsonify({"message": f"angle command for motor {motor} started", "target": angle}), 200
    except Exception as e:
        return jsonify({"error": str(e), "trace": traceback.format_exc()}), 500

@app.post("/api/motor/axe")
def api_motor_axe():
    try:
        data = request.get_json(force=True)
        distance = float(data.get("distance"))
        run_in_thread(bras.moteurAxe.position, distance)
        return jsonify({"message": "axe move started", "distance": distance}), 200
    except Exception as e:
        return jsonify({"error": str(e), "trace": traceback.format_exc()}), 500

@app.post("/api/zero")
def api_zero():
    try:
        data = request.get_json(force=True)
        motor = data.get("motor", "").lower()
        if motor == "axe":
            run_in_thread(bras.moteurAxe.zero)
            return jsonify({"message": "axe zero started"}), 200
        elif motor == "angle1":
            run_in_thread(bras.moteur1.positionAngle, 0.0)
            return jsonify({"message": "angle1 moving to 0°"}), 200
        elif motor == "angle2":
            run_in_thread(bras.moteur2.positionAngle, 0.0)
            return jsonify({"message": "angle2 moving to 0°"}), 200
        else:
            return jsonify({"error": "invalid motor (axe | angle1 | angle2)"}), 400
    except Exception as e:
        return jsonify({"error": str(e), "trace": traceback.format_exc()}), 500

@app.get("/api/status")
def api_status():
    try:
        status = {
            "axe_x": getattr(bras.moteurAxe, "x", None),
            "angle1": getattr(bras.moteur1, "angle", None),
            "angle2": getattr(bras.moteur2, "angle", None),
        }
        return jsonify({"message": "ok", "status": status}), 200
    except Exception as e:
        return jsonify({"error": str(e), "trace": traceback.format_exc()}), 500

@app.post("/api/motor/step")
def api_motor_step():
    try:
        data = request.get_json(force=True)
        motor = data.get("motor")
        direction = int(data.get("direction", 1))
        steps = int(data.get("steps", 1))

        if motor == "axe":
            m = bras.moteurAxe
            speed = getattr(m, "VITESSE_AXE", 0.0001)
        elif motor == "angle1":
            m = bras.moteur1
            speed = getattr(m, "VITESSE_ANGLE", 0.0001)
        elif motor == "angle2":
            m = bras.moteur2
            speed = getattr(m, "VITESSE_ANGLE", 0.0001)
        else:
            return jsonify({"error": "motor must be 'axe'|'angle1'|'angle2'"}), 400

        def do_steps():
            dir_flag = 1 if direction > 0 else 0
            for _ in range(max(1, steps)):
                try:
                    m.one_step(dir_flag, speed)
                except Exception as e:
                    print("step error:", e)

        threading.Thread(target=do_steps, daemon=True).start()
        return jsonify({"message": "step command started", "motor": motor, "direction": direction, "steps": steps}), 200

    except Exception as e:
        return jsonify({"error": str(e), "trace": traceback.format_exc()}), 500

@socketio.on('motor_move')
def on_motor_move(data):
    print("[socket] motor_move", data)
    # mapping: speed -> petits bursts sur moteur1 (angle)
    speed = int(data.get('speed', 0))
    
    # Vérification de la vitesse - s'assurer que ce n'est pas 0 si l'événement est explicitement envoyé
    if speed == 0 and data.get('speed') is None:
        print("Pas de vitesse spécifiée, rien à faire")
        emit('ack', {'msg': 'no speed specified', 'speed': speed})
        return
        
    # On augmente le nombre de pas pour des mouvements plus visibles
    steps = 0 if speed == 0 else max(1, min(80, abs(speed) // 3))  # Divisé par 3 au lieu de 5 pour plus de pas
    
    # Définir directement la direction (0 ou 1) plutôt que d'utiliser 1/-1
    if speed > 0:
        # Avancer
        dir_value = 1  # Changé à 1 pour avancer
    else:
        # Reculer
        dir_value = 0  # Changé à 0 pour reculer
    
    # Debug détaillé
    print(f"Motor move: speed={speed}, dir_value={dir_value}, steps={steps}")

    def burst():
        if steps <= 0:
            print("Aucun pas à effectuer (steps=0)")
            return
            
        print(f"Exécution motor_move: {steps} pas avec dir_value={dir_value}")
        for _ in range(steps):
            try:
                bras.moteur1.one_step(dir_value, bras.moteur1.VITESSE_ANGLE)
            except Exception as e:
                print('motor_move step error', e)
        print(f"Fin des {steps} pas")

    eventlet.spawn_n(burst)
    emit('ack', {'msg': 'moving' if steps>0 else 'stop', 'speed': speed, 'steps': steps})

@socketio.on('turn_wheel')
def on_turn_wheel(data):
    # maps to axe small step left/right
    print("[socket] turn_wheel", data)
    direction = data.get('direction','forward')
    if direction == 'right':
        # Direction pour tourner à droite
        dir_value = 0  # Changé à 0 au lieu de 1
        steps = 5      # Augmenté à 5 pas pour un mouvement plus visible
    elif direction == 'left':
        # Direction pour tourner à gauche
        dir_value = 1  # Changé à 1 au lieu de -1
        steps = 5      # Augmenté à 5 pas pour un mouvement plus visible
    else:
        emit('ack', {'msg':'forward'})
        return

    def burst():
        print(f"Exécution turn_wheel: direction={direction}, dir_value={dir_value}, steps={steps}")
        for _ in range(steps):
            try:
                # Utilisation directe de dir_value sans transformation
                bras.moteurAxe.one_step(dir_value, bras.moteurAxe.VITESSE_AXE)
            except Exception as e:
                print('turn_wheel step error', e)

    eventlet.spawn_n(burst)
    emit('ack', {'msg': 'turn', 'direction': direction})

@socketio.on('motor2_move')
def on_motor2_move(data):
    print("[socket] motor2_move", data)
    # mapping: speed -> petits bursts sur moteur2 (angle)
    speed = int(data.get('speed', 0))
    
    # Vérification de la vitesse - s'assurer que ce n'est pas 0 si l'événement est explicitement envoyé
    if speed == 0 and data.get('speed') is None:
        print("Pas de vitesse spécifiée pour moteur2, rien à faire")
        emit('ack', {'msg': 'no speed specified', 'speed': speed})
        return
        
    # On augmente le nombre de pas pour des mouvements plus visibles
    steps = 0 if speed == 0 else max(1, min(80, abs(speed) // 3))
    
    # Définir directement la direction (0 ou 1) plutôt que d'utiliser 1/-1
    if speed > 0:
        # Monter
        dir_value = 1
    else:
        # Descendre
        dir_value = 0
    
    # Debug détaillé
    print(f"Motor2 move: speed={speed}, dir_value={dir_value}, steps={steps}")

    def burst():
        if steps <= 0:
            print("Aucun pas à effectuer pour moteur2 (steps=0)")
            return
            
        print(f"Exécution motor2_move: {steps} pas avec dir_value={dir_value}")
        for _ in range(steps):
            try:
                bras.moteur2.one_step(dir_value, bras.moteur2.VITESSE_ANGLE)
            except Exception as e:
                print('motor2_move step error', e)
        print(f"Fin des {steps} pas pour moteur2")

    eventlet.spawn_n(burst)
    emit('ack', {'msg': 'moving motor2' if steps>0 else 'stop motor2', 'speed': speed, 'steps': steps})

@socketio.on('magnet')
def on_magnet(data):
    state = int(data.get('state', 0))
    try:
        bras.activerAimant(state)
    except Exception as e:
        emit('error', {'msg': str(e)})
    emit('ack', {'magnet': state})

if __name__ == "__main__":
    # pour dev: python3 api.py
    socketio.run(app, host="0.0.0.0", port=5000, debug=False)
