
<template>
  <div class="command-head" tabindex="0" @keydown="onKeyDown" @keyup="onKeyUp">
    <div class="card">
      <h2>Command Motor</h2>

      <!-- Control Instructions -->
      <div class="instructions">
        <p>Z/S: Moteur 1 (avant/arrière)</p>
        <p>Q/D: Rotation (gauche/droite)</p>
        <p>I/K: Moteur 2 (haut/bas)</p>
      </div>

      <!-- Control Layout -->
      <div class="control-container">
        <!-- Première rangée de contrôles - Moteur 1 et Rotation -->
        <div class="direction-controls">
          <div class="control-row">
            <button
              class="control-btn up"
              data-key="z"
              :class="{ active: motorKeys.has('z') }"
              @mousedown="() => handleButtonDown('z')"
              @mouseup="() => handleButtonUp('z')"
              @mouseleave="() => handleButtonUp('z')"
              @touchstart.prevent="() => handleButtonDown('z')"
              @touchend.prevent="() => handleButtonUp('z')"
            >
              <i class="icon">⬆️ </i>
              <span class="key-label">Z</span>
            </button>
          </div>

          <div class="control-row middle-row">
            <button
              class="control-btn left"
              data-key="q"
              :class="{ active: motorKeys.has('q') }"
              @mousedown="() => handleButtonDown('q')"
              @mouseup="() => handleButtonUp('q')"
              @mouseleave="() => handleButtonUp('q')"
              @touchstart.prevent="() => handleButtonDown('q')"
              @touchend.prevent="() => handleButtonUp('q')"
            >
              <i class="icon">⬅️</i>
              <span class="key-label">Q</span>
            </button>

            <div class="center-indicator">
              <div class="camera-icon">🏍️</div>
            </div>

            <button
              class="control-btn right"
              data-key="d"
              :class="{ active: motorKeys.has('d') }"
              @mousedown="() => handleButtonDown('d')"
              @mouseup="() => handleButtonUp('d')"
              @mouseleave="() => handleButtonUp('d')"
              @touchstart.prevent="() => handleButtonDown('d')"
              @touchend.prevent="() => handleButtonUp('d')"
            >
              <i class="icon">➡️</i>
              <span class="key-label">D</span>
            </button>
          </div>

          <div class="control-row">
            <button
              class="control-btn down"
              data-key="s"
              :class="{ active: motorKeys.has('s') }"
              @mousedown="() => handleButtonDown('s')"
              @mouseup="() => handleButtonUp('s')"
              @mouseleave="() => handleButtonUp('s')"
              @touchstart.prevent="() => handleButtonDown('s')"
              @touchend.prevent="() => handleButtonUp('s')"
            >
              <i class="icon">⬇️</i>
              <span class="key-label">S</span>
            </button>
          </div>
        </div>

        <!-- Seconde rangée de contrôles - Moteur 2 -->
        <div class="motor2-controls">
          <h3>Moteur 2 (Angulaire)</h3>
          <div class="motor2-buttons">
            <button
              class="control-btn motor2-up"
              data-key="i"
              :class="{ active: motorKeys.has('i') }"
              @mousedown="() => handleButtonDown('i')"
              @mouseup="() => handleButtonUp('i')"
              @mouseleave="() => handleButtonUp('i')"
              @touchstart.prevent="() => handleButtonDown('i')"
              @touchend.prevent="() => handleButtonUp('i')"
            >
              <i class="icon">↗️</i>
              <span class="key-label">I</span>
            </button>

            <button
              class="control-btn motor2-down"
              data-key="k"
              :class="{ active: motorKeys.has('k') }"
              @mousedown="() => handleButtonDown('k')"
              @mouseup="() => handleButtonUp('k')"
              @mouseleave="() => handleButtonUp('k')"
              @touchstart.prevent="() => handleButtonDown('k')"
              @touchend.prevent="() => handleButtonUp('k')"
            >
              <i class="icon">↘️</i>
              <span class="key-label">K</span>
            </button>
          </div>
        </div>
      </div>
    </div>
  </div>
  
</template>

<script setup>
import { reactive, onMounted, onBeforeUnmount } from "vue";
import { io } from "socket.io-client";

// Connexion Socket.IO
// Base: VITE_ROBOT_BASE_URL (ex: http://ip:5000) sinon même origine (utile en dev avec proxy)
const RAW_BASE = import.meta.env.VITE_ROBOT_BASE_URL || window.location.origin;
const isWs = RAW_BASE.startsWith("ws://") || RAW_BASE.startsWith("wss://");
const SOCKET_BASE = RAW_BASE;

// Connexion Socket.IO avec debug
console.log("Connexion Socket.IO à", SOCKET_BASE);
const socket = io(SOCKET_BASE, {
  transports: isWs ? ["websocket"] : ["websocket", "polling"],
  reconnection: true,
  path: "/socket.io",
});

socket.on("connect", () => {
  console.log("Socket connected", socket.id);
  // Envoi initial pour tester la connexion
  socket.emit("test_connection", { status: "connected" });
});

socket.on("connect_error", (error) => {
  console.error("Connection error:", error);
  // Afficher plus de détails sur l'erreur de connexion
  console.error("Connection details:", {
    socketBase: SOCKET_BASE,
    isWs: isWs,
    transport: socket.io.engine.transport.name,
  });
});

socket.on("disconnect", () => {
  console.log("Disconnected from server");
});

// Écouter les acquittements du serveur
socket.on("ack", (data) => {
  console.log("Server acknowledgment:", data);
});

// Fallback HTTP base for API
const HTTP_BASE = import.meta.env.VITE_API_BASE || window.location.origin;

async function httpPost(path, body) {
  try {
    console.log(`Sending HTTP POST to ${HTTP_BASE}${path}`, body);
    
    // Vérifier l'URL de base
    if (!HTTP_BASE) {
      console.error("HTTP_BASE est vide ou non défini");
      throw new Error("Base URL is undefined");
    }
    
    const response = await fetch(`${HTTP_BASE}${path}`, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body),
      // Ajouter des options supplémentaires pour aider avec CORS
      mode: "cors",
      credentials: "same-origin",
    });
    
    if (!response.ok) {
      console.error(`HTTP error status: ${response.status}`);
      throw new Error(`HTTP error! Status: ${response.status}`);
    }
    
    const data = await response.json();
    console.log(`HTTP POST response from ${path}:`, data);
    return data;
  } catch (e) {
    console.error(`HTTP post failed for ${path}:`, e);
    throw e;
  }
}

async function emitOrPost(eventName, payload) {
  // Toujours déboguer les événements
  console.log(`emitOrPost: ${eventName}`, payload, `Socket connected: ${socket.connected}`);
  
  // Enregistrer la tentative
  const timestamp = new Date().toISOString();
  console.log(`[${timestamp}] Tentative d'envoi: ${eventName}`, payload);
  
  // Essayer Socket.IO d'abord
  if (socket.connected) {
    console.log(`Emitting socket event: ${eventName}`, payload);
    
    // Ajouter un mécanisme d'acquittement avec timeout
    return new Promise((resolve) => {
      const timeout = setTimeout(() => {
        console.warn(`Socket.io timeout for event ${eventName}`);
        resolve(false);
      }, 1000);
      
      // Configuration d'un callback pour l'acquittement
      socket.emit(eventName, payload, (ack) => {
        clearTimeout(timeout);
        console.log(`Received acknowledgment for ${eventName}:`, ack);
        resolve(true);
      });
    })
    .then((acknowledged) => {
      if (!acknowledged) {
        console.log("No acknowledgment received, falling back to HTTP");
        return httpPostFallback(eventName, payload);
      }
      return true;
    });
  } else {
    console.log("Socket not connected, falling back to HTTP");
    return httpPostFallback(eventName, payload);
  }
}

async function httpPostFallback(eventName, payload) {
  try {
    console.log(`HTTP fallback for ${eventName}`, payload);
    
    if (eventName === "motor_move") {
      const steps = Math.max(1, Math.min(60, Math.floor(Math.abs(payload?.speed || 0) / 5)));
      const direction = (payload?.speed || 0) >= 0 ? 1 : -1;
      return await httpPost("/api/motor/step", { motor: "angle1", direction, steps });
    } 
    else if (eventName === "turn_wheel") {
      const dir = payload?.direction === "right" ? 1 : payload?.direction === "left" ? -1 : 0;
      if (dir !== 0) {
        return await httpPost("/api/motor/step", { motor: "axe", direction: dir, steps: 3 });
      }
    } 
    else if (eventName === "magnet") {
      return await httpPost("/api/magnet", payload || {});
    }
    return false;
  } catch (e) {
    console.error(`HTTP fallback for ${eventName} failed:`, e);
    return false;
  }
}

// Set réactif pour suivre les touches moteur enfoncées
const motorKeys = reactive(new Set());
let repeatTimer = null;

function sendMotor() {
  // Debug: voir les émissions dans la console avec plus de détails
  console.log("sendMotor keys:", Array.from(motorKeys));
  
  try {
    // Traiter les commandes séparément - ne pas envoyer de vitesse 0 si une autre touche est pressée
    
    // Gestion du moteur 1 avant/arrière (Z/S)
    if (motorKeys.has("z")) {
      console.log("emit motor_move +100");
      emitOrPost("motor_move", { speed: 100 });
    } else if (motorKeys.has("s")) {
      console.log("emit motor_move -100");
      emitOrPost("motor_move", { speed: -100 });
    } 
    // Suppression de l'envoi de vitesse 0 pour éviter d'arrêter le mouvement
    // Ne pas envoyer de commande d'arrêt si aucune touche n'est pressée

    // Gestion de la rotation (Q/D) - indépendant du mouvement avant/arrière
    if (motorKeys.has("d")) {
      console.log("emit turn_wheel right");
      emitOrPost("turn_wheel", { direction: "right" });
    } else if (motorKeys.has("q")) {
      console.log("emit turn_wheel left");
      emitOrPost("turn_wheel", { direction: "left" });
    }
    
    // Gestion du moteur 2 angulaire (I/K)
    if (motorKeys.has("i")) {
      console.log("emit motor2_move +100");
      emitOrPost("motor2_move", { speed: 100 });
    } else if (motorKeys.has("k")) {
      console.log("emit motor2_move -100");
      emitOrPost("motor2_move", { speed: -100 });
    }
    // Suppression de l'envoi de direction "forward" pour éviter d'arrêter la rotation
    // Ne pas envoyer de commande de direction neutre si aucune touche n'est pressée
  } catch (error) {
    console.error("Erreur lors de l'envoi des commandes moteur:", error);
  }
}

function startRepeat(){
  if (repeatTimer) {
    console.log("Timer already running, skipping");
    return;
  }
  
  console.log("Starting repeat timer for motor commands");
  repeatTimer = setInterval(() => {
    if (motorKeys.size > 0) {
      console.log("Repeat timer: sending motor commands");
      sendMotor();
    } else {
      console.log("Repeat timer: no keys pressed, stopping");
      stopRepeat();
    }
  }, 100); // Réduit à 100ms pour une réactivité accrue
}

function stopRepeat(){
  if (!repeatTimer) {
    console.log("No timer to stop");
    return;
  }
  
  console.log("Stopping repeat timer");
  clearInterval(repeatTimer);
  repeatTimer = null;
  
  // Envoyer une commande d'arrêt explicite lorsque toutes les touches sont relâchées
  if (motorKeys.size === 0) {
    console.log("Toutes les touches relâchées, envoi des commandes d'arrêt");
    // Envoyer un arrêt explicite pour le moteur
    emitOrPost("motor_move", { speed: 0, explicit_stop: true });
  }
}

function handleButtonDown(key) {
  if (!["z", "q", "s", "d", "i", "k"].includes(key) || motorKeys.has(key)) return;
  motorKeys.add(key);
  sendMotor();
  startRepeat();
}

function handleButtonUp(key) {
  if (!motorKeys.has(key)) return;
  motorKeys.delete(key);
  sendMotor();
  if (motorKeys.size === 0) stopRepeat();
}

function onMotorKeyDown(e) {
  const k = e.key.toLowerCase();
  if (!["z", "q", "s", "d", "i", "k"].includes(k) || motorKeys.has(k)) return;

  console.log(`Touche pressée: ${k}`);
  motorKeys.add(k);
  sendMotor();
  startRepeat();
}

function onMotorKeyUp(e) {
  const k = e.key.toLowerCase();
  console.log("Key up:", k);
  
  if (!motorKeys.has(k)) {
    console.log(`Ignoring key up: ${k} (not in active keys)`);
    return;
  }

  console.log(`Removing key: ${k} from active keys`);
  motorKeys.delete(k);
  
  // Pour éviter des commandes contradictoires, ne pas envoyer de commandes
  // intermédiaires pendant qu'on relâche plusieurs touches en même temps
  if (motorKeys.size === 0) {
    console.log("Dernière touche relâchée - arrêt complet");
    // Envoi explicite d'une commande d'arrêt
    emitOrPost("motor_move", { speed: 0, explicit_stop: true });
    stopRepeat();
  } else {
    // D'autres touches sont encore enfoncées, envoyer les commandes pour ces touches
    sendMotor();
  }
}

// Fonctions référencées dans le template
function onKeyDown(e){ 
  console.log("onKeyDown dans le template");
  onMotorKeyDown(e);
}
function onKeyUp(e){ 
  console.log("onKeyUp dans le template");
  onMotorKeyUp(e);
}

onMounted(() => {
  console.log("Composant monté - ajout des écouteurs d'événements");
  window.addEventListener("keydown", onMotorKeyDown);
  window.addEventListener("keyup", onMotorKeyUp);
  
  // Assurez-vous que l'élément a le focus pour recevoir les événements clavier
  const controlElement = document.querySelector('.command-head');
  if (controlElement) {
    console.log("Mise au point automatique sur l'élément de contrôle");
    controlElement.focus();
  }
});

onBeforeUnmount(() => {
  window.removeEventListener("keydown", onMotorKeyDown);
  window.removeEventListener("keyup", onMotorKeyUp);
  socket.disconnect();
  stopRepeat();
});
</script>

<style scoped>
.command-head {
  outline: none;
  max-width: 400px;
  margin: 0 auto;
}

.card {
  background-color: var(--surface-color, #1e1e1e);
  border-radius: 16px;
  padding: 2rem;
  box-shadow: 0 8px 24px rgba(0, 0, 0, 0.2);
  border: 1px solid var(--border-color, rgba(255, 255, 255, 0.12));
}

.card h2 {
  color: var(--primary-color, #bb86fc);
  margin-bottom: 1.5rem;
  text-align: center;
  font-size: 1.8rem;
}

.instructions {
  text-align: center;
  margin-bottom: 2rem;
  color: var(--text-secondary, rgba(255, 255, 255, 0.7));
}

.control-container {
  display: flex;
  flex-direction: column;
  gap: 2rem;
}

.direction-controls {
  display: flex;
  flex-direction: column;
  gap: 1rem;
  max-width: 240px;
  margin: 0 auto;
}

.control-row {
  display: flex;
  justify-content: center;
  gap: 1rem;
}

.middle-row {
  align-items: center;
}

.control-btn {
  background-color: var(--surface-color, #2a2a2a);
  border: 2px solid var(--border-color, rgba(255, 255, 255, 0.12));
  color: var(--text-primary, #ffffff);
  border-radius: 12px;
  padding: 1rem;
  cursor: pointer;
  transition: all 0.2s ease;
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.5rem;
  min-width: 70px;
  min-height: 70px;
  font-size: 0.9rem;
  font-weight: 500;
  user-select: none;
}

.control-btn:hover {
  background-color: var(--primary-color, #bb86fc);
  color: var(--background-color, #121212);
  transform: translateY(-2px);
  box-shadow: 0 4px 12px rgba(187, 134, 252, 0.3);
}

.control-btn:active,
.control-btn.active {
  background-color: var(--primary-color, #bb86fc);
  color: var(--background-color, #121212);
  transform: translateY(0);
  box-shadow: 0 2px 8px rgba(187, 134, 252, 0.4);
}

.icon {
  font-size: 1.5rem;
}

.center-indicator {
  display: flex;
  align-items: center;
  justify-content: center;
  min-width: 70px;
  min-height: 70px;
}

.camera-icon {
  font-size: 2rem;
  color: var(--primary-color, #bb86fc);
}

/* Mobile Responsive */
@media (max-width: 768px) {
  .card {
    padding: 1.5rem;
    margin: 0 1rem;
  }

  .card h2 {
    font-size: 1.5rem;
  }

  .control-btn {
    min-width: 60px;
    min-height: 60px;
    padding: 0.75rem;
  }

  .icon {
    font-size: 1.3rem;
  }

  .camera-icon {
    font-size: 1.5rem;
  }
}

@media (max-width: 480px) {
  .card {
    padding: 1rem;
  }

  .control-btn {
    min-width: 50px;
    min-height: 50px;
    padding: 0.5rem;
  }

  .icon {
    font-size: 1.1rem;
  }

  .center-indicator {
    min-width: 50px;
    min-height: 50px;
  }

  .camera-icon {
    font-size: 1.2rem;
  }
}

.icon {
  font-size: 1.2rem;
}

.key-label {
  font-size: 0.8rem;
  margin-top: 0.25rem;
  color: rgba(255, 255, 255, 0.7);
}

.motor2-controls {
  margin-top: 2rem;
  text-align: center;
}

.motor2-controls h3 {
  color: var(--primary-color, #bb86fc);
  margin-bottom: 1rem;
}

.motor2-buttons {
  display: flex;
  justify-content: center;
  gap: 2rem;
}

.instructions p {
  margin: 0.3rem 0;
  font-size: 0.9rem;
}
</style>
