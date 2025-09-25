<template>
  <div class="arm-layout">
    <div class="sidebar">
      <div class="motor-block">
        <h4>Moteur Axe</h4>
        <button
          class="ctl-btn"
          :class="{ active: active['axe-pos'] }"
          @mousedown.prevent="startPress('axe','pos')"
          @mouseup="endPress('axe')"
          @mouseleave="endPress('axe')"
          @touchstart.prevent="startPress('axe','pos')"
          @touchend.prevent="endPress('axe')"
        >
          Q
        </button>
        <button
          class="ctl-btn"
          :class="{ active: active['axe-neg'] }"
          @mousedown.prevent="startPress('axe','neg')"
          @mouseup="endPress('axe')"
          @mouseleave="endPress('axe')"
          @touchstart.prevent="startPress('axe','neg')"
          @touchend.prevent="endPress('axe')"
        >
          D
        </button>
      </div>

      <div class="motor-block">
        <h4>Moteur Angle 1</h4>
        <button
          class="ctl-btn"
          :class="{ active: active['1-pos'] }"
          @mousedown.prevent="startPress('1','pos')"
          @mouseup="endPress('1')"
          @mouseleave="endPress('1')"
          @touchstart.prevent="startPress('1','pos')"
          @touchend.prevent="endPress('1')"
        >
          Z
        </button>
        <button
          class="ctl-btn"
          :class="{ active: active['1-neg'] }"
          @mousedown.prevent="startPress('1','neg')"
          @mouseup="endPress('1')"
          @mouseleave="endPress('1')"
          @touchstart.prevent="startPress('1','neg')"
          @touchend.prevent="endPress('1')"
        >
         S
        </button>
      </div>

      <div class="motor-block">
        <h4>Moteur Angle 2</h4>
        <button
          class="ctl-btn"
          :class="{ active: active['2-pos'] }"
          @mousedown.prevent="startPress('2','pos')"
          @mouseup="endPress('2')"
          @mouseleave="endPress('2')"
          @touchstart.prevent="startPress('2','pos')"
          @touchend.prevent="endPress('2')"
        >
          I
        </button>
        <button
          class="ctl-btn"
          :class="{ active: active['2-neg'] }"
          @mousedown.prevent="startPress('2','neg')"
          @mouseup="endPress('2')"
          @mouseleave="endPress('2')"
          @touchstart.prevent="startPress('2','neg')"
          @touchend.prevent="endPress('2')"
        >
          K
        </button>
      </div>

      <div class="status">
        <div>WS: {{ wsState }}</div>
        <div>{{ lastResp }}</div>
      </div>
    </div>

    <div class="images">
      <img src="../img/pcb.png" alt="pcb" class="image" />
      <img src="../img/schematic.png" alt="schematic" class="image" />
      <img src="../img/fusion.png" alt="fusion" class="image" />
    </div>
  </div>
</template>

<script setup>
import { reactive, ref, onMounted, onBeforeUnmount } from 'vue'

const wsState = ref('disconnected')
const lastResp = ref('')
let ws = null

// -- configuration : mettre l'IP / host correct --
const WS_URL = 'ws://10.3.228.14:8000/ws' // <- change this to your Pi IP

// ======= CORRECTION =======
// pressedKeys initialisé explicitement (Set)
const pressedKeys = new Set()

// état visuel des boutons
const active = reactive({
  'axe-pos': false, 'axe-neg': false,
  '1-pos': false, '1-neg': false,
  '2-pos': false, '2-neg': false
})

// mapping des touches -> action
const keyMap = {
  'q': { motor: 'axe', dir: 'pos' },
  'd': { motor: 'axe', dir: 'neg' },
  'z': { motor: '1', dir: 'pos' },
  's': { motor: '1', dir: 'neg' },
  'i': { motor: '2', dir: 'pos' },
  'k': { motor: '2', dir: 'neg' }
}

function connect() {
  try {
    ws = new WebSocket(WS_URL)
  } catch (e) {
    console.error('Erreur création WebSocket:', e)
    wsState.value = 'error'
    lastResp.value = 'ws create error'
    return
  }

  ws.onopen = () => {
    wsState.value = 'connected'
    console.log('WS ouvert')
  }
  ws.onclose = (ev) => {
    wsState.value = 'closed'
    console.warn('WS closed', ev)
  }
  ws.onerror = (e) => {
    wsState.value = 'error'
    console.error('WS error', e)
  }
  ws.onmessage = (ev) => {
    try {
      const data = JSON.parse(ev.data)
      lastResp.value = JSON.stringify(data)
    } catch (e) {
      lastResp.value = ev.data
    }
  }
}

// envoie protégé
function send(obj) {
  if (!ws || ws.readyState !== WebSocket.OPEN) {
    lastResp.value = 'WS not open'
    return
  }
  ws.send(JSON.stringify(obj))
}

function startPress(motor, dir) {
  const key = `${motor}-${dir}`
  if (active[key]) return
  active[key] = true
  send({ action: 'start', motor: motor, dir: dir })
}

function endPress(motor) {
  active[`${motor}-pos`] = false
  active[`${motor}-neg`] = false
  send({ action: 'stop', motor: motor })
}

// ===== gestion clavier =====
function handleKeyDown(e) {
  const k = String(e.key).toLowerCase()
  if (!(k in keyMap)) return
  e.preventDefault()
  if (pressedKeys.has(k)) return // ignore key repeat
  pressedKeys.add(k)
  const { motor, dir } = keyMap[k]
  startPress(motor, dir)
}

function handleKeyUp(e) {
  const k = String(e.key).toLowerCase()
  if (!(k in keyMap)) return
  e.preventDefault()
  pressedKeys.delete(k)
  const { motor } = keyMap[k]
  endPress(motor)
}

// sécurité : si fenêtre perd focus, on arrête tout
function stopAllFromBlur() {
  try {
    if (pressedKeys && typeof pressedKeys.clear === 'function') {
      pressedKeys.clear()
    }
  } catch (err) {
    console.warn('pressedKeys.clear failed (ignored):', err)
  }
  ;['axe','1','2'].forEach(m => endPress(m))
}
function handleWindowBlur() { stopAllFromBlur() }
function handleVisibilityChange() {
  if (document.visibilityState !== 'visible') stopAllFromBlur()
}

onMounted(() => {
  connect()
  window.addEventListener('keydown', handleKeyDown)
  window.addEventListener('keyup', handleKeyUp)
  window.addEventListener('blur', handleWindowBlur)
  window.addEventListener('visibilitychange', handleVisibilityChange)
})

onBeforeUnmount(() => {
  window.removeEventListener('keydown', handleKeyDown)
  window.removeEventListener('keyup', handleKeyUp)
  window.removeEventListener('blur', handleWindowBlur)
  window.removeEventListener('visibilitychange', handleVisibilityChange)
  try { if (ws && ws.readyState === WebSocket.OPEN) ws.close() } catch(e){/* ignore */ }
})
</script>


<style scoped>
.arm-layout{
  display:flex;
  gap: 1rem;
  align-items:flex-start;
  padding: 1rem;
  flex-wrap:wrap;
}

/* sidebar boutons à gauche */
.sidebar{
  min-width: 220px;
  display:flex;
  flex-direction: column;
  gap: 1rem;
  align-items: stretch;
  background: rgba(255,255,255,0.02);
  padding: 8px;
  border-radius: 8px;
}

.motor-block{
  display:flex;
  flex-direction: column;
  gap:0.5rem;
  align-items: center;
  padding: 0.5rem;
  border-radius: 6px;
  background: rgba(0,0,0,0.03);
}

.ctl-btn{
  min-width: 160px;
  padding: 0.6rem 0.8rem;
  border-radius: 8px;
  border: 1px solid #dddddd54;

  cursor: pointer;
  font-weight: 600;
  user-select: none;
}

.ctl-btn.active{
  
  border-color: #0073e6ff;
  box-shadow: 0 6px 18px rgba(30,144,255,0.12);
}

.ctl-btn:active{
  transform: translateY(1px);
}

.images{
  display:flex;
  flex-direction: column;
  gap: 1rem;
}

.image{
  width: 400px;
  max-width: 85vw;
  border-radius: 6px;
  box-shadow: 0 4px 14px rgba(0,0,0,0.08);
}
.status{
  font-size: 0.85rem;
  color: #444;
  padding: 0.5rem;
}
</style>
