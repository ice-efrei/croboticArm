<template>
  <div class="p-6 max-w-3xl mx-auto">
    <h1 class="text-2xl font-bold mb-4">Contrôle du bras - Interface Vue.js</h1>

    <section class="mb-6">
      <h2 class="font-semibold">Aimant</h2>
      <div class="flex items-center gap-3 mt-2">
        <button @click="setMagnet(1)" class="px-3 py-1 rounded border">Allumer</button>
        <button @click="setMagnet(0)" class="px-3 py-1 rounded border">Éteindre</button>
        <span v-if="status.msg">{{ status.msg }}</span>
      </div>
    </section>

    <section class="mb-6">
      <h2 class="font-semibold">Moteurs d'angle</h2>
      <div class="grid grid-cols-2 gap-4 mt-2">
        <div>
          <label>Moteur 1 (angle): {{ angle1 }}°</label>
          <input type="range" min="-180" max="180" v-model.number="angle1" />
          <div class="mt-2">
            <button @click="sendAngle(1, angle1)" class="px-3 py-1 rounded border">Envoyer Moteur 1</button>
            <button @click="zeroMotor('angle1')" class="px-3 py-1 rounded border ml-2">Zero</button>
          </div>
        </div>

        <div>
          <label>Moteur 2 (angle): {{ angle2 }}°</label>
          <input type="range" min="-180" max="180" v-model.number="angle2" />
          <div class="mt-2">
            <button @click="sendAngle(2, angle2)" class="px-3 py-1 rounded border">Envoyer Moteur 2</button>
            <button @click="zeroMotor('angle2')" class="px-3 py-1 rounded border ml-2">Zero</button>
          </div>
        </div>
      </div>
    </section>

    <section class="mb-6">
      <h2 class="font-semibold">Moteur d'axe (déplacement en cm)</h2>
      <div class="mt-2">
        <label>Distance: {{ axeDistance }} cm</label>
        <input type="range" min="0" max="34" step="0.1" v-model.number="axeDistance" />
        <div class="mt-2">
          <button @click="sendAxe(axeDistance)" class="px-3 py-1 rounded border">Envoyer Axe</button>
          <button @click="zeroMotor('axe')" class="px-3 py-1 rounded border ml-2">Zero Axe</button>
        </div>
      </div>
    </section>

    <section class="mb-6">
      <h2 class="font-semibold">Déplacement multi-moteurs</h2>
      <div class="grid grid-cols-3 gap-3 mt-2">
        <div>
          <label>X (axe) cm</label>
          <input type="number" v-model.number="multi.x" />
        </div>
        <div>
          <label>A angle moteur 1 (°)</label>
          <input type="number" v-model.number="multi.a" />
        </div>
        <div>
          <label>B angle moteur 2 (°)</label>
          <input type="number" v-model.number="multi.b" />
        </div>
      </div>
      <div class="mt-3">
        <button @click="moveMulti()" class="px-3 py-1 rounded border">Déplacer</button>
      </div>
    </section>

    <section class="mb-6">
      <h2 class="font-semibold">Coordonnées (X,Y,Z)</h2>
      <div class="grid grid-cols-3 gap-3 mt-2">
        <div>
          <label>X (cm)</label>
          <input type="number" v-model.number="coord.x" />
        </div>
        <div>
          <label>Y (cm)</label>
          <input type="number" v-model.number="coord.y" />
        </div>
        <div>
          <label>Z (cm)</label>
          <input type="number" v-model.number="coord.z" />
        </div>
      </div>
      <div class="mt-3">
        <button @click="sendCoord()" class="px-3 py-1 rounded border">Aller à coordonnées</button>
      </div>
    </section>

    <section class="mb-6">
      <h2 class="font-semibold">Logs / Etat</h2>
      <pre class="mt-2 p-3 border h-40 overflow-auto">{{ logs.join('\n') }}</pre>
    </section>
  </div>
</template>

<script setup>
import { ref } from 'vue'

// Base URL de l'API
// Priorité: VITE_API_BASE (ex: http://pi:5000) > window.__BRAS_API_BASE__ > "" (utilise proxy Vite en dev)
const apiBase = ref(import.meta.env.VITE_API_BASE || window.__BRAS_API_BASE__ || '')

const angle1 = ref(0)
const angle2 = ref(0)
const axeDistance = ref(0)
const multi = ref({ x: 0, a: 0, b: 0 })
const coord = ref({ x: 0, y: 0, z: 0 })
const logs = ref([])
const status = ref({ ok: true, msg: '' })

function log(s){
  const t = new Date().toLocaleTimeString()
  logs.value.unshift(`[${t}] ${s}`)
  if(logs.value.length>300) logs.value.pop()
}

async function request(path, body){
  const url = (apiBase.value || '') + path
  try{
    const resp = await fetch(url, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body)
    })
    const data = await resp.json().catch(()=>({}))
    if(!resp.ok){
      log(`ERREUR ${resp.status} ${path} ${JSON.stringify(data)}`)
      status.value = { ok:false, msg:`Erreur ${resp.status}` }
      return { ok:false, data }
    }
    log(`OK ${path} -> ${JSON.stringify(data)}`)
    status.value = { ok:true, msg: data.message || 'ok' }
    return { ok:true, data }
  }catch(e){
    log(`EXCEPTION ${path} ${e}`)
    status.value = { ok:false, msg: e.message }
    return { ok:false, data: null }
  }
}

async function setMagnet(state){
  await request('/api/magnet', { state })
}

async function sendAngle(motor, angle){
  await request('/api/motor/angle', { motor, angle })
}

async function sendAxe(distance){
  await request('/api/motor/axe', { distance })
}

async function moveMulti(){
  await request('/api/move', { x: multi.value.x, a: multi.value.a, b: multi.value.b })
}

async function sendCoord(){
  await request('/api/coord', { x: coord.value.x, y: coord.value.y, z: coord.value.z })
}

async function zeroMotor(which){
  await request('/api/zero', { motor: which })
}
</script>

<style scoped>
input[type="range"]{ width:100% }
input[type="number"]{ width:100% }
button{ cursor:pointer }
</style>
