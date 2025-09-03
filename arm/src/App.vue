<template>
  <div>
    <header class="p-4 border-b">
      <button @click="tab = 'ui'">UI Contrôle</button>
      <button @click="tab = 'keyboard'" class="ml-2">Contrôle Clavier</button>
    </header>

    <main class="p-4">
      <BrassControl v-if="tab === 'ui'" />
      <KeyboardControl v-if="tab === 'keyboard'" />
    </main>
  </div>
</template>

<script setup>
import { ref } from "vue";
import BrassControl from "./components/BrassControl.vue";
import KeyboardControl from "./components/KeyboardControl.vue";
const tab = ref("ui");
</script>
