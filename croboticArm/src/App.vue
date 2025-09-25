<script setup>
import HelloWorld from './components/HelloWorld.vue'
</script>

<template>
  <div class="header-wrap">
    <!-- Titre avec logos insérés entre les deux mots -->
    <h1 class="title">
     
      <span class="logos-inline" aria-hidden="false" role="img" aria-label="Logos ICE et iLab">
        <a href="https://ice-efrei.notion.site/Inscris-toi-27b4310a678b45888cf65792373dd441" target="_blank" rel="noopener noreferrer" class="logo-link">
          <img src="./img/logo_ice.png" class="logo-inline" alt="Logo ICE" />
        </a></span>
         <span class="title-part">Crobotic Arm</span>
        <span>
        <a href="https://ice-efrei.notion.site/Inscris-toi-27b4310a678b45888cf65792373dd441" target="_blank" rel="noopener noreferrer" class="logo-link">
          <img src="./img/logo_ilab.png" class="logo-inline" alt="Logo iLab" />
        </a>

      </span>

    </h1>

    <!-- autres logos / icônes sous le titre -->
    <div class="tech-logos">
      <img src="https://upload.wikimedia.org/wikipedia/commons/thumb/c/c3/Python-logo-notext.svg/165px-Python-logo-notext.svg.png" class="logo2" alt="Python" />
      <img src="https://upload.wikimedia.org/wikipedia/fr/thumb/3/3b/Raspberry_Pi_logo.svg/948px-Raspberry_Pi_logo.svg.png?20171015132127" class="logo2" alt="Raspberry Pi" />
      <img src="https://upload.wikimedia.org/wikipedia/commons/9/97/Kicad_logo_new.png" class="logo2" alt="KiCad" />
      <img src="./img/3d.png" class="logo2" alt="3D" />
      <img src="./img/laser.png" class="logo2" alt="Laser" />
      <img src="https://images.seeklogo.com/logo-png/48/1/autodesk-fusion-360-logo-png_seeklogo-482400.png" class="logo2" alt="Laser" />
    </div>
  </div>

  <HelloWorld msg="Crobotic Arm" />
</template>

<style scoped>
.header-wrap {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 0.6rem;
  padding: 1rem 0;
}

/* Titre principal : Crobotic [logos] Arm */
.title{
  display: flex;
  align-items: center;
  gap: 0.75rem;
  font-size: 2.2rem;
  font-weight: 700;
  margin: 0;
  text-align: center;
  flex-wrap: wrap; /* permet au titre de s'adapter sur petits écrans */
}

.title-part{
  white-space: nowrap;
}

/* conteneur des deux logos entre les mots */
.logos-inline{
  display: flex;
  align-items: center;
  gap: 0.5rem;
}

/* petite taille pour logos inline, centrés verticalement */
.logo-inline{
  height:6rem;
  width: auto;
  display: block;
  will-change: transform, filter;
  transition: transform 180ms, filter 180ms;
}

/* lien autour de chaque logo (zone cliquable un peu plus grande) */
.logo-link{
  display: inline-flex;
  align-items: center;
  padding: 0.15rem;
  border-radius: 6px;
}
.logo-link:hover .logo-inline{
  transform: translateY(-4px);
  filter: drop-shadow(0 6px 12px rgba(0,0,0,0.12));
}

/* logos secondaires en dessous */
.tech-logos{
  display:flex;
  gap: 1rem;
  align-items:center;
  justify-content:center;
  flex-wrap:wrap;
  margin-top: 0.25rem;
}

.logo2{
  height: 3.6rem;
  padding: 0.25rem;
  will-change: filter;
  transition: filter 200ms, transform 120ms;
}
.logo2:hover{
  transform: translateY(-4px);
  filter: drop-shadow(0 6px 14px rgba(0,0,0,0.12));
}

/* responsive : réduire taille titre et logos sur écrans étroits */
@media (max-width: 480px){
  .title{
    font-size: 1.5rem;
    gap: 0.4rem;
  }
  .logo-inline{
    height: 1.8rem;
  }
  .logo2{
    height: 2.4rem;
  }
}
</style>
