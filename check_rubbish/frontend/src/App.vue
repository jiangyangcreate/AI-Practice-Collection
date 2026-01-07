<template>
  <div class="app-wrapper">
    <div class="bg-grid"></div>
    <div class="bg-glow bg-glow-1"></div>
    <div class="bg-glow bg-glow-2"></div>
    <div class="bg-glow bg-glow-3"></div>
    <router-view v-slot="{ Component }">
      <transition name="page" mode="out-in">
        <component :is="Component" />
      </transition>
    </router-view>
  </div>
</template>

<script>
export default {
  name: 'App'
}
</script>

<style>
:root {
  --color-bg: #0a0f0d;
  --color-bg-card: rgba(16, 24, 20, 0.85);
  --color-bg-input: rgba(10, 15, 13, 0.6);
  --color-border: rgba(52, 211, 153, 0.15);
  --color-border-focus: rgba(52, 211, 153, 0.5);
  --color-primary: #34d399;
  --color-primary-dim: rgba(52, 211, 153, 0.1);
  --color-accent: #10b981;
  --color-text: #ecfdf5;
  --color-text-muted: rgba(236, 253, 245, 0.6);
  --color-danger: #f87171;
  --color-danger-dim: rgba(248, 113, 113, 0.1);
  --font-family: 'Plus Jakarta Sans', -apple-system, BlinkMacSystemFont, sans-serif;
  --radius-sm: 8px;
  --radius-md: 12px;
  --radius-lg: 20px;
  --radius-xl: 28px;
  --shadow-glow: 0 0 60px rgba(52, 211, 153, 0.15);
  --shadow-card: 0 8px 32px rgba(0, 0, 0, 0.4);
  --transition-base: 0.3s cubic-bezier(0.4, 0, 0.2, 1);
}

* {
  margin: 0;
  padding: 0;
  box-sizing: border-box;
}

html, body {
  font-family: var(--font-family);
  background: var(--color-bg);
  color: var(--color-text);
  min-height: 100vh;
  overflow-x: hidden;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
}

#app {
  min-height: 100vh;
  position: relative;
  z-index: 1;
}

.app-wrapper {
  min-height: 100vh;
  position: relative;
}

/* Animated grid background */
.bg-grid {
  position: fixed;
  inset: 0;
  background-image: 
    linear-gradient(rgba(52, 211, 153, 0.03) 1px, transparent 1px),
    linear-gradient(90deg, rgba(52, 211, 153, 0.03) 1px, transparent 1px);
  background-size: 60px 60px;
  mask-image: radial-gradient(ellipse at center, black 0%, transparent 70%);
  animation: gridPulse 8s ease-in-out infinite;
  z-index: 0;
}

@keyframes gridPulse {
  0%, 100% { opacity: 0.4; }
  50% { opacity: 0.8; }
}

/* Floating glow orbs */
.bg-glow {
  position: fixed;
  border-radius: 50%;
  filter: blur(80px);
  opacity: 0.4;
  z-index: 0;
  animation: floatGlow 20s ease-in-out infinite;
}

.bg-glow-1 {
  width: 500px;
  height: 500px;
  background: radial-gradient(circle, rgba(52, 211, 153, 0.3) 0%, transparent 70%);
  top: -150px;
  right: -100px;
  animation-delay: 0s;
}

.bg-glow-2 {
  width: 400px;
  height: 400px;
  background: radial-gradient(circle, rgba(16, 185, 129, 0.25) 0%, transparent 70%);
  bottom: -100px;
  left: -100px;
  animation-delay: -7s;
}

.bg-glow-3 {
  width: 300px;
  height: 300px;
  background: radial-gradient(circle, rgba(5, 150, 105, 0.2) 0%, transparent 70%);
  top: 40%;
  left: 50%;
  transform: translateX(-50%);
  animation-delay: -14s;
}

@keyframes floatGlow {
  0%, 100% { transform: translate(0, 0) scale(1); }
  25% { transform: translate(30px, -30px) scale(1.05); }
  50% { transform: translate(-20px, 20px) scale(0.95); }
  75% { transform: translate(20px, 30px) scale(1.02); }
}

/* Page transitions */
.page-enter-active,
.page-leave-active {
  transition: all 0.4s cubic-bezier(0.4, 0, 0.2, 1);
}

.page-enter-from {
  opacity: 0;
  transform: translateY(20px);
}

.page-leave-to {
  opacity: 0;
  transform: translateY(-20px);
}

/* Scrollbar */
::-webkit-scrollbar {
  width: 6px;
}

::-webkit-scrollbar-track {
  background: transparent;
}

::-webkit-scrollbar-thumb {
  background: var(--color-border);
  border-radius: 3px;
}

::-webkit-scrollbar-thumb:hover {
  background: var(--color-border-focus);
}
</style>
