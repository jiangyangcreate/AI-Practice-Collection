<template>
  <div class="login-page">
    <div class="login-container">
      <!-- Logo & Brand -->
      <div class="brand" :class="{ 'brand-animate': mounted }">
        <div class="logo">
          <svg viewBox="0 0 48 48" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M24 4C12.954 4 4 12.954 4 24s8.954 20 20 20 20-8.954 20-20S35.046 4 24 4z" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
            <path d="M17 20c0-3.866 3.134-7 7-7s7 3.134 7 7" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
            <path d="M14 28h20M18 34h12" stroke="currentColor" stroke-width="2" stroke-linecap="round"/>
            <circle cx="24" cy="24" r="3" fill="currentColor"/>
          </svg>
        </div>
        <h1 class="brand-name">EcoScan</h1>
        <p class="brand-tagline">智能垃圾识别系统</p>
      </div>

      <!-- Login Card -->
      <div class="login-card" :class="{ 'card-animate': mounted }">
        <div class="card-header">
          <h2>欢迎回来</h2>
          <p>使用邮箱验证码登录</p>
        </div>

        <form @submit.prevent="handleLogin" class="login-form">
          <!-- Email Input -->
          <div class="form-field">
            <label for="email">
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <path d="M4 4h16c1.1 0 2 .9 2 2v12c0 1.1-.9 2-2 2H4c-1.1 0-2-.9-2-2V6c0-1.1.9-2 2-2z"/>
                <polyline points="22,6 12,13 2,6"/>
              </svg>
              电子邮箱
            </label>
            <div class="input-wrapper">
              <input
                id="email"
                v-model="email"
                type="email"
                placeholder="your@email.com"
                required
                :disabled="loading"
                @focus="emailFocused = true"
                @blur="emailFocused = false"
              />
              <div class="input-glow" :class="{ active: emailFocused }"></div>
            </div>
          </div>

          <!-- Verification Code Input -->
          <div class="form-field">
            <label for="code">
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <rect x="3" y="11" width="18" height="11" rx="2" ry="2"/>
                <path d="M7 11V7a5 5 0 0 1 10 0v4"/>
              </svg>
              验证码
            </label>
            <div class="code-input-row">
              <div class="input-wrapper code-input-wrapper">
                <input
                  id="code"
                  v-model="code"
                  type="text"
                  placeholder="000000"
                  maxlength="6"
                  required
                  :disabled="loading"
                  @focus="codeFocused = true"
                  @blur="codeFocused = false"
                />
                <div class="input-glow" :class="{ active: codeFocused }"></div>
              </div>
              <button
                type="button"
                class="btn-send-code"
                :class="{ sending: sendingCode, disabled: !email || countdown > 0 }"
                :disabled="!email || countdown > 0 || sendingCode"
                @click="sendVerificationCode"
              >
                <span v-if="sendingCode" class="loader"></span>
                <span v-else>{{ countdown > 0 ? `${countdown}s` : '发送' }}</span>
              </button>
            </div>
          </div>

          <!-- Submit Button -->
          <button
            type="submit"
            class="btn-submit"
            :class="{ loading }"
            :disabled="loading || !code || !email"
          >
            <span class="btn-content">
              <span v-if="loading" class="loader"></span>
              <template v-else>
                <span>登录</span>
                <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                  <path d="M5 12h14M12 5l7 7-7 7"/>
                </svg>
              </template>
            </span>
            <div class="btn-shine"></div>
          </button>
        </form>

        <div class="card-footer">
          <p>登录即表示同意 <a href="#">服务条款</a> 和 <a href="#">隐私政策</a></p>
        </div>
      </div>

      <!-- Decorative elements -->
      <div class="floating-icons">
        <span class="float-icon" style="--delay: 0s; --x: 10%; --y: 20%;">♻️</span>
        <span class="float-icon" style="--delay: 2s; --x: 85%; --y: 15%;">🌱</span>
        <span class="float-icon" style="--delay: 4s; --x: 15%; --y: 75%;">🌍</span>
        <span class="float-icon" style="--delay: 6s; --x: 80%; --y: 80%;">🍃</span>
      </div>
    </div>

    <!-- Toast notification -->
    <transition name="toast">
      <div v-if="toast.show" class="toast" :class="toast.type">
        <span class="toast-icon">{{ toast.type === 'success' ? '✓' : '!' }}</span>
        <span>{{ toast.message }}</span>
      </div>
    </transition>
  </div>
</template>

<script>
import axios from 'axios'

export default {
  name: 'Login',
  data() {
    return {
      email: '',
      code: '',
      loading: false,
      sendingCode: false,
      countdown: 0,
      countdownTimer: null,
      emailFocused: false,
      codeFocused: false,
      mounted: false,
      toast: {
        show: false,
        message: '',
        type: 'success'
      }
    }
  },
  mounted() {
    setTimeout(() => {
      this.mounted = true
    }, 100)
  },
  beforeUnmount() {
    if (this.countdownTimer) {
      clearInterval(this.countdownTimer)
    }
  },
  methods: {
    showToast(message, type = 'success') {
      this.toast = { show: true, message, type }
      setTimeout(() => {
        this.toast.show = false
      }, 3000)
    },
    
    /**
     * 请求发送验证码
     * 独立的API请求，只负责发送验证码到用户邮箱
     */
    async sendVerificationCode() {
      if (!this.email) {
        this.showToast('请先输入邮箱地址', 'error')
        return
      }
      
      const emailRegex = /^[^\s@]+@[^\s@]+\.[^\s@]+$/
      if (!emailRegex.test(this.email)) {
        this.showToast('请输入有效的邮箱地址', 'error')
        return
      }
      
      this.sendingCode = true
      try {
        // 请求发送验证码
        const response = await axios.post('/api/auth/send-code', {
          email: this.email
        })
        
        if (response.data.success) {
          this.showToast('验证码已发送，请查收邮箱', 'success')
          this.startCountdown()
        } else {
          this.showToast(response.data.message || '发送失败', 'error')
        }
      } catch (error) {
        console.error('发送验证码错误:', error)
        this.showToast('发送失败，请重试', 'error')
      } finally {
        this.sendingCode = false
      }
    },
    
    startCountdown() {
      this.countdown = 60
      if (this.countdownTimer) clearInterval(this.countdownTimer)
      this.countdownTimer = setInterval(() => {
        this.countdown--
        if (this.countdown <= 0) {
          clearInterval(this.countdownTimer)
          this.countdownTimer = null
        }
      }, 1000)
    },
    
    /**
     * 请求登录
     * 独立的API请求，使用邮箱和验证码进行登录验证
     */
    async handleLogin() {
      if (!this.email) {
        this.showToast('请输入邮箱地址', 'error')
        return
      }
      
      if (!this.code) {
        this.showToast('请输入验证码', 'error')
        return
      }
      
      this.loading = true
      try {
        // 请求登录
        const response = await axios.post('/api/auth/login', {
          email: this.email,
          code: this.code
        })
        
        if (response.data.success) {
          localStorage.setItem('isLoggedIn', 'true')
          localStorage.setItem('email', this.email)
          this.$router.push('/classification')
        } else {
          this.showToast(response.data.message || '登录失败', 'error')
        }
      } catch (error) {
        console.error('登录错误:', error)
        if (error.response?.status === 401) {
          this.showToast('验证码错误', 'error')
        } else {
          this.showToast('登录失败，请重试', 'error')
        }
      } finally {
        this.loading = false
      }
    }
  }
}
</script>

<style scoped>
.login-page {
  min-height: 100vh;
  display: flex;
  align-items: center;
  justify-content: center;
  padding: 24px;
  position: relative;
}

.login-container {
  width: 100%;
  max-width: 420px;
  position: relative;
  z-index: 1;
}

/* Brand Section */
.brand {
  text-align: center;
  margin-bottom: 32px;
  opacity: 0;
  transform: translateY(-20px);
  transition: all 0.6s cubic-bezier(0.4, 0, 0.2, 1);
}

.brand.brand-animate {
  opacity: 1;
  transform: translateY(0);
}

.logo {
  width: 64px;
  height: 64px;
  margin: 0 auto 16px;
  color: var(--color-primary);
  animation: logoPulse 3s ease-in-out infinite;
}

@keyframes logoPulse {
  0%, 100% { transform: scale(1); filter: drop-shadow(0 0 8px rgba(52, 211, 153, 0.3)); }
  50% { transform: scale(1.05); filter: drop-shadow(0 0 20px rgba(52, 211, 153, 0.5)); }
}

.brand-name {
  font-size: 32px;
  font-weight: 700;
  color: var(--color-text);
  letter-spacing: -0.5px;
  margin-bottom: 4px;
}

.brand-tagline {
  font-size: 14px;
  color: var(--color-text-muted);
  letter-spacing: 2px;
  text-transform: uppercase;
}

/* Login Card */
.login-card {
  background: var(--color-bg-card);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-xl);
  padding: 40px 32px;
  backdrop-filter: blur(20px);
  box-shadow: var(--shadow-card), var(--shadow-glow);
  opacity: 0;
  transform: translateY(20px);
  transition: all 0.6s cubic-bezier(0.4, 0, 0.2, 1) 0.2s;
}

.login-card.card-animate {
  opacity: 1;
  transform: translateY(0);
}

.card-header {
  text-align: center;
  margin-bottom: 32px;
}

.card-header h2 {
  font-size: 24px;
  font-weight: 600;
  color: var(--color-text);
  margin-bottom: 8px;
}

.card-header p {
  font-size: 14px;
  color: var(--color-text-muted);
}

/* Form Fields */
.login-form {
  display: flex;
  flex-direction: column;
  gap: 24px;
}

.form-field {
  display: flex;
  flex-direction: column;
  gap: 10px;
}

.form-field label {
  display: flex;
  align-items: center;
  gap: 8px;
  font-size: 13px;
  font-weight: 500;
  color: var(--color-text-muted);
  text-transform: uppercase;
  letter-spacing: 0.5px;
}

.form-field label svg {
  opacity: 0.6;
}

.input-wrapper {
  position: relative;
}

.input-wrapper input {
  width: 100%;
  padding: 16px 18px;
  background: var(--color-bg-input);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-md);
  font-family: var(--font-family);
  font-size: 16px;
  color: var(--color-text);
  transition: all var(--transition-base);
}

.input-wrapper input::placeholder {
  color: var(--color-text-muted);
  opacity: 0.5;
}

.input-wrapper input:focus {
  outline: none;
  border-color: var(--color-border-focus);
  background: rgba(10, 15, 13, 0.8);
}

.input-wrapper input:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.input-glow {
  position: absolute;
  inset: -1px;
  border-radius: var(--radius-md);
  background: linear-gradient(135deg, var(--color-primary), var(--color-accent));
  opacity: 0;
  z-index: -1;
  transition: opacity var(--transition-base);
  filter: blur(8px);
}

.input-glow.active {
  opacity: 0.3;
}

/* Code Input Row */
.code-input-row {
  display: flex;
  gap: 12px;
}

.code-input-wrapper {
  flex: 1;
}

.code-input-wrapper input {
  letter-spacing: 8px;
  font-weight: 600;
  text-align: center;
}

.btn-send-code {
  min-width: 80px;
  padding: 16px 20px;
  background: var(--color-primary-dim);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-md);
  font-family: var(--font-family);
  font-size: 14px;
  font-weight: 600;
  color: var(--color-primary);
  cursor: pointer;
  transition: all var(--transition-base);
  display: flex;
  align-items: center;
  justify-content: center;
}

.btn-send-code:hover:not(:disabled) {
  background: rgba(52, 211, 153, 0.2);
  border-color: var(--color-border-focus);
}

.btn-send-code.disabled,
.btn-send-code:disabled {
  opacity: 0.4;
  cursor: not-allowed;
}

/* Submit Button */
.btn-submit {
  position: relative;
  width: 100%;
  padding: 18px 24px;
  margin-top: 8px;
  background: linear-gradient(135deg, var(--color-primary) 0%, var(--color-accent) 100%);
  border: none;
  border-radius: var(--radius-md);
  font-family: var(--font-family);
  font-size: 16px;
  font-weight: 600;
  color: var(--color-bg);
  cursor: pointer;
  overflow: hidden;
  transition: all var(--transition-base);
}

.btn-submit:hover:not(:disabled) {
  transform: translateY(-2px);
  box-shadow: 0 8px 30px rgba(52, 211, 153, 0.4);
}

.btn-submit:active:not(:disabled) {
  transform: translateY(0);
}

.btn-submit:disabled {
  opacity: 0.5;
  cursor: not-allowed;
  transform: none;
}

.btn-content {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  position: relative;
  z-index: 1;
}

.btn-shine {
  position: absolute;
  inset: 0;
  background: linear-gradient(
    90deg,
    transparent 0%,
    rgba(255, 255, 255, 0.2) 50%,
    transparent 100%
  );
  transform: translateX(-100%);
  animation: shine 3s infinite;
}

@keyframes shine {
  0% { transform: translateX(-100%); }
  20%, 100% { transform: translateX(100%); }
}

/* Loader */
.loader {
  width: 18px;
  height: 18px;
  border: 2px solid transparent;
  border-top-color: currentColor;
  border-radius: 50%;
  animation: spin 0.8s linear infinite;
}

@keyframes spin {
  to { transform: rotate(360deg); }
}

/* Card Footer */
.card-footer {
  margin-top: 24px;
  text-align: center;
}

.card-footer p {
  font-size: 12px;
  color: var(--color-text-muted);
}

.card-footer a {
  color: var(--color-primary);
  text-decoration: none;
  transition: opacity var(--transition-base);
}

.card-footer a:hover {
  opacity: 0.8;
}

/* Floating Icons */
.floating-icons {
  position: fixed;
  inset: 0;
  pointer-events: none;
  z-index: 0;
}

.float-icon {
  position: absolute;
  font-size: 24px;
  left: var(--x);
  top: var(--y);
  animation: float 6s ease-in-out infinite;
  animation-delay: var(--delay);
  opacity: 0.4;
}

@keyframes float {
  0%, 100% { transform: translateY(0) rotate(0deg); opacity: 0.4; }
  50% { transform: translateY(-20px) rotate(10deg); opacity: 0.6; }
}

/* Toast */
.toast {
  position: fixed;
  bottom: 32px;
  left: 50%;
  transform: translateX(-50%);
  padding: 14px 24px;
  background: var(--color-bg-card);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-lg);
  backdrop-filter: blur(20px);
  display: flex;
  align-items: center;
  gap: 12px;
  font-size: 14px;
  font-weight: 500;
  z-index: 1000;
  box-shadow: var(--shadow-card);
}

.toast.success {
  border-color: var(--color-primary);
}

.toast.success .toast-icon {
  color: var(--color-primary);
}

.toast.error {
  border-color: var(--color-danger);
}

.toast.error .toast-icon {
  color: var(--color-danger);
}

.toast-icon {
  width: 24px;
  height: 24px;
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
  font-weight: 700;
}

.toast-enter-active,
.toast-leave-active {
  transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
}

.toast-enter-from,
.toast-leave-to {
  opacity: 0;
  transform: translateX(-50%) translateY(20px);
}

/* Responsive */
@media (max-width: 480px) {
  .login-card {
    padding: 32px 24px;
  }
  
  .brand-name {
    font-size: 28px;
  }
  
  .code-input-row {
    flex-direction: column;
  }
  
  .btn-send-code {
    width: 100%;
  }
}
</style>
