<template>
  <div class="classify-page">
    <header class="header" :class="{ 'header-animate': mounted }">
      <div class="header-left">
        <div class="logo-mini">
          <svg viewBox="0 0 48 48" fill="none">
            <path d="M24 4C12.954 4 4 12.954 4 24s8.954 20 20 20 20-8.954 20-20S35.046 4 24 4z" stroke="currentColor" stroke-width="2"/>
            <circle cx="24" cy="24" r="3" fill="currentColor"/>
          </svg>
        </div>
        <div class="header-title">
          <h1>EcoScan</h1>
          <span class="header-badge">智能识别</span>
        </div>
      </div>
      <div class="header-right">
        <div class="user-info">
          <span class="user-email">{{ userEmail }}</span>
        </div>
        <button @click="handleLogout" class="btn-logout">
          <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
            <path d="M9 21H5a2 2 0 0 1-2-2V5a2 2 0 0 1 2-2h4"/>
            <polyline points="16 17 21 12 16 7"/>
            <line x1="21" y1="12" x2="9" y2="12"/>
          </svg>
        </button>
      </div>
    </header>

    <main class="main-content">
      <!-- Camera/Upload Section -->
      <section class="capture-section" :class="{ 'section-animate': mounted }">
        <!-- Camera View -->
        <div v-if="cameraActive" class="camera-view">
          <video ref="videoElement" autoplay playsinline></video>
          <div class="camera-overlay">
            <div class="scan-frame">
              <span class="corner corner-tl"></span>
              <span class="corner corner-tr"></span>
              <span class="corner corner-bl"></span>
              <span class="corner corner-br"></span>
            </div>
            <p class="scan-hint">将物品置于框内</p>
          </div>
          <div class="camera-actions">
            <button @click="stopCamera" class="btn-camera-action btn-cancel">
              <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <line x1="18" y1="6" x2="6" y2="18"/><line x1="6" y1="6" x2="18" y2="18"/>
              </svg>
            </button>
            <button @click="capturePhoto" class="btn-camera-action btn-capture">
              <div class="capture-ring"></div>
            </button>
            <button class="btn-camera-action btn-placeholder"></button>
          </div>
        </div>

        <!-- Preview View -->
        <div v-else-if="imagePreview" class="preview-view">
          <div class="preview-image-wrapper">
            <img :src="imagePreview" alt="预览图片" />
            <button @click="clearImage" class="btn-remove">
              <svg width="20" height="20" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <line x1="18" y1="6" x2="6" y2="18"/><line x1="6" y1="6" x2="18" y2="18"/>
              </svg>
            </button>
          </div>
        </div>

        <!-- Upload Options -->
        <div v-else class="upload-view">
          <input
            ref="fileInput"
            type="file"
            accept="image/*"
            @change="handleFileSelect"
            hidden
          />
          <div class="upload-options">
            <button @click="openCamera" class="upload-option">
              <div class="option-icon">
                <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5">
                  <path d="M23 19a2 2 0 0 1-2 2H3a2 2 0 0 1-2-2V8a2 2 0 0 1 2-2h4l2-3h6l2 3h4a2 2 0 0 1 2 2z"/>
                  <circle cx="12" cy="13" r="4"/>
                </svg>
              </div>
              <span class="option-label">拍照识别</span>
              <span class="option-desc">使用摄像头拍摄</span>
            </button>
            <button @click="openFilePicker" class="upload-option">
              <div class="option-icon">
                <svg width="32" height="32" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="1.5">
                  <path d="M21 15v4a2 2 0 0 1-2 2H5a2 2 0 0 1-2-2v-4"/>
                  <polyline points="17 8 12 3 7 8"/>
                  <line x1="12" y1="3" x2="12" y2="15"/>
                </svg>
              </div>
              <span class="option-label">上传图片</span>
              <span class="option-desc">从相册选择</span>
            </button>
          </div>
          <div class="upload-tips">
            <p>💡 支持 JPG、PNG 格式，建议拍摄清晰的垃圾图片</p>
          </div>
        </div>
      </section>

      <!-- Action Button -->
      <section v-if="imagePreview && !cameraActive" class="action-section" :class="{ 'section-animate': mounted }">
        <button
          @click="handleClassification"
          :disabled="!imageBase64 || loading"
          class="btn-analyze"
          :class="{ loading }"
        >
          <span class="btn-content">
            <template v-if="loading">
              <span class="analyze-loader"></span>
              <span>分析中...</span>
            </template>
            <template v-else>
              <svg width="24" height="24" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <circle cx="11" cy="11" r="8"/>
                <line x1="21" y1="21" x2="16.65" y2="16.65"/>
              </svg>
              <span>开始识别</span>
            </template>
          </span>
          <div class="btn-glow"></div>
        </button>
      </section>

      <!-- Result Section -->
      <transition name="result">
        <section v-if="result" class="result-section">
          <div class="result-card">
            <div class="result-header">
              <div class="result-icon">
                📦
              </div>
              <div class="result-title">
                <span class="result-label">识别结果</span>
                <h3>垃圾分类结果</h3>
              </div>
            </div>
            <div class="result-body">
              <div class="result-markdown" v-if="result.text" v-html="renderMarkdown(result.text)"></div>
            </div>
            <button @click="resetAll" class="btn-retry">
              <svg width="18" height="18" viewBox="0 0 24 24" fill="none" stroke="currentColor" stroke-width="2">
                <polyline points="23 4 23 10 17 10"/>
                <path d="M20.49 15a9 9 0 1 1-2.12-9.36L23 10"/>
              </svg>
              重新识别
            </button>
          </div>
        </section>
      </transition>
    </main>

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
import { marked } from 'marked'

// 配置 marked 选项
marked.setOptions({
  breaks: true, // 支持 GitHub 风格的换行
  gfm: true, // 启用 GitHub 风格的 Markdown
})

export default {
  name: 'Classification',
  data() {
    return {
      imageBase64: null,
      imagePreview: null,
      loading: false,
      result: null,
      cameraActive: false,
      stream: null,
      mounted: false,
      toast: {
        show: false,
        message: '',
        type: 'success'
      }
    }
  },
  computed: {
    userEmail() {
      const email = localStorage.getItem('email') || ''
      return email.length > 20 ? email.substring(0, 20) + '...' : email
    }
  },
  mounted() {
    setTimeout(() => {
      this.mounted = true
    }, 100)
  },
  beforeUnmount() {
    this.stopCamera()
  },
  methods: {
    showToast(message, type = 'success') {
      this.toast = { show: true, message, type }
      setTimeout(() => {
        this.toast.show = false
      }, 3000)
    },

    /**
     * 将 Markdown 文本渲染为 HTML
     */
    renderMarkdown(markdown) {
      if (!markdown) return ''
      try {
        // 使用 marked.parse() 进行同步解析
        return marked.parse(markdown)
      } catch (error) {
        console.error('Markdown 渲染错误:', error)
        return markdown // 出错时返回原始文本
      }
    },


    async openCamera() {
      try {
        const stream = await navigator.mediaDevices.getUserMedia({
          video: {
            facingMode: 'environment',
            width: { ideal: 1920 },
            height: { ideal: 1080 }
          }
        })
        
        this.stream = stream
        this.cameraActive = true
        this.$nextTick(() => {
          if (this.$refs.videoElement) {
            this.$refs.videoElement.srcObject = stream
          }
        })
      } catch (error) {
        console.error('无法访问摄像头:', error)
        if (error.name === 'NotAllowedError') {
          this.showToast('摄像头权限被拒绝', 'error')
        } else if (error.name === 'NotFoundError') {
          this.showToast('未找到摄像头设备', 'error')
        } else {
          this.showToast('无法访问摄像头', 'error')
        }
      }
    },
    
    stopCamera() {
      if (this.stream) {
        this.stream.getTracks().forEach(track => track.stop())
        this.stream = null
      }
      this.cameraActive = false
      if (this.$refs.videoElement) {
        this.$refs.videoElement.srcObject = null
      }
    },
    
    capturePhoto() {
      if (!this.$refs.videoElement) return
      
      const video = this.$refs.videoElement
      const canvas = document.createElement('canvas')
      canvas.width = video.videoWidth
      canvas.height = video.videoHeight
      
      const ctx = canvas.getContext('2d')
      ctx.drawImage(video, 0, 0, canvas.width, canvas.height)
      
      const dataURL = canvas.toDataURL('image/jpeg', 0.9)
      this.imagePreview = dataURL
      this.imageBase64 = dataURL.split(',')[1]
      
      this.stopCamera()
      this.showToast('拍摄成功', 'success')
    },
    
    openFilePicker() {
      this.$refs.fileInput.click()
    },
    
    handleFileSelect(event) {
      const file = event.target.files[0]
      if (file) {
        const reader = new FileReader()
        reader.onload = (e) => {
          this.imagePreview = e.target.result
          this.imageBase64 = e.target.result.split(',')[1]
        }
        reader.readAsDataURL(file)
      }
    },
    
    clearImage() {
      this.imagePreview = null
      this.imageBase64 = null
      this.result = null
      if (this.$refs.fileInput) {
        this.$refs.fileInput.value = ''
      }
    },

    resetAll() {
      this.clearImage()
    },
    
    async handleClassification() {
      if (!this.imageBase64) {
        this.showToast('请先选择或拍摄图片', 'error')
        return
      }
      
      this.loading = true
      this.result = null
      
      try {
        const email = localStorage.getItem('email')
        // 请求识别接口 /api/classify
        const response = await axios.post('/api/classify', {
          image: this.imageBase64
        }, {
          headers: {
            'Content-Type': 'application/json',
            'X-User-Email': email || ''
          }
        })
        
        if (response.data.success) {
          // 保存返回的结果，包含 text 和 image_path 字段
          this.result = response.data.result
          this.showToast('识别完成', 'success')
        } else {
          this.showToast(response.data.message || '识别失败', 'error')
        }
      } catch (error) {
        console.error('识别错误:', error)
        if (error.response?.status === 401) {
          this.showToast('请重新登录', 'error')
          this.handleLogout()
        } else {
          this.showToast('识别失败，请重试', 'error')
        }
      } finally {
        this.loading = false
      }
    },
    
    handleLogout() {
      localStorage.removeItem('isLoggedIn')
      localStorage.removeItem('email')
      this.$router.push('/login')
    }
  }
}
</script>

<style scoped>
.classify-page {
  min-height: 100vh;
  display: flex;
  flex-direction: column;
}

/* Header */
.header {
  display: flex;
  align-items: center;
  justify-content: space-between;
  padding: 16px 24px;
  background: var(--color-bg-card);
  border-bottom: 1px solid var(--color-border);
  backdrop-filter: blur(20px);
  position: sticky;
  top: 0;
  z-index: 100;
  opacity: 0;
  transform: translateY(-20px);
  transition: all 0.5s cubic-bezier(0.4, 0, 0.2, 1);
}

.header.header-animate {
  opacity: 1;
  transform: translateY(0);
}

.header-left {
  display: flex;
  align-items: center;
  gap: 12px;
}

.logo-mini {
  width: 36px;
  height: 36px;
  color: var(--color-primary);
}

.header-title h1 {
  font-size: 18px;
  font-weight: 700;
  color: var(--color-text);
  line-height: 1;
}

.header-badge {
  font-size: 10px;
  text-transform: uppercase;
  letter-spacing: 1px;
  color: var(--color-text-muted);
}

.header-right {
  display: flex;
  align-items: center;
  gap: 16px;
}

.user-email {
  font-size: 13px;
  color: var(--color-text-muted);
}

.btn-logout {
  width: 40px;
  height: 40px;
  display: flex;
  align-items: center;
  justify-content: center;
  background: var(--color-danger-dim);
  border: 1px solid rgba(248, 113, 113, 0.2);
  border-radius: var(--radius-sm);
  color: var(--color-danger);
  cursor: pointer;
  transition: all var(--transition-base);
}

.btn-logout:hover {
  background: rgba(248, 113, 113, 0.2);
  transform: scale(1.05);
}

/* Main Content */
.main-content {
  flex: 1;
  padding: 24px;
  max-width: 600px;
  margin: 0 auto;
  width: 100%;
}

/* Capture Section */
.capture-section {
  background: var(--color-bg-card);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-xl);
  overflow: hidden;
  backdrop-filter: blur(20px);
  box-shadow: var(--shadow-card);
  opacity: 0;
  transform: translateY(20px);
  transition: all 0.5s cubic-bezier(0.4, 0, 0.2, 1) 0.1s;
}

.capture-section.section-animate {
  opacity: 1;
  transform: translateY(0);
}

/* Camera View */
.camera-view {
  position: relative;
  background: #000;
}

.camera-view video {
  width: 100%;
  display: block;
  aspect-ratio: 4/3;
  object-fit: cover;
}

.camera-overlay {
  position: absolute;
  inset: 0;
  display: flex;
  flex-direction: column;
  align-items: center;
  justify-content: center;
  pointer-events: none;
}

.scan-frame {
  width: 70%;
  aspect-ratio: 1;
  position: relative;
  max-width: 280px;
}

.corner {
  position: absolute;
  width: 24px;
  height: 24px;
  border-color: var(--color-primary);
  border-style: solid;
}

.corner-tl { top: 0; left: 0; border-width: 3px 0 0 3px; border-radius: 4px 0 0 0; }
.corner-tr { top: 0; right: 0; border-width: 3px 3px 0 0; border-radius: 0 4px 0 0; }
.corner-bl { bottom: 0; left: 0; border-width: 0 0 3px 3px; border-radius: 0 0 0 4px; }
.corner-br { bottom: 0; right: 0; border-width: 0 3px 3px 0; border-radius: 0 0 4px 0; }

.scan-hint {
  margin-top: 20px;
  padding: 8px 16px;
  background: rgba(0, 0, 0, 0.6);
  border-radius: var(--radius-lg);
  font-size: 13px;
  color: rgba(255, 255, 255, 0.8);
}

.camera-actions {
  display: flex;
  align-items: center;
  justify-content: space-around;
  padding: 24px;
  background: linear-gradient(to top, rgba(0,0,0,0.8), transparent);
  position: absolute;
  bottom: 0;
  left: 0;
  right: 0;
}

.btn-camera-action {
  border: none;
  cursor: pointer;
  transition: all var(--transition-base);
}

.btn-cancel {
  width: 48px;
  height: 48px;
  background: rgba(255, 255, 255, 0.1);
  border-radius: 50%;
  color: white;
  display: flex;
  align-items: center;
  justify-content: center;
}

.btn-cancel:hover {
  background: rgba(255, 255, 255, 0.2);
}

.btn-capture {
  width: 72px;
  height: 72px;
  background: white;
  border-radius: 50%;
  position: relative;
}

.btn-capture:hover {
  transform: scale(1.05);
}

.btn-capture:active {
  transform: scale(0.95);
}

.capture-ring {
  position: absolute;
  inset: 4px;
  border: 3px solid var(--color-bg);
  border-radius: 50%;
}

.btn-placeholder {
  width: 48px;
  height: 48px;
  background: transparent;
  visibility: hidden;
}

/* Preview View */
.preview-view {
  padding: 24px;
}

.preview-image-wrapper {
  position: relative;
  border-radius: var(--radius-lg);
  overflow: hidden;
}

.preview-image-wrapper img {
  width: 100%;
  display: block;
  border-radius: var(--radius-lg);
}

.btn-remove {
  position: absolute;
  top: 12px;
  right: 12px;
  width: 36px;
  height: 36px;
  background: rgba(0, 0, 0, 0.6);
  border: none;
  border-radius: 50%;
  color: white;
  cursor: pointer;
  display: flex;
  align-items: center;
  justify-content: center;
  transition: all var(--transition-base);
}

.btn-remove:hover {
  background: var(--color-danger);
  transform: scale(1.1);
}

/* Upload View */
.upload-view {
  padding: 32px 24px;
}

.upload-options {
  display: grid;
  grid-template-columns: repeat(2, 1fr);
  gap: 16px;
}

.upload-option {
  display: flex;
  flex-direction: column;
  align-items: center;
  gap: 12px;
  padding: 32px 20px;
  background: var(--color-bg-input);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-lg);
  cursor: pointer;
  transition: all var(--transition-base);
}

.upload-option:hover {
  border-color: var(--color-border-focus);
  background: var(--color-primary-dim);
  transform: translateY(-4px);
  box-shadow: 0 12px 40px rgba(52, 211, 153, 0.15);
}

.option-icon {
  width: 64px;
  height: 64px;
  display: flex;
  align-items: center;
  justify-content: center;
  background: var(--color-primary-dim);
  border-radius: 50%;
  color: var(--color-primary);
  transition: all var(--transition-base);
}

.upload-option:hover .option-icon {
  background: var(--color-primary);
  color: var(--color-bg);
}

.option-label {
  font-size: 16px;
  font-weight: 600;
  color: var(--color-text);
}

.option-desc {
  font-size: 12px;
  color: var(--color-text-muted);
}

.upload-tips {
  margin-top: 24px;
  text-align: center;
}

.upload-tips p {
  font-size: 13px;
  color: var(--color-text-muted);
}

/* Action Section */
.action-section {
  margin-top: 24px;
  opacity: 0;
  transform: translateY(20px);
  transition: all 0.5s cubic-bezier(0.4, 0, 0.2, 1) 0.2s;
}

.action-section.section-animate {
  opacity: 1;
  transform: translateY(0);
}

.btn-analyze {
  width: 100%;
  position: relative;
  padding: 20px 32px;
  background: linear-gradient(135deg, var(--color-primary) 0%, var(--color-accent) 100%);
  border: none;
  border-radius: var(--radius-lg);
  font-family: var(--font-family);
  font-size: 18px;
  font-weight: 600;
  color: var(--color-bg);
  cursor: pointer;
  overflow: hidden;
  transition: all var(--transition-base);
}

.btn-analyze:hover:not(:disabled) {
  transform: translateY(-3px);
  box-shadow: 0 12px 40px rgba(52, 211, 153, 0.4);
}

.btn-analyze:disabled {
  opacity: 0.5;
  cursor: not-allowed;
}

.btn-analyze .btn-content {
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 12px;
  position: relative;
  z-index: 1;
}

.btn-glow {
  position: absolute;
  inset: 0;
  background: linear-gradient(90deg, transparent, rgba(255,255,255,0.3), transparent);
  transform: translateX(-100%);
}

.btn-analyze:not(:disabled):hover .btn-glow {
  animation: btnShine 1s ease;
}

@keyframes btnShine {
  to { transform: translateX(100%); }
}

.analyze-loader {
  width: 20px;
  height: 20px;
  border: 2px solid transparent;
  border-top-color: currentColor;
  border-radius: 50%;
  animation: spin 0.8s linear infinite;
}

@keyframes spin {
  to { transform: rotate(360deg); }
}

/* Result Section */
.result-section {
  margin-top: 24px;
}

.result-card {
  background: var(--color-bg-card);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-xl);
  padding: 24px;
  backdrop-filter: blur(20px);
  box-shadow: var(--shadow-card);
}

.result-header {
  display: flex;
  align-items: center;
  gap: 16px;
  margin-bottom: 20px;
}

.result-icon {
  width: 56px;
  height: 56px;
  display: flex;
  align-items: center;
  justify-content: center;
  border-radius: var(--radius-md);
  font-size: 28px;
}

.result-icon.recyclable { background: rgba(59, 130, 246, 0.2); }
.result-icon.hazardous { background: rgba(239, 68, 68, 0.2); }
.result-icon.kitchen { background: rgba(34, 197, 94, 0.2); }
.result-icon.other { background: rgba(156, 163, 175, 0.2); }

.result-title {
  flex: 1;
}

.result-label {
  font-size: 12px;
  text-transform: uppercase;
  letter-spacing: 1px;
  color: var(--color-text-muted);
}

.result-title h3 {
  font-size: 22px;
  font-weight: 700;
  color: var(--color-text);
  margin-top: 4px;
}

.result-body {
  padding-top: 16px;
  border-top: 1px solid var(--color-border);
}

.result-description {
  font-size: 15px;
  color: var(--color-text);
  line-height: 1.6;
}

.result-markdown {
  margin-top: 8px;
  font-family: var(--font-family);
  font-size: 15px;
  color: var(--color-text);
  line-height: 1.8;
  background: var(--color-bg-input);
  padding: 20px;
  border-radius: var(--radius-md);
  border: 1px solid var(--color-border);
}

.result-markdown :deep(h1),
.result-markdown :deep(h2),
.result-markdown :deep(h3),
.result-markdown :deep(h4),
.result-markdown :deep(h5),
.result-markdown :deep(h6) {
  color: var(--color-text);
  font-weight: 600;
  margin-top: 1.5em;
  margin-bottom: 0.5em;
  line-height: 1.4;
}

.result-markdown :deep(h1) { font-size: 1.8em; }
.result-markdown :deep(h2) { font-size: 1.5em; }
.result-markdown :deep(h3) { font-size: 1.3em; }
.result-markdown :deep(h4) { font-size: 1.1em; }

.result-markdown :deep(p) {
  margin-bottom: 1em;
  color: var(--color-text);
}

.result-markdown :deep(ul),
.result-markdown :deep(ol) {
  margin-bottom: 1em;
  padding-left: 2em;
  color: var(--color-text);
}

.result-markdown :deep(li) {
  margin-bottom: 0.5em;
  color: var(--color-text);
}

.result-markdown :deep(code) {
  background: rgba(52, 211, 153, 0.1);
  color: var(--color-primary);
  padding: 2px 6px;
  border-radius: 4px;
  font-family: 'Courier New', monospace;
  font-size: 0.9em;
}

.result-markdown :deep(pre) {
  background: rgba(10, 15, 13, 0.8);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-md);
  padding: 16px;
  overflow-x: auto;
  margin-bottom: 1em;
}

.result-markdown :deep(pre code) {
  background: transparent;
  color: var(--color-text);
  padding: 0;
}

.result-markdown :deep(blockquote) {
  border-left: 4px solid var(--color-primary);
  padding-left: 16px;
  margin-left: 0;
  margin-bottom: 1em;
  color: var(--color-text-muted);
  font-style: italic;
}

.result-markdown :deep(strong) {
  color: var(--color-text);
  font-weight: 600;
}

.result-markdown :deep(em) {
  color: var(--color-text-muted);
  font-style: italic;
}

.result-markdown :deep(a) {
  color: var(--color-primary);
  text-decoration: none;
}

.result-markdown :deep(a:hover) {
  text-decoration: underline;
}

.result-markdown :deep(hr) {
  border: none;
  border-top: 1px solid var(--color-border);
  margin: 2em 0;
}

.result-markdown :deep(table) {
  width: 100%;
  border-collapse: collapse;
  margin-bottom: 1em;
}

.result-markdown :deep(th),
.result-markdown :deep(td) {
  border: 1px solid var(--color-border);
  padding: 8px 12px;
  text-align: left;
}

.result-markdown :deep(th) {
  background: var(--color-primary-dim);
  color: var(--color-primary);
  font-weight: 600;
}

.result-tips {
  margin-top: 16px;
  padding: 16px;
  background: var(--color-primary-dim);
  border-radius: var(--radius-md);
}

.result-tips h4 {
  font-size: 13px;
  font-weight: 600;
  color: var(--color-primary);
  margin-bottom: 8px;
}

.result-tips p {
  font-size: 14px;
  color: var(--color-text-muted);
  line-height: 1.5;
}

.btn-retry {
  width: 100%;
  display: flex;
  align-items: center;
  justify-content: center;
  gap: 8px;
  margin-top: 20px;
  padding: 14px 24px;
  background: var(--color-bg-input);
  border: 1px solid var(--color-border);
  border-radius: var(--radius-md);
  font-family: var(--font-family);
  font-size: 14px;
  font-weight: 500;
  color: var(--color-text);
  cursor: pointer;
  transition: all var(--transition-base);
}

.btn-retry:hover {
  border-color: var(--color-border-focus);
  background: var(--color-primary-dim);
}

/* Result animation */
.result-enter-active {
  transition: all 0.4s cubic-bezier(0.4, 0, 0.2, 1);
}

.result-leave-active {
  transition: all 0.2s ease-in;
}

.result-enter-from {
  opacity: 0;
  transform: translateY(20px) scale(0.95);
}

.result-leave-to {
  opacity: 0;
  transform: scale(0.95);
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

.toast.success { border-color: var(--color-primary); }
.toast.success .toast-icon { color: var(--color-primary); }
.toast.error { border-color: var(--color-danger); }
.toast.error .toast-icon { color: var(--color-danger); }

.toast-icon {
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
  .main-content {
    padding: 16px;
  }
  
  .header {
    padding: 12px 16px;
  }
  
  .user-email {
    display: none;
  }
  
  .upload-options {
    grid-template-columns: 1fr;
  }
  
  .upload-option {
    padding: 24px 16px;
  }
}
</style>
