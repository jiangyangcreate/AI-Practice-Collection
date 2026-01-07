# 垃圾识别系统

一个基于Vue前端、FastAPI后端和Langchain算法端的垃圾识别系统。

## 项目结构

```
.
├── frontend/          # Vue前端
│   ├── src/
│   │   ├── views/     # 页面组件
│   │   │   ├── Login.vue          # 登录页面
│   │   │   └── Classification.vue # 垃圾识别页面
│   │   ├── App.vue    # 根组件
│   │   └── main.js    # 入口文件
│   ├── index.html
│   ├── vite.config.js
│   └── package.json
├── backend/           # FastAPI后端
│   ├── main.py        # 主应用文件
│   └── requirements.txt
├── algorithm/         # Langchain算法端
│   ├── classifier.py  # 图像分类模块
│   └── requirements.txt
└── README.md
```

## 功能特性

### 前端（Vue）
- ✅ 邮箱登录页面
- ✅ 垃圾识别页面
- ✅ 支持移动端摄像头拍照
- ✅ 支持上传本地图片
- ✅ 图像预览功能
- ✅ Base64编码传输
- ✅ 自适应布局

### 后端（FastAPI）
- ✅ 用户登录接口
- ✅ 图像接收和验证接口
- ✅ 登录状态验证
- ✅ 图像格式验证
- ✅ CORS跨域支持

### 算法端（Langchain）
- ✅ 图像分类函数封装
- ✅ 大模型工具调用接口
- ✅ 异步处理支持

## 安装和运行

### 快速启动（Windows）

1. **启动后端**：双击 `start_backend.bat` 或在命令行运行
2. **启动前端**：双击 `start_frontend.bat` 或在命令行运行

### 手动启动

#### 1. 前端设置

```bash
cd frontend
npm install
npm run dev
```

前端将在 `http://localhost:3000` 运行

#### 2. 后端设置

```bash
cd backend
pip install -r requirements.txt
python main.py
```

后端将在 `http://localhost:8000` 运行

#### 3. 算法端设置

```bash
cd algorithm
pip install -r requirements.txt
```

**注意**：算法端的依赖需要与后端一起安装，因为后端会调用算法端的模块。

## 使用说明

### 1. 登录系统
- 访问 `http://localhost:3000`
- 输入邮箱地址
- 点击登录

### 2. 识别垃圾
- 登录成功后自动跳转到识别页面
- 点击"拍照"按钮使用摄像头拍摄
- 或点击"上传图片"选择本地图片
- 点击"识别"按钮进行识别

### 3. 自定义大模型工具

在 `algorithm/classifier.py` 中的 `_call_custom_llm` 方法中实现您自己的大模型调用逻辑：

```python
def _call_custom_llm(self, image_base64: str) -> str:
    # 1. 解码图像
    image_data = base64.b64decode(image_base64)
    image = Image.open(BytesIO(image_data))
    
    # 2. 调用您的大模型API
    # response = your_llm_api.analyze_image(image)
    
    # 3. 返回JSON格式结果
    return '{"category": "可回收垃圾", "description": "..."}'
```

## API接口

### POST /api/auth/login
用户登录

**请求体：**
```json
{
  "email": "user@example.com"
}
```

**响应：**
```json
{
  "success": true,
  "message": "登录成功",
  "email": "user@example.com"
}
```

### POST /api/classify
垃圾识别

**请求体：**
```json
{
  "image": "base64编码的图像数据"
}
```

**响应：**
```json
{
  "success": true,
  "result": {
    "category": "可回收垃圾",
    "description": "这是一个塑料瓶",
    "confidence": 0.95
  }
}
```

## 注意事项

1. **身份验证**：当前后端使用简单的会话存储，生产环境应使用JWT token
2. **大模型集成**：需要在 `algorithm/classifier.py` 中实现实际的大模型调用逻辑
3. **图像大小限制**：当前限制为10MB
4. **CORS配置**：后端已配置CORS，允许前端跨域请求

## 技术栈

- **前端**：Vue 3 + Vue Router + Axios + Vite
- **后端**：FastAPI + Uvicorn
- **算法端**：Langchain + Pillow

## 开发计划

- [ ] 实现JWT token身份验证
- [ ] 添加图像预处理功能
- [ ] 支持批量识别
- [ ] 添加识别历史记录
- [ ] 优化移动端体验

