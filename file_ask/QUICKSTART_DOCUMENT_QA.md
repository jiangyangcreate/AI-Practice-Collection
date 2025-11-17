# 🚀 快速开始指南

## 5 分钟上手文档问答系统

### 第一步：安装 Ollama

#### Windows

1. 访问 https://ollama.ai/download
2. 下载 Windows 安装包
3. 运行安装程序
4. 验证安装：

```powershell
ollama --version
```

#### macOS

```bash
curl -fsSL https://ollama.ai/install.sh | sh
```

#### Linux

```bash
curl -fsSL https://ollama.ai/install.sh | sh
```

### 第二步：下载 AI 模型

打开终端/命令提示符，运行：

```bash
ollama pull qwen2.5
```

等待模型下载完成（约 1.5GB）。

验证模型：

```bash
ollama list
```

应该看到 `qwen2.5` 在列表中。

### 第三步：安装 Python 依赖

#### 方法 1：使用 uv (推荐)

```bash
# 安装 uv
pip install uv

# 安装项目依赖
uv sync
```

#### 方法 2：使用 pip

```bash
pip install gradio langchain langchain-community langchain-ollama langchain-core ollama pypdf python-dotenv
```

### 第四步：启动应用

```bash
python document_qa.py
```

看到以下输出表示成功：

```
🚀 正在启动文档问答系统...
📌 请确保已安装 Ollama 并下载了 qwen2.5 模型
   安装方法: ollama pull qwen2.5

Running on local URL:  http://0.0.0.0:7860
```

浏览器会自动打开应用界面。

### 第五步：使用系统

1. **上传文档**
   - 点击右侧"上传 PDF 文档"
   - 选择一个 PDF 文件
   - 点击"📥 加载文档"
   - 等待"✅ 成功加载文档"提示

2. **提问**
   - 在左侧输入框输入问题，例如：
     - "这份文档的主要内容是什么？"
     - "请总结文档的关键要点"
   - 点击"🚀 发送"

3. **查看答案**
   - AI 会基于文档内容回答
   - 底部显示参考的文档片段

## 🎯 测试示例

### 获取测试 PDF

如果您手头没有 PDF 文件，可以：

1. **下载示例论文**：
   ```bash
   curl -o example_data/sample.pdf https://arxiv.org/pdf/2301.00234.pdf
   ```

2. **或使用在线转换工具**：
   - 访问 https://www.ilovepdf.com/
   - 将任何文档转换为 PDF

### 示例问题

上传文档后，尝试这些问题：

- "这份文档讨论了什么主题？"
- "有哪些关键发现或结论？"
- "文档中提到的方法论是什么？"
- "请列出文档中的重要数据或统计"

## ⚠️ 常见问题

### 问题 1：找不到 ollama 命令

**解决方法**：
- 重启终端/命令提示符
- 检查 PATH 环境变量
- 重新安装 Ollama

### 问题 2：模型下载速度慢

**解决方法**：
- 使用代理或 VPN
- 检查网络连接
- 耐心等待，模型较大

### 问题 3：端口 7860 被占用

**解决方法**：

修改 `document_qa.py` 中的端口：

```python
demo.launch(
    server_port=8080,  # 改为其他端口
    share=False
)
```

### 问题 4：PDF 加载失败

**检查**：
- PDF 是否包含可提取的文本？
- 文件是否损坏？
- 文件路径是否包含中文或特殊字符？

## 🎉 成功！

现在您可以：

- ✅ 上传任何 PDF 文档
- ✅ 基于文档内容提问
- ✅ 获得 AI 生成的答案
- ✅ 查看参考来源

## 📚 下一步

- 阅读完整文档：[README_DOCUMENT_QA.md](README_DOCUMENT_QA.md)
- 了解进阶功能
- 自定义配置
- 集成到您的项目

---

**遇到问题？** 查看 [README_DOCUMENT_QA.md](README_DOCUMENT_QA.md) 的故障排除部分。

