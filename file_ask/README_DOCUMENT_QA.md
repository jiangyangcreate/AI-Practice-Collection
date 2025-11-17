# 📚 智能文档问答系统使用指南

## 🎯 项目简介

这是一个基于 **RAG (检索增强生成)** 技术的本地文档问答系统，支持上传 PDF 文档并基于文档内容进行智能问答。

### 核心技术栈

- **Gradio 5.x** - 现代化的 Web UI 框架
- **LangChain 1.0+** - 最新版本的 LangChain 框架
- **Ollama** - 本地大语言模型推理
- **LangChain Vector Store** - 向量数据库 (内存存储)
- **PyPDF** - PDF 文档解析

## 🚀 快速开始

### 1. 环境准备

确保已安装 Python 3.12+：

```bash
python --version
```

### 2. 安装 Ollama

访问 [Ollama 官网](https://ollama.ai/) 下载并安装 Ollama。

安装完成后，拉取所需模型：

```bash
ollama pull qwen2.5
```

### 3. 安装项目依赖

使用 uv (推荐):

```bash
# 安装 uv
pip install uv

# 同步依赖
uv sync
```

或使用 pip:

```bash
pip install gradio langchain langchain-community langchain-ollama langchain-core ollama pypdf python-dotenv
```

### 4. 运行应用

```bash
python document_qa.py
```

应用将在 `http://localhost:7860` 启动并自动在浏览器中打开。

## 📖 使用说明

### 界面布局

应用分为两个主要区域：

#### 左侧 - 💬 智能问答区
- **对话历史**: 显示与 AI 的对话记录
- **输入框**: 输入您关于文档的问题
- **发送按钮**: 提交问题
- **示例问题**: 快速开始的问题模板
- **清空对话**: 清除对话历史记录

#### 右侧 - 📤 文档管理区
- **文件上传**: 选择 PDF 文档
- **加载文档**: 将文档加载到系统
- **上传状态**: 显示已加载的文档信息
- **清空文档**: 删除所有已加载的文档

### 使用流程

1. **上传文档**
   - 点击"上传 PDF 文档"区域
   - 选择您的 PDF 文件
   - 点击"📥 加载文档"按钮

2. **等待处理**
   - 系统会自动解析 PDF 内容
   - 将文档分割为多个文本块
   - 生成向量嵌入并存储

3. **开始提问**
   - 在左侧输入框输入问题
   - 点击"🚀 发送"或按 Enter 键
   - 等待 AI 生成回答

4. **查看结果**
   - AI 会基于文档内容回答
   - 底部显示参考的文档片段
   - 可以继续追问相关问题

## 🔧 核心功能详解

### 1. 文档加载与处理

系统使用 LangChain 的 `PyPDFLoader` 加载 PDF 文档：

```python
from langchain_community.document_loaders import PyPDFLoader

loader = PyPDFLoader(file_path)
documents = loader.load()
```

### 2. 文本分割

使用 `RecursiveCharacterTextSplitter` 将长文档分割为小块：

- **chunk_size**: 1000 字符
- **chunk_overlap**: 200 字符（保证上下文连贯性）

```python
from langchain.text_splitter import RecursiveCharacterTextSplitter

text_splitter = RecursiveCharacterTextSplitter(
    chunk_size=1000,
    chunk_overlap=200,
)
```

### 3. 向量嵌入

使用 Ollama 的 qwen2.5 模型生成文本嵌入：

```python
from langchain_ollama import OllamaEmbeddings

embeddings = OllamaEmbeddings(model="qwen2.5")
```

### 4. 向量存储与检索

使用内存向量存储进行快速检索：

```python
from langchain_core.vectorstores import InMemoryVectorStore

vector_store = InMemoryVectorStore(embedding=embeddings)
# 添加文档
vector_store.add_documents(documents=split_docs)
# 相似度搜索
similar_docs = vector_store.similarity_search(query, k=3)
```

### 5. 答案生成

基于检索到的文档内容，构建提示词并调用 Ollama 生成答案：

```python
from ollama import chat

response = chat(
    model='qwen2.5',
    messages=[{
        'role': 'user',
        'content': prompt,
    }]
)
```

## 🎨 界面特色

- **现代化设计**: 使用 Gradio 5.x 最新主题
- **实时对话**: 流畅的聊天体验
- **可视化反馈**: 清晰的状态提示和错误信息
- **参考来源**: 显示答案的文档出处
- **示例问题**: 帮助用户快速上手

## ⚙️ 自定义配置

### 修改模型

在 `document_qa.py` 中修改模型名称：

```python
qa_system = DocumentQASystem(model_name="llama3.2")  # 更换为其他模型
```

### 调整文本分块大小

```python
qa_system = DocumentQASystem(
    model_name="qwen2.5",
    chunk_size=500,      # 减小块大小
    chunk_overlap=100    # 减小重叠
)
```

### 修改检索数量

在 `answer_question` 方法中调整 `k` 参数：

```python
answer, sources = qa_system.answer_question(query, k=5)  # 检索 5 个相关文档
```

### 更改端口

```python
demo.launch(
    server_port=8080,  # 更改端口
    share=False
)
```

## 📊 系统架构

```
用户上传 PDF
    ↓
PDF 解析 (PyPDFLoader)
    ↓
文本分割 (RecursiveCharacterTextSplitter)
    ↓
生成向量嵌入 (OllamaEmbeddings)
    ↓
存储到向量数据库 (InMemoryVectorStore)
    ↓
用户提问
    ↓
相似度检索 (similarity_search)
    ↓
构建提示词
    ↓
LLM 生成答案 (Ollama)
    ↓
返回答案 + 参考来源
```

## 🔍 故障排除

### 问题 1: 找不到 Ollama 模型

**错误信息**: `Error: model 'qwen2.5' not found`

**解决方法**:
```bash
ollama pull qwen2.5
```

### 问题 2: 连接 Ollama 失败

**错误信息**: `Connection refused`

**解决方法**:
1. 确保 Ollama 服务正在运行
2. 检查默认端口 11434 是否可用

Windows:
```powershell
netstat -ano | findstr :11434
```

### 问题 3: PDF 解析失败

**可能原因**:
- PDF 文件损坏
- PDF 为扫描件（需要 OCR）
- 文件权限问题

**解决方法**:
- 尝试其他 PDF 文件
- 确保 PDF 包含可提取的文本

### 问题 4: 内存不足

**症状**: 上传大文件时系统卡顿

**解决方法**:
- 减小 `chunk_size` 参数
- 使用更小的 PDF 文件
- 考虑使用持久化向量数据库（如 Chroma）

## 🚀 进阶功能

### 1. 支持更多文档格式

添加其他文档加载器：

```python
from langchain_community.document_loaders import (
    TextLoader,      # .txt
    Docx2txtLoader,  # .docx
    UnstructuredMarkdownLoader  # .md
)
```

### 2. 使用持久化向量数据库

替换 `InMemoryVectorStore` 为 `Chroma`:

```python
from langchain_chroma import Chroma

vector_store = Chroma(
    collection_name="documents",
    embedding_function=embeddings,
    persist_directory="./chroma_db"
)
```

### 3. 添加多语言支持

使用支持多语言的模型：

```bash
ollama pull qwen2.5:14b  # 更大的模型，多语言能力更强
```

### 4. 实现对话记忆

保存对话历史并在后续问答中使用：

```python
from langchain.memory import ConversationBufferMemory

memory = ConversationBufferMemory()
```

## 📚 示例问题

针对技术文档：
- "这份文档的主要内容是什么？"
- "请解释文中的 [某个概念]"
- "文档中提到了哪些关键技术？"

针对论文：
- "这篇论文的研究方法是什么？"
- "主要的实验结果有哪些？"
- "作者得出了什么结论？"

针对手册：
- "如何完成 [某个操作]？"
- "[某个功能] 的使用步骤是什么？"
- "有哪些注意事项？"

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 📄 许可证

MIT License

## 🔗 相关链接

- [Gradio 文档](https://www.gradio.app/)
- [LangChain 文档](https://python.langchain.com/)
- [Ollama 官网](https://ollama.ai/)
- [LangChain Vector Store 文档](https://python.langchain.com/docs/concepts/vectorstores/)

---

**祝使用愉快！如有问题，请提交 Issue。** 🎉

