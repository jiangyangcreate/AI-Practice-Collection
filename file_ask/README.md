# 🤖 AI 智能问答助手

一个基于 Gradio 和 LangChain 的现代化 AI 问答系统，支持本地文档问答 (RAG)、多轮对话。

## 📚 项目列表

本仓库包含两个主要应用：

### 1. 📄 文档问答系统 (document_qa.py) - ⭐ 推荐

基于 RAG (检索增强生成) 技术的本地文档问答助手：

- **📄 PDF 文档上传**: 支持上传和解析 PDF 文档
- **🔍 智能检索**: 基于语义相似度检索相关内容
- **💬 精准问答**: 基于文档内容的准确回答
- **🤖 本地运行**: 使用 Ollama 本地大模型 (qwen2.5)
- **📊 参考来源**: 显示答案的文档出处
- **🎨 现代界面**: 友好的 Gradio 聊天界面

**快速开始**: 查看 [QUICKSTART_DOCUMENT_QA.md](QUICKSTART_DOCUMENT_QA.md)  
**详细文档**: 查看 [README_DOCUMENT_QA.md](README_DOCUMENT_QA.md)

### 2. 🗨️ 多功能问答助手 (原项目)

支持多轮对话、多种文件类型上传：

- **🗨️ 多轮对话**: 支持连续多轮对话，保持上下文
- **📎 多文件上传**: 支持上传多种类型文件
- **💬 友好界面**: 现代化的聊天界面
- **🔄 上下文管理**: 自动维护文件上传历史

## ✨ 核心特性

### 文档问答系统特性
- **🔧 RAG 技术**: 检索增强生成，确保答案准确性
- **📑 文档处理**: 自动分割和向量化文档内容
- **🧠 语义检索**: 基于内容语义而非关键词匹配
- **💾 内存存储**: 快速的向量检索响应
- **🎯 上下文感知**: 保持对话上下文，支持追问

## 🚀 快速开始

### 文档问答系统 (推荐)

#### 1. 安装 Ollama

访问 [Ollama 官网](https://ollama.ai/) 下载并安装，然后拉取模型：

```bash
ollama pull qwen2.5
```

#### 2. 安装依赖

```bash
# 使用 uv (推荐)
uv sync

# 或使用 pip
pip install gradio langchain langchain-community langchain-ollama langchain-core ollama pypdf
```

#### 3. 运行应用

```bash
python document_qa.py
```

应用将在 `http://localhost:7860` 启动，自动在浏览器中打开。

**详细教程**: 查看 [QUICKSTART_DOCUMENT_QA.md](QUICKSTART_DOCUMENT_QA.md)

### 配置阿里云知识库（可选）

如果要使用阿里云知识库功能，需要配置环境变量：

```bash
# 方式1: 创建 .env 文件
export ALIBABA_CLOUD_ACCESS_KEY_ID="your-access-key-id"
export ALIBABA_CLOUD_ACCESS_KEY_SECRET="your-access-key-secret"
export ALIYUN_INDEX_ID="your-knowledge-base-index-id"

# 方式2: Windows PowerShell
$env:ALIBABA_CLOUD_ACCESS_KEY_ID="your-access-key-id"
$env:ALIBABA_CLOUD_ACCESS_KEY_SECRET="your-access-key-secret"
$env:ALIYUN_INDEX_ID="your-knowledge-base-index-id"
```

详细配置说明请参考 [CONFIG_GUIDE.md](CONFIG_GUIDE.md)

## 📖 使用说明

### 基础界面 (f.py)

1. **发送消息**: 在文本框中输入问题，点击"发送"按钮或按 Enter 键
2. **上传文件**: 在右侧文件上传区域选择并上传文件
3. **查看历史**: 所有对话和上传的文件都会保留在上下文中
4. **清空对话**: 点击"清空对话"按钮清除聊天历史
5. **清空文件**: 点击"清空文件上下文"按钮清除已上传的文件记录

### 阿里云知识库界面 (gradio_knowledge_base_demo.py)

运行知识库演示：

```bash
python gradio_knowledge_base_demo.py
```

功能说明：

1. **智能问答**: 输入问题，自动从知识库检索并回答
2. **文档上传**: 上传文档到阿里云知识库
3. **知识检索**: 直接搜索知识库内容
4. **使用说明**: 查看详细的使用指南

## 🔧 技术栈

- **Gradio 5.x**: 现代化的机器学习界面框架
- **Python 3.12+**: 最新的 Python 版本
- **LangChain**: (可选) 用于集成 AI 模型
- **阿里云百炼 SDK**: 知识库管理和检索
- **Ollama**: (可选) 本地 AI 模型推理

## 📚 阿里云知识库 API 使用

项目提供了两个核心函数用于操作阿里云知识库：

### 函数1: 上传文件到知识库

```python
from aliyun_knowledge_base import upload_file_to_knowledge_base

# 上传文件
result = upload_file_to_knowledge_base(
    index_id="your-index-id",
    file_path="./document.pdf"
)

if result['success']:
    print(f"✅ 上传成功，任务ID: {result['job_id']}")
else:
    print(f"❌ 上传失败: {result['message']}")
```

**返回值说明**:
- `success` (bool): 上传是否成功
- `message` (str): 返回消息
- `job_id` (str): 任务ID（成功时）
- `request_id` (str): 请求ID

### 函数2: 检索知识库内容

```python
from aliyun_knowledge_base import retrieve_from_knowledge_base

# 检索内容
result = retrieve_from_knowledge_base(
    index_id="your-index-id",
    query="如何使用阿里云知识库？",
    top_k=5
)

if result['success']:
    for doc in result['documents']:
        print(f"标题: {doc['title']}")
        print(f"内容: {doc['content']}")
        print(f"得分: {doc['score']}")
else:
    print(f"❌ 检索失败: {result['message']}")
```

**返回值说明**:
- `success` (bool): 检索是否成功
- `message` (str): 返回消息
- `documents` (List[Dict]): 检索到的文档列表
  - `title` (str): 文档标题
  - `content` (str): 文档内容
  - `score` (float): 相关度得分
  - `document_id` (str): 文档ID
- `request_id` (str): 请求ID
- `total_count` (int): 结果总数

### 更多示例

查看完整的 API 使用示例：

```bash
# 运行示例代码
python example_usage.py

# 运行测试
python test_knowledge_base.py
```

详细文档：
- [阿里云知识库 API 使用指南](ALIYUN_KNOWLEDGE_BASE.md)
- [配置指南](CONFIG_GUIDE.md)

## 📁 项目结构

```
ollama_ask_demo/
├── document_qa.py                    # 📄 文档问答系统 (主应用)
├── llm.py                            # 基础 LLM 和向量存储示例
├── local_document_qa.py              # 本地文档问答示例
├── pyproject.toml                    # 项目依赖配置
├── uv.lock                           # 依赖锁定文件
├── README.md                         # 项目说明（本文件）
├── README_DOCUMENT_QA.md             # 文档问答系统详细文档
├── QUICKSTART_DOCUMENT_QA.md         # 快速开始指南
├── example_data/                     # 示例数据目录
│   └── README.md                     # 示例数据说明
└── .env                              # 环境变量配置（可选）
```

## 🎨 界面预览

界面分为两个主要区域：

- **左侧 (2/3 宽度)**: 聊天对话区域
  - 显示历史对话
  - 输入框和发送按钮
  - 清空对话按钮

- **右侧 (1/3 宽度)**: 文件上传区域
  - 5个不同类型的文件上传器
  - 清空文件上下文按钮
  - 状态显示区域

## 🔌 集成 AI 模型

当前代码提供了一个演示响应。你可以在 `chat_with_files` 函数中集成你的 AI 模型：

```python
def chat_with_files(message, history, *files):
    # 在这里集成你的 AI 模型
    # 例如: Ollama, OpenAI, LangChain Agent 等
    
    # 示例: 使用 Ollama
    # response = ollama.chat(model='llama2', messages=[...])
    
    # 示例: 使用 LangChain
    # response = agent.invoke({"messages": [...]})
    
    return response
```

## 📝 自定义配置

在 `demo.launch()` 中可以修改配置：

```python
demo.launch(
    share=False,           # 设为 True 可生成公共链接
    server_name="0.0.0.0", # 监听地址
    server_port=7860,      # 端口号
    show_error=True        # 显示错误信息
)
```

## 🛠️ 依赖项

### 核心依赖
- `gradio>=5.0.0` - Web 界面框架
- `langchain>=1.0.5` - (可选) AI 模型集成
- `ollama>=0.6.0` - (可选) 本地 AI 模型

### 阿里云知识库相关
- `alibabacloud-bailian20231229>=2.1.0` - 阿里云百炼 SDK
- `alibabacloud-tea-openapi>=0.3.9` - 阿里云 OpenAPI 基础库
- `alibabacloud-tea-util>=0.3.12` - 阿里云工具库

所有依赖已在 `pyproject.toml` 中配置，运行 `uv sync` 即可自动安装。

## 📄 许可证

MIT License

## 🤝 贡献

欢迎提交 Issue 和 Pull Request！

## 🔗 相关链接

### 官方文档
- [Gradio 官方文档](https://www.gradio.app/docs/gradio/interface)
- [阿里云百炼控制台](https://bailian.console.aliyun.com/)
- [阿里云百炼 API 文档](https://help.aliyun.com/zh/model-studio/)
- [提交知识库追加任务 API](https://help.aliyun.com/zh/model-studio/api-bailian-2023-12-29-submitindexadddocumentsjob)
- [检索知识库 API](https://help.aliyun.com/zh/model-studio/api-bailian-2023-12-29-retrieve)

### 项目文档
- [阿里云知识库 API 使用指南](ALIYUN_KNOWLEDGE_BASE.md) - 详细的 API 文档和示例
- [配置指南](CONFIG_GUIDE.md) - 环境配置和故障排除
- [demo.txt](demo.txt) - API 文档链接参考

---

基于 [Gradio 最新文档](https://www.gradio.app/docs/gradio/interface) 和 [阿里云百炼 API](https://help.aliyun.com/zh/model-studio/) 构建

