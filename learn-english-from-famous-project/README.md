## 从知名 Python 项目学英语

使用 FastAPI 后端 + 纯 HTML + Tailwind 前端，从 Python 项目的标识符中抽取英文词表，便于学习技术英语。

### 运行环境

- **Python 3.11**（推荐）

### 安装依赖

```bash
uv add -r requirements.txt
```

### 启动服务

在项目根目录执行：

```bash
uv run uvicorn app.main:app --reload
```

访问 `http://127.0.0.1:8000/`。

### 使用方式

1. 在首页的搜索框中输入：
   - 完整 GitHub 仓库 URL，例如：`https://github.com/psf/requests`
   - 或本机项目的绝对路径，例如：`C:\code\my_project`
2. 提交后，后端会：
   - 对 URL：使用 `git clone --depth 1` 克隆到 `data/repos/<项目名>` 下，例如 `data/repos/requests`。如果该目录已经存在，则**不重复拉取**，直接复用。
   - 对本地绝对路径：直接使用该目录，项目标签为目录名（例如 `myproject`）。
3. 后台通过 FastAPI 的 `BackgroundTasks` 启动分析任务，不阻塞请求。

### 分析流程

1. 使用 `pathlib.Path.rglob("*.py")` 遍历项目中所有 `.py` 文件，跳过 `.venv` 目录。
2. 使用 `ast` 解析 Python 源码，收集：
   - 模块名 / 类名 / 函数名 / 方法名
   - 变量名、参数名
   - 导入别名、属性访问（例如 `os.path.join`）
3. 将所有标识符统计为一个 `Counter`，写入 JSONL：
   - 文件：`data/results/<项目标签>_ast.jsonl`
   - 每行一个对象：`{"name": "<标识符>", "count": <出现次数>}`
4. 将标识符拆解为单词：
   - `os.path.join` → `os path join`
   - `__init__` → `init`
   - `_test_task` → `test task`
   - `ZoneInfo` → `Zone Info`
   - 同一单词如果出现多种大小写形式（如 `zoneinfo` / `ZoneInfo`），统一合并计数，并优先保留**包含大写字母**的形式作为键。
5. 写出单词统计 JSONL：
   - 文件：`data/results/<项目标签>_words.jsonl`
   - 每行一个对象：`{"word": "<单词>", "count": <出现次数>}`
6. 调用外部翻译服务（你需要在自己的服务器上实现）：
   - 函数签名：`def translate_lines(text: str) -> str`
   - 项目中通过 `app/services/translation.py` 内的 `translate_lines` 调用 HTTP 接口。
   - 约定请求：
     - `POST $TRANSLATE_API_URL`
     - `Content-Type: text/plain; charset=utf-8`
     - 请求体为以换行分割的英文单词字符串
   - 响应：
     - 响应体为以换行分割的翻译结果字符串
   - 代码会将单词与每一行翻译结果对齐，生成：
     - 文件：`data/results/<项目标签>_translate.jsonl`
     - 每行一个对象：`{"word": "<单词>", "translation": "<翻译说明>" }`

### 环境变量

- `TRANSLATE_API_URL`：外部翻译服务的 HTTP 地址。

### 前端页面

- 使用纯 HTML + Tailwind CDN。
- 首页包括：
  - 搜索框：输入完整 GitHub 仓库 URL 或本地绝对路径。
  - GitHub Trending Python 项目前 5 名：
    - 名称 + 本月 Star 数
    - 名称为超链接，点击跳转到对应 GitHub 仓库
  - 已完成的项目列表：
    - 来自 `data/results` 下的 `*_translate.jsonl`
    - 展示项目标签（例如 `requests`、`myproject`）
    - 点击“下载词表”即可下载对应 JSONL 文件

### JSON Lines 说明

本项目所有过程文件均为 **JSONL（JSON Lines）** 格式：

- 每一行是一个独立的 JSON 对象。
- 你可以用任意 JSONL 解析工具或脚本进行后续处理。

