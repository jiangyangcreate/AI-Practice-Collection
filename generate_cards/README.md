# GenStudio Client

无问芯穹 GenStudio API 的 Python 客户端（函数式风格），使用 [uv](https://docs.astral.sh/uv/) 管理环境。

## 文档与模型

- [GenStudio API 教程](https://docs.infini-ai.com/gen-studio/api/tutorial.html)
- 默认模型对应页面: https://cloud.infini-ai.com/genstudio/model/mo-dcfn7bfjnbv5n2wz
- API 端点: `https://cloud.infini-ai.com/maas/v1`
- [API 密钥申请](https://cloud.infini-ai.com/iam/secret/key)

## 环境准备（uv）

```bash
# 创建虚拟环境并安装依赖
uv sync

# 或指定 Python 版本
uv sync --python 3.12
```

## 配置

设置环境变量（二选一或同时设置）：

- `GENSTUDIO_API_KEY` 或 `OPENAI_API_KEY`：API 密钥（必填）
- `GENSTUDIO_BASE_URL`：默认 `https://cloud.infini-ai.com/maas/v1`
- `GENSTUDIO_MODEL`：默认 `mo-dcfn7bfjnbv5n2wz`

## 使用

```bash
# 设置密钥后直接运行示例
set GENSTUDIO_API_KEY=your_key
uv run python genstudio_client.py
```

在代码中：

```python
from genstudio_client import ask, make_client, user_message, system_message, chat, print_stream

# 单轮问答（同步）
reply = ask("你好")
print(reply)

# 流式输出
stream = ask("写一首短诗", stream=True)
print_stream(stream)

# 自定义多轮消息
from genstudio_client import make_client, chat, system_message, user_message
client = make_client()
messages = [
    system_message("你是一个简洁的助手。"),
    user_message("1+1=?")
]
print(chat(client, messages))
```

## 开发

```bash
uv sync --all-extras   # 安装 dev 依赖
uv run ruff check .    # 检查代码
```
