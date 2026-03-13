from __future__ import annotations

import os
from typing import Iterable, List
import dotenv

dotenv.load_dotenv()
import dashscope

def translate_lines(text: str) -> str:
    """
    通过 HTTP 调用外部翻译服务。
    你可以根据自己的服务协议，自行修改实现。
    这里假设：
      - POST TRANSLATE_API_URL
      - 请求体是 text/plain，内容为多行文本
      - 响应体是 text/plain，对应多行翻译结果
    """
    model_name = "qwen3-coder-next"
    template = f"""你需要将输入的内容翻译成中文，将所有可能缩写补充完整。

示例输入：
dir
i
windows
seconds
zhipu
ms

示例输出： 
directory 的缩写，表示目录；  
通常为 item 或 index 的缩写，表示 元素、索引；
多个窗口 window 的复数，也可能表示 Windows 操作系统; 
秒 second 的复数形式；
zhipu 的专有名词，指智谱（中国一家人工智能公司，Zhipu AI）；
表示 millisecond 毫秒 也可能表示 microsoft 微软公司;

说明：不要将多个单词合并翻译，每个单词都要翻译。翻译结果用;与换行符号\n分割。

你需要翻译的单词：
{text}
    """

    messages = [
        {'role': 'system', 'content': '你需要翻译的单词来自 python 代码的函数名 变量名 请你推测其含义'},
        {'role': 'user', 'content': template}
    ]
    response = dashscope.Generation.call(
        # 若没有配置环境变量，请用百炼API Key将下行替换为：api_key="sk-xxx"
        api_key=os.getenv('DASHSCOPE_API_KEY'),
        model=model_name,
        messages=messages,
        result_format='message',
        temperature=0.1
        )
    """
    {
    "status_code": 200,
    "request_id": "902fee3b-f7f0-9a8c-96a1-6b4ea25af114",
    "code": "",
    "message": "",
    "output": {
        "text": null,
        "finish_reason": null,
        "choices": [
        {
            "finish_reason": "stop",
            "message": {
            "role": "assistant",
            "content": "我是阿里云开发的一款超大规模语言模型，我叫千问。"
            }
        }
        ]
    },
    "usage": {
        "input_tokens": 22,
        "output_tokens": 17,
        "total_tokens": 39
    }
    }
    """

    result = response.output.choices[0].message.content
    with open("response.txt", "w", encoding="utf-8") as f:
        f.write(result)
    with open("input.txt", "w", encoding="utf-8") as f:
        f.write(text)
    return result


def translate_word_list(words: Iterable[str]) -> List[tuple[str, str]]:
    """
    将单词列表按批次调用 translate_lines。
    默认每批 20 个单词，可通过环境变量 TRANSLATE_BATCH_SIZE 调整。
    返回 [(word, translation), ...]
    """
    word_list = list(words)
    if not word_list:
        return []

    # 从环境变量读取批大小，默认 20
    try:
        batch_size = int(os.getenv("TRANSLATE_BATCH_SIZE", "20"))
    except ValueError:
        batch_size = 20
    batch_size = max(1, batch_size)

    results: List[tuple[str, str]] = []

    for i in range(0, len(word_list), batch_size):
        batch = word_list[i : i + batch_size]

        # 最多重试 3 次，直到翻译行数与原词数一致
        attempts = 0
        lines: List[str] = []
        request_text = ""
        raw_result = ""
        while attempts < 3:
            request_text = "\n".join(batch)
            raw_result = translate_lines(request_text)
            lines = raw_result.splitlines()
            if len(lines) == len(batch):
                break
            attempts += 1

        if len(lines) != len(batch):
            # 3 次之后仍不一致，记录最后一次输入和输出到日志文件
            from pathlib import Path
            from datetime import datetime

            log_path = Path(__file__).resolve().parent.parent.parent / "translation_mismatch.log"
            with log_path.open("a", encoding="utf-8") as f:
                f.write(f"[{datetime.now().isoformat()}] batch_size={len(batch)}, lines={len(lines)}\n")
                f.write("INPUT:\n")
                f.write(request_text + "\n")
                f.write("OUTPUT:\n")
                f.write(raw_result + "\n")
                f.write("=" * 40 + "\n")

        # 为了后续流程健壮性，这里仍然做长度对齐
        if len(lines) < len(batch):
            lines.extend([""] * (len(batch) - len(lines)))
        elif len(lines) > len(batch):
            lines = lines[: len(batch)]

        results.extend(zip(batch, lines))

    return results

