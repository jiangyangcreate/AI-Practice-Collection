import os
import time
from pathlib import Path

import dotenv
import requests

dotenv.load_dotenv()
API_URL = "https://cloud.infini-ai.com/maas/router/bytedance/api/v3/images/generations"
MODEL_NAME = "doubao-seedream-4-0-250828"


def generate_image(prompt: str, output_dir: str = "outputs", filename_prefix: str | None = None) -> str:
    """
    根据提示词调用 GenStudio 图像生成接口，并将结果保存为时间戳命名的文件。

    :param prompt: 文本提示词
    :param output_dir: 保存图片的文件夹（默认 outputs）
    :param filename_prefix: 文件名前缀（例如单词），若为 None 则仅使用时间戳
    :return: 保存的图片绝对路径
    """
    api_key = os.environ.get("GENSTUDIO_API_KEY")
    if not api_key:
        raise RuntimeError("环境变量 GENSTUDIO_API_KEY 未设置，请在 .env 或系统环境中配置。")

    headers = {
        "Content-Type": "application/json",
        "Authorization": f"Bearer {api_key}",
    }

    payload = {
        "model": MODEL_NAME,
        "prompt": prompt,
        "size": "2048x2048",
        "watermark": False,
    }

    resp = requests.post(API_URL, json=payload, headers=headers, timeout=60)
    resp.raise_for_status()
    data = resp.json()

    # 实际返回结构示例：
    # {
    #   "model": "...",
    #   "created": 1773210610,
    #   "data": [{
    #       "url": "https://...jpeg?...",
    #       "size": "1728x2304"
    #   }],
    #   "usage": {...}
    # }
    try:
        image_url = data["data"][0]["url"]
    except (KeyError, IndexError) as e:
        raise RuntimeError(f"响应格式异常: {data}") from e

    # 下载图片
    img_resp = requests.get(image_url, timeout=60)
    img_resp.raise_for_status()
    image_bytes = img_resp.content

    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    ts = int(time.time() * 1000)
    if filename_prefix:
        safe_prefix = "".join(c if c.isalnum() or c in ("-", "_") else "_" for c in filename_prefix.strip())
        filename = f"{safe_prefix}_{ts}.jpeg"
    else:
        filename = f"{ts}.jpeg"

    out_path = out_dir / filename

    with out_path.open("wb") as f:
        f.write(image_bytes)

    return str(out_path.resolve())


if __name__ == "__main__":
    # 简单测试：使用固定的中文提示词
    # 模板提示词 + 单词
    test_prompt = "充满活力的特写编辑肖像，模特眼神犀利，头戴雕塑感帽子，色彩拼接丰富，眼部焦点锐利，景深较浅，具有Vogue杂志封面的美学风格，采用中画幅拍摄，工作室灯光效果强烈。"
    path = generate_image(test_prompt)
    print("图片已保存：", path)