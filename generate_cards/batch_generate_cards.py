from pathlib import Path

from genstudio_client import generate_image


CARD_OUTPUT_DIR = "cards"

PROMPT_TEMPLATE = (
    "以漫画风格、蓝绿色主色调，为英语单词\"{word}\"设计一张1：1比例的插画，"
    "画面为美式漫威漫画风格，突出与含义相关的场景插画，"
    "适合青少年英语学习，纯粹的画面，去掉所有文本文字。"
)


def load_words(words_file: str = "words.txt") -> list[str]:
    path = Path(words_file)
    if not path.exists():
        raise FileNotFoundError(f"未找到单词文件: {path.resolve()}")

    words: list[str] = []
    for line in path.read_text(encoding="utf-8").splitlines():
        w = line.strip()
        if not w:
            continue
        words.append(w)
    return words


def main() -> None:
    words = load_words("words.txt")
    if not words:
        print("words.txt 为空，没有要生成的单词。")
        return

    print(f"共 {len(words)} 个单词，开始生成漫画蓝绿风格单词卡片……")

    output_paths: list[str] = []
    for idx, word in enumerate(words, start=1):
        prompt = PROMPT_TEMPLATE.format(word=word)
        try:
            img_path = generate_image(
                prompt=prompt,
                output_dir=CARD_OUTPUT_DIR,
                filename_prefix=word,
            )
            output_paths.append(img_path)
            print(f"[{idx}/{len(words)}] {word} -> {img_path}")
        except Exception as e:  # noqa: BLE001
            print(f"[{idx}/{len(words)}] 生成 {word} 失败: {e}")

    print("生成完成。总计成功:", len(output_paths))
    if output_paths:
        print("示例文件路径:", output_paths[0])


if __name__ == "__main__":
    main()

