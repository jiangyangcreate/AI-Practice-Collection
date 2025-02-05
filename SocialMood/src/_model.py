from ollama import chat
from ollama import ChatResponse
class Model:
    @classmethod
    def get_sentiment_score(cls,text):
        prompt = "请仅对用户提供的句子进行情感评分，并返回介于-1到1之间的分数。-1表示非常负面，1表示非常正面，0表示中立。无需提供其他信息或上下文。"
        messages = [
            {"role": "system", "content": prompt},
            {"role": "user", "content": text}
        ]
        response: ChatResponse = chat(model='qwen2.5', messages=messages)
        score = float(response.message.content)
        return score
