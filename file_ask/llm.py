from ollama import chat
from ollama import ChatResponse
from langchain_ollama import OllamaEmbeddings
from langchain_core.vectorstores import InMemoryVectorStore

## 加载文档

from langchain_community.document_loaders import PyPDFLoader

file_path = "2. C语言入门.pdf"
loader = PyPDFLoader(file_path)


embeddings = OllamaEmbeddings(
    model="qwen2.5",
)

## 矢量存储

vector_store = InMemoryVectorStore(embedding=embeddings())

vector_store.add_documents(documents=[loader], ids=["id1", "id2"])
def get_similar_docs(p:str):
    similar_docs = vector_store.similarity_search(
    p,
    k=3,

    )
    return similar_docs

def chat_llm(prompt:str):
    """
    包含提示词本身和 来自 知识库的检索内容作为大模型的提示词输入
    """
    prompt += get_similar_docs(prompt)
    response: ChatResponse = chat(model='qwen2.5', messages=[
    {
        'role': 'user',
        'content': prompt,
    },
    ])
    print(response.message.content)
    return response.message.content

if __name__ == "__main__":
    chat_llm("C语言是什么")