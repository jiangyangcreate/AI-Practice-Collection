"""
文档问答系统 - 基于 Gradio 和 LangChain 的 RAG 应用
支持 PDF 文档上传和基于内容的智能问答
"""

import gradio as gr
from pathlib import Path
from typing import List, Tuple, Optional
import tempfile
import shutil

from langchain_community.document_loaders import PyPDFLoader
from langchain_ollama import OllamaEmbeddings
from langchain_core.vectorstores import InMemoryVectorStore
from langchain_core.documents import Document
from langchain_text_splitters import RecursiveCharacterTextSplitter
from ollama import chat, ChatResponse


class DocumentQASystem:
    """文档问答系统核心类"""
    
    def __init__(self, model_name: str = "qwen2.5", chunk_size: int = 1000, chunk_overlap: int = 200):
        """
        初始化文档问答系统
        
        Args:
            model_name: Ollama 模型名称
            chunk_size: 文本分块大小
            chunk_overlap: 文本分块重叠大小
        """
        self.model_name = model_name
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap
        
        # 初始化 embeddings
        self.embeddings = OllamaEmbeddings(model=model_name)
        
        # 初始化向量存储
        self.vector_store = InMemoryVectorStore(embedding=self.embeddings)
        
        # 文本分割器
        self.text_splitter = RecursiveCharacterTextSplitter(
            chunk_size=chunk_size,
            chunk_overlap=chunk_overlap,
            length_function=len,
        )
        
        # 存储已加载的文档信息
        self.loaded_documents = []
        
    def load_pdf(self, file_path: str) -> List[Document]:
        """
        加载并处理 PDF 文件
        
        Args:
            file_path: PDF 文件路径
            
        Returns:
            分割后的文档列表
        """
        try:
            loader = PyPDFLoader(file_path)
            documents = loader.load()
            
            # 分割文档
            split_docs = self.text_splitter.split_documents(documents)
            
            return split_docs
        except Exception as e:
            raise Exception(f"加载 PDF 文件失败: {str(e)}")
    
    def add_documents(self, file_path: str) -> str:
        """
        添加文档到向量数据库
        
        Args:
            file_path: 文件路径
            
        Returns:
            处理结果信息
        """
        try:
            # 加载并分割文档
            split_docs = self.load_pdf(file_path)
            
            # 添加到向量存储
            self.vector_store.add_documents(documents=split_docs)
            
            # 记录已加载的文档
            file_name = Path(file_path).name
            self.loaded_documents.append({
                'name': file_name,
                'path': file_path,
                'chunks': len(split_docs)
            })
            
            return f"✅ 成功加载文档: {file_name}\n📄 共分割为 {len(split_docs)} 个文本块"
        
        except Exception as e:
            return f"❌ 加载文档失败: {str(e)}"
    
    def retrieve_relevant_docs(self, query: str, k: int = 3) -> List[Document]:
        """
        检索相关文档
        
        Args:
            query: 查询文本
            k: 返回的文档数量
            
        Returns:
            相关文档列表
        """
        try:
            similar_docs = self.vector_store.similarity_search(query, k=k)
            return similar_docs
        except Exception as e:
            print(f"检索失败: {str(e)}")
            return []
    
    def generate_answer(self, query: str, context_docs: List[Document]) -> str:
        """
        基于检索到的文档生成答案
        
        Args:
            query: 用户问题
            context_docs: 检索到的相关文档
            
        Returns:
            生成的答案
        """
        # 构建上下文
        context = "\n\n".join([doc.page_content for doc in context_docs])
        
        # 构建提示词
        prompt = f"""你是一个专业的文档问答助手。请根据以下文档内容回答用户的问题。

文档内容:
{context}

用户问题: {query}

请基于上述文档内容给出准确、详细的回答。如果文档中没有相关信息，请明确说明。"""

        try:
            # 调用 Ollama 生成答案
            response: ChatResponse = chat(
                model=self.model_name,
                messages=[{
                    'role': 'user',
                    'content': prompt,
                }]
            )
            
            return response.message.content
        
        except Exception as e:
            return f"❌ 生成答案失败: {str(e)}"
    
    def answer_question(self, query: str, k: int = 3) -> Tuple[str, List[str]]:
        """
        完整的问答流程
        
        Args:
            query: 用户问题
            k: 检索的文档数量
            
        Returns:
            (答案, 参考文档列表)
        """
        if not query.strip():
            return "请输入您的问题。", []
        
        if len(self.loaded_documents) == 0:
            return "⚠️ 请先上传文档后再提问。", []
        
        # 检索相关文档
        relevant_docs = self.retrieve_relevant_docs(query, k=k)
        
        if not relevant_docs:
            return "❌ 未找到相关文档内容，请尝试其他问题。", []
        
        # 生成答案
        answer = self.generate_answer(query, relevant_docs)
        
        # 提取参考来源
        sources = [f"📄 文档片段 {i+1}:\n{doc.page_content[:200]}..." 
                   for i, doc in enumerate(relevant_docs)]
        
        return answer, sources
    
    def get_loaded_documents_info(self) -> str:
        """获取已加载文档的信息"""
        if not self.loaded_documents:
            return "📚 暂无已加载的文档"
        
        info = "📚 已加载的文档:\n\n"
        for i, doc in enumerate(self.loaded_documents, 1):
            info += f"{i}. {doc['name']} ({doc['chunks']} 个文本块)\n"
        
        return info
    
    def clear_documents(self):
        """清空所有文档"""
        self.vector_store = InMemoryVectorStore(embedding=self.embeddings)
        self.loaded_documents = []


# 全局实例
qa_system = DocumentQASystem(model_name="qwen2.5")


def upload_document(file) -> str:
    """处理文档上传"""
    if file is None:
        return "⚠️ 请选择要上传的文件"
    
    try:
        # 获取上传的文件路径
        file_path = file.name if hasattr(file, 'name') else str(file)
        
        # 添加文档到系统
        result = qa_system.add_documents(file_path)
        
        # 更新文档列表显示
        docs_info = qa_system.get_loaded_documents_info()
        
        return f"{result}\n\n{docs_info}"
    
    except Exception as e:
        return f"❌ 上传失败: {str(e)}"


def chat_with_documents(message: str, history: List[Tuple[str, str]]) -> str:
    """处理聊天消息"""
    if not message.strip():
        return "请输入您的问题。"
    
    # 获取答案和参考来源
    answer, sources = qa_system.answer_question(message, k=3)
    
    # 构建完整响应
    response = answer
    
    if sources:
        response += "\n\n---\n**📚 参考来源:**\n\n"
        response += "\n\n".join(sources)
    
    return response


def clear_all_documents() -> str:
    """清空所有文档"""
    qa_system.clear_documents()
    return "✅ 已清空所有文档"


def create_gradio_interface():
    """创建 Gradio 界面"""
    
    # 自定义 CSS
    custom_css = """
    .gradio-container {
        font-family: 'Arial', sans-serif;
    }
    .upload-box {
        border: 2px dashed #4CAF50 !important;
        border-radius: 10px;
    }
    """
    
    with gr.Blocks(title="📚 文档问答系统", css=custom_css, theme=gr.themes.Soft()) as demo:
        
        gr.Markdown("""
        # 📚 智能文档问答系统
        
        ### 基于 RAG (检索增强生成) 技术的本地文档问答助手
        
        **功能特点:**
        - 📄 支持 PDF 文档上传和解析
        - 🔍 智能文档检索和语义匹配
        - 💬 基于文档内容的精准问答
        - 🤖 使用 Ollama 本地大模型 (qwen2.5)
        
        ---
        """)
        
        with gr.Row():
            # 左侧：聊天界面
            with gr.Column(scale=2):
                gr.Markdown("### 💬 智能问答")
                
                chatbot = gr.Chatbot(
                    label="对话历史",
                    height=400,
                    bubble_full_width=False,
                    avatar_images=("🧑", "🤖")
                )
                
                with gr.Row():
                    msg = gr.Textbox(
                        label="输入问题",
                        placeholder="在这里输入您关于文档的问题...",
                        scale=4
                    )
                    submit_btn = gr.Button("🚀 发送", variant="primary", scale=1)
                
                gr.Examples(
                    examples=[
                        "这份文档的主要内容是什么？",
                        "请总结文档中的关键要点",
                        "文档中提到了哪些重要概念？",
                    ],
                    inputs=msg,
                    label="💡 示例问题"
                )
                
                clear_chat_btn = gr.Button("🗑️ 清空对话", size="sm")
            
            # 右侧：文档管理
            with gr.Column(scale=1):
                gr.Markdown("### 📤 文档管理")
                
                file_upload = gr.File(
                    label="上传 PDF 文档",
                    file_types=[".pdf"],
                    type="filepath"
                )
                
                upload_status = gr.Textbox(
                    label="上传状态",
                    lines=8,
                    interactive=False,
                    value="📚 暂无已加载的文档"
                )
                
                with gr.Row():
                    upload_btn = gr.Button("📥 加载文档", variant="primary")
                    clear_docs_btn = gr.Button("🗑️ 清空文档", variant="stop")
                
                gr.Markdown("""
                ---
                **使用说明:**
                1. 上传 PDF 文档
                2. 点击"加载文档"按钮
                3. 在左侧输入问题开始对话
                
                **注意:** 首次使用需要确保已安装 Ollama 并下载 qwen2.5 模型
                """)
        
        # 事件处理
        def user_message(message, history):
            """处理用户消息"""
            return "", history + [[message, None]]
        
        def bot_response(history):
            """生成机器人响应"""
            user_msg = history[-1][0]
            bot_msg = chat_with_documents(user_msg, history[:-1])
            history[-1][1] = bot_msg
            return history
        
        # 绑定事件
        msg.submit(user_message, [msg, chatbot], [msg, chatbot]).then(
            bot_response, chatbot, chatbot
        )
        submit_btn.click(user_message, [msg, chatbot], [msg, chatbot]).then(
            bot_response, chatbot, chatbot
        )
        
        upload_btn.click(upload_document, file_upload, upload_status)
        clear_docs_btn.click(clear_all_documents, None, upload_status)
        clear_chat_btn.click(lambda: None, None, chatbot)
        
        # 启动时显示文档信息
        demo.load(lambda: qa_system.get_loaded_documents_info(), None, upload_status)
    
    return demo


if __name__ == "__main__":
    print("🚀 正在启动文档问答系统...")
    print("📌 请确保已安装 Ollama 并下载了 qwen2.5 模型")
    print("   安装方法: ollama pull qwen2.5")
    print()
    import webbrowser
    webbrowser.open("http://localhost:7860")
    demo = create_gradio_interface()
    demo.launch(
        server_name="0.0.0.0",
        server_port=7860,
        share=False,
        show_error=True
    )

