from etool import ManagerEmail
import os
import dotenv
import sqlite3
import random
from datetime import datetime, timedelta

dotenv.load_dotenv()

# 验证码有效期（分钟）
CODE_EXPIRY_MINUTES = 10

# 数据库文件路径
DB_PATH = 'verification_codes.db'

# 初始化数据库
def _init_database():
    """初始化sqlite3数据库，创建验证码表"""
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    cursor.execute('''
        CREATE TABLE IF NOT EXISTS verification_codes (
            id INTEGER PRIMARY KEY AUTOINCREMENT,
            email TEXT NOT NULL,
            code TEXT NOT NULL,
            created_at TIMESTAMP NOT NULL
        )
    ''')
    conn.commit()
    conn.close()

# 初始化数据库
_init_database()

def _clean_expired_codes():
    """清理过期的验证码"""
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()
    expiry_time = datetime.now() - timedelta(minutes=CODE_EXPIRY_MINUTES)
    cursor.execute('''
        DELETE FROM verification_codes 
        WHERE created_at < ?
    ''', (expiry_time,))
    conn.commit()
    conn.close()

def send_verification_code(recipient_email: str) -> dict:
    """
    发送验证码到指定邮箱
    
    参数:
        recipient_email (str): 接收者邮箱地址
    
    返回:
        dict: 包含发送状态的字典
            - status: 'success' 或 'error'
            - message: 状态消息
            - code: 验证码（仅开发调试时返回，生产环境应返回None）
    """
    try:
        # 清理过期验证码
        _clean_expired_codes()
        
        # 生成6位随机数字验证码
        verification_code = ''.join([str(random.randint(0, 9)) for _ in range(6)])
        
        # 格式化HTML消息
        html_message = f"""
        <html>
        <head>
            <style>
                body {{
                    font-family: Arial, sans-serif;
                    background-color: #f4f4f4;
                    padding: 20px;
                }}
                .container {{
                    max-width: 600px;
                    margin: 0 auto;
                    background-color: #ffffff;
                    padding: 30px;
                    border-radius: 10px;
                    box-shadow: 0 2px 4px rgba(0,0,0,0.1);
                }}
                .code {{
                    font-size: 32px;
                    font-weight: bold;
                    color: #007bff;
                    text-align: center;
                    letter-spacing: 5px;
                    padding: 20px;
                    background-color: #f8f9fa;
                    border-radius: 5px;
                    margin: 20px 0;
                }}
                .footer {{
                    margin-top: 30px;
                    font-size: 12px;
                    color: #666;
                    text-align: center;
                }}
            </style>
        </head>
        <body>
            <div class="container">
                <h2>验证码</h2>
                <p>您的验证码是：</p>
                <div class="code">{verification_code}</div>
                <p>验证码有效期为{CODE_EXPIRY_MINUTES}分钟，请勿泄露给他人。</p>
                <div class="footer">
                    <p>此邮件由系统自动发送，请勿回复。</p>
                </div>
            </div>
        </body>
        </html>
        """
        
        # 发送邮件
        ManagerEmail.send_email(
            sender=os.getenv('EMAIL'),
            password=os.getenv('EMAIL_PASSWORD'),
            recipient=recipient_email,
            subject='验证码',
            message=html_message,
        )
        
        # 删除该邮箱的旧验证码（一个邮箱只保留最新的验证码）
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        cursor.execute('''
            DELETE FROM verification_codes WHERE email = ?
        ''', (recipient_email,))
        
        # 将新验证码信息存入数据库
        cursor.execute('''
            INSERT INTO verification_codes (email, code, created_at)
            VALUES (?, ?, ?)
        ''', (recipient_email, verification_code, datetime.now()))
        conn.commit()
        conn.close()
        
        return {
            'status': 'success',
            'message': f'验证码已成功发送到 {recipient_email}，有效期为{CODE_EXPIRY_MINUTES}分钟',
            'code': None  # 生产环境不返回验证码，开发时可改为 verification_code
        }
        
    except Exception as e:
        return {
            'status': 'error',
            'message': f'发送失败: {str(e)}',
            'code': None
        }

def verify_verification_code(email: str, code: str) -> dict:
    """
    验证验证码是否正确
    
    参数:
        email (str): 用户邮箱地址
        code (str): 待验证的验证码
    
    返回:
        dict: 包含验证结果的字典
            - status: 'success' 或 'error'
            - message: 验证结果消息
            - valid: True 或 False（验证码是否有效）
    """
    try:
        # 清理过期验证码
        _clean_expired_codes()
        
        conn = sqlite3.connect(DB_PATH)
        cursor = conn.cursor()
        
        # 查询该邮箱的最新验证码
        cursor.execute('''
            SELECT code, created_at 
            FROM verification_codes 
            WHERE email = ? 
            ORDER BY created_at DESC 
            LIMIT 1
        ''', (email,))
        
        result = cursor.fetchone()
        
        if not result:
            conn.close()
            return {
                'status': 'error',
                'message': '未找到验证码，请先获取验证码',
                'valid': False
            }
        
        stored_code, created_at = result
        
        # 检查验证码是否过期
        # 处理不同的 datetime 格式
        try:
            if isinstance(created_at, str):
                if '.' in created_at:
                    created_time = datetime.strptime(created_at, '%Y-%m-%d %H:%M:%S.%f')
                else:
                    created_time = datetime.strptime(created_at, '%Y-%m-%d %H:%M:%S')
            else:
                # 如果已经是 datetime 对象
                created_time = created_at
        except (ValueError, TypeError):
            # 如果解析失败，假设已过期
            created_time = datetime.now() - timedelta(minutes=CODE_EXPIRY_MINUTES + 1)
        
        expiry_time = created_time + timedelta(minutes=CODE_EXPIRY_MINUTES)
        
        if datetime.now() > expiry_time:
            # 删除过期验证码
            cursor.execute('''
                DELETE FROM verification_codes WHERE email = ?
            ''', (email,))
            conn.commit()
            conn.close()
            return {
                'status': 'error',
                'message': '验证码已过期，请重新获取',
                'valid': False
            }
        
        # 验证验证码是否匹配
        if stored_code == code:
            # 验证成功后删除验证码（防止重复使用）
            cursor.execute('''
                DELETE FROM verification_codes WHERE email = ?
            ''', (email,))
            conn.commit()
            conn.close()
            return {
                'status': 'success',
                'message': '验证码验证成功',
                'valid': True
            }
        else:
            conn.close()
            return {
                'status': 'error',
                'message': '验证码错误',
                'valid': False
            }
            
    except Exception as e:
        return {
            'status': 'error',
            'message': f'验证失败: {str(e)}',
            'valid': False
        }

if __name__ == '__main__':
    # 测试代码
    test_email = os.getenv('EMAIL', 'test@example.com')
    print("测试发送验证码:")
    result = send_verification_code(test_email)
    print(result)
    
    if result['status'] == 'success':
        # 注意：这里需要手动输入验证码进行测试
        # 实际使用时，验证码应该从邮件中获取
        test_code = input("\n请输入收到的验证码进行测试: ")
        print("\n测试验证验证码:")
        verify_result = verify_verification_code(test_email, test_code)
        print(verify_result)