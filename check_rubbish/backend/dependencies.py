"""依赖项模块"""
from fastapi import HTTPException, status, Header
from typing import Optional

# 简单的用户会话存储（生产环境应使用Redis或数据库）
user_sessions = {}


async def verify_login(email: Optional[str] = Header(None, alias="X-User-Email")):
    """验证用户是否已登录"""
    if not email or email not in user_sessions:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="未登录或登录已过期，请先登录"
        )
    return email

