"""请求和响应模型"""
from pydantic import BaseModel, EmailStr
from typing import Optional


class SendCodeRequest(BaseModel):
    email: EmailStr


class LoginRequest(BaseModel):
    email: EmailStr
    code: str


class ClassificationRequest(BaseModel):
    image: str


class ClassificationResponse(BaseModel):
    success: bool
    result: Optional[dict] = None
    message: Optional[str] = None

