from fastapi import FastAPI, HTTPException, Depends, status
from fastapi.middleware.cors import CORSMiddleware
from send_email import send_verification_code, verify_verification_code
from models import SendCodeRequest, LoginRequest, ClassificationRequest, ClassificationResponse
from dependencies import verify_login, user_sessions
from utils import validate_image

app = FastAPI(title="垃圾识别API", version="1.0.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

@app.post("/api/auth/send-code")
async def send_code(request: SendCodeRequest):
    """发送验证码到用户邮箱"""
    result = send_verification_code(request.email)
    if result['status'] == 'error':
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=result['message'])
    return {"success": True, "message": result['message']}

@app.post("/api/auth/login")
async def login(request: LoginRequest):
    """用户登录接口（使用邮箱和验证码）"""
    verify_result = verify_verification_code(request.email, request.code)
    if not verify_result['valid']:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail=verify_result['message'])
    user_sessions[request.email] = {"email": request.email, "logged_in": True}
    return {"success": True, "message": "登录成功", "email": request.email}

@app.post("/api/classify", response_model=ClassificationResponse)
async def classify_waste(request: ClassificationRequest, email: str = Depends(verify_login)):
    """垃圾识别接口"""
    if not request.image or not validate_image(request.image):
        raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="无效的图像数据")
    from classifier import classify_image  # type: ignore
    result = await classify_image(request.image)
    if not result.get("success", False):
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail=result.get("result", "识别失败"))
    return ClassificationResponse(
        success=True,
        result={"text": result.get("result", ""), "image_path": result.get("image_path", "")},
        message="识别成功"
    )

# uvicorn main:app --reload --host 0.0.0.0 --port 8000