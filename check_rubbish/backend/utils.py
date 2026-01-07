"""工具函数模块"""
import base64


def validate_image(image_base64: str) -> bool:
    """验证base64图像是否有效"""
    if not image_base64:
        return False
    
    try:
        image_data = base64.b64decode(image_base64)
        if len(image_data) == 0 or len(image_data) > 100 * 1024 * 1024:
            return False
        
        signatures = [
            b'\xff\xd8\xff',  # JPEG
            b'\x89PNG\r\n\x1a\n',  # PNG
            b'GIF87a', b'GIF89a',  # GIF
            b'RIFF'  # WEBP
        ]
        
        return any(image_data.startswith(sig) for sig in signatures)
    except Exception:
        return False

