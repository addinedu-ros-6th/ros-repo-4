# routers/api.py
from fastapi import APIRouter, Request
from models.image_data import ImageData
from state.shared_state import session_data
from utils.helpers import get_user_id
import os, base64

router = APIRouter()

@router.post("/upload_image")
async def upload_image(request: Request, data: ImageData):
    user_id = get_user_id(request)

    # Remove header from Base64 data
    image_data = data.image_data.split(",")[1]
    image_bytes = base64.b64decode(image_data)

    # Define file path
    file_path = os.path.join("images", "faces", f"{user_id}.png")

    # Save image
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    with open(file_path, "wb") as f:
        f.write(image_bytes)

    return {"message": "Image uploaded successfully"}
