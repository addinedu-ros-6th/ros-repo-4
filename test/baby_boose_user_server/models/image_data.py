# models/image_data.py
from pydantic import BaseModel

class ImageData(BaseModel):
    image_data: str  # Base64 encoded image data
