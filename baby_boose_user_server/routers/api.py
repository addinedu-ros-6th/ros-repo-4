# routers/api.py
from fastapi import APIRouter, Request
from models.image_data import ImageData
from state.shared_state import session_data
from utils.helpers import get_user_id
import os, base64
import numpy as np
import cv2
import boto3

REKOGNITION_CLIENT = boto3.client('rekognition', region_name='ap-northeast-2')

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


@router.post("/recognition_image")
async def upload_image(request: Request, data: ImageData):
    user_id = get_user_id(request)
    origin_image_path = os.path.join("images", "faces", f"{user_id}.png")
    
    rekognition_client = REKOGNITION_CLIENT

    # Remove header from Base64 data
    image_data = data.image_data.split(",")[1]
    image_bytes = base64.b64decode(image_data)

    # opencv 작업을 위한 image 형식 변환
    np_arr = np.frombuffer(image_bytes, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    if img is None:
        raise ValueError("이미지 디코딩에 실패했습니다.")

    try:
        with open(origin_image_path, 'rb') as f:
            source_image_bytes = f.read()
    except FileNotFoundError as e:
        print(f"원본 이미지가 존재하지 않습니다. Error : {e}")

    # AWS Rekognition 으로 얼굴 비교
    response = rekognition_client.compare_faces(
        SourceImage={'Bytes': source_image_bytes},
        TargetImage={'Bytes': image_bytes},
        SimilarityThreshold=95  # 유사도 기준값 설정
    )
    
    # Rekognition 결과 처리
    if response['FaceMatches']:
        for match in response['FaceMatches']:
            similarity_by_aws = match['Similarity']
            # 확인 이미지에 유사도 표시
            cv2.putText(img, f'Similarity: {similarity_by_aws:.2f}%', (18, 18),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (36, 255, 12), 2)

    # Define file path
    file_path = os.path.join("images", "faces", f"{user_id}_reco.png")

    # Save image
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    success, encoded_image = cv2.imencode('.png', img)
    if success:
        with open(file_path, "wb") as f:
            f.write(encoded_image.tobytes()) 
        print(f"요청 이미지 저장 성공 {file_path}")
    else:
        print("요청 이미지 저장을 실패하였습니다. ")

    return {"message": "Image uploaded successfully"}
