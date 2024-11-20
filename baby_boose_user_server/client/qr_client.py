import requests
import json
from pyzbar.pyzbar import decode
from PIL import Image

def send_qr_data_to_server(qr_content):
    """
    서버로 QR 데이터를 전송
    """
    url = "http://localhost:8000/qr"  # 서버 주소와 엔드포인트
    headers = {"Content-Type": "application/json"}
    payload = {"qr_content": qr_content}

    # 전송 데이터 출력
    print("Sending payload to server:", payload)

    try:
        response = requests.post(url, json=payload, headers=headers)
        print("서버 응답:", response.json())
        return response.status_code == 200
    except Exception as e:
        print(f"서버로 QR 데이터를 전송하는 중 오류 발생: {e}")
        return False


def process_qr_image(image_path):
    """
    QR 코드 이미지를 읽고 데이터를 직렬화하여 서버로 전송
    """
    try:
        # QR 코드 이미지 열기
        img = Image.open(image_path)
        decoded_objects = decode(img)

        if not decoded_objects:
            print("QR 코드가 감지되지 않았습니다.")
            return

        for obj in decoded_objects:
            qr_raw_content = obj.data.decode('utf-8')  # QR 코드 데이터를 텍스트로 디코딩
            print("Decoded QR Content:", qr_raw_content)

            # QR 데이터를 텍스트에서 JSON 형식으로 변환
            qr_data = {}
            for line in qr_raw_content.strip().split("\n"):
                key, value = line.split(":", 1)
                # 필드 이름을 데이터베이스와 일치하도록 매핑
                key = key.strip().replace(" ", "_")  # 공백을 언더스코어로 변경
                qr_data[key] = value.strip()

            # JSON 문자열로 변환
            qr_content = json.dumps(qr_data)
            print("Serialized QR content:", qr_content)  # 디버깅: 직렬화된 데이터 확인

            # 서버로 전송
            send_qr_data_to_server(qr_content)
    except Exception as e:
        print(f"QR 코드를 처리하는 중 오류 발생: {e}")

        
if __name__ == "__main__":
    # QR 코드 이미지 파일 경로 지정
    image_path = "./qr_image.png"  # QR 코드 이미지 경로
    process_qr_image(image_path)
