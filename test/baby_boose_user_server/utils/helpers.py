from fastapi import Request
from uuid import uuid4
from state.shared_state import session_data


def get_user_id(request: Request):
    """Assign an ID to the user"""
    user_id = request.cookies.get("user_id")
    if not user_id:  # If there's no ID, create a new one
        user_id = str(uuid4())
    if user_id not in session_data:
        session_data[user_id] = {}  # Create space to store user info
    return user_id

def get_user_id_from_scope(scope):
    headers = dict(scope['headers'])
    cookie = headers.get(b'cookie')
    if cookie:
        # 바이트 문자열을 디코드하여 문자열로 변환
        cookie_str = cookie.decode()
        # 쿠키 문자열을 파싱하여 딕셔너리로 변환
        cookies = {}
        for pair in cookie_str.split(';'):
            if '=' in pair:
                key, value = pair.strip().split('=', 1)
                cookies[key] = value
        user_id = cookies.get('user_id')
        if user_id and user_id in session_data:
            return user_id
    # 유효한 user_id가 없을 경우 새로 생성
    user_id = str(uuid4())
    session_data[user_id] = {}
    return user_id