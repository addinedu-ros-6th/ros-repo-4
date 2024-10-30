## 사용 환경 설정

설치 라이브러리

* pip install fastapi
* pip install "uvicorn[standard]"

### 프로그램 실행

main.py 파일이 있는 폴더 위치 터미널에서
```
uvicorn main:app --reload --host <현재 컴퓨터 내부 IP>
```

### client 접속

브라우저에서 <IP>:8000 로 접속