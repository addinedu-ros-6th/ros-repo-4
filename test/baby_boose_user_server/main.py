from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from fastapi.websockets import WebSocketState
from fastapi.staticfiles import StaticFiles
from typing import List
import cv2
import asyncio

app = FastAPI()

app.mount("/css", StaticFiles(directory="css"), name="css")
app.mount("/js", StaticFiles(directory="js"), name="js")
app.mount("/images", StaticFiles(directory="images"), name="images")

# 템플릿 폴더 경로 설정
templates = Jinja2Templates(directory="templates")

# WebSocket 연결을 관리하기 위한 리스트 (타입 힌트)
# connected_clients: List[WebSocket] = []

# 시작 화면 start.html
@app.get("/", response_class=HTMLResponse)
async def read_root(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("start.html", context)

@app.get("/home", response_class=HTMLResponse)
async def read_root(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("home.html", context)

@app.get("/face_recognition", response_class=HTMLResponse)
async def read_root(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("face_recognition.html", context)

@app.get("/ticket_scan", response_class=HTMLResponse)
async def read_root(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("ticket_scan.html", context)

@app.get("/confirmation", response_class=HTMLResponse)
async def read_mode_page(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("confirmation.html", context)

@app.get("/cargo_open", response_class=HTMLResponse)
async def read_face_page(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("cargo_open.html", context)

@app.get("/select_mode", response_class=HTMLResponse)
async def read_mode_page(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("select_mode.html", context)

@app.get("/auto_driving", response_class=HTMLResponse)
async def read_root(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("auto_driving.html", context)

@app.get("/guiding", response_class=HTMLResponse)
async def read_root(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("guiding.html", context)

@app.get("/following", response_class=HTMLResponse)
async def read_root(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("following.html", context)

@app.get("/stop", response_class=HTMLResponse)
async def read_root(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("stop.html", context)

@app.get("/show_location", response_class=HTMLResponse)
async def read_root(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("show_location.html", context)

@app.get("/take_a_break", response_class=HTMLResponse)
async def read_root(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("take_a_break.html", context)

@app.get("/arrived_pick_up_counter", response_class=HTMLResponse)
async def read_root(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("arrived_pick_up_counter.html", context)

@app.get("/cargo_open_pick_up", response_class=HTMLResponse)
async def read_root(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("cargo_open_pick_up.html", context)

@app.get("/at_the_gate", response_class=HTMLResponse)
async def read_root(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("at_the_gate.html", context)

@app.get("/face_identification", response_class=HTMLResponse)
async def read_root(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("face_identification.html", context)

@app.get("/cargo_open_final", response_class=HTMLResponse)
async def read_root(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("cargo_open_final.html", context)

@app.get("/thanks", response_class=HTMLResponse)
async def read_root(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("thanks.html", context)

@app.get("/searching_person", response_class=HTMLResponse)
async def read_root(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("searching_person.html", context)

@app.get("/returning", response_class=HTMLResponse)
async def read_root(request: Request):
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("returning.html", context)


# WebSocket endpoint
# @app.websocket("/ws/video")
# async def video_stream(websocket: WebSocket):
#     await websocket.accept()
#     connected_clients.append(websocket)
#     try:
#         # 카메라를 열고 WebSocket으로 프레임을 스트리밍
#         cap = cv2.VideoCapture(0)
#         while True:
#             if websocket.client_state != WebSocketState.CONNECTED:
#                 break
#             ret, frame = cap.read()
#             if not ret:
#                 break
#             # 프레임을 JPEG 형식으로 인코딩하여 전송
#             _, buffer = cv2.imencode('.jpg', frame)
#             await websocket.send_bytes(buffer.tobytes())
#             await asyncio.sleep(0.03)  # 약간의 딜레이를 추가해 CPU 사용을 줄임
#     except WebSocketDisconnect:
#         connected_clients.remove(websocket)
#     finally:
#         cap.release()  # 카메라 자원 해제