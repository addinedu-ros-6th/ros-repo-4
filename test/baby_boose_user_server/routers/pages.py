# routers/pages.py
from fastapi import APIRouter, Request
from fastapi.responses import HTMLResponse
from fastapi.templating import Jinja2Templates
from state.shared_state import session_data, send_msg_q, TEMP_ROBOT_ID
from utils.helpers import get_user_id
import os

router = APIRouter()
templates = Jinja2Templates(directory="templates")

# 시작 화면 start.html
# uvicorn main:app --host 192.168.0.45 --port 8000 --reload
# http://127.0.0.1:8000/?robot_id=5 으로 접속
@router.get("/", response_class=HTMLResponse)
async def read_start(request: Request, robot_id: int):
    user_id = get_user_id(request)

    if robot_id:
        session_data[user_id]["robot_id"] = robot_id
        print(session_data)

    send_msg_q.put({"robot_id": robot_id, "state" : "start"})

    context = {"request": request}

    response = templates.TemplateResponse("start.html", context)
    response.set_cookie(key="user_id", value=user_id)  # 쿠키에 ID 저장
    return response

@router.get("/home", response_class=HTMLResponse)
async def read_home(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "home"})
    
    context = {"request": request}    
    return templates.TemplateResponse("home.html", context)

@router.get("/face_identification", response_class=HTMLResponse)
async def read_face_recognition(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "face_identification"})
   
    context = {"request": request, "name": "Goose AI Porter"}
    return templates.TemplateResponse("face_identification.html", context)

@router.get("/ticket_scan", response_class=HTMLResponse)
async def read_ticket_scan(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "ticket_scan"})

    context = {"request": request}
    return templates.TemplateResponse("ticket_scan.html", context)

@router.get("/confirmation", response_class=HTMLResponse)
async def read_confirmation(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "confirmation"})

    ticket = session_data.get("ticket", {})

    context = {
        "request": request,
        "image_name": user_id,
        "user_name": ticket.get("name", "N/A"),
        "flight_no": ticket.get("flight number", "N/A"),
        "gate": ticket.get("gate", "N/A"),
        "boarding_time": ticket.get("boarding time", "N/A")
    }

    return templates.TemplateResponse("confirmation.html", context)

@router.get("/cargo_open", response_class=HTMLResponse)
async def read_cargo_open(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "cargo"})

    context = {"request": request}
    return templates.TemplateResponse("cargo.html", context)

@router.get("/select_mode", response_class=HTMLResponse)
async def read_select_mode(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "select_mode"})

    context = {"request": request}
    return templates.TemplateResponse("select_mode.html", context)

@router.get("/auto_drive", response_class=HTMLResponse)
async def read_auto_driving(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "auto_delivery"})

    context = {"request": request, "mode_msg": "On the way of Auto Delivery"}
    return templates.TemplateResponse("auto_drive.html", context)

@router.get("/guide", response_class=HTMLResponse)
async def read_guiding(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "guide"})

    context = {"request": request}
    return templates.TemplateResponse("guide.html", context)

@router.get("/follow", response_class=HTMLResponse)
async def read_following(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "follow"})

    context = {"request": request}
    return templates.TemplateResponse("follow.html", context)

@router.get("/stop", response_class=HTMLResponse)
async def read_stop(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "stop"})

    context = {"request": request}
    return templates.TemplateResponse("stop.html", context)

@router.get("/map", response_class=HTMLResponse)
async def read_map(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "map"})

    context = {"request": request}
    return templates.TemplateResponse("map.html", context)

@router.get("/take_a_break", response_class=HTMLResponse)
async def read_take_a_break(request: Request):
    context = {"request": request}
    return templates.TemplateResponse("take_a_break.html", context)

@router.get("/arrived_pick_up_counter", response_class=HTMLResponse)
async def read_arrived_pick_up_counter(request: Request):
    context = {"request": request}
    return templates.TemplateResponse("arrived_pick_up_counter.html", context)

@router.get("/cargo_open_pick_up", response_class=HTMLResponse)
async def read_cargo_open_pick_up(request: Request):
    context = {"request": request}
    return templates.TemplateResponse("cargo_open_pick_up.html", context)

@router.get("/gate", response_class=HTMLResponse)
async def read_at_the_gate(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "gate"})

    context = {"request": request}
    return templates.TemplateResponse("gate.html", context)

@router.get("/face_recognition", response_class=HTMLResponse)
async def read_face_identification(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "face_recognition"})

    context = {"request": request}
    return templates.TemplateResponse("face_recognition.html", context)

@router.get("/cargo_open_final", response_class=HTMLResponse)
async def read_cargo_open_final(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "cargo"})

    context = {"request": request}
    return templates.TemplateResponse("cargo.html", context)

@router.get("/thanks", response_class=HTMLResponse)
async def read_thanks(request: Request, ):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "thanks"})

    context = {"request": request}
    return templates.TemplateResponse("thanks.html", context)

# @router.get("/searching_person", response_class=HTMLResponse)
# async def read_searching_person(request: Request):
    
#     context = {"request": request}
#     return templates.TemplateResponse("searching_person.html", context)

@router.get("/return", response_class=HTMLResponse)
async def read_returning(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "return"})

    context = {"request": request}
    return templates.TemplateResponse("return.html", context)