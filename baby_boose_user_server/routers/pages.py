# routers/pages.py
from fastapi import APIRouter, Request, HTTPException
from fastapi import status
from fastapi.responses import HTMLResponse, JSONResponse
from fastapi.middleware.cors import CORSMiddleware
from fastapi.templating import Jinja2Templates
from state.shared_state import session_data, send_msg_q, TEMP_ROBOT_ID, GATE
from utils.helpers import get_user_id
import os
# import mysql.connector

router = APIRouter()
templates = Jinja2Templates(directory="templates")


DB_CONFIG = {
    "host" : "127.0.0.1",
    "user" : "no_password_user",
    "password" : "",
    "database" : "googeese",
}
# def get_db_connection():
#     """Establish a database connection."""
#     try:
#         conn = mysql.connector.connect(**DB_CONFIG)
#         return conn
#     except mysql.connector.Error as err:
#         raise HTTPException(status_code=500, detail=f"Database connection failed: {err}")

# 시작 화면 start.html
# uvicorn main:app --reload
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
   
    context = {"request": request, "file_name": user_id}
    return templates.TemplateResponse("face_identification.html", context)

@router.get("/ticket_scan", response_class=HTMLResponse)
async def read_ticket_scan(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "ticket_scan"})

    context = {"request": request}
    return templates.TemplateResponse("ticket_scan.html", context)

@router.get("/confirmation", response_class=HTMLResponse)
async def read_confirmation(request: Request):
    """
    Fetch QR information, save to DB, and render confirmation page.
    """
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state": "confirmation"})

    # Extract ticket information from session_data
    ticket = session_data.get("ticket", {})
    name = ticket.get("name", "N/A")
    passport = ticket.get("passport", "N/A")
    flight_number = ticket.get("flight number", "N/A")
    from_location = ticket.get("from", "N/A")
    to_location = ticket.get("to", "N/A")
    gate = ticket.get("gate", "N/A")
    boarding_time = ticket.get("boarding time", "N/A")
    seat = ticket.get("seat", "N/A")

    # Save ticket information to the database
    conn = get_db_connection()
    cursor = conn.cursor()
    try:
        cursor.execute(
            """
            INSERT INTO user_info (name, passport, flight_number, `from`, `to`, gate, boarding_time, seat)
            VALUES (%s, %s, %s, %s, %s, %s, %s, %s)
            """,
            (name, passport, flight_number, from_location, to_location, gate, boarding_time, seat),
        )
        conn.commit()
    except Exception as e:
        conn.rollback()
        raise HTTPException(status_code=500, detail=f"Failed to save QR data: {e}")
    finally:
        cursor.close()
        conn.close()

    # Pass data to the confirmation template
    context = {
        "request": request,
        "image_name": user_id,
        "user_name": name,
        "flight_no": flight_number,
        "gate": gate,
        "boarding_time": boarding_time,
    }

    return templates.TemplateResponse("confirmation.html", context)


@router.get("/loading", response_class=HTMLResponse)
async def read_cargo_open(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "cargo open"})

    context = {"request": request}
    return templates.TemplateResponse("loading.html", context)

@router.get("/select_mode", response_class=HTMLResponse)
async def read_select_mode(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "select_mode"})

    context = {"request": request}
    return templates.TemplateResponse("select_mode.html", context)

@router.get("/auto_delivery", response_class=HTMLResponse)
async def read_auto_delivery(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : f"auto_delivery {GATE}"})

    context = {"request": request, "mode_msg": "On the way of Auto Delivery"}
    return templates.TemplateResponse("auto_delivery.html", context)

@router.get("/guide", response_class=HTMLResponse)
async def read_guide(request: Request):
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

@router.get("/follow_pause", response_class=HTMLResponse)
async def read_pause(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "follow_pause"})

    context = {"request": request}
    return templates.TemplateResponse("follow_pause.html", context)

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

@router.get("/arrived_gate", response_class=HTMLResponse)
async def read_at_the_gate(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "arrived_gate"})

    context = {"request": request}
    return templates.TemplateResponse("arrived_gate.html", context)

@router.get("/face_recognition", response_class=HTMLResponse)
async def read_face_identification(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "face_recognition"})

    context = {"request": request}
    return templates.TemplateResponse("face_recognition.html", context)

@router.get("/unloading", response_class=HTMLResponse)
async def read_cargo_open_final(request: Request):
    user_id = get_user_id(request)
    send_msg_q.put({"robot_id": session_data[user_id]["robot_id"], "state" : "cargo open"})

    context = {"request": request}
    return templates.TemplateResponse("unloading.html", context)

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


@router.post("/enqueue_action", response_class=JSONResponse)
async def enqueue_action(request: Request):
    try:
        user_id = get_user_id(request)
        robot_id = session_data[user_id]["robot_id"]
        
        # Parse the incoming JSON data
        data = await request.json()
        action = data.get("action")
        
        if not action:
            raise HTTPException(status_code=status.HTTP_400_BAD_REQUEST, detail="Action not provided")
        
        # Map actions to states if necessary
        action_to_state = {
            "cargo close": "cargo close",
        }
        
        state = action_to_state.get(action, action)  # Default to action if no mapping exists
        
        # Enqueue the message
        send_msg_q.put({"robot_id": robot_id, "state": state})
        
        return JSONResponse(content={"status": "success", "message": f"Action '{action}' enqueued."})
    
    except Exception as e:
        print(f"Error in enqueue_action: {e}")
        raise HTTPException(status_code=status.HTTP_500_INTERNAL_SERVER_ERROR, detail="Internal Server Error")