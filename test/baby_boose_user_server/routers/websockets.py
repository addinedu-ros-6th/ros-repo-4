import cv2
from fastapi import APIRouter, WebSocket, WebSocketDisconnect
import json, cv2, numpy as np, base64, asyncio
from state.shared_state import session_data, recv_msg_q
from utils.helpers import get_user_id_from_scope

router = APIRouter()


@router.websocket("/ws/ticket_scan")
async def websocket_ticket_scan(websocket: WebSocket):
    await websocket.accept()
    qr = cv2.QRCodeDetector()
    try:
        while True:
            data = await websocket.receive_text()
            message = json.loads(data)
            if message['type'] == 'frame':
                frame_data = message['data']
                # Decode Base64 to image
                img_bytes = base64.b64decode(frame_data)
                nparr = np.frombuffer(img_bytes, np.uint8)
                img = cv2.imdecode(nparr, cv2.IMREAD_COLOR)

                if "ticket" not in session_data:
                    session_data["ticket"] = {}

                data, box, straight_qrcode = qr.detectAndDecode(img)
                if data:
                    data_rows = data.split('\n')
                    for d in data_rows:
                        key_value = d.split(" : ")
                        if len(key_value) == 2:
                            key, value = key_value
                            session_data["ticket"][key.strip()] = value.strip()

                    print(f"user ticket is detected : {session_data['ticket']}")

                    # Send navigation command to client
                    navigate_message = json.dumps({
                        'type': 'navigate',
                        'data': '/confirmation'
                    })
                    await websocket.send_text(navigate_message)
                    break  # Close connection after detection
    except WebSocketDisconnect:
        print("WebSocket disconnected")

@router.websocket("/ws/cargo_open")
async def websocket_cargo_open(websocket: WebSocket):
    await websocket.accept()
    
    user_id = get_user_id_from_scope(websocket.scope)
    robot_id = session_data[user_id]["robot_id"]

    try:
        while True:
            # Asynchronously get message from recv_msg_q
            loop = asyncio.get_event_loop()
            data = await loop.run_in_executor(None, recv_msg_q.get)

            display_message = "Not yet"
            if data.get("robot_id") == robot_id:
                state = data.get("state", "").split(" ")
                if len(state) >= 3 and state[0] == "cargo":
                    if state[1] == "closed" and state[2] == "empty":
                        display_message = "Waiting for the Gate open" 
                    elif state[1] == "open" and state[2] == "empty":
                        display_message = "Please put your luggage"
                    elif state[1] == "open" and state[2] == "full":
                        display_message = "Loaded!!"
                    elif state[1] == "closed" and state[2] == "full":
                        display_message = "done"
                    else:
                        display_message = "Ready..."
            
            # Send status update to client
            await websocket.send_text(json.dumps({"type": "status_update", "message": display_message}))
    except WebSocketDisconnect:
        print("Cargo Open WebSocket disconnected")