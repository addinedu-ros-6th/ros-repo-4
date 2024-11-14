# main.py
import os
import cv2
import asyncio
import json
import base64

from fastapi import FastAPI
from fastapi.staticfiles import StaticFiles

from routers import pages, api, websockets
from state.shared_state import session_data, send_msg_q, recv_msg_q, TEMP_ROBOT_ID
from utils.helpers import get_user_id, get_user_id_from_scope

from tcp_server import TCPServer

# Initialize TCP Server
GOOGEESE_GUI_HOST = '172.25.70.196' 
GOOGEESE_GUI_PORT = 8889


# Initialize FastAPI app
app = FastAPI()

# Mount static directories
app.mount("/css", StaticFiles(directory="css"), name="css")
app.mount("/js", StaticFiles(directory="js"), name="js")
app.mount("/images", StaticFiles(directory="images"), name="images")

try:
    tcp_server = TCPServer(GOOGEESE_GUI_HOST, GOOGEESE_GUI_PORT, send_msg_q, recv_msg_q)
except Exception as e:
    print(f"Failed to initialize TCPServer: {e}")

# Include routers
app.include_router(pages.router)
app.include_router(api.router)
app.include_router(websockets.router)
