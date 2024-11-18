import queue

session_data = {}
send_msg_q = queue.Queue()
recv_msg_q = queue.Queue()
TEMP_ROBOT_ID = 5
GATE = 8