import qrcode

data = """
name : Mr. 99
passport : M12345678
flight number : AC2505
from : ICN
to : JFK
gate : K18
date : 25/11/2024
boarding time : 03:30
seat : 5A
"""

myQR = qrcode.make(data)
myQR.save("ticket_qr.png")