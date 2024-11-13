import cv2

img = cv2.imread("qr1.png")
qr = cv2.QRCodeDetector()
data, box, straight_qrcode = qr.detectAndDecode(img)
print(data)