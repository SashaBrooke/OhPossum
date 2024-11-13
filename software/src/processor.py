# TODO: Documentation

import cv2
import socket
import numpy as np
import time
import base64
from ultralytics import YOLO
import torch

torch.cuda.set_device(0)
model = YOLO('yolov8n.pt')
model.fuse()
person = 0.0 # class ID
person_label = "Person"

BUFF_SIZE = 65536
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client_socket.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, BUFF_SIZE)
host_name = "OhPossum"
host_ip = socket.gethostbyname(host_name)
print(host_ip)
port = 9999
message = b'Hello'

client_socket.sendto(message, (host_ip, port))
fps, st, frames_to_count, cnt = (0, 0, 30, 0)

while True:
	packet,_ = client_socket.recvfrom(BUFF_SIZE)
	data = base64.b64decode(packet, ' /')
	npdata = np.fromstring(data, dtype=np.uint8)
	frame = cv2.imdecode(npdata, 1)
	results = model.predict(frame, stream=True, conf=0.4) # show=True
	
	for result in results:
		boxes = result.boxes.cpu().numpy()
		xyxys = boxes.xyxy
		confidences = boxes.conf
		class_ids = boxes.cls
		for (xyxy, confidence, class_id) in zip(xyxys, confidences, class_ids):
			if class_id == person:
				cv2.rectangle(frame, 
				  (int(xyxy[0]), int(xyxy[1])), 
				  (int(xyxy[2]), int(xyxy[3])), 
				  (0,255,0), 
				  2)
				
				# cv2.putText(frame, 
				#   str(person_label) + ": " + str(round(confidence, 2)), 
				#   (int(xyxy[0]), int(xyxy[3]) - 3), 
				#   cv2.FONT_HERSHEY_SIMPLEX, 
				#   0.5, 
				#   (0, 255, 0), 
				#   2)

	frame = cv2.putText(frame, 'FPS: ' + str(fps), (5, 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

	cv2.imshow("udp://" + str(host_ip), frame)
	key = cv2.waitKey(1) & 0xFF
	if key == ord('q'):
		client_socket.close()
		# TODO: disconnection from camera process
		break
	if cnt == frames_to_count:
		try:
			fps = round(frames_to_count / (time.time() - st))
			st = time.time()
			cnt = 0
		except:
			pass
	cnt+=1
