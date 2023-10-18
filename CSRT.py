import cv2
import sys
from pyzbar.pyzbar import decode

my_track_method = cv2.legacy.TrackerCSRT_create()
tracking_initialized = False

cap = cv2.VideoCapture('QR_Code.mp4')  

frame_width = int(cap.get(3))
frame_height = int(cap.get(4))
size = (frame_width, frame_height)

result = cv2.VideoWriter('output1.avi', cv2.VideoWriter_fourcc(*'XVID'), 10, size)
while True:
    
    ret, frame = cap.read()
    if not ret:
        break
    if not tracking_initialized:
        
        decoded_objects = decode(frame)
        for obj in decoded_objects:
            qr_bbox = obj.rect
            
            left, top, width, height = qr_bbox
            tracking_bbox = (left, top, width, height)
            my_track_method.init(frame, tracking_bbox)
            tracking_initialized = True
    else:
        ret, tracking_bbox = my_track_method.update(frame)
        if ret:
            left, top, width, height = (int(tracking_bbox[0]), int(tracking_bbox[1]), int(tracking_bbox[2]), int(tracking_bbox[3]))
            right = left + width
            bottom = top + height
            cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
        else:
            cv2.putText(frame, "Tracking khong thanh cong!", (80, 100), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
    result.write(frame)
    cv2.imshow("Video", frame)
    key = cv2.waitKey(1) & 0xff
    if key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()