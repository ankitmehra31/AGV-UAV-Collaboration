
#!usr/bin/python3

import cv2
import ultralytics
from ultralytics import YOLO
import sys
import time
import rospy
from custom_msg_python.msg import custom
import random
import timeit
import time

# Initialize YOLO model
model = YOLO('/home/chitti/rambo2/src/custom_msg_python/src/best_cloudy.pt')

# Initialize video capture from the default camera (change the index if using a different camera)\

fourcc=cv2.VideoWriter_fourcc(*'XVID')
#out=cv2.VideoWriter('out_cv.mp4', fourcc, 20.0, (320,320))

cap = cv2.VideoCapture(0)
prev_frame_time=0
new_frame_time=0
global cc
global fire_detected
global xp,yp
xp=[0,0,0]
yp=[0,0,0] 
fire_detected = False
cc=0

# Below code is just for checking
msg=[320/2,320/2]
def talk():
    thickness = 3
    start_time = time.time()
    frame_count = 0
    global fire_detected
    pub=rospy.Publisher("custom_message",custom,queue_size=10)
    rospy.init_node("custom_publisher",anonymous=True)
    # rate=rospy.Rate(6)
    msg=custom()
    msg.header.frame_id="map"

    
    while not rospy.is_shutdown():
        # Read frame from camera
        global cc
        ret, frame = cap.read()
        frame=cv2.resize(frame,(320,320))
        #msg.header.stamp = rospy.Time.now()

        results=model(source=frame, show=False, conf=0.6, save=False, max_det=1)
        
        detected = False  # Initialize detection flag
        
        for result in results:
            if len(result.boxes) > 0:  # Check if any boxes are detected
                detected = True  # Set detected to True
                for bbox in result.boxes.xyxy:
                    x1, y1, x2, y2 = bbox[0].item(), bbox[1].item(), bbox[2].item(), bbox[3].item()
                    # Draw bounding box around detected object (fire)
                    color = (0, 255, 0)  # Green color
                    thickness = 2
                    final= cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness)
                    centroid = [(x1 + x2) / 2, (y1 + y2) / 2]
                    #msg=centroid  
                    msg.coordinates=centroid                
                    break  # Only consider the first detected box and centroid
            else:
                msg.coordinates=[320/2,320/2]
                #msg=[320/2,320/2]
                final=frame
        #out.write(final[:,:,::-1])
        end_time = time.time()
        elapsed_time = end_time - start_time
        frame_count += 1
        if elapsed_time > 1:
            fps = frame_count / elapsed_time
            start_time = end_time
            frame_count = 0

        if 'final' in locals():
            # Display FPS on the frame
            cv2.putText(final, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        if detected:
            fire_detected = True
            print("UAV Detected")
            #print("Centroid:", centroid)
            print("True")
            cmd = "1"
            # break  # Stop the code execution if fire is detected
        if not fire_detected:
            print("No UAV detected")
            print("False")
            msg.coordinates=[0,0]
            cmd = "0"
            # break
        
        # Display frame with bounding boxes and centroid
        cv2.imshow("UAV Detection", frame)
        cv2.waitKey(1)
        # Check for 'q' key press to exit the loop
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        #while not rospy.isshutdown():
        rospy.loginfo(msg)
        pub.publish(msg)
        #rospy.spin()
        #rate.sleep()

# talk()
if __name__=="__main__":
    try:
        talk()
    except rospy.ROSInterruptException:
        pass
