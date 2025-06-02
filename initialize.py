import time        
import jkrc  
     
tcp_pos=[0,0,0,0,0,0]
robot = jkrc.RC("10.5.5.100")
robot.login()
robot.power_on()  
robot.enable_robot()

robot.set_user_frame_data(2, tcp_pos,"2.1")
robot.set_user_frame_id(2)
print(robot.get_tcp_position())
tcp_pos=robot.get_tcp_position()[1]
robot.set_user_frame_data(2, tcp_pos,"2.1")
robot.set_user_frame_id(2)
tcp_pos=[0,0,-200,0,0,0]
ret=robot.linear_move(tcp_pos,1,True,10)  
tcp_pos=[0,0,0,0,0,0]
robot.set_user_frame_data(2, tcp_pos,"2.1")
robot.set_user_frame_id(2)
print(robot.get_tcp_position())
tcp_pos=robot.get_tcp_position()[1]
robot.set_user_frame_data(2, tcp_pos,"2.1")
robot.set_user_frame_id(2)
print(robot.get_tcp_position())
robot.logout() 
import cv2
cap = cv2.VideoCapture(0)
ret, img = cap.read()
if ret:
    cv2.imshow('c',img)
cv2.imwrite('c.jpg',img)