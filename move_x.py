import time        
import jkrc  
    
tcp_pos=[15,0,0,0,0,0]
robot = jkrc.RC("10.5.5.100")
robot.login() 
robot.power_on()
robot.enable_robot()
robot.set_user_frame_id(3)

ret=robot.linear_move(tcp_pos,1,True,10)  

robot.logout() 
