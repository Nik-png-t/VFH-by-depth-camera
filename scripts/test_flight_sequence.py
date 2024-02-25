import rospy
import time
from threading import Thread
from mavros_msgs.srv import CommandTOL, CommandBool, SetMode
from mavros_msgs.msg import PositionTarget, State


class FlightCommander:
    def __init__(self) -> None:
        self.current_state = None
        self.pose = None
        self._setpoint = None
        rospy.Subscriber('/mavros/state', State, self.state_callback)
        rospy.Subscriber('/mavros/local_position/pose', State, self.state_callback)
        self.setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        self.setpoint_thread = None
        self.stop_thread = False
        rospy.wait_for_service('/mavros/cmd/arming')
        rospy.wait_for_service('/mavros/set_mode')
        
        
    def start_setpoint_thread(self):
        self.stop_thread = False
        self.setpoint_thread = Thread(target=self.send_setpoints_loop)
        self.setpoint_thread.start()

    def stop_setpoint_thread(self):
        self.stop_thread = True
        if self.setpoint_thread:
            self.setpoint_thread.join()
            self.setpoint_thread = None

    def send_setpoints_loop(self, target_rate: float=20):
        rate = rospy.Rate(target_rate)  
        while not self.stop_thread and not rospy.is_shutdown():
            if self._setpoint is None:
                continue
            if self.current_state.armed == False:
                self.arm_vehicle
            self.setpoint_pub.publish(self._setpoint)
            rate.sleep()
        self._setpoint = None
    
    def state_callback(self, msg):
        self.current_state = msg
        # if msg.mode == "OFFBOARD" and self.setpoint_thread is None:
        #     self.start_setpoint_thread()
        if msg.mode != "OFFBOARD" and self.setpoint_thread is not None:
            self.stop_setpoint_thread()
        
    def pose_callback(self, msg):
        self.pose = msg
    
    def do_takeoff(self, altitude: float=1.0):
        
        while (self.current_state is None):
            pass
        
        if not self.current_state.armed:
            if self.arm_vehicle():
                print("Vehicle armed successfully!")
            else:
                print("Failed to arm the vehicle.")
                return

        self.set_position(0, 0, altitude, 0)
        if self.current_state.mode != "OFFBOARD":
            if self.change_mode("OFFBOARD"):
                print("Offboard mode set successfully!")
            else:
                print("Failed to set Offboard mode.")
                return
    
    def arm_vehicle(self, arm: bool=True):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
            arm_response = arm_service(arm)  # True to arm, False to disarm
            return arm_response.success
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False
        
        
    def change_mode(self, mode: str):
        try:
            if mode == "OFFBOARD" and self.setpoint_thread is None:
                self.start_setpoint_thread()
                time.sleep(1)
            set_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
            response = set_mode_service(0, mode)
            return True
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)
            return False
        
        
    def set_position(self, x: float, y: float, z: float, yaw: float=0):
        if self.current_state:
            self.change_mode("OFFBOARD")
        setpoint = PositionTarget()
        setpoint.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ + PositionTarget.IGNORE_YAW_RATE
        setpoint.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        setpoint.position.x = x
        setpoint.position.y = y
        setpoint.position.z = z
        setpoint.yaw = yaw
        self._setpoint = setpoint

    
    
if __name__ == "__main__":
    rospy.init_node('test_flight_commander')
    commander = FlightCommander()
    commander.do_takeoff()
    commander.set_position(0, 0, 1, 0)
    time.sleep(5)
    commander.set_position(0, -3, 1, -1.57)
    time.sleep(10)
    commander.set_position(-5, -3, 1, -1.57)
    time.sleep(10)
    commander.set_position(-10, -3, 1, -1.57)
    time.sleep(10)
    commander.set_position(-10, -3, 1, -1.57)
    time.sleep(5)
    commander.set_position(-10, 8, 1, 1.57)
    time.sleep(10)
    commander.set_position(-5, 8, 1, 1.57)
    time.sleep(10)
    commander.set_position(0, 8, 1, 1.57)
    time.sleep(10)
    commander.set_position(-0, 0, 1, 1.57)
    time.sleep(10)
    commander.change_mode("LAND")