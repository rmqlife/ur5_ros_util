import rospy
from ur_msgs.srv import SetIO
from time import sleep

class MyIO:
    def __init__(self, fun, pin):
        # Create a SetIO service client
        self.set_io_service = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)

        # Initialize fun and pin as instance variables
        self.fun = fun
        self.pin = pin
        self.state = 0.0

    def set_io_state(self, state):
        # Call the service with the desired parameters
        response = self.set_io_service(fun=self.fun, pin=self.pin, state=state)
        if response.success:
            rospy.loginfo(f"Successfully set IO pin {self.pin} to state {state}")
        else:
            rospy.logerr(f"Failed to set IO pin {self.pin} to state {state}")

    def toggle_state(self):
        # Toggle the state between 0 and 1
        self.state = 1.0 if self.state == 0.0 else 0.0
        self.set_io_state(self.state)

class Vaccum(MyIO):
    def __init__(self, fun, pin):   
        super().__init__(fun=1, pin=16)
    
    def grab(self):
        super() .set_io_state(1)

    def release(self):
        super().set_io_state(0)

class HLGripper(MyIO):
    def __init__(self, fun, pin):
        super().__init__(fun=1, pin=16)

    def grab(self):
        super().set_io_state(0)

    def release(self):
        super().set_io_state(1)

if __name__ == "__main__":
    # Initialize the ROS node
    rospy.init_node('set_io_client')
    # Initialize fun and pin here
    fun = 1
    pin = 16

    gripper = HLGripper(fun, pin)
    gripper.release()
    sleep(1)
    for i in range(3):
        gripper.grab()
        sleep(1)
        gripper.release()
        sleep(1)
