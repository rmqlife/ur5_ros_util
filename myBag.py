#!/usr/bin/env python
import rospy
import json
import numpy as np
import os

class MyBag:
    def __init__(self, filename=''):
        """
        Initialize the data saver.
        We will collect data in a dictionary with lists as values.
        """
        if filename and os.path.exists(filename):
            rospy.loginfo(f"Loading data from {filename}...")
            with open(filename, 'r') as json_file:
                self.data = json.load(json_file)
        else:       
            self.data = dict()
            rospy.loginfo(f"No data file found, creating new one at {filename}")
        rospy.on_shutdown(lambda: self.export_data(filename))

    def record(self, topic_name, data):
        if isinstance(data, np.ndarray):
            data = data.tolist()
        print(f'recording {topic_name}:{data}')

        if topic_name in self.data:
            self.data[topic_name].append(data)
        else:
            self.data[topic_name] = [data]
    
    def export_data(self, filename=''):
        if filename == '':  
            from datetime import datetime
            timestamp = datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
            filename = f"data/output_{timestamp}.json"
        rospy.loginfo(f"Exporting data to {filename}...")

        with open(filename, 'w') as json_file:
            json.dump(self.data, json_file, default=str, indent=4)  # Convert datetime, etc. to strings
        rospy.loginfo("Data exported successfully.")
    
if __name__ == '__main__':
    rospy.init_node('test_mybag', anonymous=True)
    
    # Initialize the data saver (array_saver)
    mybag = MyBag('data/output_2024-12-06-16-16-54.json')

    from mySensor.myFTSensor import MyFTSensor
    ft_sensor = MyFTSensor()
    
    from myRobotWithIK import init_robot
    robot = init_robot()

    while not rospy.is_shutdown():
        mybag.record('force', ft_sensor.force)
        mybag.record('robot pose', robot.get_pose())
        rospy.sleep(1/2)
        
    # When shutdown, export data to JSON file
    # array_saver.export_data()