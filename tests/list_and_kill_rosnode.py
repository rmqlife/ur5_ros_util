import rospy
import rosnode


if __name__ == '__main__':
    # Initialize the ROS node
    rospy.init_node('list_ros_nodes_node', anonymous=False)
    
    while True:
        nodes = rosnode.get_node_names()
        print("Active ROS Nodes:")
        for i, node in enumerate(nodes, start=1):
            print(f"{i}: {node}")

        # Input a number to stop the corresponding node
        node_index = int(input("Enter the number of the node to stop: ")) - 1
        if node_index < 0 or node_index >= len(nodes):
            print("Invalid input. Please enter a valid number.")
            continue
        node_name = nodes[node_index]
        rosnode.kill_nodes([node_name])
