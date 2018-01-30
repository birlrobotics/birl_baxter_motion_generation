import rospy
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose
import ipdb
class Addmodels:
    def __init__(self,scene,reference_frame,end_effector_link):
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5) 
        self.scene = scene
        self.colors = dict()
        self.end_effector_link = end_effector_link
        self.reference_frame = reference_frame       
        rospy.sleep(1)
        
    def out_obstacles(self):
        # Give each of the scene objects a unique name
#        table_id ='table'
#        box1_id = 'box1'
#        box2_id = 'box2'
        container_front_id = 'container_front'
        container_left_id = 'container_left'
        container_right_id = 'container_right'
        container_back_id = 'container_back'
        container_bottom_id = 'container_bottom'
        
        table_ground = 0      
        # Set the length, width and height of the table and boxes
#        table_size = [0.2, 0.7, 0.01]
#        box1_size = [0.1, 0.05, 0.05]
#        box2_size = [0.05, 0.05, 0.15]
        
        container_length = 0.3
        container_height = 0.2
        container_thickness = 0.02
        distance_x = 0.4
        distance_y = -0.7
        distance_z = 0
        
        container_front_size = [container_thickness,container_length,container_height]
        container_back_size = [container_thickness,container_length,container_height]
        container_left_size = [container_length,container_thickness,container_height]
        container_right_size = [container_length,container_thickness,container_height]
        container_bottom_size = [container_length,container_length,container_thickness]

        container_front_pose = PoseStamped()
        container_front_pose.header.frame_id = self.reference_frame
        container_front_pose.pose.position.x = 0 + container_length
        container_front_pose.pose.position.y = distance_y
        container_front_pose.pose.position.z = distance_z
        container_front_pose.pose.orientation.w = 1.0
        self.scene.add_box(container_front_id, container_front_pose, container_front_size)
        
        container_back_pose = PoseStamped()  # left_right side 
        container_back_pose.header.frame_id = self.reference_frame
        container_back_pose.pose.position.x = 0
        container_back_pose.pose.position.y = distance_y
        container_back_pose.pose.position.z = distance_z
        container_back_pose.pose.orientation.w = 1.0
        self.scene.add_box(container_back_id, container_back_pose, container_back_size)
        
        container_left_pose = PoseStamped() 
        container_left_pose.header.frame_id = self.reference_frame
        container_left_pose.pose.position.x = 0 + container_length/2
        container_left_pose.pose.position.y = distance_y - container_length/2
        container_left_pose.pose.position.z = distance_z
        container_left_pose.pose.orientation.w = 1.0
        self.scene.add_box(container_left_id, container_left_pose, container_left_size)
        
        container_right_pose = PoseStamped()
        container_right_pose.header.frame_id = self.reference_frame
        container_right_pose.pose.position.x = 0 + container_length/2
        container_right_pose.pose.position.y = distance_y + container_length/2
        container_right_pose.pose.position.z = distance_z
        container_right_pose.pose.orientation.w = 1.0
        self.scene.add_box(container_right_id, container_right_pose, container_right_size)
        
        container_bottom_pose = PoseStamped()
        container_bottom_pose.header.frame_id = self.reference_frame
        container_bottom_pose.pose.position.x = 0 + container_length/2
        container_bottom_pose.pose.position.y = distance_y
        container_bottom_pose.pose.position.z = distance_z - container_height/2
        container_bottom_pose.pose.orientation.w = 1.0
        self.scene.add_box(container_bottom_id, container_bottom_pose, container_bottom_size)
        
#        # Add a table top and two boxes to the scene
#        table_pose = PoseStamped()
#        table_pose.header.frame_id = self.reference_frame
#        table_pose.pose.position.x = 0.26+x_out
#        table_pose.pose.position.y = 0.0
#        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
#        table_pose.pose.orientation.w = 1.0
#        self.scene.add_box(table_id, table_pose, table_size)
#        
#        box1_pose = PoseStamped()
#        box1_pose.header.frame_id = self.reference_frame
#        box1_pose.pose.position.x = 0.21+x_out
#        box1_pose.pose.position.y = -0.1
#        box1_pose.pose.position.z = table_ground + table_size[2] + box1_size[2] / 2.0
#        box1_pose.pose.orientation.w = 1.0   
#        self.scene.add_box(box1_id, box1_pose, box1_size)
#        
#        box2_pose = PoseStamped()
#        box2_pose.header.frame_id = self.reference_frame
#        box2_pose.pose.position.x = 0.19+x_out
#        box2_pose.pose.position.y = 0.15
#        box2_pose.pose.position.z = table_ground + table_size[2] + box2_size[2] / 2.0
#        box2_pose.pose.orientation.w = 1.0   
#        self.scene.add_box(box2_id, box2_pose, box2_size)
        #ipdb.set_trace()
        # Make the table red and the boxes orange
#        self.setColor(table_id, 0.8, 0, 0, 1.0)
#        self.setColor(box1_id, 0.0, 0.4, 0, 1.0)
#        self.setColor(box2_id, 0.5, 0.4, 0, 1.0)
        self.setColor(container_front_id, 0.8, 0, 0, 1.0)
        self.setColor(container_left_id, 0.8, 0, 0, 1.0)
        self.setColor(container_right_id, 0.8, 0, 0, 1.0)
        self.setColor(container_back_id, 0.8, 0, 0, 1.0)
        self.setColor(container_bottom_id, 0.8, 0, 0, 1.0)
        # Send the colors to the planning scene
        self.sendColors() 
        
        

    def attach_objects(self):
        self.scene.remove_attached_object(self.end_effector_link, 'tool')
        # Set the length, width and height of the object to attach
        tool_size = [0.05, 0.02, 0.02]
        # Create a pose for the tool relative to the end-effector
        p = PoseStamped()
        p.header.frame_id = self.end_effector_link        
        self.scene.attach_mesh
        
        # Place the end of the object within the grasp of the gripper
        p.pose.position.x = tool_size[0] / 2.0 - 0.025
        p.pose.position.y = 0.0
        p.pose.position.z = 0.0
        
        # Align the object with the gripper (straight out)
        p.pose.orientation.x = 0
        p.pose.orientation.y = 0
        p.pose.orientation.z = 0
        p.pose.orientation.w = 1
        # Attach the tool to the end-effector
        self.scene.attach_box(self.end_effector_link, 'tool', p, tool_size)
        
    
    def setColor(self,name, r, g, b, a = 0.9):
        # Initialize a MoveIt color object
        color = ObjectColor()
        
        # Set the id to the name given as an argument
        color.id = name
        
        # Set the rgb and alpha values given as input
        color.color.r = r
        color.color.g = g
        color.color.b = b
        color.color.a = a
        # Update the global color dictionary
        self.colors[name] = color
    
    # Actually send the colors to MoveIt!
    def sendColors(self):
        
        # Initialize a planning scene object
        p = PlanningScene()
    
        # Need to publish a planning scene diff        
        p.is_diff = True
        
        # Append the colors from the global color dictionary 
        for color in self.colors.values():
            p.object_colors.append(color)
        
        # Publish the scene diff
        self.scene_pub.publish(p)

if __name__ == "__main__":
    try:
        Addmodels()
    except KeyboardInterrupt:
        raise
           