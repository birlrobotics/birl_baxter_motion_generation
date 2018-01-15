import rospy
from moveit_msgs.msg import  PlanningScene, ObjectColor
from geometry_msgs.msg import PoseStamped, Pose
import ipdb
class Addmodels:
    def __init__(self,scene,reference_frame):
        self.scene_pub = rospy.Publisher('planning_scene', PlanningScene, queue_size=5) 
        self._scene = scene
        self.colors = dict()
        rospy.sleep(1)
        # Give each of the scene objects a unique name
        table_id ='table'
        box1_id = 'box1'
        box2_id = 'box2'
        # Remove leftover objects from a previous run
        self._scene.remove_world_object(table_id)
        self._scene.remove_world_object(box1_id)
        self._scene.remove_world_object(box2_id)
        # Give the scene a chance to catch up
        self.colors = dict()
        rospy.sleep(1)
        # Set the height of the table off the ground
        table_ground = 0
        
        # Set the length, width and height of the table and boxes
        table_size = [0.2, 0.7, 0.01]
        box1_size = [0.1, 0.05, 0.05]
        box2_size = [0.05, 0.05, 0.15]
        
        # Add a table top and two boxes to the scene
        table_pose = PoseStamped()
        table_pose.header.frame_id = reference_frame
        table_pose.pose.position.x = 0.26
        table_pose.pose.position.y = 0.0
        table_pose.pose.position.z = table_ground + table_size[2] / 2.0
        table_pose.pose.orientation.w = 1.0
        scene.add_box(table_id, table_pose, table_size)
        
        box1_pose = PoseStamped()
        box1_pose.header.frame_id = reference_frame
        box1_pose.pose.position.x = 0.21
        box1_pose.pose.position.y = -0.1
        box1_pose.pose.position.z = table_ground + table_size[2] + box1_size[2] / 2.0
        box1_pose.pose.orientation.w = 1.0   
        scene.add_box(box1_id, box1_pose, box1_size)
        
        box2_pose = PoseStamped()
        box2_pose.header.frame_id = reference_frame
        box2_pose.pose.position.x = 0.19
        box2_pose.pose.position.y = 0.15
        box2_pose.pose.position.z = table_ground + table_size[2] + box2_size[2] / 2.0
        box2_pose.pose.orientation.w = 1.0   
        scene.add_box(box2_id, box2_pose, box2_size)
        ipdb.set_trace()
        # Make the table red and the boxes orange
        self.setColor(table_id, 0.8, 0, 0, 1.0)
        self.setColor(box1_id, 0.0, 0.4, 0, 1.0)
        self.setColor(box2_id, 0.5, 0.4, 0, 1.0)
        
        # Send the colors to the planning scene
        self.sendColors()    
        
        # Set the target pose in between the boxes and above the table
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.pose.position.x = 0.2
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = table_pose.pose.position.z + table_size[2] + 0.05
        target_pose.pose.orientation.w = 1.0
    
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
           