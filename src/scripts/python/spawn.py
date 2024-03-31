import rospy
from gazebo_msgs.srv import SpawnModel, DeleteModel
from geometry_msgs.msg import Pose

def spawn_model(model_path, model_name, model_pose):
    """
    Function to spawn a model in Gazebo.
    """
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        with open(model_path, "r") as f:
            model_xml = f.read()
        spawn_sdf(model_name, model_xml, "/", model_pose, "world")
    except rospy.ServiceException as e:
        rospy.logerr("Spawn model service call failed: {0}".format(e))

def delete_model(model_name):
    """
    Function to delete a model from Gazebo.
    """
    rospy.wait_for_service('/gazebo/delete_model')
    try:
        delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        delete_model(model_name)
    except rospy.ServiceException as e:
        rospy.logerr("Delete model service call failed: {0}".format(e))

def main():
    # Initialize ROS node
    rospy.init_node('spawn_and_remove_tree_models')

    # Define the 16x4 matrix
    matrix = [
        [0, 0, 0, 0],
        [0, 0, 1, 0],
        [0, 1, 0, 0],
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
        [0, 0, 0, 1],
        [0, 0, 1, 0],
        [0, 0, 1, 0],
        [0, 1, 0, 0],
        [0, 1, 0, 0],
        [1, 0, 0, 0],
        [1, 0, 0, 0],
        [0, 0, 0, 1],
        [0, 1, 0, 0]
    ]

    # Define the size of each grid cell
    cell_size = 10

    # Define the path to the tree model SDF file
    tree_model_path = "/home/lena/sky_ws/src/sky_sim/models/palm_tree/model.sdf"
    # Define the path to the box model SDF file
    box_model_path = "/home/lena/sky_ws/src/sky_sim/models/wall/box.sdf"

    # List to store the names of spawned models
    spawned_model_names = []

    # Define the dimensions of the box and the gap between the matrix and the wall of boxes
    box_dimensions = [cell_size, cell_size, 20]  # Assuming the wall is 2 meters tall
    gap = 1  # Gap between the matrix and the wall

    # Iterate through the outer boundary of the matrix to spawn boxes
    for i in range(len(matrix) + 2):
        for j in range(len(matrix[0]) + 2):
            # If we're at a position outside the matrix, spawn a box
            if i == 0 or i == len(matrix) + 1 or j == 0 or j == len(matrix[0]) + 1:
                # Calculate the position of the box
                x = (j - 1) * cell_size - gap * (j == 0 or j == len(matrix[0]) + 1)
                y = (i - 1) * cell_size - gap * (i == 0 or i == len(matrix) + 1)
                z = box_dimensions[2] / 2  # Center the box on the ground
                # Define the pose of the box model
                model_pose = Pose()
                model_pose.position.x = x
                model_pose.position.y = y
                model_pose.position.z = z
                # Spawn the box model in Gazebo
                model_name = "box_" + str(i) + "_" + str(j)  # Unique model name
                spawn_model(box_model_path, model_name, model_pose)
                # Append the spawned model name to the list
                spawned_model_names.append(model_name)

    # Iterate through each cell in the matrix to spawn trees
    for i in range(len(matrix)):
        for j in range(len(matrix[0])):
            # Check if the cell contains a tree obstacle
            if matrix[i][j] == 1:
                # Calculate the position of the tree obstacle in Gazebo
                x = j * cell_size
                y = i * cell_size
                z = 0  # Assuming the trees are placed on the ground
                # Define the pose of the tree model
                model_pose = Pose()
                model_pose.position.x = x
                model_pose.position.y = y
                model_pose.position.z = z
                # Spawn the tree model in Gazebo
                model_name = "tree_" + str(i) + "_" + str(j)  # Unique model name
                spawn_model(tree_model_path, model_name, model_pose)
                # Append the spawned model name to the list
                spawned_model_names.append(model_name)

    # Wait for some time to let the models spawn
    rospy.sleep(5)

    # Remove the spawned models
    for model_name in spawned_model_names:
        delete_model(model_name)

    rospy.spin()

if __name__ == '__main__':
    main()
