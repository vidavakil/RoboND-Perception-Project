#!/usr/bin/env python

# Import modules
import numpy as np
import sklearn
from sklearn.preprocessing import LabelEncoder
import pickle
from sensor_stick.srv import GetNormals
from sensor_stick.features import compute_color_histograms
from sensor_stick.features import compute_normal_histograms
from visualization_msgs.msg import Marker
from sensor_stick.marker_tools import *
from sensor_stick.msg import DetectedObjectsArray
from sensor_stick.msg import DetectedObject
from sensor_stick.pcl_helper import *

import rospy
import tf
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
from pr2_robot.srv import *
from rospy_message_converter import message_converter
import yaml

# For the project run `roslaunch pr2_robot pick_place_project.launch` and then `rosrun pr2_robot project_template.py`, both from ~/catkin
# src/sensor_stick/scripts/capture_features.py
# run `roslaunch sensor_stick training.launch` and then `rosrun sensor_stick capture_features.py` to generate training data
# Once capture_features is run, the result, training_set.sav is added to ~/catkin_ws (since I ran it there)
# To train, rosrun sensor_stick train_svm.py  The output will be model.sav in ~/catkin_ws
# To save output_i.yaml files, need to change pick_place_project.launch to use world_name testi.world, and load pick_list_i.yaml

# Helper function to get surface normals
def get_normals(cloud):
    get_normals_prox = rospy.ServiceProxy('/feature_extractor/get_normals', GetNormals)
    return get_normals_prox(cloud).cluster

# Helper function to create a yaml friendly dictionary from ROS messages
def make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose):
    yaml_dict = {}
    yaml_dict["test_scene_num"] = test_scene_num.data
    yaml_dict["arm_name"]  = arm_name.data
    yaml_dict["object_name"] = object_name.data
    yaml_dict["pick_pose"] = message_converter.convert_ros_message_to_dictionary(pick_pose)
    yaml_dict["place_pose"] = message_converter.convert_ros_message_to_dictionary(place_pose)
    return yaml_dict

# Helper function to output to yaml file
def send_to_yaml(yaml_filename, dict_list):
    data_dict = {"object_list": dict_list}
    with open(yaml_filename, 'w') as outfile:
        yaml.dump(data_dict, outfile, default_flow_style=False)

# Callback function for your Point Cloud Subscriber
def pcl_callback(pcl_msg):

    # Convert ROS msg to PCL data
    cloud = ros_to_pcl(pcl_msg)

    # TODO: Statistical Outlier Filtering
    outlier_filter = cloud.make_statistical_outlier_filter()
    outlier_filter.set_mean_k(60)
    threshold_scale_factor = 1.0
    outlier_filter.set_std_dev_mul_thresh(threshold_scale_factor)
    cloud = outlier_filter.filter()

    # Voxel Grid Downsampling
    vox = cloud.make_voxel_grid_filter()
    LEAF_SIZE = 0.005
    vox.set_leaf_size(LEAF_SIZE, LEAF_SIZE, LEAF_SIZE)
    cloud_filtered = vox.filter()

    # PassThrough Filter
    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'z'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.6
    axis_max = 1.1
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'y'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = -0.45
    axis_max = 0.45
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    passthrough = cloud_filtered.make_passthrough_filter()
    filter_axis = 'x'
    passthrough.set_filter_field_name(filter_axis)
    axis_min = 0.34
    axis_max = 0.86
    passthrough.set_filter_limits(axis_min, axis_max)
    cloud_filtered = passthrough.filter()

    ros_pass_through_filtered_cloud = pcl_to_ros(cloud_filtered)
    pcl_pass_through_filtered_pub.publish(ros_pass_through_filtered_cloud)

    # RANSAC Plane Segmentation
    seg = cloud_filtered.make_segmenter()
    seg.set_model_type(pcl.SACMODEL_PLANE)
    seg.set_method_type(pcl.SAC_RANSAC)
    max_distance = 0.01
    seg.set_distance_threshold(max_distance)
    inliers, coefficients = seg.segment()

    # Extract inliers and outliers
    cloud_table = cloud_filtered.extract(inliers, negative=False)
    cloud_objects = cloud_filtered.extract(inliers, negative=True)

    # Euclidean Clustering
    white_cloud = XYZRGB_to_XYZ(cloud_objects)
    tree = white_cloud.make_kdtree()

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    ec = white_cloud.make_EuclideanClusterExtraction()

    # Convert PCL data to ROS messages
    ec.set_ClusterTolerance(0.01)
    ec.set_MinClusterSize(50)
    ec.set_MaxClusterSize(20000)
    # Search the k-d tree for clusters
    ec.set_SearchMethod(tree)
    # Extract indices for each of the discovered clusters
    cluster_indices = ec.Extract()

    #Assign a color corresponding to each segmented object in scene
    cluster_color = get_color_list(len(cluster_indices))

    color_cluster_point_list = []

    for j, indices in enumerate(cluster_indices):
        for i, indice in enumerate(indices):
            color_cluster_point_list.append([white_cloud[indice][0],
                                             white_cloud[indice][1],
                                             white_cloud[indice][2],
                                             rgb_to_float(cluster_color[j])])

    #Create new cloud containing all clusters, each with unique color
    cluster_cloud = pcl.PointCloud_PointXYZRGB()
    cluster_cloud.from_list(color_cluster_point_list)

    # Create Cluster-Mask Point Cloud to visualize each cluster separately
    # Convert PCL data to ROS messages
    ros_cloud_table = pcl_to_ros(cloud_table)
    ros_cloud_objects = pcl_to_ros(cloud_objects)
    ros_cluster_cloud = pcl_to_ros(cluster_cloud)
    ros_statistical_filtered_cloud = pcl_to_ros(cloud)

    # Publish ROS messages
    pcl_objects_pub.publish(ros_cloud_objects)
    pcl_table_pub.publish(ros_cloud_table)
    pcl_cluster_pub.publish(ros_cluster_cloud)
    pcl_statistical_filtered_pub.publish(ros_statistical_filtered_cloud)

    # Classify the clusters! (loop through each detected cluster one at a time)
    detected_objects_labels = []
    detected_objects = []

    for index, pts_list in enumerate(cluster_indices):
        # Grab the points for the cluster
        pcl_cluster = cloud_objects.extract(pts_list)
        # convert the cluster from pcl to ROS using helper function
        ros_pcl_cluster = pcl_to_ros(pcl_cluster)

        # Compute the associated feature vector
        chists = compute_color_histograms(ros_pcl_cluster, using_hsv=True)
        normals = get_normals(ros_pcl_cluster)
        nhists = compute_normal_histograms(normals)
        feature = np.concatenate((chists, nhists))

        # Make the prediction
        prediction = clf.predict(scaler.transform(feature.reshape(1,-1)))
        label = encoder.inverse_transform(prediction)[0]
        detected_objects_labels.append(label)

        # Publish a label into RViz
        label_pos = list(white_cloud[pts_list[0]])
        label_pos[2] += .4
        object_markers_pub.publish(make_label(label,label_pos, index))

        # Add the detected object to the list of detected objects.
        do = DetectedObject()
        do.label = label
        do.cloud = ros_pcl_cluster  # Was ros_cluster_cloud!!!
        detected_objects.append(do)

    rospy.loginfo('Detected {} objects: {}'.format(len(detected_objects_labels), detected_objects_labels))

    # Publish the list of detected objects
    detected_objects_pub.publish(detected_objects)

    # Suggested location for where to invoke your pr2_mover() function within pcl_callback()
    # Could add some logic to determine whether or not your object detections are robust
    # before calling pr2_mover()
    try:
        pr2_mover(detected_objects)
    except rospy.ROSInterruptException:
        pass

# function to load parameters and request PickPlace service
def pr2_mover(object_list):

    # Initialize variables
    labels = []
    centroids = [] # to be list of tuples (x, y, z)
    dict_list = []

    # Get/Read parameters
    object_pick_list = rospy.get_param('/object_list')

    # Parse parameters into individual variables
    pick_list_map = {3: 1, 5:2, 8:3}
    test_scene = pick_list_map[len(object_pick_list)]
    test_scene_num = Int32()
    test_scene_num.data = test_scene

    arm_map = {"green": "right", "red": "left"}

    # TODO: Rotate PR2 in place to capture side tables for the collision map

    # Loop through the pick list
    for pick_object in object_pick_list:
        pick_object_name = pick_object['name']
        pick_object_group = pick_object['group']

        object_name = String()
        object_name.data = str(pick_object_name)

        # Find the object in the scene that matches the label
        matched_object = None
        for object in object_list:
            if object.label == pick_object_name:
                matched_object = object
                break

        if matched_object is None:
            print("Pick object: ", pick_object_name, " was not found in the scene")
            continue

        # Get the PointCloud for a given object and obtain it's centroid
        labels.append(matched_object.label)
        points_arr = ros_to_pcl(matched_object.cloud).to_array()
        centroid = np.mean(points_arr, axis=0)[:3]
        centroids.append(centroid)
        pick_pose = Pose()
        pick_pose.position.x = np.asscalar(centroid[0])
        pick_pose.position.y = np.asscalar(centroid[1])
        pick_pose.position.z = np.asscalar(centroid[2])

        # TODO: How about the orientation of grasp?

        # Assign the arm to be used for pick_place
        arm_name = String()
        arm_name.data = str(arm_map[pick_object_group])

        # TODO: Create 'place_pose' for the object
        # bottom of the dropbox is 0.605. Put the place_pose 0.05 above it.
        place_pose = Pose()
        if arm_name == "left":
            place_pose.position.x = 0.0
            place_pose.position.y = 0.71 
            place_pose.position.z = 0.655
        else:
            place_pose.position.x = 0.0
            place_pose.position.y = -0.71 
            place_pose.position.z = 0.655 

        # Create a list of dictionaries (made with make_yaml_dict()) for later output to yaml format
        yaml_dict = make_yaml_dict(test_scene_num, arm_name, object_name, pick_pose, place_pose)
        dict_list.append(yaml_dict)

        # Wait for 'pick_place_routine' service to come up
        rospy.wait_for_service('pick_place_routine')

        # TODO: Uncomment the following to run pick and place!
        # Since I am not handling collision yet, collisions can happen and objects may move.
        # So, for now I am commenting below lines, just to get to writing the yaml file.
#        try:
#            pick_place_routine = rospy.ServiceProxy('pick_place_routine', PickPlace)
#
#            # Insert your message variables to be sent as a service request
#            resp = pick_place_routine(test_scene_num, object_name, arm_name, pick_pose, place_pose)
#
#            print ("Response: ",resp.success)
#
#        except rospy.ServiceException, e:
#            print "Service call failed: %s"%e

    # Output your request parameters into output yaml file
    yaml_filename = "output_" + str(test_scene) + ".yaml"
    print("yaml_filename = ", yaml_filename)
    send_to_yaml(yaml_filename, dict_list)


if __name__ == '__main__':

    # ROS node initialization
    rospy.init_node('clustering', anonymous=True)

    # Create Subscribers
    pcl_sub = rospy.Subscriber("/pr2/world/points", pc2.PointCloud2, pcl_callback, queue_size=1)

    # Create Publishers
    pcl_statistical_filtered_pub = rospy.Publisher("/pcl_statistical_filtered", PointCloud2, queue_size=1)
    pcl_pass_through_filtered_pub = rospy.Publisher("/pcl_pass_through_filtered", PointCloud2, queue_size=1)
    pcl_objects_pub = rospy.Publisher("/pcl_objects", PointCloud2, queue_size=1)
    pcl_table_pub = rospy.Publisher("/pcl_table", PointCloud2, queue_size=1)
    pcl_cluster_pub = rospy.Publisher("/pcl_cluster", PointCloud2, queue_size=1)
    object_markers_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)
    detected_objects_pub = rospy.Publisher("/detected_objects", DetectedObjectsArray, queue_size=1)

    # Load Model From disk
    model = pickle.load(open('model.sav', 'rb'))
    clf = model['classifier']
    encoder = LabelEncoder()
    encoder.classes_ = model['classes']
    scaler = model['scaler']

    # Initialize color_list
    get_color_list.color_list = []

    # Spin while node is not shutdown
    while not rospy.is_shutdown():
        rospy.spin()

