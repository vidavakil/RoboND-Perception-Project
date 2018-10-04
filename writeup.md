## Project: Perception Pick & Place
---

# Required Steps for a Passing Submission:
1. Extract features and train an SVM model on new objects (see `pick_list_*.yaml` in `/pr2_robot/config/` for the list of models you'll be trying to identify). 
2. Write a ROS node and subscribe to `/pr2/world/points` topic. This topic contains noisy point cloud data that you must work with.
3. Use filtering and RANSAC plane fitting to isolate the objects of interest from the rest of the scene.
4. Apply Euclidean clustering to create separate clusters for individual items.
5. Perform object recognition on these objects and assign them labels (markers in RViz).
6. Calculate the centroid (average in x, y and z) of the set of points belonging to that each object.
7. Create ROS messages containing the details of each object (name, pick_pose, etc.) and write these messages out to `.yaml` files, one for each of the 3 scenarios (`test1-3.world` in `/pr2_robot/worlds/`).  [See the example `output.yaml` for details on what the output should look like.](https://github.com/udacity/RoboND-Perception-Project/blob/master/pr2_robot/config/output.yaml)  
8. Submit a link to your GitHub repo for the project or the Python code for your perception pipeline and your output `.yaml` files (3 `.yaml` files, one for each test world).  You must have correctly identified 100% of objects from `pick_list_1.yaml` for `test1.world`, 80% of items from `pick_list_2.yaml` for `test2.world` and 75% of items from `pick_list_3.yaml` in `test3.world`.
9. Congratulations!  Your Done!

# Extra Challenges: Complete the Pick & Place
7. To create a collision map, publish a point cloud to the `/pr2/3d_map/points` topic and make sure you change the `point_cloud_topic` to `/pr2/3d_map/points` in `sensors.yaml` in the `/pr2_robot/config/` directory. This topic is read by Moveit!, which uses this point cloud input to generate a collision map, allowing the robot to plan its trajectory.  Keep in mind that later when you go to pick up an object, you must first remove it from this point cloud so it is removed from the collision map!
8. Rotate the robot to generate collision map of table sides. This can be accomplished by publishing joint angle value(in radians) to `/pr2/world_joint_controller/command`
9. Rotate the robot back to its original state.
10. Create a ROS Client for the “pick_place_routine” rosservice.  In the required steps above, you already created the messages you need to use this service. Checkout the [PickPlace.srv](https://github.com/udacity/RoboND-Perception-Project/tree/master/pr2_robot/srv) file to find out what arguments you must pass to this service.
11. If everything was done correctly, when you pass the appropriate messages to the `pick_place_routine` service, the selected arm will perform pick and place operation and display trajectory in the RViz window
12. Place all the objects from your pick list in their respective dropoff box and you have completed the challenge!
13. Looking for a bigger challenge?  Load up the `challenge.world` scenario and see if you can get your perception pipeline working there!


[//]: # (Image References)

[image1]: ./misc_images/pr2_worldpoints.png
[image2]: ./misc_images/statistical_filtered.png
[image3]: ./misc_images/passthrough_filtered.png
[image4]: ./misc_images/pcl_table.png
[image5]: ./misc_images/pcl_objects.png
[image6]: ./misc_images/pcl_cluster.png
[image7]: ./misc_images/pcl_cluster_camera_1.png
[image8]: ./misc_images/normalized_confusion_matrix.png
[image9]: ./misc_images/pcl_cluster_camera_2.png
[image10]: ./misc_images/pcl_cluster_camera_3.png


## [Rubric](https://review.udacity.com/#!/rubrics/1067/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.

You're reading it!

### Exercise 1, 2 and 3 pipeline implemented
#### 1. Complete Exercise 1 steps. Pipeline for filtering and RANSAC plane fitting implemented.

Used the following parameters for different filters, starting from the provided ones and doing some trial and error and point search:
- `mean = 60`, and `threshold_scale_factor = 1.0` for statistical outlier filtering.
- LEAF_SIZE = 0.005 for voxel filtering
- For the `'z'` pass through filter used `axis_min = 0.6`, and `axis_max = 1.1`, to filter out anything except for the tabletop and objects on it. I found them by trial and error.
- I also filtered out the point cloud in the `x` and `y` directions, to remove the front edge, and the wings of the table on the right and left. I used the RViz Select tool to select a couple of points at each corner of the table, and used the `Select` panel to investigate their coordinates. Finally, I used `axis_min = -0.45`, and `axis_max = 0.45` for `'y'`, and `axis_min = 0.34` and `axis_max = 0.86` for `'x'`.
- `max_distance = 0.01` for RANSAC place segmentation.

Created publishers for relevant topics: `\pcl_statistical_filtered`, `/pcl_pass_through_filtered`, `/pcl_table`, `/pcl_objects`.

Below snapshots show the results of each of these filters for `picklist_1`. Used Rviz, and selected the corresponding topic.

Original Tabletop point cloud:

![alt text][image1]

Output of statistical filtering:

![alt text][image2]

Output of pass-through filtering, keeps only the rectangle section of the table and the objects on it:

![alt text][image3]

Output of RANSAC plane segmentation for the table surface:

![alt text][image4]

The objects on the top of the plane, after the table surface is filtered by RANSAC.

![alt text][image5]


#### 2. Complete Exercise 2 steps: Pipeline including clustering for segmentation implemented.

Used the following parameters for this part of the project:

- For Euclidean Clustering: `cluster_tolerance = 0.01`, min_cluster_size = 50`, `max_cluster_size = 20000` (found by a couple steps of trial and error)

Added additional publishers for topic `\pcl_cluster`. The following is a snapshot of the result of clustering (note that the labels are added by a subsequent part of code for object recognition that is part of the next exercise).

![alt text][image6]

The following is a snapshot of the camera view of RViz:

![alt text][image7]


#### 2. Complete Exercise 3 Steps.  Features extracted and SVM trained.  Object recognition implemented.

- Extracted the labels of the objects for the three world scenarios from `~/catkin_ws/src/RoboND-Perception-Project/pr2_robot/config/pick_list_*.yaml`
- These objects are:
`
        'biscuits',
        'soap',
        'soap2',
        'book',
        'glue',
        'snacks',
        'eraser',
        'sticky_notes'
`
- Modified `"models"` in `src/sensor_stick/scripts/capture_features.py` to include the above list of the labels.  Also specified the number of random poses to be generated for each object to be 20.
- Ran `roslaunch sensor_stick training.launch` and then `rosrun sensor_stick capture_features.py` from `~/catkin_ws` and in separate terminals to generate training data for the SVM. The output of this training step is file `~/catkin_ws/training_set.sav`.
- To train the SVM model with the training data, ran `rosrun sensor_stick train_svm.py` in `~/catkin_ws`. The output SVM model is stored in `~/catkin_ws/model.sav`.
- The model's classifier is then used during pick and place to classify the point cloud of each of the object clusters in a scene.
- The features that are used/extracted for the object clusters, both during training and classification, are a concatenation of HSV color_histograms and normal_histograms.
- Feature extraction for the above is done by methods in script `~/catkin_ws/src/sensor_stick/src/sensor_stick/features.py`, and I used the following parameters for color and normals:
    `compute_color_histograms()`: `nbins = 32` and `bins_range = (0, 256)` for the three HSV color histograms. The three are then concatenated and normalized, for a total of 32 x 3 features.
    `compute_normal_histograms()`: `nbins = 32` and `bins_range = (0.0, 1.0)` for the three normal histograms. Again, the three are concatenated and normalized, for a total of 32 x 3 features.
- In the object recognition code, the detected objects are then added to a list, each with its labels and cluster point cloud, and markers for each are published to the `/object_markers` topic.
- The markers of the detected objects can be seen in all the previous RViz snapshots.
- The following is a snapshots of the normalized confusion matrix for the pick and place project (with 8 objects).

![alt text][image8]

### Pick and Place Setup

#### 1. For all three tabletop setups (`test*.world`), perform object recognition, then read in respective pick list (`pick_list_*.yaml`). Next construct the messages that would comprise a valid `PickPlace` request output them to `.yaml` format.

For this task, implemented function `pr2_mover()` in `project_template.py`. I have commented the part of this function that makes a call to service `pick_place_routine`, and I go straight to saving the output yaml files. The reason is that I have skipped the collision avoidance challenge and thus collisions do happen and objects are pushed around as a result, which makes their grasping and moving to the bins harder.

- Switching the world scenario requires making changes in `RoboND-Perception-Project/pr2_robot/launch/pick_place_project.launch` in two places, to use world_name `testi.wolrd`, and load `pick_list_i.yaml`.
- To identify the world scenario in `pr2_mover()` (i.e., the i for `test_i.world` and `pick_list_i.yaml` pair), I used a simple trick suggested by one of the students on slack, who had noticed that the three scenarios differ in the number of objects involved (3 objects in scenario #1, 5 objects in scenario #2, and 8 objects in scenario #3). So, by checking the number of objects returned by the call to `ros.get_param('/object_list')`, the id of the scenario is identified.
- For each of those objects, an object in the scene with a matching label is found (assuming that the SVM model has correctly labeled the objects in the scene), and then arguments that need to be passed to `make_yaml_dict()` are computed. If the object is not found, then nothing will be written for it into the yams file.
- As noted in slack discussions, when putting together the arguments for the call to `make_yaml_dict()`, care had to be taken in casting floats using `np.asscalar()` and strings using `str()`, to avoid yaml output issues.
- Choosing a right `'z'` offset from the bottom of the dropboxes for the `place_pose` was important. I chose an offset of 0.05.
- Objects in `test1.world` and `test3.world` are correctly identified. However, in `test2.world`, the book object is classified as a soap, thus having 80% accuracy.
- I do not compute and fill in any values for the orientation of pick or place. However, looks like the planner already knows the correct orientation for these worlds!

The following two images show the identified labels for scenarios #2 and #3.

![alt text][image9]

![alt text][image10]

Issues and future improvements:
- Compute the orientation of the gripper for pick and place poses.
- Since I have not implemented collision avoidance, collisions do happen and objects get pushed, and sometimes as a result fall to the ground. Adding collision avoidance should greatly help, and I plan to do that later on.
- Noticed that the gripper sometime opens up while it is moving up, and as a result the object gets dropped. Hopefully in a future lesson will see how to improve the timing of grasp.
- The current implementation is not suited for more complex worlds. E.g., if the robot has to rotate to map all of its work space.
- May need to use a larger training set for SVM, and try different kernels, or parameters to increase the accuracy of SVM.


