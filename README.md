Introduction
============

Since computer vision is an important aspect in the field of robotics,
the choice of suitable 3D sensors is of great significance. In order to
evaluate the suitability of these sensors, there exists the need for
datasets that include multiple sensors in a comparable fashion. This
dataset was conceived in order to satisfy that need and provide a basis
for a quantitative comparison of 3D sensors with regard to object
recognition.\
The dataset provides 32 scenes containing a selection of 20 objects from
the YCB Object and Model set @ycb_dataset and is designed to compare
6DOF pose estimation of object recognition algorithms applied to the
data of different cameras.

The scenes also contain a marker board featuring fiducial markers
(ArUco-markers @Aruco2014) which provides a reference frame shared among
the cameras and the ground truth annotations. This marker board also
acts as a calibration means for the inter-camera calibration.

The data in this dataset are divided into two parts, based on the
acquisition method. For the first part the cameras were held in place at
nine fixed positions and recorded sequentially in order to avoid
interference between the sensors. The second part contains recordings of
the scene for which each camera moved in a trajectory around the scene.
Again, the recordings were performed sequentially in order to avoid
interference.

Important note: This dataset is not conceived as a training dataset i.e.
for neuronal networks. Since the data are heavily biased (same
background and marker board visible in every scene), the resulting
trained model will most likely yield suboptimal results in data other
than what is provided.

Sensors {#sensors .unnumbered}
=======

We included the following 3D-sensors into the dataset. Sensors that do
not yield RGB images are marked with an asterisk.

\[t\]<span>0.33</span> **Structured Light**\

-   ASUS Xtion Pro Live

-   Orbbec Astra

\[t\]<span>0.33</span> **Active Stereo**\

-   Ensenso N35

-   Intel RealSense R200

\[t\]<span>0.33</span> **Time of Flight**\

-   Microsoft Kinect2

-   PMD CamBoard pico flexx\*

-   Basler ToF Engineering Sample\*

Structure of the dataset {#structure-of-the-dataset .unnumbered}
========================

The dataset was recorded using ROS @ROS and hence is stored in the
versatile rosbag format. In each rosbag file there are multiple messages
containing the following data:

1.  Raw RGB/infrared image (for convenience also named ’rgb’ in
    the data)

2.  RGB/infrared camera intrinsics

3.  Raw Depth image

4.  Depth camera intrinsics

5.  Pointcloud registered to the rgb/infrared image

In total there are 8064 annotated frames from the snapshots and about
47000 annotated frames from the trajectory recordings. For all frames we
provide the transforms between the camera frames as well as the
transforms from the camera rig to the marker board reference frame.

The folder structure of the dataset is displayed in fig.
\[fig:folder\_structure\].

Provided code for evaluation {#provided-code-for-evaluation .unnumbered}
============================

In this repository we provide an interface for easy access to the
dataset. It is written in Python 2.7 and uses ROS, which is both needed
in order to use this code.

The YCBMulticamDataset class contains the following and :

- List of used camera names as strings

- List of used classes

returns a Python Generator which iterates over all available frames of a
given camera and from a given source (i.e. fixed positions, trajectories
or both)

1.  : string containing the name of desired camera as in , e.g.: ’astra’

2.  : a value of the enum which can be either (data from fixed
    positions), (data from trajectories) or (all available data)

3.  a boolean that indicates, whether the point cloud data should be
    retrieved

The generated frames have the format depicted in
\[fig:frame\_structure\].

Sample data {#sample-data .unnumbered}
===========

In the following figures we provide some sample data from the dataset.
Figure \[fig:cloud\_raster\] contains the depth data captured by all
used sensors, as well as ground truth annotations on its own and
rendered into the scene for the pointcloud of the Ensenso N35 camera.
Figure \[fig:cloud\_color\_raster\] contains the RGB pointclouds from
the RGBD cameras for the same scene.

<span>ccc</span> ![Pointclouds from the dataset depicting the same scene
from the same position, as well as the ground truth object arrangement
and the ground truth transformed into the Ensenso pointcloud.<span
data-label="fig:cloud_raster"></span>](pics/raster_clouds/astra_axis "fig:"){width=".33\linewidth"}
& ![Pointclouds from the dataset depicting the same scene from the same
position, as well as the ground truth object arrangement and the ground
truth transformed into the Ensenso pointcloud.<span
data-label="fig:cloud_raster"></span>](pics/raster_clouds/basler_tof_axis "fig:"){width=".33\linewidth"}
& ![Pointclouds from the dataset depicting the same scene from the same
position, as well as the ground truth object arrangement and the ground
truth transformed into the Ensenso pointcloud.<span
data-label="fig:cloud_raster"></span>](pics/raster_clouds/ensenso_axis "fig:"){width=".33\linewidth"}\
Orbbec Astra & Basler ToF & Ensenso N35\
![Pointclouds from the dataset depicting the same scene from the same
position, as well as the ground truth object arrangement and the ground
truth transformed into the Ensenso pointcloud.<span
data-label="fig:cloud_raster"></span>](pics/raster_clouds/kinect2_axis "fig:"){width=".33\linewidth"}
& ![Pointclouds from the dataset depicting the same scene from the same
position, as well as the ground truth object arrangement and the ground
truth transformed into the Ensenso pointcloud.<span
data-label="fig:cloud_raster"></span>](pics/raster_clouds/pico_flexx_axis "fig:"){width=".33\linewidth"}
& ![Pointclouds from the dataset depicting the same scene from the same
position, as well as the ground truth object arrangement and the ground
truth transformed into the Ensenso pointcloud.<span
data-label="fig:cloud_raster"></span>](pics/raster_clouds/realsense_r200_axis "fig:"){width=".33\linewidth"}\
Microsoft Kinect2& PMD CamBoard pico flexx & Intel RealSense R200\
![Pointclouds from the dataset depicting the same scene from the same
position, as well as the ground truth object arrangement and the ground
truth transformed into the Ensenso pointcloud.<span
data-label="fig:cloud_raster"></span>](pics/raster_clouds/gt_no_cloud_f_black "fig:"){width=".33\linewidth"}
& ![Pointclouds from the dataset depicting the same scene from the same
position, as well as the ground truth object arrangement and the ground
truth transformed into the Ensenso pointcloud.<span
data-label="fig:cloud_raster"></span>](pics/raster_clouds/xtion_axis "fig:"){width=".33\linewidth"}
& ![Pointclouds from the dataset depicting the same scene from the same
position, as well as the ground truth object arrangement and the ground
truth transformed into the Ensenso pointcloud.<span
data-label="fig:cloud_raster"></span>](pics/raster_clouds/gt_ensenso_f_black "fig:"){width=".33\linewidth"}\
Ground truth & ASUS Xtion Pro Live & Ground truth in the scene\

<span>ccc</span> ![RGB pointclouds from the dataset depicting the same
scene from the same position.<span
data-label="fig:cloud_color_raster"></span>](pics/raster_clouds/astra_f "fig:"){width=".33\linewidth"}
& ![RGB pointclouds from the dataset depicting the same scene from the
same position.<span
data-label="fig:cloud_color_raster"></span>](pics/raster_clouds/xtion_f "fig:"){width=".33\linewidth"}
& ![RGB pointclouds from the dataset depicting the same scene from the
same position.<span
data-label="fig:cloud_color_raster"></span>](pics/raster_clouds/ensenso_f "fig:"){width=".33\linewidth"}\
Orbbec Astra & ASUS Xtion Pro Live & Ensenso N35\
![RGB pointclouds from the dataset depicting the same scene from the
same position.<span
data-label="fig:cloud_color_raster"></span>](pics/raster_clouds/kinect2_f "fig:"){width=".33\linewidth"}
& & ![RGB pointclouds from the dataset depicting the same scene from the
same position.<span
data-label="fig:cloud_color_raster"></span>](pics/raster_clouds/realsense_r200_f "fig:"){width=".33\linewidth"}\
Microsoft Kinect2& & Intel RealSense R200\

Figure \[fig:pos\_raster\] depicts the same scene from all fixed
positions we used, as recorded by the Ensenso N35 camera.

  -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
   ![Images of a scene from every fixed position, as recorded by the Ensenso N35<span data-label="fig:pos_raster"></span>](pics/raster_pos/ensenso_pos0 "fig:"){width=".33\linewidth"}   ![Images of a scene from every fixed position, as recorded by the Ensenso N35<span data-label="fig:pos_raster"></span>](pics/raster_pos/ensenso_pos1 "fig:"){width=".33\linewidth"}   ![Images of a scene from every fixed position, as recorded by the Ensenso N35<span data-label="fig:pos_raster"></span>](pics/raster_pos/ensenso_pos2 "fig:"){width=".33\linewidth"}
   ![Images of a scene from every fixed position, as recorded by the Ensenso N35<span data-label="fig:pos_raster"></span>](pics/raster_pos/ensenso_pos5 "fig:"){width=".33\linewidth"}   ![Images of a scene from every fixed position, as recorded by the Ensenso N35<span data-label="fig:pos_raster"></span>](pics/raster_pos/ensenso_pos4 "fig:"){width=".33\linewidth"}   ![Images of a scene from every fixed position, as recorded by the Ensenso N35<span data-label="fig:pos_raster"></span>](pics/raster_pos/ensenso_pos4 "fig:"){width=".33\linewidth"}
   ![Images of a scene from every fixed position, as recorded by the Ensenso N35<span data-label="fig:pos_raster"></span>](pics/raster_pos/ensenso_pos6 "fig:"){width=".33\linewidth"}   ![Images of a scene from every fixed position, as recorded by the Ensenso N35<span data-label="fig:pos_raster"></span>](pics/raster_pos/ensenso_pos7 "fig:"){width=".33\linewidth"}   ![Images of a scene from every fixed position, as recorded by the Ensenso N35<span data-label="fig:pos_raster"></span>](pics/raster_pos/ensenso_pos8 "fig:"){width=".33\linewidth"}
  -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
