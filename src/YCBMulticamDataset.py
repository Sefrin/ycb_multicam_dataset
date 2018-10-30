#!/usr/bin/env python

import os
import rosbag
import tf2_ros as tf2
import rospy
import yaml
from tf2_geometry_msgs import PoseStamped
from sensor_msgs.msg import *
from vision_msgs.msg import Detection3DArray
import numpy as np

class YCBMulticamDataset:

	def __init__(self, path, type = "all"):
		self.type = type # ["snapshot", "recording", "all"]
		self.path = path
		self.cams = ["astra", "basler_tof", "ensenso", "kinect2", "pico_flexx", "realsense_r200", "xtion"]
		self.classes = ['002_master_chef_can', '003_cracker_box', '004_sugar_box', '005_tomato_soup_can', '006_mustard_bottle', \
                         '007_tuna_fish_can', '008_pudding_box', '009_gelatin_box', '010_potted_meat_can', '011_banana', '019_pitcher_base', \
                         '021_bleach_cleanser', '024_bowl', '025_mug', '035_power_drill', '036_wood_block', '037_scissors', '040_large_marker', \
                         '051_large_clamp', '052_extra_large_clamp', '061_foam_brick', 'getriebelager', 'raceway_long', 'raceway_short']
		
		self.exclude_objects = ['getriebelager', 'raceway_long', 'raceway_short']

		self.scenes = [scene for scene in os.listdir(path+"/scenes") if scene != "_calib" and scene != "_checkerboard_calib"]
		self.positions = range(9)


		self.recording_scene_idx = 0
		self.recording_frame_idx = 0

		self.ground_truths = {}

		self._read_ground_truths()
		self.tfBuffer = tf2.Buffer(rospy.Duration(90))

	def _read_ground_truths(self):
		for scene in self.scenes:
			stream = file(self.path + "/scenes/" + scene + "/recognition_truth.yaml", 'r')
			gt = yaml.load(stream)
			self.ground_truths[scene] = []
			for object in gt:
				obj_pose = PoseStamped()
				obj_pose.header.frame_id = "aruco_ref"
				obj_pose.pose.position.x = object["position"]["x"]
				obj_pose.pose.position.y = object["position"]["y"] 
				obj_pose.pose.position.z = object["position"]["z"]
				obj_pose.pose.orientation.x = object["orientation"]["x"]
				obj_pose.pose.orientation.y = object["orientation"]["y"] 
				obj_pose.pose.orientation.z = object["orientation"]["z"]
				obj_pose.pose.orientation.w = object["orientation"]["w"]
				self.ground_truths[scene].append({"cls": object["class"], "pose": obj_pose})


	def list_cams(self):
		return self.cams

	def list_scenes(self, clutter=False): 
		return self.scenes

	def list_calib_scenes(self):
		return ["_calib, _checkerboard_calib"]

	def list_classes(self):
		return self.classes

	def get_num_classes(self):
		return len(self.classes)
	
	# returns a generator for either the snapshots, the recordings or both
	def get_data(self, type = "snapshot", cloud=False):
		for scene in self.scenes:
			for pos in self.positions:
				yield self.get_snapshot(scene, pos, cloud)

	#############################################################################
	# returns four frames in the following format:								#											
	# either from the snapshots or from the recordings (parameter )				#															
	# camera:       | i.e. astra, basler_tof, ...																#		
	# 	- rgb 	    | list of sensor_msgs::Image								#											
	# 	- rgb_info  | list of sensor_msgs::CameraInfo							#												
	#   - depth     | list of sensor_msgs::Image								#											
	#   - depth_info| list of sensor_msgs::CameraInfo							#												
	#	  if cloud == True:														#					
	# 	- cloud 	| list of sensor_msgs::PointCloud2							#												
	# 																			#	
	# 	- ground truth: ( in camera coordinates)
	#		list of dicts containing:						#											
	# 		- pose  | tf2_geometry_msgs::PoseStamped						#													
	#       - cls   | string													#						
	#																			#
	# the whole blob will be ~ 500mb big if cloud is enabled					#														
	#############################################################################																			
	def get_snapshot(self, scene, position, cloud=False):
		self.tfBuffer.clear()
		# get the transforms in order to transform the ground truth to the respective image frame
		bag = rosbag.Bag(self.path + "/scenes/" + scene + "/snapshots/" + str(position) + "/tf.bag")
		for topic, msg, t in bag.read_messages(topics={"/tf", "/tf_static"}):
			if topic == "/tf":
				for tf in msg.transforms:
					self.tfBuffer.set_transform(tf, "")
			if topic == "/tf_static":
				for tf in msg.transforms:
					self.tfBuffer.set_transform_static(tf, "")
		bag.close()

		topics = ['depth_img', 'rgb_img', 'depth_info', 'rgb_info'] 
		if cloud:
			topics.append('cloud')

		snapshot = {}
		for cam in self.cams:
			# create dicts and lists
			snapshot[cam] = {}
			for t in topics:
				snapshot[cam][t] = []
			# get the images, info and optionally cloud
			bag = rosbag.Bag(self.path + "/scenes/" + scene + "/snapshots/" + str(position) + "/" + cam + "/snapshot.bag")
			# write images/caminfo to snapshot
			for topic, msg, t in bag.read_messages(topics=topics):
				snapshot[cam][topic].append(msg)
			bag.close()
			# transform ground truth to rgb frame
			snapshot[cam]["ground_truth"] = []
			for object in [o for o in self.ground_truths[scene] if o["cls"] not in self.exclude_objects]:
				if (self.tfBuffer.can_transform(snapshot[cam]["rgb_img"][0].header.frame_id, "aruco_ref", rospy.Time(0))):
					snapshot[cam]["ground_truth"].append({"cls": object["cls"], "pose": self.tfBuffer.transform(object["pose"], snapshot[cam]["rgb_img"][0].header.frame_id)})
				else:
					print("Cannot transform object to ground truth, skipping this frame")
					return
		return snapshot