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
from enum import Enum

class DataSource(Enum):
	SNAPSHOT = 1
	RECORDING = 2
	ALL = 3

class FaultyDataException(Exception):
	"""Raised when the read data is not in expected format"""
	pass

class NoTransformException(Exception):
	"""Raised when the read data is not in expected format"""
	pass

class YCBMulticamDataset:

	def __init__(self, path):

		self.path = path
		self.cams = ["astra", "basler_tof", "ensenso", "kinect2", "pico_flexx", "realsense_r200", "xtion"]
		self.classes = ['002_master_chef_can', '003_cracker_box', '004_sugar_box', '005_tomato_soup_can', '006_mustard_bottle', \
						'007_tuna_fish_can', '008_pudding_box', '009_gelatin_box', '010_potted_meat_can', '011_banana', '019_pitcher_base', \
						'021_bleach_cleanser', '024_bowl', '025_mug', '035_power_drill', '036_wood_block', '037_scissors', '040_large_marker', \
						'051_large_clamp', '052_extra_large_clamp', '061_foam_brick', 'getriebelager', 'raceway_long', 'raceway_short']

		self.excluded_objects = ['getriebelager', 'raceway_long', 'raceway_short']

		self.calib_scenes = ["_calib", "_checkerboard_calib"]
		self.scenes = [scene for scene in os.listdir(path+"/scenes") if scene not in self.calib_scenes]

		self.positions = range(9)

		self.__ground_truths = {}
		self.__read_ground_truths()

		self.__tfBuffer = tf2.Buffer(rospy.Duration(120))



	def __read_ground_truths(self):

		""" Reads in the ground truth annotations from yaml
		"""
		for scene in self.scenes:
			stream = file(self.path + "/scenes/" + scene + "/recognition_truth.yaml", 'r')
			gt = yaml.load(stream)
			self.__ground_truths[scene] = []
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
				self.__ground_truths[scene].append({"cls": object["class"], "pose": obj_pose})



	def get_data(self, cam, source=DataSource.ALL, scenes=None, cloud=False):
		""" returns a generator over a recording for a given scene and camera

		:param source: The data source from which the frames are taken. That can be either the Snapshots, the Recordings or both
		:param cam: The camera from which data should be fetched

		:returns: a generator over all available frames in the following format:
			- rgb_img		| list of sensor_msgs::Image
			- rgb_info		| list of sensor_msgs::CameraInfo
			- depth_img		| list of sensor_msgs::Image
		  	- depth_info	| list of sensor_msgs::CameraInfo
			if cloud == True:
			- cloud			| list of sensor_msgs::PointCloud2

			- ground truth: ( in frame of rgb image )
				list of dicts containing:
				- pose		| tf2_geometry_msgs::PoseStamped
				- cls		| string
			- metadata:
				dict containing:
				- scene_name| string
				- camera	| string
				- source	| string
				- clutter	| boolean

			the whole blob will be several GB big if cloud is enabled
		"""
		if scenes != None:
			use_scenes = scenes
		else:
			use_scenes = self.scenes
		print(use_scenes)
		if source == DataSource.RECORDING or source == DataSource.ALL :
			for scene in use_scenes:
				try:
					for frame in self.__get_recording_frames(scene, cam, cloud):
						yield frame
				except(FaultyDataException):
					print("Ignoring faulty data in scene: {}, with cam: {}. Continuing with next scene.", scene, cam)
		if source == DataSource.SNAPSHOT or source == DataSource.ALL:
			print("snap")
			for scene in use_scenes:
				for pos in self.positions:
					try:
						for frame in self.__get_snapshot(scene, pos, cam, cloud):
							yield frame
					except(FaultyDataException):
						print("Ignoring faulty data in scene: {}, position: {}, with cam: {}. Continuing with next position.", scene, pos, cam)




	def __get_recording_frames(self, scene, cam, cloud=False):

		""" returns a generator over a recording for a given scene and camera

		:param scene: The scene from which data should be fetched
		:param camera: The camera from which data should be fetched

		:returns: a generator over all available frames in the following format:
			- rgb_info		| list of sensor_msgs::Image
			- rgb_info		| list of sensor_msgs::CameraInfo
			- depth_img		| list of sensor_msgs::Image
		  	- depth_info	| list of sensor_msgs::CameraInfo
			if cloud == True:
			- cloud			| list of sensor_msgs::PointCloud2

			- ground truth: ( in camera coordinates )
				list of dicts containing:
				- pose 		| tf2_geometry_msgs::PoseStamped
				- cls		| string
			- metadata:
				dict containing:
				- scene_name| string
				- camera	| string
				- source	| string
				- clutter	| boolean

			the whole blob will be several GB big if cloud is enabled

		"""
		self.__tfBuffer.clear()
		bag_path = self.path + "/scenes/" + scene + "/recordings/"+ cam + ".bag"

		tf_bag = rosbag.Bag(bag_path)
		try:
			for topic, msg, t in tf_bag.read_messages(topics={"/tf"}):
				if topic == "/tf":
					for tf in msg.transforms:
						self.__tfBuffer.set_transform(tf, "")
		except (genpy.DeserializationError):
			raise FaultyDataException

		try:
			for topic, msg, t in tf_bag.read_messages(topics={"/tf_static"}):
				if topic == "/tf_static":
					for tf in msg.transforms:
						self.__tfBuffer.set_transform_static(tf, "")
		except (genpy.DeserializationError): # some recordings have faulty data in tf_static.. the static tfs dont change between scenes so we can use the tfs of another scene
			tf_bag.close()
			tf_bag = rosbag.Bag(self.path + "/scenes/003_007_009_010/recordings/"+ cam + ".bag")
			for topic, msg, t in tf_bag.read_messages(topics={"/tf_static"}):
				if topic == "/tf_static":
					for tf in msg.transforms:
						self.__tfBuffer.set_transform_static(tf, "")

		tf_bag.close()

		topics = ['depth_img', 'rgb_img', 'depth_info', 'rgb_info']
		if cloud:
			topics.append('cloud')

		# write images/caminfo to snapshot
		#create dicts
		frames = {}
		for t in topics:
			frames[t] = []
		ground_truth = {}
		#collect frames in which there was no transform
		invalid_frames = []
		index = 0
		bag = rosbag.Bag(bag_path)
		for topic, msg, t in bag.read_messages(topics=topics):
			if topic not in ["/tf", "/tf_static", "/joint_states"]:
				frames[topic].append(msg)
			if topic == 'rgb_img':
				# transform each gt object into rgb image frame
				ground_truth[index] = []
				for object in self.__ground_truths[scene]:
					try:
						# we know that there is no transform via ensenso
						tmpPose = object["pose"]
						tmpPose.header.frame_id = cam + "/aruco_node/aruco_ref"
						pose_transformed = self.__transform_pose(tmpPose, msg.header.frame_id, msg.header.stamp)
						ground_truth[index].append({"cls": object["cls"], "pose": pose_transformed})
					except(NoTransformException):
						# if we cannot transform it: throw this frame away
						print("did not find transform")
						invalid_frames.append(index)
				index += 1
		bag.close()
		#check if data is valid i.e. same amount of frames for all topics
		frame_count = -1
		for f in frames:
			if frame_count == -1:
				frame_count = len(frames[f])
			else:
				if len(frames[f]) != frame_count:
					raise FaultyDataException
		# convert to output data format
		for i in range(frame_count):
			if i in invalid_frames:
				continue
			result_frame = {}
			for topic in topics:
				result_frame[topic] = frames[topic][i]
			result_frame["ground_truth"] = ground_truth[i]
			result_frame["metadata"] = self.__get_metadata(scene, DataSource.RECORDING, cam)
			yield result_frame

	def __get_snapshot(self, scene, position, cam, cloud=False):

		""" returns a generator over a single snapshot for a given scene, position and camera

		:param scene: The scene from which data should be fetched
		:param position: The position for which data should be fetched
		:param camera: The camera from which data should be fetched

		:returns: a generator over four frames in the following format:
			- rgb_img		|	list of sensor_msgs::Image
			- rgb_info		|	list of sensor_msgs::CameraInfo
			- depth_img		|	list of sensor_msgs::Image
		  	- depth_info	|	list of sensor_msgs::CameraInfo
			if cloud == True:
			- cloud			|	list of sensor_msgs::PointCloud2

			- ground truth: ( in camera coordinates )
				list of dicts containing:
				- pose		|	tf2_geometry_msgs::PoseStamped
				- cls		|	string
			- metadata:
				dict containing:
				- scene_name|	string
				- camera	|	string
				- source	|	string
				- clutter	|	boolean
				- position	|	int
			the whole blob will be ~500mb big if cloud is enabled

		"""

		self.__tfBuffer.clear()
		# get the transforms in order to transform the ground truth to the respective image frame
		tf_bag = rosbag.Bag(self.path + "/scenes/" + scene + "/snapshots/" + str(position) + "/tf.bag")
		for topic, msg, t in tf_bag.read_messages(topics={"/tf", "/tf_static"}):
			if topic == "/tf": # ugly warnings for unused tfs... just dont use them
				for tf in msg.transforms:
					if tf.header.frame_id != "base_link":
						self.__tfBuffer.set_transform(tf, "")
			if topic == "/tf_static":
				for tf in msg.transforms:
					self.__tfBuffer.set_transform_static(tf, "")
		tf_bag.close()

		topics = ['depth_img', 'rgb_img', 'depth_info', 'rgb_info']
		if cloud:
			topics.append('cloud')

		frames = {}
		# create dicts and lists
		for t in topics:
			frames[t] = []
		# get the images, info and optionally cloud
		bag = rosbag.Bag(self.path + "/scenes/" + scene + "/snapshots/" + str(position) + "/" + cam + "/snapshot.bag")
		# write images/caminfo to snapshot
		for topic, msg, t in bag.read_messages(topics=topics):
			frames[topic].append(msg)
		bag.close()

		for f in frames:
			if len(frames[f]) != 4:
				raise FaultyDataException

		# transform ground truth to rgb frame
		target_frame = frames["rgb_img"][0].header.frame_id

		ground_truth = []
		for object in self.__ground_truths[scene]:
			try:
				pose_transformed = self.__transform_pose(object["pose"], target_frame, rospy.Time(0))
			except(NoTransformException):
				raise FaultyDataException
			ground_truth.append({"cls": object["cls"], "pose": pose_transformed})

		for frame_idx in range(0, 4):
			snapshot = {}
			for topic in frames:
				snapshot[topic] = frames[topic][frame_idx]
			snapshot["ground_truth"] = ground_truth
			snapshot["metadata"] = self.__get_metadata(scene, DataSource.SNAPSHOT, cam, position)
			yield snapshot

	def __get_metadata(self, scene, source, cam, position=None):

		""" returns a dict with metadata

			metadata is a dict with the following members
				- name		|	string
				- camera	|	string
				- source	|	DataSource
				- clutter	|	boolean
				- position	|	int
		"""
		flags = {}

		flags["camera"] = cam

		flags["source"] = "SNAPSHOT" if source == DataSource.SNAPSHOT else "RECORDING"
		flags["scene_name"] = scene

		if position != None:
			flags["position"] = position
		else:
			flags["position"] = None
		if "_clutter" in scene:
			flags["clutter"] = True
		else:
			flags["clutter"] = False

		return flags

	def __transform_pose(self, pose, target_frame, time):
			#try transforming with main cam transform
			if (self.__tfBuffer.can_transform(target_frame, pose.header.frame_id, time)):
				# print("found tranform from {} to {}".format(pose.header.frame_id, target_frame))
				return self.__tfBuffer.transform(pose, target_frame)
			else:
				print("Cannot transform directly to {}, trying from other cameras").format(target_frame)
				for alternative_cam in self.cams:
					tmpPose = pose
					tmpPose.header.frame_id = alternative_cam + "/aruco_node/aruco_ref"

					if (self.__tfBuffer.can_transform(target_frame, tmpPose.header.frame_id, time)):
						print("Found transform via {}").format(alternative_cam)
						return self.__tfBuffer.transform(tmpPose, target_frame)
				raise NoTransformException
