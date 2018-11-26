#!/usr/bin/env python
import rospy
import os.path
import yaml
from yaml import CLoader as Loader, CDumper as Dumper
import numpy as np
from YCBMulticamDataset import YCBMulticamDataset as ycb
from YCBMulticamDataset import DataSource
from pose_cnn.srv import posecnn_recognize_refined
from obj_pose_eval import pose_error, inout
from scipy.spatial import distance
from pyquaternion import Quaternion
from cv_bridge import CvBridge, CvBridgeError
import pypcd
data_path = "/home/dfki.uni-bremen.de/tgrenzdoerffer/storage/compare_data"
import errno


def eval_posecnn():
	rospy.sleep(10.)
	data = ycb(data_path)
	idx = 0

	for cam in data.cams:
		print("Evaluating {}".format(cam))
		for frame in data.get_data(cam, DataSource.ALL):
			rospy.wait_for_service('/posecnn_refined/posecnn_recognize_refined')
			try:
				req = rospy.ServiceProxy('/posecnn_refined/posecnn_recognize_refined', posecnn_recognize_refined)
				resp1 = req(frame["rgb_img"], frame["depth_img"], frame["rgb_info"])
				detections = vision_msgs2detection(data, resp1.detections)
				# pose needs to be converted into a more convenient format
				gt = []
				for entry in frame["ground_truth"]:
					gt.append({"cls" : entry["cls"], "pose" : make_pose(entry["pose"].pose.position, entry["pose"].pose.orientation)})
				result = eval_result(data, detections, gt, frame["depth_img"], frame["depth_info"])
				#save the intermediate results...
				save_detection(result, detections, gt, frame["metadata"], idx, cam)
			except rospy.ServiceException, e:
				print("Service call failed: %s"%e)
			idx += 1

def eval_result(data, detection, ground_truth, depth_img, depth_info):
	result = {}
	# calculate false positives/negatives
	for obj_cls in [o for o in data.classes if o not in data.excluded_objects]:
		#check if  obj_cls is in gt
		gt_classes = [entry["cls"] for entry in ground_truth]
		detection_classes = [entry["cls"] for entry in detection]

		vsd_error = None
		trans_error = None
		rot_error = None
		trans_error_2d = None

		if obj_cls not in gt_classes:
			correctness = None
		else:
			if obj_cls in gt_classes and obj_cls in detection_classes:
				gt_entry = [det for det in ground_truth if det["cls"] == obj_cls][0]
				detection_entry = [det for det in detection if det["cls"] == obj_cls][0]
				# check the pose errorrrrr
				# gt_2d_pos = [gt_entry["pose"]["position"][0], gt_entry["pose"]["position"][1]]
				# detection_2d_pos = [detection_entry["pose"]["position"][0], detection_entry["pose"]["position"][1]]
				# trans_error_2d = distance.euclidean(gt_2d_pos, detection_2d_pos)
				trans_error, rot_error, vsd_error = get_errors(obj_cls, detection_entry["pose"], gt_entry["pose"], depth_img, depth_info)
				# check range with tolerance: if object centers are within 5 cm, the recognition is considered correct
				if trans_error <= 0.05:
					correctness = "correct"
					print("CORRECT DETECTION! With euclidean distance: {}, rot error: {} and vsd error: {}".format(trans_error_2d, rot_error, vsd_error))
				else:
					correctness = "in_scene"
					print("Wrong pos :( distance: {}".format(trans_error))
			if obj_cls in gt_classes and obj_cls not in detection_classes:
				correctness = "false_neg"
			if obj_cls not in gt_classes and obj_cls in detection_classes:
				correctness = "false_pos"

		result[obj_cls+"_correctness"] = correctness # or "correct", "false_pos", "false_neg"
		result[obj_cls+"_vsd_error"] = vsd_error
		result[obj_cls+"_trans_error"] = trans_error
		# result[obj_cls+"_2d_trans_error"] = trans_error_2d
		result[obj_cls+"_rot_error"] = rot_error
	return result

def get_errors(obj_cls, det_pose, gt_pose, depth_img, depth_info):
	det_q = Quaternion(w = det_pose["orientation"][3], x = det_pose["orientation"][0], y = det_pose["orientation"][1], z = det_pose["orientation"][2])
	pose_est = {'R' : det_q.rotation_matrix, 't' : np.array(det_pose["position"])}

	gt_q = Quaternion(w = gt_pose["orientation"][3], x = gt_pose["orientation"][0], y = gt_pose["orientation"][1], z = gt_pose["orientation"][2])
	pose_gt = {'R' : gt_q.rotation_matrix, 't' : np.array(gt_pose["position"])}

	model = inout.load_ply(data_path + '/YCB_models/' + obj_cls + '/nontextured.ply')
	bridge = CvBridge()
	depth_test = bridge.imgmsg_to_cv2(depth_img, desired_encoding="passthrough")

	K = np.array(depth_info.K)
	K = np.reshape(K, (3,3))

	delta = 5
	tau = 50
	vsd = pose_error.vsd(pose_est, pose_gt, model, depth_test, delta, tau, K)
	te = np.asscalar(pose_error.te(pose_est['t'], pose_gt['t']))
	re = pose_error.re(pose_est['R'], pose_gt['R'])

	return te, re, vsd


def make_pose(position, orientation):
	pose = {}
	pose["position"] = {}
	pose["orientation"] = {}
	pose["position"] = [position.x,
							position.y,
							position.z]
	pose["orientation"] = [orientation.x,
								orientation.y,
								orientation.z,
								orientation.w]
	return pose

def vision_msgs2detection(data, vision_msg):
	detections = []
	for det in vision_msg.detections:
		object_class = data.classes[det.results[0].id-1] #our modified posecnn returns Detection3DArray msgs, the indices are ints and apparently offset by 1
		pose = make_pose(det.results[0].pose.pose.position, det.results[0].pose.pose.orientation)
		detections.append({"cls" : object_class, "pose" : pose})
	return detections

def save_detection(result, detection, ground_truth, metadata, idx, cam):
	data_points = []
	data_point = {}
	data_point["frame_idx"] = idx
	data_point["result"] = {}
	for data in metadata:
		data_point["result"][data] = metadata[data]
	for res in result:
		data_point["result"][res] = result[res]
	#stupid yaml file doesnt want np array :(

	data_point["ground_truth"] = ground_truth
	data_point["detection"] = detection
	data_points.append(data_point)



	yaml_path = data_path + '/results/' + cam + '/'+ str(idx) +'.yaml'
	if not os.path.isfile(yaml_path):
		try:
			os.makedirs(os.path.dirname(yaml_path))
		except OSError as exc: # Guard against race condition
			if exc.errno != errno.EEXIST:
				raise
		with open(yaml_path,'w') as yamlfile:
			yaml.dump(data_points, yamlfile, Dumper = Dumper)
			return

	# with open(yaml_path,'r') as yamlfile:
	# 	cur_yaml = yaml.load(yamlfile, Loader=Loader) # Note the safe_load

	# cur_yaml.append(data_point)

	# if cur_yaml:
	# 	with open(yaml_path,'w') as yamlfile:
	# 		yaml.dump(cur_yaml, yamlfile, Dumper = Dumper) # Also note the safe_dump




if __name__ == "__main__":
	rospy.init_node('posecnn_eval')
	eval_posecnn()

