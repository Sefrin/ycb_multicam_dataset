#!/usr/bin/env python
from YCBMulticamDataset import YCBMulticamDataset as ycb

def eval_posecnn():
	data = ycb("/home/dfki.uni-bremen.de/tgrenzdoerffer/storage/compare_data")
	for cam in data.get_data():
		print cam["ensenso"]["ground_truth"]

if __name__ == "__main__":
	eval_posecnn()



