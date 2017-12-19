#!/usr/bin/env python

"""
Author: Zac Ravichandran
Module to provide trained SVM, also has method for cross validation
"""

from sklearn import svm
import numpy as np
from sklearn.model_selection import train_test_split
from sklearn.model_selection import cross_validate
from sklearn import preprocessing
import os
import matplotlib.pyplot as plt


def load_in_data(file, is_stop_signs = False):
	data = []
	labels = []
	with open(file, 'r') as f:
		for line in f:
			data_point = line.replace(" ", "").split(",")
			point = [float(feature.strip("\n")) for feature in data_point[1:]]
			data.append(point)
			labels.append(int(data_point[0]) if not is_stop_signs else 1)

	d= np.array(data)
	y = np.array(labels)
	return d, y

class SVM():
	def __init__(self, c=2.500000, gamma=0.305000):
		d, y = self.get_data()
		self.scaler = preprocessing.StandardScaler().fit(d)
		d = self.scaler.transform(d)
		self.sign_svm = self.get_fit_svm(d, y, gamma, c)

	def classify_point(self, point):
		return self.sign_svm.predict(self.scaler.transform(point))

	def get_fit_svm(self, d, y, gamma, c):
		return svm.SVC(kernel='rbf', C=c, gamma=gamma).fit(d, y)

	def get_data(self):
		dir_path = os.path.dirname(os.path.realpath(__file__))
		file_loc = dir_path + "/"

		d_signs, y_signs = load_in_data(file_loc + 'stop_sign_data_2.txt', is_stop_signs = True)
		d_noise, y_noise = load_in_data(file_loc + 'not_stop_sign_data_2.txt')

		d_signs_2, y_signs_2 = load_in_data(file_loc + 'stop_sign_data.txt', is_stop_signs = True)
		d_noise_2, y_noise_2 = load_in_data(file_loc + 'not_stop_sign_data.txt')

		d = np.concatenate([d_signs, d_noise, d_signs_2, d_noise_2], axis=0)
		y = np.concatenate([y_signs, y_noise, y_signs_2, y_noise_2], axis=0)

		return d, y	

	def load_in_data(self, file, is_stop_signs = False):
		data = []
		labels = []
		with open(file, 'r') as f:
			for line in f:
				data_point = line.replace(" ", "").split(",")
				point = [float(feature.strip("\n")) for feature in data_point[1:]]
				data.append(point)
				labels.append(int(data_point[0]) if not is_stop_signs else 1)

		d= np.array(data)
		y = np.array(labels)
		print(d.shape)
		return d, y

	def cross_val(self):
		d, y = self.get_data()
		d = self.scaler.transform(d)

		gamma = np.arange(0.01, 2, 1)
		c_vals = np.arange(2.5, 15, 5)
		high_g = 0
		high_c = 0
		high = 0
		splits = min(np.count_nonzero(y == 1), np.count_nonzero(y == 0))
		error = [[],[],[]]
		for c in c_vals:
			min_c = np.Inf 
			for g in gamma:
				clf = svm.SVC(kernel='rbf', C=c, gamma=g)
				score = cross_validate(clf, d, y, cv=splits)
				print("C: %f gamma: %f: %f" %(c, g, np.mean(score['test_score'])))
				if np.mean(score['test_score']) > high:
					high = np.mean(score['test_score'])
					high_g = g 
					high_c = c
				if min_c > np.mean(score['test_score']):
					min_c = np.mean(score['test_score'])
			error[0].append(c)
			error[1].append(min_c)
		print(error)
		print("high c %f, g %f a %f" %(high_c, high_g, high))
		return svm.SVC(kernel='rbf', C=high_c, gamma=high_g).fit(d, y), error

def main():
	# For testing various parameters
	svm = SVM(0.28, 2.5)
	_, error = svm.cross_val()
	plt.plot(error[0], error[1])
	plt.xlabel("Gamma Value")
	plt.ylabel("Accuracy")
	plt.show()

if __name__ == "__main__":
	main()