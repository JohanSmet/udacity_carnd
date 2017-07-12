import numpy as np
import cv2

from sklearn.svm import LinearSVC
from sklearn.preprocessing import StandardScaler
from skimage.feature import hog
from sklearn.externals import joblib

class CarClassifier:
    def __init__(self, hog_orientations = 9, hog_pixels_per_cell = 16, hog_cells_per_block = 2, spatial_size=16, num_bins=16):
        self.hog_orientations = hog_orientations
        self.hog_pixels_per_cell = hog_pixels_per_cell
        self.hog_cells_per_block = hog_cells_per_block
        self.spatial_size = spatial_size
        self.num_bins = num_bins

        self.svc    = None     
        self.scaler = None

    def prepare_img (self, img):
        # prepare the image to be classified (e.g color space conversion, image transformations)
        return cv2.cvtColor(img, cv2.COLOR_BGR2YUV)
    
    def extract_hog_features(self, img, feature_vector=True):
        features = []

        for channel in range(img.shape[2]):
            features.append(hog(img[:,:,channel], 
                                orientations=self.hog_orientations, 
                                pixels_per_cell=(self.hog_pixels_per_cell, self.hog_pixels_per_cell),
                                cells_per_block=(self.hog_cells_per_block, self.hog_cells_per_block), 
                                block_norm='L1-sqrt',
                                transform_sqrt=True, visualise=False, feature_vector=feature_vector))
        
        return features

    def extract_color_features(self, img):
        features = []

        # spatial binning
        features.append(cv2.resize(img, (self.spatial_size, self.spatial_size)).ravel())

        # color histogram
        for channel in range(img.shape[2]):
            features.append(np.histogram(img[:,:,channel], bins=self.num_bins, range=(0,256))[0])

        return features

    def scaler_initialize(self, data):
        self.scaler = StandardScaler().fit(data)

    def scaler_apply(self, data):
        return self.scaler.transform(data)

    def classifier_train(self, X, y):
        self.svc = LinearSVC()
        self.svc.fit(X, y)
    
    def classifier_accuracy(self, X, y):
        return self.svc.score(X, y)

    def classifier_predict(self, X):
        return self.svc.predict(X)

    def save(self, filename):
        dump = {
            'hog_orientations': self.hog_orientations,
            'hog_pixels_per_cell' : self.hog_pixels_per_cell,
            'hog_cells_per_block' : self.hog_cells_per_block,
            'spatial_size' : self.spatial_size,
            'num_bins' : self.num_bins,
            'svc' : self.svc,
            'scaler' : self.scaler
        }

        joblib.dump(dump, filename)

    @staticmethod
    def restore(filename):
        dump = joblib.load(filename)

        clf = CarClassifier(dump['hog_orientations'], dump['hog_pixels_per_cell'], dump['hog_cells_per_block'], 
                            dump['spatial_size'], dump['num_bins'])
        clf.svc = dump['svc']
        clf.scaler = dump['scaler']

        return clf
