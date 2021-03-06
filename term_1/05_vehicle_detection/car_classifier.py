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

    def extract_hog_features_opencv(self, img, feature_vector=True):
        channels = []
        img_h, img_w, img_d = img.shape
        block_size = (self.hog_cells_per_block * self.hog_pixels_per_cell, self.hog_cells_per_block * self.hog_pixels_per_cell)
        block_stride = (self.hog_pixels_per_cell, self.hog_pixels_per_cell)
        cell_size = (self.hog_pixels_per_cell, self.hog_pixels_per_cell)
        n_cells = (img_h // self.hog_pixels_per_cell - 1, img_w // self.hog_pixels_per_cell - 1)

        hog = cv2.HOGDescriptor(block_size, block_size, block_stride, cell_size, self.hog_orientations)

        for channel in range(img_d):
            features = hog.compute(img)
            features = features.reshape(n_cells[0], n_cells[1], self.hog_cells_per_block, self.hog_cells_per_block, self.hog_orientations)

            if feature_vector:
                channels.append(np.ravel(features))
            else:
                channels.append(features)
        
        return channels
    
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
            features.append(cv2.calcHist([img], [channel], None, [self.num_bins], [0, 256]).ravel())

        return features

    def scaler_initialize(self, data):
        self.scaler = StandardScaler().fit(data)

    def scaler_apply(self, data):
        return self.scaler.transform(data)

    def classifier_predict(self, X):
        return self.svc.predict(X)

    def classifier_decision_function(self, X):
        return self.svc.decision_function(X)

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
