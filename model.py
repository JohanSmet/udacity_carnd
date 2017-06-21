import pandas as pd
import numpy as np

import argparse
import os.path

from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle

import cv2

from keras.layers import Flatten, Dense, Conv2D, Lambda, AveragePooling2D, Dropout, Cropping2D
from keras.models import Sequential
from keras.models import load_model
from keras import callbacks

STEERING_CORRECTION = 0.2
COL_CENTER = 0
COL_LEFT = 1
COL_RIGHT = 2
COL_STEERING = 3

def mirror_image(img):
    return np.fliplr(img)

def generator(samples, batch_size=64):
    n_samples = len(samples)

    X_batch = np.zeros((batch_size, 160, 320, 3), dtype=np.float32)
    y_batch = np.zeros((batch_size,), dtype=np.float32)

    while True:
        shuffle(samples)

        for idx in range(0, n_samples, batch_size//2):

            for batch_idx, sample in enumerate(samples[idx:idx+batch_size//2]):
                X_batch[batch_idx] = cv2.imread(sample[0])
                y_batch[batch_idx] = float(sample[1])
                X_batch[batch_idx*2+1] = mirror_image(cv2.imread(sample[0]))
                y_batch[batch_idx*2+1] = -float(sample[1])
            
            yield (X_batch, y_batch)

        # end for

    # end while

class DataSet:
    def __init__(self, path):
        self.load_data(path)
        
        print('Training set size : {}'.format(len(self.data_train)))
        print('Validation set size : {}'.format(len(self.data_validate)))

        self.generator_train = generator(self.data_train)
        self.generator_valid = generator(self.data_validate)

    def load_data(self, path, test_size=0.2):
        # load data from CSV
        data = []
        csv = pd.read_csv(path + '/driving_log.csv', header=None)

        # save the data for each camera as individual samples
        for _, row in csv.iterrows():
            if row[COL_CENTER].endswith('.jpg'):
                data.append((self.fix_image_path(path, row[COL_CENTER]), float(row[COL_STEERING])))
                data.append((self.fix_image_path(path, row[COL_LEFT]), float(row[COL_STEERING]) + STEERING_CORRECTION))
                data.append((self.fix_image_path(path, row[COL_RIGHT]), float(row[COL_STEERING]) - STEERING_CORRECTION))
    
        # split dataset into training and validation data
        self.data_train, self.data_validate = train_test_split(data, test_size=test_size)

    def fix_image_path(self, path, img):
        return path + '/IMG/' + os.path.basename(img)


class NvidiaModel:
    def __init__(self, load):
        self.tensorboard = callbacks.TensorBoard(log_dir='./logs', histogram_freq=10, write_graph=True, write_images=True)
        
        if load:
            self.model = load_model('model.h5')
        else:
            self.model = Sequential()
            self.model.add(Cropping2D(cropping=((60,20),(0,0)), input_shape=(160,320,3)))
            self.model.add(AveragePooling2D((1,2)))
            self.model.add(Lambda(lambda x : (x / 255.0) - 0.5))
            self.model.add(Conv2D(24, 5, 5, subsample=(2,2), activation='relu'))
            self.model.add(Conv2D(36, 5, 5, subsample=(2,2), activation='relu'))
            self.model.add(Conv2D(48, 5, 5, subsample=(2,2), activation='relu'))
            self.model.add(Conv2D(64, 3, 3, activation='relu'))
            self.model.add(Conv2D(64, 3, 3, activation='relu'))
            self.model.add(Dropout(0.2))
            self.model.add(Flatten())
            self.model.add(Dense(100))
            self.model.add(Dense(50))
            self.model.add(Dense(10))
            self.model.add(Dense(1))

            self.model.compile(loss='mse', optimizer='adam')

    def train(self, dataset, epochs):
        self.model.fit_generator(dataset.generator_train, samples_per_epoch=len(dataset.data_train)*2, 
                                 validation_data=dataset.generator_valid, nb_val_samples=len(dataset.data_validate)*2, 
                                 callbacks=[self.tensorboard],
                                 nb_epoch=epochs)

    def save(self):
        self.model.save('model.h5')

if __name__ == "__main__":
    # handle command line arguments
    parser = argparse.ArgumentParser(description='Driving Model Trainer')
    parser.add_argument("-l", "--load", help="Load model.h5 and continue training", action="store_true")
    parser.add_argument("-e", "--epochs", help="Set number of epochs (defaults=3)", type=int)
    parser.add_argument("datadir", help="Path to training data")
    args = parser.parse_args()

    dataset = DataSet(args.datadir)
    model = NvidiaModel(args.load)
    model.train(dataset, args.epochs if args.epochs != None else 3)
    model.save()
