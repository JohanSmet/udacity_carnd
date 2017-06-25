import pandas as pd
import numpy as np

import argparse
import os.path
import os

from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle

import matplotlib.image as mpimg

from keras.layers import Flatten, Dense, Conv2D, Lambda, AveragePooling2D, Dropout, Cropping2D
from keras.models import Sequential
from keras.models import load_model
from keras import callbacks

STEERING_CORRECTION = 0.2
COL_CENTER = 0
COL_LEFT = 1
COL_RIGHT = 2
COL_STEERING = 3

class DataSet:
    def __init__(self, path, test_size=0.2):
        # get all subdirectories containing driving data
        data_dirs = [root for root, dirs, files in os.walk(path) if 'driving_log.csv' in files]
        
        # load data from all directories
        self.data = []
        
        for dir in data_dirs:
           self.load_data(dir)

        # split dataset into training and validation data
        self.data_train, self.data_validate = train_test_split(self.data, test_size=test_size)
        print('Training set size : {}'.format(len(self.data_train)))
        print('Validation set size : {}'.format(len(self.data_validate)))

        # create the generators
        self.generator_train = self.data_generator(self.data_train)
        self.generator_valid = self.data_generator(self.data_validate)

    def load_data(self, path):
        # load data from CSV
        csv = pd.read_csv(path + '/driving_log.csv', header=None)

        # save the data for each camera as individual samples
        for _, row in csv.iterrows():
            if row[COL_CENTER].endswith('.jpg'):
                self.data.append((self.fix_image_path(path, row[COL_CENTER]), float(row[COL_STEERING])))
                self.data.append((self.fix_image_path(path, row[COL_LEFT]), float(row[COL_STEERING]) + STEERING_CORRECTION))
                self.data.append((self.fix_image_path(path, row[COL_RIGHT]), float(row[COL_STEERING]) - STEERING_CORRECTION))

    def data_generator(self, samples, batch_size=64):
        n_samples  = len(samples)
        half_batch = batch_size // 2

        X_batch = np.zeros((batch_size, 160, 320, 3), dtype=np.float32)
        y_batch = np.zeros((batch_size,), dtype=np.float32)

        while True:
            shuffle(samples)
            
            # iterate only half a batch because we're returning two samples each time
            for idx in range(0, n_samples, half_batch):
                
                batch_idx = 0

                for sample in samples[idx:idx+half_batch]:
                    # lead image
                    image = mpimg.imread(sample[0])
                    
                    # normal image
                    X_batch[batch_idx] = image
                    y_batch[batch_idx] = float(sample[1])
                    batch_idx += 1

                    # mirrored image
                    X_batch[batch_idx] = np.fliplr(image)
                    y_batch[batch_idx] = -float(sample[1])
                    batch_idx += 1
                
                # the last batch might not be a full one
                yield (X_batch[0:batch_idx,:,:,:], y_batch[0:batch_idx])

            # end for

        # end while

    def fix_image_path(self, path, img):
        return path + '/IMG/' + os.path.basename(img)

    def num_train_samples(self):
        return len(self.data_train) * 2

    def num_validation_samples(self):
        return len(self.data_validate) * 2


class NvidiaModel:
    def __init__(self, load):
        # tensorboard callback
        self.tensorboard = callbacks.TensorBoard(log_dir='./logs', histogram_freq=10, write_graph=True, write_images=True)
        
        if load:
            # load previously trained model
            self.model = load_model('model.h5')
        else:
            # create new model
            self.model = Sequential()
            self.model.add(Cropping2D(cropping=((60,20),(0,0)), name='CropTopBottom', input_shape=(160,320,3)))
            self.model.add(AveragePooling2D((1,2), name='Scale'))
            self.model.add(Lambda(lambda x : (x / 255.0) - 0.5, name='Normalize'))
            self.model.add(Conv2D(3, 1, 1, border_mode='same', name='ColorConversion'))
            self.model.add(Conv2D(24, 5, 5, subsample=(2,2), activation='relu', name='Conv1'))
            self.model.add(Conv2D(36, 5, 5, subsample=(2,2), activation='relu', name='Conv2'))
            self.model.add(Conv2D(48, 5, 5, subsample=(2,2), activation='relu', name='Conv3'))
            self.model.add(Conv2D(64, 3, 3, activation='relu', name='Conv4'))
            self.model.add(Conv2D(64, 3, 3, activation='relu', name='Conv5'))
            self.model.add(Dropout(0.2, name='Dropout'))
            self.model.add(Flatten(name='Flatten'))
            self.model.add(Dense(100, name='FC1'))
            self.model.add(Dense(50, name='FC2'))
            self.model.add(Dense(10, name='FC3'))
            self.model.add(Dense(1, name='Output'))

            self.model.compile(loss='mse', optimizer='adam')

    def train(self, dataset, epochs):
        self.model.fit_generator(dataset.generator_train, samples_per_epoch=dataset.num_train_samples(),
                                 validation_data=dataset.generator_valid, nb_val_samples=dataset.num_validation_samples(),
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