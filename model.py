import pandas as pd
import numpy as np

from sklearn.model_selection import train_test_split
from sklearn.utils import shuffle

import cv2

from keras.layers import Flatten, Dense, Conv2D, Lambda, MaxPooling2D
from keras.models import Sequential
from keras import callbacks

STEERING_CORRECTION = 0.2

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
        csv = pd.read_csv(path + '/driving_log.csv')

        # save the data for each camera as individual samples
        for _, row in csv.iterrows():
            data.append((self.fix_image_path(path, row['center']), row['steering']))
            data.append((self.fix_image_path(path, row['left']), row['steering'] + STEERING_CORRECTION))
            data.append((self.fix_image_path(path, row['right']), row['steering'] - STEERING_CORRECTION))
    
        # split dataset into training and validation data
        self.data_train, self.data_validate = train_test_split(data, test_size=test_size)

    def fix_image_path(self, path, img):
        return path + '/IMG/' + img.split('/')[-1]


class NvidiaModel:
    def __init__(self):
        self.tensorboard = callbacks.TensorBoard(log_dir='./logs', histogram_freq=10, write_graph=True, write_images=True)

        self.model = Sequential()
        self.model.add(MaxPooling2D(pool_size=(2,2), input_shape=(160,320,3)))
        self.model.add(Lambda(lambda x : (x / 255.0) - 0.5))
        self.model.add(Conv2D(24, 5, 5, subsample=(2,2), activation='relu'))
        self.model.add(Conv2D(36, 5, 5, subsample=(2,2), activation='relu'))
        self.model.add(Conv2D(48, 5, 5, subsample=(2,2), activation='relu'))
        self.model.add(Conv2D(64, 3, 3, activation='relu'))
        self.model.add(Conv2D(64, 3, 3, activation='relu'))
        self.model.add(Flatten())
        self.model.add(Dense(100))
        self.model.add(Dense(50))
        self.model.add(Dense(10))
        self.model.add(Dense(1))

        self.model.compile(loss='mse', optimizer='adam')

    def train(self, dataset):
        self.model.fit_generator(dataset.generator_train, samples_per_epoch=len(dataset.data_train)*2, 
                                 validation_data=dataset.generator_valid, nb_val_samples=len(dataset.data_validate)*2, 
                                 callbacks=[self.tensorboard],
                                 nb_epoch=5)

    def save(self):
        self.model.save('model.h5')

if __name__ == "__main__":
    dataset = DataSet('data')
    model = NvidiaModel()
    model.train(dataset)
    model.save()
