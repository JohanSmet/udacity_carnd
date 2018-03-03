# **Behavioral Cloning Project**

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report

[image1]: ./examples/model.png "Model Visualization"
[image2]: ./examples/ex_input.png "Example of input image"
[image3]: ./examples/ex_input_flipped.png "Mirrored Input Image"
[image4]: ./examples/ex_cropped.png "Cropped Image"
[image5]: ./examples/ex_scaled.png "Scaled Image"
[image6]: ./examples/ex_color.png "Color Space Converted Image"

## Rubric Points
Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/432/view) individually and describe how I addressed each point in my implementation.  

### Files Submitted & Code Quality

#### 1. Submission includes all required files and can be used to run the simulator in autonomous mode

My project includes the following files:
* model.py containing the script to create and train the model
* drive.py for driving the car in autonomous mode
* model.h5 containing a trained convolution neural network
* writeup_report.md summarizing the results
* video.mp4 showing the car driving around the first track of the simulator
* video_track2.mp2 in which the car succesfully navigates the second track

#### 2. Submission includes functional code
Using the Udacity provided simulator and drive.py file, the car can be driven autonomously around the track by executing
```sh
python drive.py model.h5  
```
The drive.py file was not changed because all preprocessing of the camera images is done in the model itself.

#### 3. Submission code is usable and readable

The model.py file contains the code for training and saving the convolution neural network. The file contains two classes:
* DataSet contains all code pertaining to the handling of the data, loading of images and the generator to feed batches of data to the model.
* NvidiaModel builds and trains the model.

The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

### Model Architecture and Training Strategy

#### 1. An appropriate model architecture has been employed
I decided to base my model on the convolutional neural network as presented by Nvidia in the [End to End Learning for Self-Driving Cars](https://arxiv.org/pdf/1604.07316v1.pdf) paper based on the lecture and recommendations by other students. The paper does not specify which activations were used, I decided to use ReLU layers to introduce nonlinearity.

The data is cropped, scaled, normalized and converted to a different colorspace in the first layers of the model. This is followed by 5 convolutional layers, a dropout layer and finally 4 fully connected layers.

#### 2. Attempts to reduce overfitting in the model

A dropout layer was added to the model right after the convolutional layers to help reduce overfitting.
The model was trained on different data sets to ensure that the model was not overfitting. Care was taken to shuffle all the samples to prevent the model from discarding data from the earlier samples and overfitting on the later samples.
The model was tested by running it through the simulator and ensuring that the vehicle could stay on both tracks.

#### 3. Model parameter tuning

The model used an adam optimizer, so the learning rate was not tuned manually.

#### 4. Appropriate training data

Training data was chosen to keep the vehicle driving on the road. I started with the dataset kindly provided by Udacity and recorded driving on track 1 in reverse and two laps of driving on track 2. Later I added a few extra recording of problem areas where the model struggled.

### Model Architecture and Training Strategy

#### 1. Solution Design Approach

The overall strategy for deriving a model architecture was to start from a model that was proven to work for this type of problem and incrementally refine it to meet the project requirements. As mentioned earlier I started from the Nvidia model and added a dropout layer to combat overfitting and a lambda layer to normalize the input data.

In order to maximize the use of the input data I used the images from all three cameras by adding a correction to the steering angle of the left and right camera. In the python generator that feeds batches of images to the model during training I also mirrored each image and flipped the steering angle, effectivly doubling the number of available images. The dataset was split into 80% training data and 20% validation data. I planned to do more augmentations to the data later (e.g. change the brightness to simulate differente lightning conditions) but in the end it wasn't necessary.

I trained the model only on the data provided by Udacity. The mean squared on both the training and validation set was low. I tested the model in the simulator on the first track and the car drove until it came to the bridge and than crashed into the side of it.

Then I recorded more data. One set of driving around the track in the opposite direction but also a set of driving over the bridge multiple times. Training was slow so I added an average pooling layer to scale down the image horizontally. I added a cropping layer to remove the top of the images (above the road) and the bottom of the image (a part of the car was visible). I also learned that drive.py was feeding RGB data to the model but in model.py I was giving it BGR data. After fixing this and training for 10 epochs the model was able to navigate the first track correctly.

Then I took up the challenge of making the model complete the second track as well. The model in its current state failed to navigate the first corner. I recorded myself driving on the second track for about two laps and trained my existing model on that additional data. This marginally improved performance on the second track but made it worse on the first track.

Looking at the difference between the two tracks one thing that's immediately noticable is the color of the road markings. I decided to help the model detect the lines without relying on the color. I added a lambda layer to the model to change the images to grayscale. I preferred doing this in the model because that meant I didn't have to modify drive.py. This helped a bit on track 2 but made things worse on the first track. The model now failed in the corners with markings and drive straight onto the dirt.

I wanted to convert the color to YUV to seperate the luminance information but still preserve the color. It should be possible to do this in a lambda layer as wel because conversion between RGB and YUV is basically a matrix multiplication. Then I realised something: why not let the model learn the best color representation itself! So I added a 1x1 convolution layer with a depth of 3 to provide the model with an opportunity to change the color space of the images.

Around this time I also released I made a mistake earlier. I had added command line parameters to model.py so it could load the current model and train a certain amount of epoch on a directory also given as a command line parameter. This made training on additional data easy but also meant the model discarded previously learned knownledge and basically overfitted on the new data. This was noticable because performance on the first track deteriorated when training on data from the second track. I fixed this by loading all data at once and shuffling it properly.

After these two changes the model successfully navigated both tracks. This was challeging but fun assignment!

#### 2. Final Model Architecture

The final model architecture consisted of a convolution neural network with a few preprocessing layers (cropping, resizing, normalization, color space conversion), 5 convolutional layers (3 with a 5x5 kernel follow by 2 3x3 kernels), a dropout and flatten layer connected to 4 fully connected layers. The keras model summary:
```
____________________________________________________________________________________________________
Layer (type)                     Output Shape          Param #     Connected to                     
====================================================================================================
CropTopBottom (Cropping2D)       (None, 80, 320, 3)    0           cropping2d_input_1[0][0]         
____________________________________________________________________________________________________
Scale (AveragePooling2D)         (None, 80, 160, 3)    0           CropTopBottom[0][0]              
____________________________________________________________________________________________________
Normalize (Lambda)               (None, 80, 160, 3)    0           Scale[0][0]                      
____________________________________________________________________________________________________
ColorConversion (Convolution2D)  (None, 80, 160, 3)    12          Normalize[0][0]                  
____________________________________________________________________________________________________
Conv1 (Convolution2D)            (None, 38, 78, 24)    1824        ColorConversion[0][0]            
____________________________________________________________________________________________________
Conv2 (Convolution2D)            (None, 17, 37, 36)    21636       Conv1[0][0]                      
____________________________________________________________________________________________________
Conv3 (Convolution2D)            (None, 7, 17, 48)     43248       Conv2[0][0]                      
____________________________________________________________________________________________________
Conv4 (Convolution2D)            (None, 5, 15, 64)     27712       Conv3[0][0]                      
____________________________________________________________________________________________________
Conv5 (Convolution2D)            (None, 3, 13, 64)     36928       Conv4[0][0]                      
____________________________________________________________________________________________________
Dropout (Dropout)                (None, 3, 13, 64)     0           Conv5[0][0]                      
____________________________________________________________________________________________________
Flatten (Flatten)                (None, 2496)          0           Dropout[0][0]                    
____________________________________________________________________________________________________
FC1 (Dense)                      (None, 100)           249700      Flatten[0][0]                    
____________________________________________________________________________________________________
FC2 (Dense)                      (None, 50)            5050        FC1[0][0]                        
____________________________________________________________________________________________________
FC3 (Dense)                      (None, 10)            510         FC2[0][0]                        
____________________________________________________________________________________________________
Output (Dense)                   (None, 1)             11          FC3[0][0]                        
====================================================================================================
Total params: 386,631
Trainable params: 386,631
Non-trainable params: 0
____________________________________________________________________________________________________
```
Here is a visualization of the architecture:

![alt text][image1]

#### 3. Creation of the Training Set & Training Process

I started from the data provided by Udacity and recorded a lap on track one driving in the opposite direction. I also recorded some extra footage on parts on the track the model seemed to struggle with. This is an example of a left turn without a seperator between the road and the dirt on the side:

![alt text][image2]

I also recorded two laps of driving on track two in order to get more datapoints.

To augment the data set, I also flipped images and angles to double the amount of images. I had planned to adjust the brightness of the images and try to level the number of "driving straight" data points against the "taking a corner" data points after I got the basic model working. But the performance on both tracks was satisfactory without these extra features so I decided not to pursue them. Here is an example of a horizontally flipped image:

![alt text][image3]

After the collection process, I had 9353 data points with 3 camera standpoints each. I added a correction factor to the left and right camera image and used all of them. In the end this gave me 28059 images which were doubled in the generator.

I finally randomly shuffled the data set and put 20% of the data into a validation set.

The model starts with a few preprocessing layers. This is an example image after the cropping layer:
![alt text][image4]

To help speed up training more the next step is to reduce the horizontal size of the image by a factor of two:
![alt text][image5]

Instead of a fixed conversion to grayscale or YUV-color I decided to let the model learn the best color representation itself. Visualizing the result of this operation isn't straightforward, so here is a grayscale image of each resulting color channel:
![alt text][image6]

I used this training data for training the model. The validation set helped determine if the model was over or under fitting. After 10 epochs the mean squared error didn't change significantly any more and the model performed decently on both tracks. I used an adam optimizer so that manually training the learning rate wasn't necessary.
