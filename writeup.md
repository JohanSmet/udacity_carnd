# **Traffic Sign Recognition**

**Build a Traffic Sign Recognition Project**

The goals / steps of this project are the following:
* Load the data set (see below for links to the project data set)
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./writeup/vis_samples.png "Data set sample images"
[image2]: ./writeup/vis_histogram.png "Distribution of classes"
[image3]: ./writeup/preproc.png "Image Preprocessing"
[image4]: ./writeup/generated.png "Generated Image"
[image5]: ./extra_data/04_speed_limit_70.jpg "Speed Limit 70"
[image6]: ./extra_data/12_priority_road.jpg "Priority Road"
[image7]: ./extra_data/17_no_entry.jpg "No Entry"
[image8]: ./extra_data/22_bumpy_road.jpg "Bumpy Road"
[image9]: ./extra_data/25_road_work.jpg "Road Work"
[image10]: ./writeup/softmax_1.png "Softmax Speed Limit 70"
[image11]: ./writeup/softmax_2.png "Softmax Priority Road"
[image12]: ./writeup/softmax_3.png "Softmax No Entry"
[image13]: ./writeup/softmax_4.png "Softmax Bumpy Road"
[image14]: ./writeup/softmax_5.png "Softmax Road Work"
[image15]: ./writeup/precision_recall.png "Precision and recall"
[image16]: ./writeup/confusion_matrix.png "Confusion Matrix"

## Rubric Points
Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/481/view) individually and describe how I addressed each point in my implementation.  

### Writeup / README

###### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf. You can use this template as a guide for writing the report. The submission includes the project code.

You're reading it! and here is a link to my [project code](https://github.com/JohanSmet/CarND-Traffic-Sign-Classifier-Project/blob/master/Traffic_Sign_Classifier.ipynb) and the [html export](https://cdn.rawgit.com/JohanSmet/CarND-Traffic-Sign-Classifier-Project/d9ce8039/report.html) of the Jupyter Notebook.

### Data Set Summary & Exploration

###### 1. Provide a basic summary of the data set. In the code, the analysis should be done using python, numpy and/or pandas methods rather than hardcoding results manually.

I used Numpy and basic Python functions to calculate summary statistics of the traffic signs data set:

* The size of training set is 34799
* The size of the validation set is 4410
* The size of test set is 12630
* The shape of a traffic sign image is 32x32x3
* The number of unique classes/labels in the data set is 43

###### 2. Include an exploratory visualization of the dataset.

I made two exploratory visualizations of the data set. The first shows some random examples taken from the training set, just to see what we're working with.
![alt text][image1]

Additionaly I added a normalized histogram to show the distribution of the classes/labels in the training, validation and testing data set.
![alt text][image2]
This clearly shows that not all classes are represented equaly in the data set. Some traffic signs have a lot more samples than others. Luckily the classes seem to be distributed nicely between the three sets.

### Design and Test a Model Architecture

###### 1. Describe how you preprocessed the image data. What techniques were chosen and why did you choose these techniques? Consider including images showing the output of each preprocessing technique. Pre-processing refers to techniques such as converting to grayscale, normalization, etc. (OPTIONAL: As described in the "Stand Out Suggestions" part of the rubric, if you generated additional data for training, describe why you decided to generate additional data, how you generated the data, and provide example images of the additional data. Then describe the characteristics of the augmented training set like number of images in the set, number of images for each class, etc.)

As a first step, I decided to convert the images to grayscale because the paper linked in the assignment (Traffic Sign Recognition with Multi-Scale Convolutional Networks, Pierre Sermanet and Yann LeCun) reported positive results from doing this.

As a second step I applied a filter to the image to normalize the contrast of the sample image. This is an effort to reduce the effect of differing lighting conditions on the results of the classifier. I used the Contrast Limited Adaptive Histogram Equalization (CLAHE) functionality provided by OpenCV.

Here is an example of a traffic sign image before, after grayscaling and after the contrast normalization:
![alt text][image3]

As a last step, I normalized the image data because it's always a good idea to use a dataset with mean zero and equal variance.

I decided to generate additional data because not all traffic signs are represented in an equal amount in the test set. I generated additional samples for each class but classes with fewer samples in the original dataset received more new images than others.

To add more data to the the data set, I used the following techniques because I wanted to simulate photos taken from different angles/positions:
1. a random translation (between [-2,2] pixels)
2. a random rotation (between [-15,+15] degrees)
3. a random scale (between [.9,1.1] ratio)
4. crop or pad the result back to 32x32 pixels

Here is an example of an original image and an altered image (original, altered, preprocessed):
![alt text][image4]

###### 2. Describe what your final model architecture looks like including model type, layers, layer sizes, connectivity, etc.) Consider including a diagram and/or table describing the final model.

My final model consisted of the following layers:

| Layer         		    |     Description	        					| Activation |
|:---------------------:|:---------------------------------------------:|:-------:|
| Input         		    | 32x32x1 grayscale image   							      |-|
| Convolution 5x5       | 1x1 stride, valid padding, outputs 28x28x6    |ReLU|
| Convolution 5x5       | 1x1 stride, same padding, outputs 28x28x6     |ReLU|
| Max pooling           | 2x2 stride, outputs 14x14x6   |-|
| Convolution 5x5       | 1x1 stride, valid padding, outputs 10x10x16    |ReLU|
| Convolution 5x5       | 1x1 stride, same padding, outputs 10x10x16     |ReLU|
| Max pooling           | 2x2 stride, outputs 5x5x16   |-|
| Dropout               | |-|
| Fully Connected       | outputs 120 nodes | ReLU |
| Fully Connected       | outputs 84 nodes | ReLU |
| Fully Connected       | outputs 43 logits | None |
| Softmax               | outputs 43 probabilities | - |


###### 3. Describe how you trained your model. The discussion can include the type of optimizer, the batch size, number of epochs and any hyperparameters such as learning rate.

To train the model, I used an Adam optimizer to minimize the softmax cross entropy. I settled on a batch size of 256 images, a learning rate of 0.001 and a keep probability of 0.75 for the dropout layer. I let the network train for 25 epochs.

###### 4. Describe the approach taken for finding a solution and getting the validation set accuracy to be at least 0.93. Include in the discussion the results on the training, validation and test sets and where in the code these were calculated. Your approach may have been an iterative process, in which case, outline the steps you took to get to the final solution and why you chose those steps. Perhaps your solution involved an already well known implementation or architecture. In this case, discuss why you think the architecture is suitable for the current problem.

My final model results were:
* training set accuracy of 0.999
* validation set accuracy of 0.984
* test set accuracy of 0.963

I started with a model based closely on the LeNet architecture as was discussed in the lecture. I tweaked the network and the preprocessing on the data set to arrive at the current solution.

* first I added a dropout layer to reduce the possibility of overfitting
* enhancing the preprocessing (grayscale, contrast normalization) increased the score
* I added extra convolution layers between the original convolution and the max pooling layer. This was inspired by the architecture of the VGG16 network.
* As a last step I extended the data set with extra images as described earlier.

I did not tweak the other hyperparameters much from the values I started out with because they gave reasonable results.

### Test a Model on New Images

###### 1. Choose five German traffic signs found on the web and provide them in the report. For each image, discuss what quality or qualities might be difficult to classify.

Here are five German traffic signs that I found on the web:

![alt text][image5] ![alt text][image6] ![alt text][image7] ![alt text][image8] ![alt text][image9]

1. The first image might be difficult to classify because there are a number of similar speed limit traffic signs
2. The background on the second image is similar to the edge of the traffic sign which could be problematic
3. The third image should not be difficult to predicting
4. The fourth image is a bit degraged (black spots)
5. The fifth image contains a humanoid figure which can easily be mistaken for other pictograms

###### 2. Discuss the model's predictions on these new traffic signs and compare the results to predicting on the test set. At a minimum, discuss what the predictions were, the accuracy on these new predictions, and compare the accuracy to the accuracy on the test set (OPTIONAL: Discuss the results in more detail as described in the "Stand Out Suggestions" part of the rubric).

Here are the results of the prediction:

| Image			        |     Prediction	        					|  |
|:---------------------:|:---------------------------------------------:|:---:|
| Speed limit 70 km/h | Speed limit 70 km/h | Correct |
| Priority Road | Priority Road | Correct |
| No Eentry | No Eentry | Correct |
| Bumpy Road | Bumpy Road | Correct |
| Road Work | Road Work | Correct |

The model was able to correctly guess the 5 traffic signs, which gives an accuracy of 100%.  The accuracy was 96.3% on the testing set thus the model does not seem to be overfitting. Although with only five samples we could just have been lucky. A more exhaustive test would be required to make a more definitive statement.

###### 3. Describe how certain the model is when predicting on each of the five new images by looking at the softmax probabilities for each prediction. Provide the top 5 softmax probabilities for each image along with the sign type of each probability. (OPTIONAL: as described in the "Stand Out Suggestions" part of the rubric, visualizations can also be provided such as bar charts)

The model is very sure of its predictions for each test image with a probability of over 0.9 for each correct guess.
These are the top five softmax probabilities for each image plotted on a bar chart. An bar chart with a logarithmic scale is also included.

![alt text][image10]
![alt text][image11]
![alt text][image12]
![alt text][image13]
![alt text][image14]

##### (Optional) 4. Analyze Image Performance in more detail
I graphed the precision and recall per class for the images in the test set. The graphs shows consistently good precision and recall for each class. The only noticable dips are for image with fewer samples in the training set. It would be good to include more real images for these categories because the additionaly generated images don't seem to be sufficient to close the gap.
![alt text][image15]

To show the mistakes I also computed and graphed the confusion matrix of the test set. As excepted the confusion matrix has the largest numbers on the main diagonal (correct predictions). We can also see that most wrong predictions are of similar traffics signs. There is noticable confusion between the different speed limit signs but also between signs that contain humanoid figures (pedestrians/children crossing) or other similar signs (e.g. caution and double curve).
![alt text][image16]
