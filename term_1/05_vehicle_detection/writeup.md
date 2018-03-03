**Vehicle Detection Project**

The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Optionally, you can also apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector. 
* Note: for those first two steps don't forget to normalize your features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream (start with the test_video.mp4 and later implement on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

[//]: # (Image References)
[image1]: ./examples/output_detections.png
[image2]: ./examples/output_heatmap.png
[image3]: ./examples/output_labels.png
[image4]: ./examples/output_vehicles.png

## [Rubric](https://review.udacity.com/#!/rubrics/513/view) Points
Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.
You're reading it!

### Histogram of Oriented Gradients (HOG)

#### 1. Explain how (and identify where in your code) you extracted HOG features from the training images.

The code for extracting HOG features can be found in `car_classifier.py` in the function `CarClassifier.extract_hog_features()`. The code for enumerating and loading the images is contained in `train.py`.

I started by reading in all the `vehicle` and `non-vehicle` images. These are converted to another colorspace (YUV) and passed to the `skimage.hog()` function. I experimented with different colorspaces, different parameters to the `skimage.hog()` function and the HOG implementation of OpenCV because it was reported to be faster.

#### 2. Explain how you settled on your final choice of HOG parameters.

I tried various combinations of parameters trying to maximize the detections and minimize the reported false positives while keeping an eye on the performance of the implementation. I started with settings that mirrored the ones used in the lecture videos and went through a gruelling iterative process of finding the best parameters and workflow.

I settled on the YUV-colorspace, 9 hog-orientations, 16 pixels per cell and 2 cells per block.

I also experimented with the OpenCV HOG implementation. Testing showed this was about 10-20 times faster than the skimage implementation but also less accurate. It produced many more false negatives and this was the reason I swiched back to the skimage implementation.

#### 3. Describe how (and identify where in your code) you trained a classifier using your selected HOG features (and color features if you used them).

I trained a linear SVM using the `TrainClassifier` class in `train.py`. I start by loading and extracting the HOG features and color features (spatial binning and color histogram) and used a StandardScaler to normalize the data. The next step is to randomly split the data into training and validation sets.

I used a grid search (`GridSearchCV`) to find the best value for the 'C' value of the LinearSVC. The SVC is then fit to the training set and the accuracy is computed on the validation set.

The final step is to save the trained classifier for later use in the video processing routine.

### Sliding Window Search

#### 1. Describe how (and identify where in your code) you implemented a sliding window search.  How did you decide what scales to search and how much to overlap windows?

I looked at the example images and the project video and noted where in the images cars appear and the sizes of the cars in various positions. I rounded the sizes to the nearest multiple of 64 and tried to space them evenly over the image.

I made a small function that writes the consecutive windows to a video to finetune the placement. It was an iterative process to find the optimal overlap and window size to search. I tried to maximize the results without sacrificing to much performance.

#### 2. Show some examples of test images to demonstrate how your pipeline is working.  What did you do to optimize the performance of your classifier?

Ultimately I searched with three window sizes (64, 96, 128)  using YUV 3-channel HOG features plus spatially binned color and histograms of color in the feature vector, which provided a nice result.  Here are some example images:

![alt text][image1]

Limiting the area of interest to where the different sized windows are most likely to detect cars was an easy optimization.

I tried using the sub-sampling approach covered in the lecture to optimize the performance of the algorithm. I found that it had a detrimental effect on the quality of the detections. It resulted in less positive detection but more false positives. The HOG-algorithm takes adjacent blocks into account and this means that the HOG-features of a car on the right of the image from sub-sampling differ substantially from the HOG-features that are extracted from the sub-image.

---

### Video Implementation

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)
Here's a [link to my video result](https://cdn.rawgit.com/JohanSmet/CarND-Vehicle-Detection/ad1f0d2d/output_images/project_video_processed.mp4). It's also available on YouTube:

[![Project Video](https://img.youtube.com/vi/MvKrEurBpF0/0.jpg)](https://www.youtube.com/watch?v=MvKrEurBpF0)

####2. Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.

I recorded the positions of positive detections in each frame of the video. From the positive detections I created a heatmap. To stabilize the detected regions and further reduce false positives I accumulated the heatmaps of the last 5 frames of the video. The resulting heatmap was thresholded to identify vehicle positions.

 I then used `scipy.ndimage.measurements.label()` to identify individual blobs in the heatmap.  I then assumed each blob corresponded to a vehicle.  I constructed bounding boxes to cover the area of each blob detected.  

Here's an example result showing the heatmap from the test images:
![alt text][image2]

Here is the output of `scipy.ndimage.measurements.label()` on the heatmap of that last image:
![alt text][image3]

Here the resulting bounding boxes are drawn onto the last image:
![alt text][image4]

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

The biggest problem I faced was finding the right balance between performance and accuracy. In the end I decided to focus mostly on accuracy at the cost of perfomance. 

I went with a linear SVM using the skimage HOG implementation to extract features. This gives adequate results but gets confused sometimes by cars on the other side of the road. Unfortunately it runs rather slowly, processing about on frame per second on my i5-4570.

I tried optimizing perfomance with the OpenCV HOG implementation and sub-sampling the HOG-features but could not get decent accuracy with these approaches.

All this led me to switch to a deep learning approach and I briefly implemented a convolutional neural network based classifier until I reread the rubric and saw that it focused heavily on the histogram of gradients. The CNN was simple (3 Conv2D layers with MaxPooling and dropout and a few fully connected layers) but it performed better and faster at about 3 fps on my GTX 970. I also uploaded a video of this implementation to [YouTube](https://www.youtube.com/watch?v=KVeJqd9-O4A)

If I were going to pursue this project further I'd investigate this further and look into a single shot detector network.

