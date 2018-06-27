from styx_msgs.msg import TrafficLight

import tensorflow as tf
import numpy as np
import rospy

#MODEL_NAME = 'mobilenet_trained_simulator'
#MODEL_NAME = 'rfcn_trained_simulator'
MODEL_NAME = 'inception_trained_simulator'

NETWORK_TO_STATE = [TrafficLight.RED, TrafficLight.YELLOW, TrafficLight.GREEN, TrafficLight.UNKNOWN]
NETWORK_TO_LABEL = ['Red', 'Yellow', 'Green', 'Unknown']

DEBUG = 1

class TLClassifier(object):
    def __init__(self):
        
        self.detection_graph = tf.Graph()

        # load saved model
        with self.detection_graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile('network/' + MODEL_NAME + '.pb', 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

            self.input_image = self.detection_graph.get_tensor_by_name('image_tensor:0')
            self.output_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
            self.output_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
            self.output_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')
            self.output_num_detections = self.detection_graph.get_tensor_by_name('num_detections:0')

        gpu_options = tf.GPUOptions(per_process_gpu_memory_fraction=0.5)
        self.tf_sess = tf.Session(graph=self.detection_graph, config=tf.ConfigProto(gpu_options=gpu_options, log_device_placement=True))



    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # run the image through the neural network
        ts_before = rospy.Time.now()
        _, scores, classes = self.run_inference(image, 0.60)

        # no traffic light detected is reported as "unknown"
        state = 3
        if len(classes) > 0:
           state = np.argmax([scores[classes == 1].sum(), scores[classes == 2].sum(), scores[classes == 3].sum()])

        ts_after = rospy.Time.now()

        if DEBUG == 1: 
            rospy.logwarn('Detected light = {} (duration = {})'.format(NETWORK_TO_LABEL[state], (ts_after - ts_before).to_sec() * 1000))

        return NETWORK_TO_STATE[state]


    def run_inference(self, img, min_score = 0.90):
        # bounding box detection.
        with self.detection_graph.as_default():
            # expand dimension since the model expects image to have shape [1, None, None, 3].
            img_expanded = np.expand_dims(img, axis=0)  
            (boxes, scores, classes, num) = self.tf_sess.run(
                [self.output_boxes, self.output_scores, self.output_classes, self.output_num_detections],
                feed_dict={self.input_image: img_expanded})
        
        # all outputs are float32 numpy arrays, so convert types as appropriate
        #  + filter on certainty
        num = int(num[0])
        mask = (scores[0] >= min_score)

        classes = classes[0].astype(np.uint8)[mask]
        boxes = boxes[0][mask]
        scores = scores[0][mask]
            
        return boxes, scores, classes
