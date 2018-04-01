from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import os
import rospy
import cv2

def draw_rect(image, bbox, thickness = 4):
    rows = image.shape[0]
    cols = image.shape[1]
    x = bbox[1] * cols
    y = bbox[0] * rows
    right = bbox[3] * cols
    bottom = bbox[2] * rows
    return cv2.rectangle(image, (int(x), int(y)), (int(right), int(bottom)), (255, 0, 0), thickness=thickness)

def load_graph(frozen_graph_filename):
    with tf.gfile.GFile(frozen_graph_filename, "rb") as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())

    with tf.Graph().as_default() as graph:
        tf.import_graph_def(graph_def, name="")

    return graph


class TLClassifier(object):
    def __init__(self):

        self.label_dict = {
            1: TrafficLight.RED,
            2: TrafficLight.YELLOW,
            3: TrafficLight.GREEN,
            4: TrafficLight.UNKNOWN
        }

        self.prediction_threshold = 0.2
        self.frozen_graph_path = os.getcwd() + '/light_classification/models/output_inference/frozen_inference_graph.pb'
	rospy.loginfo("Load graph from: " + str(self.frozen_graph_path))
        self.graph = load_graph(self.frozen_graph_path)
        self.sess = tf.Session(graph=self.graph)

        self.num_detections = self.graph.get_tensor_by_name('num_detections:0')
        self.detection_scores = self.graph.get_tensor_by_name('detection_scores:0')
        self.detection_boxes = self.graph.get_tensor_by_name('detection_boxes:0')
        self.detection_classes = self.graph.get_tensor_by_name('detection_classes:0')
        self.image_tensor = self.graph.get_tensor_by_name('image_tensor:0')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        tf_image_input = np.expand_dims(image, axis=0)
        class_id = TrafficLight.UNKNOWN

        num, scores, classes = self.sess.run([self.num_detections,
						self.detection_scores,
						self.detection_classes],
                feed_dict={self.image_tensor: tf_image_input})

        if scores[0][0] > self.prediction_threshold:
            class_id = self.label_dict[int(classes[0][0])]

        return class_id
