from styx_msgs.msg import TrafficLight
import numpy as np
import tensorflow as tf
from PIL import Image
import time
import rospy
import os

TL_THREHOLD=0.5
MODELS_FOLDER = '/tl_classifier_model/'
MODELS_FILENAME = "tl_cl_model.pb"
NUM_CLASSES=4

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        curr_dir = os.path.dirname(os.path.realpath(__file__))
	model_path = curr_dir + MODELS_FOLDER + MODELS_FILENAME
	
	item_green = {'id':1,'name':'Green'}
	item_red = {'id':2,'name':'Red'}
	item_yellow = {'id':3,'name':'Yellow'}
	
	self.label_dict = {1:item_green,2:item_red,3:item_yellow}
	self.build_model_graph(model_path)
	print("Classifier is ready")
    def detect_traffic_light(self,scores,biggest_score_idx,classes,detected_light):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
	if scores[biggest_score_idx]>TL_THREHOLD:
		rospy.logwarn("Current traffic light is:{}".format(self.label_dict[classes[biggest_score_idx]]['name']))
		if classes[biggest_score_idx]==1:
			detected_light = TrafficLight.GREEN
		elif classes[biggest_score_idx]==2:
			detected_light = TrafficLight.RED
		elif classes[biggest_score_idx]==3:
			detected_light = TrafficLight.YELLOW
	else:
		rospy.logwarn("not defined")
	return detected_light
        #TODO implement light color prediction
    def build_model_graph(self,model_path):
	self.model_graph=tf.Graph()
	with self.model_graph.as_default():
		#new Graph import
		od_graph_def=tf.GraphDef()
		with tf.gfile.GFile(model_path,'rb') as fid:
			# frozen Graph
			saved_graph = fid.read()
			od_graph_def.ParseFromString(saved_graph)
			tf.import_graph_def(od_graph_def,name='')
		self.sess = tf.Session(graph=self.model_graph)
	self.image_tensor = self.model_graph.get_tensor_by_name('image_tensor:0')
	self.scores = self.model_graph.get_tensor_by_name('detection_scores:0')
	self.classes = self.model_graph.get_tensor_by_name('detection_classes:0')
	self.num_detections = self.model_graph.get_tensor_by_name('num_detections:0')
	self.boxes = self.model_graph.get_tensor_by_name('detection_boxes:0') 
    def classify(self,image):
	detected_light = TrafficLight.UNKNOWN
	image_expanded = np.expand_dims(image,axis=0)
	with self.model_graph.as_default():
		(boxes,scores,classes,num) = self.sess.run([self.boxes,self.scores,self.classes,self.num_detections],feed_dict={self.image_tensor:image_expanded})
	scores = np.squeeze(scores)
	return self.detect_traffic_light(scores,scores.argmax(),np.squeeze(classes).astype(np.int32),detected_light)
