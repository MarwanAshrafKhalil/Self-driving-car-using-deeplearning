import cv2
from PIL import Image
import tensorflow as tf
import numpy as np
import matplotlib.pyplot as plt
import warnings
import time

from models.research.object_detection.utils import label_map_util
from models.research.object_detection.utils import visualization_utils as viz_utils

print(cv2.__version__)
dispW=320
dispH=320
flip=0
minArea = 200
color = (255, 0, 255)
camSet= 'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
cam= cv2.VideoCapture(camSet)

PATH_TO_SAVED_MODEL = "/home/corki/Documents/TensorFlow/workspace/training_demo/exported-models/my_model/saved_model"
print('Loading model...', end='')

# Load saved model and build the detection function
detect_fn=tf.saved_model.load(PATH_TO_SAVED_MODEL)
print('Done!')
category_index=label_map_util.create_category_index_from_labelmap("/home/corki/Documents/TensorFlow/workspace/training_demo/annotations/label_map.pbtxt",use_display_name=True)
warnings.filterwarnings('ignore')   # Suppress Matplotlib warnings


while True:
    ret,img =cam.read()
    input_tensor=tf.convert_to_tensor(img)
    input_tensor=input_tensor[tf.newaxis, ...]
    detections=detect_fn(input_tensor)
    num_detections=int(detections.pop('num_detections'))
    detections={key:value[0,:num_detections].numpy()
                  for key,value in detections.items()}
    detections['num_detections']=num_detections
    detections['detection_classes']=detections['detection_classes'].astype(np.int64)
    image_np_with_detections=img.copy()

    #haarcascade area mesurement code
    #for (x, y, w, h) in image_np_with_detections:
     # area = w * h
     # if area > minArea:
        #cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
        #cv2.putText(img, "Number Plate", (x, y - 5),cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, color, 2)
        #imgRoi = img[y:y + h, x:x + w]
        #cv2.imshow("ROI", imgRoi)

    print("Boxes... ", detections['detection_boxes'])
    image_f, area= viz_utils.visualize_boxes_and_labels_on_image_array(
          image_np_with_detections,
          detections['detection_boxes'],
          detections['detection_classes'],
          detections['detection_scores'],
          category_index,
          use_normalized_coordinates=True,
          max_boxes_to_draw=4,     #max number of bounding boxes in the image
          min_score_thresh=.4,      #min prediction threshold
          agnostic_mode=False)

    print("AREA= ",area)
    cv2.imshow("result",image_f)
    
    if cv2.waitKey(1)==ord('q'):
        break

cam.release()
cv2.destroyAllWindows()
