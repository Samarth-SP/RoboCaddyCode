import numpy as np
import tflite_runtime.interpreter as tflite
from picamera2 import *
import cv2
#constants declaration
normalSize = (640, 480)
lowresSize = (320, 240)

rectangles = []

def ReadLabelFile(file_path):
    with open(file_path, 'r') as f:
        lines = f.readlines()
    ret = {}
    for line in lines:
        pair = line.strip().split(maxsplit=1)
        ret[int(pair[0])] = pair[1].strip()
    return ret

def DrawRectangles(request):
   stream = request.picam2.stream_map["main"]
   fb = request.request.buffers[stream]
   with MappedArray(request, "main") as b:
       im = np.array(b.array, copy=False, dtype=np.uint8).reshape((normalSize[1],normalSize[0], 4))
       for rect in rectangles:
          print(rect)
          rect_start = (int(rect[0]*2) - 5, int(rect[1]*2) - 5)
          rect_end = (int(rect[2]*2) + 5, int(rect[3]*2) + 5)
          cv2.rectangle(im, rect_start, rect_end, (0,255,0,0))
          if len(rect) == 5:
            text = rect[4]
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(im, text, (int(rect[0]*2) + 10, int(rect[1]*2) + 10), font, 1, (255,255,255),2,cv2.LINE_AA)
       del im
    
def InferenceTensorFlow(image, model, output, label=None):
    global rectangles
    if label:
        labels = ReadLabelFile(label)
    else:
        labels = None
    interpreter = tflite.Interpreter(model_path=model, num_threads=4)
    interpreter.allocate_tensors()

    input_details = interpreter.get_input_details()
    output_details = interpreter.get_output_details()
    height = input_details[0]['shape'][1]
    width = input_details[0]['shape'][2]
    floating_model = False
    if input_details[0]['dtype'] == np.float32:
        floating_model = True

    rgb = cv2.cvtColor(image,cv2.COLOR_GRAY2RGB)
    initial_h, initial_w, channels = rgb.shape

    picture = cv2.resize(rgb, (width, height))

    input_data = np.expand_dims(picture, axis=0)
    if floating_model:
        input_data = (np.float32(input_data) - 127.5) / 127.5
    print(floating_model)
    interpreter.set_tensor(input_details[0]['index'], input_data)

    interpreter.invoke()

    detected_boxes = interpreter.get_tensor(output_details[0]['index'])
    detected_classes = interpreter.get_tensor(output_details[1]['index'])
    detected_scores = interpreter.get_tensor(output_details[2]['index'])
    num_boxes = interpreter.get_tensor(output_details[3]['index'])
    print(detected_classes)
    rectangles = []
    for i in range(int(num_boxes)):
        top, left, bottom, right = detected_boxes[0][i]
        classId = int(detected_classes[0][i])
        if classId == 36:
            score = detected_scores[0][i]
        else:
            score = 0
        if score > 0.1:
            xmin = left * initial_w
            ymin = bottom * initial_h
            xmax = right * initial_w
            ymax = top * initial_h
            box = [xmin, ymin, xmax, ymax]
            rectangles.append(box)
            if labels:
                print(labels[classId], 'score = ', score)
                rectangles[-1].append(labels[classId])
            else:
                print ('score = ', score)

camera = Picamera2()
config = camera.create_preview_configuration(main={"size": normalSize}, lores={"size": lowresSize, "format": "YUV420"})
camera.configure(config)
camera.start_preview(Preview.QTGL)
stride = camera.stream_configuration("lores")["stride"]
camera.post_callback = DrawRectangles
camera.start()

while (True):
     buffer = camera.capture_buffer("lores")
     grey = buffer[:stride*lowresSize[1]].reshape((lowresSize[1], stride))
     result = InferenceTensorFlow(grey,'mobilenet_v2.tflite', 'hello.jpg', 'coco_labels.txt')
