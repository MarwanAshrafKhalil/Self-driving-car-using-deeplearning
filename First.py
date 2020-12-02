import cv2
print(cv2.__version__)
dispW=640
dispH=480
flip=0
minArea = 200
color = (255, 0, 255)
nPlateCascade = cv2.CascadeClassifier("/home/corki/Desktop/Codes/plate.xml")
#Uncomment These next Two Line for Pi Camera
camSet='nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, format=NV12, framerate=21/1 ! nvvidconv flip-method='+str(flip)+' ! video/x-raw, width='+str(dispW)+', height='+str(dispH)+', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
cam= cv2.VideoCapture(camSet)
 
#Or, if you have a WEB cam, uncomment the next line
#(If it does not work, try setting to '1' instead of '0')
#cam=cv2.VideoCapture(0)
while True:
    ret, img = cam.read()

    if not ret:
      break
    imgGray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    numberPlates = nPlateCascade.detectMultiScale(imgGray, 1.1, 10)
    for (x, y, w, h) in numberPlates:
      area = w * h
      if area > minArea:
        cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 255), 2)
        cv2.putText(img, "Number Plate", (x, y - 5),cv2.FONT_HERSHEY_COMPLEX_SMALL, 1, color, 2)
        imgRoi = img[y:y + h, x:x + w]
        cv2.imshow("ROI", imgRoi)

    cv2.imshow("Result", img)

    # cv2.imshow('nanoCam',frame)
    if cv2.waitKey(1)==ord('q'):
        break
cam.release()
cv2.destroyAllWindows()
