from cv2 import imshow
import numpy as np
import cv2
import glob
#Load Image 'objects01.jpg OR 'objects02.jpg (either one of them works)
img = cv2.imread("Traffic_Signs.png")
#Convert the image to Gray Scale
img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)

#Crop the image in 9 objects (they are ordered in a 3x3 form in the original image) and add them to a list using two nested for loops
w = img_gray.shape[0]
h = img_gray.shape[1]
size = [int(w/2),int(h/3)]
cropped_img_t = [cv2.imread(file) for file in glob.glob("Traffic_Signs0*.png")]
cropped_img = []
names = ['stop','continue','round','right','no limit']
idx = 0
for im in cropped_img_t:
    cropped_img.append(cv2.cvtColor(cv2.pyrDown(cv2.pyrDown(im)),cv2.COLOR_BGR2GRAY))
    #cv2.imshow(names[idx],cropped_img[-1])
    idx += 1
    #cv2.waitKey(0)
#Create a list with the different names of each image

#Create an Orb detector with 1000 key-points and a scaling pyramid factor of 1.2
orb = cv2.SIFT_create() #cv2.ORB_create(1000, 1.2)
#Create a matcher with Hamming norm and crossCheck equal true
#bf = cv2.BFMatcher_create(cv2.NORM_HAMMING, crossCheck=True)
bf = cv2.BFMatcher()
#matcher = cv2.DescriptorMatcher_create(cv2.DescriptorMatcher_BRUTEFORCE_HAMMING)
#For all images detect all the key points and descriptors using an orb and append the descriptors to a list
descriptors = []
keypoints = []
for im in cropped_img:
    kp, des = orb.detectAndCompute(im, None)
    descriptors.append(des)
    keypoints.append(kp)
#Open an infinite video stream
cap= cv2.VideoCapture(0)
#Use an infinite loop that would stop with an enter key
last = 0
timer_cnt = 0
while True:
    #Capture frame and convert to Gray Scale
    ret, frame= cap.read() 
    frame_gray = cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    if cv2.waitKey(1) & 0xFF == 27 or not ret: #End conditions
        break
    kp, des = orb.detectAndCompute(frame_gray, None)
    #Compare the detectors from the captured frame with the detectors from the image
    matches = []
    num_mat = []
    
    for desn in descriptors:
        mat = bf.knnMatch(desn, des,k=2)
        good_matches = []

        for m1, m2 in mat:
            if m1.distance < 0.6*m2.distance:
                good_matches.append([m1])
        #matches.append(sorted(mat, key = lambda x:x.distance))
        matches.append(good_matches)
        num_mat.append(len(matches[-1]))#/len(desn)
    #Select the detector with the largest number of matches
    most_mat = num_mat.index(max(num_mat))
    #Print the matches of the image with the largest number of matches
    #Print the number the matches at the bottom of the image
    cv2.putText(frame, "Matches: "+str(num_mat[most_mat]), (20,frame.shape[0]-20), cv2.FONT_HERSHEY_TRIPLEX, 1, (255,255,255), 1, cv2.LINE_AA)
    #Only if the number of matches is above 200 print the label corresponding to the image with the largest number of matches at the top of the video frame. Otherwise, if the number of matches is below 200 print nothing
    if num_mat[most_mat] >= 8:
        cv2.putText(frame, names[most_mat], (20,20), cv2.FONT_HERSHEY_TRIPLEX, 1, (255,255,255), 1, cv2.LINE_AA)
        timer_cnt = 0
    elif most_mat == last and timer_cnt < 10:
        cv2.putText(frame, names[most_mat], (20,20), cv2.FONT_HERSHEY_TRIPLEX, 1, (255,255,255), 1, cv2.LINE_AA)
        timer_cnt += 1
    last = most_mat
    #Guardar los n ultimos y si decidir el que tenga más de n/m de los últimos
    cv2.imshow('frame',frame)
cv2.destroyAllWindows()