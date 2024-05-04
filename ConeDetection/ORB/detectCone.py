import numpy as np 
import cv2 

#function to resize the image befor cv2.imshow
def ResizeWithAspectRatio(image, width=None, height=None, inter=cv2.INTER_AREA):
    dim = None
    (h, w) = image.shape[:2]

    if width is None and height is None:
        return image
    if width is None:
        r = height / float(h)
        dim = (int(w * r), height)
    else:
        r = width / float(w)
        dim = (width, int(h * r))

    return cv2.resize(image, dim, interpolation=inter)

	
# Read the query image as query_img 
# and train image This query image 
# is what you need to find in train image 
# Save it in the same directory 
# with the name image.jpg  
query_img = cv2.imread('Cones/trainCones/trainRed1.jpg') 
# train_img = cv2.imread('Cones/queryCones/redCone2.jpg')
train_img = cv2.imread('Cones/queryCones/manyCones1.jpg')

# Convert it to grayscale 
query_img_bw = cv2.cvtColor(query_img,cv2.COLOR_BGR2GRAY) 
train_img_bw = cv2.cvtColor(train_img, cv2.COLOR_BGR2GRAY) 


# Initialize the ORB detector algorithm 
# orb = cv2.ORB_create()
orb = cv2.ORB_create(200, 2) 

# Now detect the keypoints and compute the descriptors for the query image 
# and train image 
queryKeypoints, queryDescriptors = orb.detectAndCompute(query_img_bw,None) 
trainKeypoints, trainDescriptors = orb.detectAndCompute(train_img_bw,None)

# draw only keypoints location,not size and orientation
query_img_keypoints = cv2.drawKeypoints(query_img_bw, queryKeypoints, None, color=(0,255,0), flags=0)
cv2.imshow("keypoints query images",ResizeWithAspectRatio(query_img_keypoints, width=700))

# Initialize the Matcher for matching the keypoints and then match the 
# keypoints 
matcher = cv2.BFMatcher() 
matches = matcher.match(queryDescriptors,trainDescriptors) 

# Sort them in the order of their distance.
matches = sorted(matches, key = lambda x:x.distance)
 
# draw the matches to the final image containing both the images
# the drawMatches() function takes both images and keypoints 
# and outputs the matched query image with its train image 
final_img = cv2.drawMatches(query_img_bw, queryKeypoints, train_img_bw, trainKeypoints, matches[:20],None,flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS) 

final_img = cv2.resize(final_img, (1000,650)) 

# Show the final image 
cv2.imshow("Matches", final_img) 
cv2.waitKey(10000) 