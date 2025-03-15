import cv2
import numpy as np
import matplotlib.pyplot as plt

def extract_and_match_features(image1_path, image2_path):
    # Load images in grayscale
    image1 = cv2.imread(image1_path, cv2.IMREAD_GRAYSCALE)
    image2 = cv2.imread(image2_path, cv2.IMREAD_GRAYSCALE)
    
    if image1 is None or image2 is None:
        print("Error: Could not load one or both images.")
        return
    
    # Initialize ORB detector
    orb = cv2.ORB_create(nfeatures=300)
    
    # Detect keypoints and compute descriptors
    keypoints1, descriptors1 = orb.detectAndCompute(image1, None)
    keypoints2, descriptors2 = orb.detectAndCompute(image2, None)
    
    # Use BFMatcher with Hamming distance (since ORB produces binary descriptors)
    bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    
    # Match descriptors
    matches = bf.match(descriptors1, descriptors2)
    
    # Sort matches by distance (lower is better)
    matches = sorted(matches, key=lambda x: x.distance)
    
    # Draw keypoints on both images
    image1_keypoints = cv2.drawKeypoints(image1, keypoints1, None, color=(0, 255, 0))
    image2_keypoints = cv2.drawKeypoints(image2, keypoints2, None, color=(0, 255, 0))
    
    # Draw matches
    matched_image = cv2.drawMatches(image1, keypoints1, image2, keypoints2, matches[:50], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)
    
    # Display feature extraction in one figure
    plt.figure(figsize=(10, 5))
    plt.subplot(1, 2, 1)
    plt.imshow(image1_keypoints, cmap='gray')
    plt.title("Image 1 - Feature Extraction")
    
    plt.subplot(1, 2, 2)
    plt.imshow(image2_keypoints, cmap='gray')
    plt.title("Image 2 - Feature Extraction")
    plt.show()
    
    # Display feature matching in another figure
    plt.figure(figsize=(10, 5))
    plt.imshow(matched_image)
    plt.title("Feature Matching")
    plt.show()


# Example usage:
extract_and_match_features("/ros2_ws/images/image_24.jpg", "/ros2_ws/images/image_28.jpg")
