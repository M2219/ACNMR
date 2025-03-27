import cv2

# Load the grayscale image
img = cv2.imread("map_slalom_org.png", cv2.IMREAD_GRAYSCALE)

# Resize: Reduce width to half while keeping the same height
resized = cv2.resize(img, (2*img.shape[1] // 3, img.shape[0]))

# Save the resized image
cv2.imwrite("map_slalom.png", resized)
