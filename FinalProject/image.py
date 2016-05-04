import cv2

# import in large photo
l_img = cv2.imread("test.png")

# location of the geofilter
x_offset=120
y_offset=175

# import in the harvard logo
s_img = cv2.imread("SEASLogo1.png", -1)

for c in range(0,3):
    l_img[y_offset:y_offset+s_img.shape[0], x_offset:x_offset+s_img.shape[1], c] = s_img[:,:,c] * (s_img[:,:,3]/255.0) +  l_img[y_offset:y_offset+s_img.shape[0], x_offset:x_offset+s_img.shape[1], c] * (1.0 - s_img[:,:,3]/255.0)

# download the new image
cv2.imwrite("new_photo.png", l_img)