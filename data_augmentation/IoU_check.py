import cv2

manually_label_img = cv2.imread('/home/iclab-giga/Documents/manually_label_image/manually_label_Koopa.png',0)
auto_label_img = cv2.imread('/home/iclab-giga/Documents/manually_label_image/training_data_4_koopa.png',0)

rows,cols = manually_label_img.shape
counter = 0

for i in range(rows):
	for j in range(cols):
		if manually_label_img[i][j] == auto_label_img[i][j]:
			counter = counter+1

IoU = float(counter)/(cols*rows)
print IoU
