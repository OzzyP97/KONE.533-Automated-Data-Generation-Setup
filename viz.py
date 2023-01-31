import pandas as pd
import cv2

def visualize(data):
	for i in range(len(data.index)):
		path = data['filename'][i]
		img = cv2.imread(path + '.png')

		cv2.line(img, ( data['xmin'][i], data['ymin'][i] ),
						( data['xmax'][i], data['ymin'][i] ), ( 0, 0, 255 ), 2)

		cv2.line(img, ( data['xmin'][i], data['ymin'][i] ),
						( data['xmin'][i], data['ymax'][i] ), ( 0, 0, 255 ), 2)

		cv2.line(img, ( data['xmin'][i], data['ymax'][i] ),
						( data['xmax'][i], data['ymax'][i] ), ( 0, 0, 255 ), 2)

		cv2.line(img, ( data['xmax'][i], data['ymin'][i] ),
						( data['xmax'][i], data['ymax'][i] ), ( 0, 0, 255 ), 2)

		cv2.imshow("Image", img)
		cv2.imwrite(path + '_viz.png', img)
		cv2.waitKey(1000)
		cv2.destroyAllWindows()

if __name__ == "__main__":
	path = input("Enter path to csv: ")
	data = pd.read_csv(path)
	visualize(data)
