# This is the main annotator script

import math
import time
import pandas as pd
import numpy as np
import simplepbr
import direct.directbase.DirectStart
import annotator
from panda3d.core import *
#loadPrcFileData('','win-size 640 480')
from PIL import Image
from math import sin, cos, radians

# Scene parameters
IMAGE_WIDTH = 800
IMAGE_HEIGHT = 600
CAMERA_FOV = 60

scene = loader.loadModel("part.bam")
middle_frame = render.attachNewNode("middle")
top_frame = render.attachNewNode("top")
static_frame = render.attachNewNode("static")
scene.reparentTo(middle_frame)
middle_frame.reparentTo(top_frame)
top_frame.reparentTo(static_frame)

min, max = scene.getTightBounds()
size = max-min

scene.setScale(1/size[0], 1/size[0], 1/size[0])

def generate(filename, command):
''' Renders the model from a specific angle'''
	min, max = scene.getTightBounds()
	size = max-min
	print(size)

	alight = AmbientLight('alight')
	alight.setColor((1.0, 1.0, 1.0, 1))
	alnp = scene.attachNewNode(alight)
	scene.setLight(alnp)

	base.camLens.setFov(CAMERA_FOV)

	#loadPrcFileData("", "win-size " + str(IMAGE_WIDTH) + " " + str(IMAGE_HEIGHT))
	#props = WindowProperties()
	#props.setSize(IMAGE_WIDTH, IMAGE_HEIGHT)

	#base.win.requestProperties(props)

	static_frame.setPos(10 * command['yoff'],
						10 * command['dist'] - 0.5*size[2] * sin(radians(command['pitch'])) - 10 * command['zoff'] - 0.1,
						- 0.5*size[2] * cos(radians(command['pitch'])) - 10 * command['xoff'])

	scene.setH(command['yaw'])
	middle_frame.setP(command['pitch'])
	top_frame.setR(command['roll'])

	base.graphicsEngine.render_frame()

	fname = Filename(filename)
	base.win.saveScreenshot(fname)

	#base.run()


def process(data):
'''Formats images as binary, generates bounding boxes and saves as .json'''

	new_data = annotator.init_df()

	for index, row in data.iterrows():
		image = Image.open(row['filename'] + '_temp.png').convert("RGBA")
		new_image = Image.new("RGBA", image.size, "BLACK")
		new_image.paste(image, (0, 0), image)
		new_image = new_image.convert("1")
		new_image = new_image.resize((IMAGE_WIDTH, IMAGE_HEIGHT))
		new_image.save(row['filename'] + '_mask.png')

		image_array = np.array(new_image.getdata()).reshape(IMAGE_WIDTH, IMAGE_HEIGHT)

		rows = np.any(new_image, axis=1)
		cols = np.any(new_image, axis=0)
		ymin, ymax = np.where(rows)[0][[0, -1]]
		xmin, xmax = np.where(cols)[0][[0, -1]]

		new_data.loc[len(new_data.index)] = [row['filename'], row['class'], IMAGE_WIDTH, IMAGE_HEIGHT, xmin, ymin, xmax, ymax]

	annotator.write_to_csv(new_data, 'test.csv')
	annotator.write_to_json(new_data, 'test.json')



def init_df():
'''Returns a Pandas DataFrame with predefined fields'''
	data = pd.DataFrame(
			{'filename': pd.Series(dtype='str'), 'class': pd.Series(dtype='str')} )
	return data


if __name__ == "__main__":
	data = pd.read_csv('bmask.csv')

	commands = pd.read_csv('commands.csv')

	for i in range( len(commands.index) ):
		#data.loc[len(data.index)] = ['whatever' + str(i), 'part']
		generate('whatever' + str(i) +'_temp.png', commands.iloc[i])
		time.sleep(0.5)

	process(data)
