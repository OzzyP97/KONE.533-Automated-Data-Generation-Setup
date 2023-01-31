import math
import pandas as pd
import simplepbr
from direct.showbase.ShowBase import ShowBase
from panda3d.core import *

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
CAMERA_FOV =  (math.pi/180)*60





class MyApp(ShowBase):
	
	def __init__(self):
		
		ShowBase.__init__(self)
		#simplepbr.init()
		#myMaterial = Material()
		#myMaterial.setAmbient((1, 1, 1, 1)) # Make this material blue
		alight = AmbientLight('alight')
		alight.setColor((1.0, 1.0, 1.0, 1))
		
		
		# Load the environment model.

		self.scene = self.loader.loadModel("part.bam")
		self.scene.setColor(1.0, 1.0, 1.0, 1.0)
		alnp = self.scene.attachNewNode(alight)
		self.scene.setLight(alnp)
		#self.scene.setMaterial(myMaterial)
		
		# Reparent the model to render.

		self.scene.reparentTo(self.render)

		# Apply scale and position transforms on the model.

		self.scene.setScale(0.25, 0.25, 0.25)

		self.scene.setPos(-8, 42, 0)


'''Returns a Pandas DataFrame with predefined fields'''
def init_df():
	data = pd.DataFrame(
			{'filename': pd.Series(dtype='str'), 'class': pd.Series(dtype='str'), 'id': pd.Series(dtype='int'), 'cam_pos': pd.Series(dtype='str'),
			 'cam_axis': pd.Series(dtype='str'), 'part_pos': pd.Series(dtype='str')}  )
	return data
		

if __name__ == "__main__":
	data = init_df()
	data.loc[len(data.index)] = ['test', 'part', 0, '[3;3;-3]', '[-3;-3;3]', '[0;0;0]']
	
	app = MyApp()
	app.run()
