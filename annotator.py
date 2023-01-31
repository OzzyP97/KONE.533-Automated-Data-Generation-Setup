# Built with help form a tutorial: https://medium.com/analytics-vidhya/how-to-convert-tensorflow-object-detection-csv-data-to-coco-json-format-d0693d5b2f75
# No license was specified, but the code being used is very generic in nature,
# so it did not feel sensible to to try and obscure this fact by writing nearly
# identical code with a different skin on it.

import pandas as pd
import numpy as np
import json

'''Returns a Pandas DataFrame with predefined fields'''
def init_df():
	data = pd.DataFrame(
			{'filename': pd.Series(dtype='str'), 'class': pd.Series(dtype='str'), 'width': pd.Series(dtype='int'), 'height': pd.Series(dtype='int'),
			 'xmin': pd.Series(dtype='int'), 'ymin': pd.Series(dtype='int'), 'xmax': pd.Series(dtype='int'), 'ymax': pd.Series(dtype='int') }  )

	return data

'''Returns an array corresponding to the image section of a COCO json'''
def image(row):
	image = {}
	image["height"] = row.height
	image["width"] = row.width
	image["id"] = row.fileid
	image["file_name"] = row.filename

	return image


'''Returns an array corresponding to the category section of a COCO json'''
def category(row):
	category = {}
	category["supercategory"] = 'None'
	category["id"] = row.categoryid
	category["name"] = row[2]

	return category


'''Returns an array corresponding to the annotation section of a COCO json'''
def annotation(row):
	annotation = {}
	area = (row.xmax -row.xmin)*(row.ymax - row.ymin)
	annotation["segmentation"] = []
	annotation["iscrowd"] = 0
	annotation["area"] = area
	annotation["image_id"] = row.fileid

	annotation["bbox"] = [row.xmin, row.ymin, row.xmax -row.xmin,row.ymax-row.ymin ]

	annotation["category_id"] = row.categoryid
	annotation["id"] = row.annid

	return annotation


'''Takes in a init() formatted dataframe, converts it to COCO json and saves to disk'''
def write_to_json(data, path):
	images = []
	categories = []
	annotations = []

	supercategory = {}
	supercategory["supercategory"] = 'none'
	supercategory["id"] = 0
	supercategory["name"] = 'None'
	categories.append(supercategory)

	data['fileid'] = data['filename'].astype('category').cat.codes
	data['categoryid']= pd.Categorical(data['class'],ordered= True).codes
	data['categoryid'] = data['categoryid']+1
	data['annid'] = data.index

	for row in data.itertuples():
		annotations.append(annotation(row))

	imagedf = data.drop_duplicates(subset=['fileid']).sort_values(by='fileid')

	for row in imagedf.itertuples():
		images.append(image(row))

	catdf = data.drop_duplicates(subset=['categoryid']).sort_values(by='categoryid')

	for row in catdf.itertuples():
		categories.append(category(row))

	data_coco = {}
	data_coco["images"] = images
	data_coco["categories"] = categories
	data_coco["annotations"] = annotations
	json.dump(data_coco, open(path, "w"), indent=4)


'''Takes an init() formatted dataframe and saves it directly to disk as a csv file.'''
def write_to_csv(data, path):
	with open(path, 'w') as f:
		f.write(data.to_csv(index=False))


def read_from_csv(path):
	data = pd.read_csv(path)

''' This is a testing function '''
if __name__ == "__main__":
	print ( "Generating test files" )
	data = init_df()
	data.loc[len(data.index)] = ['test.png', 'part', 640, 480, 310, 230, 330, 250]
	write_to_csv(data, 'out.csv')
	write_to_json(data, 'out.json')
