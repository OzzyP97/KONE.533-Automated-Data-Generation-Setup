# This script generates a commands.csv for use with moveit_controller.py.
# Parameters can be configured at the beginning of main().

import csv
import random

def generate_commands(param, order):
''' Recursive function for generating all possible combinations'''

	data = []

	if order == 0:
		param.reverse()

	if order < len(param):
		for p in param[order]:
			if order < len(param) - 1:
				rows = generate_commands(param, order + 1)
				for r in rows:
					r.append(p)
					data.append(r)
			else:
				data.append([p])

	return data

def add_randomness(commands, index, amplitude):
''' Adds random noise with specified targets and amounts '''

	randomized_commands = commands

	for c in randomized_commands:
		for i in index:
			c[i] = round( amplitude[i] * random.randint(-100, 100) / 100, 3 )

	return randomized_commands

def write_commands(commands, filename):
''' Writes the resulting commands to a file'''

	with open(filename, 'w', newline='') as csvfile:
		writer = csv.writer(csvfile, delimiter=',',
												quotechar='|', quoting=csv.QUOTE_MINIMAL)
		writer.writerow(['yaw', 'pitch', 'dist', 'roll', 'xoff', 'yoff', 'zoff'])
		writer.writerows(commands)


if __name__ == '__main__':

	# Generate every possible combination of these parameters
	yaw = [-30, 0, 20]
	pitch = [45, 60]
	dist = [0.25]
	roll = [0]
	xoff = [0]
	yoff = [0]
	zoff = [0]

	# Select parameters to randomly offset
	rnd_enable = [4,5] # xoff, yoff
	# Maximum random offset for each parameter
	rnd_amount = [0, 0, 0, 0, 0.015, 0.015, 0]

	param = [yaw, pitch, dist, roll, xoff, yoff, zoff]

	commands = generate_commands(param, 0)
	commands = add_randomness(commands, rnd_enable, rnd_amount)

	write_commands(commands, 'commands.csv')

	print( commands )
