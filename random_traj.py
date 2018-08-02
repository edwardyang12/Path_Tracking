import random
import matplotlib
import matplotlib.pyplot as plt
import math

STEP = 5
N_TEST_TRAJ = 5
N_TRAIN_TRAJ = 5

xList = []
yList = []

def generate_path(STEP):
	path_list = []
	point = [0,0]
	location = 0
	path_list.append(point)
	for _ in range(20):
		angle = random.uniform(-math.pi/4,math.pi/4)
		y_val = math.sin(angle)*STEP + point[1]
		x_val = math.cos(angle)*STEP + point[0]
		point = [x_val,y_val]
		path_list.append(point)
	return path_list

def generate_paths():
	paths =[]
	total = N_TEST_TRAJ + N_TRAIN_TRAJ
	for i in range(total):
		path_list = generate_path(STEP)
		paths.append(path_list)
	return paths


paths = generate_paths()
for i in range(len(paths)):
	xList.append(paths[0][i][0])
	yList.append(paths[0][i][1])


plt.plot(xList, yList, '-o')
plt.show()
TRAIN_TRAJ = paths[:N_TEST_TRAJ]
TEST_TRAJ = paths[N_TEST_TRAJ:]
