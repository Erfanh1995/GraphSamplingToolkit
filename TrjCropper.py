# Author: Erfan Hosseini Sereshgi (shosseinisereshgi@tulane.edu)
# Company: Tulane University
# Modified: 4/07/2022

import glob
import sys
import os

XLow = 9999999
XHigh = 0
YLow = 9999999
YHigh = 0

def IsInBound(point):
	global YLow
	global YHigh
	global XLow
	global XHigh
	if point[0] < XLow and point[1] < YLow:
		return (-1, -1)
	elif point[0] < XLow and point[1] > YHigh:
		return (-1, 1)
	elif point[0] > XHigh and point[1] < YLow:
		return (1, -1)
	elif point[0] > XHigh and point[1] > YHigh:
		return (1, 1)
	elif point[0] < XLow:
		return (-1, 0)
	elif point[0] > XHigh:
		return (1, 0)
	elif point[1] < YLow:
		return (0, -1)
	elif point[1] > YHigh:
		return (0, 1)
	else:
		return (0, 0)

def ccw(A,B,C):
	return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

# Return true if line segments AB and CD intersect
def intersect(A,B,C,D):
	return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)


def line_intersection(A,B,C,D):
	xdiff = (A[0] - B[0], C[0] - D[0])
	ydiff = (A[1] - B[1], C[1] - D[1])
	line1 = [A[0:2],B[0:2]]
	line2 = [C[0:2],D[0:2]]

	def det(a, b):
		return a[0] * b[1] - a[1] * b[0]

	div = det(xdiff, ydiff)
	#if div == 0:
	#	raise Exception('lines do not intersect')

	d = (det(*line1), det(*line2))
	x = det(d, xdiff) / div
	y = det(d, ydiff) / div
	return x, y


def find_intersection(x,y,position):
	global YLow
	global YHigh
	global XLow
	global XHigh
	if position[0] == -1:
		m = [XLow,YLow]
		n = [XLow,YHigh]
		if intersect(x,y,m,n) == True:
			inx, iny = line_intersection(x,y,m,n)
			return [inx, iny, y[2]]
	if position[0] == 1:
		m = [XHigh,YLow]
		n = [XHigh,YHigh]
		if intersect(x,y,m,n) == True:
			inx, iny = line_intersection(x,y,m,n)
			return [inx, iny, y[2]]
	if position[1] == -1:
		m = [XLow,YLow]
		n = [XHigh,YLow]
		if intersect(x,y,m,n) == True:
			inx, iny = line_intersection(x,y,m,n)
			return [inx, iny, y[2]]
	if position[1] == 1:
		m = [XLow,YHigh]
		n = [XHigh,YHigh]
		if intersect(x,y,m,n) == True:
			inx, iny = line_intersection(x,y,m,n)
			return [inx, iny, y[2]]
	return None

if __name__ == '__main__':
	help_string = "Usage: python TrjCropper.py <dataset>"
	if len(sys.argv) < 1:
		print(help_string)
		exit()
	data = sys.argv[1] # example: "/Users/erfan/Teleconverter/athens_large_vertices_osm.txt"

	with open("data/"+data+"/groundtruth/"+data+"_vertices_osm.txt",'r') as f1:
		vertices = f1.readlines()
		print("Done!")

	for i in vertices:
		a = i.strip('\n').split(',')
		if float(a[1]) > XHigh:
			XHigh = float(a[1])
		if float(a[1]) < XLow:
			XLow = float(a[1])
		if float(a[2]) > YHigh:
			YHigh = float(a[2])
		if float(a[2]) < YLow:
			YLow = float(a[2])

	pathname = 'data/'+data+'/trajectories/*.txt'
	files = glob.glob(pathname)
	if not os.path.exists('data/'+data+'/trajectories_cropped'):
		os.makedirs('data/'+data+'/trajectories_cropped')
	curr_traj = []
	counter = 0
	for name in files:
		previous = None
		with open(name, 'r') as f3:
			for line in f3:
				temp = line.strip('\n').split(" ")
				point = [float(temp[0]), float(temp[1]), float(temp[2])]
				position = IsInBound(point)
				if position == (0,0):
					if previous is not None: # first point is OUT, second point is IN
						first_point = find_intersection(previous,point,IsInBound(previous))
						if first_point is not None:
							curr_traj.append(first_point)
					curr_traj.append(point) # normal IN point
				elif previous is not None:
					if IsInBound(previous) == position: # both points are out and do not cross the frame -> Discard
						previous = point

					elif IsInBound(previous) == (0,0): # first point is IN, second point is OUT
						last_point = find_intersection(previous,point,position) # find the last point and add it to curr_traj
						curr_traj.append(last_point)
						with open('data/'+data+'/trajectories_cropped'+"/trip_"+str(counter)+".txt",'w') as f4:
							for item in curr_traj:
								f4.write(str(item[0])+" "+str(item[1])+" "+str(item[2])+"\n")
						print("file "+str(counter))
						curr_traj = []
						counter += 1

					else: # both points are OUT but might have crossing
						first_point = find_intersection(previous, point, IsInBound(previous))
						last_point = find_intersection(previous, point, position)
						if first_point is not None and last_point is not None:
							curr_traj.append(first_point)
							curr_traj.append(last_point)
							with open('data/'+data+'/trajectories_cropped'+"/trip_" + str(counter) + ".txt", 'w') as f4:
								for item in curr_traj:
									f4.write(str(item[0])+" "+str(item[1])+" "+str(item[2])+"\n")
							print("file " + str(counter))
							curr_traj = []
							counter +=1
				previous = point
		if curr_traj != []:
			with open('data/'+data+'/trajectories_cropped'+"/trip_" + str(counter) + ".txt", 'w') as f4:
				for item in curr_traj:
					f4.write(str(item[0]) + " " + str(item[1]) + " " + str(item[2]) + "\n")
			print("file " + str(counter))
			curr_traj = []
			counter +=1
		print("Done with file "+name)
	print("All done")
