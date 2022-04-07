#
# Author: Erfan Hosseini Sereshgi (shosseinisereshgi@tulane.edu)
# Company: Tulane University
# Modified: 4/07/2022
#
import glob
from leuvenmapmatching.matcher.distance import DistanceMatcher
from leuvenmapmatching.map.inmem import InMemMap
import math
import sys

#ground truth graph
graph = {}

help_string = "Usage: python MapCropper.py <dataset>"
if len(sys.argv) < 1:
	print(help_string)
	exit()
data = sys.argv[1]

#reading and adding vertices
with open("data/"+data+"/groundtruth/"+data+"_vertices_osm.txt", 'r') as f1:
	for line in f1:
		temp = line.split(',')
		graph[temp[0]]= ((float(temp[1]),float(temp[2])), [])
f1.close()

#reading and adding edges
with open("data/"+data+"/groundtruth/"+data+"_edges_osm.txt", 'r') as f2:
	for line in f2:
		temp = line.split(',')
		graph[temp[1]][1].append(temp[2])
		graph[temp[2]][1].append(temp[1])
f2.close()

#making the ground truth map
map_con = InMemMap("mymap", graph=graph, use_latlon=False)

#making the matcher obj
matcher = DistanceMatcher(map_con, max_dist=100, max_dist_init=50, obs_noise=50, obs_noise_ne=75, dist_noise=50, non_emitting_states=True)

final_v = []
final_e = []
#reading paths/trajectories and map matching
paths = []
pathname = 'data/'+data+'/trajectories/*.txt'
files = glob.glob(pathname)
num_evals = len(files)
inx = 0
for name in files:
	path = []

	progress = inx / num_evals
	barsize = int(math.floor(progress * 50))
	bar = ("=" * barsize) + ">" + ("_" * (50 - barsize))
	print("'\r{percent:3.0f}% [{bar}]".format(percent=progress * 100, bar=bar), end="")

	with open(name, 'r') as f3:
		for line in f3:
			temp = line.split()
			path.append((float(temp[0]),float(temp[1])))

	#map matching
	try:
		states, _ = matcher.match(path)
		if states == []:
			nodes = []
		else:
			nodes = matcher.path_pred_onlynodes
		#print("States\n------")
		#print(states)
		#print("Nodes\n------")
		#print(nodes)
		#node_locs = []
		#graph_nodes = []
		#graph_edges = []
		#prev_m = None
		#if nodes:
		#	for node in nodes:
		#		if type(node) == tuple:
		#			node = node[0]
		#		lat, lon = map_con.node_coordinates(node)
		#		node_locs.append((lat, lon, node))
		#		graph_nodes.append(node)
		#else:
		#	for m in matcher.lattice_best:
		#		if prev_m is not None and prev_m.edge_m.l2 == m.edge_m.l1:
		#			lat, lon = m.edge_m.p1
		#			node_locs.append((lat, lon, m.edge_m.l1))
		#			graph_nodes.append(m.edge_m.l1)
		#		lat, lon = m.edge_m.pi
		#		node_locs.append((lat, lon, m.edge_m.label))
		#		temp = m.edge_m.label.split("-")
		#		graph_edges.append([temp[0],temp[1]])
		#		prev_m = m
		#print(node_locs)
		#print(graph_nodes)
		#print(graph_edges)
	except:
		print("Did not find a matching node for path point")

	inx += 1

	#saving matched parts
	for i in nodes:
		if i not in final_v:
			final_v.append(i)
	for j in states:
		if j not in final_e:
			final_e.append(j)

#sorting vertices based on their initial index
#final_v.sort(key=int)

#writing the vertices
print("Generating the files...")
with open("data/"+data+"/groundtruth/"+data+"_vertices_crp.txt", 'w') as f4:
	for i in range(len(final_v)):
		f4.write(str(i+1)+","+str(graph[final_v[i]][0][0])+","+str(graph[final_v[i]][0][1])+"\n")
f4.close()

#writing the edges
index = 1
with open("data/"+data+"/groundtruth/"+data+"_edges_crp.txt", 'w') as f5:
	for i in final_e:
		f5.write(str(index)+","+str(final_v.index(i[0])+1)+","+str(final_v.index(i[1])+1)+",1"+"\n")
		index += 1

f5.close()

print("Done.")

