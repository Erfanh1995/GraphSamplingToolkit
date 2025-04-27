# The Visualizer
# Author: Erfan Hosseini Sereshgi (shosseinisereshgi@tulane.edu)
# Company: Tulane University
# Modified: 4/26/2025
#
import fiona
import pandas as pd
import geopandas as gpd
import sys
import contextily as ctx
import os
import glob
import yaml

from shapely.geometry import LineString
from PyQt5 import QtCore, QtWidgets

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure


class MainWindow(QtWidgets.QMainWindow):

	def __init__(self):
		super().__init__()

		# Keep track of open windows
		self.w = None

		# Read list of files in data directory
		list_of_datasets = [name for name in os.listdir('data') if os.path.isdir(os.path.join('data', name))]

		# Building the elements
		layout = QtWidgets.QVBoxLayout()
		self.datalistwidget = QtWidgets.QListWidget()
		self.datalistwidget.addItems(list_of_datasets)
		self.datalistwidget.clicked.connect(self.dataset_clicked)
		self.algolistwidget = QtWidgets.QListWidget()
		self.evallistwidget = QtWidgets.QComboBox()
		self.algolistwidget.clicked.connect(self.algorithm_clicked)
		self.button = QtWidgets.QPushButton("Start!")
		self.button.clicked.connect(self.buttonclicked)
		self.trjcheck = QtWidgets.QCheckBox("Show trajectories")
		self.rlcheck = QtWidgets.QCheckBox("Show root locations")
		self.bcpcheck = QtWidgets.QCheckBox("Show breadcrumb paths")
		self.bgmcheck = QtWidgets.QCheckBox("Show background map")
		self.cropgt = QtWidgets.QCheckBox("Use Cropped ground truth")
		layout.addWidget(self.datalistwidget)
		layout.addWidget(self.algolistwidget)
		layout.addWidget(self.evallistwidget)
		layout.addWidget(self.trjcheck)
		layout.addWidget(self.rlcheck)
		layout.addWidget(self.bcpcheck)
		layout.addWidget(self.bgmcheck)
		layout.addWidget(self.cropgt)
		layout.addWidget(self.button)
		widget = QtWidgets.QWidget()
		widget.setLayout(layout)
		self.setWindowTitle("The Visualizer")
		self.setCentralWidget(widget)

	def dataset_clicked(self):
		dataset = self.datalistwidget.currentItem()
		list_of_algorithms = [name for name in os.listdir('data/' + dataset.text() + '/algorithm') if os.path.isdir(os.path.join('data/' + dataset.text() + '/algorithm', name))]
		self.algolistwidget.clear()
		self.evallistwidget.clear()
		self.algolistwidget.addItems(list_of_algorithms)


	def algorithm_clicked(self):
		dataset = self.datalistwidget.currentItem()
		algorithm = self.algolistwidget.currentItem()
		try:
			list_of_evals = []
			for file in os.listdir('data/' + dataset.text() + '/evals'):
				if file.endswith(".txt") and algorithm.text().lower() in file.lower():
					list_of_evals.append(file.strip(".txt"))
			self.evallistwidget.clear()
			self.evallistwidget.setPlaceholderText("--Select Eval file--")
			self.evallistwidget.setCurrentIndex(-1)
			self.evallistwidget.addItems(list_of_evals)

		except IOError:
			self.evallistwidget.clear()
			print("No eval folder was found")



	def buttonclicked(self):
		dataset = self.datalistwidget.currentItem()
		algorithm = self.algolistwidget.currentItem()
		evaluation = self.evallistwidget.currentText()
		if dataset != None and algorithm != None and self.w == None :
			data = dataset.text()
			algo = algorithm.text()
			if evaluation is None:
				eval = ""
			else:
				eval = evaluation
			self.w = PlotWindow(data, algo, eval, self.trjcheck.isChecked(), self.rlcheck.isChecked(), self.bcpcheck.isChecked(), self.cropgt.isChecked(), self.bgmcheck.isChecked())
			self.hide()
			self.w.show()
		else:
			msg = QtWidgets.QMessageBox(self)
			msg.setWindowTitle("Error")
			msg.setIcon(QtWidgets.QMessageBox.Information)
			msg.setText("Please select a dataset and an algorithm")
			msg.exec_()



class PlotWindow(QtWidgets.QWidget):

	def __init__(self, data, algo, eval, trjFlag, rlFlag, bcpFlag, cropFlag, bgmFlag):
		super().__init__()

		# Create the maptlotlib FigureCanvas object, which defines a single set of axes.
		canvas = FigureCanvasQTAgg(Figure(figsize=(12, 10), dpi=100))
		axes = canvas.figure.add_subplot(111)
		canvas.figure.subplots_adjust(right=0.70)

		# Set EPSG codes
		try:
			epsg = yaml.safe_load(open("data/"+data+"/"+data+".yml"))
		except IOError:
			print("No config file was found - using the default EPSG:3857")
			epsg = "EPSG:3857"


		if cropFlag is True:
			if os.path.exists("data/"+data+"/groundtruth/"+data+"_vertices_crp.txt") and os.path.exists("data/"+data+"/groundtruth/"+data+"_edges_crp.txt"):
				ext = "crp"
			else:
				print("No cropped ground truth file was found - Using original ground truth")
				ext = "osm"
		else:
			ext = "osm"


		# Reading the ground truth
		try:
			# Reading vertices
			vertices = {}
			with open("data/"+data+"/groundtruth/"+data+"_vertices_"+ext+".txt", 'r') as f1:
				for line in f1:
					temp = line.strip('\n').split(',')
					vertices[temp[0]] = (float(temp[1]), float(temp[2]))

			# Reading edges
			edges = []
			with open("data/"+data+"/groundtruth/"+data+"_edges_"+ext+".txt", 'r') as f2:
				for line in f2:
					temp = line.strip('\n').split(',')
					edges.append([vertices[temp[1]], vertices[temp[2]]])

			graph = pd.DataFrame(edges, columns=['p1', 'p2'])
			geometry = [LineString(p1p2) for p1p2 in zip(graph.p1, graph.p2)]
			groundtruth = gpd.GeoDataFrame(graph, geometry=geometry, crs=epsg)
			#groundtruth = groundtruth.to_crs(epsg=3857)
			base = groundtruth.plot(ax=axes, zorder=1, label="Ground truth")

		except IOError:
			print("No ground truth file was found")


		if rlFlag == True:
			# Reading root locations
			try:
				rootlocations = []
				with open("data/"+data+"/"+data+"_"+algo+"_rootlocations.txt", 'r') as f3:
					for line in f3:
						temp = line.strip('\n').split(',')
						rootlocations.append((float(temp[2]), float(temp[3])))

				rootlocation_points = pd.DataFrame(rootlocations, columns=['x', 'y'])
				rl_points = gpd.GeoDataFrame(rootlocation_points, geometry=gpd.points_from_xy(rootlocation_points.x, rootlocation_points.y), crs=epsg)
				#rl_points = rl_points.to_crs(epsg=3857)
				rl = rl_points.plot(ax=axes, marker='*', color='black', markersize=7, zorder=4, label="Root locations")

			except IOError:
				print("No root locations file was found")


		if trjFlag == True:
			# Reading GPS trajectories
			try:
				pathname = "data/" + data + "/trajectories/*.txt"
				files = glob.glob(pathname)
				paths = []
				for name in files:
					previous = None
					with open(name, 'r') as f3:
						for line in f3:
							temp = line.strip('\n').split()
							if previous is None:
								previous = (float(temp[0]), float(temp[1]))
							else:
								paths.append([previous, (float(temp[0]), float(temp[1]))])
								previous = (float(temp[0]), float(temp[1]))

				path_graph = pd.DataFrame(paths, columns=['p1', 'p2'])
				geo = [LineString(p1p2) for p1p2 in zip(path_graph.p1, path_graph.p2)]
				trajectories = gpd.GeoDataFrame(path_graph, geometry=geo, crs=epsg)
				trajs = trajectories.plot(ax=axes, color='lime', zorder=2, label="Trajectories")

			except IOError:
				print("No trajectory file was found")


		# Reading the reconstructed map/dataset
		try:
			# Reading vertices
			rcm_vertices = {}
			with open("data/"+data+"/algorithm/"+algo+"/"+data+"_"+algo+"_vertices.txt", 'r') as f1:
				for line in f1:
					temp = line.strip('\n').split(',')
					rcm_vertices[temp[0]] = (float(temp[1]), float(temp[2]))

			# Reading edges
			rcm_edges = []
			with open("data/"+data+"/algorithm/"+algo+"/"+data+"_"+algo+"_edges.txt", 'r') as f2:
				for line in f2:
					temp = line.strip('\n').split(',')
					rcm_edges.append([rcm_vertices[temp[1]], rcm_vertices[temp[2]]])

			reconstructed_map = pd.DataFrame(rcm_edges, columns=['p1', 'p2'])
			geometry1 = [LineString(p1p2) for p1p2 in zip(reconstructed_map.p1, reconstructed_map.p2)]
			rcm_map = gpd.GeoDataFrame(reconstructed_map, geometry=geometry1, crs=epsg)
			#rcm_map = rcm_map.to_crs(epsg=3857)
			rcm = rcm_map.plot(ax=axes, color='red', zorder=3, label="Reconstructed map")

		except IOError:
			print("No Reconstructed map file was found")


		if eval != "":
			# Reading matched/unmatched breadcrumbs
			try:
				matchedpoints1 = []
				matchedpoints2 = []
				unmatched1 = []
				unmatched2 = []
				connected_edges = []
				counter = 0
				prev = ""
				with open("data/"+data+"/evals/"+eval+".txt", 'r') as f4:
					for line in f4:
						temp = line.strip('\n').split(',')
						if temp[0] != '1000000' and temp[0] != '2000000':
							matchedpoints1.append((float(temp[1]), float(temp[2])))
							matchedpoints2.append((float(temp[3]), float(temp[4])))
							connected_edges.append([matchedpoints1[-1],matchedpoints2[-1]])
						elif temp[0] == '1000000':
							if prev == '2000000':
								counter += 1
							elif prev != '1000000':
								counter += 1
							unmatched1.append((float(temp[1]), float(temp[2])))
						elif temp[0] == '2000000':
							if prev != '1000000' and prev != '2000000':
								counter += 1
							unmatched2.append((float(temp[3]), float(temp[4])))
						prev = temp[0]

				#Computing Scores
				match = len(matchedpoints2)
				precision = match/(len(matchedpoints2)+len(unmatched2))
				recall = match/(len(matchedpoints1)+len(unmatched1))
				f_score = 2*match/(len(unmatched1)+len(unmatched2)+2*match)

				# Showing ground truth's unmatched points
				um1 = pd.DataFrame(unmatched1, columns=['x', 'y'])
				um1_converted = gpd.GeoDataFrame(um1, geometry=gpd.points_from_xy(um1.x, um1.y), crs=epsg)
				# um1_converted = um1_converted.to_crs(epsg=3857)
				um1_c = um1_converted.plot(ax=axes, marker='+', color='orange', markersize=5, zorder=6, label="Unmatched on GT")
				# Showing dataset's unmatched points
				um2 = pd.DataFrame(unmatched2, columns=['x', 'y'])
				um2_converted = gpd.GeoDataFrame(um2, geometry=gpd.points_from_xy(um2.x, um2.y), crs=epsg)
				# um2_converted = um2_converted.to_crs(epsg=3857)
				um2_c = um2_converted.plot(ax=axes, marker='+', color='yellow', markersize=5, zorder=6, label="Unmatched on RCM")
				# Showing ground truth's matched points
				mp1 = pd.DataFrame(matchedpoints1, columns=['x', 'y'])
				mp1_converted = gpd.GeoDataFrame(mp1, geometry=gpd.points_from_xy(mp1.x, mp1.y), crs=epsg)
				#mp1_converted = mp1_converted.to_crs(epsg=3857)
				mp1_c = mp1_converted.plot(ax=axes, marker='x', color='cyan', markersize=5, zorder=6, label="Matched on GT")
				# Showing dataset's matched points
				mp2 = pd.DataFrame(matchedpoints2, columns=['x', 'y'])
				mp2_converted = gpd.GeoDataFrame(mp2, geometry=gpd.points_from_xy(mp2.x, mp2.y), crs=epsg)
				#mp2_converted = mp2_converted.to_crs(epsg=3857)
				mp2_c = mp2_converted.plot(ax=axes, marker='x', color='purple', markersize=5, zorder=6, label="Matched on RCM")

				# Showing connected parts between matches/matchings
				connected = pd.DataFrame(connected_edges, columns=['p1', 'p2'])
				geometry2 = [LineString(p1p2) for p1p2 in zip(connected.p1, connected.p2)]
				connected_map = gpd.GeoDataFrame(connected, geometry=geometry2, crs=epsg)
				#connected_map = connected_map.to_crs(epsg=3857)
				con_map = connected_map.plot(ax=axes, color='magenta', zorder=4, label="Matching")


			except IOError:
				print("No Evaluation file was found")
				match = 0
				unmatched1 = []
				unmatched2 = []
				precision = 0
				recall = 0
				f_score = 0

		else:
			match = 0
			unmatched1 = []
			unmatched2 = []
			precision = 0
			recall = 0
			f_score = 0


		if bcpFlag == True:
			# Reading and showing breadcrumb paths
			try:
				breadcrumb_path1 = []
				with open("data/"+data+"/breadcrumb_paths/map1_breadcrumb_path_new.txt", 'r') as f5:
					for line in f5:
						temp = line.strip('\n').split(',')
						breadcrumb_path1.append((float(temp[0]), float(temp[1])))
				bcp1 = pd.DataFrame(breadcrumb_path1, columns=['x', 'y'])
				bpc1_converted = gpd.GeoDataFrame(bcp1, geometry=gpd.points_from_xy(bcp1.x, bcp1.y), crs=epsg)
				#bpc1_converted = bpc1_converted.to_crs(epsg=3857)
				bpc1_converted.plot(ax=axes, marker = '^', color="purple", markersize=5, zorder=5, label="Breadcrumb path on GT")


				breadcrumb_path2 = []
				with open("data/"+data+"/breadcrumb_paths/map2_breadcrumb_path_new.txt", 'r') as f6:
					for line in f6:
						temp = line.strip('\n').split(',')
						breadcrumb_path2.append((float(temp[0]), float(temp[1])))
				bcp2 = pd.DataFrame(breadcrumb_path2, columns=['x', 'y'])
				bpc2_converted = gpd.GeoDataFrame(bcp2, geometry=gpd.points_from_xy(bcp2.x, bcp2.y), crs=epsg)
				#bpc2_converted = bpc2_converted.to_crs(epsg=3857)
				bpc2_converted.plot(ax=axes, marker='^', color="purple", markersize=5, zorder=5, label="Breadcrumb path on RCM")

			except IOError:
				print("No Breadcrumb path file was found")

		# Drawing background map
		if bgmFlag is True:
			try:
				ctx.add_basemap(axes, crs=epsg) #
			except:
				print("Background map could not be found")
		#else:
			#axes.set_facecolor('#d3d3d3')

		# Adding legend and making it dynamic
		leg = axes.legend(loc='upper left', bbox_to_anchor=(1.05, 1), borderaxespad=0)

		labels = [t.get_text() for t in leg.texts]
		handles = leg.legend_handles
		label2handle = dict(zip(labels, handles))
		handle2text = dict(zip(handles, leg.texts))

		lookup_artist = {}
		lookup_handle = {}
		for artist in leg.axes.get_children():
			if artist.get_label() in labels:
				handle = label2handle[artist.get_label()]
				lookup_handle[artist] = handle
				lookup_artist[handle] = artist
				lookup_artist[handle2text[handle]] = artist

		lookup_handle.update(zip(handles, handles))
		lookup_handle.update(zip(leg.texts, handles))

		for artist in leg.texts + leg.legend_handles:
			artist.set_picker(True)

		def on_pick(event):
			handle = event.artist
			if handle in lookup_artist:
				artist = lookup_artist[handle]
				artist.set_visible(not artist.get_visible())
				update()

		def on_click(event):
			if event.button == 3:
				visible = False
			elif event.button == 2:
				visible = True
			else:
				return

			for artist in lookup_artist.values():
				artist.set_visible(visible)
			update()

		def update():
			for artist in lookup_artist.values():
				handle = lookup_handle[artist]
				if artist.get_visible():
					handle.set_visible(True)
				else:
					handle.set_visible(False)
			canvas.draw()

		canvas.mpl_connect('pick_event', on_pick)
		canvas.mpl_connect('button_press_event', on_click)

		#Creating the score box
		textstr = '\n'.join((
			r'Precision = %f' % (precision,),
			r'Recall = %f' % (recall,),
			r'F-score = %f' % (f_score,)))
		details = '\n'.join((
			r'# of matched samples = %i' % (match,),
			r'# of smaples on GT = %i' % (match+len(unmatched1),),
			r'# of samples on RCM = %i' % (match+len(unmatched2),)))

		# Creating a message box for the scores
		def button_clicked():
			msg = QtWidgets.QMessageBox(self)
			msg.setWindowTitle("Scores")
			msg.setText(textstr)
			msg.setDetailedText(details)
			msg.exec_()

		def save():
			canvas.figure.savefig(data+" - "+algo+" ("+eval.split("_")[-1]+").svg", format="svg")

		info = QtWidgets.QPushButton("Scores", )
		info.clicked.connect(button_clicked)
		save_vsg = QtWidgets.QPushButton("Save as SVG", )
		save_vsg.clicked.connect(save)

		# Creating toolbar, passing canvas as first param, parent (self, the MainWindow) as second.
		toolbar = NavigationToolbar(canvas, self)
		toolbar.addWidget(info)
		toolbar.addWidget(save_vsg)

		layout = QtWidgets.QVBoxLayout()
		layout.addWidget(toolbar)
		layout.addWidget(canvas)

		# Creating a placeholder widget to hold our toolbar and canvas.
		#widget = QtWidgets.QWidget()
		#widget.setLayout(layout)
		self.setLayout(layout)
		#self.setCentralWidget(widget)
		self.setWindowTitle("The Visualizer - "+data+" - "+algo+" ("+eval.split("_")[-1]+")")
		#self.show()


app = QtWidgets.QApplication(sys.argv)
w = MainWindow()
w.show()
app.exec_()
