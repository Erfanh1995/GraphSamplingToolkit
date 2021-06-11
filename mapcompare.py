#
# Implementation of MapCompare algorithm.
# Author: James P. Biagioni (jbiagi1@uic.edu)
# Company: University of Illinois at Chicago
# Created: 6/7/11
#
# Author: Mahmuda Ahmed
# Company: The University of Texas at San Antonio
# Modified: 2013
#
# Author: Jorren Hendriks (j.a.m.hendriks@student.tue.nl)
# Institute: Eindhoven University of Technology  (TU/e)
# Modified: 4/30/2020
#
# Author: Erfan Hosseini Sereshgi (shosseinisereshgi@tulane.edu)
# Company: Tulane University
# Modified: 6/10/2021
#
# Author: Jordi Aguilar Larruy (jordi.aguilar.larruy@estudiantat.upc.edu)
# Company: Universitat Politècnica de Catalunya
# Modified: 5/31/2021
#
import math
import os
import random
import csv
import sys
import getopt
import time
from streetmap import StreetMap
import spatialfunclib
from rtree import Rtree
import networkx as nx

# global parameters
breadcrumb_interval = 5.0  # meters
breadcrumb_max_distance = float('infinity')  # meters
match_distance_threshold = 15.0  # meters
debug_mode = False
bearing_limit = 45.0  # degrees
compare_mode = "knn"


class Breadcrumb:
    def __init__(self, id, latitude, longitude, bearing):
        self.id = id
        self.latitude = latitude
        self.longitude = longitude
        self.bearing = bearing


class BreadcrumbPath:
    def __init__(self):
        self.breadcrumbs = {}
        self.breadcrumb_index = Rtree()

    def add_breadcrumb(self, breadcrumb):
        self.breadcrumbs[breadcrumb.id] = breadcrumb
        self.breadcrumb_index.insert(breadcrumb.id, (breadcrumb.longitude, breadcrumb.latitude))

    def delete_breadcrumb(self, breadcrumb):
        del self.breadcrumbs[breadcrumb.id]
        self.breadcrumb_index.delete(breadcrumb.id, [breadcrumb.longitude, breadcrumb.latitude, breadcrumb.longitude, breadcrumb.latitude])


class MatchedCrumbs:
    def __init__(self, map1_breadcrumb, map2_breadcrumb, distance):
        self.map1_breadcrumb = map1_breadcrumb
        self.map2_breadcrumb = map2_breadcrumb
        self.distance = distance


class MapCompare:
    def __init__(self):
        self.map1 = None
        self.map2 = None

    def load_maps(self, map1_filename, map2_filename):
        map1_loaded = False
        map2_loaded = False

        # create new StreetMap object
        self.map1 = StreetMap()

        # load map1 OSMDB
        if self.map1.load_textdb_osm(map1_filename):
            map1_loaded = True

        # create new StreetMap object
        self.map2 = StreetMap()

        # load map2 GraphDB
        if self.map2.load_textdb_algo(map2_filename):
            map2_loaded = True

        return map1_loaded, map2_loaded

    def generate_root_locations(self, root_locations_filename):

        print("generating root locations...")
        # output newline for formatting
        sys.stdout.write("\n")
        sys.stdout.flush()

        # open evaluation output file
        root_locations_file = open_mkdir(root_locations_filename, 'w')

        # select root location from map1 and map2
        root_location1 = self._find_components(self.map1)
        root_location2 = self._find_components(self.map2)

        #root_location1 = root_location1 + root_location2
        #for rl in range(len(root_location1)):
        #    root_locations_file.write(str(rl) + "," + str(root_location1[rl][0]) + "," + str(root_location1[rl][1]) + "\n")

        for rl in range(len(root_location1)):
            root_locations_file.write("1," + str(rl) + "," + str(root_location1[rl][0]) + "," + str(root_location1[rl][1]) + "\n")
        for rl in range(len(root_location2)):
            root_locations_file.write("2," + str(rl) + "," + str(root_location2[rl][0]) + "," + str(root_location2[rl][1]) + "\n")

        root_locations_file.flush()
        root_locations_file.close()
        print("done.")

    def compare_maps(self, eval_filename, root_locations_filename):

        # if there are no nodes or edges in map2
        if ((len(self.map2.nodes) == 0) or (len(self.map2.edges) == 0)):
            # open evaluation output file
            eval_file = open(eval_filename, 'w')

            # write nothing into evaluation output file
            eval_file.write("")

            # close evaluation output file
            eval_file.close()

            # terminate comparison
            return

        # output newline for formatting
        sys.stdout.write("\n")
        sys.stdout.flush()

        # output start of map comparison
        sys.stdout.write("\nComparing maps...\n")
        sys.stdout.flush()

        print("reading root locations...")
        rootlocations1 = {}
        rootlocations2 = {}
        # open evaluation output file
        with open_mkdir(root_locations_filename, 'r') as f:
            c = csv.reader(f, delimiter=',', skipinitialspace=True)
            # iterate through all query results
            for map, id, lat, lon in c:
                if int(map) == 1:
                    rootlocations1[int(id)] = (float(lat), float(lon))
                if int(map) == 2:
                    rootlocations2[int(id)] = (float(lat), float(lon))
        # clear evaluation output file
        open_mkdir(eval_filename, 'w').close()
        print("Seeds on map1: "+str(len(rootlocations1)))
        print("Seeds on map2: "+str(len(rootlocations2)))

        # get breadcrumb edges of both maps
        print("Getting breadcrumb edges")
        map1_breadcrumb_edges = []
        map2_breadcrumb_edges = []
        for i in rootlocations1.values():
            map1_breadcrumb_edges += self._get_breadcrumb_edges(self.map1, i, match_distance_threshold)
        for i in rootlocations2.values():
            map2_breadcrumb_edges += self._get_breadcrumb_edges(self.map2, i, match_distance_threshold)

        print("Getting breadcrumb paths")
        # get breadcrumb paths of both maps

        map1_breadcrumb_path = self._get_breadcrumb_path(self.map1, rootlocations1[0], map1_breadcrumb_edges, None, None)
        map2_breadcrumb_path = self._get_breadcrumb_path(self.map2, rootlocations2[0], map2_breadcrumb_edges,
                                                             None, None)

        if (debug_mode):
            print("root location: " + str(rootlocations1[0][0]) + "," + str(rootlocations1[0][1]))
            self._write_breadcrumb_path_to_file(map1_breadcrumb_path, "map1_breadcrumb_path.txt")
            self._write_breadcrumb_path_to_file(map2_breadcrumb_path, "map2_breadcrumb_path.txt")

        print("Comparing breadcrumb paths using "+compare_mode)
        # compare breadcrumb paths for map1 and map2
        if compare_mode == "knn":
            self._compare_breadcrumb_paths(map1_breadcrumb_path, map2_breadcrumb_path, eval_filename)
        elif compare_mode == "wmm":
            self._weighted_maximum_matching(map1_breadcrumb_path, map2_breadcrumb_path, eval_filename)
        elif compare_mode == "mm":
            self._maximum_matching(map1_breadcrumb_path, map2_breadcrumb_path, eval_filename)
        elif compare_mode == "g":
            self._compare_breadcrumb_paths_greedy(map1_breadcrumb_path, map2_breadcrumb_path, eval_filename)
        elif compare_mode == "gs":
            self._compare_breadcrumb_paths_greedy_shortest(map1_breadcrumb_path, map2_breadcrumb_path, eval_filename)
        elif compare_mode == "gsb":
            self._compare_breadcrumb_paths_greedy_shortest_bearing(map1_breadcrumb_path, map2_breadcrumb_path, eval_filename)
        else:
            print("Not a valid comparing option --- using the default k-nearest neighbors matching algorithm")
            self._compare_breadcrumb_paths(map1_breadcrumb_path, map2_breadcrumb_path, eval_filename)

        # reset edge visited flags in both maps
        self.map1.reset_edge_visited_flags()
        self.map2.reset_edge_visited_flags()

        # finished with current evaluation
        print("'\rEvaluation run " + "... " + "done.", end="")

        # DEBUG -- terminate after one evaluation
        if (debug_mode):
            exit()
        print(flush=True)

    def compute_results(self, eval_file_name):
        counter = {
            "gTot": 0,
            "hTot": 0,
            "pos": 0
        }

        # open evaluation output file
        with open_mkdir(eval_file_name, 'r') as f:
            c = csv.reader(f, delimiter=',', skipinitialspace=True)
            # iterate through all results
            for d, x, y, u, v in c:
                dist = int(float(d))
                if dist == 1000000:
                    counter["hTot"] += 1
                elif dist == 2000000:
                    counter["gTot"] += 1
                else:
                    for k in counter.keys():
                        counter[k] += 1


        m, n, k = counter["gTot"], counter["hTot"], counter["pos"]

        print(("-[ Sample Counts ]" + "-" * 42 + "\n"
                                                 "m = #samples in G | n = #samples in H | k = #matched samples\n"
                                                 "{m:17d} | {n:17d} | {k:20d}\n" + "-" * 60)
              .format(m=m, n=n, k=k))

        print(("-[ Sample Statistics ]" + "-" * 11 + "\n"
                                                     "precision |    recall |   F-score\n"
                                                     "{p:1.7f} | {r:1.7f} | {f:1.7f}\n" + "-" * 33)
              .format(p=k / m, r=k / n, f=2 * k / (n + m)))

    def _find_components(self, curr_map):
        rootlocations = []
        bfs_queue = []
        list_of_edges = list(curr_map.edges.values())
        connected_component_nodes = []
        connected_component = []
        list_of_nodes = list(curr_map.nodes.values())
        while len(list_of_nodes) > 0:
            source_node = list_of_nodes[0]
            if len(source_node.out_nodes) == 0:
                list_of_nodes.remove(source_node)
                continue
            bfs_queue.append(source_node)
            while (len(bfs_queue) > 0):
                curr_node = bfs_queue.pop(0)
                connected_component_nodes.append(curr_node)
                list_of_nodes.remove(curr_node)
                for out_node in curr_node.out_nodes:
                    if out_node not in bfs_queue and out_node in list_of_nodes:
                        bfs_queue.append(out_node)

            for node in connected_component_nodes:
                for edge in list_of_edges:
                    if edge.in_node.id == node.id or edge.out_node.id == node.id:
                        connected_component.append(edge)
                        list_of_edges.remove(edge)
            rootlocations.append(self._select_map_root_location(connected_component))
            connected_component = []
            connected_component_nodes = []
        return rootlocations

    def _maximum_matching(self, map1_breadcrumb_path, map2_breadcrumb_path, eval_file_name):
        # storage for list of matched breadcrumbs
        matched_breadcrumbs = []
        matched = 0
        # DEBUG -- output number of breadcrumbs
        if (debug_mode):
            print("map1 breadcrumbs: " + str(len(map1_breadcrumb_path.breadcrumbs)))
            print("map2 breadcrumbs: " + str(len(map2_breadcrumb_path.breadcrumbs)))

        # append results to eval file
        with open_mkdir(eval_file_name, 'a+') as eval_file:
            graph = nx.Graph()
            graph.add_nodes_from(map1_breadcrumb_path.breadcrumbs)
            graph.add_nodes_from(map2_breadcrumb_path.breadcrumbs)
            for node1 in map1_breadcrumb_path.breadcrumbs.values():
                for node2 in map2_breadcrumb_path.breadcrumbs.values():
                    breadcrumb_distance = self._distance(node1,node2)
                    if breadcrumb_distance <= match_distance_threshold:
                        bearing_difference = spatialfunclib.bearing_difference(node1.bearing,node2.bearing)
                        if bearing_difference < bearing_limit:
                            graph.add_edge(node1,node2)
            print("The graph is loaded - starting the matching algorithm")
            matching = nx.algorithms.matching.max_weight_matching(graph, maxcardinality=True)
            print("Matching is done - writing files")
            for edge in matching:
                breadcrumb_distance = self._distance(edge[0],edge[1])
                if (edge[0] in map1_breadcrumb_path.breadcrumbs.values()) and (edge[1] in map2_breadcrumb_path.breadcrumbs.values()):
                    matched_breadcrumb = MatchedCrumbs(edge[0], edge[1], breadcrumb_distance)
                else:
                    matched_breadcrumb = MatchedCrumbs(edge[1], edge[0], breadcrumb_distance)

                eval_file.write(str(matched_breadcrumb.distance) + "," + str(
                    matched_breadcrumb.map1_breadcrumb.latitude) + "," + str(
                    matched_breadcrumb.map1_breadcrumb.longitude) + "," + str(
                    matched_breadcrumb.map2_breadcrumb.latitude) + "," + str(
                    matched_breadcrumb.map2_breadcrumb.longitude) + "\n")

                matched += 1

                if matched_breadcrumb.map1_breadcrumb in map1_breadcrumb_path.breadcrumbs.values():
                    map1_breadcrumb_path.delete_breadcrumb(matched_breadcrumb.map1_breadcrumb)
                elif matched_breadcrumb.map1_breadcrumb in map2_breadcrumb_path.breadcrumbs.values():
                    map2_breadcrumb_path.delete_breadcrumb(matched_breadcrumb.map1_breadcrumb)
                if matched_breadcrumb.map2_breadcrumb in map2_breadcrumb_path.breadcrumbs.values():
                    map2_breadcrumb_path.delete_breadcrumb(matched_breadcrumb.map2_breadcrumb)
                elif matched_breadcrumb.map2_breadcrumb in map1_breadcrumb_path.breadcrumbs.values():
                    map1_breadcrumb_path.delete_breadcrumb(matched_breadcrumb.map2_breadcrumb)

            for map1_breadcrumb in map1_breadcrumb_path.breadcrumbs.values():
                # output map1 unmatched edge value
                eval_file.write(
                    "1000000" + "," + str(map1_breadcrumb.latitude) + "," + str(map1_breadcrumb.longitude) + "," + str(
                        map1_breadcrumb.latitude) + "," + str(map1_breadcrumb.longitude) + "\n")

            # iterate through remaining map2 breadcrumbs
            for map2_breadcrumb in map2_breadcrumb_path.breadcrumbs.values():
                # output map2 spurious edge value
                eval_file.write(
                    "2000000" + "," + str(map2_breadcrumb.latitude) + "," + str(map2_breadcrumb.longitude) + "," + str(
                        map2_breadcrumb.latitude) + "," + str(map2_breadcrumb.longitude) + "\n")

            eval_file.flush()


    def _weighted_maximum_matching(self, map1_breadcrumb_path, map2_breadcrumb_path, eval_file_name):
        # storage for list of matched breadcrumbs
        matched_breadcrumbs = []
        matched = 0
        # DEBUG -- output number of breadcrumbs
        if (debug_mode):
            print("map1 breadcrumbs: " + str(len(map1_breadcrumb_path.breadcrumbs)))
            print("map2 breadcrumbs: " + str(len(map2_breadcrumb_path.breadcrumbs)))

        # append results to eval file
        with open_mkdir(eval_file_name, 'a+') as eval_file:
            graph = nx.Graph()
            graph.add_nodes_from(map1_breadcrumb_path.breadcrumbs)
            graph.add_nodes_from(map2_breadcrumb_path.breadcrumbs)
            for node1 in map1_breadcrumb_path.breadcrumbs.values():
                for node2 in map2_breadcrumb_path.breadcrumbs.values():
                    breadcrumb_distance = self._distance(node1,node2)
                    if breadcrumb_distance <= match_distance_threshold:
                        bearing_difference = spatialfunclib.bearing_difference(node1.bearing,node2.bearing)
                        if bearing_difference < bearing_limit:
                            graph.add_edge(node1,node2, weight=match_distance_threshold - breadcrumb_distance)

            print("The graph is loaded - starting the matching algorithm")
            matching = nx.algorithms.matching.max_weight_matching(graph)
            print("Matching is done - writing files")
            for edge in matching:
                breadcrumb_distance = self._distance(edge[0],edge[1])
                if (edge[0] in map1_breadcrumb_path.breadcrumbs.values()) and (edge[1] in map2_breadcrumb_path.breadcrumbs.values()):
                    matched_breadcrumb = MatchedCrumbs(edge[0], edge[1], breadcrumb_distance)
                else:
                    matched_breadcrumb = MatchedCrumbs(edge[1], edge[0], breadcrumb_distance)

                eval_file.write(str(matched_breadcrumb.distance) + "," + str(
                    matched_breadcrumb.map1_breadcrumb.latitude) + "," + str(
                    matched_breadcrumb.map1_breadcrumb.longitude) + "," + str(
                    matched_breadcrumb.map2_breadcrumb.latitude) + "," + str(
                    matched_breadcrumb.map2_breadcrumb.longitude) + "\n")

                matched += 1

                if matched_breadcrumb.map1_breadcrumb in map1_breadcrumb_path.breadcrumbs.values():
                    map1_breadcrumb_path.delete_breadcrumb(matched_breadcrumb.map1_breadcrumb)
                elif matched_breadcrumb.map1_breadcrumb in map2_breadcrumb_path.breadcrumbs.values():
                    map2_breadcrumb_path.delete_breadcrumb(matched_breadcrumb.map1_breadcrumb)
                if matched_breadcrumb.map2_breadcrumb in map2_breadcrumb_path.breadcrumbs.values():
                    map2_breadcrumb_path.delete_breadcrumb(matched_breadcrumb.map2_breadcrumb)
                elif matched_breadcrumb.map2_breadcrumb in map1_breadcrumb_path.breadcrumbs.values():
                    map1_breadcrumb_path.delete_breadcrumb(matched_breadcrumb.map2_breadcrumb)

            for map1_breadcrumb in map1_breadcrumb_path.breadcrumbs.values():
                # output map1 unmatched edge value
                eval_file.write(
                    "1000000" + "," + str(map1_breadcrumb.latitude) + "," + str(map1_breadcrumb.longitude) + "," + str(
                        map1_breadcrumb.latitude) + "," + str(map1_breadcrumb.longitude) + "\n")

            # iterate through remaining map2 breadcrumbs
            for map2_breadcrumb in map2_breadcrumb_path.breadcrumbs.values():
                # output map2 spurious edge value
                eval_file.write(
                    "2000000" + "," + str(map2_breadcrumb.latitude) + "," + str(map2_breadcrumb.longitude) + "," + str(
                        map2_breadcrumb.latitude) + "," + str(map2_breadcrumb.longitude) + "\n")

            eval_file.flush()



    def _compare_breadcrumb_paths_greedy_shortest_bearing(self, map1_breadcrumb_path, map2_breadcrumb_path, eval_file_name):
        # storage for list of matched breadcrumbs
        matched_breadcrumbs = []
        matched = 0

        # DEBUG -- output number of breadcrumbs
        if (debug_mode):
            print("map1 breadcrumbs: " + str(len(map1_breadcrumb_path.breadcrumbs)))
            print("map2 breadcrumbs: " + str(len(map2_breadcrumb_path.breadcrumbs)))

        # append results to eval file
        with open_mkdir(eval_file_name, 'a+') as eval_file:

            # iterate through map1 breadcrumbs, find nearest neighbor for each, and store in matched_breadcrumbs
            # This computes a many-to-many matching
            bearing_flag = 0
            if (len(map2_breadcrumb_path.breadcrumbs) > 0):
               for map1_breadcrumb in map1_breadcrumb_path.breadcrumbs.values():
                    while bearing_flag >= 0:
                        # find closest breadcrumb in map2. Make sure there are some breadcrumbs left
                        closest_map2_breadcrumb_id = list(map2_breadcrumb_path.breadcrumb_index.nearest((map1_breadcrumb.longitude, map1_breadcrumb.latitude), bearing_flag+1))[bearing_flag]
                        closest_map2_breadcrumb = map2_breadcrumb_path.breadcrumbs[closest_map2_breadcrumb_id]
                        breadcrumb_distance = self._distance(map1_breadcrumb, closest_map2_breadcrumb)
                        bearing_difference = spatialfunclib.bearing_difference(map1_breadcrumb.bearing, closest_map2_breadcrumb.bearing)
                        if bearing_difference < bearing_limit:
                            # create matched crumbs object
                            matched_breadcrumb = MatchedCrumbs(map1_breadcrumb, closest_map2_breadcrumb, breadcrumb_distance)
                            matched_breadcrumbs.append(matched_breadcrumb)
                            bearing_flag = -1
                        else:
                            bearing_flag+=1
                    bearing_flag = 0

            # Now filter this matching to create a 1-to-1 matching, prioritizing shortest distances

            # 1. Sort list of matched breadcrumbs in ascending order according to distance
            matched_breadcrumbs.sort(key=lambda x: x.distance)

            # 2. Continue while there are matched breadcrumbs to be processed
            while (len(matched_breadcrumbs) > 0):
                # if distance between closest matched breadcrumbs is greater than the threshold
                if (matched_breadcrumbs[0].distance > match_distance_threshold):
                    # Exit loop early. All nearest neighbor matches are too far away.
                    break

                # compute bearing difference between breadcrumbs
                #bearing_difference = spatialfunclib.bearing_difference(matched_breadcrumbs[0].map1_breadcrumb.bearing, matched_breadcrumbs[0].map2_breadcrumb.bearing)
                #print(bearing_difference)
                # if map2 breadcrumb is still available
                if ( (matched_breadcrumbs[0].map2_breadcrumb.id in map2_breadcrumb_path.breadcrumbs.keys())):

                    # Fix this matching.

                    # remove map1 breadcrumb from path
                    map1_breadcrumb_path.delete_breadcrumb(matched_breadcrumbs[0].map1_breadcrumb)
                    # remove map2 breadcrumb from path
                    map2_breadcrumb_path.delete_breadcrumb(matched_breadcrumbs[0].map2_breadcrumb)

                    matched += 1
                    # output distance between breadcrumbs for evaluation output file
                    eval_file.write(str(matched_breadcrumbs[0].distance) + "," + str(
                        matched_breadcrumbs[0].map1_breadcrumb.latitude) + "," + str(
                        matched_breadcrumbs[0].map1_breadcrumb.longitude) + "," + str(
                        matched_breadcrumbs[0].map2_breadcrumb.latitude) + "," + str(
                        matched_breadcrumbs[0].map2_breadcrumb.longitude) + "\n")

                    matched_breadcrumbs.pop(0)

                # If map2 breadcrumb is no longer available, find a new nearest neighbor
                else:

                    if (len(map2_breadcrumb_path.breadcrumbs) > 0):
                        bearing_flag = 0
                        while bearing_flag >= 0:
                        # find closest breadcrumb in map2.
                            closest_map2_breadcrumb_id = list(map2_breadcrumb_path.breadcrumb_index.nearest((matched_breadcrumbs[0].map1_breadcrumb.longitude, matched_breadcrumbs[0].map1_breadcrumb.latitude), bearing_flag+1))[bearing_flag]
                            bearing_difference = spatialfunclib.bearing_difference(map1_breadcrumb.bearing, map2_breadcrumb_path.breadcrumbs[closest_map2_breadcrumb_id].bearing)
                            if bearing_difference < bearing_limit:
                                matched_breadcrumbs[0].map2_breadcrumb = map2_breadcrumb_path.breadcrumbs[closest_map2_breadcrumb_id]
                                matched_breadcrumbs[0].distance = self._distance(matched_breadcrumbs[0].map1_breadcrumb, matched_breadcrumbs[0].map2_breadcrumb)
                                # sort list of matched breadcrumbs in ascending order according to distance
                                # Sadly, very inefficient
                                matched_breadcrumbs.sort(key=lambda x: x.distance)
                                bearing_flag = -1
                            else:
                                bearing_flag += 1
                    else:
                        # Can't find a match
                        map1_breadcrumb_path.delete_breadcrumb(map1_breadcrumb)
                        # output map1 unmatched edge value
                        eval_file.write(
                             "1000000" + "," + str(map1_breadcrumb.latitude) + "," + str(map1_breadcrumb.longitude) + "," + str(
                                 map1_breadcrumb.latitude) + "," + str(map1_breadcrumb.longitude) + "\n")

            # iterate through remaining map1 breadcrumbs
            for map1_breadcrumb in map1_breadcrumb_path.breadcrumbs.values():
                # output map1 unmatched edge value
                eval_file.write(
                    "1000000" + "," + str(map1_breadcrumb.latitude) + "," + str(map1_breadcrumb.longitude) + "," + str(
                        map1_breadcrumb.latitude) + "," + str(map1_breadcrumb.longitude) + "\n")
                #plt.scatter(map1_breadcrumb.latitude,map1_breadcrumb.longitude, c='#3FBFBF', s=2)

            # iterate through remaining map2 breadcrumbs
            for map2_breadcrumb in map2_breadcrumb_path.breadcrumbs.values():
                # output map2 spurious edge value
                eval_file.write(
                    "2000000" + "," + str(map2_breadcrumb.latitude) + "," + str(map2_breadcrumb.longitude) + "," + str(
                        map2_breadcrumb.latitude) + "," + str(map2_breadcrumb.longitude) + "\n")
                #plt.scatter(map2_breadcrumb.latitude,map2_breadcrumb.longitude, c='#BF3F3F', s=2)

            #eval_file.write("0,0,0,0,0\n")
            # flush results to file
            eval_file.flush()


    def _compare_breadcrumb_paths_greedy_shortest(self, map1_breadcrumb_path, map2_breadcrumb_path, eval_file_name):
        # storage for list of matched breadcrumbs
        matched_breadcrumbs = []
        matched = 0

        # DEBUG -- output number of breadcrumbs
        if (debug_mode):
            print("map1 breadcrumbs: " + str(len(map1_breadcrumb_path.breadcrumbs)))
            print("map2 breadcrumbs: " + str(len(map2_breadcrumb_path.breadcrumbs)))

        # append results to eval file
        with open_mkdir(eval_file_name, 'a+') as eval_file:

            # iterate through map1 breadcrumbs, find nearest neighbor for each, and store in matched_breadcrumbs
            # This computes a many-to-many matching
            if (len(map2_breadcrumb_path.breadcrumbs) > 0):
               for map1_breadcrumb in map1_breadcrumb_path.breadcrumbs.values():
                    # find closest breadcrumb in map2. Make sure there are some breadcrumbs left
                    closest_map2_breadcrumb_id = list(map2_breadcrumb_path.breadcrumb_index.nearest((map1_breadcrumb.longitude, map1_breadcrumb.latitude), 1))[0]
                    closest_map2_breadcrumb = map2_breadcrumb_path.breadcrumbs[closest_map2_breadcrumb_id]
                    breadcrumb_distance = self._distance(map1_breadcrumb, closest_map2_breadcrumb)
                    # create matched crumbs object
                    matched_breadcrumb = MatchedCrumbs(map1_breadcrumb, closest_map2_breadcrumb, breadcrumb_distance)
                    matched_breadcrumbs.append(matched_breadcrumb)

            # Now filter this matching to create a 1-to-1 matching, prioritizing shortest distances

            # 1. Sort list of matched breadcrumbs in ascending order according to distance
            matched_breadcrumbs.sort(key=lambda x: x.distance)

            # 2. Continue while there are matched breadcrumbs to be processed
            while (len(matched_breadcrumbs) > 0):
                # if distance between closest matched breadcrumbs is greater than the threshold
                if (matched_breadcrumbs[0].distance > match_distance_threshold):
                    # Exit loop early. All nearest neighbor matches are too far away.
                    break

                # if map2 breadcrumb is still available
                if (matched_breadcrumbs[0].map2_breadcrumb.id in map2_breadcrumb_path.breadcrumbs.keys()):

                    # Fix this matching.

                    # remove map1 breadcrumb from path
                    map1_breadcrumb_path.delete_breadcrumb(matched_breadcrumbs[0].map1_breadcrumb)
                    # remove map2 breadcrumb from path
                    map2_breadcrumb_path.delete_breadcrumb(matched_breadcrumbs[0].map2_breadcrumb)

                    matched += 1
                    # output distance between breadcrumbs for evaluation output file
                    eval_file.write(str(matched_breadcrumbs[0].distance) + "," + str(
                        matched_breadcrumbs[0].map1_breadcrumb.latitude) + "," + str(
                        matched_breadcrumbs[0].map1_breadcrumb.longitude) + "," + str(
                        matched_breadcrumbs[0].map2_breadcrumb.latitude) + "," + str(
                        matched_breadcrumbs[0].map2_breadcrumb.longitude) + "\n")

                    matched_breadcrumbs.pop(0)

                # If map2 breadcrumb is no longer available, find a new nearest neighbor
                else:

                    if (len(map2_breadcrumb_path.breadcrumbs) > 0):
                        # find closest breadcrumb in map2.
                        closest_map2_breadcrumb_id = list(map2_breadcrumb_path.breadcrumb_index.nearest((matched_breadcrumbs[0].map1_breadcrumb.longitude, matched_breadcrumbs[0].map1_breadcrumb.latitude), 1))[0]
                        matched_breadcrumbs[0].map2_breadcrumb = map2_breadcrumb_path.breadcrumbs[closest_map2_breadcrumb_id]
                        matched_breadcrumbs[0].distance = self._distance(matched_breadcrumbs[0].map1_breadcrumb, matched_breadcrumbs[0].map2_breadcrumb)

                        # sort list of matched breadcrumbs in ascending order according to distance
                        # Sadly, very inefficient
                        matched_breadcrumbs.sort(key=lambda x: x.distance)

                    else:
                        # Can't find a match
                        map1_breadcrumb_path.delete_breadcrumb(map1_breadcrumb)
                        # output map1 unmatched edge value
                        eval_file.write(
                             "1000000" + "," + str(map1_breadcrumb.latitude) + "," + str(map1_breadcrumb.longitude) + "," + str(
                                 map1_breadcrumb.latitude) + "," + str(map1_breadcrumb.longitude) + "\n")

            # iterate through remaining map1 breadcrumbs
            for map1_breadcrumb in map1_breadcrumb_path.breadcrumbs.values():
                # output map1 unmatched edge value
                eval_file.write(
                    "1000000" + "," + str(map1_breadcrumb.latitude) + "," + str(map1_breadcrumb.longitude) + "," + str(
                        map1_breadcrumb.latitude) + "," + str(map1_breadcrumb.longitude) + "\n")
                #plt.scatter(map1_breadcrumb.latitude,map1_breadcrumb.longitude, c='#3FBFBF', s=2)

            # iterate through remaining map2 breadcrumbs
            for map2_breadcrumb in map2_breadcrumb_path.breadcrumbs.values():
                # output map2 spurious edge value
                eval_file.write(
                    "2000000" + "," + str(map2_breadcrumb.latitude) + "," + str(map2_breadcrumb.longitude) + "," + str(
                        map2_breadcrumb.latitude) + "," + str(map2_breadcrumb.longitude) + "\n")
                #plt.scatter(map2_breadcrumb.latitude,map2_breadcrumb.longitude, c='#BF3F3F', s=2)

            #eval_file.write("0,0,0,0,0\n")
            # flush results to file
            eval_file.flush()


    def _compare_breadcrumb_paths_greedy(self, map1_breadcrumb_path, map2_breadcrumb_path, eval_file_name):
        # storage for list of matched breadcrumbs
        matched_breadcrumbs = []
        matched = 0

        # DEBUG -- output number of breadcrumbs
        if (debug_mode):
            print("map1 breadcrumbs: " + str(len(map1_breadcrumb_path.breadcrumbs)))
            print("map2 breadcrumbs: " + str(len(map2_breadcrumb_path.breadcrumbs)))

        # append results to eval file
        with open_mkdir(eval_file_name, 'a+') as eval_file:

            # iterate through map1 breadcrumbs
            map1_breadcrumb = None
            while len(map1_breadcrumb_path.breadcrumbs.values())>0:
                map1_breadcrumb = list(map1_breadcrumb_path.breadcrumbs.items())[0][1]


                # Are there breadcrumbs left to map to?
                if(len(map2_breadcrumb_path.breadcrumbs) == 0):
                    break

                # find closest breadcrumb in map2. Make sure there are some breadcrumbs left
                closest_map2_breadcrumb_id = list(map2_breadcrumb_path.breadcrumb_index.nearest((map1_breadcrumb.longitude, map1_breadcrumb.latitude), 1))[0]
                closest_map2_breadcrumb = map2_breadcrumb_path.breadcrumbs[closest_map2_breadcrumb_id]
                breadcrumb_distance = self._distance(map1_breadcrumb, closest_map2_breadcrumb)

                # Nearest neighbor is too far away. Can't find a match
                if breadcrumb_distance > match_distance_threshold:
                    map1_breadcrumb_path.delete_breadcrumb(map1_breadcrumb)
                    # output map1 unmatched edge value
                    eval_file.write(
                        "1000000" + "," + str(map1_breadcrumb.latitude) + "," + str(map1_breadcrumb.longitude) + "," + str(
                        map1_breadcrumb.latitude) + "," + str(map1_breadcrumb.longitude) + "\n")
                else:
                    # create matched crumbs object
                    # In this method, matched_breadcrumbs only contains one-to-one matched breadcrumbs
                    matched_breadcrumb = MatchedCrumbs(map1_breadcrumb, closest_map2_breadcrumb, breadcrumb_distance)
                    matched_breadcrumbs.append(matched_breadcrumb)
                    map1_breadcrumb_path.delete_breadcrumb(map1_breadcrumb)
                    map2_breadcrumb_path.delete_breadcrumb(closest_map2_breadcrumb)
                    matched += 1

                    # output distance between breadcrumbs for evaluation output file
                    eval_file.write(str(matched_breadcrumb.distance) + "," + str(
                        matched_breadcrumb.map1_breadcrumb.latitude) + "," + str(
                        matched_breadcrumb.map1_breadcrumb.longitude) + "," + str(
                        matched_breadcrumb.map2_breadcrumb.latitude) + "," + str(
                        matched_breadcrumb.map2_breadcrumb.longitude) + "\n")


            # iterate through remaining map1 breadcrumbs
            for map1_breadcrumb in map1_breadcrumb_path.breadcrumbs.values():
                # output map1 unmatched edge value
                eval_file.write(
                    "1000000" + "," + str(map1_breadcrumb.latitude) + "," + str(map1_breadcrumb.longitude) + "," + str(
                        map1_breadcrumb.latitude) + "," + str(map1_breadcrumb.longitude) + "\n")
                #plt.scatter(map1_breadcrumb.latitude,map1_breadcrumb.longitude, c='#3FBFBF', s=2)

            # iterate through remaining map2 breadcrumbs
            for map2_breadcrumb in map2_breadcrumb_path.breadcrumbs.values():
                # output map2 spurious edge value
                eval_file.write(
                    "2000000" + "," + str(map2_breadcrumb.latitude) + "," + str(map2_breadcrumb.longitude) + "," + str(
                        map2_breadcrumb.latitude) + "," + str(map2_breadcrumb.longitude) + "\n")
                #plt.scatter(map2_breadcrumb.latitude,map2_breadcrumb.longitude, c='#BF3F3F', s=2)

            #eval_file.write("0,0,0,0,0\n")
            # flush results to file
            eval_file.flush()



    def _compare_breadcrumb_paths(self, map1_breadcrumb_path, map2_breadcrumb_path, eval_file_name):

        # storage for list of matched breadcrumbs
        matched_breadcrumbs = []
        matched = 0

        # DEBUG -- output number of breadcrumbs
        if (debug_mode):
            print("map1 breadcrumbs: " + str(len(map1_breadcrumb_path.breadcrumbs)))
            print("map2 breadcrumbs: " + str(len(map2_breadcrumb_path.breadcrumbs)))

        # append results to eval file
        with open_mkdir(eval_file_name, 'a+') as eval_file:

            # if there are some map2 breadcrumbs
            if (len(map2_breadcrumb_path.breadcrumbs) > 0):

                # iterate through map1 breadcrumbs
                for map1_breadcrumb in map1_breadcrumb_path.breadcrumbs.values():

                    # find closest breadcrumb in map2
                    # closest_map2_breadcrumb = map2_breadcrumb_path.breadcrumbs[list(map2_breadcrumb_path.breadcrumb_index.nearest((map1_breadcrumb.longitude, map1_breadcrumb.latitude), 1))[0]]

                    # find closest breadcrumb ids in map2
                    closest_map2_breadcrumb_ids = map2_breadcrumb_path.breadcrumb_index.nearest(
                        (map1_breadcrumb.longitude, map1_breadcrumb.latitude), 10)

                    # iterate through all breadcrumb ids
                    for map2_breadcrumb_id in closest_map2_breadcrumb_ids:

                        # grab map2 breadcrumb
                        map2_breadcrumb = map2_breadcrumb_path.breadcrumbs[map2_breadcrumb_id]

                        # compute bearing difference between breadcrumbs
                        bearing_difference = spatialfunclib.bearing_difference(map1_breadcrumb.bearing,
                                                                               map2_breadcrumb.bearing)

                        # if bearing difference is less than 45 degrees
                        if (bearing_difference < bearing_limit):
                            # find distance to breadcrumb in map2
                            map2_breadcrumb_distance = self._distance(map1_breadcrumb, map2_breadcrumb)

                            # create matched crumbs object
                            matched_breadcrumbs.append(
                                MatchedCrumbs(map1_breadcrumb, map2_breadcrumb, map2_breadcrumb_distance))

                            # exit loop
                            break
                        else:
                            continue

            # sort list of matched breadcrumbs in ascending order according to distance
            matched_breadcrumbs.sort(key=lambda x: x.distance)

            # continue while there are matched breadcrumbs to be processed
            while (len(matched_breadcrumbs) > 0):

                # if distance between closest matched breadcrumbs is greater than the threshold
                if (matched_breadcrumbs[0].distance > match_distance_threshold):
                    # exit loop early
                    break

                # if map2 breadcrumb is still available
                if (matched_breadcrumbs[0].map2_breadcrumb.id in map2_breadcrumb_path.breadcrumbs.keys()):

                    # remove map1 breadcrumb from path
                    del map1_breadcrumb_path.breadcrumbs[matched_breadcrumbs[0].map1_breadcrumb.id]

                    # remove map2 breadcrumb from path
                    del map2_breadcrumb_path.breadcrumbs[matched_breadcrumbs[0].map2_breadcrumb.id]

                    matched += 1

                    # output distance between breadcrumbs for evaluation output file
                    eval_file.write(str(matched_breadcrumbs[0].distance) + "," + str(
                        matched_breadcrumbs[0].map1_breadcrumb.latitude) + "," + str(
                        matched_breadcrumbs[0].map1_breadcrumb.longitude) + "," + str(
                        matched_breadcrumbs[0].map2_breadcrumb.latitude) + "," + str(
                        matched_breadcrumbs[0].map2_breadcrumb.longitude) + "\n")
                    #plt.scatter(matched_breadcrumbs[0].map1_breadcrumb.latitude,matched_breadcrumbs[0].map1_breadcrumb.longitude, c='blue', s=2)
                    #plt.scatter(matched_breadcrumbs[0].map2_breadcrumb.latitude,matched_breadcrumbs[0].map2_breadcrumb.longitude, c='red', s=2)
                    #plt.plot([matched_breadcrumbs[0].map1_breadcrumb.latitude,matched_breadcrumbs[0].map2_breadcrumb.latitude],[matched_breadcrumbs[0].map1_breadcrumb.longitude,matched_breadcrumbs[0].map2_breadcrumb.longitude], c='green')


                    # remove matched breadcrumbs from list
                    matched_breadcrumbs.pop(0)

                # else, if map2 breadcrumb is no longer available
                else:

                    # set map2 matched breadcrumb to None
                    matched_breadcrumbs[0].map2_breadcrumb = None

                    # get list of all_closest breadcrumbs in map2
                    all_closest_map2_breadcrumbs = list(map2_breadcrumb_path.breadcrumb_index.nearest(
                        (matched_breadcrumbs[0].map1_breadcrumb.longitude,
                         matched_breadcrumbs[0].map1_breadcrumb.latitude),
                        10))

                    # iterate through list of all closest breadcrumbs in map2
                    for map2_breadcrumb_id in all_closest_map2_breadcrumbs:

                        # if map2 breadcrumb is still available
                        if (map2_breadcrumb_id in map2_breadcrumb_path.breadcrumbs.keys()):

                            # grab map2 breadcrumb
                            map2_breadcrumb = map2_breadcrumb_path.breadcrumbs[map2_breadcrumb_id]

                            # compute bearing difference between breadcrumbs
                            bearing_difference = spatialfunclib.bearing_difference(
                                matched_breadcrumbs[0].map1_breadcrumb.bearing, map2_breadcrumb.bearing)

                            # if bearing difference is less than 45 degrees
                            if (bearing_difference < bearing_limit):
                                # update map2 breadcrumb in matched crumbs object
                                matched_breadcrumbs[0].map2_breadcrumb = map2_breadcrumb

                                # update distance in matched crumbs object
                                matched_breadcrumbs[0].distance = self._distance(matched_breadcrumbs[0].map1_breadcrumb,
                                                                                 matched_breadcrumbs[0].map2_breadcrumb)

                                # break out of loop
                                break

                    # if there were no map2 breadcrumbs to match with
                    if (matched_breadcrumbs[0].map2_breadcrumb is None):

                        # remove matched breadcrumbs from list
                        matched_breadcrumbs.pop(0)

                    # otherwise, ...
                    else:

                        # sort list of matched breadcrumbs
                        matched_breadcrumbs.sort(key=lambda x: x.distance)

            # iterate through remaining map1 breadcrumbs
            for map1_breadcrumb in map1_breadcrumb_path.breadcrumbs.values():
                # output map1 unmatched edge value
                eval_file.write(
                    "1000000" + "," + str(map1_breadcrumb.latitude) + "," + str(map1_breadcrumb.longitude) + "," + str(
                        map1_breadcrumb.latitude) + "," + str(map1_breadcrumb.longitude) + "\n")
                #plt.scatter(map1_breadcrumb.latitude,map1_breadcrumb.longitude, c='#3FBFBF', s=2)

            # iterate through remaining map2 breadcrumbs
            for map2_breadcrumb in map2_breadcrumb_path.breadcrumbs.values():
                # output map2 spurious edge value
                eval_file.write(
                    "2000000" + "," + str(map2_breadcrumb.latitude) + "," + str(map2_breadcrumb.longitude) + "," + str(
                        map2_breadcrumb.latitude) + "," + str(map2_breadcrumb.longitude) + "\n")
                #plt.scatter(map2_breadcrumb.latitude,map2_breadcrumb.longitude, c='#BF3F3F', s=2)

            #eval_file.write("0,0,0,0,0\n")
            # flush results to file
            eval_file.flush()


    def _get_breadcrumb_path(self, curr_map, root_location, breadcrumb_edges, valid_edges, valid_edge_pairs):

        # iterate through all edges in the current map
        for edge in curr_map.edges.values():
            # intialize breadcrumb storage for edge
            edge.breadcrumbs = []

        # create storage for breadcrumb path
        breadcrumb_path = BreadcrumbPath()

        # initialize breadcrumb id
        breadcrumb_id = 0

        # iterate through all breadcrumb edges
        for breadcrumb_edge in breadcrumb_edges:

            # if valid edges list is supplied, and the current breadcrumb edge is not on it
            if ((valid_edges is not None) and (breadcrumb_edge[0] not in valid_edges)):
                continue

            # create breadcrumb path starting with current breadcrumb edge
            (breadcrumb_path, breadcrumb_id) = self._create_breadcrumb_path(curr_map, root_location, breadcrumb_edge[0],
                                                                            breadcrumb_edge[1], valid_edge_pairs,
                                                                            breadcrumb_path, breadcrumb_id)

        # return breadcrumb path
        return breadcrumb_path

    def _create_breadcrumb_path(self, curr_map, root_location, source_edge, source_location, valid_edge_pairs, breadcrumb_path, breadcrumb_id):

        # initialize start distance for breadcrumbing
        #source_edge.start_distance = self._distance_coords(source_edge.in_node.latitude, source_edge.in_node.longitude,source_location[0], source_location[1])
        distance_to_source_location = self._distance_coords(source_edge.in_node.latitude, source_edge.in_node.longitude, source_location[0], source_location[1])
        if(distance_to_source_location > breadcrumb_max_distance):
           source_edge.start_distance = distance_to_source_location - breadcrumb_max_distance
        else:
           source_edge.start_distance = 0
        #source_edge.start_distance = 0
        # initialize maximum distance for breadcrumbing
        source_edge.max_distance = 0.0

        # create breadth-first edge queue for searching map
        bfs_queue = []

        # enqueue source edge
        bfs_queue.append(source_edge)

        # append edges adjacent to the beginning of the source edge
        reciprocal_edge = curr_map.edge_lookup_table[(source_edge.out_node, source_edge.in_node)]
        dist_from_root = self._distance_coords(root_location[0], root_location[1], reciprocal_edge.out_node.latitude, reciprocal_edge.out_node.longitude)
        if dist_from_root < breadcrumb_max_distance:
            for out_edge in self._find_valid_out_edges(curr_map, reciprocal_edge):
                out_edge.start_distance = breadcrumb_interval
                out_edge.max_distance = 0.0
                bfs_queue.append(out_edge)

        # while the edge queue is not empty
        while (len(bfs_queue) > 0):

            # dequeue first edge in the queue
            curr_edge = bfs_queue.pop(0)

            # if the current edge has no length
            if (curr_edge.length == 0.0):
                if ((valid_edge_pairs is not None) and (curr_edge in valid_edge_pairs)):
                    for out_edge in valid_edge_pairs[curr_edge]:
                        if (out_edge not in bfs_queue):
                            out_edge.start_distance = curr_edge.start_distance
                            out_edge.max_distance = curr_edge.max_distance
                            bfs_queue.append(out_edge)
                else:
                    for out_edge in self._find_valid_out_edges(curr_map, curr_edge):
                        if (out_edge not in bfs_queue):
                            out_edge.start_distance = curr_edge.start_distance
                            out_edge.max_distance = curr_edge.max_distance
                            bfs_queue.append(out_edge)

                # continue to next edge in queue
                continue

            # determine breadcrumb start location
            #breadcrumb_start_location = self._point_along_line(curr_edge.in_node, curr_edge.out_node,(curr_edge.start_distance / curr_edge.length))

            # see if there are any breadcrumbs nearby on the current edge
            #closest_breadcrumb_distance = self._find_closest_breadcrumb_distance(curr_edge, breadcrumb_start_location)

            # if the closest breadcrumb is further than (breadcrumb_interval + 1.0 meters) away
            #if (closest_breadcrumb_distance > (breadcrumb_interval + 1.0)):
            #if (len(curr_edge.breadcrumbs) == 0):
            if (not curr_edge.visited):
                curr_edge.visited = True
                # grab reciprocal edge
                reciprocal_edge = curr_map.edge_lookup_table[(curr_edge.out_node, curr_edge.in_node)]
                reciprocal_edge.visited = True

                # drop breadcrumbs along current edge
                (breadcrumb_id, breadcrumb_start_distance,
                 breadcrumb_max_distance_from_source) = self._drop_breadcrumbs_along_edge(curr_edge, breadcrumb_path,
                                                                                          breadcrumb_id, root_location)

                # if we should breadcrumb the successor edges
                if (breadcrumb_start_distance is not None):
                    if ((valid_edge_pairs is not None) and (curr_edge in valid_edge_pairs)):
                        for out_edge in valid_edge_pairs[curr_edge]:
                            if (out_edge not in bfs_queue):
                                out_edge.start_distance = breadcrumb_start_distance
                                out_edge.max_distance = breadcrumb_max_distance_from_source
                                bfs_queue.append(out_edge)
                    else:
                        for out_edge in self._find_valid_out_edges(curr_map, curr_edge):
                            if (out_edge not in bfs_queue):
                                out_edge.start_distance = breadcrumb_start_distance
                                out_edge.max_distance = breadcrumb_max_distance_from_source
                                bfs_queue.append(out_edge)

        # return breadcrumb path and id
        return (breadcrumb_path, breadcrumb_id)

    def _find_closest_breadcrumb_distance(self, edge, location):

        # storage for closest breadcrumb
        closest_breadcrumb_distance = float('infinity')

        # if the edge has breadcrumbs
        if (len(edge.breadcrumbs) > 0):

            # iterate through all breadcrumbs
            for curr_breadcrumb in edge.breadcrumbs:

                # compute distance to current breadcrumb
                curr_breadcrumb_distance = self._distance_coords(location[0], location[1], curr_breadcrumb.latitude,
                                                                 curr_breadcrumb.longitude)

                # if the current breadcrumb is closer than any seen before
                if (curr_breadcrumb_distance < closest_breadcrumb_distance):
                    # store current breadcrumb as closest
                    closest_breadcrumb_distance = curr_breadcrumb_distance

        # return closest breadcrumb distance
        return closest_breadcrumb_distance

    def _drop_breadcrumbs_along_edge(self, curr_edge, breadcrumb_path, breadcrumb_id, root_location):

        # initialize current distance along edge
        curr_distance_along_edge = curr_edge.start_distance

        # initialize maximum breadcrumb distance from root location
        breadcrumb_max_distance_from_root = curr_edge.max_distance

        # initialize previous breadcrumb distance. This will tell us if we are approaching the "allowed" circle.
        previous_breadcrumb_distance_from_root = float('infinity')

        # drop breadcrumbs along current edge
        while (curr_distance_along_edge <= curr_edge.length):

            # determine point along current edge to drop breadcrumb
            breadcrumb_location = self._point_along_line(curr_edge.in_node, curr_edge.out_node,
                                                         (curr_distance_along_edge / curr_edge.length))

            # determine breadcrumb distance from root location
            breadcrumb_distance_from_root = self._distance_coords(root_location[0], root_location[1],
                                                                  breadcrumb_location[0], breadcrumb_location[1])

            # if the current breadcrumb distance exceeds the limit, or is closer to the root than the furthest placed breadcrumb
            if ((breadcrumb_distance_from_root > breadcrumb_max_distance) and (previous_breadcrumb_distance_from_root < breadcrumb_distance_from_root)):
                # return updated breadcrumb id and None start and maximum distance for child edges
                return (breadcrumb_id, None, None)

            # if the current breadcrumb distance exceeds the limit BUT we are getting closer to the allowed circle by the root location
            elif ((breadcrumb_distance_from_root > breadcrumb_max_distance) and (previous_breadcrumb_distance_from_root > breadcrumb_distance_from_root)):
                # do not place breadcrumb but increase the curr_distance_along_edge. We are getting closer to the "allowed" area and we may eventually drop one!

                # update previous_breadcrumb_distance_from_root
                previous_breadcrumb_distance_from_root = breadcrumb_distance_from_root

                # increment current distance along edge
                curr_distance_along_edge += breadcrumb_interval

                continue

            # create new breadcrumb
            new_breadcrumb = Breadcrumb(breadcrumb_id, breadcrumb_location[0], breadcrumb_location[1],
                                        curr_edge.bearing)

            # increment breadcrumb id
            breadcrumb_id += 1

            # add breadcrumb to path
            breadcrumb_path.add_breadcrumb(new_breadcrumb)

            # add breadcrumb to current edge
            curr_edge.breadcrumbs.append(new_breadcrumb)

            # increment current distance along edge
            curr_distance_along_edge += breadcrumb_interval

            # if the current breadcrumb distance from source location is greater than the maximum seen so far
            if (breadcrumb_distance_from_root > breadcrumb_max_distance_from_root):
                # update maximum breadcrumb distance from source location
                breadcrumb_max_distance_from_root = breadcrumb_distance_from_root

        # compute breadcrumb start distance for next edge
        breadcrumb_start_distance = (curr_distance_along_edge - curr_edge.length)

        # return updated breadcrumb id, start and maximum distance
        return (breadcrumb_id, breadcrumb_start_distance, breadcrumb_max_distance_from_root)

    def _find_valid_out_edges(self, curr_map, curr_edge):

        # storage for valid out edges
        valid_out_edges = curr_edge.out_edges

        # if the current edge has a reciprocal edge
        if ((curr_edge.out_node, curr_edge.in_node) in curr_map.edge_lookup_table):

            # grab reciprocal edge
            reciprocal_edge = curr_map.edge_lookup_table[(curr_edge.out_node, curr_edge.in_node)]

            # if the reciprocal edge is in the valid out edges list
            if (reciprocal_edge in valid_out_edges):
                # remove it from the valid out edges list
                valid_out_edges.remove(reciprocal_edge)

        # return valid out edges
        return valid_out_edges


    def _get_breadcrumb_edges(self, curr_map, root_location, match_distance_threshold):

        # storage for breadcrumb edges
        breadcrumb_edges = []

        # iterate through closest map edge ids
        for curr_map_edge_id in curr_map.edge_spatial_index.nearest((root_location[1], root_location[0]), 1):

            # grab current map edge
            curr_map_edge = curr_map.edges[curr_map_edge_id]

            # project root location onto current map edge
            (projected_point, projection_fraction, projection_distance) = self._projection_onto_line_coords(
                curr_map_edge.in_node.latitude, curr_map_edge.in_node.longitude, curr_map_edge.out_node.latitude,
                curr_map_edge.out_node.longitude, root_location[0], root_location[1])

            # if projection is beyond the out node
            if (projection_fraction >= 1.0):

                # set current map location to the edge out node
                curr_map_edge_location = [curr_map_edge.out_node.latitude, curr_map_edge.out_node.longitude]

            # else, if the projection is beyond the in node
            elif (projection_fraction <= 0.0):

                # set current map2 location to the edge in node
                curr_map_edge_location = [curr_map_edge.in_node.latitude, curr_map_edge.in_node.longitude]

            # else, if the projection is onto the line
            else:

                # set current map2 location to projected point
                curr_map_edge_location = [projected_point[0], projected_point[1]]

            # determine distance to current map location
            curr_map_edge_distance = self._distance_coords(root_location[0], root_location[1],
                                                           curr_map_edge_location[0], curr_map_edge_location[1])

            # store current edge and its distance in breadcrumb edges list
            breadcrumb_edges.append((curr_map_edge, curr_map_edge_location, curr_map_edge_distance))

        # filter out breadcrumb edges with distance greater than the match distance threshold
        breadcrumb_edges = list(filter(lambda x: x[2] <= match_distance_threshold, breadcrumb_edges))

        # sort remaining breadcrumb edges by distance
        breadcrumb_edges.sort(key=lambda x: x[2])

        # return sorted breadcrumb edges
        return breadcrumb_edges

    def _select_map_root_location(self, map_valid_edges):

        # get map1 ordered edges and distances
        (map_ordered_edges, map_ordered_edge_distances) = self._get_ordered_edges_and_distances(map_valid_edges)

        # determine maximum distance along ordered edges
        max_distance = map_ordered_edges[-1].end_distance

        # select a distance along ordered edges
        distance_along = (1/2 * max_distance)

        # find closest key in distance lookup table
        closest_key = min((abs(distance_along - i), i) for i in map_ordered_edge_distances)[1]

        # grab candidate edges
        candidate_edges = map_ordered_edge_distances[closest_key]

        # storage for selected map1 edge
        map_edge = None

        # iterate through candidate edges
        for candidate_edge in candidate_edges:

            # if selected distance along falls on the edge
            if ((distance_along >= candidate_edge.start_distance) and (distance_along <= candidate_edge.end_distance)):
                # we have found our map1 edge
                map_edge = candidate_edge

                # and we're done!
                break

        # determine the fraction along the edge
        if map_edge.length == 0:
            fraction_along = 0
        else:
            fraction_along = ((distance_along - map_edge.start_distance) / map_edge.length)

        # determine the location along the edge
        map_location = self._point_along_line(map_edge.in_node, map_edge.out_node, fraction_along)

        # return map1 root location
        return map_location


    def _get_ordered_edges_and_distances(self, valid_edges):

        # list storage for ordered edges
        ordered_edges = []

        # storage for cumulative distance
        cumulative_distance = 0.0

        # iterate through valid edges
        for valid_edge in valid_edges:
            # add start distance attribute to edge
            valid_edge.start_distance = cumulative_distance

            # add edge length to cumulative distance
            cumulative_distance += valid_edge.length

            # add end distance attribute to edge
            valid_edge.end_distance = cumulative_distance

            # add edge to ordered edges list
            ordered_edges.append(valid_edge)

        # storage for ordered edge distances
        ordered_edge_distances = {}

        # iterate through valid edges
        for valid_edge in valid_edges:

            # if start distance bucket does not exist
            if (valid_edge.start_distance not in ordered_edge_distances):
                # create bucket
                ordered_edge_distances[valid_edge.start_distance] = []

            # add edge to bucket
            ordered_edge_distances[valid_edge.start_distance].append(valid_edge)

            # if end distance bucket does not exist
            if (valid_edge.end_distance not in ordered_edge_distances):
                # create bucket
                ordered_edge_distances[valid_edge.end_distance] = []

            # add edge to bucket
            ordered_edge_distances[valid_edge.end_distance].append(valid_edge)

        # return ordered edges and distances
        return (ordered_edges, ordered_edge_distances)

    def _write_breadcrumb_path_to_file(self, breadcrumb_path, breadcrumb_path_filename="breadcrumb_path.txt"):

        # output that we are starting the writing process
        sys.stdout.write("\nWriting breadcrumb path to file " + str(breadcrumb_path_filename) + "... ")
        sys.stdout.flush()

        # open breadcrumb path file
        breadcrumb_path_file = open(breadcrumb_path_filename, 'w')

        # iterate through all breadcrumbs on path
        for curr_breadcrumb in breadcrumb_path.breadcrumbs.values():
            # output current breadcrumb to file
            breadcrumb_path_file.write(str(curr_breadcrumb.latitude) + "," + str(curr_breadcrumb.longitude) + "," + str(
                curr_breadcrumb.id) + "\n")

        # close breadcrumb path file
        breadcrumb_path_file.close()

        print("done.")

    def _projection_onto_line(self, location1, location2, location3):
        return spatialfunclib.projection_onto_line(location1.latitude, location1.longitude, location2.latitude,
                                                   location2.longitude, location3.latitude, location3.longitude)

    def _projection_onto_line_coords(self, location1_latitude, location1_longitude, location2_latitude,
                                     location2_longitude, location3_latitude, location3_longitude):
        return spatialfunclib.projection_onto_line(location1_latitude, location1_longitude, location2_latitude,
                                                   location2_longitude, location3_latitude, location3_longitude)

    def _point_along_line(self, location1, location2, fraction_along):
        return spatialfunclib.point_along_line(location1.latitude, location1.longitude, location2.latitude,
                                               location2.longitude, fraction_along)

    def _path_bearing(self, location1, location2):
        return spatialfunclib.path_bearing(location1.latitude, location1.longitude, location2.latitude,
                                           location2.longitude)

    def _distance(self, location1, location2):
        return spatialfunclib.euclideandistance(location1.latitude, location1.longitude, location2.latitude,
                                                location2.longitude)

    def _distance_coords(self, location1_latitude, location1_longitude, location2_latitude, location2_longitude):
        return spatialfunclib.euclideandistance(location1_latitude, location1_longitude, location2_latitude,
                                                location2_longitude)

    def _fast_distance(self, location1, location2):
        return spatialfunclib.euclideandistance(location1.latitude, location1.longitude, location2.latitude,
                                                location2.longitude)


def open_mkdir(filename, *args, **kwargs):
    """
    Open a file, creating directories if necessary. Wraps the default python 'open' function
    """
    dirname = os.path.dirname(filename)
    if not os.path.exists(dirname):
        os.makedirs(dirname)
    return open(filename, *args, **kwargs)


if __name__ == '__main__':
    help_string = ("Usage: python graphsampling.py <data_folder> <dataset name> <solution name> <mode>"
                   "[-i <breadcrumb_interval>] "
                   "[-b <bearing_limit>] "
                   "[-c <compare_mode>] "
                   "[-t <match_distance_threshold>] "
                   "[-f] [-h]\n")

    if len(sys.argv) < 4:
        print(help_string)
        exit()

    (opts, args) = getopt.getopt(sys.argv[5:], "i:b:t:c:fh")
    force_computation = False

    for o, a in opts:
        if o == "-i":
            breadcrumb_interval = float(a)
        if o == "-b":
            bearing_limit = float(a)
        if o == "-t":
            match_distance_threshold = float(a)
        if o == "-c":
            compare_mode = str(a)
        if o == "-f":
            force_computation = True
        if o == "-h":
            print(help_string)
            exit()

    start_time = time.time()
    data_path = sys.argv[1].strip("/\\")
    dataset = sys.argv[2]
    solution = sys.argv[3]
    compute_mode = sys.argv[4]

    print("[MapCompare] Comparing {} against ground-truth on the {} dataset.\n".format(solution, dataset))

    f_data = "{folder}/{dataset}".format(folder=data_path, dataset=dataset)
    f_ground_truth = "{folder}/groundtruth/{dataset}_{{type}}.txt".format(folder=f_data, dataset=dataset)
    f_dataset = "{folder}/algorithm/{algorithm}/{dataset}_{algorithm}_{{type}}.txt".format(folder=f_data,
                                                                                            dataset=dataset,
                                                                                            algorithm=solution)
    f_results = "{folder}/evals/{dataset}_{{file}}.txt".format(folder=f_data, dataset=dataset)

    m = MapCompare()
    has_ground_truth, has_solution = m.load_maps(f_ground_truth, f_dataset)
    #valid_edges = f_ground_truth.format(type=valid_edge_file)
    #has_ground_truth = has_ground_truth and os.path.isfile(valid_edges)

    if "l" in compute_mode.lower():
        if has_ground_truth:
            if force_computation or input("Please confirm you want to compute and override root locations on {}. "
                                          "(Y/n) ".format(dataset)).lower() == "y":
                m.generate_root_locations(f_results.format(file=solution + "_rootlocations"))
        else:
            print("Tried to generate root locations without complete ground truth", file=sys.stderr, flush=True)
    if "c" in compute_mode.lower():
        if has_ground_truth and has_solution:
            if force_computation or input("Please confirm you want to compare {} to {} and overwrite previous results. "
                                          "(Y/n) ".format(solution, dataset)).lower() == "y":
                root_locations = f_results.format(file=solution + "_rootlocations")
                if os.path.isfile(root_locations):
                    m.compare_maps(f_results.format(file=solution + "_" + compare_mode + "{t:.0f}".format(t=match_distance_threshold)),
                                   root_locations)
                else:
                    print("Tried to compare maps without root location file", file=sys.stderr, flush=True)
        else:
            print("Tried to compare {}ground truth against {}solution\n".format('' if has_ground_truth else 'a missing ', '' if has_solution else 'a missing '), file=sys.stderr, flush=True)
    if "r" in compute_mode.lower():
        m.compute_results(f_results.format(file=solution + "_" + compare_mode + "{t:.0f}".format(t=match_distance_threshold)))

    print("[MapCompare] Operations completed in {sec:.0f} seconds.\n".format(sec=time.time() - start_time))
