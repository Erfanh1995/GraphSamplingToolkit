# Graph Sampling Evaluation

Graph sampling is a method for map comparison. This program takes two graphs as inputs and returns precision, recall and f-score values quantifying their similarities.
![Athens_small OSM vs TeleAtlas](https://github.com/Erfanh1995/GraphSamplingToolkit/blob/main/figs/teaser.jpg)
![Berlin_small OSM vs TeleAtlas](https://github.com/Erfanh1995/GraphSamplingToolkit/blob/main/figs/roadmap.png)

```bash
python3 mapcompare.py <data_folder> <dataset_name> <solution_name> <mode>
```
<data_folder> is the path to the data folder e.g. data

<dataset_name> name of the dataset folder inside <data_folder> e.g. athens_small

<solution_name> name of the reconstructed map in <data_folder>/<dataset_name>/algorithm directory

The <mode> string will be used to determine which computations to do. Each of the following computations corresponds with a letter:
- Computing sampling starting points/finding connected components: L
- Comparing map against ground-truth/map1 against map2: C
- Evaluating results/statistics: R

Each of these steps creates a file in evals directory in <dataset_name> so for instance if you have previously used L on two maps you can skip it in the future computations for said two maps.

# Parameters
Parameters | Note
--------------------- | -------------
[-i <breadcrumb_interval>]  | Default is 5 meters
[-b <bearing_limit>]  | Default is 45 degrees
[-c <compare_mode>] | There are three options: mm, wmm and knn. Default is knn (fast greedy algorithm).
[-t <match_distance_threshold>] | Default is 15 meters
[-f] | force computation

The <compare_mode> parameter will be used to determine which matching algorithm to use: mm (maximum cardinality matching), (wmm) maximum weight matching or (knn) 10-nearest-neighbor greedy matching

All-in-One example:
```bash
python3 mapcompare.py data chicago james LCR -i 10 -b 30 -c wmm -t 10 -f
```

# Dependency
* [Rtree](https://pypi.org/project/Rtree/)
* [Networkx](https://pypi.org/project/networkx/)



# Map Cropper

MapCropper.py follows the same directories as mapcompare.py you can crop ground truth maps located in <dataset_name>/groundtruth using hidden markov matching. The code only asks for <dataset_name> in data/. You will need a trajectories folder in <dataset_name> containing GPS trajectories.
![Chicago OSM cropped (red)](https://github.com/Erfanh1995/GraphSamplingToolkit/blob/main/figs/hmm.png)


```bash
python3 MapCropper.py
```

# Dependency
* [leuvenmapmatching](https://pypi.org/project/leuvenmapmatching/)



# Visualizer

In order to properly run this tool, the file structure should look as it is in the data folder.
This tool can be used to visualize reconstructed maps and their precision/recall evaluations
For the background map/Contextily to work you need a working internet connection.
Visualizer.py follows the same directories as mapcompare.py.
Each <dataset_name> needs a <dataset_name>.yml file containing its corresponding EPSG code in this format: "EPSG:<code>" e.g. EPSG:4326

```bash
python3 Visualizer.py
```

# Dependency
* Dependencies:
* [Fiona](https://pypi.org/project/Fiona/) (geographical operations)
* [Pandas](https://pypi.org/project/pandas/) (data handling)
* [GeoPandas](https://pypi.org/project/geopandas/) (geographical data handling)
* [Shapely](https://pypi.org/project/Shapely/) (geometrical operations)
* [Matplotlib](https://pypi.org/project/matplotlib/) (plotting)
* [PyQt5](https://pypi.org/project/PyQt5/) (user interface)
* [PyYAML](https://pypi.org/project/PyYAML/) (reading the config file)
* [Contextily](https://pypi.org/project/contextily/) (background map)
