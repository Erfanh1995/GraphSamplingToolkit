# Graph Sampling Evaluation

Graph sampling is a method for map comparison. This program can be used to compare two input
graphs globally, using a matching algorithm. The starting point of our implementation was Biagioni's code that he made available to us. Our code consists of 3 steps: generating a first point for the traversal on each connected component of the input graphs and returning a file, sampling all identified connected components and producing an evaluation file containing matched and unmatched samples and
calculating precision, recall and F-score based on the evaluation file. Some options to set the matching algorithm, bearing threshold,
sampling interval and match distance threshold are also available.   

![Berlin_small OSM vs TeleAtlas](https://github.com/Erfanh1995/GraphSamplingToolkit/blob/main/figs/roadmap.png)

```bash
python3 mapcompare.py <data_folder> <dataset_name> <solution_name> <mode>
```   
`<data_folder>` is the path to the data folder e.g. `data`

`<dataset_name>` is the name of the dataset folder inside `<data_folder>` e.g. `athens_small`

`<solution_name>` is the name of the reconstructed map in `<data_folder>/<dataset_name>/algorithm` directory

The `<mode>` string will be used to determine which computations to do. Each of the following computations corresponds with a letter:  
- Computing sampling starting points/finding connected components on the input graphs: `L`
- Comparing reconstructed map against ground-truth/map1 against map2: `C`
- Evaluating and printing results/statistics: `R`

Each of these steps creates a file in `evals` folder in `<dataset_name>` directory so for instance if you have previously used `L` on two maps you can skip it in the future computations for said two maps.   

Your input maps have to be in the same coordinate system. Please take a look at the example input files.  
***input files must be in Lon Lat format***  


# Parameters
Parameters | Note
--------------------- | -------------
[-i <sampling_interval>]  | Default is 5 meters
[-b <bearing_limit>]  | Default is 45 degrees
[-c <compare_mode>] | There are three options: mm, wmm and knn. Default is knn (fast greedy algorithm).
[-t <match_distance_threshold>] | Default is 15 meters
[-f] | force computation

The `<compare_mode>` parameter will be used to determine which matching algorithm to use: `mm` (maximum cardinality matching), `wmm` (maximum weight matching) or `knn` (10-nearest-neighbor greedy matching)   

All-in-One example:
```bash
python3 mapcompare.py data chicago james LCR -i 10 -b 30 -c wmm -t 10 -f
```   
For more datasets and maps visit [mapconstruction.org](http://www.mapconstruction.org/)    
      

# Dependency
* [Rtree](https://pypi.org/project/Rtree/)
* [Networkx](https://pypi.org/project/networkx/)


---


# Map Cropper
In the context of map reconstruction from GPS traces, finding a suitable ground truth map to obtain sensible recall values is a challenge.
One way to tackle the issue is by cropping the ground truth map based on the input trajectories, to ensure that the ground truth covers the same areas as the reconstruction, thus making the comparison fair. Map Cropper is a map matching function, based on Hidden Markov map matching algorithm which takes as input a map and a set of GPS traces, and produces a cropped map containing only those map edges that could be matched to some input trajectory. This is arguably one of the most popular map matching algorithms, it is conceptually simple, and rather efficient. 
`MapCropper.py` follows the same directories as `mapcompare.py` and can be used to crop ground truth maps located in `<dataset_name>/groundtruth` using Hidden Markov matching.      
The code only asks for `<dataset_name>` in `data/`. You will need a `trajectories` folder in `<dataset_name>` containing GPS trajectories.    

![Chicago OSM cropped (red)](https://github.com/Erfanh1995/GraphSamplingToolkit/blob/main/figs/hmm.png)


```bash
python3 MapCropper.py
```

# Dependency
* [leuvenmapmatching](https://pypi.org/project/leuvenmapmatching/)


---


# Visualizer

***In order to properly run this tool, the file hierarchy and file names should follow the same structure in the provided `data` folder.***   
The visualizer provides options to select the desired sets of samples or inputs to be visualized. It also has the option to display precision and recall values of the selected reconstructed map based on its evaluation file. Furthermore, it is possible to view reconstructed maps,  their corresponding ground truth and to overlay a trajectory dataset (that is often the input to map reconstruction programs).   
For the background map/Contextily to work you need a working internet connection.    
`Visualizer.py` follows the same directories as `mapcompare.py`.       
Each `<dataset_name>` needs a `<dataset_name>.yml` file containing its corresponding EPSG code in this format: `EPSG:<number>` e.g. `EPSG:4326`    


![Visualizer](https://github.com/Erfanh1995/GraphSamplingToolkit/blob/main/figs/Legend_picking.gif)      
  
```bash
python3 Visualizer.py
```

# Dependency
* [Fiona](https://pypi.org/project/Fiona/) (geographical operations)
* [Pandas](https://pypi.org/project/pandas/) (data handling)
* [GeoPandas](https://pypi.org/project/geopandas/) (geographical data handling)
* [Shapely](https://pypi.org/project/Shapely/) (geometrical operations)
* [Matplotlib](https://pypi.org/project/matplotlib/) (plotting)
* [PyQt5](https://pypi.org/project/PyQt5/) (user interface)
* [PyYAML](https://pypi.org/project/PyYAML/) (reading the config file)
* [Contextily](https://pypi.org/project/contextily/) (background map)


---

# Contributing

If you are interested in contributing to this project feel free to do a pull request or contact me via email.
