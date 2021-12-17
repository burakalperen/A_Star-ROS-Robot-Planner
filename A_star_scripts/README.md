# A Star Algorithm 

Purpose of these scripts is implementation of A Star algoritm in occupancy grid map.
For now, tested on binary map image. 


* **buidMap.py**:         It converts map image to grid map. If there is obstacle in grid cell, it returns True otherwise returns False.
* **astar.py**  :         Determines appropriate path according to goal world coordinates.
* **transformations.py**: Useful functions for transformations between world and map coordinates
* **node.py**   :         It consists of class that represents path nodes.