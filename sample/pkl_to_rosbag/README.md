# pkl_to_rosbag

## Build

```shell
mkdir build && cd build
cmake ..
make
```

The target file named **pkl_to_rosbag** will be generated in the **build** folder.



## Options

**Single letter options:**

```
-i [path with .pkl files]
-o [path to the rosbag.bag file]
```

**Long options:**

```
--input_path=[path with .pkl files]
--output_path=[path to the rosbag.bag file] 
```

**Points of Attention:**

- When using the long options, there must be no spaces on either side of the equal sign
- When using the long options,  the path cannot be with ~ (e.g. ~/Desktop/)
- All the paths should end with /
- When no path is specified, the current working directory is used by default
- The topic frame is **base_link**



## Sample

No path is specified

```
./pkl_to_rosbag 
```

Specify the path

```
~/Desktop/pkl_to_rosbag -d ~/Desktop/test/ -o ~/Desktop/
```

```
./pkl_to_rosbag --input_path=/home/znqc/Desktop/test/ --output_path=/home/znqc/Desktop/
```

