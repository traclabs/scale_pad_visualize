# Visualization of scaling and padding of geometric_shapes

Original mesh | Scaled and padded mesh
:----:|:----:
![](images/no-scale-pad.png) | ![](images/scaled-and-padded.png)

To run:

```sh
roslaunch scale_pad_visualize visualize.launch
```

There are three parameters provided for the launch file:
* ``mesh`` - full path to a mesh file (eg. .stl, .dae) (default: *$(find scale_pad_visualize)/stl/ros2.stl*)
* ``scale`` (default: 1.0)
* ``padding`` (default: 0.0)

Example usage:

```sh
roslaunch scale_pad_visualize visualize.launch mesh:=$(rospack find pr2_description)/meshes/base_v0/base.dae scale:=0.8 padding:=0.2
```

The scale and padding can be modified during runtime, using ``rosparam set``:

```sh
rosparam set /visualize/scale 1.2
rosparam set /visualize/padding 0.1
```
