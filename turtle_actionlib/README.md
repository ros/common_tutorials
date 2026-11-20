## Overview

This tutorial package provides a `shape_server` and example
`shape_client` for drawing regular polygons with the
`turtlesim_node`. The default client will draw a pentagon in
`turtlesim`.

The `Shape.action` is defined below: 

```
#goal definition
int32 edges
float32 radius
---
#result definition
float32 interior_angle
float32 apothem
---
#feedback
```



## Running the Tutorial Code

To run the default example, start by getting the required dependencies
and making the package.



```
$ rosdep install turtle_actionlib
$ rosmake turtle_actionlib
```



Then in separate terminals `rosrun` the following nodes:


```
$ rosrun turtlesim turtlesim_node
$ rosrun turtle_actionlib shape_server
$ rosrun turtle_actionlib shape_client
```



You will see:

![attachment:turtle_shape.png](doc/turtle_shape.png)
