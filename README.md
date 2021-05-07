# RoutePlanning
This Package is supposed to be the base for the autonomous navigation of our robot in a road like environment. Some of the used Packages wont be found in this repository since i don't have permission to share them.

The Robot is supposed to follow a road on the correct lanes and dodge upcoming obstacles as needed. At the same time it is maping the surounding area and is localizing itself in the map.

The route following and obstacle avoidance will be acomplished using move base and sending new goals in a given interval.

Goals will be found by approximating either circles or lines on the border of the road extracted by a roaddetection node.
After the first round the slam result should be good enough, so we can extract new goals straight from there.

The global and the local costmap are both non static or rolling window so the robot will "forget" old obstacles.

To get the global planner to work like it is requiered here a few changes have been made that are located in my fork of David Lu's navigation repo

This work will be done during my bachelor thesis and hopefully will be improved and used by other people in the future


As my thesis is finished now this repository will remain largly unchanged.
It contains two branches one for the Sourcecode and one for the full Documentation of the Thesis written in LaTeX

