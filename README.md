Assignment A5

 Wall squeeze CG Group 14 
 https://youtu.be/J2vI5Q-txbw 
 
 Plane ingress CG Group 14
 https://youtu.be/9qO_MUMFOpU 
 
 Hallway four way rounded roundabout CG Group 14 
 https://youtu.be/fwh1C3knhEU 
 
 Double squeeze CG Group 14 
 https://youtu.be/34lVXvQEG2k
 
bottleneck squeeze CG Group 14 
https://youtu.be/kbqUvrYE1rA 

office complex CG Group 14 
https://youtu.be/4iIxO_zA3ms 

maze CG Group 14
https://youtu.be/yZ5s-hunnME 

plane egress CG Group 14
https://youtu.be/75xuuXNNBmg 

doorway two way CG Group 14
https://youtu.be/ZAhxFKZjubA 

hallway two way CG Group 14
https://youtu.be/6xyE49ldB9w 

Crowd crossing CG Group 14 
https://youtu.be/hiDJ5Dt19Bo

self edited testcase Airport CG Group 14
https://youtu.be/NdibtJ2kwZk

To execute our simulation the command is
./steersim –config Myconfig.xml –testcase testcasename –ai sfAI
The idea of this complex crowd simulation is to combine social forces and A* pathfinder.
The idea of pathfinder work with social forces has put the sequence of nodes that generate by A* into the waypoint and goal queue list, after reach, the current local goal, set the current local goal to the next goal in the waypoint and goal queue list. Sometimes, the agent will crack when it reach the corner of obstacles, so we need to recompute the path in four-way A*.

We merge the A* pathfinder into our social forces AI

In reset function push the sequence of the node that computer by A* into waypoints and mid term path and goalQueue list.

In the updateAI we use "hasLineOfSightTo" function to check is there any obstacles between the agent current position and the current local goal. If there is an obstacle on the way to the local goal, recalculate the path with four-way A* algorithm. in addition, if the local goal is to far away from the current position, it will also recalculate the path.

In most case the OBSTACLE_CLEARANCE = 0 In Maze test case, the agent size is bigger than another test case, so we increase the OBSTACLE_CLEARANCE = 2;

In A* we decide to choice weight to 2 and also chose big G value and Manhattan distance.
In the test case hallway-four-way-rounded-roundabout we have solved the polygon obstacles

Implementation method:
1. How to detect if polygon obstacle is in the agents' query area?
Using GJK to detect if agent. Position + query radius colloid with polygon obstacle. If so, we start to compute the repulsion force between polygon obstacle and agents. else, do nothing.

2. How to calculate the force between agents and polygon obstacle?
If polygon obstacle is in the query area of agents, first, we get the nearest vertices in the polygon and split the angel of this vertices into two parts. Then we compare the vector from the vertices to the agent and the vector from vertices to its next vertices to determine which side of the vertices the agent is. Then we treat the face of the side of the polygon obstacle as the wall to calculate the repulsion force.

In the office complex, we got a segmentation fault. The reason is in this test case the wide of the obstacle is so small so many agent cracks in the corner of the obstacles. So many recalculate path in this test case.

We increase the goal force time 4. So it can reduce the bounce between agents.

Our created test case is an airport simulation. Some plane is departing and some are arriving. And we also change the shape of agent and color. So it likes to look UFO. Please replace the DrawLib.cpp with our own DrawLib.cpp





SteerLite
==========

-----------
 CONTENTS
-----------
- Introduction
- Documentation
- Directory Structure
- Compiling SteerSuite
- Contact Information
- Copyright and License
- Credits

---------------
 INTRODUCTION
---------------

Welcome!  SteerSuite is a set of test cases, tools, and a C++ library
for developing and evaluating agent steering AI behaviors.

The SteerSuite web page is:

  http://steersuite.eecs.yorku.ca/

On the web page you can find a description of SteerSuite, download the
latest version of SteerSuite, read the latest documentation, and join
the discussion group.

Please send us any comments and feedback about how to make SteerSuite
easier to use.  Your input is greatly appreciated.

Enjoy!


----------------
 DOCUMENTATION
----------------

If you want to build your own documentation from this package, read
the file documentation/readme.txt for more information.  Otherwise,
the latest documentation can be found at the SteerSuite web page at:

  http://steersuite.eecs.yorku.ca/

The documentation includes:

  - The User Guide explains how to use most features of SteerSuite
  - The Reference Manual contains a comprehensive listing of
    SteerSuite components
  - The Code Reference is doxygen-generated documentation of the C++
    code.



----------------------
 DIRECTORY STRUCTURE
----------------------

The directory structure of this package is as follows:

	- build/          Unix scripts and Visual Studio 2012 solution file 
                  to compile all components of SteerSuite.
	- documentation/  raw unprocessed documentation and instructions for
                  building the documentation.
	- external/       external dependencies that are (legally) included
                  for convenience, but NOT part of SteerSuite.
	- pprAI/          source directory for the PPR steering module, a
                  demo steering algorithm for SteerSim.
	- rvo2AI/         source directory for the ORCA steering module, based
                  on the RVO2 steering algorithm library.
	- socialForcesAI/ source directory for the social foces steering module, a
                  implementation of the social forces steering algorithm.
	- simpleAI/       source directory for the simpleAI module, a basic
                  demo plugin for SteerSim.
	- steerbench/     source directory for SteerBench, a tool used to
                  score and analyze steering AI.
	- steerlib/       - source directory for SteerLib, a shared library
                  containing most of SteerSuite's functionality.
	- steersim/       source directory for SteerSim, a modular simulation
                  tool.
	- steertool/      source directory for SteerTool, a utility for useful
                  miscellaneous tasks
	- testcases/      XML test cases and the XML schema describing the
                  test case format.



-----------------------
 COMPILING STEERSUITE
-----------------------

Below are quick instructions for compiling with default options. For
more complete instructions, refer to the SteerSuite User Guide.  

As with any graphics library you will need to make sure you already have the
opengl libraries on you computer. For example on Ubuntu 14.04 you will want
to install
```
freeglut3-dev build-essential libx11-dev libxmu-dev libxi-dev libgl1-mesa-glx libglu1-mesa libglu1-mesa-dev libglew1.6-dev mesa-utils libglew-dev
```
This will install opengl and glew.  

Windows XP/Vista/7/8 with Visual Studio 2012:
  1. Open the Visual Studio 2012 solution file, located in 
     build/win32/steersuite.sln
  2. Choose Debug or Release mode,
  3. All components should compile successfully.
  4. All compiled components will be in the build/win32/Debug/ or
     build/win32/Release/ folder.

Mac OS X:
  For now, the process is the same as Linux/Unix.  With OS X version
  10.4 or earlier, you may need to use an LD_LIBRARY_PATH environment
  variable for the executable to properly link with shared and dynamic
  libraries.

Linux/Unix:
  1. From a command line, go to the build/ directory
  2. Run ./buildall <platform>
      - make sure you are in the build/ directory.
      - running ./buildall with no args will list the possible platforms.
  3. All of the given components should compile succefuly. A few librarys
   that are not included may fail.
  4. All components are copied into the build/bin/, build/lib/, and
     build/modules/ directories.


----------------------
 CONTACT INFORMATION
----------------------

To report bugs or give general feedback, email Shawn or Glen or Mubbasir.

Contact Information:
  Glen Berseth      gberseth@cs.ubc.ca
  Mubbasir Kapadia  mubbasir@cs.ucla.edu
  Petros Faloutsos  pfal@cse.yorku.ca
  Glenn Reinman     reinman@cs.ucla.edu

SteerSuite web page:
  http://steersuite.eecs.yorku.ca/

------------------------
 COPYRIGHT AND LICENSE
------------------------

SteerSuite, SteerBench, SteerBug, SteerSim, and SteerLib,
Copyright (c) 2008-2014 Shawn Singh, Glen Berseth, Mubbasir Kapadia, Petros
Faloutos, Glenn Reinman.

See license.txt for complete license.

NOTE:
The contents of the external/ directory are NOT part of SteerSuite.
Each component in external/ has its own authors, copyright, and
license, and those sources are only included for convenience.

----------
 CREDITS
----------

Refer to the SteerSuite web page for credits and acknowledgements.


