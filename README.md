# FCL Collision Checker
![demo_image](https://raw.githubusercontent.com/pradeepr-roboticist/FCLCollisionChecker-MEX-Matlab/master/docs/image.png)

This repository contains the following:

1) A MATLAB interface to the FCL collision checking library [FCL](https://github.com/flexible-collision-library/fcl)
2) A minimal working example (in C++) of using FCL for collision checking

I developed this interface primarily for performing collision checking in robotic motion planning.
In my work, I use a ball approximation of robot geometries a lot.
This interface allows for collision detection between (1) objects defined as STL files and (2) a set of balls.
It can be extended or modified to allow collision detection between two STL objects. But, I have not taken that route yet.

I hope this code is of use to anyone wanting to:

1) Perform collision detection between objects and balls
2) Compute distance information between objects and balls
3) Extend code in the repository to support collision detection and/or distance computation between user-defined group of objects

## Credits

I am grateful to the authors of the following packages:

1) Example MATLAB class wrapper for a C++ class by Oliver Woodford
https://www.mathworks.com/matlabcentral/fileexchange/38964-example-matlab-class-wrapper-for-a-c-class

2) FCL library
https://github.com/flexible-collision-library/fcl

3) binvox executable
https://www.patrickmin.com/binvox/download.php?id=4

4) Octomap library
https://github.com/OctoMap/octomap

5) stlTools by Pau Micó
https://www.mathworks.com/matlabcentral/fileexchange/51200-stltools?focused=3878420&tab=function


## Installation Notes

1. ~~Ensure libccd is installed.~~
2. ~~Ensure octomap is installed.~~
3. ~~Make sure FCL is not installed on /usr/local/include/. It will have conflicts. If possible rename the fcl directory, to say "fcl_bkp".~~
4. ~~Install both FCL 0.5 and 0.6. See below.~~
5. ~~Go through "compile_mfcl.m" and ensure each listed package is available at the specific location on the filesystem~~

1. Run "prep_dependencies.sh" from the root of the repo



## C++ wrapper usage
1) Download or clone this repo into your folder.
2) Run the script "prep_dependencies.sh".
3) Checkout the "examples" folder.
4) Be sure to compile using "build.sh"


## Some tips
1) You need to have set a MEX compiler for this to work
2) This was tested in MATLAB 2018b. But, it should work for any recent version of MATLAB.

## Author

**Pradeep Rajendran**

* [github/pradeepr-roboticist](https://github.com/pradeepr-roboticist)

## License

Copyright © 2019 [Pradeep Rajendran](https://github.com/pradeepr-roboticist)
Released under the [GNU General Public License](https://github.com/pradeepr-roboticist/FCLCollisionChecker-MEX-Matlab/blob/master/LICENSE).

# Why do you need both FCL 0.5 and 0.6?
I need FCL 0.5 for handling octomap collision detection. But, it does not work well for distance calculation. It return distances in object coordinates (which was fixed in 0.6). In FCL 0.6, distance calculation returns distances in world coordinates. I could not figure out exactly which code in FCL 0.6 was changed although I was able to narrow it down to a pull request. But, I was not confident and did not have the time to just apply that commit to FCL 0.5 branch.
Strangely enough, in FCL 0.6, collision detection with octomap did not work as expected. Collision detections were spotty and quite random. I wasted a day trying find out the source of the problem but I could not figure it out.
So, the hack proceeds as follows. I use both FCL 0.5 and FCL 0.6. I use a preprocessor directive "FCL_NEW" and "FCL_OLD" in the code files "FCLCollisionChecker.h" and "fcl_collision_checker_interface.cpp". From "compile_mfcl.m", I compile the MEX wrapper twice (once for each library). Thus, I end up having "fcl_collision_checker_interface_fcl_new" and  "fcl_collision_checker_interface_fcl_old".
I use these two MEX files in unison via "FCLCollisionChecker.m" class file. Every operation is duplicated on both MEX interfaces except the following. Pointer (handles) to each interface is separate. And, collision detection queries are made only via "fcl_collision_checker_interface_fcl_old" and distance queries are made with "fcl_collision_checker_interface_fcl_new".

WARNING: This has been taken care of within FCLCollisionChecker.h already. But, it is good to know. For some bizarre, reason you need to "move" the objects added to the environment at least once globally for distance computation to work in FCL 0.6. i.e. you need to issue an "update_object" command with R = eye(3) and t = zeros(3, 1). An easy way to do this is to use "interactive_object.translate([0 0 0])".

# Installing FCL 0.6
1. Clone the FCL repository into fcl_new. Be sure to checkout master branch.
2. Go into fcl and run "mkdir build && cd build". Use command: cmake -DCMAKE_INSTALL_PREFIX=./install -DFCL_STATIC_LIBRARY=ON ..
3. Then, sudo make install
4. Get rid of .git folder

# Installing FCL 0.5
1. Clone the FCL repository into fcl_old. Be sure to checkout fcl-0.5 branch using git checkout fcl-0.5
2. Go into fcl and run "mkdir build && cd build". Use command: cmake -DCMAKE_INSTALL_PREFIX=./install -DFCL_STATIC_LIBRARY=ON ..
3. Then, sudo make install
4. Get rid of .git folder

