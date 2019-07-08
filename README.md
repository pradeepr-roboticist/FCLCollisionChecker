# Installation Notes

1. Ensure libccd is installed.
2. Ensure octomap is installed.
3. Install both FCL 0.5 and 0.6. See below.
4. Go through "compile_mfcl.m" and ensure each listed package is available at the specific location on the filesystem


# Why do you need both FCL 0.5 and 0.6
I need FCL 0.5 for handling octomap collision detection. But, it does not work well for distance calculation. It return distances in object coordinates (which was fixed in 0.6). In FCL 0.6, distance calculation returns distances in world coordinates. I could not figure out exactly which code in FCL 0.6 was changed although I was able to narrow it down to a pull request. But, I was not confident and did not have the time to just apply that commit to FCL 0.5 branch.

Strangely enough, in FCL 0.6, collision detection with octomap did not work as expected. Collision detections were spotty and quite random. I wasted a day trying find out the source of the problem but I could not figure it out.

So, the hack proceeds as follows. I use both FCL 0.5 and FCL 0.6. I use a preprocessor directive "FCL_NEW" and "FCL_OLD" in the code files "FCLCollisionChecker.h" and "fcl_collision_checker_interface.cpp". From "compile_mfcl.m", I compile the MEX wrapper twice (once for each library). Thus, I end up having "fcl_collision_checker_interface_fcl_new" and  "fcl_collision_checker_interface_fcl_old".

I use these two MEX files in unison via "FCLCollisionChecker.m" class file. Every operation is duplicated on both MEX interfaces except the following. Pointer (handles) to each interface is separate. And, collision detection queries are made only via "fcl_collision_checker_interface_fcl_old" and distance queries are made with "fcl_collision_checker_interface_fcl_new".

WARNING: For some bizarre, reason you need to "move" the objects added to the environment at least once globally for distance computation to work in FCL 0.6. i.e. you need to issue an "update_object" command with R = eye(3) and t = zeros(3, 1). An easy way to do this is to use "interactive_object.translate([0 0 0])".

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

