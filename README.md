# Installation Notes

1. Ensure libccd is installed.
2. Ensure octomap is installed.
3. Clone the FCL repository. Be sure to checkout fcl-0.5 branch.
4. Go into fcl and run "mkdir build". Use ccmake to ensure FCL_STATIC_LIB is on. Then, run "make".
5. Go through "compile_mfcl.m" and ensure each listed package is available at the specific location on the filesystem
