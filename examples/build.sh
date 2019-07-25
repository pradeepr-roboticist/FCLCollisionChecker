ROOT_DIR=`dirname $PWD`
CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$ROOT_DIR/dependencies/fcl_old/fcl/build/install
CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$ROOT_DIR/dependencies/octomap/octomap/build/install/share

CURR_DIR=$PWD
mkdir -p build
cd build && cmake .. && make
