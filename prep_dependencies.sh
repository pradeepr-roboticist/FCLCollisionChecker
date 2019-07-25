# [ "$UID" -eq 0 ] || exec sudo bash "$0" "$@"

FCL_OLD_BRANCH='fcl-0.5'
FCL_NEW_BRANCH='master'

echo "Creating required directories ..."
mkdir -p dependencies/fcl_old/fcl
mkdir -p dependencies/fcl_new/fcl
mkdir -p dependencies/octomap/

echo "Cloning packages ..."
git clone -b $FCL_OLD_BRANCH https://github.com/flexible-collision-library/fcl ./dependencies/fcl_old/fcl
git clone -b $FCL_NEW_BRANCH https://github.com/flexible-collision-library/fcl ./dependencies/fcl_new/fcl
git clone -b master https://github.com/OctoMap/octomap ./dependencies/octomap/

sudo apt-get install libccd-dev

ROOT_FOLDER=$PWD
echo "Building packages ..."
cd dependencies/fcl_old/fcl && \
mkdir -p build && cd build/ && \
cmake -DCMAKE_INSTALL_PREFIX=./install -DFCL_STATIC_LIBRARY=ON .. && \
make -j10 && \
sudo make install && \
cd $ROOT_FOLDER &
pids[0]=$!


cd dependencies/fcl_new/fcl && \
mkdir -p build && cd build/ && \
cmake -DCMAKE_INSTALL_PREFIX=./install -DFCL_STATIC_LIBRARY=ON .. && \
make -j10 && \
sudo make install && \
cd $ROOT_FOLDER &
pids[1]=$!

cd dependencies/octomap/octomap && \
mkdir -p build && cd build/ && \
cmake -DCMAKE_INSTALL_PREFIX=./install .. && \
make -j10 && \
sudo make install && \
cd $ROOT_FOLDER &
pids[2]=$!

# wait for all pids
for pid in ${pids[*]}; do
    wait $pid
done
echo "##########################"
echo "Done building packages"
echo "##########################"
