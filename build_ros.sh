#echo "Building ROS nodes - ORB_SLAM2"
#pushd Examples/ROS/ORB_SLAM2
#mkdir build
#pushd build
#cmake .. -DROS_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
#make -j
#popd
#popd

echo "Building ROS nodes - ORB_VIO"
pushd Examples/ROS/ORB_VIO
mkdir build
pushd build
cmake .. -DROS_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
make -j
popd
popd
