```
git clone git@github.com:hexonxons/ros2_service_test.git
cd ros2_service_test
docker run --rm -it --net host -v $(pwd):/repo ros:foxy /bin/bash
cd repo
colcon build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=RelWithDebInfo --no-warn-unused-cli --merge-install
source install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/repo/default_profile.xml
python3 ipc_test.py
```
