colcon build --packages-select video_reader master_node detector person_generator pose_detector tracker psg_count psg_collector --cmake-args -DCMAKE_C_COMPILER=clang --cmake-args -DCMAKE_CXX_COMPILER=clang++ --cmake-args
#  -DCMAKE_BUILD_TYPE=Debug
