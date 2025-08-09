## Part 1
```
bash ./start_host.sh
ros2 launch robot sim.py
```

## Part 2
```
bash ./start_host.sh
source install/local_setup.sh
python3 src/robot/test_openai_setup.py
ros2 launch robot openai_api.py
```