# Usage

demo video (sim) : https://drive.google.com/file/d/1Tt2c6cHXqR5_8o-zYrpyNigNpXnME4Ah/view?usp=sharing

demo video (real): https://drive.google.com/file/d/1j-n2xtFEd0NqzKw6yC7QEz7C2n8Ubgek/view?usp=sharing

1. build and source 
2. edit config/nav_srv.yaml
3. launch navigation
4. launch custom_nav service server

```bash
ros2 launch custom_nav custom_nav.launch.py 
```

5. call service client

```bash
ros2 run custom_nav client {first_param} {second_param}
(example: ros2 run custom_nav client 1 1)
```

- first parameter: mode
    - 1: load map and initial pose
    - 2: navigate to pose
- second parameter: map ID
