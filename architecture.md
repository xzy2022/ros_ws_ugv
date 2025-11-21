```Mermaid

graph TD
    A[Camera Node] -->|/camera/image_raw| B(Perception Node)
    B -->|/path_planner/target_path| C{Control Node}
    D[Config.yaml] -.->|load params| C
    C -->|/cmd_vel| E[Gazebo Car]
```