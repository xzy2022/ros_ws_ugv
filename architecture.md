```Mermaid

graph TD
    A[Camera Node] -->|/camera/image_raw| B(Perception Node)
    B -->|/path_planner/target_path| C{Control Node}
    D[Config.yaml] -.->|load params| C
    C -->|/cmd_vel| E[Gazebo Car]
```

```Mermaid
graph TD
    %% 定义样式
    classDef launch fill:#f9f,stroke:#333,stroke-width:2px;
    classDef yaml fill:#cfc,stroke:#333,stroke-width:1px;
    classDef node fill:#ddd,stroke:#333,stroke-width:1px;

    subgraph "Root: pure_pursuit_sim.launch"
        Root[pure_pursuit_sim.launch]:::launch
        
        %% Main Logic Nodes
        NodeLoader(global_loader<br/>global_path_loader.py):::node
        NodePlanner(local_planner<br/>local_path_planner.py):::node
        NodeLogger(ground_truth_logger<br/>ground_truth_logger.py):::node
        NodeTracker(pure_pursuit<br/>pure_pursuit_node.py):::node
        
        %% Configs
        ConfTracker[tracking.yaml]:::yaml

        %% Connections
        Root --> NodeLoader
        Root --> NodePlanner
        Root --> NodeLogger
        Root --> NodeTracker
        ConfTracker -.->|load| NodeTracker
    end

    subgraph "Included: sim_env.launch"
        SimEnv[sim_env.launch]:::launch
        
        %% Sim Env Configs
        ConfGlobal[global_params.yaml]:::yaml
        
        %% Sim Env Children
        LaunchDesc[upload_description.launch]:::launch
        LaunchGazebo[sim.launch]:::launch
        LaunchControl[controllers.launch]:::launch
        NodeRviz(rviz):::node

        %% Connections within Sim Env
        SimEnv --> LaunchDesc
        ConfGlobal -.->|load ns=smart| SimEnv
        
        %% Conditionals resolved based on args passed from Root
        %% start_gazebo=true, start_rviz=true
        SimEnv --"if start_gazebo=true"--> LaunchGazebo
        SimEnv --"if start_gazebo=true"--> LaunchControl
        SimEnv --"if start_rviz=true"--> NodeRviz
    end

    %% Cross-file Include
    Root -->|include args: gazebo=T, rviz=T| SimEnv
```