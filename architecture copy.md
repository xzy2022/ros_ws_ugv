
运行通信图
```Mermaid
graph TD
    subgraph "System: pure_pursuit_sim"
        %% Nodes defined in the main launch file
        GL[("Node: global_loader<br>(global_path_loader.py)")]
        LP[("Node: local_planner<br>(local_path_planner.py)")]
        GTL[("Node: ground_truth_logger<br>(ground_truth_logger.py)")]
        PP[("Node: pure_pursuit<br>(pure_pursuit_node.py)")]
        
        %% Nodes from included Sim Env
        subgraph "Include: sim_env"
            RVIZ[("Node: rviz")]
            %% Included logic placeholders
            GAZ[("Sim Simulation<br>(my_robot_gazebo)")]
            CTRL[("Controllers<br>(my_robot_control)")]
        end
    end

    %% Styling
    style GL fill:#f9f,stroke:#333,stroke-width:2px
    style LP fill:#f9f,stroke:#333,stroke-width:2px
    style GTL fill:#f9f,stroke:#333,stroke-width:2px
    style PP fill:#f9f,stroke:#333,stroke-width:2px
    style RVIZ fill:#ccf,stroke:#333,stroke-width:1px
```

启动依赖图
```Mermaid

graph TD
    %% Files
    L_ROOT["pure_pursuit_sim.launch<br>(my_robot_bringup)"]
    L_SIM["sim_env.launch<br>(my_robot_bringup)"]
    L_DESC["upload_description.launch<br>(my_robot_description)"]
    L_GAZ["sim.launch<br>(my_robot_gazebo)"]
    L_CTRL["controllers.launch<br>(my_robot_control)"]
    
    %% Global Params
    Y_GLOB["global_params.yaml"]

    %% Relations
    L_ROOT -->|include| L_SIM
    
    L_SIM -- "load (ns: robot_name)" --> Y_GLOB
    L_SIM -->|include| L_DESC
    
    L_SIM -- "include (if start_gazebo)" --> L_GAZ
    L_SIM -- "include (if start_gazebo)" --> L_CTRL
    
    %% Node Launches
    L_ROOT -.->|launches| N_GL(global_loader)
    L_ROOT -.->|launches| N_LP(local_planner)
    L_ROOT -.->|launches| N_GTL(ground_truth_logger)
    L_ROOT -.->|launches| N_PP(pure_pursuit)
    L_SIM  -. "launches (if start_rviz)" .-> N_RVIZ(rviz)

    style L_ROOT fill:#f96,stroke:#333,stroke-width:2px
    style L_SIM  fill:#ff9,stroke:#333



```

参数映射图
```Mermaid
graph LR
    subgraph "Node: global_loader"
        P_CSV("param: csv_path")
        P_VEL("param: velocity_kmph")
        P_DEC("param: max_decel")
    end
    
    subgraph "Node: local_planner"
        P_LOOK("param: lookahead_wps")
        P_FREQ("param: frequency")
    end

    subgraph "Node: ground_truth_logger"
        P_OUT("param: output_dir")
        P_NS("param: use_robot_namespace")
    end

    subgraph "Node: pure_pursuit"
        Y_TRK["config/tracking.yaml"]
    end

    %% Sources
    SRC_CSV["$(find my_robot_bringup)/data/waypoints.csv"] --> P_CSV
    VAL_VEL["10.0"] --> P_VEL
    VAL_DEC["1.0"] --> P_DEC
    
    VAL_LOOK["20"] --> P_LOOK
    VAL_FREQ["10.0"] --> P_FREQ

    SRC_OUT["/home/xzy/self-driving-my..."] --> P_OUT
    VAL_NS["true"] --> P_NS
    
    Y_TRK -.->|load| pure_pursuit

```