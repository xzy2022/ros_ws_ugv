
运行通信图
```Mermaid
graph TD
    subgraph "System: pure_pursuit_sim"
        %% Nodes
        GL[("global_loader")]
        LP[("local_planner")]
        PP[("pure_pursuit")]
        GTL[("ground_truth_logger")]
        
        subgraph "Sim Env"
            GAZ[("Gazebo/Sim")]
        end
    end

    %% Topics (Defined in global_params.yaml)
    T_BASE([topic: base_waypoints])
    T_FINAL([topic: final_waypoints])
    T_CMD([topic: cmd_vel])
    T_GT([topic: ground_truth/state])
    T_ODOM([topic: odom])

    %% Connections (Inferred from standard logic + yaml)
    GL -->|Pub| T_BASE
    T_BASE -->|Sub| LP
    
    LP -->|Pub| T_FINAL
    T_FINAL -->|Sub| PP
    
    PP -->|Pub| T_CMD
    T_CMD -->|Sub| GAZ
    
    GAZ -->|Pub| T_GT
    T_GT -->|Sub| GTL
    
    GAZ -->|Pub| T_ODOM
    T_ODOM -->|Sub| PP

    %% Styling
    style GL fill:#f9f,stroke:#333
    style LP fill:#f9f,stroke:#333
    style PP fill:#f9f,stroke:#333
    style T_CMD fill:#ffc,stroke:#f90,stroke-dasharray: 5 5
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
    %% --- Global Config Context ---
    subgraph "Namespace: $(arg robot_name)"
        subgraph "global_params.yaml"
            Y_TOPICS["topics:
            - cmd_vel
            - odom
            - ground_truth
            - base_waypoints
            - final_waypoints"]
            
            Y_PHYS["vehicle_physics:
            - wheelbase: 1.868
            - track_width: 1.284"]
            
            Y_LIM["limits:
            - max_speed: 2.0"]
        end
    end

    %% --- Node Context ---
    subgraph "Node: pure_pursuit"
        subgraph "tracking.yaml (Private)"
            Y_TRK_LOOK["lookahead_distance: 0.8"]
            Y_TRK_GAIN["lookahead_gain: 0.0"]
            Y_TRK_FRAME["map_frame: world"]
        end
    end

    subgraph "Node: global_loader"
        P_CSV("param: csv_path")
        P_VEL("param: velocity_kmph")
    end

    subgraph "Node: ground_truth_logger"
        P_OUT("param: output_dir")
    end

    %% --- Mappings ---
    %% Sim Env loads global params
    L_SIM["sim_env.launch"] -->|load| Y_TOPICS
    L_SIM -->|load| Y_PHYS
    L_SIM -->|load| Y_LIM

    %% Pure Pursuit loads its private config
    L_ROOT["pure_pursuit_sim.launch"] -->|load| Y_TRK_LOOK

    %% Implicit Dependency (Nodes reading Global Params)
    Y_TOPICS -.->|used by| GL
    Y_TOPICS -.->|used by| LP
    Y_TOPICS -.->|used by| PP
    Y_TOPICS -.->|used by| GTL
    
    Y_PHYS -.->|used by| PP

    style Y_PHYS fill:#e1f5fe,stroke:#01579b
    style Y_TRK_LOOK fill:#fff9c4,stroke:#fbc02d

```