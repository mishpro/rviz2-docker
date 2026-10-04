# Диаграммы SO-101 IK

Три Mermaid-диаграммы для понимания работы IK-узла.

## 1. Поток данных IK-узла

```mermaid
flowchart LR
    User([Пользователь / Тест])

    subgraph Input["Источники целей"]
        Teleop["teleop_keyboard.py<br/>клавиатурный ввод"]
        DriftTest["run_drift_tests.py<br/>PoseStamped поток"]
        ManualTest["run_all_tests.sh<br/>27 целей A/B/C/D"]
    end

    subgraph ROS["ROS 2 Jazzy"]
        TargetTopic[/"/target_pose"<br/>PoseStamped/]
        IKNode["so101_ik_node<br/>6×6 DLS + null-space"]
        JointTopic[/"/joint_states"/]
        EETopic[/"/ee_pose"/]
        TargetMarker[/"/target_marker"/]
        CollisionTopic[/"/collision_marker"/]
        RSPub["robot_state_publisher<br/>TF tree"]
    end

    RViz["RViz2<br/>визуализация"]

    User --> Teleop
    User --> DriftTest
    User --> ManualTest
    Teleop --> TargetTopic
    DriftTest --> TargetTopic
    ManualTest --> TargetTopic

    TargetTopic --> IKNode
    IKNode --> JointTopic
    IKNode --> EETopic
    IKNode --> TargetMarker
    IKNode --> CollisionTopic

    JointTopic --> RSPub
    RSPub --> RViz
    EETopic --> RViz
    TargetMarker --> RViz
    CollisionTopic --> RViz
```

## 2. Конечный автомат одной итерации solveIK

```mermaid
stateDiagram-v2
    [*] --> WarmStart
    WarmStart: warm-start q = q_start_interp
    WarmStart --> ComputeErr: err = W @ [e_pos; e_rot]
    ComputeErr --> CheckConvergence: ‖err‖ < ik_eps?
    CheckConvergence --> Converged: да
    CheckConvergence --> BuildJacobian: нет
    BuildJacobian: J = ∂EE/∂q (6×nv)
    BuildJacobian --> CheckManip: μ = det(JJᵀ)
    CheckManip: проверка манипулятивности
    CheckManip --> Stuck: μ мал, stuck_count > patience
    CheckManip --> DLS: μ OK
    DLS: v_task = Jᵀ·W·(W·JJᵀW + λ²I)⁻¹·W·e
    DLS --> NullSpace: z = sum of attractors
    NullSpace: w_jc, w_prev, w_pref, w_man, w_drift, w_coll
    NullSpace --> Integrate: q += dt·(v_task + N·z)
    Integrate --> Clamp
    Clamp: velocity clamp + CBF bounds + position clip
    Clamp --> LoopCheck
    LoopCheck: iter < max_iter AND не застрял?
    LoopCheck --> ComputeErr: да
    LoopCheck --> Multistart: iter >= max_iter
    Multistart: perturb q, restart до 5 попыток
    Multistart --> WarmStart
    Stuck: сохранить лучшее, пробовать multistart
    Stuck --> Multistart
    Converged --> Done
    Done: опубликовать q, target_marker, ee_pose, collision_marker
    Done --> [*]
```

## 3. Системная архитектура

```mermaid
flowchart TB
    subgraph Host["Хост-машина"]
        X11["X11 Server<br/>$DISPLAY"]
        User([Пользователь])
        Docker["Docker Engine"]
    end

    subgraph Container["Контейнер so101-ik<br/>osrf/ros:jazzy-desktop"]
        subgraph Workspace["/workspace/ros2_ws"]
            subgraph App["src/so101_ik/"]
                IKNode["app/so101-ik-node.cpp<br/>6×6 DLS IK"]
                Launch["app/launch/so101_ik.launch.py"]
                Config["app/config/so101.rviz"]
                Scripts["scripts/<br/>run_all_tests.sh<br/>run_drift_tests.py<br/>teleop_keyboard.py"]
            end
            URDF["URDF модель<br/>so101_new_calib.urdf<br/>5 DOF arm + gripper"]
            Build["colcon build<br/>--merge-install"]
        end

        Pinocchio["ros-jazzy-pinocchio<br/>FK + Jacobian"]
        Eigen3["libeigen3-dev<br/>линейная алгебра"]
        Rclcpp["rclcpp / rclpy<br/>ROS 2 клиент"]
    end

    User --> X11
    User --> Docker
    X11 --> Container
    Docker --> Build
    Build --> Workspace
    App --> Pinocchio
    App --> Eigen3
    App --> Rclcpp
    App --> URDF
    App --> RViz2[RViz2]
    Launch --> Container
    Container --> IKNode
    IKNode --> Rclcpp
    RViz2 --> X11
```

## Использование

### Просмотр в Markdown-рендерере (GitHub, GitLab, VS Code)

Просто откройте `app/diagrams.md` — Mermaid-блоки отрендерятся автоматически.

### Рендеринг в SVG/PNG

```bash
# Требует @mermaid-js/mermaid-cli (npm install -g @mermaid-js/mermaid-cli)
npx -p @mermaid-js/mermaid-cli mmdc -i app/diagrams.md -o app/diagrams.svg

# Или через Python (без mmdc)
python <SKILL_ROOT>/diagram-generator/scripts/render_diagram.py \
  app/diagrams.md --format svg --out app/diagrams.svg
```
