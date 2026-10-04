# Sequence Diagram: /target_pose → /joint_states

PlantUML-версия в [`sequence.puml`](./sequence.puml) (для рендеринга plantuml).
Mermaid-версия здесь (для GitHub / GitLab / VS Code).

## Mermaid sequenceDiagram

```mermaid
sequenceDiagram
    actor User
    participant Input as "teleop_keyboard.py<br/>или run_drift_tests.py"
    participant TargetPose as "/target_pose<br/>PoseStamped"
    participant IKNode as "so101_ik_node"
    participant JointStates as "/joint_states<br/>JointState"
    participant RSPub as "robot_state_publisher"
    participant TF as "/tf<br/>TFMessage"
    participant RViz as "RViz2"

    User->>Input: нажимает клавишу или генерирует PoseStamped
    Input->>TargetPose: публикует target PoseStamped
    TargetPose->>IKNode: callback (50 Гц polling)

    activate IKNode
    Note over IKNode: forwardKinematics(q) → ee_pos, ee_rot
    Note over IKNode: computeJacobian(q) → J
    Note over IKNode: runIKLoop
    loop пока ‖err‖ >= eps
      Note over IKNode: err = [target_pos - ee_pos; log(R_target·ee_rotᵀ)]
      Note over IKNode: v_task = Jᵀ·W·(W·JJᵀW + λ²I)⁻¹·W·e
      Note over IKNode: N = I - J⁺·J
      Note over IKNode: z = sum_null_space_attractors
      Note over IKNode: q += dt·(v_task + N·z)
      Note over IKNode: clamp velocity, apply CBF bounds
      alt stuck
        Note over IKNode: multistart perturb (до 5 попыток)
      end
    end
    Note over IKNode: computeCollisionMarkers
    deactivate IKNode

    IKNode->>JointStates: publish JointState (50 Гц)
    IKNode->>TargetPose: publish target_marker
    IKNode->>TargetPose: publish ee_pose
    IKNode->>TargetPose: publish collision_marker

    JointStates->>RSPub: подписка
    RSPub->>RSPub: read URDF, compute FK
    RSPub->>TF: publish tf2_msgs/TFMessage (100 Гц)

    TF->>RViz: TF buffer update
    JointStates->>RViz: прямой subscriber
    TargetPose->>RViz: markers
```

## Использование

### GitHub
README в репо → диаграмма рендерится автоматически (`.md` с Mermaid-блоками).

### VS Code
Расширение «Markdown Preview Mermaid Support» → авто-рендер.

### PlantUML
```bash
plantuml -tsvg app/docs/sequence.puml
```

## Источник

- [`app/docs/sequence.puml`](./sequence.puml) — PlantUML-оригинал
- [`app/robot_state_publisher.md`](./robot_state_publisher.md) — описание компонента robot_state_publisher
- [`docs/topics.md`](../../docs/topics.md) — форматы всех ROS 2 топиков
