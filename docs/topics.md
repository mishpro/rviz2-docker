# ROS 2 топики rviz2-docker

В проекте **5 пользовательских топиков** + 1 виртуальный (TF).

## Сводная таблица

| Топик | Тип | Направление | Rate | Frame | Назначение |
|---|---|---|---|---|---|
| `/target_pose` | `geometry_msgs/msg/PoseStamped` | вход | external | `base_link` | Целевая поза EE |
| `/joint_states` | `sensor_msgs/msg/JointState` | выход | 50 Гц | — | Углы суставов для `robot_state_publisher` |
| `/ee_pose` | `geometry_msgs/msg/PoseStamped` | выход | 50 Гц | `base_link` | Реальная поза EE после IK |
| `/target_marker` | `visualization_msgs/msg/Marker` | выход | per-target | `base_link` | Визуализация цели в RViz (цвет = статус IK) |
| `/collision_marker` | `visualization_msgs/msg/Marker` | выход | per-update | `base_link` | Красная сфера при self-collision |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | выход | ~100 Гц | — | Дерево трансформаций из URDF |

## 1. `/target_pose` — входной (subscribed)

**Тип:** `geometry_msgs/msg/PoseStamped`
**Направление:** внешний → IK-узел
**QoS:** default (10)
**Frame:** `base_link`

```
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id            # "base_link"
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x                # [м]
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x
    float64 y
    float64 z
    float64 w                # кватернион (x, y, z, w)
```

**Пример (только position, identity orientation):**
```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: base_link},
    pose: {position: {x: 0.25, y: 0.05, z: 0.10},
           orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```

**Пример (с rotation, 5° вокруг X):**
```bash
# sin(2.5°)≈0.0436, cos(2.5°)≈0.999
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: base_link},
    pose: {position: {x: 0.25, y: 0.05, z: 0.10},
           orientation: {x: 0.0436, y: 0.0, z: 0.0, w: 0.999}}}'
```

**Backwards-compat rule:** если `‖q‖ < 1e-6` (т.е. `(0,0,0,0)`), IK приводит к `Identity` → position-only mode.

## 2. `/joint_states` — выходной (published, 50 Гц)

**Тип:** `sensor_msgs/msg/JointState`
**Направление:** IK-узел → RViz (через `robot_state_publisher`)
**QoS:** default (10)
**Rate:** `publish_rate` (default 50 Гц)

```
std_msgs/Header header
  builtin_interfaces/Time stamp
string[] name               # ["shoulder_pan", "shoulder_lift", "elbow_flex",
                           #  "wrist_flex", "wrist_roll", "gripper"]  (6 шт)
float64[] position          # [рад], размер = name.size()
float64[] velocity          # [рад/с], обычно пусто
float64[] effort            # [Н·м], обычно пусто
```

**Пример (публикуется IK-узлом автоматически):**
```json
{
  "header": {"stamp": {"sec": 1790943626, "nanosec": 474538292}, "frame_id": ""},
  "name": ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"],
  "position": [0.0, -0.3, 1.0, -0.7, 0.0, 0.0],
  "velocity": [],
  "effort": []
}
```

Подписка для просмотра:
```bash
ros2 topic echo /joint_states
```

## 3. `/ee_pose` — выходной (published, 50 Гц)

**Тип:** `geometry_msgs/msg/PoseStamped`
**Направление:** IK-узел → RViz
**Frame:** `base_link`

```
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id            # "base_link"
geometry_msgs/Pose pose
  geometry_msgs/Point position
    float64 x                # [м] - текущая позиция EE
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x                # текущая ориентация EE
    float64 y
    float64 z
    float64 w
```

**Пример (после IK на target [0.25, 0.05, 0.10] identity):**
```json
{
  "header": {"stamp": {"sec": 1790943626, "nanosec": 593454121}, "frame_id": "base_link"},
  "pose": {
    "position": {"x": 0.250, "y": 0.049, "z": 0.099},
    "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
  }
}
```

## 4. `/target_marker` — выходной (published, per-target update)

**Тип:** `visualization_msgs/msg/Marker`
**Направление:** IK-узел → RViz
**Lifetime:** `rclcpp::Duration(0, 0)` — бесконечный

```
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id            # "base_link"
string ns                    # "ik_target"
int32 id                      # 0
int32 type                    # visualization_msgs::Marker::SPHERE
int32 action                  # visualization_msgs::Marker::ADD
geometry_msgs/Pose pose
  geometry_msgs/Point position  # target_pos (та же что в /target_pose)
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x = 0
    float64 y = 0
    float64 z = 0
    float64 w = 1
geometry_msgs/Vector3 scale
  float64 x = 0.01
  float64 y = 0.01
  float64 z = 0.01
std_msgs/ColorRGBA color    # зависит от статуса IK
  float32 r
  float32 g
  float32 b
  float32 a = 1.0
builtin_interfaces/Duration lifetime
  int32 sec = 0
  int32 nanosec = 0
```

**Цвета по статусу IK (из README §6):**

| Статус | RGB |
|---|---|
| 🟢 Converged | `r=0.2, g=0.9, b=0.2` |
| 🟡 Approximate | `r=1.0, g=0.85, b=0.0` |
| 🟠 Projected | `r=1.0, g=0.6, b=0.0` |
| 🔴 Failed | `r=0.9, g=0.1, b=0.1` |

## 5. `/collision_marker` — выходной (published, per-update)

**Тип:** `visualization_msgs/msg/Marker`
**Направление:** IK-узел → RViz
**Назначение:** визуализация self-collision

```
std_msgs/Header header
  builtin_interfaces/Time stamp
  string frame_id            # "base_link"
string ns                    # "self_collision"
int32 id                      # 0
int32 type                    # visualization_msgs::Marker::SPHERE
int32 action                  # ADD или DELETE
geometry_msgs/Pose pose
  geometry_msgs/Point position  # середина между линками в коллизии
    float64 x
    float64 y
    float64 z
  geometry_msgs/Quaternion orientation
    float64 x = 0
    float64 y = 0
    float64 z = 0
    float64 w = 1
geometry_msgs/Vector3 scale
  float64 x = depth = max(0, collision_d_min - d)  # масштаб = глубина проникновения
  float64 y = same
  float64 z = same
std_msgs/ColorRGBA color    # красный при коллизии
  float32 r = 0.9
  float32 g = 0.1
  float32 b = 0.1
  float32 a = 1.0
builtin_interfaces/Duration lifetime
  int32 sec = 0
  int32 nanosec = 0
```

**Логика публикации** (из `publishCollisionMarker()`): если `worst_pair_idx_ >= 0` — `ADD` красная сфера, иначе — `DELETE` (маркер исчезает).

## 6. `/tf` и `/tf_static` — виртуальные (RViz + robot_state_publisher)

**Тип:** `tf2_msgs/msg/TFMessage`
**Направление:** `robot_state_publisher` → TF buffer → RViz
**Генерируются автоматически** из URDF + текущих `/joint_states`

Структура массива `transforms`:
```
geometry_msgs/TransformStamped[]
  std_msgs/Header header
    builtin_interfaces/Time stamp
    string frame_id             # parent frame, e.g. "base_link"
    string child_frame_id       # e.g. "shoulder_link"
  geometry_msgs/Transform transform
    geometry_msgs/Vector3 translation
      float64 x, y, z           # в метрах
    geometry_msgs/Quaternion rotation
      float64 x, y, z, w        # кватернион
```

**Не публикуется вручную** — `robot_state_publisher` строит дерево из URDF и публикует `base_link → shoulder_link → ... → gripper_link` каждый такт.

## Пример комплексного теста

```bash
# В одном терминале — запуск launch
ros2 launch so101_ik so101_ik.launch.py

# В другом — подписка на все топики
ros2 topic echo /target_pose &
ros2 topic echo /joint_states &
ros2 topic echo /ee_pose &

# Публикация цели
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: base_link},
    pose: {position: {x: 0.20, y: 0.0, z: 0.15},
           orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'

# Проверка маркера
ros2 topic echo /target_marker --no-arr --no-str

# Проверка TF дерева
ros2 run tf2_tools view_frames -o /tmp/frames.pdf
```

## Источники

- `app/so101-ik-node.cpp:107-115` — объявления publishers/subscribers
- `app/so101-ik-node.cpp:563-580` — структура JointState/Marker
- `app/so101-ik-node.cpp:599-635` — структура PoseStamped/Marker для EE/collision
