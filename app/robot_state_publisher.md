# `robot_state_publisher` в rviz2-docker

## Что это

`robot_state_publisher` — стандартный ROS 2 пакет (`ros-jazzy-robot-state-publisher`), который публикует дерево TF-трансформаций на основе URDF-модели робота и текущих значений суставов.

## Зачем он нужен в этом проекте

В rviz2-docker есть **3 независимых источника данных** для 3D-визуализации робота:

1. **IK-узел** публикует `/joint_states` (SensorMsg/JointState) — **только** углы суставов (радианы)
2. **RViz2** — нужен для отображения 3D-модели робота в реальном времени
3. **robot_state_publisher** — мост между ними: вычисляет forward kinematics и публикует TF-трансформации

Без `robot_state_publisher`:
- IK работает корректно (вычисляет углы суставов)
- В RViz2 невозможно отобразить 3D-модель — нет данных о положениях линков
- Drift-тесты, teleop и другие функциональные тесты работают (они читают `JointState` напрямую)
- Только визуализация в RViz2 ломается

## Как настроен в проекте

Из `app/launch/so101_ik.launch.py:35-42`:

```python
Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    parameters=[{'robot_description': urdf_content}],
    output='screen',
)
```

- `urdf_content` — XML-строка, загруженная из `/workspace/SO-ARM100/Simulation/SO101/so101_new_calib.urdf` (с заменой путей к mesh-файлам на абсолютные `file:///...`)

## Цепочка данных

```
┌─────────────────┐      ┌──────────────┐      ┌────────────────────────┐
│ so101_ik_node    │      │ robot_state_ │      │ RViz2 (TF + URDF)       │
│  (IK solver)     │ ───► │ publisher    │ ───► │ 3D модель в реальном  │
│                  │ /joint_         │ /tf,        │  времени                │
│ 6×6 DLS          │     │ states        │ /tf_static  │                          │
└─────────────────┘      └──────────────┘      └──────────────────────────┘
       │                       │                       │
       │  читает URDF ◄─────────┘                       │
       │                       │                       │
       └──────── публикует target_marker, ee_pose, joint_states
```

### Шаг 1: IK публикует `JointState`
```python
# app/so101-ik-node.cpp:562-568
sensor_msgs::msg::JointState msg;
msg.header.stamp = now();
msg.name = joint_names_;  # ["shoulder_pan", "shoulder_lift", ...]
msg.position.resize(q_current_.size());
for (int i = 0; i < q_current_.size(); ++i)
    msg.position[i] = q_current_[i];
joint_pub_->publish(msg);
```

### Шаг 2: robot_state_publisher читает URDF + JointState
- **URDF** (`robot_description` параметр) — определяет цепочку линков: `base_link → shoulder_link → upper_arm_link → lower_arm_link → wrist_link → gripper_link`
- **JointState** (из `/joint_states`) — текущие углы суставов

### Шаг 3: robot_state_publisher вычисляет FK
- Проходит по дереву URDF от `base_link`
- Для каждого подвижного сустава применяет `q[i]` к соответствующему `<joint>` transformation
- Получает `T_link_in_world` для каждого линка

### Шаг 4: публикация TF
- `/tf` — динамические трансформации (~100 Гц)
- `/tf_static` — статические (fixed joints, sensors) — публикуются один раз при старте

### Шаг 5: RViz2 использует TF
- TF buffer обновляется на каждом тике
- RobotModel plugin рендерит 3D-модель URDF с правильными позами линков

## Альтернативы

| Подход | Плюсы | Минусы |
|---|---|---|
| **robot_state_publisher** (выбран) | Стандарт ROS 2, поддержка URDF из коробки, динамический и статический TF, поддерживает все типы joints (revolute, prismatic, fixed, ...) | Требует полный URDF-файл с правильными joint-цепочками |
| Прямая публикация TF из IK-узла | Меньше зависимостей | Нужно дублировать логику FK в IK-узле, не получить статические TF для sensors/cameras, нужно вручную вычислять цепочку линков |
| `static_transform_publisher` | Простота для статических TF (например, для камеры) | Только статические TF, не динамические (не подходит для движущихся суставов) |

`robot_state_publisher` — **стандартное решение** в ROS 2 для публикации TF-дерева из URDF + JointState. В данном проекте это правильный выбор, потому что:
- У нас есть полный URDF-файл (5-DOF arm + gripper)
- Нужны динамические TF для анимации в RViz2
- Стандартный пакет ROS 2 — нет кастомного кода для поддержки

## Проверка работоспособности

```bash
# Внутри контейнера
ros2 run tf2_tools view_frames -o /tmp/frames.pdf
# Должно показать дерево base_link → ... → gripper_link

# Или быстрая проверка через tf2_echo
ros2 run tf2_ros tf2_echo base_link gripper_link
```

Если дерево не строится или есть ошибки, частая причина — некорректные пути к mesh-файлам в URDF. В нашем launch-файле это решается через `urdf_content.replace('filename="assets/', 'filename="file:///workspace/SO-ARM100/Simulation/SO101/assets/')` — превращает относительные пути в абсолютные.

## Связанные компоненты

- `app/launch/so101_ik.launch.py:18-21` — загрузка URDF и подстановка путей к mesh
- `app/so101-ik-node.cpp:107` — Publisher для `/joint_states`
- `Dockerfile:5` — установка `ros-jazzy-ros-base` (включает `robot_state_publisher`)

## Источники

- ROS 2 docs: <https://docs.ros.org/en/jazzy/p/robot_state_publisher/>
- `tf2_ros` package: <https://docs.ros.org/en/jazzy/p/tf2_ros/>
- URDF спецификация: <http://wiki.ros.org/urdf>
