# SO-101 IK (ROS 2 Jazzy + Pinocchio + RViz2)

3D Inverse Kinematics узел для робота SO-ARM101, работающий в Docker (ROS 2 Jazzy + RViz2 + Pinocchio).

URDF: `/workspace/SO-ARM100/Simulation/SO101/so101_new_calib.urdf` (монтируется в контейнер при сборке образа).

---

## 1. Алгоритм

Базовый алгоритм — **Damped Least Squares (DLS)** в операционном пространстве (CLIK):

1. Прямая кинематика `EE(q)` для текущей конфигурации `q`.
2. Ошибка позиции энд-эффектора `e = target − EE(q)`.
3. Позиционный якобиан `J_p` (3 × n_v).
4. **Primary task**: `v_task = J_pᵀ · (J_p · J_pᵀ + λ²I)⁻¹ · e`.
5. **Null-space проектор**: `N = I − J_p⁺ · J_p` (n_v × n_v).
6. **Null-step** — сумма аттракторов:
   ```
   z = w_jc·(q_mid−q) + w_prev·(q_start−q) + w_pref·(q_pref−q) + w_man·∇μ
   ```
7. **Шаг**: `q ← integrate(q, dt · (v_task + N · z))`, затем клип по `lower/upperPositionLimit`.

Итерации повторяются до сходимости (`err < ik_eps`) или исчерпания `ik_max_iter`.

---

## 2. Улучшения IK

| # | Улучшение | Функция / место | Параметры |
|---|-----------|-----------------|-----------|
| 1 | Консервативная оценка `r_max` | `computeWorkspaceRadius()` | `link_sum * 1.02`, минимум из семплинга 2000 случайных конфигураций |
| 2 | Проекция недостижимой цели | `projectToWorkspace()` | `reach_padding` = 0.01 |
| 3 | Адаптивное демпфирование по μ (Yoshikawa) | `solveIK()` | `ik_damp` = 1e-6, `ik_man_k` = 1e-4, `ik_man_thresh` = 1e-4 |
| 4 | Адаптивный шаг `dt` | `solveIK()` | `ik_dt` = 0.1, `ik_dt_min` = 0.05, `ik_dt_max` = 0.5 |
| 5 | Детектор застревания | `solveIK()` | `ik_stuck_patience` = 40 |
| 6 | Многостарт с разными масштабами | `perturbConfig()` | `perturb_scales` = [0.05, 0.15, 0.30, 0.50, 0.80] |
| 7 | Flip-рестарт shoulder_pan для целей за base | `flipShoulderPan()` | `auto_flip_restart` = true |
| 8 | Null-space: joint centering (к середине диапазона) | `solveIK()` | `weight_jc` = 0.5 |
| 9 | Null-space: минимум движения (к текущей позе) | `solveIK()` | `weight_prev` = 0.3 |
| 10 | Null-space: предпочтительная поза (elbow-up) | `solveIK()` | `weight_pref` = 0.5, `preferred_q` = [0.0, −0.3, +1.0, −0.7, 0.0, 0.0] |
| 11 | Null-space: анти-сингулярность (градиент μ) | `numericalManipGradient()` | `weight_man` = 0.2 |
| 12 | Мягкая классификация результата | `solveIK()` | `ik_eps` = 1e-4, `ik_eps_visual` = 0.01 |

`preferred_q` подобран так, чтобы избежать самопересечения `upper_arm_link` / `lower_arm_link` в типичных позах (особенно при целях за base_link).

---

## 3. Запуск

### Сборка образа
```bash
docker build -t so101-ik:latest .
```

### Запуск контейнера
```bash
docker run --rm --net=host --name so101-dev -it so101-ik:latest
```

(с пробросом X11 при визуальном RViz на хосте:
`docker run --rm --net=host -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --name so101-dev -it so101-ik:latest`)

### Запуск стенда (RViz + IK + robot_state_publisher)
```bash
ros2 launch so101_ik so101_ik.launch.py
```

ROS setup (`/opt/ros/jazzy/setup.bash`, `install/setup.bash`) прописан в `~/.bashrc` образа — не требуется вручную.

---

## 4. Публикация целей

**Топик**: `/target_pose`
**Тип**: `geometry_msgs/msg/PointStamped`
**Frame**: `base_link`

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PointStamped \
  '{header: {frame_id: base_link}, point: {x: 0.25, y: 0.05, z: 0.10}}'
```

Полный набор из 27 тестов (группы A, B, C, D) — в скрипте [`scripts/run_all_tests.sh`](scripts/run_all_tests.sh). Запуск в контейнере после `ros2 launch`:

```bash
bash scripts/run_all_tests.sh
```

Логика пауз: между отдельными тестами 3 с, внутри серий D2/D3 — 1 с.

### Описание групп

| Группа | Тестов | Ожидаемое поведение |
|--------|--------|---------------------|
| **A** — внутри workspace | 6 | 🟢 `IK converged`, err ≈ 0.0001 |
| **B** — за пределами | 5 | 🟠 `WARN: Target outside workspace`, проекция на границу; EE близко к границе, иногда 🟢/иногда не сходится (см. ограничения) |
| **C** — у границы | 2 | 🟢 или 🟡 `IK approximate` |
| **D** — серии плавных движений | 14 (1 + 5 + 8) | 🟢 все |

---

## 5. Наблюдение

| Топик | Тип | Описание |
|-------|-----|----------|
| `/joint_states` | `sensor_msgs/JointState` | Текущие углы суставов (для `robot_state_publisher`) |
| `/ee_pose` | `geometry_msgs/PoseStamped` | Реальная поза EE после IK в `base_link` |
| `/target_marker` | `visualization_msgs/Marker` | Сфера-маркер цели; цвет = статус IK |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Дерево трансформаций |

### Цвета маркеров (статусы IK)

| Цвет | Статус | Условие |
|------|--------|---------|
| 🟢 зелёный | `Converged` | err < `ik_eps_visual` (1 см) |
| 🟡 жёлтый | `Approximate` | 0.01 ≤ err < 0.05 |
| 🟠 оранжевый | `Projected` | цель была вне workspace, спроецирована |
| 🔴 красный | `Failed` | ни один из 5+1 рестартов не дал нужной точности |

Если статус `Projected`, EE может как сойтись (🟢), так и не сойтись (🟠) — зависит от того, достижима ли спроецированная точка после IK-итераций.

---

## 6. Настройка в рантайме

```bash
# Все параметры IK-узла
ros2 param list /so101_ik_node

# Изменить без пересборки
ros2 param set /so101_ik_node weight_pref 0.3
ros2 param set /so101_ik_node preferred_q "[0.0, -0.3, 1.5, -1.0, 0.0, 0.0]"
ros2 param set /so101_ik_node perturb_scales "[0.05, 0.15, 0.30, 0.50]"
```

Полный список параметров — в `so101-ik-node.cpp` (`declare_parameter` в конструкторе).

---

## 7. Логирование

```bash
# Все строки IK-узла в реальном времени
docker logs -f so101-dev 2>&1 | grep so101_ik_node

# Только ключевые события (проекции, IK-результат)
docker logs -f so101-dev 2>&1 | grep -E "IK |Workspace|outside"
```

Каждая публикация цели порождает строки:
- `New target: [x, y, z]`
- `Target outside workspace (r=…, r_max=…), projected to [x, y, z]` (если проекция)
- `IK converged|approximate|projected|failed, final EE=[…] err=…`

---

## 8. Известные ограничения и направления улучшения

### Текущие ограничения

1. **Только 3D IK (позиция)**. Ориентация энд-эффектора не контролируется. При 6D IK потребуется `PoseStamped` вместо `PointStamped` и работа с 6×n_v якобианом.
2. **Сингулярность на границе workspace**. Тесты B1, B3, B5 проецируют цель на расстояние ≈ `r_max` от base, что соответствует полностью вытянутому локтю. Damped pseudoinverse в этой зоне делает микро-шаги и не сходится за 500 итераций. На графике маркер остаётся оранжевым.
3. **Нет полного collision-check**. Аттрактор `weight_pref` удерживает elbow_flex в зоне «elbow-up», что снижает вероятность пересечения звеньев, но не гарантирует отсутствие коллизий в произвольных позах.
4. **`preferred_q` подобран под SO-101**. Для другой морфологии манипулятора нужно пересмотреть.
5. **Семплинг workspace детерминирован** (`std::mt19937 rng(42)`). Если URDF меняется — `r_max` пересчитывается автоматически, но `preferred_q` остаётся прежним.

### Возможные улучшения

| Идея | Сложность | Эффект |
|------|-----------|--------|
| Levenberg-Marquardt line search (пробовать `dq`, `dq/2`, `dq/4`) | средняя | Быстрее выход из локальных минимумов |
| FCL collision-check между звеньями | высокая | Полная гарантия отсутствия пересечений |
| Расширение до 6D PoseStamped | низкая | Контроль ориентации EE |
| Cartesian CLIK per tick (IK каждый кадр таймера) | низкая | Гладкая траектория EE вместо joint-space интерполяции |
| Стохастический multi-start с большим N | низкая | Лучшее покрытие конфигурационного пространства |
| Joint-space velocity / effort limits в DLS | средняя | Реалистичное движение |

---

## 9. Структура проекта

```
rviz2-docker/
├── Dockerfile
├── README.md
├── app/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── so101-ik.cpp            # offline-пример без ROS
│   ├── so101-ik-node.cpp       # основной IK-узел
│   ├── launch/
│   │   └── so101_ik.launch.py  # robot_state_publisher + ik + rviz
│   └── config/
│       └── so101.rviz          # конфиг RViz (Interact активен по умолчанию)
└── scripts/
    └── run_all_tests.sh        # полный набор тестов для /target_pose
```

URDF и ассеты подгружаются в `/workspace/SO-ARM100/...` на этапе сборки образа (см. `Dockerfile`, шаг `git clone https://github.com/TheRobotStudio/SO-ARM100.git`).
