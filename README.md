# SO-101 IK (ROS 2 Jazzy + Pinocchio + RViz2)

Inverse Kinematics узел для робота SO-ARM101, работающий в Docker (ROS 2 Jazzy + RViz2 + Pinocchio).
Поддерживает **6D IK** (позиция + ориентация), **velocity/CBF joint bounds**, **quintic smoother**, **self-collision barrier**.

URDF: `/workspace/SO-ARM100/Simulation/SO101/so101_new_calib.urdf` (монтируется в контейнер при сборке образа).

---

## 1. Алгоритм

Базовый алгоритм — **Damped Least Squares (DLS)** в операционном пространстве (CLIK) с **task priority** на двух разных фреймах:

1. **Два EE-фрейма**:
   - `gripper_link` — **для position task** (на оси `wrist_roll`, не сдвигается при вращении кисти).
   - `gripper_frame_link` — **для orientation task** (на конце кисти, ~10 см впереди, смещён от оси вращения). Используется также для публикации `/ee_pose`.
2. **Прямая кинематика** → `ee_pos_pos = oMf[gripper_link].translation`, `ee_rot_curr = oMf[gripper_frame_link].rotation`.
3. **Position error**: `e_pos = target_pos − ee_pos_pos` (3×1).
4. **Backward-compat**: если `R_target ≈ I` (identity quaternion), orientation task пропускается → только position control.
5. **Orientation error**: `e_rot = log3(R_target · R_rot_currᵀ)` (3×1, axis-angle).
6. **Position Jacobian** (3×6) — линейная часть для `gripper_link` через `getFrameJacobian(LOCAL_WORLD_ALIGNED)`.
7. **Position DLS**: `dq_pos = J_pos^T · (J_pos · J_pos^T + λ²I)⁻¹ · e_pos` (точно решает позицию).
8. **Null-space проектор**: `N_pos = I − J_pos⁺ · J_pos` (ранг 3).
9. **Orientation DLS** в null-space (только если `R_target ≠ I`):
   - `J_rot_proj = J_rot · N_pos` (3×6, rotation в null-space).
   - `dq_rot = N_pos · J_rot_proj^T · (J_rot_proj · J_rot_proj^T + λ²I)⁻¹ · e_rot`.
   - `v_task = dq_pos + dq_rot` (position решается точно, orientation — в null-space).
10. **Null-step** — сумма аттракторов (вторичные цели):
    ```
    z = w_jc·(q_mid−q) + w_prev·(q_start−q) [+ w_drift·(q_start−q)] +
        w_pref·(q_pref−q) + w_man·∇μ [+ w_gripper·(q_start[5]−q[5])] [+ w_coll·CBF barrier]
    ```
    `z(5)` — null-space критерий для фиксации gripper (q[5] = `moving_jaw`).
11. **Шаг**: `q ← integrate(q, dt · (v_task + N_pos · z))`, затем velocity clamp + CBF joint bounds + position clip.
12. Итерации до сходимости (`‖e_pos‖ < ik_eps`) или исчерпания `ik_max_iter`.

---

## 2. Улучшения IK

| # | Улучшение | Функция / место | Параметры |
|---|-----------|-----------------|-----------|
| 1 | Консервативная оценка `r_max` | `computeWorkspaceRadius()` | `link_sum * 1.02`, минимум из семплинга 2000 случайных конфигураций |
| 2 | Проекция недостижимой цели (по позиции) | `projectToWorkspace()` | `reach_padding` = 0.01 |
| 3 | Адаптивное демпфирование по μ (Yoshikawa, 6D) | `solveIK()` | `ik_damp` = 1e-6, `ik_man_k` = 1e-4, `ik_man_thresh` = 1e-4 |
| 4 | Адаптивный шаг `dt` | `solveIK()` | `ik_dt` = 0.1, `ik_dt_min` = 0.05, `ik_dt_max` = 0.5 |
| 5 | Детектор застревания | `solveIK()` | `ik_stuck_patience` = 40 |
| 6 | Многостарт с разными масштабами | `perturbConfig()` | `perturb_scales` = [0.05, 0.15, 0.30, 0.50, 0.80] |
| 7 | Flip-рестарт shoulder_pan для целей за base | `flipShoulderPan()` | `auto_flip_restart` = true |
| 8 | Null-space: joint centering | `solveIK()` | `weight_jc` = 0.5 |
| 9 | Null-space: минимум движения | `solveIK()` | `weight_prev` = 0.3 |
| 10 | Null-space: **drift-free criterion** (closed-loop) | `solveIK()` | `weight_drift` = 0.1 (deferred — см. §8) |
| 11 | Null-space: предпочтительная поза (elbow-up) | `solveIK()` | `weight_pref` = 0.5, `preferred_q` = [0.0, −0.3, +1.0, −0.7, 0.0, 0.0] |
| 12 | Null-space: анти-сингулярность (градиент μ) | `numericalManipGradient()` | `weight_man` = 0.2 |
| 13 | **Velocity limit** per joint (clamp `dq`) | `solveIK()` | `vel_max` = [1.0×6] rad/s |
| 14 | **CBF joint bounds** (dynamic bound у лимитов) | `solveIK()` | `mu_boundary` = 5.0 |
| 15 | **Self-collision CBF barrier** (link-to-link) | `solveIK()` | `weight_collision` = 0.5, `collision_d_min` = 0.005 м, `collision_margin` = 0.010 м |
| 16 | 6D error + weighted DLS (см. §1) | `solveIK()` | `weight_pos` = 1.0, `weight_orient` = 1.0 |
| 17 | Мягкая классификация результата | `solveIK()` | `ik_eps` = 1e-4, `ik_eps_visual` = 0.01 |
| 18 | **Quintic min-jerk smoother** (interpolation в `timerCallback`) | `timerCallback()` | `interp_steps` = 100 |
| 19 | **Task priority IK**: position (gripper_link) → orientation (gripper_frame_link) в null-space | `solveIK()` | `ee_pos_id`="gripper_link", `ee_rot_id`="gripper_frame_link" |
| 20 | **Null-space фиксация gripper (q[5] = moving_jaw)** | `solveIK()` | `gripper_mode` = "hold", `weight_gripper` = 5.0 |

---

## 3. Self-Collision (Phase 2)

Пары линков для проверки (non-adjacent, hardcoded в `collision_pairs_`):
```
base_link      ↔ lower_arm_link, wrist_link, gripper_link
shoulder_link  ↔ lower_arm_link, wrist_link
upper_arm_link ↔ wrist_link, gripper_link
lower_arm_link ↔ gripper_link
```

Алгоритм:
- Sphere-sphere distance: `d = ‖p_a − p_b‖ − (r_a + r_b)`, с hardcoded `sphere_radii_` per link.
- В каждой итерации IK считается `d` для всех 8 пар.
- **CBF barrier** активируется **только если** `d < collision_d_min + collision_margin`:
  - Numerical gradient `∂d/∂q_i` через finite differences.
  - Repulsive force: `z += w_collision · (collision_d_min − d) · ∇d`.
- **Visualization**: красная сфера на `/collision_marker` при обнаружении коллизии (scale = penetration depth).

---

## 4. Запуск

### Сборка образа
```bash
docker build -t so101-ik:latest .
```

### Запуск контейнера (с X11 для RViz2)
```bash
xhost +local:docker    # один раз на хосте

docker run --rm --net=host \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --name so101-dev -it so101-ik:latest
```

### Запуск стенда (RViz + IK + robot_state_publisher)
```bash
ros2 launch so101_ik so101_ik.launch.py
```

ROS setup (`/opt/ros/jazzy/setup.bash`, `install/setup.bash`) прописан в `~/.bashrc` образа — не требуется вручную.

---

## 5. Публикация целей (6D PoseStamped)

**Топик**: `/target_pose`
**Тип**: `geometry_msgs/msg/PoseStamped`
**Frame**: `base_link`

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: base_link},
    pose: {position: {x: 0.25, y: 0.05, z: 0.10},
           orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}'
```

`orientation` — кватернион `(x, y, z, w)`. Identity `w=1, x=y=z=0` → только position control (backward-compat).
Для 6D control — задавайте не-identity quaternion.

**Где IK достигает каждой цели (task priority):**
- `position` (3 trans) применяется к **`gripper_link`** — фрейм на оси `wrist_roll`, не сдвигается при вращении кисти.
- `orientation` (3 rot) применяется к **`gripper_frame_link`** — фрейм на конце кисти, описывает окружность при вращении `wrist_roll`. При вращении кисти `gripper_link` остаётся на месте, `gripper_frame_link` поворачивается.

Импульсы `position` и `orientation` разделены: position решается **точно** (приоритет 1), orientation — в null-space позиции (приоритет 2). Конфликта между ними нет.

### Тесты (27 случаев)

```bash
bash scripts/run_all_tests.sh           # внутри контейнера
python3 scripts/run_drift_tests.py \
  --publish-dt 2.5                       # closed-loop drift тест
```

| Группа | Тестов | Ожидаемое поведение |
|--------|--------|---------------------|
| **A** — внутри workspace | 6 | 🟢 `IK converged`, err ≈ 0.0001 |
| **B** — за пределами | 5 | 🟠 проекция; EE близко к границе |
| **C** — у границы | 2 | 🟢 или 🟡 `IK approximate` |
| **D** — серии плавных движений | 14 (1 + 5 + 8) | 🟢 все |

### Keyboard Teleop (Phase 5)

Standalone Python node для ручного тестирования:
```bash
python3 scripts/teleop_keyboard.py
```
внутри контейнера. Или выполнить на хосте для входа в контейнер с последующим запуском телеопа:
```
docker exec -it so101-dev bash -c "python3 /workspace/ros2_ws/src/so101_ik/scripts/teleop_keyboard.py"
```

Клавиши:
```
T/G: ±X    A/D: ±Y    W/S: ±Z           (translation)
J/L: yaw   I/K: pitch U/O: roll         (rotation)
+/-: 1mm/10mm step                       (translation)
[/]: 1°/10° step                         (rotation)
R: reset to home       H: help      Q: quit
```

---

## 6. Наблюдение

| Топик | Тип | Описание |
|-------|-----|----------|
| `/joint_states` | `sensor_msgs/JointState` | Текущие углы суставов (для `robot_state_publisher`) |
| `/ee_pose` | `geometry_msgs/PoseStamped` | Реальная поза EE после IK (включая orientation) |
| `/target_marker` | `visualization_msgs/Marker` | Сфера-маркер цели; цвет = статус IK |
| `/collision_marker` | `visualization_msgs/Marker` | Красная сфера при обнаружении self-collision |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | Дерево трансформаций |

### Цвета маркеров (статусы IK)

| Цвет | Статус | Условие |
|------|--------|---------|
| 🟢 зелёный | `Converged` | weighted err < `ik_eps_visual` |
| 🟡 жёлтый | `Approximate` | weighted err в [eps_visual, 0.05] |
| 🟠 оранжевый | `Projected` | цель была вне workspace, спроецирована |
| 🔴 красный | `Failed` | ни один из 5+1 рестартов не дал нужной точности |

---

## 7. Настройка в рантайме

```bash
ros2 param list /so101_ik_node
ros2 param set /so101_ik_node weight_pref 0.3
ros2 param set /so101_ik_node weight_orient 0.5       # 6D orientation weight
ros2 param set /so101_ik_node vel_max "[2.0, 2.0, ...]" # увеличить velocity limit
ros2 param set /so101_ik_node gripper_mode free       # разморозить gripper (как до фикса)
ros2 param set /so101_ik_node weight_gripper 10.0     # сильнее тянуть gripper к q_start
```

Полный список параметров — в `app/so101-ik-node.cpp` (`declare_parameter` в конструкторе).

| Параметр | Default | Назначение |
|----------|---------|------------|
| `publish_rate` | 50.0 | Hz, timer для joint_states |
| `interp_steps` | 100 | шагов интерполяции (quintic) |
| `ik_max_iter` | 500 | макс итераций IK |
| `perturb_scales` | [0.05, 0.15, 0.30, 0.50, 0.80] | амплитуды multistart |
| `auto_flip_restart` | true | flip shoulder_pan для целей за base |
| `ik_eps` | 1e-4 | сходимость по weighted norm |
| `ik_eps_visual` | 0.01 | порог 🟢 |
| `ik_dt`, `ik_dt_min`, `ik_dt_max` | 0.1, 0.05, 0.5 | адаптивный шаг |
| `ik_damp` | 1e-6 | DLS damping базовый |
| `ik_man_k`, `ik_man_thresh` | 1e-4, 1e-4 | адаптивное демпфирование по μ |
| `weight_jc`, `weight_man`, `weight_prev`, `weight_pref`, `weight_drift` | 0.5, 0.2, 0.3, 0.5, 0.1 | null-space компоненты |
| `preferred_q` | [0.0, -0.3, 1.0, -0.7, 0.0, 0.0] | предпочтительная поза |
| `weight_pos`, `weight_orient` | 1.0, 0.5 | 6D task weights |
| `vel_max` | [1.0×6] rad/s | velocity limit per joint |
| `mu_boundary` | 5.0 | CBF joint bound coefficient |
| `weight_collision` | 0.5 | self-collision barrier weight |
| `collision_d_min` | 0.005 м | минимальное расстояние между линками |
| `collision_margin` | 0.010 м | margin для активации barrier |
| `ik_stuck_patience` | 40 | итераций до объявления "stuck" |
| `reach_padding` | 0.01 | padding для workspace projection |
| `init_target_x/y/z` | 0.20, 0.0, 0.15 | начальная IK-поза при старте |
| `gripper_mode` | `"hold"` | "hold" фиксирует gripper (q[5]) в q_start через null-space; "free" — как раньше (управляется DLS) |
| `weight_gripper` | 5.0 | сила null-space фиксации gripper (только при hold) |

---

## 8. Известные ограничения

1. **Drift-free criterion неэффективен при `weight_drift = 0.1`** (Phase 1.2 deferred). На closed-loop траектории в XZ-плоскости (`circle-xz`) wrist_flex дрифтит ~26° за один оборот. Причина: конкурирует с `weight_prev` за тот же вектор `(q_start − q)`. **TODO**: пересмотреть — убрать `weight_prev` или поднять `weight_drift` до 1.0+.
2. **B4 regression**: target `[-0.300, 0.0, 0.100]` (за base_link) flip-restart занимает ~3x больше времени из-за `vel_max = 1.0 rad/s`. Решается runtime-увеличением `vel_max` до 2.0+ или `ik_max_iter` до 1000.
3. **6D orientation control слабый при не-identity target**. При `weight_orient = 1.0` 5° rotation тест показал pos err 0.08, rot err 1.14 rad. Причина: для SO-101 6DOF вблизи границ workspace 6×6 Jacobian сингулярный → solver застревает. **Workaround**: упростить target pose (меньший rotation offset), или увеличить `ik_max_iter`.
4. **Sphere approximation** для collision detection — bounding spheres вместо capsules. Точная collision detection требует FCL/hpp-fcl (отложено).
5. **Numerical gradient** для self-collision — finite differences с h=1e-3. Шумный у сингулярностей.
6. **`preferred_q` подобран под SO-101**. Для другой морфологии манипулятора нужно пересмотреть.
7. **Joint-origin как link center** для sphere collision check. Sphere radius покрывает бо́льшую область → более консервативная оценка.
8. **Task priority IK**: position (gripper_link) — главная задача, orientation (gripper_frame_link) — вторичная в null-space. Это **гарантирует**, что при вращении кисти позиция `gripper_link` остаётся на месте (т.к. она на оси `wrist_roll`). Если ориентация требует позы вне досягаемости — она «жертвуется», позиция сохраняется. `gripper_mode=hold` (по умолчанию) фиксирует q[5] (`moving_jaw`) через null-space criterion; `gripper_mode=free` отключает фиксацию (использовать только при добавлении отдельного управления gripper).
9. **`gripper_frame_link` смещён от оси `wrist_roll`** на 9.8 см по Z и 7.9 мм по X. Использование `gripper_link` (на оси) для position task исключает эффект «вращение кисти смещает кончик».

---

## 9. Структура проекта

```
rviz2-docker/
├── Dockerfile                 # COPY app + scripts, сборка Pinocchio
├── README.md                  # этот файл
├── app/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── so101-ik.cpp            # offline-пример без ROS
│   ├── so101-ik-node.cpp       # основной IK-узел (6D + collision)
│   ├── launch/
│   │   └── so101_ik.launch.py  # robot_state_publisher + ik + rviz
│   └── config/
│       └── so101.rviz          # конфиг RViz (Interact активен по умолчанию)
└── scripts/
    ├── run_all_tests.sh        # 27 тестов A/B/C/D (6D PoseStamped)
    ├── run_drift_tests.py      # closed-loop drift тест (с --publish-dt)
    └── teleop_keyboard.py      # standalone Python node для ручного управления
```

URDF и ассеты подгружаются в `/workspace/SO-ARM100/...` на этапе сборки образа (см. `Dockerfile`, шаг `git clone https://github.com/TheRobotStudio/SO-ARM100.git`).
