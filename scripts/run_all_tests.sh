#!/bin/bash
# Полный набор тестов для /target_pose (SO-101 IK, 6D PoseStamped).
#
# Использование: запустить стенд в другом терминале
#   ros2 launch so101_ik so101_ik.launch.py
# затем в контейнере выполнить
#   bash scripts/run_all_tests.sh
#
# Между отдельными тестами: 3 с
# Внутри серий (D2, D3):     1 с
# Все тесты используют identity quaternion (w=1, x=y=z=0) — backward-compat с 3D.

set -e

PUB="ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped"
SLEEP=3
INNER=1

# Identity orientation as YAML shorthand for readability
IDQ="orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"

(
echo "=== Group A: внутри workspace (identity orient) ===" && \
$PUB "{header: {frame_id: base_link}, pose: {position: {x: 0.250, y: 0.050, z: 0.100}, $IDQ}}" && sleep $SLEEP && \
$PUB "{header: {frame_id: base_link}, pose: {position: {x: 0.350, y: 0.050, z: 0.100}, $IDQ}}" && sleep $SLEEP && \
$PUB "{header: {frame_id: base_link}, pose: {position: {x: 0.250, y: -0.200, z: 0.100}, $IDQ}}" && sleep $SLEEP && \
$PUB "{header: {frame_id: base_link}, pose: {position: {x: 0.250, y: 0.050, z: 0.300}, $IDQ}}" && sleep $SLEEP && \
$PUB "{header: {frame_id: base_link}, pose: {position: {x: 0.270, y: 0.070, z: 0.120}, $IDQ}}" && sleep $SLEEP && \
$PUB "{header: {frame_id: base_link}, pose: {position: {x: 0.450, y: 0.000, z: 0.050}, $IDQ}}" && sleep $SLEEP && \

echo "=== Group B: за пределами workspace (проекция) ===" && \
$PUB "{header: {frame_id: base_link}, pose: {position: {x: 1.000, y: 0.000, z: 0.000}, $IDQ}}" && sleep $SLEEP && \
$PUB "{header: {frame_id: base_link}, pose: {position: {x: 0.000, y: -0.600, z: 0.100}, $IDQ}}" && sleep $SLEEP && \
$PUB "{header: {frame_id: base_link}, pose: {position: {x: 0.000, y: 0.000, z: 1.500}, $IDQ}}" && sleep $SLEEP && \
$PUB "{header: {frame_id: base_link}, pose: {position: {x: -0.300, y: 0.000, z: 0.100}, $IDQ}}" && sleep $SLEEP && \
$PUB "{header: {frame_id: base_link}, pose: {position: {x: 0.800, y: 0.500, z: 0.400}, $IDQ}}" && sleep $SLEEP && \

echo "=== Group C: у границы workspace ===" && \
$PUB "{header: {frame_id: base_link}, pose: {position: {x: 0.480, y: 0.000, z: 0.200}, $IDQ}}" && sleep $SLEEP && \
$PUB "{header: {frame_id: base_link}, pose: {position: {x: 0.500, y: 0.000, z: 0.200}, $IDQ}}" && sleep $SLEEP && \

echo "=== Group D (single): возврат к init-позе ===" && \
$PUB "{header: {frame_id: base_link}, pose: {position: {x: 0.200, y: 0.000, z: 0.150}, $IDQ}}" && sleep $SLEEP && \

echo "=== D2 series: плавное смещение по X (внутренний sleep=1) ===" && \
for x in 0.220 0.240 0.260 0.280 0.300; do
  $PUB "{header: {frame_id: base_link}, pose: {position: {x: $x, y: 0.000, z: 0.150}, $IDQ}}"
  sleep $INNER
done && sleep $SLEEP && \

echo "=== D3 series: плавное смещение по Y (внутренний sleep=1) ===" && \
for y in 0.050 0.100 0.150 0.200 0.250 -0.100 -0.200 -0.300; do
  $PUB "{header: {frame_id: base_link}, pose: {position: {x: 0.250, y: $y, z: 0.150}, $IDQ}}"
  sleep $INNER
done && \
echo "=== DONE ==="
)
