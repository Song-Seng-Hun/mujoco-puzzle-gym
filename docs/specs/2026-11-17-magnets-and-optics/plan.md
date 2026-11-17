# Plan — 자석과 빛

Spec: [spec.md](spec.md) · Intent: [intent.md](intent.md) · 가지: `optics/magnets-and-light`

## 단계 0 — 탐침 (끝남, 2026-11-17)

부품을 쓰기 전에 조합으로 되는지 방으로 재는 단계(R97의 레시피). 결과는 intent.md의 표에 있다.
탐침 방은 스크래치패드에만 두고 싣지 않는다 — 싣는 것은 그 결과를 고정하는 테스트(R104)다.

## 파일 순서

| 순서 | 파일 | R |
|---|---|---|
| 1 | `src/mujoco_puzzle_gym/parts/device/magnet.py` | R98 |
| 2 | `tests/test_magnet.py` | R98, R104 |
| 3 | `presets/parts/device/magnet.yaml`, `presets/puzzles/tabletop/fetch_the_steel.yaml` | R102 |
| 4 | `src/mujoco_puzzle_gym/optics.py` (광선 추적 + 광학 부품 공통) | R99 |
| 5 | `parts/device/optics.py` (`mirror`, 뒤에 `splitter`·`filter`), `parts/input/photocell.py` | R99~R101 |
| 6 | `tests/test_optics.py` | R99~R101, R104 |
| 7 | `presets/puzzles/light/aim_the_mirror.yaml`, `long_way_round.yaml` | R102 |
| 8 | `docs/parts-guide{,.en,.ja}.md`, `docs/bodies{,.en,.ja}.md`, `examples/`, `tests/test_examples.py` | R103 |

틀은 베껴 쓴다: 힘을 걸고 되돌리는 방식은 `trap/wind.py`(`xfrc_applied` 더하고 빼기), 광선은
`parts/input/laser.py`(`mj_ray`, 기하 id → 부품), 실행 중 값은 `device/setter.py`.

## 증거 (R마다)

- R98: `tests/test_magnet.py` 5개 + `fetch_the_steel` 20/20 + demo 한 단계 0/5 + 미끼 변위 < 1 mm.
- R99: `tests/test_optics.py` 6개 + `aim_the_mirror` 20/20 + 판 안 밟으면 0/5 + `--all-bodies`.
- R100·R101: `long_way_round` 20/20 + 짧은 길만으로는 0/5. 선택이 성립하지 않으면 숫자와 함께 철회.
- R103: `tests/test_guide.py`(세 언어), `tests/test_capability_gate.py`, `tests/test_examples.py`.
- 싣기 전: 빠른 스위트 전체, 그다음 느린 스위트(26분).

## 위험

- **힘의 안정성**(Codex가 #30에 "unsure"로 표시): 접촉 근처에서 일정한 힘은 떨린다. `hold` 안에서
  힘을 끊는 것으로 먼저 측정하고, 떨리면 거리에 따라 깎는다.
- **선 수 폭발**: 스플리터는 가지를 늘린다. 전체 선 수에 상한을 두고 넘으면 가지를 버린다.
- **거울이 벽이다**: 거울은 몸에 닿으므로 계획 단계에서 칸이 막힌다. `aim_the_mirror`의 길이 막히지
  않는지 `mpg check`의 `order` 단계로 확인한다.
- **팔 방의 이웃 문제**(R92에서 측정): 큐브 둘이 붙어 있으면 그립이 깨진다. `fetch_the_steel`은 미끼를
  **닿는 곳 밖에** 두어 팔이 건드릴 일이 없게 한다.
