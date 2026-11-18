# 도전 — 방마다 비어 있는 칸 하나

이 저장소에는 방 프리셋 **44개**가 있고, 그중 **학습된 정책의 기록이 있는 방은 네 개**입니다.
나머지 41개의 "학습된 정책" 칸은 **비어 있습니다.** 아래 표가 그 빈 칸입니다.

정답 스크립트는 모든 방을 풉니다(저장소가 매번 20시드로 확인합니다). 무작위 정책은 거의 모든 방에서
0입니다. 그 사이가 당신 알고리즘의 자리입니다.

한 줄만 먼저 보고 가셔도 됩니다:

> `puzzles/tabletop/pick_and_place` — PPO로 **25 M 스텝, 성공률 0.00**.
> 방의 정답 스크립트를 복제하면 **1.7 M 스텝에 1.00**.
> 당신 알고리즘은 몇 스텝에 얼마입니까?

<p align="center"><img src="../assets/challenge-hammer-lose.gif" width="380" alt="프레스가 내려와 자물쇠를 때리고 자물쇠가 어두운 붉은색으로 바뀐다. 자물쇠에서 문으로 깔린 케이블은 그대로지만, 코인을 더 주워도 문은 열리지 않고 네 발 로봇이 닫힌 문 앞에 머문다."></p>

<p align="center"><b>지는 판입니다.</b> 프레스가 자물쇠를 때리면(붉게 변합니다) 코인을 세던 계수기가 그 판 내내 0을 말하고,
자물쇠에서 문까지 깔린 케이블이 전할 신호가 사라집니다 — 문은 <b>영원히</b> 열리지 않습니다.</p>

<p align="center"><img src="../assets/challenge-hammer-win.gif" width="300" alt="같은 방을 이기는 판: 코인 세 개를 제때 모아 문이 열리고, 뒤늦게 프레스가 내려와도 이미 지나간 문은 열린 채로 남는다."></p>

<p align="center">이기는 판은 이렇습니다 — 28초에 문을 열어 두면, 40초에 프레스가 내려와도 <b>이미 늦었습니다</b>(문은 <code>oneshot</code>, 한 번 열리면 고정).</p>

## 규약 — 숫자 하나, 고정 시드, 선언된 예산

비교할 수 있어야 도전입니다. 이 저장소에서 같은 방의 성공률이 **0.00과 0.52로 갈린 적이
있습니다**(시드만 달랐습니다). 그래서 세 가지를 같이 적습니다.

```bash
uv run mpg train examples/task_<방>.yaml --minutes <예산> --id mine
uv run mpg eval runs/mine --episodes 100            # held-out 시드 1,000,000~1,000,099
uv run mpg eval runs/mine --episodes 100 --policy random   # 같은 시드의 기준선
```

- **숫자 하나**: `mpg eval`의 `success_rate`. 학습 중 성공률이 아닙니다.
- **고정 시드**: 평가 시드는 학습이 절대 쓰지 않는 구간입니다(`EVAL_SEED_BASE`).
- **선언된 예산**: 벽시계 분 **그리고** 환경 스텝. 둘 다 `runs/<id>/summary.json`에 있습니다.

## 방 갤러리 — 어떤 조건들이 있나

열 개 방, 네 바디, 열 가지 기구입니다. 숫자는 전부 이 저장소가 잰 것입니다: **정답 스크립트**는 20시드 검사의 평균 소요, **무작위**는 같은 방에 균일 난수를 20시드 돌린 결과, **학습된 정책**은 기록이 있으면 그 숫자입니다.

| | 바디 | 화면에서 벌어지는 일 | 정답 스크립트 | 무작위 | 학습된 정책 |
|---|---|---|---:|---:|---|
| <img src="../assets/challenge-hammer-lose.gif" width="260" alt="coins/beat_the_hammer"> | 네 발 | `coins/beat_the_hammer` — 40초 뒤 프레스가 코인 자물쇠를 부순다 — 그 전에 문을 열어야 한다 | 45.6s | 0/20 | **비어 있음** |
| <img src="../assets/gallery/tabletop_pick_by_hand.gif" width="260" alt="tabletop/pick_by_hand"> | 팔 | `tabletop/pick_by_hand` — 용접 없이 **손가락 둘로 눌러** 블록을 들어 상자에 넣는다 | 11.66s | 0/20 | 0.52 |
| <img src="../assets/gallery/light_split_the_beam.gif" width="260" alt="light/split_the_beam"> | 오리 | `light/split_the_beam` — 한 줄기 빛을 분광기로 나눠 두 광전지에 **몫**을 맞춘다 | 11.1s | 1/20 | **비어 있음** |
| <img src="../assets/gallery/ball_bowling.gif" width="260" alt="ball/bowling"> | 오리 | `ball/bowling` — 공을 밀어 핀 셋 중 하나를 쓰러뜨리면 문이 열린다 | 23.82s | 0/20 | **비어 있음** |
| <img src="../assets/gallery/roller_glide_under.gif" width="260" alt="roller/glide_under"> | 오리(롤러) | `roller/glide_under` — 롤러를 달고 **낮은 바 밑을 미끄러져** 지나간다 | 14.66s | 0/20 | **비어 있음** |
| <img src="../assets/gallery/wide_pit_detour.gif" width="260" alt="wide/pit_detour"> | 네 발 | `wide/pit_detour` — 5×4 m 방에서 구덩이를 돌아간다 — 큰 바디가 설 수 있는 곳만 길이다 | 30.78s | 0/20 | **비어 있음** |
| <img src="../assets/gallery/tabletop_build_a_tower.gif" width="260" alt="tabletop/build_a_tower"> | 팔 | `tabletop/build_a_tower` — 블록 둘을 **쌓는다**(쌓임을 읽는 부품이 판정) | 10.6s | 0/20 | **비어 있음** |
| <img src="../assets/gallery/tabletop_conveyor_triage.gif" width="260" alt="tabletop/conveyor_triage"> | 팔 | `tabletop/conveyor_triage` — 두 벨트, 두 통 — 색이 다른 블록을 한 번에 하나씩 분류 | 18.8s | 0/20 | **비어 있음** |
| <img src="../assets/gallery/circuit_fuse_repair.gif" width="260" alt="circuit/fuse_repair"> | 오리 | `circuit/fuse_repair` — 끊긴 회로를 **퓨즈를 물어다 끼워** 되살린다 | 26.73s | 0/20 | **비어 있음** |
| <img src="../assets/gallery/tow_sled_delivery.gif" width="260" alt="tow/sled_delivery"> | 오리 | `tow/sled_delivery` — 썰매를 로프로 **끌어서** 배달한다 | 19.36s | 1/20 | **비어 있음** |

## 사다리

별은 의견이 아니라 측정에서 나옵니다 — **무작위 정책이 세 번 이상 푸는 방은 ★**, 한두 번 풀거나
정답 스크립트가 12초 미만이면 ★★, 25초 미만이면 ★★★, 그 이상이면 ★★★★입니다. 바디는 그 방의 task
파일이 정한 바디입니다.

### go2

| | 방 | 정답 스크립트 | 무작위 | 학습된 정책 |
|---|---|---:|---:|---|
| ★★★ | `wide/switch_door` | 24.58s | 0/20 | **0.96** (네 발, PPO 30분) |
| ★★★★ | `wide/pit_detour` | 30.78s | 0/20 | **비어 있음** |
| ★★★★ | `coins/beat_the_hammer` | 45.6s | 0/20 | **비어 있음** |

### microduck

| | 방 | 정답 스크립트 | 무작위 | 학습된 정책 |
|---|---|---:|---:|---|
| ★★★ | `light/aim_the_mirror` | 33.4s | 1/20 | **비어 있음** |
| ★ | `roller/brake_strip` | 15.82s | 4/20 | **비어 있음** |
| ★ | `roller/mode_switch_route` | 16.0s | 3/20 | **비어 있음** |
| ★★ | `roller/speed_gate` | 10.4s | 1/20 | **비어 있음** |
| ★★ | `light/split_the_beam` | 11.1s | 1/20 | **비어 있음** |
| ★★ | `circuit/clock_door` | 14.23s | 1/20 | **비어 있음** |
| ★★ | `tow/sled_delivery` | 19.36s | 1/20 | **비어 있음** |
| ★★★ | `roller/crosswind` | 12.46s | 0/20 | **비어 있음** |
| ★★★ | `budget/two_openings` | 13.2s | 0/20 | **비어 있음** |
| ★★★ | `circuit/press_relay` | 13.27s | 0/20 | **비어 있음** |
| ★★★ | `roller/glide_under` | 14.66s | 0/20 | **비어 있음** |
| ★★★ | `ball/ball_ramp` | 14.78s | 0/20 | **비어 있음** |
| ★★★ | `roller/timed_corridor` | 16.38s | 0/20 | **비어 있음** |
| ★★★ | `tow/cleat_hold` | 17.25s | 0/20 | **비어 있음** |
| ★★★ | `tow/drop_and_go` | 17.29s | 0/20 | **비어 있음** |
| ★★★ | `circuit/jam_the_turret` | 17.83s | 0/20 | **비어 있음** |
| ★★★ | `tow/npc_leash` | 20.23s | 0/20 | **비어 있음** |
| ★★★ | `tow/tethered_start` | 20.32s | 0/20 | **비어 있음** |
| ★★★ | `ball/mini_soccer` | 22.0s | 0/20 | **비어 있음** |
| ★★★ | `ball/bowling` | 23.82s | 0/20 | **비어 있음** |
| ★★★ | `circuit/cut_the_lock` | 24.36s | 0/20 | **비어 있음** |
| ★★★ | `roller/one_way_station` | 24.52s | 0/20 | **비어 있음** |
| ★★★★ | `circuit/wrong_wire` | 25.05s | 0/20 | **비어 있음** |
| ★★★★ | `tow/chain_dropoff` | 25.43s | 0/20 | **비어 있음** |
| ★★★★ | `circuit/peck_cable` | 25.73s | 0/20 | **비어 있음** |
| ★★★★ | `tow/pit_bridge` | 26.19s | 0/20 | **비어 있음** |
| ★★★★ | `circuit/fuse_repair` | 26.73s | 0/20 | **비어 있음** |
| ★★★★ | `circuit/sabotage_order` | 29.52s | 0/20 | **비어 있음** |
| ★★★★ | `ball/color_sort` | 30.1s | 0/20 | **비어 있음** |
| ★★★★ | `roller/roll_delivery` | 37.45s | 0/20 | **비어 있음** |
| ★★★★ | `tow/overload` | 49.7s | 0/20 | **비어 있음** |

### so101

| | 방 | 정답 스크립트 | 무작위 | 학습된 정책 |
|---|---|---:|---:|---|
| ★ | `tabletop/reach_target` | 0.6s | 5/20 | **비어 있음** |
| ★★ | `tabletop/reach_the_marker` | 1.44s | 1/20 | 0.93 (PPO 15분, 센서만) |
| ★★ | `tabletop/weigh_it_down` | 7.6s | 0/20 | **비어 있음** |
| ★★ | `tabletop/lever_arm` | 7.8s | 0/20 | **비어 있음** |
| ★★ | `tabletop/pick_and_place` | 8.2s | 0/20 | 1.00 (정답 스크립트 복제, 특권) |
| ★★ | `tabletop/fetch_the_steel` | 9.6s | 0/20 | **비어 있음** |
| ★★ | `tabletop/build_a_tower` | 10.6s | 0/20 | **비어 있음** |
| ★★ | `tabletop/pick_by_hand` | 11.66s | 0/20 | 0.52 (정답 스크립트 복제, 센서만; 세 번에 한 번) |
| ★★★ | `tabletop/dead_wedge` | 13.1s | 0/20 | **비어 있음** |
| ★★★ | `tabletop/conveyor_triage` | 18.8s | 0/20 | **비어 있음** |

## 아직 아무도 못 푼 세 가지

| 무엇 | 측정된 상태 |
|---|---|
| **손가락으로 집어 운반** (`tabletop/pick_by_hand`) | 완주하는 정책이 **있습니다** — 50시드 0.52. 그런데 그 조합이 그런 정책을 내는 건 **세 번에 한 번**이고, 아홉 번 적합을 더 돌려도 바뀌지 않았습니다(분산은 데이터 쪽) |
| **40 ms 액추에이터 지연** | 팔 방 열 개 중 **다섯**이 여기서 무너집니다. 한 가지 원인이 아니라 **넷**입니다: 셋은 `grab`을 끝내지 못하고(용접이 3.04~11.39 N으로 찢기거나, `_pinched`가 끝까지 거짓이어서 접촉 집기가 성립하지 않습니다), 둘만 `goto goal`에 도달해 멈춥니다. 일곱 번의 수정 시도가 전부 다른 방을 깨거나 아무 방도 못 열었습니다. 어느 방이 어느 하위 목표에서 멈추는지는 `uv run python tools/latency_stall.py`가 찍습니다 |
| **단서 없는 순서 추론** (`order_blind`) | 이론적 상한 1/3에 **정확히 붙은 0.29**. 단서를 못 읽는 정책이 낼 수 있는 최선이고, 그 위로 올라간 정책은 없습니다 |

## 알려진 기록

| 방 | 바디 | 관측 | 기록 | 방법 |
|---|---|---|---|---|
| `tabletop/reach_the_marker` | 팔 | **센서만** | **0.93** | PPO 15분, 4.08 M 스텝 |
| `tabletop/pick_and_place` | 팔 | 특권 | **1.00** | 정답 스크립트 복제(`algo: bc`), 1.70 M 스텝 |
| `tabletop/pick_by_hand` | 팔 | **센서만** | **0.52** | 정답 스크립트 복제 + DAgger, 세 번에 한 번 |
| `wide/switch_door` | 네 발 | 특권 | **0.96** | PPO 30분, 2.85 M 스텝 |
| `examples/switch_door.yaml` | 오리 | 특권 | 0.99 | PPO 15분 (예제 방, 프리셋 아님) |

숫자의 출처와 실패한 시도들은 [학습 기록](training-log.md)에 있습니다. 그 문서에는 측정 함정
**열 개**도 같이 적혀 있습니다 — 재는 쪽에서 속는 방법이 그만큼 많았습니다.

## 새 도전 방

**`puzzles/coins/beat_the_hammer`** (위 GIF): 코인 셋을 모으면 문이 열립니다. 그런데 40초 뒤
망치가 내려와 자물쇠를 한 방에 부수고, 부서진 자물쇠는 그 판 내내 0을 읽습니다. 정답 스크립트는
20/20(평균 45.6초), 무작위는 0/20, **퓨즈를 15초로 줄이면 정답 스크립트도 0/5**입니다.
