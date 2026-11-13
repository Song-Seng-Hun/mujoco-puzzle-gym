# Plan: mujoco-puzzle-gym 마일스톤 2 (부품 카탈로그)
Status: approved 2026-11-12 (E2 split into E2a/E2b/E2c: "승인"); approved 2026-11-11 (사용자 "전부 승인": R31-R36 + R2b + review fixes rounds 2-4; coin step moved to E1 as 5b per user "2번으로")
View (read-only copy): (아직 없음)

근거: 같은 폴더의 spec.md(draft), 선행 `../2026-11-10-duck-puzzle/`. 이 문서만 보고도 만들 수 있게 쓴다.

## 0. 전제
- 시작 조건: 다음 셋이 끝난 뒤 그 위에 쌓는다.
  - M1의 PR C(`feat/train-eval`)
  - M1의 PR D(문서, CI)
  - 사용자의 병합
- 브랜치는 PR마다 새로 만든다: `feat/m2-signals`, `feat/m2-devices`, `feat/m2-grab`. push와 PR은 매번 사용자 승인을 받고, 병합은 사용자가 한다.
- 이 spec 폴더는 첫 PR에 `docs/specs/2026-11-11-parts-catalog/`로 옮긴다.
- 경로: 아래 `mujoco_puzzle_gym/...`는 `src/mujoco_puzzle_gym/...`이다.
- 새 의존성은 없을 것으로 본다. MuJoCo 4.0.0의 mocap, `eq_active`, `mj_ray`, `mj_getState`/`mj_setState`를 쓴다.

## Files that change
| File | Change | R |
|---|---|---|
| `mujoco_puzzle_gym/signals.py` | 새로. 채널 버스, 평가 순서(입력→논리→장치), 순환 검사 | R1 |
| `mujoco_puzzle_gym/expr.py` | 새로. `ast` 허용 목록 검사, 상태를 갖는 함수(delay, hold, pulse, latch, count, timer) | R7 |
| `mujoco_puzzle_gym/schema.py` | 새로. 값 스키마(단위, 기본값, 범위, 실행 중 변경 가능), 검사, 마크다운 표 생성 | R3, R23 |
| `mujoco_puzzle_gym/presets/parts/<분류>/*.yaml` | 새로. 프리셋. 분류: input, logic, device, floor, trap, object, beak, npc (spec R3) | R3 |
| `mujoco_puzzle_gym/parts/base.py` | `in/out/enable`, `schema`, 런타임 변경 훅 추가 | R1, R3, R13 |
| `mujoco_puzzle_gym/parts/input/{button,pressure,zone,laser,receptacle,target,checkpoint}.py` | 새로. 소켓·걸이·바구니는 `receptacle` 프리셋 | R4–R6 |
| `mujoco_puzzle_gym/parts/logic/{gate,counter,transceiver}.py` | 새로. M1 `switch.py`는 `button` 프리셋 `floor_momentary`의 별칭으로 남긴다 | R7, R8, R2 |
| `mujoco_puzzle_gym/parts/device/{door,trapdoor,platform,pillar,turntable,conveyor,setter,warp,spring}.py` | 새로. `setter.py`에 `bind`(R13). M1 `door.py`는 이 파일로 옮기고 별칭을 남긴다 | R9, R11–R15 |
| `mujoco_puzzle_gym/parts/trap/{wind,pusher,spinner,pendulum,pit,cannon}.py`, `parts/pool.py` | 새로. 대포는 탄 풀을 디스펜서와 공유 코드로. `cannon_turret` 프리셋과 탄도 정적 점검 | R16 |
| `mujoco_puzzle_gym/parts/floor/tile.py` | 새로. 마찰·기울기(경첩+구동기)·높이(슬라이드+구동기), 신호로 두 값 사이 이동 | R10 |
| `mujoco_puzzle_gym/parts/object/cube.py` | 새로 | R17 |
| `mujoco_puzzle_gym/parts/npc/{rover,behaviors}.py`, `presets/parts/npc/*.yaml` | 새로. 평면 관절 로봇, 행동 규칙 8종, 광선 벽 회피 | R29 |
| `mujoco_puzzle_gym/parts/device/addon_station.py`, `presets/parts/device/addon_roller.yaml`, `bodies/microduck.py`(모드 전환·관절 이름 매핑), 자산 받기(바퀴 메시·`roller.onnx`·`roller_crouch.onnx` 고정 커밋+해시) | 새로/고침. 합친 모델, `set_addon`(높이 → 속도 0 → 충돌·정책), 정책 교체 | R30 |
| `mujoco_puzzle_gym/presets/puzzles/roller/*.yaml` | 새로. 바퀴 퍼즐 조립 프리셋 9종 | R30 |
| `mujoco_puzzle_gym/parts/device/rope.py`, `parts/device/hitch_station.py`, `presets/parts/device/{rope,anchor,hitch_station}.yaml`, `bodies/microduck.py`(`tow_site`) | 새로/고침. 매듭 몸체 + 길이 제한 tendon, 걸이별 `connect` 구속, 채널·패드·부리 연결, 끊어짐, `length` 실행 중 값 | R31 |
| `mujoco_puzzle_gym/presets/puzzles/tow/*.yaml`, `examples/rooms/tow_yard.yaml` | 새로. 견인 퍼즐 조립 프리셋 8종과 예제 방 | R31 |
| `mujoco_puzzle_gym/parts/npc/owner.py`, `parts/input/zone.py`·`parts/input/receptacle.py`(`attach_to`), `presets/puzzles/pet/*.yaml`, `examples/rooms/walkies.yaml` | 새로/고침. 주인 경로·기다리기·명령, 움직이는 구역, 동행 프리셋 8종 | R32 |
| `mujoco_puzzle_gym/parts/logic/source.py`, `parts/logic/wire.py`, `parts/logic/base.py`(`body: true`) | 새로/고침. 신호기 4방식, 보이는 전선과 연결 끊김, 논리 부품 몸체 | R33 |
| `mujoco_puzzle_gym/parts/breakable.py`, `parts/device/destroyer.py`, `parts/logic/relay.py`, `check/circuit.py`, `presets/puzzles/circuit/*.yaml`, `examples/rooms/power_cut.yaml` | 새로. 파괴 속성(원인 4가지, hp, 고치기), 파괴기, 중계기, 회로 그래프 점검, 회로 프리셋 8종 | R34 |
| `mujoco_puzzle_gym/randomize.py`, `env.py`(`sample_layout` 교체, 쉬는 구역), `room.py`(샘플러 스키마, `count`, `holdout`, M1 `random:` 읽기), `check.py`(다시 뽑기 비율), `obs.py`(사용 중 마스크, 연결 비트, `randomized`), `task.py`(`randomize:`), `curriculum.py`(`randomize.<층>` 난이도, 저장본 시작 재뽑기), `evaluate.py`(`--randomize`), `examples/rooms/shuffle_switches.yaml` | 새로/고침. 샘플러 5종, 층별 세기, 다시 뽑기(그래프 순서 + 뽑힌 값 정적 점검), 연결 단서, 평가 전용 값, 뽑힌 방 기록 | R35 |
| `mujoco_puzzle_gym/parts/floor/coin.py`, `puzzles/coins/coin_lock.yaml`, `obs.py`(`coins`), `rewards.py`(`coin_pick`), `check.py`(코인 순서, 닿지 않는 코인), `examples/rooms/coin_room.yaml` | 새로/고침. 코인, 묶음 채널, 코인 잠금 프리셋 | R36 |
| `mujoco_puzzle_gym/body.py`(`capabilities`), `bodies/microduck.py`(기본 능력), `schema.py`(`requires_body`), `check.py`(능력 정적 점검) | 고침. 능력 태그 자리와 검사만, 새 몸 없음 | R2b |
| `mujoco_puzzle_gym/parts/device/rope_door.py` | 새로. 중력으로 닫히는 수동 문(R31 `cleat_hold`) | R31 |
| `mujoco_puzzle_gym/parts/beak/{grabbable,pull_cord,dial,drawer,lever,dispenser,scale}.py`, `presets/parts/beak/*.yaml` | 새로. 집을 물체 공통 클래스(질량 한도, 깨짐), 부리 장치 | R28 |
| `mujoco_puzzle_gym/markers.py`, `mujoco_puzzle_gym/presets/markers/4x4_50/*.png` | 새로. 마커 번호 배정, 텍스처 박기, 번호 검사 | R26 |
| `mujoco_puzzle_gym/parts/input/gaze.py`, `obs.py`의 `markers_seen` | 새로. 시야각·거리·면 방향·가림 판정, dwell | R27 |
| `mujoco_puzzle_gym/room.py` | 프리셋 읽기·덮어쓰기, `links:`→채널 변환, 바닥 타일, `stages:`, `demo:`, `checkpoint_states:` | R2, R3, R10, R14, R20, R21 |
| `mujoco_puzzle_gym/logic.py` → `walkmap.py` | 이름 변경. 격자 길 찾기는 그대로 | R20 |
| `mujoco_puzzle_gym/solver.py` | `demo:` 하위 목표 실행기(goto, press, grab, place, wait) | R20 |
| `mujoco_puzzle_gym/check.py` | 정적 단계와 동적 단계, 저장본 기록 | R20 |
| `mujoco_puzzle_gym/snapshot.py` | 새로. `mj_getState(mjSTATE_INTEGRATION)` 전체 상태(`eq_active` 포함) + 부품·신호·어댑터 이력 저장·복원 | R6, R21 |
| `mujoco_puzzle_gym/bodies/base.py` | `step(data, cmd, on_substep=None)`(물리 스텝 콜백), `teleport`, `kick`, `grab_site`, `view_frame`, `history_state`/`load_history` 추상 메서드 | R14, R15, R18, R6 |
| `mujoco_puzzle_gym/bodies/microduck.py` | 위 메서드 구현, 부리 끝 사이트, 머리 명령 연결 | R14, R15, R18, R19 |
| `mujoco_puzzle_gym/env.py`, `obs.py`, `rewards.py` | 행동 확장, 관측 `checkpoints`, 체크포인트 재시작, 다음 목표를 신호 그래프로 계산 | R6, R19, R21 |
| `mujoco_puzzle_gym/curriculum.py` | 새로. forward/reverse/none, 구간 표집, 기록 | R21 |
| `mujoco_puzzle_gym/{train,evaluate}.py`, `cli.py` | `--curriculum`, `--advance-at`, `--replay`, `eval --full/--segments`, `mpg parts` | R21, R22, R3 |
| `mujoco_puzzle_gym/registry.py` | `register_logic` 추가 | R7 |
| `examples/rooms/*.yaml`, `examples/broken/*.yaml` | 예제 6개 이상, 망가진 방 4개 | R20, R24 |
| `docs/parts-guide.md` | 새로 | R23 |
| `AGENTS.md`, `skills/mujoco-puzzle-gym/SKILL.md` | 가이드 연결, 프리셋 조립 순서 | R25 |
| `tests/test_*.py` | Proof 표의 테스트 | 각 R |

## Order of work
PR E1 `feat/m2-signals` (신호와 입력·논리, 흐름 부품)
1. `schema.py`와 `Part` 기반 확장. M1 부품 3종에 스키마를 단다. M1 테스트가 녹색인지 확인한다(R2).
1b. 몸 능력 태그(R2b): `BodyAdapter.capabilities` → 마이크로덕 기본 능력 → 부품 `requires_body` 스키마 → `mpg check` 정적 단계에서 빠진 능력 거절. 이후 단계의 부품은 필요한 능력을 이 자리에 선언한다.
2. `signals.py`를 만들고 `links:` 변환을 넣는다. `test_signals.py`.
   - 구현 기록 (2026-11-11, 커밋 f02e924·eef4d5c·fc0e88a, 100 passed incl. slow, 예제 방 solver 20/20):
     - `Capabilities`는 `bodies/base.py`에 두었다. 마이크로덕 기본 능력은 `walk`, `beak_grab`, `max_lift_mass` 0.05.
     - 문 `open_dir`은 1 또는 -1만 받는다.
     - 상태 없는 순환은 `Room.load`에서 `RoomError`로 거절한다. `mpg check`는 `ok: false`와 `error`에 순환 경로를 낸다. 정적 단계(`failed` 목록)로 옮기는 일은 6단계에서 한다.
     - `logic.py` 이름 바꾸기는 미뤘다. 부품 갱신 순서는 입력 → 논리 → 장치(R1).
     - 남은 일: `check_order`는 문을 여는 출처를 채널에 직접 쓰는 부품으로만 찾는다. 3단계에서 논리 부품을 거쳐 거슬러 올라가게 고치고 시험을 더한다(게이트를 거쳐 열리는 문이 있는 방이 order 단계를 통과).
     - 가이드의 "점검 3단계" 설명과 R2b "M2 예제 방 통과" 시험은 8단계에서 한다.
3. `expr.py`와 게이트, 카운터(R8b), 트랜시버. `test_logic.py`(거절 사례 포함), `test_counter.py`.
4. 버튼 3종(바닥형만. 벽형은 E3에서), 센서(압력판은 E3의 큐브 뒤에 시험), 체크포인트와 `snapshot.py`, 어댑터의 `teleport`와 이력 저장·복원. 소켓은 `receptacle` 부품(R5)으로 여기서 만들고, E1 시험은 시험 전용 자유 상자로 한다. 물고 있는 물체 제외 시험은 15단계에서 한다. `snapshot.py`는 처음부터 `mjSTATE_INTEGRATION`을 쓴다. 같은 단계에서 `BodyAdapter.step(..., on_substep=None)` 콜백과 `env.advance`의 부품 `substep` 묶음을 넣는다(압력판은 쓰지 않음).
   - 구현 기록 3·4단계 (2026-11-11, 커밋 caf9a19…1c277ad, 빠른 229 passed·전체 240 passed, 예제 방 solver 20/20):
     - 바닥 버튼·구역 센서는 위치로 판정한다(몸통 중심이 판 위, 물체는 중심이 `max_height` 아래). 접촉 판정 판은 발을 3.7 mm 들어 올렸다. 압력판만 4 mm 솟은 판의 접촉력을 쓴다.
     - 상자 `eject`는 물건을 벽 너머로 던진다(spec 문구 수정). 모양은 `box`·`plate`만, 걸이는 16b. `accept`의 마커 번호는 R26 뒤. `require`는 `{count: 2}` 형태도 받는다.
     - 버튼 `mount`는 E3 전까지 `floor`만. 버튼 기본 `trigger: body`.
     - 게이트는 채널 없는 `op` 형태(`{op: delay, seconds: 1}`)도 받는다. `xor`는 `or`와 같은 우선순위. 점 이름(`laps.count`)은 채널 이름이고 산술은 없다.
     - 체크포인트 복원 뒤 재생은 1초에 약 2 mm 어긋난다(이력 복원 없이는 4.3 cm). spec R6에 알려진 오차로 적음.
     - `respawn`은 task.yaml `env.respawn: start|checkpoint`. `checkpoint`면 `respawn_penalty`(-1)가 자동으로 붙는다.
     - 물체로만 켜지는 센서는 역할 `sensor`. 6단계 `demo: place` 전까지 그 뒤의 문은 순서 점검에서 실패한다. M1 `links:`는 이것에서 시작할 수 없고 `in:`을 쓴다.
     - `check_order`가 논리 부품을 거슬러 문 입력을 추적한다(c1df057, 2단계에서 넘어온 수정).
     - 한 번, 어느 테스트인지 모르는 빠른 테스트 실패 1건(이후 6회 재현 없음). 다시 나오면 테스트 이름을 남긴다.
5. 문 3종, 바닥 타일 부품(R9, R10), 워프(R14).
   - 타일 바닥에서 M1-R3 걷기 시험을 다시 한다.
   - 기울기·단차·마찰을 나눠 시험해 기본 허용 범위를 정한다(Open question 5).
   - 구현 기록 5–5d단계 (2026-11-11, HEAD 5fcf2ea, 빠른 317 passed(데스크 확인)·전체 334 passed(작업자 보고)):
     - 5단계 커밋 031e866: 문, 바닥 타일, 트랩도어, 워프 발판, `stages:`(R9, R10, R14). 예제 방 `switch_door` solver 20/20(작업자 보고). `examples/rooms/` 폴더를 이 단계에서 만들었다.
     - Open question 5 측정값과 결정은 spec에 적었다(`max_step` 0.01 m, `max_slope` 5°, 기울기 ±13°, 높이 ±0.1 m, 마찰 0.2 이상, 타일 geom `priority` 1).
     - 받아들인 차이: 여닫이 문은 벽(움직이는 판)으로 흉내 낸다. 문 속도 값은 선택이다. 워프는 최소 구현이다. 풀이 순서 격자는 트랩도어를 무시한다(6단계 할 일). 넘어짐 높이는 `BodyAdapter.state(data, floor_z=0.0)`와 `fall_height`로 잰다.
5d. 공과 골대(R38): Open question 10 측정(밀 수 있는 질량) → `ball` 스키마(탄성은 solref, 튀는 비율 시험) → `goal_net`(scored/count/accepts) → `demo:`의 `push` → 정적 점검(입구 크기, 닿지 않는 칸) → 관측 `balls`, 보상 `ball_progress` → 프리셋 4종 → 예제 `mini_soccer.yaml` → `test_ball.py`.
   - 구현 기록 (2026-11-11, 커밋 82b2232·4b68e46·0a47496, 병합 6ec9a4e, 뒤이어 5fcf2ea):
     - 공, 핀(`pin`), 골대와 부품 정적 점검 단계 `parts`(82b2232). solver 공 밀기, 관측 `balls`, 보상 `ball_progress`(4b68e46). `mini_soccer` 예제와 프리셋(0a47496). 5fcf2ea에서 공이 워프·구역·높은 바닥을 넘어 동작하게 하고 `ball_ramp`를 더했다.
     - solver: `mini_soccer` 19/20(시드 13 시간 초과, 데스크 확인). spec 기준 18/20 이상은 넘는다.
     - Open question 10 측정값과 공·골대 기본값은 spec에 적었다.
     - 받아들인 차이: `push` 명령 대신 `push.py` 자동 밀기를 쓴다(6단계 `push`가 대신함). solver는 앞에 벽이 있으면 돈다. 공 프리셋은 방 파일 통째다(8단계 프리셋 로더 전까지). 골대 벽은 튀지 않는 벽(반발 없음, 공보다 접촉 우선)이라 들어간 공이 나오지 않는다. `ball_ramp`는 R37 경사로(3.4°)로 만든다.
5c. 글자 지도(R37, spec 확정 뒤): `room.py`에 `map`/`cell`/`legend` 읽기 → 타일 바닥 생성(같은 높이 칸 합치기, 경사로) → 바닥 없음 낙하 판정 → 마이크로덕 `max_step`·`max_slope`(Open question 5 측정값) → `logic.py` 격자에 높이·경사 이동 조건 → 정적 점검의 칸 좌표 이유 → 스크립트 해답 경로 → 예제 `two_rooms_map.yaml`, `stairs_map.yaml` → `test_map.py`.
   - 구현 기록 (2026-11-11, 커밋 dba3ece):
     - solver: `two_rooms_map`, `stairs_map` 20/20(작업자 보고).
     - 받아들인 차이: `max_step`은 오를 때와 내려갈 때 모두 적용한다. 풀이 순서 격자 칸은 약 0.033 m다. 지도 점검(벽·바닥 없음 칸)은 `order` 단계에 있다. 방 값 `on_fall: reset`을 지원한다(기본 `fail`, 빠지면 에피소드를 끝내지 않고 되돌림). 지도 방과 `stages:`는 아직 함께 쓰지 않는다.
5b. 코인(R36, 사용자 2026-11-11 "2번으로"로 E3에서 E1로 당김): `coin` 부품과 묶음 채널 → `coin_lock` 프리셋(R8b 카운터 재사용) → 관측 `coins`, 보상 `coin_pick` → `check.py` 코인 순서와 닿지 않는 코인 거절 → R35 샘플러 연결 → 예제 `coin_room.yaml`.
   - 구현 기록 (2026-11-11, 커밋 52be311, 병합 16ceba8, 뒤이어 5fcf2ea):
     - 5fcf2ea에서 코인이 워프·구역·높은 바닥을 넘어 동작하게 했다.
     - solver: `coin_room` 20/20(데스크 확인).
     - 받아들인 차이: `coin_lock`은 8단계 프리셋 로더 전까지 부품 종류다. 묶음은 숨은 `coin_group` 부품이 맡는다. `.picked` 채널은 펄스 사이를 띄워 낸다. 관측 `coins`는 최대 3개(`COIN_OBS_K`). 코인 위치 샘플러는 16g로 미뤘다.
6. `check.py` 두 단계와 `solver.py`의 `demo:`(goto, press, wait). 망가진 방 4개.
   - 5–5d에서 넘어온 일: `demo:`의 `push` 명령(`push.py` 자동 밀기를 대신함). 풀이 순서 격자가 트랩도어와 구덩이를 피하게 한다. 공 밀기 여유를 늘린다(`mini_soccer` 19/20).
   - 구현 기록 (2026-11-11, 커밋 a87748d·816f1b4·60fce07, 병합 4990f57, 빠른 371 passed(병합 뒤, 데스크 확인)·전체 363/364 passed(브랜치별, 작업자 보고)):
     - solver: 예제 `switch_door`, `coin_room`, `mini_soccer`, `stairs_map`, `two_rooms_map` 모두 20/20(데스크 확인). 프리셋 `ball_ramp` 20/20, `color_sort` 19/20, `bowling` 18/20.
     - 받아들인 차이: 방을 읽을 때 나던 오류는 정적 단계 `values`, `channels`, `demo`가 되었다(출력에는 `error` 키를 남김). 정적 단계 `overlap`을 새로 넣었다(규칙은 spec R20). `demo:`는 중간에 멈출 수 있고 그 뒤는 격자 해답이 이어 간다. 골대·핀 목표는 `demo:` 없이 자동으로 민다. `push.py`는 이제 `push` 명령의 구현이고 골대에서 0.3 m보다 멀면 더 빨리 민다. `examples/switch_door.yaml`에는 `demo:`를 넣지 않았다(시험이 그 방을 복사해 씀). 일반 "바닥이 구멍" 훅을 두었고 지금은 트랩도어만 쓴다. 체크포인트 저장본은 `runs/check/<room>/checkpoints/`에 `.pkl`로 남는다.
     - 넘어가는 일: 8단계(`bowling` 핀 밀기, 가이드 점검 단계)와 12단계(구덩이 구멍 훅)에 적었다.
7. `curriculum.py`, 관측 `checkpoints`, `eval --full/--segments`. 가짜 정책 테스트.
   - 구현 기록 (2026-11-11, 커밋 4ef10fd·d927e81·cbfe027, 병합 2172f3d, 빠른 432 passed(병합 뒤, 데스크 확인)·전체 421 passed(브랜치, 작업자 보고)):
     - 짧은 실행(깃발 2개 임시 방): `--curriculum forward` 1분에 구간이 6초·9초에 열림, `eval --full --segments` 8회에서 전체 1.0, 구간 0.875/1.0/1.0, 이어 붙임 실패 0(작업자 보고).
     - 받아들인 차이: 체크포인트 깃발이 목표 순서에 들어간다(전에는 `progress` 보상이 깃발을 건너뛰어 첫 구간이 0에 머묾). 깃발이 있는 모든 방의 풀이 경로가 바뀌었고 예제 5개는 그대로 20/20(데스크 확인). 저장본이 없으면 `mpg check`를 먼저 돌리라는 오류를 낸다. `--full`은 지금 일반 평가와 같은 에피소드다. 깃발마다 저장본이 하나라 구간 에피소드 시작 상태가 같고 시드는 잡음만 바꾼다. 구간 성공은 `info["segment_success"]`, `info["success"]`는 여전히 골이다. `CurriculumVec`는 train.py에 둔다.
     - 넘어가는 일: 12단계 난이도, 16g `checkpoint_states:`, 9단계 앞 `mpg check`, guide.en.md·guide.ja.md 번역은 E1 PR.
7b. 마커(R26)와 시선 센서(R27). `view_frame`은 `head_camera`를 쓴다. 머리 숙이기 없이 몸을 돌려 보는 경우만 시험한다.
   - 구현 기록 (2026-11-11, 커밋 69b4d50·8fb10ae, 병합 ebe5c1f, 빠른 432 passed·1 skipped(병합 뒤, 데스크 확인)·전체 426 passed(브랜치, 작업자 보고)):
     - 예제 5개 solver 20/20(병합 뒤, 데스크 확인). 마커 PNG 50개는 `tools/make_aruco_markers.py`로 만들었고 OpenCV `generateImageMarker`와 같음(작업자가 시험 밖에서 확인).
     - OpenCV는 개발 의존성으로 넣지 않는다(사용자 2026-11-11 "응 승인"). 검출 시험은 cv2가 없으면 건너뛰고, 렌더한 마커 칸을 직접 읽는 시험이 대신 돈다.
     - 받아들인 차이: 마커 오류 3가지는 `values` 단계에서 거절한다. 디스펜서가 아직 없어 복제 예외는 `marker_clone_of` 속성으로 둔다. 마커 부품 9종(button, pressure, checkpoint, door, swing_door, trapdoor, receptacle, warp, floor_tile), M1 `switch`는 8단계 별칭 뒤, 큐브·플랫폼은 그 단계에서. `size`는 검은 사각형이고 흰 테두리 한 칸을 더한다. 문은 기본 `face: both`. 방 값 `gaze: {fov: 60, max_dist: 2.0, max_tilt: 70, dwell: 0.5}`(서 있는 몸이 바닥 마커를 보도록 70°). 표시된 부품 자신과 몸의 geom은 광선을 막지 않는다. `.seen`은 숨은 `_gaze` 부품이 쓰고 읽는 부품이 있을 때만 생긴다. `markers_seen` 슬롯은 (seen, x, y, nx, ny) 진행 방향 기준이고 센서 항목으로 센다.
     - 넘어가는 일: `look` 명령은 16b(사용자 2026-11-11 "응 승인"). README 문장은 E1 PR. guide.en.md·guide.ja.md 번역은 E1 PR.
7c. 실물 센서 관측(R39): (5b·5d에서 넘어온 일: task.yaml에서 관측 항목에 값을 주는 방법, 예: 코인 수, 공 수.) Open question 11 측정(카메라 렌더 비용) → `obs.py`에 `rangefinder`(어댑터가 머리 사이트에 센서 추가)와 `camera`(환경당 렌더러 1개, 결정 스텝에만 렌더) → 관측 공간 Box/Dict 전환 → `train.py`에서 Dict면 `MultiInputPolicy` → `critic_obs`용 비대칭 정책 클래스(가치망 입력만 따로) → R35 `body` 잡음 연결 → `mpg bench` 카메라 수치 → `test_sensor_obs.py`. README·가이드 문장은 구현 확인 뒤 같은 PR에서 고친다.
   - 구현 기록 (2026-11-11, 커밋 54db102·d628d19·5bbecc9, 병합 9e5e516, 빠른 371 passed(병합 뒤, 데스크 확인)·전체 363/364 passed(브랜치별, 작업자 보고)):
     - Open question 11 측정값과 결정(64×64, 10 Hz, 그림자·반사 끔)은 spec과 `docs/bench.md` 3차 측정에 적었다.
     - 짧은 실행: `mpg train --minutes 1`을 센서만 설정과 `critic_obs` 설정으로 각각 돌려 정책을 저장했고, `mpg eval`이 돌았다(성공 0, 예상대로).
     - 받아들인 차이: spec R39 "구현에서 정한 것"에 적었다(카메라 렌즈 축, 수평 광선, uint8 영상, 카메라 항목 하나, sac 거절, `noise`와 `randomize: {body: k}`).
     - 7b가 다시 쓰도록 `BodyAdapter.view_camera`, `view_frame`, `add_rangefinders`를 더했다.
     - 남은 일: README 문장 고침은 E1 PR에서 한다.
8. 프리셋 폴더, `mpg parts`, 가이드 1차, 예제 방(버튼·논리, 문, 체크포인트 4개). M1 `switch` → `oneshot` 별칭(spec R2).
   - 5b·5d에서 넘어온 일: `coin_lock`과 공 프리셋을 프리셋 로더로 잇는다. 문서에 `ball_ramp`를 더한다.
   - 6단계에서 넘어온 일: 프리셋을 예제 방으로 옮길 때 `bowling` 핀 밀기를 다듬는다(18/20이 R20 기준선). `docs/guide.md`에 새 점검 단계를 적는다. `overlap` 단계가 문(`role == "gate"`)을 모든 벽에서 빼는 것을 자기 벽만 빼도록 좁힌다(사용자 2026-11-11 "응").
   - 구현 기록 (2026-11-11, 커밋 3b80bfe·7fbc715·5fe807b, 병합 358ac50, 빠른 499 passed·1 skipped(병합 뒤, 데스크 확인)·전체 529 passed(브랜치, 작업자 보고)):
     - solver: 새 예제 `logic_lock`, `doors`, `four_flags` 20/20, 프리셋 `bowling` 20/20(병합 뒤, 데스크 확인). 기존 예제 5개·`ball_ramp` 20/20, `color_sort` 19/20(작업자 보고). `bowling`은 핀 0.3 m 너머를 겨누게 해 18/20 → 20/20(평균 21.3 s → 24.0 s).
     - 받아들인 차이: 조립 프리셋은 방 맨 위 `preset: puzzles/<topic>/<name>`(이름 하나나 목록)으로 읽고, 부품은 이름으로 합치며 `links`는 더한다. `coin_lock`은 `need: all` 코드가 필요해 부품 종류로 남기고 `puzzles/coins/coin_lock` 프리셋이 그것을 쓴다. 공 프리셋은 방 파일 그대로 `presets/puzzles/ball/`에 둔다. `overlap`은 문이 자기 줄의 벽(5° 안 평행, 중심선 위)에서만 빠진다. 이로 드러난 `ball_ramp` 문 1 cm 파고듦을 고쳤다(폭 0.5 → 0.48). `switch`는 버튼 하위 종류라 마커와 버튼 색을 갖는다. 바뀐 값은 task.resolved.yaml `room_parts`에 남긴다. `doors`는 oneshot 버튼만 쓴다. `mpg parts --plugin <module>`을 더했다.
     - 넘어가는 일: 13단계 `mpg demo` 풀이 MP4(R24), 12b단계 momentary 버튼 → oneshot 문 계획, 17단계 AGENTS.md·SKILL.md(R25), E1 PR에서 README·guide/parts en·ja 번역과 parts.md "다음 부품" 단락.
9. 데모 방 `four_flags`에서 forward 학습 1회. 목표는 Open question 3에서 정한 값이다. 학습 조정은 subagent에 맡긴다. (7단계에서 넘어온 일: 학습 전에 `mpg check`로 깃발 저장본을 먼저 만든다.)
   - 구현 기록 (2026-11-11, 커밋 bb6f0c6, 병합 e22d521): 학습 설정 `examples/task_four_flags.yaml`(PPO, 7 env, 에피소드 60 s, 관측에 `checkpoints`), `forward`·`none`을 같은 때 2시간씩(작업자 보고).
     - 결과(평가 100회, 결정적): `forward` 전체 1.00·구간 모두 1.00·이어 붙임 실패 0·평균 19.7 s, `none` 1.00·1.00·0·15.3 s. 목표 달성, 다시 돌리지 않음. 기록은 `docs/training-log.md`.
     - 해석: 깃발이 풀이 순서라 `target_pos`와 진행 보상이 길을 알려 주어 `none`도 약 1분에 80%를 넘었다. 1분 시험에서 `forward`는 결정적 평가 첫 구간이 0.02(학습 중 0.66–0.8)였고 원인은 보지 않았다.
     - 넘어가는 일: 단계별 학습 이득 비교는 12단계 난이도 방에서. 정책 파일은 저장소 `runs/ff-fwd-120`, `runs/ff-none-120`(git 밖).

PR E2는 셋으로 나눈다(2026-11-12): E2a `feat/m2-devices` 10·11단계와 그 예제 방, E2b `feat/m2-traps` 12단계와 함정 예제 방, E2c `feat/m2-npc` 12b·13단계.
10. 플랫폼·엘리베이터, 승강 기둥, 회전판(관절과 위치·속도 액추에이터, R11·R12). 컨베이어는 띠 위의 몸과 물체 위치를 옮긴다(OQ1 (c)). 몸이 띠와 플랫폼 위에서 3초 넘어지지 않는지는 slow 시험으로 기록한다.
   - 구현 기록 (2026-11-12, 브랜치 `feat/m2-devices`, 빠른 571 passed·1 skipped(작업자 보고)):
     - 플랫폼 `path`는 `pos`에서 잰 `[dx, dy, z]`다(z는 바닥 위 윗면 높이). 프리셋(엘리베이터)이 경로를 가질 수 있고 `random`이 경로째 옮긴다.
     - 미끄럼 관절은 경로가 움직이는 축에만 둔다(수직 엘리베이터는 z 하나). 움직이지 않는 축은 관절 없이 고정이다.
     - 구동기 목표를 몸 스텝(20 ms)마다 한 번 옮기면 판이 덜컥거려 0.3 m/s, 3초에 큐브가 3.1 cm 미끄러졌다. 물리 스텝마다 목표를 이어 옮기도록(`substep`) 바꿔 0.0 cm가 되었다(`tests/test_platform.py`).
     - `run: on_signal`은 상승 순간마다 한 번 운행한다(once: 끝까지, pingpong: 반대 끝까지, loop: 한 바퀴). 엘리베이터 프리셋은 `pingpong` + `on_signal`이다.
     - 컨베이어 띠와 스프링 패드는 바닥에 그린 것이고 충돌하지 않는다. 옮기기·쏘기는 `World.requests`의 `move`·`kick`으로 env가 몸 스텝 뒤에 한다. 몸을 옮기려고 `BodyAdapter.shift`도 더했다.
     - 계획(풀이 순서, 해답)은 플랫폼이 쓸고 가는 칸과 회전판 칸, 솟을 기둥 칸을 막힌 칸으로 본다.
     - slow 기록(통과 기준 아님): 플랫폼 위 몸 0.1 m/s 3초 넘어지지 않음(판 0.296 m, 몸 0.295 m 이동), 0.3 m/s 넘어지지 않음(판 0.888 m, 몸 0.800 m). 띠 위 몸 0.1 m/s 3초 넘어지지 않음(0.300 m), 0.2 m/s 넘어지지 않음(0.600 m).
11. 스프링과 어댑터의 `kick`, 설정 변경기.
   - 구현 기록 (2026-11-12, 브랜치 `feat/m2-devices`):
     - 실행 중 변경 값은 부품의 `live`에 두고 부품이 매 스텝 거기서 읽는다. `set_value`로 바꾸고, `reset`이 방 값으로 되돌리며, 저장본(`Snapshot.values`)이 담는다.
     - `bind`는 설정 변경기만의 값이 아니라 모든 부품의 공통 값이다(`bind: {<값>: {from, map}}`, spec R13 예시 형태). 버스가 그 부품의 `update` 직전에 적용한다. 수 값만 묶을 수 있다(스프링 `velocity` 같은 벡터는 거절).
     - 설정 변경기는 자기 채널(적용 중이면 1)을 내고 대상이 그 채널을 읽게 해서, 같은 몸 스텝에 대상보다 먼저 갱신된다. 순서를 두지 않았을 때 스프링이 바뀌기 전 속도로 쏘았다.
     - 실행 중 변경이 아닌 값(문 `width`), 범위 밖 값, `bind`와 설정 변경기가 함께 바꾸는 값은 방을 읽을 때 `values` 단계에서 거절한다(별도 정적 단계를 두지 않음). `add_room_parts` 오류의 단계를 `values`로 정했다.
     - 바닥 타일의 `tilt`·`height`를 설정 변경기나 `bind`가 바꾸면 그 관절을 만든다(`claim_value`). 압력판 `min_mass`도 `live`에서 읽는다.
     - R13 Check의 "바람 세기"는 바람(R16)이 E2b에서 들어올 때 시험한다.
     - 스프링 측정: 큐브 발사 속도는 설정값과 같고(±5% 안), 착지 거리는 탄도 계산의 1.002배·1.009배(속도 [1, 0, 2], [0.6, 0.4, 1.5]).
     - 기본 `spring`은 `trigger: any`, 프리셋 `device/spring_pad`는 `trigger: object`다(몸 발사는 학습 분포 밖).
     - 예제 `examples/rooms/lift_spring.yaml`(버튼 → 문, 엘리베이터, 설정 변경기로 스프링 켜기) `mpg check` solver 20/20.
   - 리뷰 반영 (2026-11-12, claude-mac-reviewer, 빠른 575 passed·1 skipped(데스크 확인)):
     - 받음: 높은 칸 위 플랫폼이 reset마다 z를 잃던 것(`place`가 x, y만 씀), R13 전에 저장한 저장본에 `values`가 없어 복원이 깨지던 것(방 값으로 되돌림), 타일 `on_tilt`·`on_height`의 `bind`가 관절을 만들지 않던 것, 복원 뒤 계획이 지난 `place`의 이동량을 쓰던 것(`xy - base_xy`로 계산). 각각 시험을 더했다.
     - 버림: 설정 변경기가 자기 입력 판의 `min_mass`를 바꾸면 loop 오류가 나는 것(드문 설계이고 안전하게 막힌다), 입력 단계 대상이 한 몸 스텝 늦게 바뀌는 것(20 ms, 고장 아님).
     - spec R11 문구를 코드에 맞췄다(움직이는 축에만 관절, `path`는 `pos` 기준 오프셋; 사용자 "승인").
12. 물체 보관 선반 코드(`pool.py`, 대포·디스펜서 공용)를 먼저 만든다. 함정 6종(대포 포함). 탄에 맞을 때 넘어지는 세기를 재서 대포 기본값을 정한다. `curriculum.py`에 난이도 단계(R21 `difficulty`)를 더한다. (6단계에서 넘어온 일: 구덩이가 "바닥이 구멍" 훅을 써서 풀이 경로가 피하게 한다.) `cannon_turret`, R13 `bind`, 과녁(R5 `target`)과 탄도 정적 점검(조준 채널 범위 기준)도 여기서 만든다. 레버 칸 기준 점검의 시험은 16b에서 한다. (7단계에서 넘어온 일: 난이도는 `Curriculum.levels`에 `{"segment": i, "difficulty": k}`로 붙인다. `eval --full`이 가장 어려운 난이도를 고르게 한다.)
   - 구현 기록 (2026-11-12, 브랜치 `feat/m2-traps`, 빠른 620 passed·1 skipped(작업자 보고), slow 넘어짐 기록 5개·`trap_run` 점검 통과):
     - `parts/pool.py`: 미리 만든 공 `pool`개를 방 아래(z −5 m)에 세워 두고 돌려 쓴다. 세워 둘 때는 충돌 비트 0과 `body_gravcomp` 1, 쏠 때 둘 다 켠다. 둘 다 `snapshot.MODEL_FIELDS`에 있고, 어느 공이 언제 나갔는지는 부품 `save_state`에 담는다. 공 geom은 충돌하는 상태로 짓고 `reset`에서 세운다. 짓자마자 비트를 0으로 두면 걷는 몸 컨트롤러가 처음 한 번 부르는 `mj_setConst`가 몸체 충돌 비트를 0으로 굳혀 공이 바닥을 뚫었다.
     - 함정 6종(`wind`, `pusher`, `spinner`, `pendulum`, `pit`, `cannon`)과 프리셋 `trap/crosswind`, `pusher`, `spinning_bar`, `pendulum`, `pit`, `cannon`, `cannon_turret`. 공통 값 `enabled`(실행 중 변경)와 `power`. 구덩이, 회전 막대·진자가 쓸고 가는 자리, 밀대가 나오는 자리는 "바닥이 구멍" 훅으로 계획이 피한다. 미끄러운·끈적한 바닥 프리셋은 전부터 있었다.
     - 탄은 `World.kinds`에서 `shell`이라 버튼·압력판·구역의 `trigger`에 들지 않는다. 과녁과 R34용 훅은 `pool.Impacts`(탄이 닿기 시작한 물리 스텝과 그 직전 속도)다.
     - 넘어짐 기록(서 있는 몸, 0.4 m, 옆/앞 각 3번, 넘어짐/맞음, slow `test_knockdown_record`):

       | 탄 kg | 3 m/s | 4 m/s | 5 m/s | 6 m/s | 8 m/s |
       |---|---|---|---|---|---|
       | 0.02 | 0/3 0/3 | 0/3 0/3 | 0/3 0/3 | 0/3 0/3 | 0/3 0/3 |
       | 0.05 | 0/3 0/3 | 0/3 0/3 | 0/3 0/3 | 0/3 0/3 | 0/3 0/3 |
       | 0.1 | 0/3 0/3 | 0/3 0/3 | 0/3 0/3 | 0/3 3/3 | 3/3 3/3 |
       | 0.2 | 0/3 0/3 | 1/3 3/3 | 3/3 3/3 | 3/3 3/3 | 3/3 3/3 |
       | 0.4 | 0/3 3/3 | 3/3 3/3 | 3/3 3/3 | 3/3 3/3 | 3/3 3/3 |

       (리뷰 반영 때 겨눔을 포구 끝에서 계산하도록 고친 뒤 다시 잰 표. 고치기 전에는 0.2 kg 4 m/s가 6/6이었다.) 0.5 N·s 이하는 한 번도 넘어지지 않았고, 0.6 N·s(0.1 kg, 6 m/s, 앞)부터 넘어지기 시작한다. 대포 기본값은 0.2 kg, 5 m/s(1.0 N·s, 6/6 넘어짐). 걷는 중에는 더 약하다: 0.4 N·s 탄을 2.5초마다 쏜 예제 방 초안은 solver 14/20(6번 넘어짐)이라 예제에서 대포를 뺐다.
     - 과녁 `target`(R5): 탄만 세고 `min_speed` 이상일 때 `<이름>.hits`를 올린다. `mode: latch`(기본) 또는 `pulse`. 포탑 정적 점검은 `parts` 단계에서 `aim: signal` 대포마다 `bind` 출력 범위의 겨눔 격자(0.25° 간격, 원천이 `channel_cells`를 주면 그 칸)로 포물선을 계산하고, `spread` > 0이면 원뿔 가장자리 8방향 모두 판에 맞아야 한다. 시험: 맞음, 뒤에 있음, 너무 멂, 원뿔 전체, 점검이 맞다는 겨눔이 시뮬레이션에서도 맞음.
     - 난이도 단계(R21): task.yaml `difficulty:` 행은 `steps: n` 또는 `at_step: k`. 실행 중 변경이나 에피소드 시작 값만, `bind` 대상은 거절. 매 `reset`이 단계 값을 넣고 시작·체크포인트 저장본에도 같은 값을 넣는다. `Curriculum.levels`가 `{"segment", "difficulty"}` 쌍이고 수준 이름은 `<구간>@d<단계>`. `curriculum.order` 기본 `difficulty_first`. 뽑기 없이(평가, 직접 `reset`) 마지막 단계, `mpg eval --difficulty k`로 고른다(`--onnx`와는 함께 못 씀). `info["difficulty"]`와 에피소드 기록에 남는다.
     - 스키마에 `Field(episode=True)`(에피소드 시작 때만 변경)를 더했다. 지금 이 표시를 쓰는 기본 부품 값은 없다.
     - 예제 `examples/rooms/trap_run.yaml`(구덩이, 회전 막대, 약한 옆바람) `mpg check` solver 20/20.
     - 계획과 다른 점:
       - 대포 세기는 `power`(발사 속도, R16)다. R21 예시의 `cannon_a.speed`는 `cannon_a.power`로 쓴다. `enabled`는 모든 함정의 값이다.
       - `shell_mass`는 지을 때만 정한다(실행 중·에피소드 시작 변경 없음). MuJoCo가 자유 물체의 질량 행렬을 캐시해서, 실행 중에 바꾸면 떨어지는 가속도가 39 m/s²로 틀렸다.
       - `in`이 없는 함정은 늘 작동한다. 구덩이에는 `power`가 없고, `in`이 있으면 꺼진 동안 뚜껑이 덮인다.
       - `at_body`·`lead`는 닿지 않는 거리면 45°로 쏜다. 포신 그림은 겨눔을 따라 돌지 않는다.
       - 레버 칸 기준 점검은 `channel_cells` 훅만 두고 시험은 16b에서 한다(계획대로).
       - R21 `param: randomize.<층>`(무작위화 세기 단계)은 R35를 기다린다.
   - 리뷰 반영 (2026-11-12, 사용자 승인 4건):
     - 난이도 단계 값은 `_reset_sim`에서 방 값을 되돌린 직후, 부품 `reset`과 계획 전에 넣는다. 타일 마찰이 첫 스텝부터 단계 값이고, 진자 계획이 단계 `power`로 쓸고 간다. 관절이 없는 타일의 `tilt`·`height` 단계는 task를 읽을 때 거절한다(`in`, 설정 변경기, `bind`를 주라는 안내). 깃발 저장본에서 시작하는 구간은 복원 뒤 단계 값을 다시 넣는다(타일 마찰은 저장본 값에서 이어 바뀐다).
     - `in`이 없는 구덩이는 늘 열려 있어서 `enable`, `enabled: 0`, `enabled`의 설정 변경기·`bind`·난이도 단계를 거절한다.
     - 시험 추가: `mpg eval --difficulty`와 `--full`의 마지막 단계, `--onnx`와 `--difficulty` 거절, `CurriculumVec`의 `@d<k>` 기록, `mode: none` + 난이도, 움직이는 표적에 `aim: lead`.
     - `lead` 시험에서 드러난 것: 겨눔을 포구 중심에서 계산해 탄이 실제로 나가는 포구 끝(0.13 m 앞, 0.09 m 위)과 어긋났다. 사거리 끝 근처(4 m/s, 1.5 m)에서 0.28 m 넘게 빗나갔다. 포구 끝을 고정점 반복으로 넣어 1 cm 안으로 맞췄다. 그래서 넘어짐 표를 다시 쟀고 기본 속도를 4 → 5 m/s로 바꿨다.
12b. NPC 로봇(R29): 몸체와 행동 8종, `demo:`의 `herd`, 예제 `herd.yaml`. `lure`와 `shy_gaze`는 E3의 잡기와 시선이 필요하므로 E3에서 시험한다.
   - 구현 기록 (2026-11-12, 브랜치 `feat/m2-npc`, 빠른 666 passed·1 skipped, slow `herd`·`warp_hop` 3 passed(작업자 보고)):
     - `parts/npc/rover.py`(부품 `npc`, 종류 `npc`)와 `behaviors.py`(행동 규칙 8개). 뿌리 몸체에 x·y 미끄럼 관절과 z 경첩, 속도 구동기 3개. 몸통 상자는 바닥 위 6 mm에 떠 있고 z 관절이 없다(바퀴·코는 그림만). 질량이 있어 다른 몸·물체와 부딪힌다. 마커(R26)는 `/chassis` 윗면.
     - 값: `behavior`, `size`, `mass`, `speed`(기본 0.08 m/s), `accel`, `sense`, `leash`, `keep`, `reach`, `path`, `button`, `bait`, `awake`, `asleep`, `invert`. 행동마다 필요한 값(`path`, `button`, `bait`)이 없으면 방을 읽을 때 거절한다.
     - 벽 피하기는 앞 0°, ±35°, ±70° 광선 5개(1 m). 앞이 막히면 여유가 큰 쪽부터 고른다. `leash`는 미끄럼 관절 범위(딱딱한 `solref_limit`)라 차거나 밀어도 벗어나지 않는다(2 m/s 차기에도 5 mm 안).
     - 신호 `<이름>.moving`, `<이름>.on`(버튼 위), 관측 `npcs`(로봇마다 상대 x·y, 몸 기준 상대 속도 x·y, moving, acting).
     - `demo:`의 `herd <npc> <부품 | x y>`와 `herd.py`: flee는 감지 원을 돌아 뒤쪽 선 위로 간 뒤 천천히 밀고, follow는 목표 너머 `keep`만큼 앞에서 이끈다(lure는 E3부터). 감지 범위 밖으로 처지면 기다린다.
     - 시험 `tests/test_npc.py` 25개(그중 slow 1개: `herd.yaml` 점검). lure는 E3 잡기 뒤에 시험한다.
     - 예제 `examples/rooms/herd.yaml`(도망 로봇을 구석 버튼으로 몰아 문 열기) `mpg check` solver 20/20, 풀이 25.5–30.9 s(평균 27.5 s, 에피소드 40 s).
     - 계획과 다른 점:
       - spec의 "최고 속도"와 공통 값 `speed`는 같은 값 하나다(`speed` = 최고 속도). `accel`이 그 속도까지 오르는 빠르기다.
       - `shy_gaze`는 E3를 기다리지 않는다. R27 시선 센서(`<이름>.seen`, 머무름 시간)가 이미 있어 그것을 읽고 지금 시험한다. 보지 않을 때 몸 쪽으로 `keep`까지 다가온다.
       - `lure`는 E3 잡기가 채울 `World.held`를 읽는다. 지금은 늘 비어 있어 제자리에 있다.
       - `seek_button`은 `in`이 없으면 늘 버튼으로 가고, `in`이 꺼지면 그 자리에 선다(집으로 돌아가지 않음).
       - `leash`는 로봇 중심의 범위다.
       - 압력판(`pressure`)은 `trigger`가 npc를 세면 판 안 로봇의 질량을 더한다.
       - `snapshot.MODEL_FIELDS`에 `jnt_range`를 더했다(배치 `random`이 `leash` 관절 범위를 옮긴다).
       - 워프·컨베이어·스프링은 로봇의 미끄럼 관절 값을 옮긴다(`shift`·`kick`·`teleport`).
       - 몰 수 있는 로봇(flee, follow)의 `leash` 안(없으면 어디든)에 있고 npc를 세는 입력(`objects` 목록 없음)은 계획에서 `switch`로 본다. 그래야 풀이 순서가 그 입력을 `herd`로 누를 수 있다.
       - `seek_button` 로봇의 `button`은 계획에서 로봇의 `in`이 켜지면 켜진 것으로 보고(`plan_driver`), 몸을 그 위로 보내지 않는다(`sensor`).
       - 정적 점검 `parts` 단계에 경고(실패 아님)를 더했다: `speed` 0.16 m/s 이상(몸 걷기 속도).
   - 리뷰 반영 (2026-11-12, 사용자 승인 6건, 빠른 675 passed·1 skipped, slow `herd`·`warp_hop` 3 passed):
     - `seek_button`의 협동 문: 로봇의 `button`이 어떤 경우에도 `switch`가 되지 않아 "스위치 A → 로봇(`in: [A]`) → 판 → 문" 방이 `order` 단계에서 실패했다. 입력 부품에 `plan_driver`를 두어, 계획에서 로봇의 `in`이 켜지면(`in` 없으면 늘) 그 입력도 켜진 것으로 본다. 로봇의 `in`이 순서상 뒤에 계산되는 채널일 수 있어, 그런 방에서만 `plan_channels`가 값이 멈출 때까지 다시 돈다. 몸이 누를 수 있는 버튼(`trigger: any`)이 목표여도 `sensor`로 내려 몸을 그 위로 보내지 않는다(몸과 로봇이 버튼 가운데서 부딪히지 않게).
     - 로봇이 켤 수 없는 입력(`objects` 목록이 있는 구역)은 몰기용 `switch`로 올리지 않는다.
     - `lure`를 몰 수 있는 행동(`HERDABLE`)에서 뺐다. `World.held`가 E3까지 비어 있어 `herd <lure 로봇>`이 끝나지 않는다. spec R29의 "`follow`/`lure`는 끌고 간다"는 E3에서 다시 넣는다(R 줄은 그대로).
     - 시험 추가: 컨베이어(`push` move → `Npc.shift`), 워프 판(`trigger: npc` → `Npc.teleport`).
     - 높은 바닥: `floor_tile`의 `height` 위에 둔 로봇이 바닥 0에 지어져 타일에 34 mm 묻히고 순찰을 거의 못 했다(점검 단계는 통과시켰다). 글자 지도 칸은 방을 지을 때 이미 올려 두어 문제가 없었다. 공처럼 env가 넣는 `floor_z`를 받아 `reset`에서 몸체 높이(`body_pos` z)로 쓰고, 벽 피하기 광선 높이도 그만큼 올린다. 위아래 관절이 없어서 계단은 로봇에게 벽이다.
13. 예제 방(플랫폼·워프, 함정, 몰기), 가이드 갱신, `mpg bench`를 다시 잰다.
   - 구현 기록 (2026-11-12, 브랜치 `feat/m2-npc`):
     - 예제: 플랫폼은 `lift_spring.yaml`, 함정은 `trap_run.yaml`(E2a·E2b), 몰기 `herd.yaml`, 워프 `warp_hop.yaml`(막힌 벽 너머로 워프, solver 20/20, 평균 9.53 s)을 새로 더했다. 워프 예제에서 되돌림 판(`reset_pad`)은 뺐다. 계획이 되돌림 판을 피하지 않아 그 위를 걸어 8/20이었다.
     - 가이드: `guide*.md` 예제 목록, `parts-guide*.md` 값 표(카탈로그에서 다시 만듦), NPC 로봇 절, `herd` 하위 목표, `parts` 단계 경고. README 마일스톤 2 줄을 완료로 표시했다.
     - `mpg bench` 전후(`switch_door`, 환경 스텝/초, 두 번씩): 전 `35466cd` 단일 572·562, 병렬 12 4,546·4,499 → 후 단일 569·557, 병렬 12 4,515·4,523. 잡음 안이라 새 부품이 없는 방은 느려지지 않았다. 표는 `docs/bench.md` 4차 측정.

PR E3 `feat/m2-grab` (큐브, 잡기, 부리 누르기)
14. Open question 2를 확인한다: 상위 학습 설정의 머리 명령 범위, 머리를 숙이고 걷는 시험.
   - 구현 기록 (2026-11-12, 브랜치 `feat/m2-grab`): 범위는 넓고(neck·head pitch ±1.10 rad) 머리를 숙여 걷기·돌기·벽에 붙기 모두 넘어짐 0/10. 측정값과 결정(`head` 행동 → 숙임 정도 t, 명령 t × (−1.1, +1.1) rad)은 spec Open question 2에 적었다. Risks의 가슴 누름 벽 버튼 대안은 필요 없다.
   - 잡기 위치 측정(15·16단계 설계용): 바닥 물체는 닿지 않는다. 높이 0.17–0.19 m 받침대 앞 모서리에 둔 4 cm 큐브(옆면 손잡이 1.5 cm, 오리 쪽)는 머리를 든 채 걸어 들어가면 부리 끝이 손잡이 4 cm 안에 8/8 들어오고 큐브는 거의 밀리지 않는다(0–1.6 cm). 받침대 0.15 m는 몸이 큐브를 밀어 4 cm 안에 1/8. 그래서 `grab_radius` 기본값을 0.04 m로 두고, 예제 방은 글자 지도의 높은 바닥 칸을 받침대로 쓴다.
15. 큐브, 압력판·소켓 시험, 부리 끝 사이트, weld 잡기. 잡은 채 저장한 체크포인트를 복원하면 계속 잡고 있는지 시험한다(`eq_active`).
   - 구현 기록 (2026-11-12, 브랜치 `feat/m2-grab`): `parts/object/cube.py`, `grab.py`(숨은 입력 부품 `_beak`), `BodyAdapter.grab_site()`(마이크로덕 `mouth_tip`). 시험은 `tests/test_grab.py` 11개(질량·손잡이 면, 압력판 0.02/0.04 kg 대 `min_mass` 0.03, 놓은 뒤에만 세는 소켓, 0.3 m 넘게 옮기는 동안 2 cm 안·놓으면 떨어짐, 반경 밖, 너무 무거움, 힘 한도로 놓침과 다시 잡기, 잡은 채 저장본 복원, 관측 `beak`).
   - 계획과 다른 점:
     - spec R17 Check는 `tests/test_cube.py`인데, 큐브 시험을 잡기 시험과 한 파일(`tests/test_grab.py`)에 두었다. 큐브만 따로 볼 시험이 적고 도우미(`to_beak`)를 같이 쓰기 때문이다.
     - 머리와 큐브의 접촉을 뺐다(`spec.add_exclude`). 손잡이가 `jaw_soft`에 파고든 채 weld를 켜면 구속 힘이 약 26 N으로 튀어 바로 놓쳤다. 뺀 뒤 들고 걷는 동안 부리와의 거리 변화 0.5 mm 아래, 구속 힘 약 0.35 N.
     - 큐브 손잡이 기본은 옆면 넷(`+x`, `-x`, `+y`, `-y`)이다. 윗면 손잡이는 부리가 닿지 않는다(14단계 측정).
     - `_beak`은 방 순서 맨 앞의 입력 부품이다. 같은 몸 단계에서 센서가 이번 `World.held`를 보게 하려는 것이다.
     - 방에 큐브가 있으면, 큐브가 켤 수 있는 입력 센서(압력판, 소켓, 물체 버튼·구역)를 풀이 순서에서 발판으로 친다(R20 순서 계산이 큐브 방을 풀 수 있게).
16. 행동 확장(`head`, `grab`), 벽 버튼, `demo:`의 grab과 place.
   - 구현 기록 (2026-11-12, 브랜치 `feat/m2-grab`): task `actions: [move]` + `head`·`grab`(이 순서, 각 1칸), 벽 버튼(`mount: wall`, `height`, `facing`), `demo:` `grab <큐브>`·`place <큐브> <부품>`(`carry.py`), 관측 `beak`. 시험 `tests/test_actions.py`(행동 크기 4가지, `check_env`, 값 전달, 잘못된 `actions` 거절), `tests/test_buttons.py` 벽형 종류 3 × enable 2와 높이·옆 어긋남, R19 slow 시험(판 중심 0.15 m, 해답이 머리를 숙여 누름, 시드 0–5 6/6). 예제 `examples/rooms/cube_plate.yaml`(받침대 큐브 → 압력판 → 문): `mpg check` 해답 20/20, 평균 26.4 s.
   - 계획과 다른 점:
     - 벽 버튼 누름 거리 `PRESS_DEPTH`를 6 cm로 두었다. 벽에 붙어 머리를 숙이면 가슴이 먼저 닿아 부리 끝이 판에서 4–5 cm 떨어진다(14단계 측정 3.7–5.4 cm). spec R4의 "닿으면 눌림"을 이 거리 안으로 읽었다.
     - task에 `head`·`grab` 행동이 없을 때 스크립트 해답은 머리 숙임과 잡기 명령을 env에 직접 넣는다(`env.scripted_action`). 그래서 기본 `actions: [move]` 방도 `mpg check` 해답 단계가 큐브와 벽 버튼을 다룬다.
     - `place`는 큐브가 목표 가운데 3 cm 안에 오면 서서 흔들림이 멎기를 기다린 뒤 놓고(걷는 중에 놓으면 머리 흔들림으로 큐브가 십여 cm 튐), 놓은 뒤 몸을 돌려 큐브에서 0.35 m 물러난다(다음 걷기가 큐브를 차서 판에서 밀어냄). 벽 버튼도 누른 뒤 0.3 m 물러난다(벽에 붙은 채로는 길찾기가 시작점을 못 찾음).
     - 예제 방은 글자 지도의 높은 바닥 칸(0.18 m)을 받침대로 쓴다(14단계 측정).
   - `lure` 되살림(12b단계에서 넘어온 일): 잡기가 `World.held`를 채우므로 `lure`를 `HERDABLE`에 다시 넣었다(spec R29 "`follow`/`lure`는 끌고 간다"). 시험: 미끼를 물고 있는 동안만 다가와 미끼에서 `keep`만큼 떨어져 서고, 놓으면 멈춤(fast). `grab bait` → `herd bot pad` 방이 시드 3개 모두 성공(slow, 같은 방 `mpg check` 20/20). 가이드 3개 언어의 lure 줄과 몰기 설명, 값 표, README 진행 줄(시뮬레이션 단순화 문구)을 고쳤다.
   - `lure` 계획과 다른 점:
     - 미끼가 잡을 수 있는 물체(손잡이가 있는 부품)인 `lure`만 몰 수 있다(`behaviors.can_herd`). 아니면 `herd`를 거절하고 로봇용 입력도 발판으로 올리지 않는다.
     - 몰기에서 `lure`는 몸이 아니라 문 미끼가 이끄는 점으로 가고, 기다릴 때 로봇 쪽으로 돌지 않고 선다(돌면 미끼가 휘둘림). `follow`·`lure` 모두 로봇이 `sense` 밖으로 처지면 그쪽으로 걸어 돌아간다(전에는 서서 기다려 끝나지 않았다).
   - 14–16단계 리뷰 반영 (2026-11-12, 사용자 승인 5건, 빠른 714 passed·1 skipped, slow `grab|button|cube|npc|examples|check2` 21 passed):
     - 워프(R14): 몸이 워프할 때 물고 있는 큐브도 머리에 대한 자세를 지켜 함께 옮기고 속도를 0으로 둔다(`env.warp`, 위치 지정·`next_stage` 공통). 시험: 잡은 채 워프 → 함께 옮겨지고 계속 잡혀 있음.
     - 행동(R19, R20): 16단계의 "스크립트 해답이 머리·잡기를 env에 직접 넣는다"를 없앴다. 기본 `actions: [move]`로는 정책이 큐브를 잡을 수 없는데 `mpg check`는 통과했기 때문이다. 정적 단계 `actions`를 더해 큐브를 옮겨야 하는데 `grab`이 없거나, 풀이가 누르는 벽 버튼 판 윗변이 머리를 든 부리 끝(몸 값 `beak_height` 0.21 m)보다 낮은데 `head`가 없으면 실패한다. `cube_plate`는 `examples/task_cube_plate.yaml`(`[move, head, grab]`)로 점검한다. 이 task로 돌리니 해답이 16/20이었다(옛 코드에 같은 task를 줘도 16/20). 놓은 뒤 제자리 오른쪽 돌기가 시작되지 않아 멈췄다(seed 11). 물러날 때 천천히 걸으며 돌게 고쳐(`carry._walk_off`) 20/20.
     - 벽 버튼(R4): 판 앞은 6 cm, 판 뒤는 판 두께 + 1 cm까지만 눌림으로 센다. 얇은 벽 건너편 부리 끝은 누르지 못한다(시험 추가).
     - 큐브 시험을 spec R17 Check대로 `tests/test_cube.py`로 옮겼다(15단계 "계획과 다른 점" 첫 줄은 이제 맞지 않음). 리셋하면 잡기가 풀리는 시험(`eq_active`, `World.held`, `beak.holding`)을 더했다.
16b. 부리 기믹 프리셋(R28): 집을 물체 6종, 부리 장치 8종, `demo:`에 turn·pull 하위 목표 추가. 뒤로 걷지 않는 제약에 맞춰 당기기 방향을 정한다. `lever`와 `receptacle`의 걸이·바구니 프리셋도 여기서 만든다. 예제 `turret.yaml`. (7b단계에서 넘어온 일: `demo:`에 `look` 명령을 더해 `.seen` 문이 있는 방도 풀이가 통과하게 한다. 지금은 그런 방이 `order` 단계에서 실패한다.)
   - PR 나누기 (2026-11-12, 사용자 승인 "E3b를 PR 3개로"): 16b는 세 PR로 나눈다.
     - E3b-1 `feat/m2-beak-objects`: 집을 물체 6종(`cube_weight`, `laundry`, `key`, `handle_ball`, `egg`, `stick`), `receptacle`의 걸이·바구니 프리셋, 저울(`scale`), `wall_button_beak`(`low`, `mid`), 잡기를 큐브 밖 물체로 넓히기, 해답의 걸기·넣기·깨기·막대로 누르기.
     - E3b-2 `feat/m2-beak-devices`: `lever`, `dial`, `drawer`, `pull_cord`, `demo:`의 turn·pull·look, 예제 `turret.yaml`.
     - E3b-3 `feat/m2-dispenser`: `dispenser`.
   - E3b-1 구현 기록 (2026-11-12, 브랜치 `feat/m2-beak-objects`): 잡을 수 있는 물체의 바탕 `Grabbable`(`parts/object/grabbable.py`, `cube`도 이것을 상속), 물체 `laundry`·`key`·`handle_ball`·`egg`·`stick`(`parts/beak/objects.py`), `receptacle`의 `shape: hook`, 받는 부품·압력판의 `.mass` 채널, 프리셋 11개(`beak/` 아래 `cube_weight`, `laundry`, `key`, `handle_ball`, `egg`, `stick`, `hook`, `basket`, `scale`, `wall_button_beak_low`, `wall_button_beak_mid`), 관측 `beak` 물체 종류 6가지(`obs.GRAB_KINDS`), `demo:`의 `drop`, 해답의 걸기·막대로 누르기(`carry.py`). 시험 `tests/test_beak_presets.py`: 빠른 19개(알 낙하 높이 0.02·0.035 m 안 깨짐, 0.08·0.15 m 깨짐, `on_break: fail`과 리셋, 들고 옮기는 동안 안 깨지고 놓으면 깨짐, 열쇠 7·8 대 `accept: [7]` 소켓, 관계없는 마커 번호 겹침 거절, 막대 창, 물지 않은 막대, 바구니 개수·질량, 저울 값, 걸기·저장본·다시 물어 빼기, 관측 종류, `drop` 읽기, 낮은·중간 벽 버튼의 `actions` 점검), slow 13개(프리셋 7가지 해답이 시드 5개 중 4개 이상 신호를 켬, 낮은·중간 벽 버튼 시드 3개씩).
   - 잰 값:
     - 알 `break_height` 0.05 m(0.99 m/s): 접촉 시작 법선 상대 속도. 부리에서 바닥으로 놓기 1.42 m/s(머리 가장 낮게)–1.86 m/s(머리 든 채, 자유 낙하와 맞음), 받침대에서 집어 바닥 판에 놓기 1.68–1.87 m/s, 집고 나르는 동안 0.24 m/s 이하(받침대에 밀면 0.49 m/s), 내려앉은 뒤 건드림 0.35 m/s 이하.
     - 막대 누름 창: R4 창(판 앞 6 cm, 판 뒤 두께 + 1 cm)을 그대로 쓴다. 스크립트로 천천히 다가가 10/10 눌렀고 막대 끝은 판 면 앞 2.9–5.9 cm, 높이 차 2.7 cm 이하, 누르기 전 용접 힘 0.89 N 이하. 벽으로 그냥 걸어 들어가면 막대 끝이 벽에 닿고(−0.004 m) 6번 중 4번 힘 3 N을 넘어 놓쳤다(2번 눌림). 머리 1단위에 막대 끝 기울기 약 0.43 rad.
     - 바구니 벽: 5 cm 벽은 몸이 걸리거나 타고 넘음(4번 중 3번 넘어짐). 10 cm 벽 너머로 머리를 든 부리 끝이 벽 면 앞 5.8–6.8 cm까지 닿음. 그래서 `basket` 벽 높이 0.1 m.
     - 해답 성공(시드 10개, 한 번이라도 켜짐): 걸이 9–10, 빨래·바구니 10, 막대·벽 버튼 10, 알 깨기 10, 열쇠·소켓 9, 추·저울 9, 고리 공·바구니 8(바구니에서 2번 넘어짐), 낮은·중간 벽 버튼 6/6. `cube_plate` `mpg check`(`task_cube_plate.yaml`) 20/20, 평균 24.4 s.
   - 계획과 다른 점:
     - `Grabbable`은 `parts/beak`이 아니라 `parts/object/grabbable.py`에 두었다. `parts/object`와 `parts/beak` 사이 순환 import를 피하려는 것이다.
     - `receptacle`에 `.mass`(담긴 물체 질량 합), `pressure`에 `.mass`(판 위 질량, 1 g 단위)를 모든 판에 더했다. 저울은 `pressure` 프리셋이다.
     - `accept`에 마커 번호(정수)를 쓸 수 있게 했다. 그 번호는 소켓과 열쇠가 함께 달아도 된다(`markers.assign_ids`, 다른 겹침은 여전히 거절). 방향은 소켓 `angle_tolerance`만 본다.
     - 깨기 해답은 `break` 대신 `drop <물체> <부품>`이다. `break`는 R34 하위 목표 이름으로 남겨 둔다.
     - 걸이는 보이기만 하는 걸이와 숨은 weld다(걸이는 부딪히지 않고, 기둥과 잡을 수 있는 물체의 접촉은 끔). 물지 않은 물체의 손잡이가 걸이 끝 `tolerance` 안에 오면 건다.
     - `check.py` `map_placement`가 벽 버튼(`mount: wall`)을 "벽 위"로 잘못 거절해서, 벽 부품은 크기 0으로 본다. 글자 지도 방에서 벽 버튼 `pos`는 바닥 칸 안(벽 면 0.9가 아니라 0.899)이어야 한다. 벽 칸 위면 몸체 높이가 벽 꼭대기(0.25 m)로 올라간다.
     - `carry.py`: 놓을 자리에서 물체가 몸 옆에 걸려 제자리 돌기가 안 될 때 `STUCK`(1.5 s) 뒤 물러났다 다시 온다. 받는 부품이 `place_tolerance`를 주면 그 안에서 놓는다(바구니·저울은 가장자리 여유를 둔 안쪽). 물러날 때 앞에 고정된 것(벽, 바구니 벽)이 0.16 m 안이거나 방금 놓은 물체가 바로 앞이면 걸으며 돌지 않고 옆걸음으로 제자리 돈다(걸으며 돌면 넘어지거나 저울 위 큐브를 쓸어냄). 막대 끝·앞장선 손잡이 높이 맞추기는 이득을 거리에 따라 줄인다(`PITCH_PER_HEAD` 0.8, 잰 값 0.43에서는 넘침).
     - `demo:` `grab`·`place`·`drop`은 손잡이가 있는 모든 물체를 받는다. 오류 문구는 "is not a grabbable object".
   - 알려진 한계: 깨진 알도 받는 부품이 센다. 열쇠는 번호로만 맞추고 꽂는 방향은 소켓 각도 허용으로만 본다. 바구니 벽은 0.1 m 아래로 두면 해답이 넘어진다. 막대를 문 채 벽에 부딪히면 놓친다. 바구니·저울 놓기 해답은 시드에 따라 10번 중 8–10번.
    - E3b-1 리뷰 반영 (2026-11-12, 사용자 승인: 1·2·5 반영, 4 기각, 알 기본값 A, 빠른 747 passed·1 skipped, slow `grab|button|cube|npc|examples|check2|beak` 34 passed):
      - 마커 번호(R26): 같은 번호는 그 번호를 `accept`하는 받는 부품 하나와 물체 하나(소켓과 열쇠)만 함께 쓴다. 전에는 그 번호의 첫 주인하고만 견주어 소켓 뒤의 문이나 둘째 열쇠도 통과했다. 이제 앞선 주인 모두와 견준다. 시험 추가.
      - 알 저장본 복원: 복원 때 지금 접촉과 속도로 `_touching`·`_v_prev`를 다시 채운다. 전에는 떨어지는 중에 복원하면 옛 속도로 충격을 재 바로 깨졌다. 시험 추가.
      - 가이드 3종과 `warp.py`의 "큐브만" 문구를 잡을 수 있는 물체 전체로 고쳤다.
      - 기각: 막대가 있으면 모든 벽 버튼의 먼 목표 원이 생기는 문제. 막대로만 닿는 버튼이면 `mpg check`의 풀이 단계에서 실패하므로 조용히 넘어가지 않는다.
      - 알 `break_height`는 0.05 m 그대로(사용자 결정 A). 부리 끝이 바닥 위 13.5 cm 이상이라 바닥·바구니에 놓으면 깨진다. 성한 채 전하려면 걸이나 높은 받침을 쓴다. 가이드와 프리셋 설명에 적었다.
   - E3b-2 구현 기록 (2026-11-12, 브랜치 `feat/m2-beak-devices`): 장치 `lever`(`push`·`grab`)와 `pull_cord`(`parts/beak/devices.py`), 프리셋 2개(`beak/lever`: `mode: push`, `detents: 3`; `beak/pull_cord`: `distance: 0.06`, `latch`), 관측 `beak` 물체 종류 `lever` 7·`pull_cord` 8, `demo:`의 `turn <레버> <값>`·`pull <줄>`·`look <부품>`, 격자 해답이 레버를 1까지 밀고 줄을 당김(`carry.py` `turn_act`·`pull_act`·`look_act`, `solver.py` `_work`), 예제 `turret.yaml`. 다이얼과 서랍은 만들지 않았다(아래 잰 값, 멈춤 규칙). 시험 `tests/test_beak_presets.py`: 빠른 19개(레버 칸과 `detents: 1` 거절, 레버 저장본·리셋에서 거짓 신호 없음, 잡는 레버 손잡이 물기, 포탑 탄도 점검이 레버 칸마다 겨눔: 맞는 칸 있음·없음·연속, 당김줄 flexcomp가 가만히 매달림, 당김줄 켜짐·`latch`·리셋·저장본, `turn`·`pull`·`look` 읽기와 오류 문구, 당김줄의 `actions` 점검, `look` 방 풀이 순서), slow 6개(미는 레버 1로·0으로·1 뒤 -1로, 당김줄: 시드 5개 중 4개 이상, `look`으로 본 문 열기 격자 해답·`demo:` 두 가지). `tests/test_examples.py`에 `turret`.
   - 잰 값 (기본 몸, `max_hold_force` 3 N, 명령을 직접 줌):
     - 잡는 레버(연속, 손잡이 앞에 몸을 옮겨 놓고 물린 뒤 옆걸음 vy ±0.3, 12초, 몸 방향 3가지): 왼쪽 옆걸음은 몸이 안 움직임(0.0–0.2°). 오른쪽은 9.5–15.7° 돈 뒤 0.3–0.7초에 용접 힘이 3 N을 넘어 놓침. 힘 한도 100 N으로 4초: 오른쪽 +12.8°, 왼쪽 -0.7°, 용접 힘 중앙값 0.5–1.1 N·최대 7 N. 같은 자리에서 잡지 않은 옆걸음은 4초에 왼쪽 0.25 m·오른쪽 0.19 m, 큐브를 문 옆걸음 0.23 m. 용접을 무르게(solref 0.1·0.5) 하거나 머리를 0.5로 내려도 멈춤은 같다. 그래서 `grab` 기본 `max_angle` 10°(`GRAB_MAX`), 프리셋은 `push`.
     - 다이얼(멈춤 규칙으로 뺌): 가운데를 보며 옆걸음하는 제어(vy ±0.3, 이득 2·5, 앞먹임 0·0.5·1, 3 N)로 7° 이하, 0.3–1.0초에 자주 놓침. vy ±0.3과 wz ±0.5·±1을 함께 주면 0.14–0.86초에 놓쳤고 최대 13.4°. 100 N이면 [0, 0.3, 1]로 -90°까지 갔지만 몸이 -110° 돌며 힘이 7.9 N까지 올랐다. R28 "몸이 vy+wz로 원을 돌며 돌린다"가 지켜지지 않는다.
     - 서랍(멈춤 규칙으로 뺌): 3 N, 8초: 왼쪽으로 디디면 몸 5 mm 이동·0% 열림, 오른쪽은 0.30–0.32초에 놓치고 9% 이하. 100 N이면 몸이 손잡이를 축으로 67–263° 돌았고 53% 이하, 힘 10.6 N까지. 몸 방향 되먹임, 무른 용접, 머리 낮추기도 소용없었다. R28 "옆걸음으로 연다"가 지켜지지 않는다.
     - 당김줄: MjSpec `make_flex`(1D grid, 9마디)로 만들어진다. 손잡이 없이도 제자리 반시계 돌기는 3초에 +9°뿐(시계 방향은 -110°). 손잡이를 문 시계 방향 돌기 4.5–5초. 앞으로 걸으면 줄에서 0.13–0.16 m에서 멈춘다. 중력 보정을 켠 뒤 당긴 길이: 용수철 1 N/m 6.7–6.8 cm(3.6–4.0초에 켜짐), 2 N/m 7.1–7.3 cm(5.4–5.9초), 4 N/m 3.5–3.7 cm(안 켜짐), 중력 보정 없는 8 N/m 1.5–1.7 cm. 그래서 `stiffness` 2 N/m, `distance` 6 cm.
     - 미는 레버(vx 0.25, 방향 유지): 1까지 3.2–4.3초 5/5, 넘어짐 없음. 0까지 2.7–3.6초, 그 뒤 0에 머묾.
     - 해답: 1로 미는 데 5.2–5.7초 3/3, 0으로 밀면 0 유지, 1 뒤 -1 성공. 당김줄 켜짐 9.3–11초 3/3. `look` 방 `mpg check` 격자 해답 10/10, `look door` demo 10/10. `turret.yaml` `mpg check` solver 20/20, 평균 24.93 s.
   - 계획과 다른 점:
     - `dial`·`drawer`는 만들지 않았다(위 잰 값). spec.md와 Open questions는 고치지 않았다. 다른 설계로 바꾸지 않았다.
     - 잡는 레버는 해답이 돌리지 않는다. `turn`은 `mode: push` 레버만 받고, 잡는 레버면 "the solver turns levers with mode: push" 오류를 낸다. 격자 해답도 잡는 레버를 다루지 않는다.
     - 당김줄은 줄을 늘이지 않고 팔 안 용수철 미닫이에서 빼낸다(줄 마디 사이 equality solref 0.005로 늘어남 2.5 cm를 막음). 줄과 손잡이는 중력 보정으로 무게가 없고 충돌하지 않는다. 몸이 머리를 0.3 N쯤으로 끌리면 앞으로 못 걷기 때문이다. 당기는 법은 물고 시계 방향으로 돌아 앞으로 걷기다(그림은 가이드에 글 그림으로 넣음).
     - 미는 레버는 풀이용 막힘으로 판이 쓸고 가는 자리를 낸다. 그래야 두 번째 `turn`(되돌리기)이 레버 반대편으로 돌아간다. `turn`은 민 뒤 서서 시계 방향으로 돈 다음 걸어 나간다(걸으며 돌면 레버를 1까지 더 밀었다).
     - `look`: 보는 부품마다 숨은 `Look` 스위치(`_look_<이름>`, 몸체 없음)를 더해 `GazeWatch` 풀이 값을 대신하고, 풀이 순서가 "보고 지나가기"를 세운다. `look`의 끝은 지금 `.seen`이 아니라 래치다(한 번 보면 열리는 문이 미끄러져 표지가 시야에서 빠지므로).
     - 신호로 겨누고 켜질 때 쏘는 포탑이 있으면 과녁 판(`input/target`)이 스위치가 되고, 목표 영역은 포탑 `in` 채널에 쓰는 부품들의 자리다. 포탑의 탄도 점검은 `bind` 원천이 `channel_cells`를 주면(칸이 있는 레버) 그 칸마다 겨눈다.
     - 잡기는 물체 몸체 대신 `handle_body`(장치는 공 관절 손잡이)를 용접한다(`grab.py`, `Grabbable`에 `handle_body`·`handle_bid`).
     - `check.py` `actions`: `demo:`의 `pull`이나 풀이 순서의 손잡이 장치가 있으면 `grab`을 요구한다.
   - E3b-2 리뷰 반영 (2026-11-12, 사용자 승인):
     - `env.warp("body")`: 부리가 장치 손잡이(자유 물체가 아님)를 물고 있으면 놓고 몸만 옮긴다. 전에는 장치 몸체의 관절 주소 -1로 몸의 마지막 관절을 써서 ValueError가 났다. 시험 추가.
     - 숨은 `Look`(`_look_<부품>`)의 위치는 보는 부품의 위치를 따른다(전에는 [0, 0]이라 `target_pos`가 원점을 가리켰다). `part_pos`·`part_state` 관측은 `_look_` 부품을 빼서 `.seen`을 읽는 방의 관측 길이가 E3b-2 전과 같다. 문이 원점이 아닌 방으로 시험.
     - 부품에 `target_xy()`(기본: 자기 위치)를 더해 `target_pos` 관측과 격자 해답이 쓴다. 포탑이 있는 방의 과녁 판은 쏘는 입력(발판)의 위치를 준다(목표 영역과 같음). 시험 추가.
     - `.angle`을 `bind`만 읽는(문 같은 `in`·`enable`이 읽지 않는) 레버는 역할 `prop`이다. 포탑 겨눔 레버를 1로 밀어도 `new_on`이 없고 풀이 순서에도 없다. `turret.yaml` 풀이 순서는 `board`, `door`, `goal`, `mpg check` 20/20(평균 24.72 s). 시험 추가.
     - 격자 해답 당기기: 목표가 줄에서 다음으로 넘어가면 해답이 바로 손잡이를 놓는다(전에는 `pull_act`가 다시 불릴 때만 놓아 몸이 손잡이를 끌었다). `demo:` 없는 당기기 slow 시험(시드 5개 모두 켜진 뒤 0.1 s에 놓고 골 도착, 고치기 전에는 5개 모두 골 실패).
     - `grab.py` 설명의 "dial, drawer"를 지웠다.
   - E3b-2 다이얼·서랍 재설계 구현 (2026-11-12, 사용자 승인, spec 07c2414의 R28 `dial`·`drawer`):
     - `dial`(`parts/beak/devices.py`, 프리셋 `beak/dial`): 세운 축에 살 4개(높이 0.07–0.12 m, 반지름 0.15 m), 칸마다 멈춤 용수철. 몸이 `facing` 쪽 축 옆 0.10 m 선을 축을 왼쪽에 두고 걸어 지나가면 한 칸(90°)씩 반시계로 돈다. `<이름>.angle` = 칸 번호(`channel_cells` 0..3, 포탑 조준), `<이름>` = 칸이 `target` 안이면 1. `turn <다이얼> <칸>`과 격자 해답(첫 목표 칸)은 `carry.dial_act`: 선 시작점으로 걸어가 제자리 시계 돌기로 선을 향하고 천천히 지나가기, 칸이 모자라면 걸으며 시계 돌기로 되돌아와 다시(`facing` 쪽 약 1 m 사용).
     - `drawer`(프리셋 `beak/drawer`): 캐비닛 속 받침 높이(0.17 m) 서랍, 0.15 m 나옴, 앞판에 무게 없는 띠(1D flexcomp) 0.1 m 끝 손잡이(0.19 m). 손잡이를 물고 제자리에서 시계로 돌면 열리고, 97%에서 걸린다(미끄럼 `jnt_range`를 줄임, 저장본 필드). 걸리면 `<이름>` 1, 띠 손잡이는 잡기 대상에서 빠지고 물고 있으면 놓는다(`grabbable_now`, grab.py). 닫힌 서랍 위의 자유 물체는 서랍 바닥에 놓이고(`carry_xy`, env 리셋) 함께 나온다. 앞에서 0.09 m보다 깊으면 `check_static` 실패. `pull <서랍>`과 격자 해답은 `pull_act`(당김줄과 같음).
     - `pull_act`: 다가가는 동안 잡기를 끈다. 물고 6초(`PULL_STALL`) 동안 1 cm도 못 당기면 놓고 다시 온다(몸이 손잡이를 축으로 반대로 돌며 밀려나는 경우).
     - 측정 정정: 앞선 서랍 재설계 측정(돌기만으로 1.2 s에 90%)은 측정 스크립트가 몸 명령에 행동 단위 [-1, 0, -1]을 넣어 실제로는 뒤로 걸으며 돈 값이었다. 바른 명령(제자리 시계 돌기 [0, 0, -1])으로 다시 쟀다. spec 문장(물고 제자리에서 시계로 돌면 열림)은 다시 잰 값으로도 맞다. 다만 걸리는 시간이 1.2 s가 아니라 4.8–14 s이다.
   - 잰 값 (기본 몸, 시드 10개):
     - 다이얼 한 번 지나가기(헤딩 유지): 축 옆 0.10 m는 vx 0.25·0.4 모두 10/10 정확히 90°. 0.12 m는 10/10·9/10, 0.14 m(vx 0.4)는 5/10, 0.08 m(vx 0.25)는 7/10(3번은 축 쪽으로 밀려 잘못 돎). 넘어짐 0. 0.8 m 지나가기 vx 0.4 약 4 s, vx 0.25 약 8 s. 시계 고리로 칸 3 맞추기 10/10(32.7–33.5 s), 칸 2(vx 0.25) 10/10(29.6–31.6 s).
     - 서랍 열기(손잡이를 부리 끝에 맞춰 물고 제자리 시계 돌기): 띠 6 cm는 손잡이를 맞게 물면 10/10(8.2–14.1 s, 용접 힘 0.45 N 이하). 손잡이 위치와 방향을 흩뜨리면(±2.5 cm, ±0.15 rad) 6/10. 띠 8 cm는 7/10, 10 cm는 10/10(4.8–14.1 s, 0.38 N 이하, 놓침 0). 걸으며 돌기(vx 0.25, wz -1)는 0/10(24–32% 열림). 마찰 0, 이동 0.1 m 등은 9/10, 8/10. 그래서 띠 기본 0.1 m다(spec "약 6 cm"와 다름, 아래).
     - 서랍 속 큐브(`cube_weight`): 걸린 뒤 손잡이를 빼면 해답의 `grab`이 10/10 집음(9.2–9.8 s, 서랍 98.5% 이상 그대로). 손잡이를 빼지 않으면(기존 잡기 반경 4 cm) 9/10이 띠 손잡이를 다시 물어 서랍을 닫았다. 큐브가 앞에서 0.12 m 안쪽이면 앞판 너머로 못 닿아 0/10, 0.08 m면 10/10.
     - 해답: `pull dr`·`grab c` 10/10(22.7–54.7 s; 2번은 멈춤 감지 뒤 다시 물어 성공). 문을 서랍·다이얼로 여는 방의 격자 해답 `mpg check` 10/10씩(서랍 평균 28.6 s, 다이얼 칸 2 평균 37.4 s).
   - 계획·spec과 다른 점:
     - 서랍 띠는 spec "약 6 cm"가 아니라 0.1 m다(위 잰 값: 흩어진 잡기에서 6 cm 6/10, 10 cm 10/10). 사용자 승인으로 spec이 "약 10 cm"로 고쳐졌다(83e6b60).
     - 다이얼 살은 4개로만 쟀다(`spokes` 3–8 허용, 기본 4). 칸 번호는 0–3을 돈다.
     - 다이얼 되돌아오는 고리 자리(`facing` 쪽 약 1 m)는 정적 점검이 보지 않는다.
   - E3b-2 다이얼·서랍 리뷰 반영 (2026-11-12, 사용자 승인):
     - 다이얼은 한쪽으로만 돈다(spec R28). 칸이 정해질 때마다 경첩의 아래 멈춤(`jnt_range`, 저장본 필드)을 그 칸 -0.3칸으로 올린다(`RATCHET`). 리셋은 처음 범위로 되돌린다. 전에는 같은 0.10 m 선을 반대로 걸으면 칸 2가 1로 내려갔다(5/5). 고친 뒤 반대로 지나가기 5/5 모두 칸 2 그대로, 정방향 지나가기는 5/5 칸 3. 시험: 칸 2에서 반대 방향 힘 2 N·m 2초 뒤에도 칸 2, 리셋 뒤 멈춤 원위치.
     - 다이얼의 길 찾기 장애물(`blockers`)이 축 기둥(0.05 m)이 아니라 살 반지름(0.15 m)을 덮는다. 전에는 다가가는 길이 살을 밀어 칸이 돌았다(시드 1에서 0→3). 고친 뒤 같은 방 5/5 성공, 칸은 해답이 돌린 0→1만. `dial_act`의 선 지나가기는 그대로 된다.
     - 다이얼 `target`의 from이 to보다 크면(예: [3, 0]) 불러올 때 거절한다(전에는 통과했지만 켜질 수 없었다).
     - 서랍 속 물체: 리셋이 모든 부품을 먼저 놓고(무작위 흔들림 포함), 그다음 방 파일 위치가 닫힌 서랍 바닥 위인 자유 물체를 서랍의 이번 위치 기준 같은 자리에 놓는다(`Drawer.on_tray`·`carry_xy`, 물체 자체의 흔들림은 버림). 전에는 서랍 위치를 놓기 전에 봐서 `random`이 있으면 물체가 서랍 밖에 놓였다. `check_static` 깊이도 방 파일 위치로 잰다. 시험: 서랍 `random` 0.3 m, 시드 4개에서 큐브가 서랍 기준 같은 자리, 바닥 높이 0.19 m.
     - 관측 `beak`의 가장 가까운 손잡이에서 지금 잡을 수 없는 물체(걸린 서랍 띠, `grabbable_now`)를 뺀다(`Beak.nearest`와 같음). 잡을 것이 없으면 손잡이 값은 0. 시험: 걸린 서랍만 있는 방.
   - 알려진 한계: 잡는 레버는 10°까지만 돌고 해답이 다루지 않는다. 기본 몸이 제자리에서 반시계 방향으로 거의 못 돌아 해답의 돌기는 모두 시계 방향이다. 당김줄은 켜지기까지 해답 기준 9–11초 걸린다. 미는 레버를 되돌리려면 반대편으로 돌아가야 한다.
   - E3b-3 구현 기록 (2026-11-12, 브랜치 `feat/m2-dispenser`): `dispenser`(`parts/beak/dispenser.py`, `button: floor | wall_beak | none`, `item`, `stock` 최대 20, `max_active`, `cooldown`, `refill`, `recycle`, `exit`·`exit_vel`, 채널 `.stock`·`.empty`), 복제품 `<이름>/<번호>`를 방을 읽을 때 미리 만듦(`add_room_parts`, 같은 마커 번호 `marker_clone_of`), 보관(`Part.parked`; 물체는 바닥 아래 5 m에서 충돌 끔·중력 보정, NPC는 방 밖 100 m 바닥 높이에서 구동 끔; `FreeObject`·`Npc`의 `park`·`unpark`), 받는 부품의 `consume`, 구덩이·낭떠러지로 떨어진 복제품 되돌림, 보관 복제품을 관측·잡기·시야·센서·해답에서 뺌, 프리셋 `beak/dispenser`·`beak/dispenser_wall`, `demo:`의 `take <디스펜서>`와 그 뒤의 `grab`·`place`·`drop <디스펜서>`, 격자 해답이 받는 부품에 넣을 물체가 없으면 디스펜서에서 받음(`solver.py` `_deliver`·`take_act`), `mpg check` `actions` 단계가 받아 오는 벽 판을 봄. 시험 `tests/test_beak_presets.py`: 빠른 18개(상승 순간마다 하나·재고·`empty`·`cooldown`, `max_active`와 소비·`recycle`, `refill`, `consume` 없는 바구니, 바닥 아래로 떨어진 복제품 두 가지, 마커 번호 공유, 저장본·리셋 왕복, 보관 복제품이 관측·잡기에서 빠짐, NPC 복제품, 설정 오류 3가지, `take` 읽기와 오류, 디스펜서 방의 `actions` 점검 4가지), slow 4개(두 프리셋의 `demo:` 해답과 격자 해답, 시드 5개 중 4개 이상).
   - 잰 값 (시드 10개, 디스펜서 `[-0.2, 0.35]` `facing: -90`, 바구니 `consume: true`, `demo: [take d, grab d, place d bk]`):
     - 발판: 바구니 `[0.45, 0.2]` 10/10(바구니 켜짐 10.0–12.6 s), `[0.4, -0.1]` 8/10, `[0.45, -0.2]` 7/10, `[0.2, -0.2]` 9/10.
     - 벽 판: 바구니 `[0.5, -0.3]` 10/10(21.2–25.4 s), `[0.6, -0.25]` 8/10, `[0.6, 0.0]` 7/10, `[0.45, 0.2]` 6/10. `consume: false`로 같은 방을 돌려도 `[0.45, 0.2]`는 6/10으로 같아서, 실패(놓을 때 바구니 벽에 기대 넘어짐, 가장자리에 걸림)는 소비가 아니라 E3b-1의 바구니 놓기 한계다. 시험은 프리셋마다 10/10인 자리를 쓴다.
     - 격자 해답(바구니가 목표 앞 문을 엶, 바구니 `[0.05, -0.3]`): 바구니 켜짐 발판 9/10(10.8–14.2 s), 벽 판 9/10(20.3–25.4 s). 목표까지는 두 프리셋 모두 1/10(아래 한계).
   - 계획과 다른 점:
     - 소비된 물체는 놓은 뒤 물러나지 않는다(`carry._let_go`). 물러나기는 놓은 물체를 건드리지 않으려는 것인데 소비된 물체는 없고, 사라진 자리(보관 자리)에서 물러나려 하거나 바구니 옆에서 돌다 넘어졌다(물러나게 하면 발판 7/10, 벽 판 5/10).
     - 격자 해답은 놓는 중에 받는 부품이 켜져 다음 목표로 넘어가도 놓기를 끝낸다(`solver._move`). 전에는 그 자리에서 잡기를 풀고 바로 걸어 넘어지거나 멈췄다.
     - 떨어진 복제품은 물체만 본다(z가 바닥 − `CATCH_DEPTH`/2 아래). NPC는 평면 관절이라 떨어지지 않는다.
     - `button: none`이면 갱신 단계가 `device`, 판이 있으면 `input`이다. 판은 디스펜서 몸체에 붙은 내부 `Button`(`momentary`, `trigger: body`)으로 누름을 잰다.
   - 알려진 한계: 바구니 놓기 성공률은 바구니 자리에 따라 6–10/10이다. 격자 해답이 바구니를 채운 뒤 문 방 목표까지 가는 것은 표준 문틈(0.56 m) 방에서 발판 40/40, 벽 판 35/40이다(아래 "바구니에서 나오기", `mpg check` 20/20·19/20). 로봇을 내보내는 디스펜서는 해답이 쓰지 않는다(`take` 뒤 `grab`이 거절). 디스펜서 방은 버튼을 다시 밟아 복제품이 더 나올 수 있다(격자 해답이 바구니 옆에서 서성이다 발판을 밟음).
   - E3b-3 리뷰 반영 (2026-11-12, 사용자 승인 1–7, 1건 기각, 빠른 817 passed·1 skipped, slow `beak|examples|check2|gaze|grab|cannon|look|dispenser` 49 passed):
     - `take`가 멈추던 문제(`solver.take_act`): 판이 이미 눌려 있거나 디스펜서가 지금 내보낼 수 없으면(`cooldown`, `Dispenser.ready`) 판에서 내려서고(벽 판은 머리를 들고 물러남) 준비되면 다시 누른다. 벽 판은 기다리는 동안 0.35 m 물러났다가 다시 다가온다(판 앞에 남으면 제자리 반시계 돌기로 40 s 넘게 멈췄다). `[take d, take d]`, `max_active: 2`, `cooldown` 0·5 s: 발판·벽 판 모두 2개를 받는다(전에는 1개에서 멈춤). slow 시험 추가.
     - 보관된 물체는 잡히지 않는다: `Grabbable.grabbable_now`가 `parked`면 거짓(디스펜서 복제품만이 아니라 소비된 보통 물체도). 관측 `beak`가 바닥 아래 5 m의 손잡이를 가리키지 않는다.
     - 격자 해답이 가져올 물체를 고를 때 켜져 있는 다른 입력 안이나 위의 물체(채워진 받는 부품, 눌린 압력판)는 건너뛴다(`solver._in_use`). 시험 추가.
     - 떨어진 복제품: 디스펜서 바닥 높이가 아니라 받침면(`-CATCH_DEPTH` + 0.2 m = -0.3 m, 가장 낮은 바닥 -0.2 m보다 아래)과 견준다. 전에는 0.3 m 높은 바닥에 선 디스펜서의 복제품이 0 m 바닥으로 내려가면 되돌려졌다. 높은 바닥·낮은 바닥·받침면 시험 추가.
     - 내보낼 때 복제품을 리셋한다(`_hand_out`): 되돌아온 깨진 알이 성한 채 나온다. 시험 추가.
     - slow 시험 방이 `mpg check` 정적 단계를 통과한다: 디스펜서 `[-0.2, 0.33]`(캐비닛 뒤가 벽 면 0.45 m에 닿음, 전에는 2 cm 겹침), 바구니 `[0.25, -0.1]`(전 벽 판 자리 `[0.5, -0.3]`은 벽 칸에 걸림). 빠른 시험이 이 방과 문 방의 정적 점검을 본다.
     - 다시 잰 값 (시드 10개, 시험 방 그대로, `demo: [take d, grab d, place d bk]`): 바구니 `[0.25, -0.1]` 발판 9/10(9.9–13.3 s, 1번 넘어짐), 벽 판 10/10(12.7–25.8 s). 다른 자리(벽 판): `[0.25, -0.15]` 8, `[0.3, 0.0]` 6, `[0.35, -0.1]` 4; 방 벽에 붙은 `[0.5, -0.28]` 4(바구니와 벽 사이 구석으로 물러나 갇힘).
     - 소비된 물체를 놓은 뒤: 1.5 s(`SETTLE`) 서서 몸이 가라앉은 다음, 놓은 자리에서 물러난다. E3b-3 첫 구현은 서 있기만 했는데, 그러면 가슴이 바구니 벽에 붙은 채 남아 격자 해답이 다음 목표로 못 갔다(목표 0/10). 놓자마자 물러나면 벽 판 `demo:`가 4/10(놓은 직후 앞으로 기운 채 돌다 넘어짐)이라 기다림을 넣었다.
     - 바구니 → 문 → 목표 (7번, 목표치 못 맞춤): 문 방(칸 x 0.65 m, 문 폭 0.3 m, 바구니 `[0.35, 0.15]`)에서 격자 해답은 바구니를 발판 10/10(9.0–11.7 s), 벽 판 9/10(11.7–21.6 s) 켰고, 목표까지는 발판 7/10(21.8–31.3 s), 벽 판 5/10(30.7–39.5 s)이었다(`mpg check` 동적 단계 18/20에 못 미침). 원인 잰 값: (1) 놓는 동안 몸이 바구니 쪽으로 기울고(중력 x 성분 0.54, 약 33°) 놓은 직후 넘어짐 — 발판 2/10, 벽 판 1/10. (2) 계획 격자가 바구니 벽을 장애물로 보지 않아 바구니 벽을 따라 가는 길을 고르고, 몸이 벽에 붙어 반시계로 돌지 못해 멈춤. 바구니가 `[0.25, -0.15]`이면 목표 1–2/10. (3) 40 s 안에 목표에 못 닿고 멈춘 판이 벽 판 3번(바구니를 켜는 데만 18–22 s), 발판 1번. 상자 받는 부품의 벽을 계획 장애물(`blockers`)로 넣어 보았으나 `demo:` 6/10·8/10, 목표 0/10·1/10으로 나빠져(놓은 뒤 몸이 부풀린 장애물 칸 안에서 시작해 길이 없음) 되돌렸다. 고치려면 놓은 뒤 바구니에서 벗어나는 걸음(뒤로 걷기나 옆걸음)과 벽 장애물을 함께 다뤄야 해서 여기서 멈춘다.
     - 기각(알려진 한계): 해답은 `button: none`(다른 입력이 켬)의 조건이나 `max_active`가 막는 경우를 보지 않는다. `mpg check`의 동적 단계가 잡고, 조립 템플릿 단계에서 다룬다.
   - 바구니에서 나오기 (2026-11-12, 브랜치 `feat/m2-basket-exit`, 사용자 승인 1–3): 위 7번(문 방 목표)의 원인 세 가지를 차례로 고쳤다. 잰 값은 문 방(`gate_disp_room`), 40 s 판(`mpg check` 동적 단계와 같음), 시드 40개.
     - 바꾼 것: (1) 상자 받는 부품(바구니)에 놓은 뒤 물러나기를 놓은 물체 자리가 아니라 바구니 가운데에서 `max(half)` + `footprint_radius` + `CLEARANCE`(0.28 m)까지 잰다(`carry._let_go`, 디스펜서 발판의 `take_act`와 같은 꼴). `SETTLE` 서 있기는 그대로. (2) 해답의 계획 격자(`solver.field`, `_live_field`)가 바구니 벽을 장애물로 본다(`Receptacle.wall_segments`, `solver.plan_free`). `blockers`가 아닌 것은 정적 점검의 닿음·순서 검사가 바구니 안을 막힌 곳으로 보기 때문이다. `carry._free`(놓으러 갈 때의 길)에는 넣지 않았다: 넣으면 고리 공·바구니가 0/20(넣지 않으면 16/20)으로, 바구니 가운데 대신 가장 가까운 빈 칸(바구니 위쪽)으로 걸으며 받침대 위로 공을 끌다 떨어뜨렸다. (3) 놓으러 가는 중에 바구니 벽이 바로 앞이고(`_blocked_ahead`) 물체가 바구니 안쪽 위에 있으면 더 걷지 않고 서서 놓는다(`place_act`).
     - 목표 도달: 고치기 전 발판 29/40(넘어짐 6, 40 s 초과 5), 벽 판 16/40(넘어짐 7, 초과 17). (1)만 발판 14/20, 벽 판 7/20(시드 20개). (1)+(2) 발판 28/40(넘어짐 7, 초과 5), 벽 판 17/40(넘어짐 6, 초과 17). (1)+(2)+(3) 발판 33/40(넘어짐 0, 초과 7), 벽 판 25/40(넘어짐 0, 초과 15). (3)은 넘어짐을 13번에서 0번으로 줄여 남겼다. `mpg check`(시드 20개): 발판 16/20, 벽 판 12/20으로 18/20에 못 미친다.
     - 걸린 시간(고친 뒤): 바구니 켜짐 발판 8.9–11.6 s(가운데값 10.2), 벽 판 12.0–23.1 s(20.7). 켜진 뒤 목표까지(성공한 판) 발판 7.9–30.1 s(16.7), 벽 판 9.1–20.2 s(14.7).
     - 회귀(시드 10개, 고치기 전 → 뒤): 추·저울 9 → 9, 빨래·바구니 10 → 10, 고리 공·바구니 6 → 8, 열쇠·소켓 9 → 9, 알 10 → 10, 빨래·걸이 9 → 9, 디스펜서 `demo:` 발판 9 → 10, 벽 판 10 → 10.
     - 남은 40 s 초과(위 문 방): 발판 7번 중 5번, 벽 판 15번 중 10번이 문 안쪽 x 0.66–0.76 m에서 멈췄다. 그 방의 문틈 0.3 m가 계획 격자의 부풀린 몸 폭(0.13 m + 격자 0.03 m, 양쪽 0.32 m)보다 좁고 목표 칸도 깊이 0.2 m라, 목표 단계의 거리장에 유한한 칸이 하나(0.72, 0.0)뿐이었다(해답이 그 칸을 곧장 겨누고 둘레를 돎). 정적 점검은 부풀림 없이 보므로 통과했다.
     - 문 방을 표준 문틈으로 다시 지음(사용자 승인 1안): `examples/rooms/doors.yaml`과 같은 틈(칸막이 x 0.4 m, 벽 끝 y ±0.28 m 사이 0.56 m, 벽 두께를 빼면 실제 틈 0.50 m), 기본 `device/sliding_door`(폭 0.5), 목표 `[0.7, 0]`(목표 방 깊이 0.47 m), 방 안쪽 1.8 × 0.9 m, 디스펜서·출발점은 그대로. 바구니 자리(시드 40개, 발판 / 벽 판): `[0.15, -0.25]` 40/40 / 35/40(넘어짐 0, 벽 판 실패 5번 중 3번은 바구니를 못 켬, 2번은 목표 앞 40 s 초과), `[0.0, -0.25]` 40/40 / 34/40, `[0.1, 0.0]`(문 가는 길 위) 28/40 / 7/40. 시험은 `[0.15, -0.25]`를 쓴다. `mpg check`(시드 20개): 발판 20/20, 벽 판 19/20(시드 14 바구니 못 켬), 정적 단계 통과. 목표 도달 시간 발판 17.4–26.7 s, 벽 판 26.5–37.7 s.
     - 시험: slow `test_grid_answer_fills_the_basket_and_reaches_the_goal`(두 프리셋, 40 s 판, 5개 중 4개), 빠른 `test_solver_plans_round_basket_walls`. 문 방(`gate_disp_room`)을 쓰는 다른 시험도 새 방을 쓴다.
16c. 애드온 장착대(R30): 먼저 공식 바퀴 모델과 합친 모델에서 `roller.onnx`를 나란히 돌려 OQ7을 정한다(자산 추가 받기는 사용자 승인 후). 그다음 장착대, 모드 전환, `crouch` 행동, 예제 `roller_rink.yaml`. `set_addon` 순서와 리셋·복원 경로, `crouch` 최소 속도, 바퀴 퍼즐 조립 프리셋 9종, 예제 `roller_route.yaml`. 2b부 범위 바뀜(2026-11-13, 사용자 "승인"): 낮은 막대 부품 `low_bar`와 해답 `glide` 하위 목표를 더하고 `glide_under`·`roll_delivery`를 만든다. `fast_herd`는 뺀다(도망 NPC는 감지 거리 안에서만 움직여 걷기로도 몰림). 프리셋은 모두 8종.
   - OQ7 측정 기록 (2026-11-13, 브랜치 `feat/m2-addon-roller`): 받은 자산 8개(`robot_groundcontact_rollers_backlash.xml`, 메시 `roller_blade`·`tire`·`rim`·`ankle_{l,r}_v1`, `roller.onnx`·`roller_crouch.onnx`)를 `upstream.lock.json`에 고정(사용자 "승인"). 합친 모델: `robot_spec()` 발 모델의 `ankle_left`/`ankle_right`에 공식 바퀴 몸 `tire`~`tire_4`를 `attach_body`(접두 `w_`)로 붙인다. 관절 순서·바퀴 자유도 번호가 공식 모델과 같다.
     - 발 충돌만 끄면 vx 0.3/0.4/0.6에서 0.09/0.14/0.18 m/s(공식 0.23/0.30/0.33). 공식 충돌 목록을 쓰면 명령 20종 × 시드 5개 모두 기준 통과(기준은 결과 전에 적음).
     - 모드 바꾸기(R30 순서): 시드 20개, 명령 0/0.3/0.4에서 처음 2초 넘어짐 0, 최대 기울기 9.3°/14.0°/14.9°. 떼고 걷기 0/20.
     - 명령 0 표류: 처음 떨어진 뒤 10초에 최대 0.107 m, 단 직후 6/20 시드에서 3초에 0.05–0.14 m(최고 0.083 m/s). 민 뒤 명령 0이면 1초 안에 멈춤(0.10–0.22 m 미끄러짐).
     - 방향: 목표와의 차이를 ±0.5로 잘라 넣으면 vx 0.3/0.5에서 90° 0.8초, 180° 1.5초. vx 0에서는 90° 3/5, 180° 0/5.
     - `crouch`: 넘어짐 0, 몸통 최저 0.065 m. 미끄러짐 0.2 m/s 미만 0.008 m, 0.25–0.3 m/s 0.15 m, 0.35 m/s 넘음 0.31 m(최대 0.38 m).
     - 재지 않음: 움직이는 중 바꾸기, 경사·계단, 바퀴 마찰 0, `action_scale` 1.0.
     - 스크립트(커밋 안 함): scratchpad `oq7/`의 `common.py`, `ab_run.py`, `c_run.py`, `d_run.py`.
   - 구현 기록 1부 (2026-11-13, 브랜치 `feat/m2-addon-roller`, 커밋 50eecdc·7881f7f·2cafede·76647e4):
     - 몸(`bodies/microduck.py`, `bodies/base.py`): `BodyAdapter`에 `use_addons`·`set_addon`·`crouch`·`addon`·`crouching`. 방(`room.build_spec`)이 장착대 부품의 `addon`을 모아 몸을 놓기 전에 `use_addons`를 부르고, 그때만 `robot_spec()`이 바퀴 몸 4개를 `attach_body`(접두 `w_`)로 붙인다. 바퀴는 뗀 상태로 컴파일(충돌 0, 재질 없이 자기 색 알파 0)하고, 컴파일러가 몸체 단위 충돌 필터(`body_contype`)를 0으로 묶으므로 바인드 때 바퀴 몸체 필터만 1로 연다. 장착대 없는 방은 모델이 그대로다(빠른 시험 전체 통과, `nq`·충돌 수 비교 시험).
     - `set_addon(mode)`: 메시 최저점 높이 → 몸통 속도 0 → 충돌(단 상태 = 공식 목록 13개, 몸체·메시 이름으로 맞춤) → 바퀴 보임 → 정책(`velstand` ↔ `roller`, `action_scale` 1.0 ↔ 0.8) → `last_action` 0 → `mj_forward`. `keep_pose`(리셋·복원)는 모델 값과 정책만 맞춘다. `reset()`은 늘 발 상태, 저장본은 몸 이력에 모드·숙이기 남은 스텝·위상·목표 방향을 담아 복원 때 `set_addon(keep_pose)`를 부른다. 충돌·색은 이미 `MODEL_FIELDS`에 있다. 워프(`teleport`)는 바퀴를 단 채 옮긴다.
     - 바퀴 명령: 행동 앞 값 < −0.6이면 vx 0(멈춤), 그 위는 0.25–0.6. 회전 값 × 1 rad/s가 목표 방향의 회전 속도이고, 어댑터가 매 몸 스텝 목표를 적분해 차이를 ±0.5로 잘라 셋째 칸에 넣는다(목표는 몸 방향 ±0.5 밖으로 쌓이지 않음). vy 0, 바퀴 상태에서 머리 명령은 0. 바퀴 관절 `frictionloss` 0.0015(OQ7 시드 범위 0.0005–0.003의 가운데).
     - 장착대 `parts/device/addon_station.py`, 프리셋 `device/addon_roller`: 패드 0.3 × 0.3 m, `mode` attach/detach/toggle, `enable`, `cooldown` 1 s, `max_speed` 0.1 m/s. `in`이 있으면 켜지는 순간 무장(켜진 채면 나중에 패드에서 멈춰도 바꿈), 없으면 패드에 들어설 때 무장. 바꾸기는 `World.requests`의 `("addon", 부품, 모드)`로 env가 몸 스텝 뒤에 한다. 출력 `<이름>.attached`, `body.addon`(방의 첫 장착대가 씀). 관측 `addon`(원-핫 2칸). 패드 제동은 넣지 않았다(선택).
     - `crouch` 행동: task `actions` 순서 `move, head, grab, crouch`. 0.5 초과 + 바퀴 상태 + 수평 속도 ≥ `env.crouch_min_speed`(기본 0.2)일 때만 시작, 175 몸 스텝(3.5 s) 위상 명령, 끝나면 `roller`로 돌아가며 이력 0. 숙이는 동안 넘어짐은 기울기만.
     - 해답: `demo:`에 `addon <장착대>`(패드 안쪽 6 cm까지 걸어가 서서 모드가 바뀔 때까지). 장착대 없는 방의 해답은 그대로.
     - 잰 값(어댑터 그대로, OQ7 시드 변수): 바꾸기 시드 20 × 명령 0/0.3/0.4에서 단 뒤 2초 넘어짐 0, 최대 기울기 9.3°/13.8°/14.1°, 뗀 뒤 2초 넘어짐 0(10.6° 이하), 이어 걷기 넘어짐 0. 높이 +23.6 mm, 뗄 때 −14.9 ~ −23.4 mm. 방향(시드 5): 행동 (1,0,±1) → 1.00 rad/s, (1,0,0.5) → 0.50, (0,0,1) → 1.01, 선 채 (−1,0,1) → 0.78 rad/s, 직진 (1,0,0) 0.32 m/s, 넘어짐 0. 숙이기(시드 10 × 앞 값 4): 진입 속도 0.24/0.37/0.41 m/s에서 8/10·10/10·10/10 시작(멈춤은 0/10), 몸통 최저 0.067 m, 3.5초 미끄러짐 평균 0.41–0.54 m, 넘어짐 0.
     - 예제 `examples/rooms/roller_rink.yaml`(3.0 × 1.2 m, 출발점 옆 장착대 → 반대편 골): `mpg check` 정적 통과, 해답 20/20(평균 7.7 s, 장착 약 2.2 s).
     - 시험 `tests/test_addon.py`: 빠른 13개(장착대 없음/있음 모델, 왕복 충돌·높이·정책, 없는 애드온 거절, 패드 밖·움직임·신호·쿨다운·`enable`·떼기, 리셋, 바퀴 저장본 복원, `crouch` 무시, 행동 순서·`crouch_min_speed` 검증, `demo` 해석), 느린 4개(평지·1 cm 계단 타일에서 바꾼 뒤 2초 시드 10개, 속도에서 `crouch`, `roller_rink` 점검). `tests/test_examples.py`의 KINDS에 `roller_rink`.
     - 알려진 한계·안 한 것: 예제 방은 사용자 범위대로 가장 단순한 판(달고 골까지)이다. spec R30의 `roller_rink` 문장(시간제 문, 복도 끝 떼기 장착대, 계단)은 아직 따르지 않는다. 낮은 막대 밑 `crouch` 통과는 막대 부품이 없어 재지 않았다(몸통 최저만 잼). 움직이는 중 바꾸기, 바퀴로 경사·계단, 바퀴 상태 경로의 계단 경고(정적 점검), 끈(R31) 풀기는 없음. 다음 PR: `puzzles/roller/` 9종과 `roller_route.yaml`.
     - 리뷰 고침: 떼기·바꾸기 장착대가 `crouch` 중에 바꾸면 몸통이 약 22 mm 내려가 0.053 m(높이 기준 0.07 m 아래)가 되어 선 채로 넘어짐으로 판정됐다(숙인 지 2.5 s, 0.026 m/s). 장착대는 숙이는 동안 바꾸지 않고 무장을 유지해 숙이기가 끝나면 바꾼다(`test_station_waits_for_a_crouch_to_end`).
   - 구현 기록 2a부 (2026-11-13, 브랜치 `feat/m2-roller-puzzles`, `feat/m2-addon-roller` 위):
     - 구역 센서 `min_speed`(R5, R30 `speed_gate`): 대상이 그 속도 이상으로 움직이는 동안만 센다. 속도는 스텝 사이 움직임의 0.5초 저역 통과(속도 벡터, `SPEED_TAU`)다. 순간 몸통 속도는 걸음 흔들림 때문에 걷기 0.16–0.38 m/s, 바퀴 0.28–0.57 m/s로 겹쳐서 쓰지 않았다. 고른 값(시드 3개, 4초): 걷기 끝까지 0.20–0.25 m/s(돌며 0.18–0.24), 바퀴 끝까지 0.32–0.53 m/s. 프리셋은 0.28. `rearm: leave`·`direction`이 있으면 켜질 때만 본다. 기본 없음은 예전 동작 그대로(빠른 시험 전체 통과).
     - 제동 띠 `brake_strip`(`parts/trap/brake.py`, 프리셋 `trap/brake_strip`): 몸통이 띠 위에 있고 함정 규칙으로 켜진 동안 바퀴 관절 4개(`BodyAdapter.wheel_dofs`)의 `frictionloss`/`damping`을 띠 값으로, 떠나면 바인드 때 값으로. 방의 첫 띠가 모든 띠의 큰 값을 쓴다. `dof_frictionloss`·`dof_damping`을 `snapshot.MODEL_FIELDS`에 더해 저장본·리셋이 되돌린다. 잰 값(바퀴 0.29–0.48 m/s로 들어올 때, 시드 5개): 마찰 0.01은 밀기 0에서 0.08–0.12 m(0.4–0.6 s), 계속 밀어도 0.12–0.13 m에 멈춤, 넘어짐 0/10. 0.004는 밀면 계속 굴러감, 0.006은 밀며 0.25–0.33 m 뒤 멈춤, 0.03은 0.44 m/s에서 1/3 넘어짐, 0.05는 2/3. `damping` 0.01·0.03은 3개 중 2–3개, 마찰 0.006 + `damping` 0.002는 5개 중 4개 넘어짐(갑자기 멈춤). 기본 0.01, `damping` 0.
     - 계단 경고(`AddonStation.check_warnings`, `mpg check` `parts` 단계, 실패 아님): 방의 첫 장착대가 달기·바꾸기 패드마다 스위치를 모두 켠 바닥을 채운다(다른 떼기·바꾸기 패드는 막음). 높이가 3 mm 넘게 다른 칸(글자 지도 계단, 높이 있는 바닥 칸, `on_height`가 있는 바닥 칸)에 닿으면 경고한다.
     - 조립 프리셋 `puzzles/roller/` 6종(방 하나 전체, `demo:` 포함, `mpg check` 시드 20개):
       - `speed_gate`(3.6 × 0.8 m, 장착대 → 1.2 m 도움닫기 → `min_speed` 0.28 구역 → 한 번 열림 문): 20/20(평균 10.3 s), 걷기만(`addon` 뺀 `demo:`) 0/20.
       - `timed_corridor`(4.2 × 0.8 m, 버튼에서 문까지 2.4 m, `logic/hold` 7.5 s): 버튼이 꺼진 뒤 문 통과까지 걷기 10.1–10.4 s, 바퀴 5.2–5.7 s(시드 10개). 20/20(16.5 s), 걷기만 0/20.
       - `mode_switch_route`(글자 지도 3.6 × 1.2 m, 1 cm 계단 둘 뒤 단 위 골, 떼기 패드 0.7 × 1.18 m): 20/20(15.5 s). 복도 폭 0.8 m에서는 4/6이었다: 굴러가다 벽 쪽(y −0.25)으로 밀린 채 떼면 계획 격자의 부풀린 칸 안이라 해답이 제자리 돌기(`[-1, 0, 1]`)를 내는데, 걷는 몸의 제자리 돌기는 3초에 0.15 rad뿐이라 멈춘다.
       - `brake_strip`(같은 지도, 짧은 떼기 패드 0.3 m를 0.4 m 제동 띠 위에): 20/20(16.7 s). 띠 마찰을 0.0015(제동 없음)로 하면 18/20, 16.1–35.2 s.
       - `one_way_station`(글자 지도 4.0 × 1.2 m, 서쪽 달기 전용, 속도 구역 문, 동쪽 계단 앞 떼기 전용; 출발은 가운데에서 서쪽을 봄): 20/20(24.1 s).
       - `crosswind`(시간제 복도 4.2 × 1.2 m + 옆바람 0.6 m): 기본 0.5 N 20/20(12.9 s). 세기별(시드 20개, `demo:` 해답): 0.5 N 20/20, 0.75 N 18/20(넘어짐 2), 1.0 N 12/20(실패는 모두 넘어짐; 시드 0–9만은 9/10), 2 N·3 N 0/10. 1 N 프리셋 `trap/crosswind`를 그대로 두지 않고 0.5로 했다.
     - 예제: `roller_rink.yaml`을 spec R30 문장대로 바꿈(글자 지도 칸 0.15 m, 바닥 5.25 × 1.5 m: 장착대 → 버튼 → 2.5 m 복도 → 시간제 문 7.5 s → 떼기 패드 0.7 m → 계단 1 cm 둘 → 골). `mpg check` 19/20(평균 21.7 s, 시드 9는 뗀 뒤 계단에서 넘어짐), 걷기만 0/20. 새 `roller_route.yaml`(같은 지도: 장착대 → 속도 구역 → 한 번 열림 문 → 옆바람 0.3 N → 제동 띠 위 떼기 패드 0.3 m → 계단 → 골): 20/20(26.6 s).
       - 거쳐 온 판: 칸 0.1 m 지도는 54칸이라 공유 방 한도(한 변 40칸, `test_community`)를 넘어 칸 0.15 m로 다시 그렸다. 복도 1.2 m·문틈 0.6 m에서 `roller_rink` 13/20: 굴러가며 y로 ±0.25 m 흔들려 문틀에 부딪혀 넘어지거나(2) 걸려 멈춤(5). 문틈을 1.0 m로 넓혀 19/20, 칸 0.15 m(복도 1.2 m, 문틈 0.9 m)로 바꾸니 17/20(문틀 걸림 1, 벽 옆에서 뗀 뒤 제자리 돌기로 멈춤 1, 계단 넘어짐 1) → 복도 1.5 m, 문틈 1.2 m(문 폭 1.16 m)로 19/20. `roller_route`의 옆바람은 0.5 N에서 18/20(복도 1.2 m), 0.3 N과 0 N 모두 19/20(시드 4는 바람과 무관하게 장착대 직후 방향이 틀어져 벽에 붙어 넘어짐)이라 0.3 N으로 두었고, 복도 1.5 m에서 20/20.
     - 시험: `tests/test_roller_puzzles.py` 빠른 15개(`min_speed` 켬·끔·leave·기본값, 제동 켬·끔·리셋·저장본·`in`·띠 둘·바퀴 없음 경고, 계단 경고 있음·없음, `roller_rink` 경고 없음, 프리셋 목록·정적 단계 6개), 느린 3개(걷기만으로 `timed_corridor`·`speed_gate`·`roller_rink`를 시드 3개 모두 못 품). 프리셋 20시드 점검은 `test_examples.py`의 방 프리셋 시험, 예제는 `test_check2.py`가 맡는다(KINDS에 `roller_route`).
     - 알려진 한계: 해답의 바퀴 조향이 흔들린다(굴러가며 방향 ±0.6 rad, y ±0.25 m). 그래서 문틈과 복도를 넓게 잡았다. 장착대 직후 방향이 틀어진 판(시드 4)은 바람 없이도 벽에 붙어 넘어진다. 계단 경고는 떼기 패드가 길을 다 막을 때만 조용하다. 장착대 패드 자체의 제동은 없다(제동 띠로). 0.01 띠 위에 멈춘 오리는 바퀴로 다시 못 나간다(떼기 장착대 밑이나 길 끝에만). 움직이는 중 바꾸기, 바퀴로 경사·계단은 여전히 재지 않음.
     - 2b부로 미룸(잰 값 필요): `glide_under`(낮은 막대 부품과 `crouch` 통과 측정), `fast_herd`(도망 NPC 속도를 걷기와 바퀴 사이로, NPC 속도 경고 끄기), `roll_delivery`(물건을 문 채 바퀴로 옮기기). 낮은 막대 부품도 만들지 않았다.
     - 스크립트(커밋 안 함): scratchpad `r30b/`의 `common.py`, `speeds.py`·`speeds2.py`(속도), `brake.py`(제동 값), `turn.py`(제자리 돌기), `runroom.py`(해답 판, `--walk`, `--set`), `timing.py`(버튼에서 문까지 시간), `trace.py`·`dbg_msr.py`(판 추적), `chk.py`(`mpg check`).
   - 구현 기록 2b부 (2026-11-13, 브랜치 `feat/m2-roller-2b`, `feat/m2-roller-puzzles` 위; 범위는 사용자 "승인": 낮은 막대와 `glide`, `fast_herd` 뺌, `roll_delivery`는 떼고 놓기):
     - 측정(결과 전에 기준을 적음, scratchpad `r30c/criterion.txt`): 숙이기는 몸통을 낮추지만 머리(`jaw_soft`) 꼭대기는 0.256–0.268 m에서 0.197–0.200 m로만 내려간다(숙인 뒤 약 0.7–2.3초). 막대 아랫면 창 0.210–0.246 m. 0.228 m 막대(1.5 m 복도, 시드 20개): 숙이지 않으면 20/20 막힘. 숙이기 시작 거리 0.2/0.25/0.3/0.35/0.4/0.5 m에서 진입 0.32–0.48 m/s는 6/14/20/20/20/7, 0.35–0.41 m/s는 19/20/20/20/0/0 통과, 진입 0.29–0.35 m/s는 9/20 이하. 끝까지 밀면 3–4초 뒤 0.38–0.45 m/s.
     - 낮은 막대 `low_bar`(`parts/device/low_bar.py`, 프리셋 `device/low_bar`): 관절 없는 상자 4 × 3 cm, `length`·`axis`·`bottom`(기본 0.228, 범위 0.21–0.246 m). 상태 없음. 계획: 방에 바퀴를 다는 장착대(`attach`/`toggle`)가 있으면 막대가 막지 않고(`blockers` 없음), 없으면 벽처럼 막고 `parts` 단계가 경고한다. 걷는 몸이 밑으로 가는지는 계획이 아니라 해답 점검이 본다.
     - 해답 `glide <막대>`(`solver.glide_act`): 중심선에서 1 m(`GLIDE_START`)보다 가까우면 1.2 m(`GLIDE_RUNUP`)로 물러남 → 제자리에서 막대를 마주 봄(0.15 rad) → 막대 가운데를 지나는 선을 따라 끝까지 밀며 옆 어긋남 1 m당 1.5 rad로 고침 → 몸통이 중심선 0.30 m(`GLIDE_CROUCH`, 두 진입 속도 모두 20/20인 유일한 값) 앞에서 `crouch` → 중심선 0.3 m 지나면 끝. `mpg check` `actions` 단계는 `glide`가 있으면 `crouch` 행동을 요구한다.
     - `glide_under`(5.0 × 1.5 m, 장착대 막대 1.8 m 앞, 골 뒤): `mpg check` 20/20(평균 14.7 s), 숙이기 시작 0.26–0.30 m 앞 0.36–0.53 m/s, 막대 접촉 0. `glide` 뺀 `demo:`(바퀴로 그냥 감) 0/20(넘어짐 10), 걷기만 0/20.
     - `roll_delivery`(글자 지도 칸 0.1 m, 7.6 × 1.6 m: 받침대 위 열쇠 → 장착대 → 버튼 → 2.4 m 복도 → 시간제 문 7.5 s → 떼기 패드 → 바구니 → `basket.ok` 문 → 골): `mpg check` 19/20(평균 37.5 s, 시드 2는 문틀에 부딪혀 넘어짐), 걷기만(`addon` 뺀 `demo:`) 0/20(16번은 닫힌 문 앞에서 기다림, 넘어짐 4). 시간 제한은 `timed_corridor`와 같은 잰 값(버튼에서 문까지 걷기 10.1–10.4 s, 바퀴 5.2–5.7 s)이다. 전체 에피소드 길이로 제한하는 판은 버리고 문으로 했다: 잡기·달기·놓기 시간 흔들림(판마다 약 6 s)이 걷기와 바퀴의 차이보다 커서 걷기만 실패를 보장하지 못한다.
       - 거쳐 온 판(시드 20개): 복도 폭 떼기 패드 0.7 × 1.58 m 14/20(실패 6개 중 5개는 굴러가며 남쪽 벽으로 흘러 열쇠를 떨어뜨리거나 문틀에 부딪힘. 패드 전체가 목표라 해답 조향에 가운데로 돌아오는 힘이 없음). 패드 0.7 × 0.6 m 7/20(패드에 못 멈추고 지나침), 버튼 폭 0.6 m 9/20(바퀴로 버튼을 놓치고 못 돌아옴), 거기에 복도 폭 제동 띠 10/20. 패드 0.7 × 1.0 m 15/20, 여기에 같은 크기 제동 띠(`trap/brake_strip`) 19/20 → 채택.
     - task: 두 프리셋은 행동이 더 필요해 `examples/task_glide_under.yaml`(`[move, crouch]`)·`examples/task_roll_delivery.yaml`(`[move, head, grab]`, 60 s)를 두고, `tests/test_examples.py`의 방 프리셋 점검이 이 task로 돈다.
     - `fast_herd`는 뺐다: 도망 NPC는 `sense` 0.4 m 안에서만 움직여, NPC 속도 0.25/0.28/0.30 m/s 모두 걷기로 20/20 몰린다(바퀴도 20/20). 새 NPC 행동과 함께 다시 본다. 바퀴 상태로 놓기는 0/20(머리 명령 0 고정, `microduck.py` 관측). 문 채 장착 뒤 1.5 m 구르기는 열쇠 20/20.
     - 시험: `tests/test_roller_puzzles.py` 빠른 시험 더함(막대 정적·충돌·높이·상태 없음, `bottom` 범위 밖 둘, 계획 규칙 장착대 없음/attach/toggle/detach, `glide` 해석과 오류, `crouch` 행동 요구, `glide_act` 단계: 물러남·돌기·밀기와 조향·`crouch`·끝; 프리셋 8종 목록·정적 단계), 느린 시험 더함(`roll_delivery` 걷기만, `glide` 없이 막힘, 시드 3개).
     - 알려진 한계: 막대 통과는 진입 속도에 달려 있어 1 m 넘는 곧은 도움닫기가 필요하다. 해답은 막대에 막히면 다시 시도하지 않는다. 계획 규칙은 장착대가 있는지만 보고 그 장착대에 닿는지·도움닫기 길이는 보지 않는다(해답 점검이 잡는다). 걷는 몸의 최고점은 0.266–0.275 m(명령 0·0.15·0.3, 시드 10개, 리뷰 때 잼)로 바퀴 상태보다 높아서, 허용 범위 끝 0.246 m 막대에도 막힌다. `roll_delivery`의 바퀴 조향은 여전히 흔들려(넓은 목표에서 옆으로 흐름) 패드를 복도보다 좁게 하고 제동 띠로 멈춘다. `cube_weight`처럼 무거운 물건은 발로 놓다 떨어뜨려 12/20이라 쓰지 않았다.
     - 스크립트(커밋 안 함): scratchpad `r30c/`(측정: `a1_profile.py`, `a2_bar.py`, `b_herd.py`, `c_grab.py`, `c_relay.py`), `r30d/`(`runroom.py` 해답 판·`--demo`·`--set`, `chk.py` `mpg check`, `rd_*.yaml` 판들).
16d. 견인 끈(R31): 먼저 OQ9를 잰다(빈 방에서 썰매 질량·마찰을 올려 가며 velstand 넘어짐과 속도, 장력 기록). 그다음 `rope.py`(매듭, tendon, `connect` 구속, 채널), 부리 묶기·풀기, 끊어짐, `hitch_station`, 저장본·리셋 경로, 정적 점검 경고(문 폭, 꺾임, 질량), `tow`/`untow` 하위 목표, 프리셋 8종, 예제 `tow_yard.yaml`. 구속은 사이트 두 개 `connect`, 붙이기는 매듭 이동·속도 0 → `eq_active` 순서. 워프는 끈 묶음 전체, 고정 끝 끈은 먼저 풂. 매듭 고리 손잡이. NPC 목줄은 구동 이득 0. 매듭 20 g 이상·바닥만 충돌·트리거 제외, `reel_speed`, 순간이동·`set_addon` 전 끈 처리, `rope_door`.  NPC 끈 워프는 평면 관절 값으로, 이득·활동 영역은 끈 상태에서 다시 계산. `attach_failed`.
   - 구현 기록 1부 (2026-11-13, 브랜치 `feat/m2-tow`; 범위: R31에서 `puzzles/tow/` 프리셋, `rope_door`·도르래, `tow_yard.yaml`을 뺀 전부):
     - 몸 걸이: `BodyAdapter.tow_site(spec)`(기본 None). Microduck은 끈이 있는 방에서만 몸통에 사이트 `tow_hitch`를 더한다(몸통 기준 (0.007, 0, −0.0756) m = 엉덩이 관절 축 가운데 아래, 선 자세에서 바닥 위 0.04 m, OQ9). 몸 이름은 `bodies/` 안에만 있다.
     - 끈 `parts/device/rope.py`: 매듭 둘(부품 `<끈>/a`, `<끈>/b`, 자유 몸체 원판, `knot_mass` 기본·최소 0.02 kg, 바닥 기하와만 `pair` 접촉, 고리 `handle_height` 0.16 m를 부리가 잡음, 종류 `knot`는 TRIGGERS에 없음), 공간 힘줄 `part/<끈>/rope`(범위 0..현재 길이), 걸이마다 `connect` 구속 둘(`part/<끈>/<a|b>@<걸이>`, 꺼진 채 컴파일, `eq_active`로 켬). 묶기 = 다른 끝까지 거리 ≤ 현재 길이 확인(아니면 `attach_failed` 한 스텝) → 매듭을 걸이 사이트로 옮기고 속도 0 → `eq_active` 켬. 장력 = 그 힘줄의 `mjCNSTR_LIMIT_TENDON` 행 `efc_force` 합(물리 스텝마다, 몸 스텝 평균을 `.tension`으로). `max_tension` 0.9 N을 `snap_time` 0.2 s 넘게 넘으면 두 끝을 풀고 `snapped`(리셋까지). 길이는 실행 중 값(`bind` 가능), 몸 스텝마다 `reel_speed`로 목표를 따라가며 `tendon_range`를 고친다. 현재값·묶인 끝·타이머는 `save_state`, `tendon_range`는 복원 때 현재값으로 다시 씀.
     - 걸이: 몸, `hitch: true` 물체(모든 잡을 수 있는 물체에 새 값, 끈 있는 방에서 그 물체의 충돌 기하 `priority` 2라 물체 마찰이 바닥보다 우선), NPC(`hitch_offset`), 고정 기둥 `anchor`(새 부품, 그리기만). 방의 걸이 수 ≤ `max_hitches`(16, 몸 포함), 넘으면 방 읽기 오류. 중계기(R34)는 아직 없어 뺐다.
     - 묶기·풀기: 연결 패드 `hitch_station`(새 부품, 장착대와 같은 무장·`max_speed` 0.1 m/s·`cooldown`·`enable`, `mode` attach/detach/toggle, 출력 `.towing`), 채널 `attach_a`/`detach_a`/`attach_b`/`detach_b`(켜지는 순간), 부리(매듭을 걸이 `clip_radius` 0.04 m 안에서 놓으면 묶음, 묶인 매듭을 `untie_time` 0.5 s 물면 풂). `a`/`b`가 없거나 패드면 `hitch_radius`(0.3 m, 끈 값) 안 가장 가까운 걸이.
     - 출력: `<끈>.attached_a/attached_b/tension/snapped/attach_failed`, `body.towing`(방의 첫 끈이 씀). 관측 `tow` 4칸. 관측 `beak` 물체 종류 `knot` 10.
     - NPC: 묶인 동안 구동 이득(`gainprm`/`biasprm`) 0, 활동 영역(`jnt_range`)을 넓힘. 풀리면 이득을 되돌리고, 활동 영역 밖이면 안쪽 점으로 걸어 들어간 뒤 한계를 다시 건다(`returning`). 리셋 뒤(`after_reset`), 저장본 복원, 묶기 변화마다 끈 상태에서 다시 계산(`sync_npcs`).
     - 리셋·저장본·순간이동: 새 `Part.after_reset`(리셋의 `mj_forward` 뒤)에서 기본 끝을 묶는다. 몸 워프(`env.warp`)는 끈으로 이어진 묶음(BFS)을 같은 상대 자세로 옮기고(NPC는 평면 관절 값), 묶음 안 `anchor` 끝은 먼저 푼다. 물체 워프, 스프링 `kick`, 디스펜서 보관(`park`), 받는 부품 뱉기는 그 물체의 끈을 먼저 풀고(`World.untie`), `set_addon` 요청은 몸 쪽 끝을 먼저 푼다. 스프링이 몸을 차는 경우는 풀지 않는다. 끈 없는 방은 이 경로를 타지 않는다.
     - 점검: 정적 실패 = `hitch_radius`·`clip_radius` > `length`, 기본 연결 두 끝이 무작위 범위 최악에서 `length` 밖. 경고 = 끄는 짐(마찰 × 무게) > 0.4 N, `demo:`의 `tow`~`untow` 사이 길에 끄는 물체보다 좁은 문, `length` 안 90° 넘는 꺾임.
     - 해답: `demo:` `tow <끈> <걸이>`, `untow <끈>`(그 끈의 맞는 `mode` 패드 중 걸이에 가장 가까운 것으로 걸어가 섬; 이미 쓴 패드 위면 `_walk_off`로 내려섰다가 다시 올라섬). 끌고 목적지까지 가는 하위 목표는 없다(2부 프리셋이 필요하면 더함).
     - 부리: `add_beak`가 이미 있는 부리에 새 잡을 물체를 더한다(끈이 매듭을 나중에 더해도 됨).
     - 잰 값: 0.4 m 끈, 0.3 kg 썰매 마찰 0.1(짐 0.29 N), 걷기 5초: 넘어짐 0, 끊어짐 0, 매듭 사이 최대 0.4022 m, 썰매 0.81 m·몸 0.96 m 이동(이 방은 무작위 값이 없어 시드 20개가 같은 궤적). 장력은 출발 순간 최대 1.23 N(0.2 s 미만이라 안 끊어짐), 2초 뒤 평균 0.23 N. 0.2 kg·마찰 0.1은 0.1–0.43 N, 0.3 kg·마찰 0.5(1.47 N)는 출발하자 끊어짐. 레버로 목표 0.4 → 0.1 m(기둥에 묶인 0.3 kg, 마찰 0.5, `max_tension` 100): 최대 장력 `reel_speed` 0.05/0.1/0.5/2.0 m/s에서 1.86/2.11/5.06/13.7 N(시험 기준: 0.05에서 < 1.5 × 1.47 N, 2.0에서 > 3 × 1.47 N). 해답 `tow`→`untow`(toggle 패드, 쿨다운 0.5 s) 8.3 s.
     - 시험 `tests/test_rope.py` 23개(모두 빠름): 끈 없는 방 모델 그대로(힘줄·`pair`·몸 사이트 없음, 구속 수, `priority` 0), 걸이 이름·상한 오류, 늘어짐·당김, `attach_failed`, 채널·패드(밖·움직임 포함)·부리 묶기와 풀기, 무작위 배치 뒤 매듭 1 cm, 레버 감기, 몸 워프(같은 상대 위치)와 `anchor` 풀기, 물체 워프·스프링·디스펜서 회수·`set_addon` 풀기, 매듭이 버튼 안 누름, 끊어짐·`on_snap: fail`, 저장본·리셋, 같은 시드, NPC 이득·목줄·돌아오기, 관측 `tow`, `demo:` 해석과 해답, 정적 점검·경고.
     - 리뷰 고침: (1) 끌리는 NPC는 이득 0·바닥 마찰 없음이라 몸이 서도 0.16 m/s로 미끄러져 몸을 지나쳤다. 묶인 동안 평면 이동 관절 `dof_damping` 1.0 N·s/m(풀면 되돌림; `MODEL_FIELDS`라 저장본 그대로). 잰 값(0.3 kg NPC, 4초 끝까지 걷기 뒤 멈춤): 감쇠 0/0.2/0.5/0.8/1.0/1.5에서 0.02 m/s 아래까지 안 멈춤/3초 넘음/1.66/1.04/0.82/0.56 s, 걷는 동안 장력 평균 0/0.04/0.10/0.17/0.20/0.29 N(0.4 N 한도 아래). (2) 부리로 물어 푼 매듭을 그 자리에서 놓으면 `clip_radius`로 다시 묶였다: 이번에 물고 있는 동안 푼 끝은 놓을 때 묶지 않는다(`hold_untied`). (3) 가이드의 `.tension` 단위를 `max_tension` 대비 0..1로 고침. (4) 길 경고 시험 더함(곧은 길 경고 없음, 0.4 m 문을 0.1 m 큐브는 통과·0.5 m 막대는 경고, 벽 끝을 도는 U자 길은 꺾임 경고); 코드 버그는 나오지 않음. 시험 25개.
     - 알려진 한계·안 한 것: 매듭 고리 0.16 m는 정한 값(OQ8 R31 높이는 열려 있음). 중계기 걸이 없음(R34 전). 끊어진 끈은 리셋까지 못 씀. 몸이 스프링에 차일 때는 안 풂. 부품 프리셋은 2부(`puzzles/tow/`)와 함께. 2부: `puzzles/tow/` 8종, `rope_door`·도르래, `tow_yard.yaml`.
   - 구현 기록 2a부 (2026-11-13, 브랜치 `feat/m2-tow-2`, `feat/m2-tow` 위; 범위: `drag` 하위 목표, `puzzles/tow/` 중 `rope_door`·도르래·`handoff`가 필요 없는 것, `tow_yard.yaml`. `via:`·`handoff`·`rope_door`와 그 프리셋은 2b):
     - 해답 `drag <끈> <부품>`(`room.parse_demo`, `ScriptedSolver.drag_act`): 끌리는 걸이 = 끈에서 몸 반대쪽 끝(몸에 안 묶인 끈이면 다른 끈들로 몸에 이어진 끝의 반대쪽, 사슬). 단계가 시작할 때 걸이에서 부품 가운데를 지나는 선 위, 가운데 너머 `max(끈 현재 길이, 몸–걸이 거리) + DRAG_PAST`(0.25 m) 점(벽 앞이면 그 선에서 빈 칸까지 당김)으로 `_point_field` 길을 걷는다. 걸이가 부품 바닥 영역 안 `DRAG_MARGIN`(5 cm)에 들어오면 끝. 가는 길에 걸이가 풀리면(부품의 `detach_*` 채널) 그것도 끝, 시작부터 끌 것이 없으면 섬, 겨눈 점 0.1 m 안인데 안 들어왔으면 섬. 부품은 바닥 영역(`half`)이 있어야 함(압력판·구역·패드·구덩이; 벽 판·골은 오류).
     - 점검: `drag` 대상 센서는 `actions`의 `grab` 요구에서 뺀다. 길 경고(`tow_path_warnings`)에 `drag` 구간과, 리셋 때 몸과 물체·NPC를 잇는 끈의 시작부터 길을 더함. 새 정적 실패: `drag <끈> <구덩이>`로 끄는 상자 높이가 구덩이 깊이(가장자리–받침면 `CATCH_DEPTH` 0.5 m)와 1 cm 넘게 다르면(`pit_fill_problems`).
     - 연결 패드 고침: 몸에 묶이고 다른 끝이 풀린 끈을 `attach`(`toggle`) 패드가 아무것도 안 하던 것(`towing`만 보던 조건)을 그 끝을 묶게 고침. `tow_yard`(기둥 끝을 버튼으로 푼 끈을 썰매에 다시 묶음)에서 나옴.
     - 프리셋 `puzzles/tow/` 4종(task는 `move`만, 짐 0.4 N 아래, 모두 정적 단계 경고 없음): `sled_delivery`(0.2 kg·마찰 0.1 썰매를 패드에서 이어 구역으로, 한 번 열림 문), `drop_and_go`(묶인 채 시작, 수레 0.12 m·0.15 kg, 따라 열리는 문, 구역 끝 풀기 패드), `chain_dropoff`(수레 0.1 kg 둘 사슬, 구역 `drop_b`(북, `cart_b`만)·`drop_a`(남, `cart_a`만)이 각 끈의 `detach_b`, 문은 둘 다), `tethered_start`(기둥에 0.6 m 끈, `on_snap: fail`, 버튼이 `detach_b`). 예제 `examples/rooms/tow_yard.yaml`(기둥 → 버튼 → 패드에서 썰매 → 구역 → 풀기 패드 → 문).
     - 잰 값: 판정 기준은 20시드 전에 `criterion.txt`에 적음(정적 통과, `tow_yard` ≥ 18/20, 방 프리셋 시험이 프리셋도 ≥ 18/20 요구). `mpg check` 20시드: `sled_delivery` 20/20(평균 19.5 s), `drop_and_go` 20/20(17.4 s), `chain_dropoff` 1판(구역 둘 다 길 위 y 0) 20/20(21.6 s)·2판(구역 y +0.35/−0.35) 20/20(25.6 s), `tethered_start` 20/20(20.5 s), `tow_yard` 20/20(31.2 s, 40 s 판). 음성 대조(각 20시드): `sled_delivery` `demo: [goto goal]` 0/20·`[tow, goto goal]` 0/20, `drop_and_go` `untow` 뺌 0/20, `chain_dropoff` 2판 `[goto goal]` 0/20·뒷수레 `drag` 뺌 0/20, `tethered_start` `[goto goal]` 0/20(끊어짐), `tow_yard` 버튼 뺌 0/20(끊어짐)·`untow` 뺌 0/20. 골로 똑바로 걷기(방향 유지, 문 무시): 프리셋 4종(2판)·`tow_yard` 0/20, `chain_dropoff` 1판은 14/20이라 2판으로 구역을 비킴. 압력판 가장자리(4 mm): 0.1 m 큐브 마찰 0.1을 0.4 m 끈으로 끌면 0.2 kg은 가장자리에서 끊어짐(최대 2.8 N), 0.15 kg은 넘어감(최대 2.04 N, 상자 가운데가 2.1 cm 들림), 0.06 m·0.1 kg은 넘어감(최대 1.39 N; 무작위 없는 방 한 판씩). 넓은 수레 시도: 0.3 m 큐브(손잡이 포함 0.33 m) + 0.3 m 문(벽 틈 0.36 m)은 해답이 문 앞에서 섬 0/5, 벽 틈 0.40/0.44/0.48 m는 3/3씩이라 큐브가 문보다 넓을 수 없음.
     - 시험: `tests/test_rope.py`에 `drag` 해석·오류, 겨눈 점과 끝남, 끌 것 없음·풀림, 사슬 걸이, 짧은 끌기(7.2 s), `grab` 면제, `drag` 길 경고, 풀린 끝 패드 묶기(8개 더해 33개). 새 `tests/test_tow_puzzles.py`: 프리셋 목록·정적 통과, 구덩이 채움 점검, slow 음성 대조(끈 단계 뺌·똑바로 걷기, 각 시드 3개). `test_examples.py` KINDS에 `tow_yard`.
     - 계획·spec과 다른 점: (1)·(2)는 사용자 "승인"(2026-11-13), spec R31에 씀(647c3e9). 나머지는 기록만. (1) 짐을 두는 곳이 압력판(R5)이 아니라 물체 구역(`input/object_zone`, R5)이다(`sled_delivery`, `chain_dropoff`의 "판", `tow_yard`의 압력판). 압력판의 4 mm 솟은 가장자리가 끌리는 0.2 kg 상자를 막는다(위 잰 값). (2) `drop_and_go`의 수레는 문보다 넓지 않다(위 시도). 풀기는 따라 열리는 문과 구역이 강제한다. (3) `pit_bridge`는 만들지 않았다: 구덩이는 바닥 없이 0.5 m 아래 받침면까지 뚫려 있고 큐브는 최대 0.3 m이며, 계획(`hole_now`)은 상자가 들어가도 구덩이를 구멍으로 본다. 구덩이 `depth`(바닥판) 값과 "채운 구덩이는 바닥" 규칙(R16 변경) 또는 더 큰 상자 부품이 필요하다. 채움 점검만 넣었다. (4) `drag`는 걸이가 가는 길에 풀려도 끝난다(spec은 "영역 안에 들어오면 끝").
     - 리뷰 고침: (1) 길 경고의 `demo:` 구간(`tow_legs`)이 다른 끈의 `drag`에서 몸 위치를 옮기지 않아 구간을 건너뛰었고, 목표 구역이 그 끈의 `detach_*` 채널로 풀어도 계속 끄는 것으로 봤다(`chain_dropoff`의 `front`: [-2.1, 0] → [0.6, -0.35], 그 뒤 골까지도 `cart_a`). 이제 모든 끈의 `drag`가 몸을 옮기고, 그 끈을 푸는 부품으로의 `drag` 뒤에는 끌지 않는다. (2) 길 경고·구덩이 채움 점검이 끄는 끝을 설정의 `b` 끝으로 잡아, 사슬 끈을 `a: cart_b, b: cart_a`로 쓰면 다른 수레를 쟀다. 리셋 때 묶인 끈들로 몸에서 이어지는 반대쪽 끝을 찾는다(`far_end`, 해답 `towed_hitch`와 같은 함수; 리셋 때 사슬로 끄는 끈도 시작부터 끄는 것으로 봄). 시험 2개 더함(`chain_dropoff` 구간, 거꾸로 쓴 사슬).
     - 알려진 한계: `drag`는 곧은 선으로 겨눈다(끌리는 물체가 모서리를 돌지 않게 길을 곧게). `tow_yard`는 40 s 판에서 평균 31.2 s라 여유가 9 s 정도다. 프리셋 task 파일은 만들지 않았다(모두 `move`만). 2b: `cleat_hold`, `npc_leash`, `overload`, `pit_bridge`(위 결정 뒤), `rope_door`, `via:`, `handoff`.
   - 구현 기록 2b부 (2026-11-13, 브랜치 `feat/m2-tow-3`, main(1부·2a부 합침) 위; 범위: 끈 `via:`, `rope_door`, 패드 `handoff`, NPC 활동 영역 확인, 프리셋 `cleat_hold`·`npc_leash`. `overload`·`pit_bridge`는 2c):
     - 도르래 `via:`(`rope.py`): `[[x, y, z], ...]` 월드 사이트(`part/<끈>/via<i>`, 바퀴 그림, 충돌 없음)를 힘줄이 a → b 차례로 지난다. 길이 한계는 경로 전체. 묶기·패드 묶기(`_span`/`path_len`)와 정적 점검(기본 두 끝, 무작위 범위 최악을 첫·마지막 도르래까지 더함)이 경로 길이로 잰다. `hitch_radius`·`clip_radius` 점검은 `length`에서 첫–마지막 도르래 사이 경로를 뺀 값과 비교. 몸 워프: 도르래 끈은 함께 못 옮기므로 묶음 쪽 끝을 풀고 끈은 남긴다(고정 끝 규칙의 도르래판).
     - 고정 걸이: 걸이 종류를 `hitch_kind`(부품 속성)로 정함. `anchor`·`rope_door`가 고정(`FIXED_KINDS`): 워프 때 풀고, `reset_towed`는 끌리는 것으로 보지 않음.
     - 끈 문 `parts/device/rope_door.py`(새 부품, 프리셋 `device/rope_door`): 수직 미끄럼 관절, 구동기 없음, 중력으로 닫힘(아래 한계에 얹힘), 벽·바닥과 충돌 제외, 몸·물체와 충돌. R9 문처럼 `width`/`axis`/`thickness`/`height`, 마커. 새 값 `mass`(기본 0.05, 최대 0.15 kg = 1.47 N), `clearance`(0.34 m), `travel`(0.45 m), `damping`(0.3). 걸이는 판 윗변 가운데. 역할 gate: 정적 순서는 묶일 수 있는 끈(`a`/`b`로 이름 불림이나 빈 끝이 있는 끈)에 판 위 도르래가 있으면 열림(`plan_inputs_on`; 규칙은 아래 리뷰 고침), 아니면 정적 실패. 살아 있는 계획(`logic.live_free`, 해답 캐시 키)은 새 `gate_passable`로 `passable_state` 1.0까지(다 들렸을 때만) 지나게 함. 다른 문은 그대로 0.3. 출력 `<문>.lift`(`clearance` 대비 0..1 = `state()`), `<문>.open`.
       - `clearance` 잰 값: 걷는 Microduck 충돌 기하 꼭대기(aabb) 최대 0.300 m(빈 방, 걸으며 1.5 초), 행동에 `head`가 있는 방에서 머리 명령 −1/0/1일 때 0.300/0.303/0.295 m, `cleat_hold` 문 밑을 지날 때 0.286–0.288 m(시드 3개). 4 cm 여유로 0.34 m.
     - 패드 `handoff`(`hitch_station.py`, `Rope.station_handoff`): `mode: handoff`와 `to: <걸이>`(필수, `body`·없는 이름은 오류, 다른 방식에 `to`는 오류). 몸에 묶인 끝을 같은 스텝에 `to` 걸이로 옮김(`tie`가 옛 구속 끔 → 매듭 이동·속도 0 → 새 구속 켬). 다른 끝까지 경로가 현재 길이보다 길면 `attach_failed`, 끝은 몸에 남음. `enable`·`cooldown`·`max_speed`·무장 규칙은 다른 방식과 같음. 해답 `untow`가 `handoff` 패드도 씀(새 하위 목표 없음).
     - NPC 활동 영역: 1부에서 이미 끈 상태로 다시 계산(리셋 뒤 `after_reset`, 저장본 복원 `load_state`, 묶기 변화 `tie`/`untie`의 `sync_npcs`), `towed`·`returning`은 `state_keys`에 없음. 빠진 것이 없어 시험만 더함(복원 두 경우, 채널로 푼 뒤 돌아오기). 새 계획 규칙: 끈이 NPC에 묶일 수 있으면 NPC가 켜는 센서(`trigger` npc/any, `objects` 없음)는 목줄 밖이어도 스위치(`Rope.add_room_parts`; 물체 센서의 `Grabbable` 규칙과 같은 방식). 이것이 없으면 `npc_leash`의 구역이 `sensor`로 남아 정적 순서가 실패했다.
     - 프리셋 `puzzles/tow/cleat_hold`(4.0 × 1.5 m, 칸막이 x 0.6, 끈 문 0.05 kg, 도르래 (0.6, 0, 0.85), 끈 1.5 m `max_tension` 1.5 N, 오리는 묶인 채 문 앞 0.35 m에서 서쪽을 보고 시작, 고정 기둥 x −0.4, `handoff` 패드 x −0.55; `demo: [untow rope, goto goal]`). 끈 길이는 오리가 문에서 0.4 m 안이면 문이 닫히게(1.5 ≥ 0.597 + √(0.4² + 0.81²)), 기둥 거리는 들린 높이 0.38 m가 되게 정함(잰 값 0.381 m). `puzzles/tow/npc_leash`(5.0 × 1.5 m, 도망 NPC 우리 `leash` 0.2 × 0.12 m, 우리 옆 `attach` 패드, 끈 0.55 m·`hitch_radius` 0.5 m·`detach_b: zone`, `trigger: npc` 구역 0.6 m, 한 번 열림 문; `demo: [tow rope bot, drag rope zone, goto goal]`).
     - 잰 값: 판정 기준은 20시드 전에 `criterion.txt`에 적음(정적 통과·경고 없음, `mpg check` ≥ 19/20, 음성 대조 각 0/20). 시도한 판(시드 3개 미리 보기): `npc_leash` 1판(구역 0.5 m, 구역 끝 `detach` 패드) 0/3(오리 길이 y −0.25로 처져 NPC가 구역 안 5 cm 선 1 cm 밖 y −0.24에서 멈춤), 2판(구역 0.6 m, 패드 x 0.55) 3/3, 3판(패드 대신 구역이 `detach_b`) 3/3 → 3판을 20시드에. `cleat_hold` 1판 3/3 → 20시드에. 20시드(`mpg check`): `cleat_hold` 20/20(평균 17.4 s), `npc_leash` 20/20(20.3 s). 음성 대조(각 20시드): 똑바로 걷기 둘 다 0/20, `cleat_hold` `demo: [goto goal]` 0/20·`handoff` 패드 빼고 `[goto post, goto goal]` 0/20, `npc_leash` `[goto goal]` 0/20·`[herd bot zone, goto goal]` 0/20. 덧붙임(기준 밖): 기둥까지 간 뒤 똑바로 걸으면 0/20, 문 0.2 m 안에서 들린 높이 < 1 mm(닫힘), 오리는 x 0.47–0.50에서 막힘.
       - `cleat_hold` 끈(20시드, 물리 스텝 장력): 들어 올리며 걷는 동안 평균 0.72–0.74 N(판 0.05 kg + 매듭 0.02 kg = 0.69 N), 95%값 0.88–0.90 N, 최대 0.95–2.50 N, 0.9 N 위 연속 최대 0.08 s, 1.5 N 위 최대 0.025 s. 걷는 속도 0.211–0.221 m/s(빈 방 0.222). `handoff` 때 들린 높이 0.410–0.451 m(오리가 기둥보다 6 cm쯤 더 감; 0.45는 `travel` 한계), 뒤에는 0.381 m. `handoff` 순간 문이 약 5 cm 떨어져 걸리며 3.5–4.5 N이 0.03 s(끊어짐 기준 0.2 s 아래). 기본 `max_tension` 0.9 N으로도 3판에서 끊어지지 않았지만(0.9 N 위 최대 0.03 s) 95%값이 0.89 N이라 여유가 없어 1.5 N으로 둠. 들린 문 밑 오리 꼭대기 여유 9.7 cm(시드 3개, 판 접촉 0).
       - `handoff` 없이 기둥까지 갔다 돌아오기: 해답은 문 앞 0.84–0.88 m에서 문이 0.27–0.30 m(< 0.34)로 내려와 선다(live 계획이 막음).
     - 시험: `tests/test_rope.py` 44개(빠름; 도르래 4, 끈 문 2, `handoff` 3, NPC 활동 영역 2 더함). `tests/test_tow_puzzles.py`: NAMES에 두 프리셋(목록·정적), slow 음성 대조(`cleat_hold`·`npc_leash` `[goto goal]`, `npc_leash` 몰기, `cleat_hold` `handoff` 없이, 똑바로 걷기; 각 시드 3개). 프리셋 20시드 점검은 `test_examples.py` 방 프리셋 시험. 가이드 표(ko/en/ja) 갱신.
     - 계획·spec과 다른 점: (1)·(2)는 사용자 "승인"(2026-11-13), spec R31에 씀(647c3e9). 나머지는 기록만. (1) `npc_leash`는 버튼이 아니라 `trigger: npc` 구역으로 끈다(2a의 구역 선택과 같은 이유로 맞춤; 버튼으로는 재지 않음). 구역이 NPC 쪽 끝을 풀어(`detach_b`) 활동 영역 밖에서 풀린 NPC가 돌아가는 것을 방 안에서 보인다. (2) 새 계획 규칙 둘: 끈 문은 끈이 묶일 수 있으면 정적으로 열리는 문, 살아 있는 계획은 다 들렸을 때만 통과(`passable_state`); 끈이 NPC에 묶일 수 있으면 NPC 센서가 스위치. (3) 도르래 끈의 몸 워프는 묶음 쪽 끝을 풀고 끈을 남긴다(R14 "고정 끝 끈은 워프 전에 푼다"를 도르래에 적용한 해석). (4) `hitch_radius`·`clip_radius` 점검이 도르래 사이 경로만큼 엄해짐. (5) NPC는 다른 끝이 풀려도 끈 끝이 걸려 있으면 계속 "묶임"(1부 그대로; 풀린 끈을 매단 NPC는 움직이지 않음). (6) 커밋을 나눠 중간 커밋(끈 문~프리셋)에서는 `test_guide`(가이드 표 갱신 전)와 `test_boundary`(끈 문 docstring의 몸 이름)가 실패하고, 문서 커밋에서 맞춰짐.
     - 리뷰 고침 (2026-11-13): (가) 끈 문 정적 점검이 묶일 수 있는 끈만 보고 도르래를 안 봐서, 문 위 도르래 없는 끈으로도 통과했다. 이제 묶일 수 있는 끈의 `via` 점 하나가 판 위에 있어야 열 수 있는 문(`RopeDoor._over`): 점의 xy가 판 바닥 자리(축 방향 `width`/2 + 0.01, 가로 `thickness`/2, 판 가운데 기준) 안이고 z가 열렸을 때 걸이 높이(`height` + 0.003 + `clearance`, 기본 0.593 m)보다 높다. 아니면 `parts` 단계가 그 규칙을 적은 메시지로 실패. 시험: 경계 7가지(`test_rope_door_needs_a_pulley_over_its_footprint`), `cleat_hold`에서 `via`를 빼면 정적 실패(`test_cleat_hold_without_its_pulley_fails_static`), 원래 `cleat_hold`는 정적 통과. (나) "is not a hitch"·걸이 개수 오류 메시지에 끈 문을 넣음. 받지 않음: NPC 센서 규칙이 닿을 수 있는지 안 본다는 지적 — 승인된 spec 문구와 기존 잡기·몰기 규칙과 같은 방식이고, 닿지 못하는 경우는 동적 점검(20시드)이 잡는다.
     - 알려진 한계: `cleat_hold`의 `handoff` 순간 문이 약 5 cm 내려앉으며 짧은 튐(최대 4.5 N, 0.03 s)이 난다(기둥이 오리가 서는 자리보다 문 쪽에 있어서; 반대면 경로가 길어 `attach_failed`). 끈 문은 끈으로만 연다(신호 `in` 없음). 도르래는 고정점이라 끈이 벽과 부딪히지 않는 것은 그대로. `clearance`는 Microduck 값이라 다른 몸은 값을 줘야 한다.
     - 2c 다음: `overload`(큐브 개수와 `max_tension` 사이 폭을 20시드로), `pit_bridge`와 사용자 승인(2026-11-13, "승인")한 R16 변경을 2c 시작 때 spec.md에 쓴다: 구덩이에 바닥판이 있는 선택값 `depth`; 그 위에 놓인 상자(기울기 5° 이하, 윗면이 바닥 높이 ±1 cm)는 그 칸을 낙하 높이와 계획(`hole_now`)에서 바닥으로 만들고, 상자가 떠나면 다시 구멍; 구덩이 입구는 상자보다 0–4 cm 넓게(정적 점검).
   - 구현 기록 2c부 (2026-11-13, 브랜치 `feat/m2-tow-4`, `feat/m2-tow-3` 위; 커밋 ab6f648(구덩이 `depth`)·269481a(`pit_bridge`)·ac54041(`overload`)·문서 커밋; 범위: R16 구덩이 `depth`(spec b478b29), 프리셋 `pit_bridge`·`overload`):
     - 구덩이 `depth`(`parts/trap/pit.py`, 0.06–0.45 m): 그 깊이에 바닥판(마찰 1.0, 접촉 우선순위 3: 판 위 상자가 미끄러지지 않게)과 입구 바로 밖 옆벽 4개(떨어진 것이 바닥 밑으로 빠지지 않게). 어두운 판은 `depth` 없을 때만. 채움은 저장하지 않고 물리 상태에서 매번 읽는다(`filler_top`: 보관 중이 아닌 큐브, 가운데가 입구 안, 기울기 5° 이하, 윗면이 바닥 ±1 cm·밑면이 판 ±1 cm) → `height_at`(바닥 높이 = 큐브 윗면), 빠짐 판정, `hole_now`. 저장본·같은 시드는 따로 할 일 없음. 정적 `hole_mask`: 맞는(`fits`) `hitch: true` 큐브가 있고 끈이 그것에 묶일 수 있으면(`a`/`b` 이름이나 빈 끝) 구멍이 아님(`fillable`). `fits`: 높이 ±1 cm, 입구 각 변 − 상자의 가장 큰 가로 크기(손잡이 포함, 새 `Cube.extent`) ∈ [0, 0.04 m]. `rope.pit_fill_problems`가 `fits` 문구로 실패.
     - 판정 기준(`criterion.txt`, 20시드 전에 적고 날짜 붙은 덧붙임만 3개): 0줄 게이트 = 구덩이 폭 그대로의 복도에서 열린 구덩이를 걸어 건너기(하나라도 성공하면 `pit_bridge`를 만들지 않음). 입구 0.33·0.37 m, `depth` 없음·0.3 모두 0/20(몸통이 x −0.054를 못 넘음). 덧붙임: 최종 입구 0.31 × 0.345 m(`depth` 0.3)도 0/20.
     - `pit_bridge` 시도한 판(미리 보기): 1판(오리가 구덩이를 돌아감, 입구 0.34) 0/3: 끌기가 일찍 끝나고 상자가 45°로 끼고 풀 때 끊어짐. 2판(북쪽 출발, 곧게 끌기, 0.34) 0/3, 45°로 낌. 3판(0.37 정사각, ±x 손잡이) 평평히 들어앉지만 끈 끊어짐(`on_snap: fail`)이나 0.42 m 틈이 격자 정렬로 막혀 골에 못 감. 4판(틈 0.44, `max_tension` 100) 서쪽 5 cm 틈에 발을 디뎌 빠짐, 상자가 판 위에서 미끄러짐(마찰 0.1) → 판에 우선순위 3. 5·6판(패드 서쪽, 틈 0.48) 그래도 빠짐, 미끄러운 상자 윗면(마찰 0.1)에서 오리가 미끄러짐. 7판(상자 0.08 kg, 마찰 0.5) 남쪽 7 cm 틈에 빠짐. 8판(상자가 멈추는 y에 틈 가운데) 3/6. 입구 y 0.34는 2/4에서 낌, 0.35·0.36은 들어앉음. 9–11판: 옆 처짐으로 틈에 빠짐(10판은 패드가 끈 길이 밖). 12판(벽이 이끄는 상자, ±y 손잡이) 손잡이 위에서 흔들려 1/8. 13판(−x 5 mm 손잡이 하나, 0.31 × 0.345) 8/8. 14판 = 13판 + `on_snap: fail`·출발 자리 고침(끈 정적 점검 1.13 > 1.1 m) → 프리셋.
     - `pit_bridge` 20시드(프리셋 파일): 정적 통과·경고 없음, `mpg check` 20/20(평균 26.3 s), 안정 통과. 상자 들어앉음 3.2–3.5 s(20/20), 기울기 0.0°, 윗면 −0.01 cm, 장력 최대 1.48–3.09 N, 끊어짐·빠짐 0. 음성 대조 모두 0/20: 똑바로 걷기, `drag` 빼기, `untow` 빼기(몸 x 0.91–0.92에서 붙잡힘, 최대 3.46–5.23 N, 안 끊어짐), 덧붙임의 모서리 질러가기 북·남.
     - `overload` 끈 여유 미리 재기: 걸음마다 장력이 0 가까이 떨어져, 끊는 것은 평균이 아니라 0.2 s(`snap_time`) 동안 버틴 가장 큰 장력 M이다(덧붙임 2). 기본 0.9 N 끈은 큐브 3개로도 안 끊어짐 → 프리셋이 `max_tension`을 빈 수레 M과 짐 M 사이로 정함. 수레 마찰 1.0은 넘어짐, 걸이 아닌 짐 큐브는 마찰 0.5 수레에서 미끄러져 떨어짐(우선순위: 걸이 수레 2가 이김). 5시드: 수레 0.04 kg·마찰 0.5에서 M 0/1/2/3개 = 0.10–0.16/0.28–0.34/0.42–0.49/0.59–0.64 N. 수레 0.06 kg·마찰 0.4(짐 0.24 N)로 정함. 열린 바닥 20시드(큐브 `hitch: true`): M 0.11–0.21/0.27–0.34/0.42–0.54/0.55–0.67 N, 0.3 N 끈에서 끊어짐 0/7/20/20, 배달 20/13/0/0.
     - `overload` 시도한 판(시드 20개 달리 적지 않으면): v1(서쪽 출발, 짐 +x 쪽) 0/2: 오리가 돌아가며 수레를 밀어냄. 이때 `on:` 필드가 YAML 1.1에서 불(True) 키로 읽혀 조용히 빠짐 → 필드 이름 `rest_on`. v2(동쪽 출발, 버림 구역 북쪽) 1/3: 버림 구역 안에서 걸어 나오는 광선이 구역 상자(group 1)에 막혀 제자리를 돔 → `carry.py` 광선 고침 뒤 17/20(실패: 집은 큐브를 돌다 놓침, 수레가 구역을 비껴 지나침, 연결 패드가 걸이 큐브 c1에 묶음). v3(버림 구역 남쪽) 1/20(돌아오다 c2를 수레에서 쳐 떨어뜨림, 시드 7 NaN 한 번). v4(패드 남쪽, 구역 0.5 m) 15/20(끌기 중 수레가 버림 구역 쪽으로 끌려 끊어짐, 구역 뒤 문에서 끊어져 `on_snap: fail`로 실패). v5(짐 큐브 걸이 끔, `rest_on` 큐브 우선순위 2) 17/20. v6(`on_snap` 기본, 구역 x 0) 17/20. v7(구역 0.5 × 0.8) 18/20. v8(구역 0.5 × 1.4, 방 폭) 19/20, 공식 세트: `mpg check` 19/20(68.9 s), 똑바로 걷기 0/20, 그러나 짐 안 내리고 끌기 12/20(끊어짐 18): 추적하니 끌기 단계에서 오리가 수레 서쪽을 돌아 끈이 느슨한 채(장력 0 N) 수레를 밀어 넣음(실은 수레 미는 힘 0.63 N) → 내보내지 않음, 덧붙임 3(밀 수 없는 칸, 끌기인지 확인). v9(서쪽 벽 칸 폭 0.26, 패드 (−2.15, 0.25)) 1/5, 짐 안 내리고 0/5: 빈 수레가 칸에서 비스듬히 끌려 나오며 끊어짐. 칸 모양 재기(안 끊어지는 끈, 시드 4, 끌기 M): 폭 0.26 0.00–0.93 N, 폭 0.34·칸 벽이 수레 가운데까지 0.11–0.15 N, 폭 0.34·벽 x −2.30·패드 (−2.05, 0.2) 0.14–0.16 N. v10(마지막 칸) 18/20, 짐 안 내리고 0/20: 실패 2는 c1을 들고 패드를 천천히 지나다 실린 수레에 묶임. v11(패드 칸 입구 남쪽 (−2.05, −0.2)) 19/20(평균 49.8 s), 짐 안 내리고 0/20 → 프리셋.
     - `overload` 20시드(프리셋 파일, v11과 같은 내용): 정적 통과·경고 없음, `mpg check` 19/20(평균 49.79 s; 실패 시드 17은 `drop c1 dump` 단계에서 멈춤), 안정 통과. 음성 대조: 똑바로 걷기 0/20, 짐 안 내리고 끌기(`demo: [tow rope cart, drag rope zone, goto goal]`) 0/20(끊어짐 20). 끌기인지(v11 해답 성공 19개): 끌기 시간 중 장력 > 0.02 N 비율 0.42–0.97, 수레 가운데가 몸통 뒤인 제어 스텝 0.58–1.00. 끈 여유(끌기 구간만: 남길 큐브만 두고 연결 패드에서 구역 쪽을 보고 시작, 패드·버림 구역 뺌, 끈 a: body·b: cart 리셋 때 묶음; 평균·95%값·M은 안 끊어지는 끈, 끊어짐·배달은 0.3 N):
       | 큐브 수 | 장력 평균 | 95%값 | M | 끊어짐 | 배달 |
       |---|---|---|---|---|---|
       | 0 | 0.33 N | 0.59 N | 0.12–0.19 N | 0/20 | 20/20 |
       | 1 | 0.52 N | 0.83 N | 0.27–0.37 N | 10/20 | 10/20 |
       | 2 | 0.70 N | 1.04 N | 0.43–0.51 N | 20/20 | 0/20 |
     - 시험: `tests/test_traps.py` 구덩이 `depth` 빠른 시험(평평한 상자면 바닥, 떠나면 다시 구멍, 짧은 상자는 못 채움, 저장본, `depth` 없으면 그대로, 정적 순서 채울 수 있음 5경우, 입구 맞춤 4경우). `tests/test_tow_puzzles.py`: 깊은 구덩이 입구 정적 점검, NAMES에 두 프리셋(`overload`는 task로), slow 음성 대조 `pit_bridge` `drag` 빼기·`untow` 빼기, `overload` 짐 안 내리고 끌기, 똑바로 걷기. `tests/test_cube.py` `rest_on` 4개(윗면에서 시작·밑 큐브 무작위를 따라감·쉼, 우선순위, 거절 4경우, 얹힌 큐브 위에 얹기 거절). `tests/test_grab.py` 구역 상자 안에서 걸어 나오는 광선이 막히지 않음(고치기 전 광선으로는 막힘을 확인). `tests/test_examples.py` 방 프리셋 20시드 점검에 `overload` task.
       - 시험 수: 빠른 966 passed·1 skipped(132 deselected), slow 113 passed(`beak`·`grab`·`gaze`·`examples`·`check2`·`traps`·`addon`·`roller`·`rope`·`tow`·`npc`·`devices`·`logic`, 11분).
     - 계획·spec과 다른 점(보고만, spec.md는 고치지 않음): (1) 입구 맞춤은 축마다가 아니라 상자의 가장 큰 가로 크기(손잡이 포함)로 두 변 모두 잰다(상자가 돌아 들어가도 맞게). (2) 새 정적 규칙: `depth` 구덩이는 맞는 묶일 수 있는 걸이 큐브가 있으면 정적 순서에서 구멍이 아님(spec은 `hole_now`만 말함). (3) 바닥판 마찰 1.0·우선순위 3, 옆벽. (4) `pit_bridge` 끈 `max_tension` 5 N·`on_snap: fail`, 벽이 이끄는 상자, 5 mm 손잡이 하나(가로 0.305 m), 입구 0.31 × 0.345 m(끄는 방향 0.345: 0.34는 45°로 낌). (5) 큐브 새 필드 `rest_on`(이름 `on`은 YAML 불 키라 못 씀)과 그 상자의 접촉 우선순위 2. (6) 해답 고침: 걸어 나오기 광선이 group 1(구역 상자)을 보지 않음. (7) `overload` 끈 `max_tension` 0.3 N(기본 0.9 N보다 낮음: 끊어짐에 0.2 s 연속이 필요해 0.9 N은 안 끊어짐), `on_snap` 기본(`fail`이면 구역 뒤 문에서 끊어져 실패). (8) 배달 구역은 방 폭(0.5 × 1.4 m; 끌기의 옆 처짐), 버림 구역은 `drop` 목표일 뿐 출력은 안 씀. (9) 수레는 막힌 칸 안(끌기만 됨): v8에서 밀어 넣기 우회를 찾아서. (10) `overload`는 task(집기 행동, 120 s)가 필요. (11) 중간 커밋 269481a·ac54041은 가이드 갱신 전이라 `test_guide`가 실패하고 문서 커밋에서 맞춰짐.
     - 리뷰 고침 (2026-11-13, 커밋 67872ee): `filler_top`이 입구를 얼마나 덮는지 보지 않아, 가운데가 입구 안이고 높이가 `depth`와 맞는 큐브면 아무리 작아도 입구 전체가 바닥으로 읽혔다(`fits`는 `fillable`과 `demo:` 정적 점검만 걸렀다). 재현: 구덩이 `size: [0.62, 0.31]`, `depth: 0.3`에 0.3 m 큐브를 서쪽 반에 두면 `filler_top()` ≈ 0, `hole_now` None, 열린 동쪽 반의 `floor_height`가 높이를 돌려줘 살아 있는 계획이 그 위를 지나고 오리가 판으로 떨어진다. 게다가 `floor_height`가 None이 아니라 `env._fall_resets`가 False가 되어 `on_fall: reset` 구덩이도 되살리지 않고 실패로 끝나며, `Pit.update`가 `fell`을 세우지 않아 `failed()`·`["pit"]`도 안 나온다. 고침: 채울 후보를 `add_room_parts`에서 거른다(`p.cubes = [c for c in cubes if p.fits(c.extent()) is None]`) — `filler_top`에서 매 스텝 `fits`를 다시 재지 않고 "후보는 맞는 큐브"라는 불변식이 한 곳에 남아 더 깔끔하다. `fillable`은 그 목록에서 묶일 수 있는 걸이 큐브를 찾는다. 맞는 큐브가 없는 구덩이는 구멍으로 남는다(낙하 규칙, `hole_now`, `fell`/`failed()` 모두 열린 구덩이와 같음). 시험 2개(`test_a_wide_deep_pit_is_not_filled_by_a_box_that_does_not_fit`: 후보 없음·`filler_top` None·`hole_now` 있음·열린 쪽과 상자 쪽 `floor_height` None; `test_a_wide_deep_pit_with_on_fall_reset_still_resets`: 되살아남, 실패 아님) — 둘 다 고침 전 코드에서는 실패한다. 내보낸 프리셋은 그대로다: `pit_bridge`의 `pit.cubes == ['box']`·`fillable` True, 두 프리셋 `mpg check --static` 통과, 구덩이 행동이 안 바뀌므로 20시드 동적 재실행은 필요 없다.
     - 받지 않음: `overload` 19/20이 `need=18`에 한 시드 여유뿐이라는 지적 — 승인된 판정 기준(≥ 19/20)을 지킨다. 아래 알려진 한계에 적는다.
     - spec R31 `overload` 문구 제안(사용자 승인 필요): "`overload`: 부리로 못 드는 수레에 짐 큐브(`rest_on`)가 얹혀 있다. 끈의 `max_tension`은 끌기 구간에서 잰, 0.2 s 동안 버틴 가장 큰 장력(M)의 빈 수레 값과 짐을 다 실은 값 사이에 둔다. 시드 20개로 다 실으면 배달 0/20(끈이 끊어짐), 짐을 내리면 20/20·끊어짐 0, 빈 수레의 짐(마찰 × 무게)은 0.4 N 이하. 수레는 목표까지 밀어 옮길 수 없게 둔다(몸보다 좁은 막힌 칸 등): 배달은 끌기여야 한다. 잰 값: 빈 수레 M ≤ 0.19 N, 끈 0.3 N, 큐브 2개 M ≥ 0.43 N."
     - 알려진 한계: `overload` 여유가 좁다(위 표; 큐브 1개만 내리면 10/20 끊어짐). `mpg check` 19/20은 기본 `need`(20시드의 18)에 한 시드 여유뿐이다(리뷰 지적, 기준은 지킴). 해답은 부리가 큐브를 놓쳐도 `drop` 단계를 놓은 것으로 끝낸다(시드 3·v2에서 본 실패 꼴). 배달 뒤 묶인 채 문을 지나며 끊어지는 일(실패 아님). 다른 견인 프리셋의 밀어 옮기기는 아래 후속 점검에서 쟀다.
     - 후속 점검: 밀어 옮기기 우회 (2026-11-13, 브랜치 `feat/m2-tow-5`, `feat/m2-tow-4` 위): v8에서 본 우회(끈을 느슨하게 두고 짐을 밀어 넣기)가 2a 프리셋에도 있는지 쟀다. 기준은 재기 전에 적었다(`r31e/criterion.txt`: 성공 시드마다 `drag` 구간의 장력 > 0.02 N 비율 ≥ 0.30, 짐 가운데가 몸통 뒤인 비율 ≥ 0.50). 20시드: `sled_delivery` 20/20 팽팽 0.62–0.67·뒤 0.98–1.00, `drop_and_go` 20/20 0.61–0.68·1.00, `chain_dropoff` 20/20 rear(cart_b) 0.71–0.76·1.00 / front(cart_a) 0.88–0.94·1.00. 모두 기준 안이라 확인용 대조군은 돌리지 않았다. `tethered_start`는 배달할 짐이 없고 끈을 안 풀면 끊어져 실패하므로 점검에서 뺐다. 회귀 시험 `test_a_drag_leg_pulls_the_load_behind`(slow, 3시드, 제어 스텝마다 `tension_now`로 한 번씩 읽음; 제어 스텝 표본으로는 팽팽 0.59–0.95·뒤 1.00). 첫 `chain_dropoff` 측정은 rear 끈의 짐을 cart_a로 잡아(양 끝이 다 몸이 아님) `reset_towed`로 체인 끝을 보게 고쳐 다시 돌렸다. 한계: 데모 솔버를 잰 것이므로 학습된 정책이 밀어 옮기는 길을 못 찾는다는 증명은 아니다.
16e. 주인과 동행(R32): `zone.attach_to` → `owner.py`(경로, 기다리기, 명령, `lost`) → 저장본 → `follow` 하위 목표 → 프리셋 8종 → 예제 `walkies.yaml`. `leash_walk`·`roller_walk`는 R31·R30 뒤. `stay` 동안 `near_ratio`·`lost`·`waits` 멈춤. `receptacle.attach_to`(NPC 자식 몸체). `attach_to` 부품은 상대 `pos`, `place()` 건너뜀, 위치는 `xpos`.
16f. 신호기와 파괴(R33, R34): 논리 부품 몸체(`body: true`) → `source` → `wire`(연결 끊김) → `breakable`(원인 4가지, 고치기) → `relay` → `destroyer` → 회로 그래프 정적 점검 → `break`/`repair` 하위 목표 → 프리셋 8종 → 예제 `power_cut.yaml`. 전선 `carries`/`to` 간선과 맞는 부피, 꼬리표 마커. `substep` 훅으로 탄·눌림 판정(한 접촉 한 번). 부서짐 모양은 복원 뒤 상태에서 다시 입힘. R7 순서 잠금 `.wrong`. `break ... via` 하위 목표. 움직이는 중계기 `hitch`. `crush`는 물체만. 움직이는 부품에 전선 연결 금지(정적 점검).
  - 나누기(2026-11-13, 사용자 "승인"): 0단계 측정 → 3a 논리 부품 몸체와 `source` → 3b `wire`·`breakable`·`relay`·`destroyer` → 3c 회로 그래프 점검·하위 목표·프리셋 8종·예제. PR은 3a/3b/3c 세 개다.
  - 0단계 측정: 쪼기 닿기 (2026-11-13, 브랜치 `feat/m2-circuit-3a`). 사전 기준을 쓰고 빈 방 시드 20개씩 세 가지로 쟀다(기록: 스크래치패드 `r33a/criterion.txt`, `wall20.txt`, `reach20.txt`).
    - 걸어 지나가며 쪼기는 잡기 에지가 부피에 든 적 0/20이다(닿기 자체는 20/20: 머리 숙임 부리 끝 0.127–0.130 m, 머리 듦 0.207–0.212 m).
    - 제자리 유지는 불가능하다. 위치 피드백을 줘도 목표점을 통과해 0.14 m/s로 계속 걸었다(명령 0 전진 흐름 + 뒤로 못 걸음). **빈 바닥 가운데 쪼기 대상은 쓸 수 없다.**
    - 벽으로 걸어 들어가 버티면 40/40 성립, 넘어짐 0. 머리 숙임 때 부리 끝은 벽면에서 0.029–0.048 m·높이 0.124–0.133 m, 머리 듦 때 0.001–0.012 m·0.212–0.221 m다.
    - 결정(스펙 R33·R34·OQ8에 반영): 부술 수 있는 전선 덮개는 벽이나 오리를 막는 몸체에 붙이고 돌출 0.04 m·높이 0.18 m, 쪼기 거리 0.02 m. 실제 geom에서 빡빡하면 돌출 0.06 m·높이 0.16 m. 정적 점검에 "`peck` 대상이 막히는 자리에 없음" 경고를 더한다.
    - 한계: 걷기 정책의 스크립트 접근만 쟀다. 정면 접근만 쟀고 비스듬한 접근은 재지 않았다.
  - 0단계 측정: `crush_force` (2026-11-14, 브랜치 `feat/m2-circuit-3b`, 기록: 스크래치패드 `r34crush/criterion.txt`, `crush20.txt`). 몸체 있는 `source` 상자(0.08 × 0.08 m, 높이 0.12 m)에 기본 큐브(30 g)가 준 최대 접촉력, 조건마다 시드 20개.
    - 고의: 5 cm 낙하 0.87 N(모든 시드 같음 — 충돌이 결정적), 오리가 밀어 누름 3.11–12.20 N(중앙값 7.1, 넘어짐 1/20). 참고: 20 cm 낙하 1.62 N, 오리 자신이 상자에 준 힘 0–41.7 N(`crush`로 세지 않음).
    - 우연: 위에 얹음 0.30 N(= 무게), 옆에 3 mm 띄움 0.00 N.
    - 결정(스펙 R34·OQ8에 반영): 기본값 **0.5 N**(우연 최대와 고의 최소의 기하평균, 양쪽 여유 1.7배). 얹힐 수 있는 물체 무게가 `crush_force`를 넘으면 정적 점검이 경고한다(약 51 g 이상).
    - 방법 보정 2건(기준 파일에 날짜로 기록, 첫 20시드 회차는 무효 `crush20-void.txt`): 큐브 손잡이가 ±0.035 m라 틈 3–5 mm는 초기 관통으로 1.9–2.8 N 스파이크를 냈고, 오리는 0.19 m/s라 4 s 안에 큐브에 닿지 못했다. 배치는 실제 외곽(0.035 m) 기준, 접근은 최대 12 s, 배치에 시드별 ±0.01 m 흔들기를 넣었다.
    - 한계: 기본 큐브 하나(30 g, 0.04 m)만, 정면 밀기만 쟀다. 무거운 물체·비스듬한 충돌·`crush` 반복 타격은 재지 않았다. 오리가 미는 최대 힘은 명령 세기와 거의 무관하다(0.4든 0.1이든 2.6–12.2 N).
  - 3c 구현 기록 (2026-11-14, 브랜치 `feat/m2-circuit-3c`, main `48246fa` 위): 회로 그래프 점검·`break`/`repair` 하위 목표·`repair_with`·프리셋 8종·예제 방.
    - `repair_with: <마커 번호>`(+`socket_dist` 0.06 m)는 별도 부품 없이 `Breakable` 안에 넣었다. 그 마커를 가진 자유 물체가 측정 geom에서 `socket_dist` 안에 있으면 소켓에 꽂힌 것이다(0.2 m 상자 위에는 부리가 닿는 자리가 없어 옆에 내려놓는 것을 소켓으로 본다). `has_repair()`가 `repair`·`repair_with` 둘 중 하나라도 있는지 알려 주고 회로 점검이 그것을 읽는다. `place_tolerance()`는 부서질 수 있는 부품이면 `자기 폭 + socket_dist`를 돌려주어 퓨즈 놓기와 `crush` 떨어뜨리기가 모두 성립한다.
    - 새 모듈 `circuit.py`: `break_edges`(부품 → 부서지면 읽는 값이 바뀌는 부품들; 전선은 `to`의 독자만), `affected`, `lost_channels`, `shuts_gates`, `demo_breaks`, `check_circuit`. `mpg check`에 정적 단계 `circuit`을 더했다(`order` 뒤, `actions` 앞). 실패: `demo:`가 `on_break: fail` 부품을 부수거나 아무도 안 읽는 부품을 부술 때. 경고: 고칠 수단이 없는 부품의 파괴가 해답이 지나는 문을 닫을 때(계획에 파괴를 넣어 판정). 원인별 경고 3종은 `Breakable.check_warnings`(= `parts` 단계)에 있다.
    - 계획(R6a) 확장: `logic.plan_channels(parts, on, broken)`과 `solution_order(..., broken=)`에 "부서진 것으로 볼 부품" 집합을 더하고, `check`와 `env.reset`이 `demo_breaks(room)`을 넘긴다. `Breakable.plan_broken_values`는 값 출력을 `broken_value`로, `.broken`을 1로 둔다. `Wire.plan_broken_values`는 나르는 채널 자체를 떨어뜨린다(채널당 값이 하나뿐인 모델이라, 그 채널을 전선 밖에서도 읽으면 낙관적이다 — 동적 해답이 증명한다). 부서짐은 순서에 넣지 않는다(걸어가는 자리가 아니고, `env.targets`에 들어가면 해답이 그 부품 앞에서 멈춘다).
    - 하위 목표: `room.parse_demo`에 `break <부품> via <원인>`·`repair <부품>`(원인이 `by`에 없거나 고칠 수단이 없으면 `demo` 단계에서 거절), `solver.break_act`/`repair_act`. `peck`은 덮개가 보는 쪽 0.3 m 앞으로 길을 찾아 간 뒤 기대서 환경 스텝마다 잡기를 켰다 끈다(`Wire.peck_normal`), `shell`은 조준 채널 칸마다 `hits_board`로 맞는 칸을 찾아 레버로 밀고 발사 버튼을 누른다(`Breakable.break_board` → `ShellBoard` 어댑터), `crush`는 물체를 집어 떨어뜨리고, `destroyer`는 그 부품을 때리는 파괴기의 `in`을 쓰는 스위치를 누른다. `check_actions`가 `peck`·`crush`·`repair_with`에 `grab`을, 부리를 숙여야 하는 `peck`에 `head`를 요구한다.
    - 해답에서 고친 것 2가지(둘 다 측정으로 드러남): (1) 부수거나 고친 뒤 부품에서 0.35 m 물러나지 않으면(최대 3초) 몸이 막힌 칸에 서 있어 다음 하위 목표가 영원히 멈춘다(`fuse_repair` 0/3 → 3/3). (2) 조준 레버가 문 뒤에 있으면 `turn_act`의 직선 걷기가 벽에 낀다. 레버까지 0.7 m는 격자 계획으로 간다(`power_cut` 2/3 → 4/4).
    - 쪼기 닿기 재측정: 걸어와 기댄 오리의 부리는 벽 덮개에서 1.4–2.2 cm, 바닥에 선 상자에서 4.4 cm 떨어져 멈춘다. 3a가 쟀던 0.02 m는 부리를 대 준 값이라, 프리셋은 `peck_dist`를 0.04 m(`logic/wire_cut` 프리셋 자체)와 0.06 m(상자를 쪼는 두 방)으로 둔다.
    - 프리셋 `puzzles/circuit/` 8종과 `examples/rooms/power_cut.yaml`(task `examples/task_power_cut.yaml`, 120초): 시드 20개 `mpg check`로 8종 모두 20/20, `power_cut` 20/20(평균 50.5초). `power_cut`의 시계 문은 지나는 중에 닫히면 오리가 문간에서 꺾다 넘어진다: 주기 5초·열림 0.6에서 17/20, 틈을 넓히면 13/20(문짝에 더 자주 걸림), 주기 10초·열림 0.6에서 20/20.
    - 시험: `tests/test_circuit.py`를 새로 만들고(그래프 3개, `circuit` 단계 4개, 하위 목표 파싱 8개, 프리셋 8개 정적 + 8종 존재, slow 2개로 해답이 실제로 부수는지), `tests/test_breakable.py`에 `repair_with` 4개와 원인 경고 3개를 더했다. `tests/test_check2.py`·`test_examples.py`·`test_presets.py`에 새 방·프리셋·task를 등록했다.
    - 계획·spec과 다른 점(보고만): (1) `press_relay`는 중계기를 프레스 아래로 밀거나 끌어다 놓는 대신 프레스 아래 선 중계기를 버튼으로 부순다. 평범한 물체를 부품으로 밀어 넣는 하위 목표가 없고(`push`는 공 전용, `drag`는 끈 전용), 움직이는 중계기는 방 작성자의 선택으로 남겼다. (2) `crush`를 쓰는 프리셋이 없다(스펙이 적은 8종 중 어느 것도 `crush`가 아니다). 원인은 시험으로만 확인한다. (3) `jam_the_turret`은 대포 대신 옆바람을 쓴다(스펙이 "옆바람이나 대포"로 허용). (4) 중계기와 신호기는 마커를 달 수 없어(`marked = False`) 프리셋에서는 전선에만 마커를 붙였다. (5) `fuse_repair`는 "부서진 중계기"를 시작부터 부서지게 하는 필드가 없어, 충전 1회 프레스와 상시 신호기로 첫 몸 스텝에 부순다.
    - 알려진 한계: 파괴기가 논리 부품을 때리면 그 부품의 다음 갱신(한 몸 스텝 뒤)에 반영된다(3b에서 이어짐). `Wire.plan_broken_values`의 낙관(위). `break ... via shell`은 조준 채널에 칸(`channel_cells`)이 있어야 풀 수 있다(연속 조준은 못 푼다). `hp`가 2 이상인 `shell`·`destroyer` 파괴는 해답이 버튼을 다시 누르러 걸어 나갔다 와야 해서 프리셋에서는 쓰지 않았다.
16g. 에피소드 무작위화(R35): (5b에서 넘어온 일: 코인 위치 샘플러.) 샘플러 5종(`range`, `region`, `candidates`, `shuffle`, `rooms`) → 층별 세기(task.yaml) → 다시 뽑기와 `mpg check` 비율 점검 → 연결 단서 자동 표시 → 결정성·기록·저장본 → `rooms`(stages 구역 재사용, 관측 마스크) → `mpg eval --randomize` 3종 → 예제 `shuffle_switches.yaml`과 비교 학습 기록. M1 `random:`은 `range`의 줄임으로 그대로 읽는다. 다시 뽑기 점검은 그래프 목표 순서 + 뽑힌 값이 입력인 정적 점검(탄도, 끈 길이, 질량). `holdout` 후보는 학습 모델에 미리 구역/선반으로 만들고 학습 뽑기에서 뺌. 순서 추론 시험: `rewards.py`에 `progress_free` → Open question 12로 기준 확정 → `shuffle_switches` 단서 켬·끔 두 정책 학습 → `wide` 비교를 학습 기록에. (7단계에서 넘어온 일: 저장본이 없을 때 쓰는 YAML 대체 `checkpoint_states:`.)
17. 예제 방(큐브·소켓, 벽 버튼, 빨래 `laundry.yaml`). 가이드 완성, AGENTS.md와 SKILL.md 연결.
   - 나누기 (2026-11-12, 사용자 결정 "추천대로 가자"): (a) 예제 방 3개는 지금 한다. (b) 가이드 완성, AGENTS.md와 SKILL.md 연결은 16g 뒤로 옮긴다.
   - (a) 구현 기록 (2026-11-12, 브랜치 `feat/m2-examples-beak`, `feat/m2-basket-exit` 위): 예제 `examples/rooms/cube_socket.yaml`·`wall_button.yaml`·`laundry.yaml`, task `examples/task_cube_socket.yaml`(`[move, head, grab]`)·`task_laundry.yaml`(`[move, head, grab]`, 60 s 판). 세 방 모두 표준 문틈(벽 끝 0.56 m 사이, 실제 틈 0.50 m, `device/sliding_door`)이고 격자 해답으로 풀며 `demo:`는 없다. 시험: `tests/test_examples.py`의 KINDS와 task 표, slow `test_demo_video_of_the_scripted_solution`(빨래·벽 버튼 방의 해답을 MP4로 그림, 성공·10 kB 이상), `tests/test_check2.py`의 예제 방 정적·20시드 점검에 새 task. 가이드 3종 예제 목록과 parts-guide 3종 조립 예 표.
   - 잰 값 (`mpg check`와 같은 해답 판, 따로 적지 않으면 시드 100개):
     - `wall_button.yaml`: `beak/wall_button_beak_mid`(판 가운데 0.2 m) 북쪽 벽 `[-0.3, 0.699]`, 기본 `actions: [move]`. 100/100(평균 15.6 s), `mpg check` 20/20. 같은 방에 `wall_button_beak_low`(0.15 m, `[move, head]`)를 두면 85–88%다: `[-0.3, 0.699]` 35/40, `[0.1, 0.699]` 37/40·85/100, `[-0.6, 0.699]` 35/40, `[0.0, 0.699]` 35/40, `[-0.1, 0.699]` 34/40, 칸막이 서쪽 면 `[0.469, 0.45]` 31/40, `[0.469, -0.45]` 16/40, 서쪽 벽 `[-1.199, 0.2]` 30/40, 칸막이 옆 `[0.25, 0.699]` 0/40. 실패는 모두 누른 뒤다: 머리를 숙인 채(0.26–0.29) 벽에서 물러나기(옆걸음 + 시계 돌기)를 하다 몸이 벽에 밀려 반시계로 160°쯤 돌고 멈춘다(시드 3, 12, 22). `mid`는 같은 자리 100/100, 칸막이 면 100/100, `[0.1, 0.699]` 99/100이라 예제는 `mid`로 했다.
     - `cube_socket.yaml`: 열쇠(마커 7)를 받침대(x -0.5..-0.3, y 0.25..0.35, 0.18 m) `[-0.48, 0.3]`에, 소켓(`accept: [7]`, `angle_tolerance: 45`)을 `[0.1, 0.0]`에. 93/100(평균 20.8 s), `mpg check` 19/20(시드 4). 소켓 기본 15°면 90/100. 다른 자리(45°): 받침대 x -0.7..-0.5에서 소켓 `[0.0, 0.05]` 94/100(시드 2, 11 실패라 `mpg check` 18/20), `[0.0, -0.25]` 93/100(시드 2, 4, 11 → 17/20), `[-0.1, -0.1]` 36/40, `[0.1, -0.15]` 33/40, 받침대 동쪽 `[-0.2, 0.3]` 34/40·`[0.15, 0.3]` 33/40, `[-0.3, 0.2]` 17/40; 받침대 x -0.5..-0.3에서 `[0.0, -0.25]` 91/100, `[0.2, -0.25]` 28/40. 열쇠 대신 `cube_weight`(소켓 15°, 주기 90°)는 29/100. 실패: 소켓을 지나쳐 시계로 돌아오는 동안 열쇠를 놓쳐 바닥에 떨어지거나(부리가 닿지 않음) 소켓 둘레에서 멈춘다. 60 s 판으로 늘려도 94 → 94.
     - `laundry.yaml`: 받침대 두 개(북쪽, x -0.8..-0.6과 -0.2..0.0), 빨래 `[-0.74, 0.3]`·`[-0.14, 0.3]`(받침대 서쪽 끝에 맞춤), 바구니 `[0.2, -0.3]`(`count: 2`, 남동쪽, 문 가는 길 밖). 40 s 판 79/100, 60 s 96/100(가운데값 31 s, 다시 하는 판은 60 s 가까이), 90 s 98/100. `mpg check`(task 60 s) 18/20(평균 33.6 s, 시드 1·10: 첫 빨래를 나르다 놓쳐 바닥에 떨어짐). 둘레 자리(60 s): 바구니 `[0.15, -0.3]` 91, `[0.25, -0.3]` 87, `[0.2, -0.25]` 84, `[0.1, -0.3]` 88, 빨래를 받침대 안쪽으로 4 cm 옮기면 87. 받침대 짝·바구니 자리 24가지(시드 20개, 60 s): 바구니가 남쪽 벽 가까이(y -0.3)면 13–19/20, y -0.25면 6–16/20, 북쪽(y 0.3)이면 0–15/20. 처음 배치(받침대 x -0.9..-0.7·-0.3..-0.1, 40 s)는 0/40: 첫 빨래를 넣은 뒤 몸이 바구니 쪽을 보고 서 있고 다음 빨래가 왼쪽 뒤라 제자리 반시계 돌기가 필요해 멈췄다. 받침대를 서쪽·남쪽에 두고 옆에서 다가가면 0/40(물고 제자리 시계 돌기가 받침대에 걸림).
     - 해답 영상: slow 시험이 시드 0 해답을 640×480, 10 fps MP4로 쓴다(빨래 약 166 kB, 30 s; 벽 버튼 약 76 kB). 저장소에는 넣지 않는다(커밋된 MP4가 없고 `runs/`는 무시 목록).
   - 계획·spec과 다른 점:
     - 빨래는 바닥이 아니라 0.18 m 받침대 위에 있다(spec R28 "바닥의 빨래 2장"). 머리를 가장 숙여도 부리 끝이 0.135 m라 바닥 물체는 잡히지 않는다(Open question 8 미정, `cube_plate`와 같은 받침대).
     - 벽 버튼 예제는 `wall_button_beak_low`가 아니라 `mid`다(위 잰 값). 머리 숙여 누르기(R19)는 `tests/test_beak_presets.py`의 낮은 벽 버튼 시험이 본다.
     - `laundry` task는 60 s 판이다(`four_flags`처럼). 40 s로는 두 번 나르기에 79/100.
     - R24의 `mpg demo <방>`(스크립트 해답 영상 명령)은 아직 없다. 영상은 시험의 그리기 도우미(`render_solution`)로 만든다.
   - 알려진 한계: `laundry.yaml`과 `cube_socket.yaml`의 해답은 시드 100개에서 90% 안팎이라 `mpg check` 18/20·19/20의 여유가 적다(둘레 자리 84–94%). 실패는 해답·몸의 한계(제자리 반시계 돌기, 나르다 놓친 물체가 바닥에 떨어지면 다시 못 잡음)이고 배치로는 더 줄이지 못했다. 낮은 벽 버튼은 누른 뒤 벽에서 물러나기가 85–88%다.
18. 처음 보는 작업자 시험(R25).

## Risks
- 플랫폼·스프링·워프 위의 몸은 걷기 정책의 학습 분포 밖이라 넘어질 수 있다. 측정한 뒤 프리셋 기본값을 낮추고, 가이드에 안전 범위를 적는다.
- 머리 명령 범위가 좁거나, 머리를 숙인 채 걸으면 넘어진다면 벽 버튼과 잡기가 막힌다. 대안은 벽 버튼의 높이를 낮춰 가슴으로 누르는 프리셋이다. 이 경우 spec R4·R18·R19의 변경을 사용자에게 다시 묻는다.
- 바닥 타일이 걷기를 망칠 수 있다(타일 사이 틈과 접촉). 타일 사이에 틈이 없도록 붙이고, R10 시험으로 확인한다.
- 움직이는 타일(기울기, 높이)은 경첩·슬라이드 관절이 생겨 스텝이 느려진다. 움직이지 않는 타일은 관절 없이 고정 몸체로 만든다.
- 신호 평가와 부품 수 증가로 속도가 떨어질 수 있다. 평가는 매 스텝 파이썬 루프라 부품 50개 이하를 가정한다. bench로 확인한다.
- 체크포인트 저장본 복원 후 걷기 정책의 이력이 어긋나면 첫 몇 스텝이 흔들린다. R6 테스트에 복원 직후 3초 동안 넘어지지 않는지를 넣는다.
- PR E3는 부리 기믹이 늘어 E1만큼 커졌다. 16b를 별도 PR(E4 `feat/m2-beak`)로 나눌 수 있다.
- `max_lift_mass` 기본값 0.05 kg은 실측 전 값이다. 실물 시연 영상의 물체 무게를 모르므로, 사용자가 실물 값을 알게 되면 바꾼다.
- PR E1만으로도 크다. 1–8단계와 9단계(학습)를 나눠 9단계를 별도 PR로 낼 수 있다.

## Proof
| R | 증거 |
|---|---|
| R1 | `tests/test_signals.py` |
| R2 | M1 테스트 전체 통과, `mpg check examples/switch_door.yaml --json` |
| R2b | `tests/test_body_caps.py` |
| R3 | `tests/test_presets.py`, `mpg parts --json` |
| R4 | `tests/test_buttons.py` (벽형은 E3) |
| R5 | `tests/test_sensors.py` |
| R6 | `tests/test_checkpoint.py`; `mjSTATE_INTEGRATION`으로 잡기 용접·끈 연결이 복원 뒤 유지됨 |
| R7 | `tests/test_logic.py` |
| R8 | `tests/test_logic.py::test_transceiver_delay` |
| R8b | `tests/test_counter.py` |
| R9 | `tests/test_doors.py` |
| R10 | 타일 바닥 방의 걷기 시험(slow), `tests/test_floor.py` |
| R11 | `tests/test_platform.py` |
| R12 | `tests/test_devices.py` |
| R13 | `tests/test_setter.py` |
| R14 | `tests/test_warp.py` |
| R15 | `tests/test_spring.py` |
| R16 | `tests/test_traps.py` |
| R17 | `tests/test_cube.py` |
| R18 | `tests/test_grab.py` |
| R29 | `tests/test_npc.py`, `herd.yaml` check |
| R30 | `tests/test_addon.py`, `roller_rink.yaml` check, OQ7 비교 기록 |
| R31 | `tests/test_rope.py`(재부착 위치, `reel_speed`, 워프·회수·`set_addon`, 매듭 트리거 제외 포함), `puzzles/tow/` 정적 check, `tow_yard.yaml` check 18/20(slow), OQ9 측정 기록 |
| R32 | `tests/test_owner.py`, `puzzles/pet/` 정적 check, `walkies.yaml` check 18/20(slow) |
| R33 | `tests/test_source.py` |
| R34 | `tests/test_breakable.py`, `puzzles/circuit/` 정적 check, `power_cut.yaml` check 18/20(slow) |
| R35 | `tests/test_randomize.py`, `mpg check` 다시 뽑기 비율, `mpg eval --randomize nominal/train/wide`, `shuffle_switches.yaml` 비교 기록(slow) |
| R36 | `tests/test_coin.py`, `mpg check examples/rooms/coin_room.yaml --json` |
| R28 | `tests/test_beak_presets.py`, `laundry.yaml` 데모 영상 |
| R19 | `tests/test_actions.py`, 벽 버튼 데모 영상 |
| R20 | `tests/test_check2.py` |
| R21 | `tests/test_curriculum.py`, `runs/<id>/curriculum.json` |
| R22 | `tests/test_eval_full.py`, four_flags의 `mpg eval --full --segments` JSON |
| R23 | `tests/test_guide.py` |
| R24 | `tests/test_examples.py`(slow), MP4 8개(`turret`, `roller_route` 포함) |
| R26 | `tests/test_markers.py` |
| R27 | `tests/test_gaze.py` |
| R25 | 처음 보는 작업자 시험의 작업 기록 |
