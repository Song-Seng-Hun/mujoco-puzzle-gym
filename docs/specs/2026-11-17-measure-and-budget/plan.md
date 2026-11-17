# Plan — 측정과 배분

Status: 개정 2026-11-17 (Claude-Mac). Spec: [spec.md](spec.md) R90~R97 (R89 철회).
Baseline: `f3619c0`. 브랜치: `measure/budget-and-balance`.

이 계획만 보고 만들 수 있게 쓴다. 대화를 본 적 없는 사람이 기준이다.

## 본받을 코드 (읽고 확인함)

| 새 부품 | 본 | 가져올 것 |
|---|---|---|
| `beam` | `parts/input/stack.py` | `accept`/`accepts()` 규칙, `candidates()`, `settle` + `REST_SPEED`, `_on_out` 분리, `check_static` |
| `budget` | `parts/logic/counter.py` | `BodiedLogic`, `stateful`, `suffixes`, `_rising()`, `plan_values()`, `save_state`/`load_state` |

중요한 전례 둘:

- 필드 이름은 `accept`다. `accepts`로 하면 `grabbable.takes` 프로토콜의 `accepts(obj)` 메서드를
  리스트로 덮어써서 큐브가 든 방이 아예 안 열린다(`stack.py` 머리글에 기록된 실패).
- 질량은 `update`에서 읽고 `substep`을 쓰지 않는다. 피크를 쓰면 5 cm에서 떨어뜨린 30 g 큐브가
  0.35 kg로 읽힌다(R5 측정).

## 단계 0 — 탐침 (R97, 새 부품마다 먼저)

새 부품을 만들기 전에 **기존 부품으로 짜 보는 방**을 돌린다. 2026-11-17에 이 단계를 건너뛰어 R89를
다 설계한 뒤에 철회했다. 15분이면 끝난다.

```bash
.venv/bin/python -m mujoco_puzzle_gym.cli check <탐침 방>
```

`beam`에 대해 돌린 결과: 토크(질량 x 거리)는 식으로 표현할 수 없다 — 게이트 식이 산술을 거절한다
("BinOp is not allowed"). `budget`: `count()`가 횟수 예산이므로 그 방식은 빼고, 누적 시간과 공용 풀만
남는다(`timer()`는 꺼지면 0).

## 파일과 순서

각 단계 끝에 그 단계의 검사를 돌린다. 앞 단계가 초록이 아니면 다음으로 안 간다.

1. **`src/mujoco_puzzle_gym/parts/input/beam.py`** (R90) — 새 파일. `Beam(Part)`.
   - 중앙 힌지(`mjJNT_HINGE`, y축) 판 하나. `span`, `pivot_height`, `damping`.
   - 켜짐 판정은 **각도**로 한다(`|angle| <= balance_band`), 토크를 직접 계산하지 않는다. 물리가 이미
     토크를 풀고 있으니 각도가 그 답이다. 질량 x 거리는 물리에서 나온다.
   - `settle` 동안 각속도가 `REST_RATE` 아래여야 켜진다(떨림 방지).
   - **위험**: 가벼운 하중에서 힌지가 떨린다(Codex "unsure: pivot friction"). 1차는 `damping`을
     노출하고 기본값을 측정해서 정한다. 머리글에 측정값을 적는다. 가라앉지 않으면 `armature`를 더한다.
   - 기울어진 판이 길이 되는지는 `max_slope`로 본다(오리 5도, go2 6도) — 방 R93에서 쓴다.
   - 검사: `::test_a_beam_balances_equal_mass_at_equal_distance`,
     `::test_a_beam_tips_when_the_same_mass_moves_outward`, `::test_a_beam_settles_before_it_reads`.

2. **`src/mujoco_puzzle_gym/parts/logic/budget.py`** (R91) — 새 파일. `Budget(BodiedLogic)`.
   - 값: `amount`, `spend`(enum `time`/`use`/`flow`), `rate`, `loads`(채널 목록), `refill`(채널|None),
     `pos`, `body`.
   - 세 방식의 차이를 분명히 한다. `time`: 켜진 load **하나당** `rate`/s. `use`: load의 상승 에지
     하나당 `rate`. `flow`: load가 하나라도 켜져 있으면 전체로 `rate`/s(공용 수도꼭지).
   - `loads`는 **채널 이름 목록**으로 명시한다. 소비처를 코드가 추측하지 않는다 — 방 파일이
     `door.enable: budget`과 `budget.loads: [door_cmd]`를 둘 다 적는다. `BodyAdapter` 선언 원칙과 같다
     (공유 코드는 추측하지 않고, 선언을 읽는다).
   - 출력: 자기 채널(남았으면 1), `.left`(남은 양). 0이 되면 영구히 0 — `refill`이 있을 때만 돌아온다.
   - `save_state`/`load_state`에 남은 양을 넣는다. 새 에피소드는 `amount`에서 시작.
   - `plan_values`: 정적 `order` 단계가 쓸 값. `time`/`flow`는 "충분하다"로 보고 1을 낸다(계획만으로는
     몇 초 켜질지 모른다). 이것 때문에 `order`가 통과해도 해답이 예산을 넘길 수 있다 —
     `sequence` 게이트와 같은 한계이고, 방 머리글에 적는다.
   - 검사: `::test_a_time_budget_drains_only_while_a_load_is_on`,
     `::test_a_use_budget_counts_passes_not_seconds`, `::test_an_empty_budget_stays_empty`,
     `::test_a_budget_survives_a_checkpoint`.

3. **등록** — `parts/input/__init__.py`, `parts/logic/__init__.py`에 import/`__all__`,
   `parts/__init__.py`에 `register_part("beam", ...)`, `("budget", ...)`.
   - 검사: `mpg parts --json`에 셋이 나오고 값 스키마가 보인다.

4. **부품 프리셋** — `presets/parts/input/beam.yaml`,
   `presets/parts/logic/budget_time.yaml`, `budget_flow.yaml`.

5. **방 넷** (R92~R95) — `presets/puzzles/tabletop/odd_one_out.yaml`,
   `lever_arm.yaml`(걷는 몸), `two_openings.yaml`, `weigh_then_spend.yaml`, 그리고
   `examples/task_*.yaml` 넷. `tests/test_examples.py`의 `PRESET_TASKS`에 등록.
   - 방마다: `mpg check --all-bodies`, 해답 20/20, `demo`를 `goto goal` 하나로 끊어 0/5.
   - R94는 음성 대조를 하나 더 한다: `budget.amount`를 1.5배로 올려 **방이 실제로 쉬워지는지**.
     안 쉬워지면 예산이 제약이 아니었다는 뜻이고 그 방은 거짓이다. M5에서 두 번 당한 종류의 실수를
     여기서 막는다(제 방 설계 자체가 버그였던 것 — 반지름 0.19의 둘째 블록, 처음부터 들어 있던 패치).

6. **문서** (R96)와 **조합 잠금 테스트** (R97) — `docs/parts-guide.md` + `.en.md` + `.ja.md`: `TABLE_BEGIN/END` 사이를
   `mpg parts --markdown`으로 다시 만들고, 방 넷을 행에 더하고, R93을 "그 몸이어야 풀리는 방"에 더한다.
   `docs/bodies.md` ×3에는 `beam`의 힌지 측정값을 적는다.

## 위험과 대응

| 위험 | 신호 | 대응 |
|---|---|---|
| `beam` 힌지가 가벼운 하중에서 떨린다 | `settle` 안에 각속도가 안 떨어진다 | `damping` 기본값을 측정해 정한다. 안 되면 `armature` 추가. 끝까지 안 되면 각도 대신 토크 계산으로 바꾸고 spec을 고친다 |
| 해답 스크립트가 예산을 넘긴다 | solver 20/20이 안 나온다 | `plan_values`의 한계를 방 머리글에 적고, `demo:`에 끄는 단계를 명시한다 |
| 방이 거짓(예산이 제약이 아니었다) | 6번의 1.5배 대조에서 안 쉬워진다 | 방을 다시 짠다. 부품 탓이 아니다 |

## R별 증거

- R89/R90/R91: 위 테스트 이름들이 통과. `mpg parts --json`에 값 스키마.
- R92~R95: 방마다 20/20 + demo 끊어 0/5 + `--all-bodies` exclusive. 숫자를 방 머리글에 적는다.
- R94: 1.5배 대조 결과도 머리글에 적는다.
- R96: `tests/test_docs.py` 통과, `PRESET_TASKS` 등록.
- 전체: fast suite와 slow suite 둘 다 초록. 커밋은 R 단위로 끊는다.
