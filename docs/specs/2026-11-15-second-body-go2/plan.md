# Plan: M3 두 번째 몸 (Unitree Go2)
Status: draft 2026-11-15
View (read-only copy): (아직 없음)

근거: intent.md, spec.md (둘 다 approved 2026-11-15, 같은 폴더). 이 문서만 보고도 만들 수 있게 쓴다.

## 0. 전제
- 저장소 `~/dev/mujoco-puzzle-gym`, HEAD `a82ecb3` 기준. `python`은 PATH에 없다 — `/Users/songseunghun/dev/mujoco-puzzle-gym/.venv/bin/python`을 쓴다. `timeout`도 없다(`perl -e 'alarm …'`).
- 구현자: Claude-Mac. 실험이 여러 번 이어지는 단계(R43 걷기 조정, R47 치수 실측)는 subagent에 맡기고 데스크는 결과 확인과 승인을 맡는다.
- 상위 자료(읽기만, 저장소에 넣지 않음):
  - `google-deepmind/mujoco_menagerie` `unitree_go2/` — BSD-3-Clause. 관절 순서 **FL, FR, RL, RR**(hip/thigh/calf). `base` 충돌 박스 half-extents `0.1881 0.04675 0.057`, 앞쪽 실린더 `x=0.285`·구 `x=0.293`, 질량 6.921 kg, 키프레임 `home`의 `qpos z=0.27`. **카메라 없음**(`site name="imu"` 하나).
  - Hugging Face `diasAiMaster/unitree-go2-velocity-flat` — bsd-3-clause. `policy.onnx` 14,196 B + `policy.onnx.data` 757,248 B, `params/deploy.yaml`, `params/env.yaml`. ONNX 서명 `obs [1,45]` → `actions [1,12]`. `git/unitree_rl_mjlab.diff`는 빈 diff(업스트림 무수정 학습).
- 새 의존성 없음. `onnxruntime`은 이미 있다(마이크로덕이 쓴다). **BAM(better-actuator-models)은 Go2에 쓰지 않는다** — 이 정책은 PD 게인을 `deploy.yaml`로 직접 준다.
- push·PR은 매번 사용자 승인. 병합과 브랜치 삭제는 사용자. `rm` 쓰지 않는다. `gh`는 `~/tools/gh/gh_2.103.3_macOS_arm64/bin/gh`. CI는 폴링하지 않는다.

## 1. 착수 전 반드시 읽을 코드 (여기에 M3의 진짜 위험이 있다)

구현 전 이 네 곳을 먼저 확인한다. 몸이 하나일 때는 안전했던 **관대한 기본값**들이고, 값을 선언하지 않는 몸이 들어오면 `mpg check`가 **자신 있게 틀린 답**을 낸다.

| 위치 | 현재 코드 | 몸이 값을 안 주면 |
|---|---|---|
| [room.py:846](src/mujoco_puzzle_gym/room.py:846) | `vals.get("max_step", math.inf)`, `vals.get("max_slope", 90.0)` | 턱과 경사가 **아무것도 막히지 않는다**. 플래너가 못 넘는 턱을 넘는 경로를 낸다. |
| [check.py:248](src/mujoco_puzzle_gym/check.py:248) | 같은 기본값으로 `m.step_reasons(...)` | R37 턱·경사 검사가 **전부 통과**한다. |
| [grab.py:190](src/mujoco_puzzle_gym/grab.py:190) | `values.get("max_lift_mass", np.inf)` | 무게 제한 없이 **무엇이든 든다**. |
| [check.py:369](src/mujoco_puzzle_gym/check.py:369) | `values.get("beak_height")` → None이면 건너뜀 | 부리 없는 몸에서 머리 높이 검사를 건너뛴다. **이건 올바른 동작**(비교용으로 적어 둠). |

→ **결론: Go2의 `Capabilities.values`는 비워 둘 수 없다.** `max_step`, `max_slope`, `max_lift_mass`를 실측해 반드시 선언한다(R45). 선언하지 않으면 게이트가 아니라 거짓말이 된다. 참고로 마이크로덕은 [microduck.py:166](src/mujoco_puzzle_gym/bodies/microduck.py:166)에서 `max_lift_mass 0.05`, `max_step 0.01`, `max_slope 5.0`, `beak_height 0.21`을 선언한다(OQ5 실측).

함께 확인할 접합부:
- [env.py:107](src/mujoco_puzzle_gym/env.py:107) `make_body(DEFAULT_BODY if body_name == "default" else body_name)` — **환경은 이미 몸 이름을 받는다.** R49는 CLI에서 이 인자까지 연결하는 일이다.
- [check.py:91-97](src/mujoco_puzzle_gym/check.py:91) — 능력 게이트와 거부 이유 문구가 **이미 구현되어 있다.** R45는 새로 만드는 게 아니라 **처음으로 작동시키는** 일이다.
- `make_body()`를 인자 없이 부르는 곳: [bench.py:41](src/mujoco_puzzle_gym/bench.py:41), [room.py:981](src/mujoco_puzzle_gym/room.py:981). R49에서 몸을 넘기도록 고친다.
- [tests/test_body_caps.py:90](tests/test_body_caps.py:90) `assert make_body().capabilities.missing(part.requires_body) == []` — 몸이 하나라서 항상 통과했던 단정. 마이크로덕용으로 남기고 Go2용 대응 테스트를 R45에서 새로 만든다.

## Files that change
| File | Change | R |
|---|---|---|
| `src/mujoco_puzzle_gym/upstream.go2.lock.json` | **새로** (기존 lock은 몸 하나 구조라 분리; 사용자 승인). Menagerie 고정 커밋 `370c4396…` + HF 고정 리비전, 파일별 SHA256, BSD-3-Clause 문구 2건 | R40 |
| `src/mujoco_puzzle_gym/assets.py` | github 소스에 `url_template`이 있으면 **파일 단위 raw 받기**(tar 전체 657 MB를 메모리에 올리는 현재 경로를 피한다), 없으면 기존 tar 경로 그대로 | R40 |
| `src/mujoco_puzzle_gym/bodies/go2.py` | 새로. `UnitreeGo2Adapter` | R41–R45 |
| `src/mujoco_puzzle_gym/bodies/__init__.py` | `make_body("go2")` 분기, `__all__` | R41 |
| `src/mujoco_puzzle_gym/room.py` | `grid()`의 관대한 기본값을 "값 없으면 거부"로, `make_body()` 호출에 몸 전달 | R45, R49 |
| `src/mujoco_puzzle_gym/check.py` | 같은 기본값 처리, 거부 이유 문구 다듬기 | R45 |
| `src/mujoco_puzzle_gym/grab.py` | `max_lift_mass` 기본값 처리 | R45 |
| 후보 부품들(`parts/device/hitch_station.py`, `parts/.../rope.py`, 발판, 스프링, 컨베이어) | 무게·치수 기준을 몸 상대값으로, 안 되면 `requires_body` 선언 | R48 |
| `src/mujoco_puzzle_gym/bench.py` | `make_body()` → 몸 선택 | R49 |
| `src/mujoco_puzzle_gym/cli.py` | 모든 명령에 `--body {microduck,go2}`, 기본 `microduck`, `--json`과 `task.resolved.yaml`에 기록 | R49 |
| `src/mujoco_puzzle_gym/presets/` + `examples/` | Go2용 방 프리셋 신규. **기존 마이크로덕 방은 수정 금지** | R47 |
| `tests/test_go2_contract.py` | 새로. 45/12 서명, `deploy.yaml` 일치, 관절 매핑, `command_from_action` | R41, R42, R44 |
| `tests/test_go2_walk.py` | 새로. 전진·정지·회전 부호·횡이동 부호 | R43 |
| `tests/test_capability_gate.py` | 새로. 능력별 거부 + 마이크로덕 통과 + `docs/bodies.md` 표 일치 | R45, R46 |
| `tests/test_assets.py` | 짝 파일 손상 2케이스 | R40 |
| `tests/test_body_caps.py` | 마이크로덕 전용으로 범위 명시 | R45 |
| `docs/bodies.md` | 새로. 부품 × 몸 표, 출처·라이선스, "공식 아님" 표기 | R46, R51 |
| `README.md` | 몸 두 개, 출처·라이선스, 커뮤니티 체크포인트 명시 | R51 |
| `.github/workflows/ci.yml` | 샤드별 최대 RSS 출력 | R50 |

## Order of work
PR 네 묶음. 단계마다 테스트를 먼저 쓰고 통과시킨다.

**PR A — 자산과 계약 (R40, R41, R42, R44)**
1. `assets.py`에 짝 파일 묶음 지원. 실패는 **원자적**: 한쪽만 틀려도 둘 다 `.bad` 격리, `policy.onnx`는 로드되지 않음. 먼저 `tests/test_assets.py`의 두 케이스를 쓴다.
2. `upstream.lock.json`에 Go2 항목(Menagerie 커밋 고정, HF 리비전 고정, 측정된 SHA256 4~N건). `mpg fetch-assets --json`으로 `verified: true` 확인.
3. `bodies/go2.py`: `deploy.yaml`을 읽어 관측 순서·차원·action scale·offset·PD 게인·명령 범위를 **런타임에 가져온다**. 코드에는 "45차원이어야 한다"는 기대만 두고, 어긋나면 로드에서 터진다. 숫자를 코드에 복사하지 않는다. `joint_ids_map`은 **읽지도 쓰지도 않는다**(R42: 실기 SDK용). PD는 `spawn`에서 Menagerie의 `motor`를 `position` 액추에이터로 바꿔 모델에 박고, R53 표의 나머지 항목(calf armature, 발 마찰·condim·solimp, 초기 높이 0.32)도 같은 자리에서 메모리 편집한다.
4. `command_from_action` 재정의: action[0] = −1 → `vx = 0`(정지, 후진 아님), +1 → `vx = 1.0`. `cmd_limits` 기본 `vx [-0.5, 1.0]`·`vy [-0.5, 0.5]`·`wz [-1.0, 1.0]`, `full_command_range=True`면 `deploy.yaml` 범위.
5. `tests/test_go2_contract.py` 통과.

**PR B — 걷는 것을 눈으로 확인 (R43)**
6. 평지에서 전진·정지·회전·횡이동 측정. **여기가 M3의 관문이다**: 걷지 않으면 나머지 R은 의미가 없으므로, 막히면 멈추고 사용자에게 보고한다(정책 교체 또는 M3 중단 판단).
7. 회전·횡이동 **부호** 테스트를 반드시 넣는다. 좌우 관절 뒤집힘은 차원 검사로 절대 안 잡히고, 뒤집힌 채로도 그럴듯하게 걷는다.
8. 실측값으로 R43의 허용 범위와 `cmd_limits`의 적정성(spec Open question)을 확정. 전진 영상 1개를 사용자에게 보낸다.

**PR C — 능력 정직화 (R45, R46, R48)**
9. Go2 `Capabilities`: `names`는 `{"walk"}`만. `values`에 실측 `max_step`, `max_slope`, `max_lift_mass`를 **반드시** 넣는다. `grab_site`/`tow_site`/`view_camera` → None, 애드온 없음, `add_rangefinders` 미구현, `set_head` 무시.
10. §1 표의 관대한 기본값 세 곳을 고친다: 몸이 값을 선언하지 않으면 **무한 허용이 아니라 거부**(또는 명시적 오류). 마이크로덕은 값을 다 주므로 기존 동작이 바뀌지 않는다 — 기존 테스트 전체가 그 증거다.
11. `tests/test_capability_gate.py`: 능력별 방 하나씩 Go2로 `mpg check` → 전부 `ok: false` + 부품명·능력명이 이유에 포함. 같은 방들이 마이크로덕으로는 통과.
12. R46 감사: 기존 테스트와 예제 방을 `--body go2`로 돌려 부품별 (a)선언대로 거부 (b)선언 없는데 깨짐 (c)선언 있는데 잘 됨 으로 분류. (b)(c)는 버그로 보고 `requires_body`를 고친다. 결과를 `docs/bodies.md` 표로.
13. R48: 로프 `max_hold_force` 3 N([hitch_station.py](src/mujoco_puzzle_gym/parts/device/hitch_station.py) 문서 기준 `Knot.TIED_HOLD`), 발판 문턱, 스프링 `kick`, 컨베이어 `shift`를 Go2 체급(몸통만 6.921 kg)으로 재검토. 몸 상대값으로 바꾸거나 `requires_body`로 거부. **마이크로덕 쪽 기존 테스트가 전부 그대로 통과해야 한다.**

    실제로 한 것(2026-11-16, 사용자 "승인"으로 spec R48 개정): 숫자를 다시 잡는 대신 **몸 상대 경고**를 넣었다. 기존 숫자를 바꾸면 마이크로덕 방·프리셋·측정 기록이 흔들리므로(R47의 재현성 보존과 같은 이유), 그 몸에서 숫자가 무의미할 때 `check_parts`가 말하게 했다. 스프링·컨베이어는 **질량 무관**이라 고칠 것이 없었고(계획의 전제가 틀렸다), 대신 계획에 없던 `wind`(힘 [N], 스키마 상한이 Go2 무게의 13 %)와 `npc` 속도 경고(`DUCK_SPEED` 하드코딩)가 나왔다. 측정과 근거는 [docs/bodies.md](../../bodies.md) §5.

**PR D — 방·CLI·평가·문서 (R47, R49, R50, R51, R52)**
14. `footprint_radius`, `fall_height` 실측(참고: 서 있는 몸통 높이 0.27 m, 몸통 박스 0.376 × 0.0935 m에 앞쪽 돌출 x=0.293). Go2용 방 프리셋 신규 — 문 폭·발판 크기·골 반경을 이 치수로. 기존 방은 건드리지 않는다.
15. CLI `--body`, `bench.py`·`room.py`의 인자 없는 `make_body()` 정리, `task.resolved.yaml` 기록. 없는 몸은 0이 아닌 종료 + 쓸 수 있는 몸 목록.
16. 마이크로덕 회귀 확인: 예제 방 `mpg check` 결과와 R10 성공률이 M3 이전과 같다.
17. R52 평가: Go2 프리셋, 학습에 안 쓴 시드 100 에피소드, 성공률 ≥ 60% + 성공 MP4. 미달이면 `mpg diagnose` 분류와 근거 숫자를 먼저 보고한다(목표를 내리기 전에 원인을 가린다).
18. R50: 샤드별 최대 RSS 출력. 2샤드 예산을 넘으면 Go2 테스트 분리 여부를 사용자에게 확정받는다.
19. R51: `docs/bodies.md`와 README에 출처·라이선스 2건, 평지 학습·`gait_phase` 없음·action scale 0.5·업스트림 무수정, 그리고 **공식 배포가 아님**. "공식" 문자열 검사 테스트.

### PR D에서 한 것 (2026-11-16)

**15 → R49 (CLI `--body`).** `build`·`check`·`train`·`eval`·`render`·`demo`·`diagnose`에 붙였다. 계획과 두 군데 다르다. ① **기본값을 `microduck`으로 두면 안 됐다** — 그러면 `--body`를 생략한 호출이 task의 `body:`를 덮어써서 둘째 몸용 task가 전부 조용히 첫째 몸으로 돌아간다. task를 읽는 명령의 기본값은 "주지 않음"이고 `bodies.resolve`가 한 곳에서 해석한다. ② **run 폴더에서는 고르지 않고 확인한다**(`evaluate.require_run_body`): 정책은 학습한 몸에 묶여 있고, 행동 공간이 같은 몸끼리는 다른 몸으로 돌려도 터지지 않아 조용히 틀린 숫자가 나온다. 계획의 "`room.py`의 인자 없는 `make_body()` 정리"는 **정리가 아니라 버그 수정**이었다: `mpg build`가 방이 어느 몸을 위한 것이든 늘 기본 몸을 컴파일해, MJCF와 PNG가 그 방에서 쓰이지 않을 몸을 보여 줬다. 새 테스트 `tests/test_body_flag.py` 18개.

**14 → R47 (큰 몸용 방).** `presets/puzzles/wide/`에 둘. `switch_door` 20/20(평균 24.6 s), `pit_detour` 19/20(평균 30.8 s). 폴더 이름이 `go2/`가 될 수 없다는 것부터 배웠다 — 프리셋은 패키지 안이고 R2 경계 검사가 패키지의 모든 파일을 훑는다. 그래서 디렉터리는 치수 요구(`wide`)이고 몸은 패키지 밖 task 파일이 말한다. 치수를 잡으며 네 가지를 측정했고, 그중 하나는 **공용 코드 수정으로 이어졌다**:
- 문틈 1.0 m는 계획 불가(남는 통로 = 틈 − 약 1.0 m; 1.0 m에서 0칸, 1.2 m에서 0.20 m, 1.4 m에서 0.40 m).
- 넓은 문은 `speed`를 적어야 한다(패널 이동 거리 = 문폭; 없으면 1.4 m 문이 40 s 판을 거의 다 쓴다).
- 목표는 벽 깃 0.46 m 밖에, 무작위 오프셋까지 포함해 둔다.
- **해답 조향이 이 몸을 운전할 수 없었고, 답은 공용 코드가 아니라 경계에 있었다.** `solver._steer`가 방향을 맞출 때 명령 범위의 중간을 써서 선회 반경이 전진 속도 ÷ 회전 속도만큼 남았다(이 몸 1.0 m, 최대 전진 2.0 m = 방 반쪽). 몸이 0.6 m 버튼을 한 바퀴에 두 번 0.07 m 차로 스쳐 지나며 40 s를 다 썼다(5/20).

  공용 조향에 규칙을 쓰려는 시도를 네 번 하고 네 번 다 반대쪽을 깼다. 순서대로: ① 늘 제자리 회전 → 새 방 20/20, 느린 묶음 **7건 실패**(바퀴 5건, 몰기, 끌기). ② 큰 각도(1.2 rad)에서만 → 끌기 10건 회복, 몰기 실패 유지, 새 방 15/20. ③ 몸 상태 조건(애드온·든 것·끌기) → 새 방 20/20, 바퀴 17/20. 계측해 보니 바퀴 구간에서는 조건이 정확히 작동했고 깨진 곳은 **바퀴를 달기 전 1.6 s**였다 — 그 방들은 문이 7.5 s만 열리는 시간제라 제자리 회전으로 잃는 초가 치명적이다. "서는 것은 공짜"가 틀렸다. ④ Dubins 도달 조건(`d < 2R sin θ`) → 바퀴 17/20, 새 방 16/20.

  네 번째에서 상수 깎기를 멈췄다. **두 몸이 서로 다른 조향을 원하고 조향기는 공용이면, 답을 가진 쪽은 몸이다.** `BodyAdapter.stand_to_turn_above`(rad, 기본 `inf` = 절대 서지 않음)를 두고 둘째 몸만 `0.0`을 선언했다. 기본값이 무한이라 기존 몸에서 `_steer`의 식이 원본과 동일하고, 그래서 기존 측정값이 흔들릴 수 없다 — 추론이 아니라 스태시해서 HEAD에서 직접 재어 확인했다(`roller_rink` 18/20 평균 21.82 s, `crosswind` 19/20 평균 12.46 s, 내 트리와 **같은 값**). 그 과정에서 `docs/parts-guide.md`가 이 둘을 20/20으로 적고 있는 것이 **내 작업 이전부터 낡은 숫자**임을 알았다. 낡은 줄을 지금 고치지 않는다: 다시 재어야 할 프리셋이 그 둘만인지 모르고, 24개 × 20시드는 별 작업이다.

  `PEEK`(앞이 벽이면 제자리 회전)는 지웠다가 되살렸다 — 중간 구간의 원래 의미에 포함돼 있었다.

**18 → R50.** `tools/peak_rss.py`를 두고 CI의 fast 샤드를 그것으로 감쌌다. `ru_maxrss`(자식 중 **가장 큰 한 프로세스**의 고수위, 동시 합계가 아니다)를 `peak-rss-mib <n>` 한 줄로 찍고 종료 코드를 그대로 전달한다 — 메모리로 죽은 샤드에서 그 숫자가 가장 필요하다. 리눅스 측정은 CI가 돌아야 나오므로, **2샤드 유지냐 Go2 테스트 분리냐는 첫 CI 로그를 보고 사용자가 확정**한다(spec의 열린 질문 그대로).

**16 → 마이크로덕 회귀.** 공용 조향을 건드렸으므로 빠른 묶음(실패 0)만으로는 부족하다. 20시드 방 점검은 느린 집합에 있어 `-m ''` 전체 실행으로 따로 확인한다.

남는 한계를 적어 둔다: 느린 묶음이 단정하는 것은 **통과 여부**(20개 중 18개 이상)이고, `docs/parts-guide.md`의 프리셋 표에 적힌 **평균 통과 시간**은 옛 조향으로 잰 값이다. 이제 해답이 방향을 맞출 때 서므로 시간이 조금씩 달라질 수 있는데, 그 숫자를 전부 다시 재는 것은 프리셋 24개 × 20시드이므로 이 PR에서 하지 않았다. 숫자가 틀렸다는 뜻이 아니라, **다시 재지 않았다**는 뜻이다.

**조향 수정이 왜 안전한가(사후 확인).** 걷기 조향은 네 곳에 있다. `carry.steer`와 `herd._steer`는 **이미** 방향이 틀렸을 때 `[-1.0, 0, sign(err)]`, 즉 제자리 회전을 쓰고 있었다. `push._steer`는 `slow_turn`일 때만 그렇게 한다. 중간 속도를 쓰던 곳은 `solver._steer` **하나뿐**이었다. 그러니 이 수정은 동작을 바꾼 것이 아니라 네 형제 중 혼자 다르던 하나를 맞춘 것이다 — 두 형제가 이미 같은 선택을 하고 있었다는 것이 이 방향이 맞다는 가장 강한 증거다. `push`는 손대지 않았다: 밀다가 서면 공과의 접촉을 잃을 수 있어 `slow_turn` 조건이 있는 것으로 보이고, 큰 몸으로 공 미는 방을 측정한 적이 없어 추측으로 고치지 않았다(`docs/bodies.md` §7에 열린 항목으로 적었다).

**추가로 잡은 버그.** `bench`에 `--body`를 붙이자 병렬 측정이 첫째 몸의 비율(몸 스텝당 물리 4스텝, 고수준 스텝당 몸 5스텝)을 **하드코딩**하고 있는 것이 드러났다. 둘째 몸은 10스텝이어서 단일 프로세스 행은 맞고 병렬 행은 일어나지 않은 물리 스텝을 보고했다. 측정한 몸의 비율을 넘겨 고쳤고, `bench`의 결과에 몸 이름이 들어간다 — 몸 이름 없는 "초당 스텝"은 어떤 것과도 비교할 수 없다.

## Proof (R별 증거)
| R | 무엇을 보면 통과인가 |
|---|---|
| R40 | `mpg fetch-assets --json`의 go2 항목 `verified: true`; `tests/test_assets.py`의 `.onnx.data` 손상·`policy.onnx` 손상 두 케이스가 둘 다 0 아닌 종료 + `.bad` 격리; 캐시 증가분 ≤ 771,444 B |
| R41 | `tests/test_go2_contract.py`: ONNX 입력 `[1,45]`·출력 `[1,12]`; `deploy.yaml` 6항목 차원 합 45; 골든 관측 벡터 일치(1e-6); 가짜 `deploy.yaml`(46차원)로 로드 실패 |
| R42 | 컴파일된 모델의 관절 순서 == `FL, FR, RL, RR`; `default_joint_pos` hip 부호가 L 음수·R 양수; 어댑터가 `joint_ids_map`을 쓰지 않음; R43의 부호 테스트 통과 |
| R53 | 컴파일된 모델의 액추에이터 종류·kp·kv·forcerange, 관절별 armature(calf 0.02), 발 friction·condim·solimp, reset 후 몸통 높이 0.32가 R53 표의 "학습 당시" 값과 일치 |
| R43 | `tests/test_go2_walk.py`: `vx 0.5` 5초 → 1.5–3.0 m 전진, 넘어짐 없음; 명령 0 → 이동 < 0.15 m; `wz +0.5` → yaw 반시계 1.0–3.5 rad; `vy +0.5` → 왼쪽 ≥ 1.0 m |
| R44 | action[0] = −1 → `vx == 0`; +1 → `vx == 1.0`; `full_command_range=True`의 `cmd_limits` == `deploy.yaml` 범위 |
| R45 | `tests/test_capability_gate.py`: 능력별 방이 Go2로 전부 `ok: false` + 이유에 부품명·능력명; 같은 방이 마이크로덕으로 통과; Go2 `values`에 `max_step`·`max_slope`·`max_lift_mass`가 있음을 단정 |
| R46 | `docs/bodies.md`에 전 부품 행; 각 부품 `requires_body`가 표와 일치(갈라지면 실패) |
| R47 | Go2 프리셋이 `mpg check` (a)(b)(c) 통과, 스크립트 해답 20개 중 ≥18; 마이크로덕 예제 방 결과·성공률 **불변**(회귀) |
| R48 | 후보 부품마다 Go2에서 "의도대로 작동" 또는 "이유를 말하며 거부" 중 하나, 조용히 틀린 결과 없음; 마이크로덕 기존 테스트 전부 통과 |
| R49 | `tests/test_cli_json.py`가 모든 명령을 `--body go2`로도 JSON 파싱; `--body g1`은 0 아닌 종료 + 몸 목록 |
| R50 | Actions 녹색 + 샤드별 최대 RSS 로그 + 12 GB 대비 여유 기록 |
| R51 | 두 문서에 출처·라이선스·"공식 아님" 문장; "공식" 문자열 검사 테스트 통과 |
| R52 | `runs/<id>/summary.json` 성공률 ≥ 0.60 + 성공 MP4; 미달 시 diagnose 분류와 근거 숫자가 `docs/bodies.md`에 |

## Risks
1. **정책이 Menagerie 모델에서 안 걷는다** (가장 큰 위험, 2026-11-15 조사로 상당히 줄었다). 확인된 것: 상류 `unitree_rl_mjlab`의 Go2 OBJ 메시는 Menagerie와 **바이트 단위로 동일**하고 관절·바디 순서도 같다. 남은 차이는 상류가 파이썬으로 주입하는 물리값뿐이고, 그 목록을 `go2_constants.py`에서 전부 읽어 R53의 표로 고정했다(액추에이터, calf armature, 발 마찰, 초기 높이). 즉 "다를 수 있다"가 아니라 "무엇이 다른지 안다". 남는 미지는 상류 `<option>`(solver·cone·impratio; 상류는 Menagerie의 `cone="elliptic" impratio="100"`을 제거하고 mjlab 기본값을 쓴다)과 타임스텝이다. → PR B가 관문이다. 막히면 `<option>`을 먼저 의심하고, 그래도 안 되면 멈추고 보고한다.
2. **좌우 관절 뒤집힘**. 차원 검사로 안 잡히고 그럴듯하게 걷는다. → R43의 부호 테스트가 유일한 방어선. 이 테스트를 뒤로 미루지 않는다. **주의: `deploy.yaml`의 `joint_ids_map`을 시뮬레이션에 적용하면 이 버그를 직접 만든다**(R42 참조, 2026-11-15에 정정). 그 맵은 실기 SDK용이다.
3. **관대한 기본값을 고치다 마이크로덕이 깨진다**. §1의 세 곳을 "값 없으면 거부"로 바꾸는 변경이다. 마이크로덕은 값을 모두 선언하므로 영향이 없어야 하고, 기존 테스트 전체가 그 증거다. 깨지면 기본값 변경이 아니라 호출부 문제다.
4. **CI 메모리**. #45 2샤드 직후다. Go2 테스트가 들어가면 다시 올라간다. → R50이 측정을 의무화하고, 퇴로(분리)를 미리 정해 뒀다.
5. **방 치수와 명령 범위가 서로 묶여 있다**. R44의 `vx [-0.5, 1.0]`는 방 크기를 모르는 상태에서 10 Hz × 0.2 m로 제안한 값이다. R47 실측 후 한 번 다시 본다(바꾸려면 spec 개정).
6. **커뮤니티 체크포인트 의존**. HF 저장소가 사라지거나 리비전이 바뀔 수 있다. → R40이 리비전과 SHA256을 고정하므로 내용이 바뀌면 조용히 바뀌지 않고 검증에서 터진다. 저장소 자체가 사라지는 경우는 막을 수 없고, 그때는 자산 출처를 바꾼다.

## 측정 기록

PR A에서 끝난 것 (2026-11-15, 맥 로컬):
- **걷기**(R43): 평지 5초. `vx 0.5` → 2.27–2.49 m(시드 10개 편차 1%), `vx 1.0` → 5.45 m, `vx -0.5` → −1.37 m, `vy +0.5` → +2.17 m(왼쪽), `wz +0.5` → yaw +1.28~1.29 rad(반시계), 정지 → 0.11–0.14 m. 전부 넘어짐 0. 추종률은 전진 99%, 횡 87%, 회전 52%.
- **접촉 설정이 회전을 좌우한다**(R53): 상류처럼 `cone`·`impratio`·발 접촉을 벗기면 `wz ±0.5`에서 몸통이 0.058 m까지 주저앉고 양방향 모두 넘어진다. Menagerie 설정을 그대로 두면 0.367 m로 서서 돈다.
- **각속도 프레임**: MuJoCo 자유관절의 `qvel[3:6]`은 이미 바디 프레임이다. 스파이크에서 한 번 더 회전시켰다가 고쳤고, 결과는 거의 같았다(회전 중 각속도는 대체로 `[0,0,wz]`이고 z축 회전은 z 성분을 바꾸지 않는다). 즉 회전 추종률 52%는 obs 버그가 아니라 정책의 특성이다.
- **메모리**(R50): Go2 비용은 **일회성 260 MB**(컴파일 중 일시 할당이 힙에 남음, 반복 컴파일에도 347 MB에서 평탄 — 누적 아님) **+ 살아 있는 모델당 39 MB**(`MjModel` 배열 38 MB와 일치; 그중 BVH 24 MB가 시각 전용 메시에도 생긴다). 빠른 묶음 전체 peak footprint는 Go2 포함 5,594 MB 대 Go2 테스트 제외 5,670 MB로 **Go2 쪽이 오히려 낮다**(실행 간 잡음). Go2의 일회성 고수위는 묶음 초반에 찍히고 정점은 후반(rope·sensors·render)에서 나오므로 정점이 올라가지 않는다. 처음에 과거 일지의 3,578 MB와 비교해 "+2.2 GB"로 본 것은 **조건이 다른 수치와의 교차 비교로 인한 오판**이었다. 리눅스 CI 측정은 R50에서 따로 한다.
- 빠른 묶음 1300 passed / 1 skipped / 실패 0 (130.9 s).

PR C 2단계에서 끝난 것 (2026-11-16, 10시드/점, 시드는 시작 자세를 ±0.02 m·±0.03 rad 흔든 것; vx 0.5, MuJoCo 기본 바닥 마찰):
- **`max_step` 0.03 m**(위): 0.01·0.02·0.03 m 10/10, 0.04 m 3/10, 0.05 m 0/10, 0.08~0.10 m 0/10(0.06 m 1/10은 표본 잡음). **어느 높이에서도 넘어지지 않았다** — 한계를 넘으면 턱에 막혀 멈출 뿐이다. 즉 오르기 한계이고 자세 유지 한계가 아니다. 아래로는 0.09 m까지 무낙상(0.09까지 10/10, 0.10과 0.12 모두 6/10, 0.15부터 0/10)이고 이쪽 실패는 실제 낙상이다. 내려가기 한계를 소비하는 코드가 없어 선언은 위쪽만 한다.
- **`max_slope` 6 deg**: 4·5·6 deg 10/10, 7 deg 9/10, 8 deg 이상 0/10. 역시 20 deg까지 **넘어지지 않고 정체한다**. 10초간 오른 거리가 4 deg 2.78 m → 6 deg 2.02 → 8 deg 1.44 → 10 deg 0.38으로 완만히 떨어지므로, 이 수치는 "쓸 만한 전진이 남아 있다"는 뜻이지 "넘어지지 않는다"가 아니다. 두 번째 기준(≥1.5 m 등반)도 같은 경계를 준다. 전진이 무너지는 지점은 8~9 deg.
- **`footprint_radius` 0.41 m**: 리셋 자세의 컴파일된 모델에서 몸통 중심에서 가장 먼 geom이 0.406 m(시각 calf 메시), 충돌 geom만 세면 0.397 m(뒤 thigh 충돌 박스). **먼저 적어 둔 0.38은 실제 몸보다 작았다** — `inflate = footprint_radius + CLEARANCE`로 벽을 부풀리는 데 쓰이므로 작게 적으면 지나갈 수 없는 틈을 지나간다고 계획한다. subagent 보고를 받고 독립 측정해 0.4060/0.3970으로 확인했다.
- **`max_lift_mass` 0.0은 자리표가 아니다**: `beak_grab`이 없으니 들 수단 자체가 없다.
- 측정 스크립트와 원시 결과: 스크래치패드 `go2caps/measure.py`, `raw_results.tsv`(점마다 `# SUMMARY` 줄). 경계점 0.05 m를 재실행해 0/10 재현 확인.
- 무한 허용 기본값 제거(PR C 1단계): `bodies/base.py`의 `UNDECLARED_LIMIT = 0.0`과 `declared_limit()`, `Capabilities.limit()`. `room.grid`·`check.map_steps`·`grab`이 이걸 쓴다. `roommap`의 비교가 `> 한계 + EPS`라 평평한 바닥은 영향 없음. 빠른 묶음 1302 passed로 마이크로덕 무변경 확인.

PR C 3·4단계에서 끝난 것 (2026-11-16, 부품 상호작용은 5시드/점, 시드는 시작 자세를 ±0.03 m·±0.05 rad 흔든 것):
- **감사 대상 59개 부품 전부** 정적 분류 + 설정 변형 7종. 전체 표와 근거는 [docs/bodies.md](../../bodies.md). (a) 선언대로 거부 24개, (b) 선언 없는데 깨짐 **4개**, (c) 선언 있는데 잘 됨 **0개**.
- **(b) 네 건**: `lever` `mode: push`(Go2 5/5 `.angle`이 시작값 −1.000 그대로, 접촉 0회 — 몸통 아래 0.237–0.243 m가 패들 윗면 0.120 m 위를 지나가고 발은 반폭 0.060 m 밖으로 지나간다. **어려운 게 아니라 안 닿는다**); `dial`(통과선에서 1/5만 한 칸, **2/5는 스포크에 걸려 넘어짐**, 아무 일도 없어야 하는 반대 통과가 +2.0–+5.1칸); `hitch_station`(로프 hitch 목록에 `body`가 없어 영원히 안 묶임); `breakable` `by: [peck]`(`beak_site`가 −1이라 영원히 안 깨짐). 네 건 다 수정 전 Go2로 모든 `mpg check` 단계가 `ok=True`였다.
- **치수로만 막히는 것**: `low_bar`는 `bottom` 스키마가 0.21–0.246 m로 묶여 Go2(몸통 윗면 0.41 m)가 어떤 설정으로도 통과 못 한다 → `roller` 선언.
- **몸 질량·속도 실측**: 마이크로덕 0.737 kg(무게 7.2 N) / Go2 15.206 kg(149.2 N), 20.6배. 최고 지령 정상 속도 0.220 / 1.019 m/s, 4.6배.
- **작동 확인**(선언 그대로 맞음): 스프링(1.5 m/s 킥에 0.358 → 0.470 m, 5/5 무낙상, 3 s 뒤에도 걷기 2.25 m), 컨베이어(3 s에 0.85–0.90 m 실림, 지령 0의 제자리 전진 0.26–0.35 m를 뺀 값), 문 폭 0.5 m 통과 5/5(문틈 내 몸 최대 y폭 0.273–0.288 m), 바닥 센서 밟기(몸 중심 좌표 판정이라 치수 무관).
- **의미가 달라지는 것**: 압력판(기본 0.25 m 판에서 Go2는 평균 2.2–5.2 kg, 최대 9.2, 1–5 % 스텝에서 0 kg — 문턱이 스텝별 동전 던지기. 0.9 m 판에서 13.7 kg 정상), `pit` 0.3 m(5시드 중 1개만 건넘, 4개는 테두리에서 넘어져 "구덩이 실패"가 아닌 평범한 낙상. 0.6·0.9 m는 5/5 빠짐 — 경계는 0.3–0.6 m 사이로만 안다).
- **R2 경계 규칙의 구멍**: `tests/test_boundary.py`의 금지 이름 목록에 둘째 몸 이름이 없어, Go2 이름이 `assets.py` 주석에 들어가 있었다. `go2`·`unitree`를 넣고 핀 파일을 허용 목록에 더해 잡았다.
- 측정 스크립트와 원시 결과: 스크래치패드 `go2audit/`(`results.tsv` 214행 + 점마다 `# SUMMARY`, `decl.py`·`rope3.py`·`peck.py`로 네 건의 조용한 통과를 재현).
- 가장 무거운 주장(푸시 레버에 안 닿는다 → 능력 이름을 새로 만드는 근거)은 **독립 재측정**했다(`go2audit/verify_lever.py`, 레버 geom 접촉 수를 직접 셈): 마이크로덕 접촉 87회·각도 +1.000 도달, Go2 **접촉 0회·각도 −1.000 고정**(각 3시드). subagent 결과와 일치한다. 첫 시도에서는 두 몸 다 접촉 0으로 나왔는데, 그건 6 s 안에 레버에 도달하지 못한 것이었다(마이크로덕 0.22 m/s) — 거리와 시간을 맞추고 다시 쟀다.
- 빠른 묶음: 수정 후 1299 passed / 1 skipped / 실패 0. 기존 방·테스트에 새 경고가 하나도 걸리지 않았다.

아직 남은 것:
- 샤드별 최대 RSS, 리눅스 CI 수치 → PR D(R50)
- Go2 성공률과 평균 소요 시간 → PR D(R52)
- `pit` 경계 0.3–0.6 m 사이 좁히기(쓸 일이 생기면)
- `docs/bodies.md`의 en/ja 번역과 `guide.en.md`·`guide.ja.md`의 링크 → PR D(R51). 지금은 한국어판만 있고 링크도 `guide.md`에만 있다.
