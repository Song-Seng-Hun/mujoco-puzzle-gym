# Plan: mujoco-puzzle-gym 첫 마일스톤
Status: approved 2026-11-11 (user: "승인")
View (read-only copy): (아직 없음)

근거: intent.md, spec.md (둘 다 approved 2026-11-11, 같은 폴더). 이 문서만 보고도 만들 수 있게 쓴다.

## 0. 전제
- 저장소: `Song-Seng-Hun/mujoco-puzzle-gym` (GitHub private, 2026-11-11 기준 비어 있음). 로컬 경로 `~/dev/mujoco-puzzle-gym`.
- 구현자: Claude-Mac. 실험이 여러 번 이어지는 단계(걷기 확인, 보상 조정)는 subagent에 맡기고, 데스크는 결과 확인과 사용자 승인을 맡는다.
- 상위 자료(읽기만, 저장소에 넣지 않음):
  - `pollen-robotics/microduck_rl` (Apache-2.0 코드, 메시 CC BY-NC-SA 4.0): `src/mjlab_microduck/robot/microduck/`(MJCF, 메시), `scripts/infer_policy.py`(CPU MuJoCo + onnxruntime + BAM 실행 예, `bam.model.load_model`, `bam.mujoco.MujocoController` 사용).
  - Hugging Face `pollen-robotics/microduck-policies` (Apache-2.0): `velstand.onnx`, manifest.
- 의존성(파이썬 3.12 고정, `uv`): `mujoco`, `onnxruntime`, `better-actuator-models==1.0.3`, `numpy`, `gymnasium`, `stable-baselines3`, `torch`(CPU), `pyyaml`, `imageio[ffmpeg]`, `matplotlib`, 테스트용 `pytest`. 버전은 1단계에서 `uv.lock`으로 고정한다. MuJoCo 버전은 상위 MJCF가 그대로 로드되는 가장 최신으로 고른다.
- push와 PR은 매번 사용자 승인. 병합은 사용자.
- 경로 표기: 아래 표의 `mujoco_puzzle_gym/...`는 실제로 `src/mujoco_puzzle_gym/...`에 있다(uv 기본 src 구조, 2026-11-11 1단계에서 정함). MuJoCo는 4.0.0이 설치됐다. 상위 MJCF는 4.0.0에서 그대로 로드된다(2026-11-11 확인).

## Files that change
| File | Change | R |
|---|---|---|
| `pyproject.toml`, `uv.lock`, `.python-version`, `.gitignore`, `LICENSE`(Apache-2.0) | 새로 | 전체 |
| `docs/specs/2026-11-10-duck-puzzle/{intent,spec,plan}.md` | hive에서 옮겨 옴 | 전체 |
| `mujoco_puzzle_gym/upstream.lock.json` | 고정 커밋, HF 리비전, 파일별 SHA256, 라이선스 문구 | R1 |
| `mujoco_puzzle_gym/assets.py` | 받기, 해시 확인, 캐시 경로 | R1 |
| `mujoco_puzzle_gym/bodies/base.py` | `BodyAdapter`, `BodyState` | R2 |
| `mujoco_puzzle_gym/bodies/microduck.py` | 마이크로덕 걷기(관측 61, BAM, ONNX, 명령 자르기), 출처 주석 | R3 |
| `mujoco_puzzle_gym/parts/{base,switch,door,goal}.py` | 부품 3종: MJCF 조각 + 상태기계 | R4 |
| `mujoco_puzzle_gym/registry.py` | `register_part`, `register_obs`, `register_reward`, `mujoco_puzzle_gym.algos` 진입점 읽기 | R17, R8 |
| `mujoco_puzzle_gym/room.py` | room.yaml 읽기·검증, `MjSpec`로 몸+방 합치기, 무작위 배치, PNG | R5 |
| `mujoco_puzzle_gym/layout.py` | 방 격자(4 cm): 막힘, flood fill, 측지 거리장 (2026-11-11 PR B 추가) | R6, R7 |
| `mujoco_puzzle_gym/logic.py` | 풀이 순서 찾기(격자 + 링크) (PR B 추가) | R6, R7 |
| `mujoco_puzzle_gym/solver.py` | 스크립트 해답(특권 상태, 격자 경로 따라가기) (PR B 추가) | R6 |
| `mujoco_puzzle_gym/check.py` | 풀이 순서 그래프, 안정성, 스크립트 해답 | R6 |
| `mujoco_puzzle_gym/env.py`, `obs.py`, `rewards.py` | `PuzzleEnv`, 기본 관측·보상 항목 | R7, R17 |
| `mujoco_puzzle_gym/task.py` | task.yaml 읽기, 기본값 채우기, `task.resolved.yaml` 쓰기 | R17 |
| `mujoco_puzzle_gym/algos/{base,sb3}.py` | `Trainer` 인터페이스, SB3 PPO/SAC/A2C 감싸기 | R8 |
| `mujoco_puzzle_gym/{train,diagnose,evaluate,render,bench}.py` | 각 명령 본체 | R8–R11, R13 |
| `mujoco_puzzle_gym/cli.py` | `mpg` 명령 모음, 공통 `--json`·종료 코드 | R12 |
| `examples/switch_door.yaml`, `examples/task_ppo.yaml`, `examples/broken/*.yaml` | 예제 방·과제, 진단용 망가진 설정 3개, check 실패용 방 2개 | R5, R6, R9 |
| `tests/test_*.py` | Proof 표의 테스트 | 각 R |
| `AGENTS.md`, `skills/mujoco-puzzle-gym/SKILL.md` | 에이전트 안내와 스킬 | R12, R18 |
| `README.md`, `README.en.md`, `README.ja.md`, `assets/banner-{light,dark}.svg`, `docs/bench.md` | 문서 | R14, R13 |
| `.github/workflows/ci.yml` | ubuntu, 자산 받기 + 빠른 테스트 | R15 |

## Order of work
단계마다 테스트를 먼저 쓰고 통과시킨다. PR은 네 묶음(A–D)으로 나눈다.

**PR A — 뼈대, 자산, 걷기, 첫 속도 측정**
1. 뼈대 (R12 기반): `uv init`, 패키지 구조, `cli.py`에 공통 출력 함수(`--json`이면 JSON 한 줄, 실패면 종료 코드 1 + `{"ok": false, "error": ...}`). 스펙 세 문서를 `docs/specs/`로 복사. 첫 커밋은 main에 직접(빈 저장소라 PR 대상이 없음), push 승인 후.
2. 상위 고정 (R1): `microduck_rl` develop의 최신 커밋 하나와 HF 리비전 하나를 고른다. 받을 파일 목록(MJCF·메시·`LICENSE-MODELS.md`·`velstand.onnx`·manifest)과 SHA256을 `upstream.lock.json`에 적고, spec의 Open question "고정할 커밋과 HF 리비전"을 닫는다(값만 채우는 것이라 R은 바뀌지 않음).
3. 자산 받기 (R1): GitHub는 `codeload.github.com/.../tar.gz/<commit>`에서 로봇 폴더만 풀고, HF는 `huggingface.co/.../resolve/<rev>/<file>`로 받는다. 받은 파일은 `~/.cache/mujoco-puzzle-gym/<commit>/`에 두고 해시가 다르면 지우지 않고 `.bad`로 이름을 바꾼 뒤 실패한다. 두 번째 실행은 해시만 확인. 라이선스·출처 출력.
4. 몸 경계 (R2): `BodyAdapter`(메서드 `spawn(spec, pose)`, `reset(data, pose)`, `step(data, cmd)`, `state(data) -> BodyState(pos, yaw, lin_vel, ang_vel, gravity, fallen)`, 상수 `cmd_limits`, `control_dt`)와 경계 테스트.
5. 마이크로덕 (R3): `infer_policy.py`(고정 커밋)를 읽고 관측 61차원, 기본 관절 자세, 행동 스케일, BAM 설정을 옮겨 다시 쓴다. 넘어짐 판정은 투영 중력 z와 몸통 높이로. `test_walk.py` 통과까지 subagent가 반복해도 된다.
   - 2026-11-11 구현 메모: `infer_policy.py` 기본값과 학습 설정(`Mjlab-VelStand-Rough-Backlash-MicroDuck`)이 다른 곳은 학습 쪽을 따랐다: 로봇 `robot_allcollisions_backlash.xml`, 엔코더가 백래시를 거쳐 읽힘(관측과 펌웨어 PD), 액추에이터 지연 4 물리스텝(학습 3–6). 이것 없이는 wz=−1.0, vx=−0.4 등에서 제자리에 선다. BAM은 고정 의존성 1.0.3(학습은 BAM git `62bd8ce`, XL330 m6 계수가 다름).
   - 경계 보조: `bodies/__init__.py`의 `make_body()`, `bodies/base.py`의 `flat_floor_spec()`, 어댑터 속성 `timestep`과 `clip_cmd()`. 합친 모델 안의 몸 이름에는 `body/` 접두사가 붙는다(`MjSpec.attach`).
   - **R3 거리 기준 (2026-11-11 사용자 승인으로 spec을 0.5–1.1 m로 변경)**: velstand는 vx를 절반 정도만 따른다. 측정: 명령 0.3 → 5초 0.73 m(약 0.16 m/s), 0.4 → 약 0.23 m/s. 상위 저장소 자체 평가도 0.4 → 0.17 m/s(`docs/velstand_policy.md`). 처음 spec의 1.05–1.95 m는 이 정책으로 닿을 수 없어 0.5–1.1 m로 바꿨다.
6. 첫 속도 측정 (R13 일부): 걷기만 돌려 초당 물리 스텝을 잰다. 위층 1스텝 = 걷기 5스텝이므로 위층 스텝/초를 추정해 `docs/bench.md`에 적는다. 이 숫자로 2시간 안에 몇 스텝을 학습할 수 있는지 먼저 계산해 데스크에 보고한다.

**PR B — 부품, 방, 점검, 환경**
7. 등록 함수 (R17 일부): `register_part`부터. 기본 부품도 같은 함수로 등록한다.
8. 부품 (R4): 발판 = 얇은 판(충돌 없음) + 몸 중심의 xy가 판 안이면 켜짐. 문 = 미닫이 판(slide 관절 + 위치 액추에이터), 연결된 발판이 켜지면 목표 위치를 열림으로. 골 = 반경 안이면 성공. 상태기계는 MuJoCo 데이터만 읽고 액추에이터 목표만 쓴다.
9. 방 (R5): room.yaml 스키마(방 크기, 벽, 부품 목록과 이름, `links: [{from: switch_a, to: door_b}]`, 무작위 범위). 이름 검증 실패는 `ok: false`. `MjSpec`로 몸 MJCF에 방을 붙이고, 위에서 본 PNG를 저장.
10. 점검 (R6): (a) 링크 그래프에서 골까지 가는 길에 있는 문마다 그 문을 여는 발판이 문 앞쪽에서 닿는지 확인. (b) 명령 0으로 10초. (c) 스크립트 해답: 다음 목표 방향 오차로 wz, 오차가 작으면 vx=0.3. 배치 20개 중 18개 이상 성공. 실패 예제 방 2개(문이 연결 없음, 발판이 벽 안).
11. 환경 (R7, R17): `PuzzleEnv`(10 Hz, 행동 (vx, vy, wz)를 [-1, 1]로 받아 몸의 `cmd_limits`로 늘림), 관측·보상을 등록 항목으로 조립, `info["reward_terms"]`. task.yaml 읽기와 `register_obs`, `register_reward`.
   - **PR B 구현 메모 (2026-11-11)** — 위 7–11단계와 달라진 곳:
     - 행동 매핑은 몸 어댑터에 둔다: `BodyAdapter.command_from_action(action)`. 기본은 `cmd_limits`로 선형(몸 무관). 약속: action[0] 앞(−1 = 가장 느림/멈춤, +1 = 가장 빠름), action[1] 옆, action[2] 회전(+ = 왼쪽). 마이크로덕은 데드존을 피하도록 덮어쓴다: a0 < −0.6 → 서기(vx 0), 이때 |a2| ≥ 0.5면 제자리 회전 wz ±1.0, 아니면 0; a0 ∈ [−0.6, 1] → vx 0.25–0.4 선형, wz = a2 × 1.0 선형(걸으면서는 모든 wz가 돈다); vy = a1 × 0.3. 뒤로 걷기는 쓰지 않는다. 근거 측정(5초, 평지): vx 0.25 → 0.12 m/s, 0.3 → 0.15, 0.4 → 0.22; vx ≥ 0.25에서 wz ±1이면 0.9–1.0 rad/s로 돌며 3–6 cm/s 전진; vx 0.4·wz 0은 −0.2 rad/s로 휜다; vx 0.2·wz 0.3은 제자리. 무작위 행동 20 에피소드에서 넘어짐 0.
     - 어댑터에 `footprint_radius`(마이크로덕 0.11 m) 추가: 스크립트 해답의 경로 여유.
     - 발판은 기본으로 잠긴다(`latch: true`, 한 번 켜지면 에피소드 끝까지 켜짐). 몸이 하나라 발판을 밟은 채 문을 지날 수 없기 때문이다. `latch: false`면 순간형이고, 그때 문은 발판이 꺼지면 닫힌다. 문은 `require: all|any`(연결된 발판 전부/하나).
     - 문은 world 몸과 충돌 제외(`exclude`)라 벽 안으로 미끄러져 들어간다. 몸과는 충돌한다.
     - (a) 풀이 순서: 링크 그래프만이 아니라 방 격자(벽, 닫힌 문)에서 스폰부터 flood fill → 닿는 발판 → 그 발판이 여는 문 → 반복, 골에 닿으면 성공. 몸 크기는 보지 않는다(부품 영역을 한 칸 넓혀 판정). 몸이 실제로 들어가는지는 (c)가 본다. (a)가 실패하면 (c)는 건너뛴다.
     - (c) 스크립트 해답: 벽을 `footprint_radius` + 2 cm 부풀린 격자에서 현재 목표까지 측지 거리장을 만들고 0.3 m 앞 경로점을 겨눈다. 방향 오차 < 0.4 rad면 a0 = 1(최고 속도), 아니면 a0 = 0(천천히 걸으며 회전, 마이크로덕 vx ≈ 0.31). a2 = clip(2.5 × 오차). PuzzleEnv를 그대로 돌리므로 환경도 함께 검증된다.
     - 진척 보상은 직선 거리가 아니라 측지 거리(벽을 돌아, 열린 문으로)의 감소다. 목표는 풀이 순서의 발판·골 차례.
     - 관측 항목 이름은 부품 종류에 묶지 않는다: `part_pos`(부품마다 몸 기준 xy 2), `part_state`(부품마다 0–1). 추가 항목 `target_pos`(현재 목표, 기본 꺼짐).
     - 느린 테스트(20 배치 전체 check)는 `@pytest.mark.slow`이고 기본 실행에서 빠진다(`pytest -m slow`로 실행).
12. 전체 속도 측정 (R13): `mpg bench`로 위층 스텝/초(단일, 병렬 N개). **관문**: 2시간에 닿는 스텝 수가 부족해 보이면 여기서 멈추고 사용자와 다시 정한다(주기, 병렬 수, 목표, 학습 시간).
   - 2026-11-11 측정(`docs/bench.md` 2차): 예제 방 PuzzleEnv 단일 576, 병렬 12 4,627 환경 스텝/초(gymnasium `AsyncVectorEnv`, 16은 4,077로 더 느림). 2시간 ≈ 3,330만 스텝 상한, PPO 업데이트를 빼면 2,000–2,500만. 구현자 판단: 80%에 충분할 가능성이 높아 관문 통과로 본다(데스크·사용자 확인 대기). `mpg bench`에 `--room`, `--no-env` 추가.

**PR C — 학습, 진단, 평가, 영상**
13. 알고리즘 (R8): `Trainer`(train, save, load, predict) + SB3 감싸기(PPO 기본, SAC, A2C). 진입점으로 사용자 알고리즘 등록, 가짜 알고리즘 테스트.
14. 학습 (R8): `SubprocVecEnv`, 체크포인트, `curves.png`, `summary.json`, 항목별 보상, `task.resolved.yaml`.
15. 진단 (R9): 네 분류 규칙과 근거 숫자, 실패 영상. 망가진 설정 3개로 테스트.
16. 평가 (R10)와 영상 (R11): 학습에 안 쓴 시드 범위, `mujoco.Renderer` offscreen MP4(CI는 `MUJOCO_GL=egl`).
17. 보상·하이퍼파라미터 조정 (R10): 성공률 80%까지. 조정 반복은 subagent, 최종 설정만 커밋.

**PR D — 에이전트 안내, 문서, CI, 끝에서 끝까지**
18. `AGENTS.md`(R12), `skills/mujoco-puzzle-gym/SKILL.md`(R18).
19. README 세 언어 + 배너 SVG 두 장(R14), `mpg demo`.
20. CI (R15): ubuntu, uv, 자산 받기 캐시, 빠른 테스트만, 배지.
21. 끝에서 끝까지 (R16)와 R18 새 작업자 시험: 이 대화를 모르는 subagent에게 저장소와 스킬만 주고 "발판 2개 → 문" 방을 만들게 한다. 작업 기록을 `docs/agent-trial/`에 남긴다.

## Risks
- **학습 속도 (가장 큼)**: 위층 1스텝에 물리 20스텝 + ONNX 5회. 2시간에 PPO가 필요한 스텝에 못 미칠 수 있다. 6단계에서 먼저 재고, 12단계 관문에서 멈춘다. 줄이는 방법 후보: 주기 낮추기(spec 허용 5–25 Hz), 병렬 늘리기, 방 무작위 범위 좁히기.
- **걷기 재현**: 관측 순서, 기본 자세, BAM 설정이 하나라도 다르면 오리가 넘어진다. `infer_policy.py`를 줄 단위로 대조하고 R3 테스트로 막는다.
- **걷기 명령의 실제 반응 (2026-11-11 측정)**: 앞으로 최대 약 0.23 m/s(명령 0.4), vx 0.1 이하는 제자리, 뒤로는 −0.2에서 제자리·−0.4에서 −0.2 m/s, 제자리 회전은 |wz| 0.5에서 0, 1.0에서 약 0.7–0.9 rad/s. PR B의 스크립트 해답(10단계 c)과 `PuzzleEnv` 행동 스케일은 이 데드존을 감안해야 한다. → PR B에서 처리: 11단계 아래 구현 메모의 행동 매핑.
- **방이 걷기를 망침**: 바닥 마찰·높이, 발판 두께가 걷기 정책 학습 조건과 다르면 넘어진다. 발판은 충돌 없는 판으로 두고 바닥은 상위 scene과 같은 값으로.
- **문 통과**: 문이 얇거나 빠르면 몸이 뚫고 지나갈 수 있다. R4 테스트로 확인.
- **보상 해킹**: 진척 보상만 크면 발판 근처를 맴돈다. R9 진단이 이 경우를 잡도록 만들어 둔다.
- **상위 변경**: 커밋·리비전·해시 고정. BAM 버전 고정.
- **라이선스**: 메시는 저장소에 넣지 않는다. 영상·README 그림의 메시 노출은 공개 전 다시 확인(spec Areas of concern).

## Proof
| R | Evidence |
|---|---|
| R1 | `pytest tests/test_assets.py` 통과; `mpg fetch-assets --json` → `"verified": true`, 두 번째 실행 `"downloaded": 0` |
| R2 | `pytest tests/test_boundary.py` 통과 |
| R3 | `pytest tests/test_walk.py` 통과 (vx=0.3 5초 → 0.5–1.1 m, 넘어짐 없음; 명령 0 → 0.1 m 미만) |
| R4 | `pytest tests/test_parts.py` 통과 |
| R5 | `mpg build examples/switch_door.yaml --json` → `ok: true` + MJCF·PNG 경로; 잘못된 링크 → `ok: false` + 이유 |
| R6 | `mpg check examples/switch_door.yaml --json` 세 항목 통과; 실패 예제 2개가 각각 (a), (c)에서 실패 |
| R7 | `check_env` 통과, `pytest tests/test_reward_terms.py` 통과 |
| R8 | `mpg train examples/task_ppo.yaml --minutes 5` 후 `runs/<id>/` 산출물 + 무작위 정책 대비 보상; `pytest tests/test_algo_plugin.py`; `--algo sac --minutes 1` 종료 코드 0 |
| R9 | `pytest tests/test_diagnose.py` 통과 (세 설정이 각각 보상 해킹, 탐색 부족, 몸 실패) |
| R10 | `mpg eval runs/<id> --episodes 100 --json` → `success_rate ≥ 0.8` |
| R11 | 맥과 CI에서 `mpg render` MP4 생성, 길이 > 0 |
| R12 | `pytest tests/test_cli_json.py` 통과; `AGENTS.md` 존재 |
| R13 | `docs/bench.md`에 맥 스튜디오 측정값과 2시간 추정, 관문 결정 기록 |
| R14 | 빈 디렉터리 clone → 세 줄 실행 기록; 세 README 섹션 수 일치; 밝은/어두운 배너 스크린샷 |
| R15 | Actions 녹색 실행 링크, README 배지 |
| R16 | 실행 로그 + `runs/<id>/summary.json` + MP4 |
| R17 | `pytest tests/test_registry.py` 통과; `runs/<id>/task.resolved.yaml` 존재 |
| R18 | `docs/agent-trial/` 작업 기록에 새 room.yaml과 `mpg check` 통과 출력 |
