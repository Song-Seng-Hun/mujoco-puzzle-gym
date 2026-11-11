# Plan: 퍼즐 방과 정책 공유 (community sharing)
Status: approved 2026-11-11 (사용자 "승인": onnx 의존성, 실패 확인용 임시 커밋 포함)
Spec: docs/specs/2026-11-11-community-sharing/spec.md (approved 2026-11-11)
Branch: feat/community (main 9420f09에서). PR 하나에 spec.md, plan.md, 코드, 문서를 함께 넣는다.

## Decisions (spec Open questions)
1. 제한값은 제안값(부품 64개, 한 변 10 m, 글자 지도 40×40)을 그대로 쓴다. 지금 예제 방 최대치는 부품 6개(coin_room, doors), 한 변 5.12 m(broken/room_far_switch), 지도 26×14(two_rooms_map)다. 모두 제한의 절반 아래다.
2. task 해시는 `task.resolved.yaml` 파일 전체의 SHA256이다. `policy.json`에 어느 task로 학습했는지 남기는 기록이고, 평가 때 비교하지 않는다. 정책을 쓸 수 있는지는 관측 차원과 행동 차원으로 판단하고, 다르면 거절한다. 차원은 같은데 관측 항목 이름이나 크기가 다르면 경고한다. 관측·행동 키만 골라 해시하면 어느 키가 관측에 영향을 주는지 목록을 따로 관리해야 하고, 그 목록이 틀리면 조용히 틀린 결과가 나온다.
   - 구현 중 변경(2026-11-11): 처음엔 "해시가 다르면 경고"였다. 그런데 평가 때 받는 것은 사용자의 task.yaml이고, run이 저장한 resolved 파일과는 늘 해시가 달라 경고가 매번 뜬다. 그래서 해시 비교를 빼고 관측 항목 비교로 바꿨다.

## Order and files

### Step 1. R6 pickle 경로 제한 (먼저: 나머지가 이 함수를 쓴다)
- 새 파일 `src/mujoco_puzzle_gym/trust.py`
  - `require_trusted(path, what)`: 경로를 resolve해서 아래 둘 중 하나의 안이면 통과한다.
    - `Path.cwd() / "runs"`
    - 저장소에 들어 있는 데모 run `examples/pretrained/` (`mpg demo`가 읽는다)
  - 환경 변수 `MPG_TRUST=1`이면 통과한다. `--trust`가 이 변수를 켠다. 학습과 평가의 작업 프로세스도 같은 변수를 물려받으므로 따로 넘기지 않아도 된다.
  - 거절 메시지: "<what> <path>는 runs/ 밖에 있어요. pickle 파일은 열 때 코드를 실행할 수 있으니 직접 만든 파일만 여세요. 믿을 수 있으면 --trust를 붙이세요."
- 막는 곳
  - `train.py` `load_run`: summary를 읽기 전, 즉 `load_task`(플러그인 import)보다 먼저 막는다.
  - `curriculum.py` `load_snapshots`: `snapshot_dir`가 정한 폴더를 검사한다. task.yaml의 `curriculum.snapshots`가 바깥을 가리키는 경우도 여기서 걸린다.
- `cli.py`: eval, diagnose, render, train에 `--trust`를 더한다.
- 테스트 영향: 지금 테스트 여러 개가 `tmp_path`에 학습한 run을 읽는다(test_train, test_eval_full, test_diagnose 등). `tests/conftest.py`에 autouse fixture로 `MPG_TRUST=1`을 켜 두고, `tests/test_trust.py`만 이 변수를 지워서 시험한다.
- `tests/test_trust.py`
  - 바깥 run 폴더 거절, `--trust`(환경 변수)로 통과, `runs/` 아래 통과
  - 바깥을 가리키는 `curriculum.snapshots` 거절
  - `mpg eval <바깥 경로>`가 JSON 오류를 내고, 메시지에 "pickle"이 들어 있음

### Step 2. R2 `mpg check --community`
- 새 파일 `src/mujoco_puzzle_gym/community.py`
  - `check_raw(path)`: 방을 불러오기 전에 YAML만 보고 막는다.
    - 최상위 `plugins` 키
    - `stages:` 경로가 방 파일 폴더 밖을 가리킴 (resolve한 뒤 비교, `..`과 절대 경로 포함)
    - 글자 지도가 40×40칸을 넘음
  - `check_loaded(room)`: 불러온 뒤에 막는다.
    - 부품 종류가 내장이 아님 (`get_part(type).__module__`이 `mujoco_puzzle_gym.`으로 시작하지 않음)
    - 부품이 64개를 넘음 (stages 안의 부품 포함)
    - 방 한 변이 10 m를 넘음 (`extent()`, stages 포함)
- `check.py` `check(..., community=False)`: 켜면 결과에 `community` 단계를 더하고, 실패하면 시뮬레이션 단계는 건너뛴다. 방식은 지금 static 단계 실패 처리와 같다.
- `cli.py`: `check`에 `--community`를 더한다. `--community`와 `--task`를 함께 쓰면 거절한다. task.yaml은 `plugins:`로 코드를 불러올 수 있기 때문이다.
- `tests/test_community.py`: 거절 사례 6개(위 6가지)를 tmp 방 파일로 하나씩 확인하고, `examples/rooms/*.yaml`이 모두 `community` 단계를 통과하는지 확인한다.

### Step 3. R1 + R3 CI 잡 `community`
- 새 파일 `tools/check_community_files.py` (표준 라이브러리만 씀)
  - 바뀐 파일 목록을 받아서 아래를 확인한다.
    - 위치가 `community/rooms/<이름>/<파일>` 형태인지
    - 확장자가 `.yaml` 또는 `README.md`인지
    - 크기가 64 KB 이하인지
  - 위반을 줄마다 출력하고 exit 1로 끝난다.
- `.github/workflows/ci.yml`에 잡 `community`를 더한다.
  - 조건: `if: github.event_name == 'pull_request'`
  - 체크아웃: `fetch-depth: 0`
  - 바뀐 파일 구하기: `git diff --name-only --diff-filter=AMR origin/$BASE...HEAD -- community/`
  - 바뀐 파일이 없으면 첫 단계에서 바로 성공으로 끝낸다. 워크플로 전체의 `paths` 필터는 쓰지 않는다. 다른 잡까지 꺼지기 때문이다.
  - 파일 검사 → OSMesa, uv 설치, 자산 받기(지금 `tests` 잡과 같음) → 방마다 `timeout 600 uv run mpg check --community --seeds 20 --json <방>`을 돌리고, 결과의 `ok`가 하나라도 false면 실패한다.
  - `timeout-minutes: 30`. 권한은 워크플로 최상위의 `contents: read`를 그대로 쓴다.
- 예제 제출 `community/rooms/Song-Seng-Hun/`: `README.md`와 방 하나(switch_door 변형)를 넣는다. 이 PR에서 통과하는 경로를 CI로 증명하는 용도이고, 새 사용자가 보고 따라 할 본보기다.
- `tests/test_community.py`에 `check_community_files.py`의 거절 사례(확장자, 크기, 폴더 깊이)를 더한다.

### Step 4. R5 ONNX 내보내기와 평가
- 의존성: `torch.onnx.export`는 `onnx` 패키지가 필요하다. 이 Mac의 venv에서 직접 확인했다. dynamo=False면 "Module onnx is not installed!", dynamo=True면 `onnxscript`가 없다는 오류가 난다.
  - `onnx`를 선택 설치 묶음 `[project.optional-dependencies] export = ["onnx"]`와 dev 그룹에 더한다. 그리고 `uv lock`을 다시 만든다.
  - 내보내기는 dynamo=False로 한다. 그러면 `onnxscript`는 필요 없다.
  - 평가 쪽은 이미 들어 있는 onnxruntime만 쓴다.
- 새 파일 `src/mujoco_puzzle_gym/onnx_policy.py`
  - `export(run_dir, out_dir)`
    - `load_trainer`로 SB3 모델을 연다. Step 1의 경로 검사를 그대로 거친다.
    - 정해진 행동(평균)을 내는 부분만 감싸서 내보낸다.
      - PPO, A2C: `policy(obs, deterministic=True)`
      - SAC: `policy.actor(obs, deterministic=True)`
    - 관측 공간이 Dict(카메라)면 거절한다.
    - `policy.json`을 쓴다: 관측 항목 이름과 차원, 관측 전체 차원, 행동 차원과 low/high, 알고리즘, PPO·A2C의 clip 여부, SAC의 unscale 여부, task 해시, 방 이름.
  - `OnnxPolicy(path)`: onnxruntime으로 돌리고, SB3 `predict`와 같은 후처리를 한다. PPO·A2C는 행동 범위로 clip하고, SAC는 [-1, 1]에서 행동 범위로 되돌린다.
- `evaluate.py`: 지금 `run_jobs`와 `evaluate`는 run 폴더만 받는다. 정책을 만드는 함수와 task를 받도록 조금 고친다. `--onnx` 경로에서는 pickle을 전혀 읽지 않는다.
- `cli.py`
  - `mpg export-onnx runs/<id> [--out 폴더]`: 기본 출력 폴더는 `runs/<id>/onnx/`
  - `mpg eval --onnx <file> --task <task.yaml> [--episodes] [--workers]`: `run`은 선택 인자가 된다. `--onnx`와 `run`은 둘 중 하나만 받는다.
  - 관측 차원이나 행동 차원이 다르면 거절한다. 관측 항목 이름이나 크기가 다르면 경고만 한다.
- `tests/test_onnx.py`
  - 짧게 학습(max-steps 몇백)한 PPO와 SAC를 내보낸다.
  - 무작위 관측 100개에 대해 SB3 `predict`와 ONNX 예측의 차이가 1e-5 안인지 확인한다.
  - `mpg eval --onnx`가 2에피소드를 돌고 JSON을 내는지 확인한다.
  - Dict 관측 거절을 확인한다.
  - 시간이 들면 slow로 표시한다. 그러면 PPO 하나는 fast에 남긴다.

### Step 5. R7 문서
- `docs/community.md`를 한국어로 쓰고, 영어(`.en.md`)와 일본어(`.ja.md`)로 옮긴다. 번역은 E1 때처럼 서브에이전트에 맡긴다.
  - 방 올리는 법: 폴더, `mpg check --community`, PR
  - 받는 것과 안 받는 것: 방 YAML만 받는다. 플러그인, pkl, zip은 받지 않는다. 정책은 ONNX 링크로 공유한다.
  - ONNX로 공유하고 받는 법: `export-onnx`, `eval --onnx`
  - 다른 사람의 run 폴더나 pkl을 열지 말라는 경고와 그 이유
- README 세 언어에 링크 한 줄씩을 더한다.
- `.github/pull_request_template.md`에 방 제출 점검표를 넣는다.
- `docs/guide.md`(와 en, ja)의 eval 명령 표에 `--onnx`, `--trust`, `export-onnx`를 더한다.

## Risks
- `--trust`를 환경 변수로 넘기므로 셸에서 `MPG_TRUST=1`을 켜 두면 늘 통과한다. 이건 사용자가 직접 켜는 것이라 막지 않고, 문서에만 적는다.
- 방 PR이 `community/` 밖의 코드(검사기 자체)도 함께 고치면 CI가 고친 검사기로 돈다. 이건 막지 않고 리뷰에서 본다. PR 템플릿 점검표에 "community/ 밖 파일 없음"을 넣는다.
- R3의 실패 경로 증명: spec은 "실패하는 방 PR의 CI 결과"를 요구한다. 이 PR 안에서 일부러 나쁜 방을 넣은 커밋을 한 번 올려 CI 실패를 확인하고, 다음 커밋에서 지운다. 따로 PR을 만들지 않아서 남는 브랜치가 없다. 대신 커밋 2개가 기록에 남는다.
- SB3 정책 내부 구조(`policy.actor` 등)는 SB3 버전에 따라 바뀔 수 있다. test_onnx가 예측 일치를 직접 비교하므로, 바뀌면 테스트가 잡는다.
- 새 의존성 `onnx`가 추가된다(선택 설치 묶음과 dev 그룹). `uv lock`과 설치가 필요하므로, 이 계획을 승인하면 그것까지 포함해 승인하는 것으로 본다.

## Proof per R
| R | 증거 |
|---|---|
| R1 | `tests/test_community.py`의 파일 검사 사례 + 이 PR에서 일부러 넣은 나쁜 파일 커밋의 CI 실패 기록 |
| R2 | `tests/test_community.py`의 거절 6가지 + 예제 방 통과, `mpg check --community examples/rooms/doors.yaml --json` 출력 |
| R3 | 이 PR의 CI: 예제 제출 방으로 `community` 잡 통과 1회, 나쁜 방 커밋으로 실패 1회. community/를 안 바꾸는 PR에서는 첫 단계에서 끝나는 기록 |
| R4 | 머지 후 따로 승인을 받고 저장소 설정을 바꾼다. Discussions 탭 화면으로 확인 |
| R5 | `tests/test_onnx.py`(1e-5 일치, eval --onnx, Dict 거절) |
| R6 | `tests/test_trust.py` |
| R7 | 세 언어 community 문서의 절 수 일치(스크립트로 `^#` 개수 비교), README 세 곳 링크 |

전체 확인: fast 테스트, `-m ''` 전체 테스트, 예제 방 `mpg check --seeds 20` 결과가 지금(11개 20/20, color_sort 19/20)과 같은지.

## Who does what
- 데스크(이 세션)가 Step 1~4를 직접 구현한다. 보안 경계라서 맥락을 가진 쪽이 쓴다.
- Step 5의 영어·일본어 번역은 서브에이전트에 맡긴다.
- 머지 전에 claude-mac-reviewer로 diff를 리뷰한다. 확인할 점: 보안 경계, 우회 경로, CI 권한.
