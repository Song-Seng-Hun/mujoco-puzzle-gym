# 방과 정책 나누기

직접 만든 퍼즐 방은 이 저장소에 PR로 올리고, 학습한 정책은 ONNX 파일로 바꿔 외부 링크로 나눕니다. 이야기는 GitHub Discussions에서 합니다.

## 받는 것과 받지 않는 것

| 받는 것 | 받지 않는 것 |
|---|---|
| 방 파일 `*.yaml` (파일당 64 KB 이하) | 파이썬 파일, 플러그인 부품 |
| 폴더 설명 `README.md` | `*.pkl`, `*.zip`, 체크포인트, run 폴더 |
| 정책의 외부 링크(ONNX) | 저장소에 직접 올리는 정책 파일 |

방 파일은 코드를 담을 수 없습니다. YAML은 `yaml.safe_load`로 읽고, 논리식은 정해진 연산만 허용하는 해석기로 계산합니다. 그래서 받는 것을 방 파일로 한정합니다.

## 방 올리기

1. 저장소를 포크하고 `community/rooms/<GitHub 사용자 이름>/` 폴더를 만듭니다. 하위 폴더는 쓰지 않습니다.
2. 방 파일과, 원하면 `README.md`를 넣습니다. 본보기는 `community/rooms/Song-Seng-Hun/`에 있습니다.
3. 올리기 전에 직접 점검합니다.

   ```bash
   uv run mpg check --community --seeds 20 --json community/rooms/<이름>/<방>.yaml
   ```

   결과가 `"ok": true`여야 합니다.
4. PR을 엽니다. PR에는 내 폴더 `community/rooms/<내 이름>/` 안의 파일만 넣습니다. 다른 사람의 폴더나 `community/` 밖의 파일이 섞이면 CI가 실패합니다. CI는 PR 안의 검사 코드로 돌기 때문에, 방 PR이 검사 코드를 함께 고치지 못하게 막는 것입니다. PR 템플릿의 점검표를 채워 주세요.

PR이 열리면 CI 잡 `community`가 바뀐 방마다 같은 점검을 돌립니다(방당 10분, 전체 30분). 모두 통과하면 관리자가 읽어 보고 머지합니다. 자동 머지는 하지 않습니다.

## `--community` 점검이 거절하는 것

| 거절 | 이유 |
|---|---|
| 방 파일의 `plugins` 키 | 플러그인 모듈은 코드를 실행합니다 |
| 방 파일 폴더 밖을 가리키는 `stages:` 경로 | 다른 곳의 파일을 읽지 않게 합니다 |
| 내장이 아닌 부품 종류 | 플러그인 부품은 코드 검토가 필요합니다 |
| 부품 64개 초과 | CI 시간과 학습 비용을 묶어 둡니다 |
| 방 한 변 10 m 초과 | 같은 이유 |
| 글자 지도 한 변 40칸 초과 | 같은 이유 |

`--community`는 `--task`와 함께 쓸 수 없습니다. task 파일의 `plugins:`가 코드를 불러올 수 있기 때문입니다. 점검은 내장 task로 합니다.

## 정책 나누기 (ONNX)

정책은 저장소에 올리지 않습니다. 대신 ONNX로 바꿔 Hugging Face 같은 곳에 올리고, Discussions의 `Rooms` 분류에 링크를 남깁니다.

```bash
uv pip install 'mujoco-puzzle-gym[export]'
uv run mpg export-onnx runs/<id>
```

`runs/<id>/onnx/`에 두 파일이 생깁니다.

- `policy.onnx`: 결정적(deterministic) 행동을 내는 정책 신경망
- `policy.json`: 관측 항목과 크기, 행동 범위, 학습한 task의 SHA256, 방 이름

벡터 관측 정책(PPO, A2C, SAC)만 내보낼 수 있습니다. 카메라 관측 정책은 아직 안 됩니다.

받은 정책은 pickle 없이 평가합니다. 두 파일을 같은 폴더에 두고, 그 정책을 학습한 방을 쓰는 내 task 파일을 `--task`로 줍니다.

```bash
uv run mpg eval --onnx policy.onnx --task task.yaml --episodes 100
```

관측 크기나 행동 크기가 task와 다르면 평가를 거절합니다. 크기는 같은데 관측 항목이 다르면 결과에 `warnings`가 붙습니다.

## 다른 사람의 run 폴더와 pkl을 열지 마세요

SB3 체크포인트(`checkpoint.zip`), 체크포인트 저장본(`*.pkl`), run 폴더의 `task.resolved.yaml`은 열 때 코드를 실행할 수 있습니다. 그래서 `mpg`는 이 파일들을 현재 폴더의 `runs/` 아래와 저장소에 든 데모에서만 읽습니다. 그 밖의 경로는 이유와 함께 거절합니다.

직접 만든 파일을 다른 곳에 옮겨 두었을 때만 `--trust`를 붙입니다(`train`, `eval`, `diagnose`, `render`, `export-onnx`). `--trust`는 그 명령이 도는 동안 환경 변수 `MPG_TRUST=1`을 켭니다. 셸에서 이 변수를 늘 켜 두면 경로 검사가 꺼진 상태가 되니 그렇게 하지 마세요.

## Discussions

- `Rooms`: 만든 방 자랑, 정책 링크
- `Q&A`: 질문
- `Ideas`: 새 부품이나 기능 제안
