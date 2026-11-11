# Spec: 퍼즐 방과 정책 공유 (community sharing)
Status: approved 2026-11-11 (사용자 "승인", R1-R7; Open questions 1-2는 구현 때 정함)
View (read-only copy): (없음)

근거: 사용자 2026-11-11 "서로 만든 퍼즐과 파라미터 공개하고 노는 공간 있었으면 좋았을텐데", "신호변환기 같은 부분에 코드 넣을만한 공간 있는거 아냐? yaml이라고 안전하나", 인터뷰 답(받는 경로 "PR + Discussions", 정책 "링크만, ONNX", pickle "경로 제한 + 경고").

사실(코드, main 9420f09):
- 방 YAML은 `yaml.safe_load`로 읽고, 논리식은 `expr.py`가 ast 허용 목록으로 해석한다(`eval` 없음). 방 파일에는 코드를 넣을 자리가 없다.
- 코드를 실행할 수 있는 곳은 셋이다. task.yaml `plugins:`(`task.py:100` `importlib.import_module`), 체크포인트 저장본(`curriculum.py:134` `pickle.loads`), SB3 체크포인트 zip(`train.py:377` `cls.load`, 안에 pickle).
- 방 파일의 `stages:`는 다른 방 파일 경로를 받는다(`room.py:293`, 방 파일 기준 상대 경로, 바깥으로 나가는 경로를 막지 않음).
- `onnxruntime`은 이미 의존성이다(걷기 정책 `velstand.onnx`).

## Requirements
- R1: 공유 폴더 `community/rooms/<GitHub 사용자 이름>/`. 받는 파일은 방 파일 `*.yaml`과 설명 `README.md`뿐이고 파일당 64 KB 이하다. Check: CI 잡이 다른 확장자나 큰 파일이 든 PR을 실패시킨다(테스트 PR 기록).
- R2: `mpg check --community`가 정적 단계 `community`를 더한다. 거절: 내장이 아닌 부품 종류(플러그인 부품), `stages:`가 방 파일 폴더 밖을 가리킴, 부품 64개 초과, 방 한 변 10 m 초과, 글자 지도 40×40 칸 초과. 방 YAML에 `plugins` 키가 있으면 거절한다. Check: `tests/test_community.py`가 각 거절 사례와 예제 방 통과를 확인한다.
- R3: CI 잡 `community`: `community/**`가 바뀐 PR에서만 돈다. 바뀐 방마다 `mpg check --community --seeds 20 --json`을 방당 10분, 잡 전체 30분 제한으로 돌리고, 모두 `ok: true`여야 통과한다. 잡 권한은 `contents: read`, PR 코드의 스크립트는 실행하지 않는다(`pull_request` 트리거, `pull_request_target` 안 씀). Check: 통과하는 방과 실패하는 방 PR 각각의 CI 결과.
- R4: GitHub Discussions를 켜고 분류 `Rooms`(만든 방 자랑과 링크), `Q&A`, `Ideas`를 둔다. Check: 저장소 Discussions 탭에 세 분류가 보인다. (저장소 설정 변경이라 실행 때 사용자 승인.)
- R5: 정책은 저장소에 올리지 않는다. 공유는 외부 링크(예: Hugging Face)와 ONNX 형식만 안내한다. `mpg export-onnx runs/<id>`가 `policy.onnx`와 `policy.json`(관측 항목과 차원, 행동 범위, task 해시, 방 이름)을 만든다. `mpg eval --onnx <file> --task <task.yaml>`이 pickle 없이 onnxruntime으로 평가한다. 벡터 관측(Box)만 지원하고, 카메라(Dict) 관측은 거절한다. Check: `tests/test_onnx.py`가 짧게 학습한 정책을 내보내 SB3 예측과 ONNX 예측이 1e-5 안에서 같음을 확인하고, `mpg eval --onnx`가 돈다.
- R6: pickle을 읽는 두 곳에 경로 제한을 둔다. 체크포인트 저장본과 SB3 run 폴더는 현재 폴더의 `runs/` 아래에 있을 때만 읽는다. 그 밖의 경로는 `--trust`(task.yaml `curriculum.snapshots`가 바깥을 가리킬 때도 같음)가 없으면 이유("pickle 파일은 코드를 실행할 수 있음, 직접 만든 파일만 여세요")와 함께 거절한다. Check: `tests/test_trust.py`가 바깥 경로 거절, `--trust`로 통과, `runs/` 아래 통과를 확인한다.
- R7: 문서 `docs/community.md`(+ en, ja): 방 올리는 법(폴더, `mpg check --community`, PR), 받는 것과 안 받는 것(방 YAML만, 플러그인·pkl·zip 금지, 정책은 ONNX 링크), 다른 사람 run 폴더나 pkl을 열지 말라는 경고. README에 한 줄 링크. PR 템플릿에 방 제출 점검표. Check: 세 언어 파일의 절 수 일치, README 링크.

## Out of scope
- 방 순위표, 점수 서버, 웹 갤러리.
- 카메라 관측 정책의 ONNX 내보내기.
- 플러그인 부품 공유(코드 검토가 필요한 일).
- 받은 방의 자동 머지. 머지는 사용자가 한다.

## Open questions
1. R2의 제한값(부품 64개, 한 변 10 m, 지도 40×40)은 제안값이다. 지금 예제 방 최대치로 확인한 뒤 정한다.
2. R5의 task 해시: `task.resolved.yaml` 전체의 SHA256으로 할지, 관측·행동 관련 키만으로 할지.
