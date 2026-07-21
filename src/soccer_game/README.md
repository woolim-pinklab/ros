# 🐢⚽ soccer_game — 여럿이 하는 turtle soccer

turtlesim 없이 **심판 노드가 직접 물리를 계산**하는 헤드리스 축구.
화면은 브라우저 **three.js 3D 뷰어** 하나뿐이다.

**같은 `ROS_DOMAIN_ID` 를 쓰는 사람들이 각자 이름을 적고 참가**하면 같은 경기장에서 함께 뛴다.
처음 켜면 경기장과 골대만 있고, 참가하는 사람마다 머리에 **이름표를 단 거북이**가 생긴다.

## ROS2 통신 3종을 용도대로 쓴다

| 통신 | 이름 | 왜 이걸 쓰나 |
|---|---|---|
| **토픽** | `/world_state` (`soccer_msgs/WorldState`) | 경기 상황이 20Hz로 계속 흘러나온다 — 계속 흐르는 데이터라 토픽 |
| **서비스** | `/register`, `/leave`, `/move_to` | 요청하면 응답이 오고 끝나는 일회성 작업 |
| **액션** | `/shoot` (`soccer_msgs/Shoot`) | 시간이 걸리고 진행 상황(정지 카운트다운)을 보고해야 하니 액션 |

## 팀

참가하면 **인원이 적은 팀으로 자동 배정**된다(같으면 blue).

골대는 **지키는 팀의 것**이다. 상대 골대에 넣어야 득점한다.

| 골대 | 주인(지키는 팀) | 색 | 키퍼 | 여기에 넣으면 득점 |
|---|---|---|---|---|
| **왼쪽** (`goal_left_x`) | red | 빨강 | red 키퍼 | **blue** |
| **오른쪽** (`goal_right_x`) | blue | 파랑 | blue 키퍼 | **red** |

| 팀 | 공략(= 상대 골대) | 지키는 골대 | 시작 진영 |
|---|---|---|---|
| **blue** | **왼쪽** (red 골대) | 오른쪽 | 오른쪽 (x≈12.5) |
| **red** | **오른쪽** (blue 골대) | 왼쪽 | 왼쪽 (x≈3.5) |

각 골대 앞에는 **자동으로 위아래를 왕복하는 키퍼**가 한 명씩 있다.
점수는 심판이 집계하고, 골이 들어가면 **공은 가운데에서 다시 시작**한다.
뷰어 왼쪽/오른쪽 패널에 **팀별 인원과 명단(개인 득점 포함)** 이 나온다.

## 빌드 & 실행

```bash
cd ~/dev_ws/ros
colcon build --packages-select soccer_msgs soccer_game
source install/setup.bash
```
> 새 터미널마다 `source install/setup.bash` 를 해줘야 `soccer_msgs` 타입이 잡힌다.

### 심판은 한 머신에서 한 명만

```bash
ros2 launch soccer_game soccer.launch.py     # 심판 + 뷰어
```
- 브라우저에 3D 경기장이 자동으로 열린다. 자동 오픈을 끄려면 `open_ui:=false`.
- 참가자 전원이 심판과 **같은 `ROS_DOMAIN_ID`** 를 써야 한다: `export ROS_DOMAIN_ID=42`

> ⚠️ **심판을 두 대 이상 띄우면 안 된다.** 다른 머신이면 포트 충돌도 안 나서
> 겉보기엔 멀쩡하지만, 실제로는 이렇게 깨진다:
> - `/register` 요청 하나가 **두 심판 모두에게** 전달된다(각자 따로 참가 처리)
> - 두 심판이 물리를 **독립적으로** 돌려 공·키퍼·점수가 곧 달라진다
> - `/world_state` 에 두 심판의 상태가 **번갈아 섞여** 들어온다
>
> 즉 사람마다 다른 경기를 보게 된다. 심판은 반드시 한 명만 띄우자.

### 참가자 (심판 안 띄움)

```bash
export ROS_DOMAIN_ID=42
source ~/dev_ws/ros/install/setup.bash
ros2 service call /register soccer_msgs/srv/Register "{name: '내이름'}"
```

### 관전만 하기 — 심판과 UI 분리

뷰어는 그냥 HTML 파일이라 심판과 **다른 머신에서 얼마든지 띄워도 된다**(관전은 몇 명이든 OK).
심판 머신의 IP만 알려주면 된다:

```bash
ros2 launch soccer_game viewer.launch.py host:=192.168.0.10
```

브라우저에서 직접 열어도 된다 — 주소 끝에 `?host=` 를 붙인다:

```
file://.../share/soccer_game/viewer/index.html?host=192.168.0.10
```
`?host=` 를 안 붙이면 이 머신(localhost)의 심판을 찾는다.
연결이 안 되면 화면 왼쪽 아래에 시도 중인 주소가 표시된다.

> 뷰어는 three.js 를 CDN(jsdelivr)에서 불러오므로 브라우저에 인터넷 연결이 필요하다.

## 플레이

### ① 참가 — 이름은 필수다

```bash
ros2 service call /register soccer_msgs/srv/Register "{name: '내이름'}"
```
`name` 이 비어 있으면 참가가 **거부**된다. 이미 쓰는 이름도 안 된다(최대 12자).
응답에 배정된 팀, 공략할 골대, 그리고 **비밀 토큰**이 나온다.

```
ok=True, token='33775de655fc0372',
message='내이름 → blue 팀 배정! 왼쪽 골대를 공략하자. ...'
```

> **토큰을 잘 보관하자.** 이후 `/move_to`, `/shoot`, `/leave` 에 이 토큰을 같이 보내야
> 내 거북이를 조종할 수 있다. 남이 내 이름을 흉내내도 토큰이 없으면 거부된다.

### ② 상황 파악 — 토픽

```bash
ros2 topic echo /world_state
```
여기서 공 위치·속도, 양쪽 골대, 경기장 경계, 점수, 그리고 **모든 선수의 이름·팀·위치**가 나온다.
내 위치 / 우리 편 / 상대편은 이름과 팀으로 걸러서 알아낸다:

```python
me       = next(p for p in msg.players if p.name == 내이름)
teammate = [p for p in msg.players if p.team == me.team and p.name != me.name]
opponent = [p for p in msg.players if p.team != me.team]
goal_x   = msg.goal_left_x if me.team == 'blue' else msg.goal_right_x
```

### ③ 이동 — 서비스

```bash
ros2 service call /move_to soccer_msgs/srv/MoveTo \
  "{name: '내이름', token: '받은토큰', x: 8.0, y: 5.5}"
```
목표 지점까지 걸어가고 **도착하면 응답**한다. 경기장 밖 좌표는 안쪽으로 잘린다.

### ④ 슛 — 액션

```bash
ros2 action send_goal /shoot soccer_msgs/action/Shoot \
  "{name: '내이름', token: '받은토큰', power: 1.0}" --feedback
```
- **공은 "나 → 공" 방향으로 날아간다.** 어느 쪽에서 다가가느냐가 곧 조준이다.
  골대 반대편에 서서 차면 골대 쪽으로 간다.
- 공이 **사거리(1.6) 밖**이면 헛발질(`kicked: false`)이고 정지도 없다.
- 차고 나면 **1초간 정지**한다. 그동안 `freeze_left` 가 feedback 으로 흘러나오고,
  뷰어에서는 그 거북이가 **반투명해지며 머리 위에 남은 시간이 링 게이지로** 표시된다.
- 결과: `kicked` / `goal` / `blocked` / `message`.

### ⑤ 퇴장

```bash
ros2 service call /leave soccer_msgs/srv/Register "{name: '내이름', token: '받은토큰'}"
```

## 내 거북이만 조종하기 (토큰)

ROS2(DDS)에는 기본 인증이 없다. 같은 도메인에 있으면 **누구나 아무 이름으로** 서비스를
호출할 수 있으므로, 요청의 `name` 만으로는 본인임을 증명하지 못한다.

그래서 참가할 때 심판이 **비밀 토큰**을 발급하고, 조종 요청(`/move_to`, `/shoot`, `/leave`)마다
이름과 토큰이 맞는지 확인한다. 틀리면 거부하고 심판 로그에 경고를 남긴다.

```
message='토큰이 틀렸다. 남의 거북이는 조종할 수 없다.'
```

> 이 방식은 **남의 이름을 타이핑하는 수준의 장난을 막는다.** 다만 같은 도메인에서
> DDS 트래픽을 직접 들여다보면 토큰이 보인다. 진짜 인증이 필요하면 **SROS2**
> (참가자별 인증서 + 접근제어)를 써야 하는데, 키스토어 발급·배포가 필요해 부담이 크다.
>
> 같은 이유로 **기기당 1명 제한은 신뢰성 있게 강제할 수 없다** — DDS 요청에는 발신 기기
> 정보가 실려 오지 않는다. 굳이 한다면 클라이언트가 기기 식별자를 계산해 보내는
> best-effort 방식뿐이고, 마음먹으면 우회된다.

## 공 물리

슛은 공에 **초기 속도만** 주고, 그 뒤는 물리 타이머(50Hz)가 굴린다.

- **마찰** — `FRICTION`(1.5 units/s²) 으로 감속하다 `STOP_SPEED` 아래면 멈춘다.
- **반사** — 법선 방향으로 튕긴다. 빗맞으면 비스듬히 튄다.

| 부딪히는 대상 | 반발 계수 |
|---|---|
| 키퍼 | 0.85 |
| 사람 거북이 | 0.75 |
| 골포스트 | 0.7 |
| 벽 | 0.6 |

- **거북이끼리는 서로 밀어낸다** — 겹치면 절반씩 밀려난다(키퍼는 안 밀린다).
- **움직이는 거북이가 공에 닿으면 그 속도로 공이 밀려 나간다** — 몸으로 드리블할 수 있다.
  단 몸에 닿는 거리는 0.8 이므로, 조준해서 차려면 공에서 **0.9~1.6 사이**에 서는 게 좋다.

## 경기장 좌표

축구장처럼 가로로 긴 직사각형이다.

- x `0.5 ~ 15.5`, y `0.5 ~ 10.5`, 가운데 `(8.0, 5.5)`
- 오른쪽 골대 라인 x=15.8, 왼쪽 골대 라인 x=0.2
- 골문은 양쪽 다 y `3.7 ~ 7.3` (중심 5.5, 반폭 1.8)
- 키퍼는 x=15.0 / x=1.0 에서 y `3.7 ~ 7.3` 왕복

값을 바꿔 난이도·손맛을 조절할 수 있다(`soccer_referee.py` 상단 상수):
`KEEPER_SPEED` ↑ 더 어려움 · `FRICTION` ↓ 공이 더 멀리 구름 · `KICK_RANGE` ↑ 차기 쉬움.


## 테스트

### 물리 검증 (심판 안 띄우고, 몇 초)

```bash
python3 src/soccer_game/test/test_physics.py    # pytest 로도 실행 가능
```
벽 반사·양쪽 골 판정·골포스트/키퍼 충돌·마찰·거북이끼리 반발·거북이와 공 충돌·
골 후 리스폰·팀 방향 계약까지 22개 항목을 확인한다. 실제 경기에서는 우연히만 나오는 경로를
결정적으로 검사하므로, 물리를 손댔으면 이걸 먼저 돌리자.

### 예제 봇으로 혼자 멀티플레이 시험

```bash
ros2 run soccer_game example_bot --name 봇하나 --rounds 10
ros2 run soccer_game example_bot --name 봇둘  --rounds 10   # 다른 터미널
```
토픽으로 상황을 읽고 서비스로 이동해 액션으로 차는 참가자다.
팀 자동 배정이라 봇 2개면 blue vs red 가 된다.
`example_bot.py` 의 `pick_target()` 만 고치면 자기 전략이 되므로, 학생 과제 틀로도 쓸 수 있다.
