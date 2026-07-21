"""초간단 축구 심판 (헤드리스 + three.js 뷰어, 여러 명 참가 / 팀 대전).

turtlesim 없이 심판이 직접 물리를 계산한다. 상태(선수들/공/키퍼/점수)는
표준 라이브러리만으로 만든 WebSocket 서버로 20Hz JSON 방송하고,
브라우저 three.js 뷰어(viewer/index.html)가 받아 3D로 그린다.

같은 ROS_DOMAIN_ID 를 쓰는 사람들이 각자 이름을 적고 참가하면 된다.
팀(blue/red)은 참가할 때 인원이 적은 쪽으로 자동 배정된다.
  · blue 는 왼쪽 골대를 공략하고 오른쪽 골대를 지킨다
  · red  는 오른쪽 골대를 공략하고 왼쪽 골대를 지킨다
각 골대 앞에는 자동으로 위아래를 왕복하는 키퍼가 한 명씩 서 있다.

  ① 참가 (서비스) — 이름은 필수
     ros2 service call /register soccer_msgs/srv/Register "{name: '내이름'}"
  ② 상황 파악 (토픽) — 공/골대/내 위치/아군/적군이 계속 흘러나온다
     ros2 topic echo /world_state
  ③ 이동 (서비스) — 거북이 위치 조종
     ros2 service call /move_to soccer_msgs/srv/MoveTo \
       "{name: '내이름', x: 5.0, y: 5.5}"
  ④ 슛 (액션) — 찬 뒤 1초간 정지
     ros2 action send_goal /shoot soccer_msgs/action/Shoot \
       "{name: '내이름', power: 1.0}" --feedback
  ⑤ 퇴장 (서비스)
     ros2 service call /leave soccer_msgs/srv/Register "{name: '내이름'}"

브라우저 뷰어에서도 참가·이동·슛을 할 수 있다 — 뷰어→심판 명령을 상태 방송과
같은 WebSocket 으로 보낸다. 이름만 있으면 그 거북이를 조종한다(학습용 게임).
"""
import asyncio
import base64
import hashlib
import json
import math
import threading
import time

import rclpy as rp
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from soccer_msgs.msg import PlayerState, WorldState
from soccer_msgs.srv import MoveTo, Register
from soccer_msgs.action import Shoot


# ---- 경기장 상수 ----
# 축구장처럼 가로로 긴 직사각형: 가로 15 x 세로 10
FIELD_X_MIN = 0.5
FIELD_X_MAX = 15.5
FIELD_Y_MIN = 0.5
FIELD_Y_MAX = 10.5
CENTER_X = 8.0                 # 가운데 x
CENTER_Y = 5.5                 # 가운데 y
BALL_START = (CENTER_X, CENTER_Y)   # 공 시작 위치(가운데)
GOAL_RIGHT_X = 15.8            # 오른쪽 골대 라인 — red 가 공략
GOAL_LEFT_X = 0.2              # 왼쪽 골대 라인 — blue 가 공략
GOAL_CENTER_Y = CENTER_Y       # 골문 중심 (양쪽 공통)
GOAL_HALF = 1.8                # 골문 반폭 → y 3.7 ~ 7.3

KEEPER_RIGHT_X = 15.0          # 오른쪽 골대 키퍼 (blue 소속 — red 공격을 막는다)
KEEPER_LEFT_X = 1.0            # 왼쪽 골대 키퍼 (red 소속 — blue 공격을 막는다)
KEEPER_Y_MIN = 3.7             # 키퍼 왕복 하한
KEEPER_Y_MAX = 7.3             # 키퍼 왕복 상한
KEEPER_SPEED = 2.0             # 키퍼 왕복 속도 (units/s)
KEEPER_RADIUS = 0.5            # 키퍼 충돌 반지름

PLAYER_SPEED = 3.0             # 선수 이동 속도 (units/s)
PLAYER_RADIUS = 0.5            # 선수 충돌 반지름
PLAYER_RESTITUTION = 0.75      # 선수 몸에 맞고 튀는 정도
BALL_SPEED = 6.0               # 슛 기본 속도 (power=1.0 기준)
# 몸으로 공에 닿는 거리는 PLAYER_RADIUS + BALL_RADIUS = 0.8 이다.
# 사거리를 그보다 넉넉히 잡아야 "공을 밀지 않고 조준해서 차는" 구간이 생긴다.
KICK_RANGE = 1.6               # 이 거리 안에 공이 있어야 찰 수 있다
FREEZE_TIME = 1.0              # 슛 후 정지 시간(초)

# 팀별 공격 방향 — 이 골대에 넣으면 그 팀 득점
# blue 는 왼쪽 골대, red 는 오른쪽 골대를 공략한다.
TEAM_TARGET = {'blue': GOAL_LEFT_X, 'red': GOAL_RIGHT_X}
# 공략하는 골대의 반대편(자기 진영)에서 시작한다
TEAM_SPAWN_X = {'blue': 12.5, 'red': 3.5}
# 골대의 '주인'은 지키는 팀이다: 왼쪽=red 의 골대, 오른쪽=blue 의 골대.
# 따라서 그 골대에 공이 들어가면 반대편 팀이 득점한다.
GOAL_KEEPER_TEAM = {'left': 'red', 'right': 'blue'}    # 그 골대를 지키는 팀
SCORING_TEAM = {'left': 'blue', 'right': 'red'}        # 그 골대에 넣으면 득점하는 팀

# ---- 물리 ----
PHYSICS_DT = 0.02              # 물리 적분 간격(초)
BALL_RADIUS = 0.3              # 공 반지름
POST_RADIUS = 0.12             # 골포스트 반지름
FRICTION = 1.5                 # 마찰 감속 (units/s²) — 공이 점점 느려진다
STOP_SPEED = 0.15              # 이보다 느려지면 완전히 멈춘 것으로 본다
WALL_RESTITUTION = 0.6         # 벽 반발 계수 (0=안 튐, 1=완전탄성)
KEEPER_RESTITUTION = 0.85      # 키퍼 반발 — 세게 튕겨 나온다
POST_RESTITUTION = 0.7         # 골포스트 반발
SHOT_TIMEOUT = 10.0            # 슛 액션이 결과를 기다리는 최대 시간(초)

MAX_NAME_LEN = 12              # 이름표에 들어갈 만한 길이 제한

WS_HOST = "0.0.0.0"
WS_PORT = 8765
_WS_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


# ================= WebSocket (표준 라이브러리만) =================
def _accept_key(key: str) -> str:
    return base64.b64encode(
        hashlib.sha1((key + _WS_GUID).encode()).digest()).decode()


def _text_frame(text: str) -> bytes:
    data = text.encode("utf-8")
    n = len(data)
    header = bytearray([0x81])  # FIN + text opcode
    if n < 126:
        header.append(n)
    elif n < 65536:
        header.append(126)
        header += n.to_bytes(2, "big")
    else:
        header.append(127)
        header += n.to_bytes(8, "big")
    return bytes(header) + data


async def _read_ws_message(reader):
    """브라우저가 보낸 프레임 하나를 텍스트로 읽는다(RFC 6455).

    클라이언트→서버 프레임은 마스킹되어 있다. 닫힘/EOF 면 None,
    text 가 아닌 프레임(ping 등)은 빈 문자열을 돌려준다.
    """
    try:
        h = await reader.readexactly(2)
        opcode = h[0] & 0x0f
        masked = h[1] & 0x80
        length = h[1] & 0x7f
        if length == 126:
            length = int.from_bytes(await reader.readexactly(2), "big")
        elif length == 127:
            length = int.from_bytes(await reader.readexactly(8), "big")
        mask = await reader.readexactly(4) if masked else b""
        data = await reader.readexactly(length)
    except asyncio.IncompleteReadError:
        return None
    if masked:
        data = bytes(b ^ mask[i % 4] for i, b in enumerate(data))
    if opcode == 0x8:                       # close
        return None
    if opcode == 0x1:                       # text
        return data.decode("utf-8", "replace")
    return ""                               # ping/pong/기타 — 무시


async def _ws_handler(reader, writer, node):
    try:
        req = await asyncio.wait_for(reader.readuntil(b"\r\n\r\n"), timeout=5)
    except Exception:
        writer.close()
        return
    key = None
    for line in req.decode("latin1").split("\r\n"):
        if line.lower().startswith("sec-websocket-key:"):
            key = line.split(":", 1)[1].strip()
    if not key:
        writer.close()
        return
    writer.write((
        "HTTP/1.1 101 Switching Protocols\r\n"
        "Upgrade: websocket\r\n"
        "Connection: Upgrade\r\n"
        f"Sec-WebSocket-Accept: {_accept_key(key)}\r\n\r\n"
    ).encode())
    await writer.drain()
    node.get_logger().info("뷰어 접속됨")

    # write() 는 프레임 하나를 통째로 쓰므로 두 코루틴이 섞여 써도 프레임은 안 깨진다
    async def broadcaster():
        # 20Hz 로 경기 상태를 계속 내보낸다
        while True:
            writer.write(_text_frame(json.dumps(node.snapshot())))
            await writer.drain()
            await asyncio.sleep(1.0 / 20)

    async def receiver():
        # 뷰어가 보내는 참가/이동/슛/퇴장 명령을 처리하고, 필요하면 응답한다
        while True:
            text = await _read_ws_message(reader)
            if text is None:
                break
            if not text:
                continue
            reply = node.handle_ui_command(text)
            if reply is not None:
                writer.write(_text_frame(json.dumps(reply)))
                await writer.drain()

    tasks = [asyncio.ensure_future(broadcaster()),
             asyncio.ensure_future(receiver())]
    try:
        await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
    except Exception:
        pass
    finally:
        for t in tasks:
            t.cancel()
        node.get_logger().info("뷰어 접속 종료")
        try:
            writer.close()
        except Exception:
            pass


async def _ws_serve(node):
    try:
        server = await asyncio.start_server(
            lambda r, w: _ws_handler(r, w, node), WS_HOST, WS_PORT)
    except OSError as e:
        # 보통 이전 심판이 안 죽고 남아 포트를 물고 있는 경우다.
        # 이대로 두면 뷰어가 영영 연결되지 않으니 이유를 분명히 남긴다.
        node.get_logger().error(
            f'포트 {WS_PORT} 를 열 수 없다 ({e}). 뷰어가 연결되지 않는다.\n'
            f'  → 이전 심판이 떠 있는지 확인:  pkill -9 -f soccer_referee')
        return
    async with server:
        await server.serve_forever()


# ================= 심판 노드 =================
class SoccerReferee(Node):
    def __init__(self):
        super().__init__('soccer_referee')
        self.cb = ReentrantCallbackGroup()

        # ---- 경기 상태 ----
        # 처음엔 선수가 아무도 없다 — 경기판과 골대만 보인다.
        self.players = {}          # 이름 -> {team, x, y, theta, frozen_until, goals}
        self.lock = threading.Lock()   # players 는 여러 스레드가 건드린다
        self.ball = {'x': BALL_START[0], 'y': BALL_START[1], 'vx': 0.0, 'vy': 0.0}
        # 골대마다 키퍼 한 명씩 — 자기 팀 골대를 지킨다.
        # 소속은 GOAL_KEEPER_TEAM 에서 끌어와 표와 어긋날 일이 없게 한다.
        self.keepers = {
            'right': {'x': KEEPER_RIGHT_X, 'y': GOAL_CENTER_Y,
                      'theta': math.pi, 'dir': 1.0,
                      'team': GOAL_KEEPER_TEAM['right']},
            'left': {'x': KEEPER_LEFT_X, 'y': GOAL_CENTER_Y,
                     'theta': 0.0, 'dir': -1.0,
                     'team': GOAL_KEEPER_TEAM['left']},
        }
        self.score = {'blue': 0, 'red': 0}
        self.msg = '이름을 적고 참가하세요!'
        self.hit_keeper = False    # 이번 슛에서 키퍼를 맞혔나
        self.goal_side = None      # 방금 골이 들어간 골대 ('right'/'left')
        self.goal_reset_at = None  # 이 시각이 지나면 공을 가운데로 되돌린다

        # ---- 서비스 / 액션 ----
        self.create_service(Register, 'register', self.register_cb,
                            callback_group=self.cb)
        self.create_service(Register, 'leave', self.leave_cb,
                            callback_group=self.cb)
        self.create_service(MoveTo, 'move_to', self.move_to_cb,
                            callback_group=self.cb)
        ActionServer(self, Shoot, 'shoot', self.shoot_cb, callback_group=self.cb)

        # ---- 토픽: 경기 상태를 계속 방송한다 ----
        self.world_pub = self.create_publisher(WorldState, 'world_state', 10)
        self.create_timer(0.05, self.publish_world, callback_group=self.cb)

        # ---- 키퍼 자동 왕복 + 선수 이동 + 공 물리 ----
        self.create_timer(0.02, self.keeper_patrol, callback_group=self.cb)
        self.create_timer(PHYSICS_DT, self.drive_players, callback_group=self.cb)
        self.create_timer(PHYSICS_DT, self.physics_step, callback_group=self.cb)

        self.get_logger().info('준비 완료! 뷰어: ws://localhost:8765')
        self.get_logger().info(
            '참가: ros2 service call /register soccer_msgs/srv/Register "{name: \'내이름\'}"')

    # 뷰어로 방송할 상태 스냅샷
    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            players = [{
                'name': name,
                'team': p['team'],
                'x': p['x'], 'y': p['y'], 'theta': p['theta'],
                'frozen': p['frozen_until'] > now,
                'frozen_left': max(0.0, p['frozen_until'] - now),
                'goals': p['goals'],
            } for name, p in self.players.items()]
        return {
            'field': {
                'x_min': FIELD_X_MIN, 'x_max': FIELD_X_MAX,
                'y_min': FIELD_Y_MIN, 'y_max': FIELD_Y_MAX,
                'center_x': CENTER_X, 'center_y': CENTER_Y,
                'goal_right_x': GOAL_RIGHT_X, 'goal_left_x': GOAL_LEFT_X,
                'goal_center_y': GOAL_CENTER_Y, 'goal_half': GOAL_HALF,
                'freeze_time': FREEZE_TIME,
            },
            'players': players,
            'keepers': [
                # name 은 뷰어가 키퍼를 구분하는 키다 — 빠지면 둘이 겹쳐 하나만 그려진다
                {'name': f'keeper_{side}', 'side': side, 'team': k['team'],
                 'x': k['x'], 'y': k['y'], 'theta': k['theta']}
                for side, k in self.keepers.items()
            ],
            'ball': dict(self.ball),
            'score': dict(self.score),
            'msg': self.msg,
        }

    # ---------------- 참가 / 퇴장 ----------------
    def pick_team(self):
        """인원이 적은 팀으로 배정한다(같으면 blue)."""
        blue = sum(1 for p in self.players.values() if p['team'] == 'blue')
        red = len(self.players) - blue
        return 'red' if red < blue else 'blue'

    def do_register(self, name):
        """이름으로 참가시킨다. (성공?, 안내문구, 팀) 반환 — 서비스·뷰어 공용.

        토큰은 없다. 같은 도메인/뷰어에 있으면 이름만으로 조종한다(학습용).
        """
        name = (name or '').strip()
        if not name:
            self.get_logger().warn('참가 거부: 이름 없음')
            return False, '이름이 비어 있다. 이름을 적어야 참가할 수 있다.', ''
        if len(name) > MAX_NAME_LEN:
            return False, f'이름이 너무 길다 (최대 {MAX_NAME_LEN}자).', ''

        with self.lock:
            if name in self.players:
                return False, f"'{name}' 은 이미 참가 중인 이름이다. 다른 이름을 쓰자.", ''

            team = self.pick_team()
            mates = sum(1 for p in self.players.values() if p['team'] == team)
            self.players[name] = {
                'team': team,
                'x': TEAM_SPAWN_X[team],
                'y': clamp(2.0 + 1.4 * (mates % 6), FIELD_Y_MIN, FIELD_Y_MAX),
                'theta': math.pi if team == 'blue' else 0.0,
                'vx': 0.0, 'vy': 0.0,
                'target': None,          # 걸어갈 목표 지점 (없으면 정지)
                'frozen_until': 0.0,
                'goals': 0,
            }
            blue = sum(1 for p in self.players.values() if p['team'] == 'blue')
            red = len(self.players) - blue

        goal = '왼쪽' if team == 'blue' else '오른쪽'
        self.msg = f'{name} 참가! ({team})'
        self.get_logger().info(f'참가: {name} → {team} 팀 (blue {blue} : {red} red)')
        return True, (f'{name} → {team} 팀 배정! {goal} 골대를 공략하자. '
                      f'(blue {blue} : {red} red)'), team

    def register_cb(self, request, response):
        ok, message, _team = self.do_register(request.name)
        response.ok = ok
        response.message = message
        return response

    def do_leave(self, name):
        """참가 중이면 퇴장시킨다. 실제로 나갔으면 True."""
        name = (name or '').strip()
        with self.lock:
            player = self.players.pop(name, None)
        if player is None:
            return False
        self.msg = f'{name} 퇴장'
        self.get_logger().info(f'퇴장: {name} ({player["team"]})')
        return True

    def leave_cb(self, request, response):
        name = (request.name or '').strip()
        if self.do_leave(name):
            response.ok = True
            response.message = f'{name} 퇴장 완료.'
        else:
            response.ok = False
            response.message = f"'{name}' 은 참가 중이 아니다."
        return response

    def get_player(self, name):
        with self.lock:
            return self.players.get((name or '').strip())

    # ---------------- 뷰어(WebSocket)에서 오는 명령 ----------------
    def handle_ui_command(self, text):
        """브라우저 뷰어가 보낸 JSON 명령을 처리한다. 응답 dict 또는 None.

        {"cmd":"join","name":..} / {"cmd":"move","name":..,"x":..,"y":..}
        {"cmd":"shoot","name":..,"power":..} / {"cmd":"leave","name":..}
        토큰이 없으므로 이름만 맞으면 그 거북이를 조종한다(서비스/액션과 동일).
        """
        try:
            c = json.loads(text)
            cmd = c.get('cmd')
            name = (c.get('name') or '').strip()

            if cmd == 'join':
                ok, message, team = self.do_register(name)
                return {'type': 'join', 'ok': ok, 'name': name,
                        'team': team, 'message': message}

            if cmd == 'leave':
                return {'type': 'leave', 'ok': self.do_leave(name), 'name': name}

            if cmd == 'move':
                player = self.get_player(name)
                if player is None:
                    return {'type': 'move', 'ok': False, 'message': '참가 중이 아니다.'}
                tx = clamp(float(c.get('x', player['x'])), FIELD_X_MIN, FIELD_X_MAX)
                ty = clamp(float(c.get('y', player['y'])), FIELD_Y_MIN, FIELD_Y_MAX)
                with self.lock:
                    player['target'] = (tx, ty)
                return None

            if cmd == 'shoot':
                player = self.get_player(name)
                if player is None:
                    return {'type': 'shoot', 'ok': False, 'message': '참가 중이 아니다.'}
                kicked, message = self.do_kick(player, float(c.get('power', 1.0)))
                if kicked:
                    self.msg = f'{name} 슛!'
                return {'type': 'shoot', 'ok': kicked, 'message': message}
        except Exception as e:
            self.get_logger().warn(f'뷰어 명령 처리 실패: {e}')
        return None

    def wait_unfrozen(self, player):
        """정지 중이면 풀릴 때까지 기다린다."""
        while True:
            left = player['frozen_until'] - time.monotonic()
            if left <= 0:
                return
            time.sleep(min(left, 0.05))

    # ---- 키퍼: 자기 골대 앞에서 위아래 자동 왕복 ----
    def keeper_patrol(self):
        for k in self.keepers.values():
            y = k['y'] + k['dir'] * KEEPER_SPEED * 0.02
            if y >= KEEPER_Y_MAX:
                y, k['dir'] = KEEPER_Y_MAX, -1.0
            elif y <= KEEPER_Y_MIN:
                y, k['dir'] = KEEPER_Y_MIN, 1.0
            k['y'] = y

    # ---- 공 물리: 마찰 + 반사 ----
    def physics_step(self):
        self.separate_bodies()      # 거북이끼리 겹침 해소는 항상 돌린다
        b = self.ball

        # 골이 들어갔으면 잠깐 보여준 뒤 공을 가운데에서 다시 시작
        if self.goal_reset_at is not None and time.monotonic() >= self.goal_reset_at:
            b['x'], b['y'] = BALL_START
            b['vx'] = b['vy'] = 0.0
            self.goal_reset_at = None
            self.goal_side = None
            return

        speed = math.hypot(b['vx'], b['vy'])

        if speed >= 1e-9:
            # 마찰로 일정하게 감속하고, 충분히 느려지면 멈춘다
            decel = FRICTION * PHYSICS_DT
            if speed <= decel or speed < STOP_SPEED:
                b['vx'] = b['vy'] = 0.0
            else:
                f = (speed - decel) / speed
                b['vx'] *= f
                b['vy'] *= f
                b['x'] += b['vx'] * PHYSICS_DT
                b['y'] += b['vy'] * PHYSICS_DT

        # 공이 멈춰 있어도 충돌은 검사한다 — 거북이가 밀고 들어올 수 있다
        self.resolve_ball_collisions()

    def resolve_ball_collisions(self):
        for k in self.keepers.values():
            if self.bounce_circle(k['x'], k['y'], KEEPER_RADIUS, KEEPER_RESTITUTION):
                self.hit_keeper = True
        # 사람이 조종하는 거북이와도 부딪히면 공이 튕겨 나간다.
        # 움직이는 거북이가 들이받으면 그 속도만큼 공이 밀려난다.
        with self.lock:
            bodies = [(p['x'], p['y'], p['vx'], p['vy'])
                      for p in self.players.values()]
        for px, py, pvx, pvy in bodies:
            self.bounce_circle(px, py, PLAYER_RADIUS, PLAYER_RESTITUTION, pvx, pvy)
        for goal_x in (GOAL_RIGHT_X, GOAL_LEFT_X):
            for post_y in (GOAL_CENTER_Y - GOAL_HALF, GOAL_CENTER_Y + GOAL_HALF):
                self.bounce_circle(goal_x, post_y, POST_RADIUS, POST_RESTITUTION)
        self.bounce_walls()

    def bounce_circle(self, cx, cy, radius, restitution, ovx=0.0, ovy=0.0):
        """원형 장애물과 충돌하면 반사시킨다. 맞았으면 True.

        ovx, ovy 는 장애물 자신의 속도. 움직이는 거북이가 공을 들이받으면
        상대 속도로 반사한 뒤 장애물 속도를 더해 주므로 공이 밀려 나간다.
        """
        b = self.ball
        dx, dy = b['x'] - cx, b['y'] - cy
        dist = math.hypot(dx, dy)
        rsum = BALL_RADIUS + radius
        if dist >= rsum or dist < 1e-9:
            return False

        nx, ny = dx / dist, dy / dist       # 충돌면 법선
        b['x'] = cx + nx * rsum             # 파고든 만큼 밀어낸다
        b['y'] = cy + ny * rsum

        rvx, rvy = b['vx'] - ovx, b['vy'] - ovy   # 장애물 기준 상대 속도
        vn = rvx * nx + rvy * ny
        if vn < 0:                          # 다가오는 중일 때만 반사
            rvx -= (1 + restitution) * vn * nx
            rvy -= (1 + restitution) * vn * ny
            b['vx'], b['vy'] = rvx + ovx, rvy + ovy
        return True

    def separate_bodies(self):
        """거북이끼리 겹치면 서로 밀어낸다. 키퍼는 밀리지 않는다."""
        with self.lock:
            plist = list(self.players.values())

        # 선수끼리 — 겹친 만큼 절반씩 서로 밀어낸다
        for i in range(len(plist)):
            for j in range(i + 1, len(plist)):
                a, c = plist[i], plist[j]
                dx, dy = c['x'] - a['x'], c['y'] - a['y']
                dist = math.hypot(dx, dy)
                overlap = 2 * PLAYER_RADIUS - dist
                if overlap <= 0:
                    continue
                if dist < 1e-9:             # 완전히 겹쳤으면 임의 방향으로
                    dx, dy, dist = 1.0, 0.0, 1.0
                nx, ny = dx / dist, dy / dist
                push = overlap / 2
                a['x'] = clamp(a['x'] - nx * push, FIELD_X_MIN, FIELD_X_MAX)
                a['y'] = clamp(a['y'] - ny * push, FIELD_Y_MIN, FIELD_Y_MAX)
                c['x'] = clamp(c['x'] + nx * push, FIELD_X_MIN, FIELD_X_MAX)
                c['y'] = clamp(c['y'] + ny * push, FIELD_Y_MIN, FIELD_Y_MAX)

        # 선수 vs 키퍼 — 키퍼는 제자리를 지키므로 선수만 밀려난다
        for p in plist:
            for k in self.keepers.values():
                dx, dy = p['x'] - k['x'], p['y'] - k['y']
                dist = math.hypot(dx, dy)
                rsum = PLAYER_RADIUS + KEEPER_RADIUS
                if dist >= rsum:
                    continue
                if dist < 1e-9:
                    dx, dy, dist = -1.0, 0.0, 1.0
                nx, ny = dx / dist, dy / dist
                p['x'] = clamp(k['x'] + nx * rsum, FIELD_X_MIN, FIELD_X_MAX)
                p['y'] = clamp(k['y'] + ny * rsum, FIELD_Y_MIN, FIELD_Y_MAX)

    def bounce_walls(self):
        """벽 반사. 좌우는 골문 안이면 통과(골), 밖이면 벽."""
        b = self.ball
        if b['y'] < FIELD_Y_MIN:
            b['y'] = FIELD_Y_MIN
            b['vy'] = -b['vy'] * WALL_RESTITUTION
        elif b['y'] > FIELD_Y_MAX:
            b['y'] = FIELD_Y_MAX
            b['vy'] = -b['vy'] * WALL_RESTITUTION

        in_mouth = abs(b['y'] - GOAL_CENTER_Y) <= GOAL_HALF

        if b['x'] > FIELD_X_MAX:            # 오른쪽 — red 의 목표
            if in_mouth:
                if b['x'] >= GOAL_RIGHT_X and self.goal_side is None:
                    self.record_goal('right', SCORING_TEAM['right'])
            else:
                b['x'] = FIELD_X_MAX
                b['vx'] = -b['vx'] * WALL_RESTITUTION
        elif b['x'] < FIELD_X_MIN:          # 왼쪽 — blue 의 목표
            if in_mouth:
                if b['x'] <= GOAL_LEFT_X and self.goal_side is None:
                    self.record_goal('left', SCORING_TEAM['left'])
            else:
                b['x'] = FIELD_X_MIN
                b['vx'] = -b['vx'] * WALL_RESTITUTION

    def record_goal(self, side, team):
        """득점 처리 — 슛이든 몸으로 밀어 넣었든 여기서 한 번만 센다."""
        self.goal_side = side
        self.score[team] += 1
        self.ball['vx'] = self.ball['vy'] = 0.0
        # 잠시 뒤 공을 가운데로 리스폰(슛 액션과 무관하게 심판이 책임진다)
        self.goal_reset_at = time.monotonic() + 0.8
        self.msg = f'⚽ {team.upper()} 득점! [{self.score["blue"]} : {self.score["red"]}]'
        self.get_logger().info(
            f'⚽ {team} 득점  [blue {self.score["blue"]} : {self.score["red"]} red]')

    # ---------------- 선수 이동 ----------------
    def drive_players(self):
        """목표(target)가 있는 선수를 매 틱 그쪽으로 한 걸음 걸어가게 한다.

        서비스(/move_to)든 뷰어 클릭이든 target 만 찍어 주면 여기서 움직인다.
        이동 중 속도(vx,vy)를 남겨 둔다 — 공을 들이받을 때 이 속도로 밀어낸다.
        """
        now = time.monotonic()
        step = PLAYER_SPEED * PHYSICS_DT
        with self.lock:
            for p in self.players.values():
                if p['frozen_until'] > now:      # 정지 중엔 안 움직인다
                    p['vx'] = p['vy'] = 0.0
                    continue
                tgt = p.get('target')
                if tgt is None:
                    p['vx'] = p['vy'] = 0.0
                    continue
                dx, dy = tgt[0] - p['x'], tgt[1] - p['y']
                dist = math.hypot(dx, dy)
                if dist <= step or dist < 1e-6:  # 도착
                    p['x'], p['y'] = tgt
                    p['vx'] = p['vy'] = 0.0
                    p['target'] = None
                else:
                    ux, uy = dx / dist, dy / dist
                    p['x'] += ux * step
                    p['y'] += uy * step
                    p['vx'], p['vy'] = ux * PLAYER_SPEED, uy * PLAYER_SPEED
                    p['theta'] = math.atan2(dy, dx)

    def move_to_cb(self, request, response):
        name = (request.name or '').strip()
        player = self.get_player(name)
        if player is None:
            response.arrived = False
            response.message = f"'{name}' 은 참가 중이 아니다. 먼저 /register 로 참가하자."
            return response

        self.wait_unfrozen(player)          # 정지 중이면 풀릴 때까지 대기
        tx = clamp(request.x, FIELD_X_MIN, FIELD_X_MAX)
        ty = clamp(request.y, FIELD_Y_MIN, FIELD_Y_MAX)
        with self.lock:
            player['target'] = (tx, ty)

        # 도착(타깃 소진) 또는 퇴장까지 기다렸다가 응답한다
        end = time.monotonic() + 30.0
        while time.monotonic() < end:
            if name not in self.players:
                response.arrived = False
                response.message = '이동 중 퇴장 처리되었다.'
                return response
            if player.get('target') is None:
                break
            time.sleep(0.02)

        response.arrived = True
        response.x = float(player['x'])
        response.y = float(player['y'])
        response.message = '도착'
        return response

    # ---------------- 토픽: 경기 상태 방송 ----------------
    def publish_world(self):
        now = time.monotonic()
        msg = WorldState()
        msg.ball_x = float(self.ball['x'])
        msg.ball_y = float(self.ball['y'])
        msg.ball_vx = float(self.ball['vx'])
        msg.ball_vy = float(self.ball['vy'])
        msg.goal_right_x = float(GOAL_RIGHT_X)
        msg.goal_left_x = float(GOAL_LEFT_X)
        msg.goal_center_y = float(GOAL_CENTER_Y)
        msg.goal_half = float(GOAL_HALF)
        msg.field_x_min = float(FIELD_X_MIN)
        msg.field_x_max = float(FIELD_X_MAX)
        msg.field_y_min = float(FIELD_Y_MIN)
        msg.field_y_max = float(FIELD_Y_MAX)
        msg.score_blue = int(self.score['blue'])
        msg.score_red = int(self.score['red'])

        with self.lock:
            items = list(self.players.items())
        for name, p in items:
            ps = PlayerState()
            ps.name = name
            ps.team = p['team']
            ps.x = float(p['x'])
            ps.y = float(p['y'])
            ps.theta = float(p['theta'])
            ps.frozen = p['frozen_until'] > now
            ps.frozen_left = float(max(0.0, p['frozen_until'] - now))
            ps.goals = int(p['goals'])
            msg.players.append(ps)

        for side, k in self.keepers.items():
            ks = PlayerState()
            ks.name = f'keeper_{side}'
            ks.team = k['team']
            ks.x = float(k['x'])
            ks.y = float(k['y'])
            ks.theta = float(k['theta'])
            msg.keepers.append(ks)

        self.world_pub.publish(msg)

    # ---------------- 슛 ----------------
    def do_kick(self, player, power):
        """사거리 안이면 '나 → 공' 방향으로 찬다. (찼는지, 안내문구) 반환.

        정지 중이거나 사거리 밖이면 안 차고 이유를 돌려준다(예외를 던지지 않는다).
        차고 나면 1초 정지 + 이동 취소. 액션(/shoot)과 뷰어가 공용으로 쓴다.
        어디서 다가가느냐가 곧 조준이다 — 골대 반대편에 서서 차면 골대로 간다.
        """
        now = time.monotonic()
        if player['frozen_until'] > now:
            return False, '아직 정지 중이다. 잠시 후 다시 차자.'
        b = self.ball
        dx, dy = b['x'] - player['x'], b['y'] - player['y']
        dist = math.hypot(dx, dy)
        if dist > KICK_RANGE:
            return False, (f'공이 너무 멀다 (거리 {dist:.2f} > 사거리 {KICK_RANGE}). '
                           f'공에 더 다가가자.')
        power = clamp(power, 0.0, 1.0) or 1.0
        speed = BALL_SPEED * power
        norm = dist or 1.0
        self.hit_keeper = False
        self.goal_side = None
        b['vx'] = dx / norm * speed
        b['vy'] = dy / norm * speed
        player['theta'] = math.atan2(dy, dx)
        player['frozen_until'] = now + FREEZE_TIME   # 찬 직후 1초 정지
        with self.lock:
            player['target'] = None                  # 차면 이동을 멈춘다
        return True, '슛!'

    def shoot_cb(self, goal_handle):
        req = goal_handle.request
        result = Shoot.Result()
        name = (name or '').strip()
        player = self.get_player(name)
        if player is None:
            goal_handle.succeed()
            result.message = f"'{name}' 은 참가 중이 아니다. 먼저 /register 로 참가하자."
            return result

        self.wait_unfrozen(player)          # 정지 중이면 풀릴 때까지 대기
        b = self.ball

        kicked, message = self.do_kick(player, req.power)
        if not kicked:
            goal_handle.succeed()
            result.kicked = False
            result.ball_x = float(b['x'])
            result.ball_y = float(b['y'])
            result.message = message
            self.msg = f'{name} 헛발질!'
            return result

        self.msg = f'{name} 슛!'

        # 골이 들어가거나 공이 멈출 때까지 지켜본다(정지 카운트다운도 함께 보고)
        feedback = Shoot.Feedback()
        goal_x = TEAM_TARGET[player['team']]
        side = None
        waited = 0.0
        while waited < SHOT_TIMEOUT:
            if self.goal_side is not None:
                side = self.goal_side    # 심판이 리셋하며 지우기 전에 붙잡는다
                break
            if math.hypot(b['vx'], b['vy']) == 0.0:
                break
            feedback.freeze_left = float(
                max(0.0, player['frozen_until'] - time.monotonic()))
            feedback.ball_x = float(b['x'])
            feedback.dist_to_goal = float(
                math.hypot(goal_x - b['x'], GOAL_CENTER_Y - b['y']))
            goal_handle.publish_feedback(feedback)
            time.sleep(0.05)
            waited += 0.05

        goal_handle.succeed()
        scoring_team = SCORING_TEAM.get(side)
        goal = side is not None
        own_goal = goal and scoring_team != player['team']
        blocked = self.hit_keeper and not goal

        # 점수 집계와 공 리스폰은 심판(record_goal/physics_step)이 이미 했다.
        # 여기서는 누가 넣었는지만 기록한다.
        if goal:
            score = f'[{self.score["blue"]} : {self.score["red"]}]'
            if own_goal:
                self.msg = f'😱 {name} 자책골! ({scoring_team} 득점) {score}'
                self.get_logger().info(
                    f'😱 자책골 — {name}({player["team"]}) → {scoring_team} 득점 {score}')
            else:
                player['goals'] += 1
                self.msg = f'⚽ {name} GOAL!!! ({scoring_team}) {score}'
                self.get_logger().info(
                    f'⚽ GOAL!!! — {name}({scoring_team}) 개인 {player["goals"]}골 {score}')
        elif blocked:
            self.msg = f'🧤 {name} 슛 — 키퍼가 막았다!'
            self.get_logger().info(f'🧤 키퍼가 막았다! ({name})')
        else:
            self.msg = f'{name} 슛 — 빗나감...'
            self.get_logger().info(f'빗나감... ({name})')

        result.kicked = True
        result.goal = goal and not own_goal
        result.blocked = blocked
        result.ball_x = float(b['x'])
        result.ball_y = float(b['y'])
        result.message = (
            '자책골...' if own_goal else
            'GOAL!' if goal else
            ('키퍼가 막음' if blocked else '빗나감'))
        return result


def main(args=None):
    rp.init(args=args)
    node = SoccerReferee()
    # WebSocket 서버는 백그라운드 스레드, rclpy.spin 은 메인 스레드
    threading.Thread(
        target=lambda: asyncio.run(_ws_serve(node)), daemon=True).start()
    executor = MultiThreadedExecutor()
    try:
        rp.spin(node, executor=executor)
    finally:
        node.destroy_node()
        if rp.ok():
            rp.shutdown()


if __name__ == '__main__':
    main()
