"""공 물리 + 충돌 검증 — ROS 실행 없이 심판의 물리 함수를 직접 부른다.

    python3 src/soccer_game/test/test_physics.py     # 직접 실행
    pytest  src/soccer_game/test/test_physics.py     # pytest 로도 실행

심판 노드를 띄우지 않으므로 빠르고, 실제 경기에서는 우연히만 나오는 경로
(벽 반사·골포스트 충돌·양쪽 골 판정)까지 결정적으로 확인한다.
"""
import math
import sys
import threading
import types

from soccer_game import soccer_referee as R

Ref = R.SoccerReferee
FAILURES = []


class Fake:
    """심판의 물리 메서드가 쓰는 속성만 흉내낸 스텁."""

    def __init__(self, ball=(8.0, 5.5, 0.0, 0.0), players=None):
        self.ball = {'x': ball[0], 'y': ball[1], 'vx': ball[2], 'vy': ball[3]}
        self.players = players or {}
        self.keepers = {
            'right': {'x': R.KEEPER_RIGHT_X, 'y': 5.5, 'theta': math.pi,
                      'dir': 1.0, 'team': R.GOAL_KEEPER_TEAM['right']},
            'left': {'x': R.KEEPER_LEFT_X, 'y': 5.5, 'theta': 0.0,
                     'dir': -1.0, 'team': R.GOAL_KEEPER_TEAM['left']},
        }
        self.lock = threading.Lock()
        self.goal_side = None
        self.goal_reset_at = None
        self.hit_keeper = False
        self.score = {'blue': 0, 'red': 0}
        self.msg = ''

    def get_logger(self):
        return types.SimpleNamespace(info=lambda *a, **k: None)


# 물리 메서드를 그대로 빌려 쓴다(서로를 self 로 부르므로 클래스에 붙인다)
for _m in ('bounce_circle', 'bounce_walls', 'separate_bodies',
           'resolve_ball_collisions', 'physics_step', 'record_goal'):
    setattr(Fake, _m, getattr(Ref, _m))


def P(x, y, vx=0.0, vy=0.0, team='blue'):
    return {'x': x, 'y': y, 'vx': vx, 'vy': vy, 'theta': 0.0,
            'team': team, 'frozen_until': 0.0, 'goals': 0}


def check(name, cond, detail):
    print(f"{'PASS' if cond else 'FAIL'}  {name}: {detail}")
    if not cond:
        FAILURES.append(name)


def test_physics():
    del FAILURES[:]

    # ---------- 벽 반사 ----------
    f = Fake(ball=(8.0, R.FIELD_Y_MAX + 0.2, 0.0, 4.0))
    f.bounce_walls()
    check("위쪽 벽 반사",
          f.ball['vy'] < 0 and f.ball['y'] == R.FIELD_Y_MAX,
          f"vy 4.00 -> {f.ball['vy']:.2f}, y={f.ball['y']}")

    f = Fake(ball=(8.0, R.FIELD_Y_MIN - 0.2, 0.0, -4.0))
    f.bounce_walls()
    check("아래쪽 벽 반사",
          f.ball['vy'] > 0 and f.ball['y'] == R.FIELD_Y_MIN,
          f"vy -4.00 -> {f.ball['vy']:.2f}, y={f.ball['y']}")

    # ---------- 골 판정 ----------
    f = Fake(ball=(R.GOAL_RIGHT_X + 0.05, 5.5, 5.0, 0.0))
    f.bounce_walls()
    check("오른쪽 골문 통과 → red 득점",
          f.goal_side == 'right' and f.score['red'] == 1
          and f.ball['vx'] == 0.0 and f.goal_reset_at is not None,
          f"side={f.goal_side}, score={f.score}, "
          f"리스폰예약={f.goal_reset_at is not None}")

    f = Fake(ball=(R.GOAL_LEFT_X - 0.05, 5.5, -5.0, 0.0))
    f.bounce_walls()
    check("왼쪽 골문 통과 → blue 득점",
          f.goal_side == 'left' and f.score['blue'] == 1,
          f"side={f.goal_side}, score={f.score}")

    f = Fake(ball=(R.FIELD_X_MAX + 0.2, 9.5, 5.0, 0.0))   # 골문(3.7~7.3) 밖
    f.bounce_walls()
    check("오른쪽 골문 밖 → 벽 반사(득점 없음)",
          f.ball['vx'] < 0 and f.score['red'] == 0,
          f"vx 5.00 -> {f.ball['vx']:.2f}, score={f.score}")

    f = Fake(ball=(R.FIELD_X_MIN - 0.2, 1.5, -5.0, 0.0))
    f.bounce_walls()
    check("왼쪽 골문 밖 → 벽 반사(득점 없음)",
          f.ball['vx'] > 0 and f.score['blue'] == 0,
          f"vx -5.00 -> {f.ball['vx']:.2f}, score={f.score}")

    f = Fake(ball=(R.FIELD_X_MAX + 0.1, 5.5, 5.0, 0.0))   # 골문 안, 골라인 전
    f.bounce_walls()
    check("골문 안이지만 골라인 전 → 계속 진행",
          f.goal_side is None and f.ball['vx'] == 5.0,
          f"side={f.goal_side}, vx={f.ball['vx']}")

    # ---------- 원형 장애물 반사 ----------
    post_y = R.GOAL_CENTER_Y + R.GOAL_HALF
    f = Fake(ball=(R.GOAL_RIGHT_X - 0.3, post_y - 0.1, 6.0, 0.0))
    hit = f.bounce_circle(R.GOAL_RIGHT_X, post_y,
                          R.POST_RADIUS, R.POST_RESTITUTION)
    dist = math.hypot(f.ball['x'] - R.GOAL_RIGHT_X, f.ball['y'] - post_y)
    check("골포스트 충돌 → 반사 + 겹침 해소",
          hit and f.ball['vx'] < 0
          and abs(dist - (R.BALL_RADIUS + R.POST_RADIUS)) < 1e-9,
          f"hit={hit}, vx 6.00 -> {f.ball['vx']:.2f}, 거리={dist:.3f}")

    f = Fake(ball=(R.KEEPER_RIGHT_X - 0.5, 5.5, 6.0, 0.0))
    hit = f.bounce_circle(R.KEEPER_RIGHT_X, 5.5,
                          R.KEEPER_RADIUS, R.KEEPER_RESTITUTION)
    check("키퍼 충돌 → 반사",
          hit and f.ball['vx'] < 0,
          f"hit={hit}, vx 6.00 -> {f.ball['vx']:.2f} (반발 {R.KEEPER_RESTITUTION})")

    f = Fake(ball=(3.0, 2.0, 6.0, 0.0))
    hit = f.bounce_circle(R.KEEPER_RIGHT_X, 5.5,
                          R.KEEPER_RADIUS, R.KEEPER_RESTITUTION)
    check("멀리 있으면 충돌 없음", not hit and f.ball['vx'] == 6.0, f"hit={hit}")

    # ---------- 마찰 ----------
    f = Fake(ball=(8.0, 5.5, 6.0, 0.0))
    speeds, steps = [], 0
    while math.hypot(f.ball['vx'], f.ball['vy']) > 0 and steps < 3000:
        f.physics_step()
        speeds.append(math.hypot(f.ball['vx'], f.ball['vy']))
        steps += 1
    mono = all(speeds[i] >= speeds[i + 1] for i in range(len(speeds) - 1))
    check("마찰: 단조 감속 후 정지",
          mono and speeds[-1] == 0.0,
          f"{speeds[0]:.2f} -> 0.00, {steps}스텝"
          f"({steps * R.PHYSICS_DT:.1f}초), 단조={mono}")

    # ---------- 거북이끼리 반발 ----------
    f = Fake(players={'a': P(8.0, 5.5), 'b': P(8.4, 5.5)})
    f.separate_bodies()
    a, b = f.players['a'], f.players['b']
    d = math.hypot(b['x'] - a['x'], b['y'] - a['y'])
    check("거북이끼리 반발(겹침 해소)",
          abs(d - 2 * R.PLAYER_RADIUS) < 1e-6,
          f"거리 0.40 -> {d:.3f} (목표 {2 * R.PLAYER_RADIUS})")

    f = Fake(players={'a': P(8.0, 5.5), 'b': P(8.0, 5.5)})
    f.separate_bodies()
    d = math.hypot(f.players['b']['x'] - f.players['a']['x'],
                   f.players['b']['y'] - f.players['a']['y'])
    check("완전히 겹쳐도 분리(0 나누기 방지)", d > 0.9, f"거리 0.00 -> {d:.3f}")

    f = Fake(players={'a': P(3.0, 5.5), 'b': P(9.0, 5.5)})
    f.separate_bodies()
    check("떨어져 있으면 그대로",
          f.players['a']['x'] == 3.0 and f.players['b']['x'] == 9.0,
          f"a.x={f.players['a']['x']}, b.x={f.players['b']['x']}")

    f = Fake(players={'a': P(R.KEEPER_RIGHT_X - 0.2, 5.5)})
    kx = f.keepers['right']['x']
    f.separate_bodies()
    d = math.hypot(f.players['a']['x'] - kx, f.players['a']['y'] - 5.5)
    check("거북이 vs 키퍼 — 거북이만 밀림",
          abs(d - (R.PLAYER_RADIUS + R.KEEPER_RADIUS)) < 1e-6
          and f.keepers['right']['x'] == kx,
          f"거리 0.20 -> {d:.3f}, 키퍼 고정={f.keepers['right']['x'] == kx}")

    # ---------- 거북이 ↔ 공 ----------
    f = Fake(ball=(8.0, 5.5, 0.0, 0.0),
             players={'a': P(7.4, 5.5, vx=R.PLAYER_SPEED)})
    f.resolve_ball_collisions()
    check("움직이는 거북이가 멈춘 공을 밀어냄",
          f.ball['vx'] > 0.5,
          f"공 vx 0.00 -> {f.ball['vx']:.2f} (거북이 속도 {R.PLAYER_SPEED})")

    f = Fake(ball=(8.0, 5.5, 5.0, 0.0), players={'a': P(8.6, 5.5)})
    f.resolve_ball_collisions()
    check("멈춘 거북이에 공이 반사",
          f.ball['vx'] < 0,
          f"공 vx 5.00 -> {f.ball['vx']:.2f} (반발 {R.PLAYER_RESTITUTION})")

    # 공이 멈춰 있어도 충돌 검사가 돌아야 한다(조기 반환 회귀 방지)
    f = Fake(ball=(8.0, 5.5, 0.0, 0.0),
             players={'a': P(7.4, 5.5, vx=R.PLAYER_SPEED)})
    f.physics_step()
    check("정지한 공도 physics_step 에서 충돌 처리",
          f.ball['vx'] > 0.5, f"공 vx -> {f.ball['vx']:.2f}")

    # ---------- 골 후 리스폰 ----------
    f = Fake(ball=(R.GOAL_RIGHT_X + 0.05, 5.5, 5.0, 0.0))
    f.bounce_walls()
    f.goal_reset_at = 0.0            # 리스폰 시각이 이미 지난 것으로
    f.physics_step()
    check("골 뒤 공이 가운데로 리스폰",
          (f.ball['x'], f.ball['y']) == R.BALL_START
          and f.goal_side is None and f.goal_reset_at is None,
          f"공=({f.ball['x']}, {f.ball['y']}), side={f.goal_side}")

    # ---------- 팀 방향 계약 ----------
    check("blue 는 왼쪽, red 는 오른쪽 골대를 공략",
          R.TEAM_TARGET['blue'] == R.GOAL_LEFT_X
          and R.TEAM_TARGET['red'] == R.GOAL_RIGHT_X
          and R.SCORING_TEAM['left'] == 'blue'
          and R.SCORING_TEAM['right'] == 'red',
          f"TEAM_TARGET={R.TEAM_TARGET}, SCORING_TEAM={R.SCORING_TEAM}")

    # 골대의 주인은 지키는 팀. 공략하는 팀과 반드시 반대여야 한다.
    check("골대 주인(키퍼 팀)은 공략 팀의 반대",
          R.GOAL_KEEPER_TEAM['left'] == 'red'
          and R.GOAL_KEEPER_TEAM['right'] == 'blue'
          and all(R.GOAL_KEEPER_TEAM[s] != R.SCORING_TEAM[s]
                  for s in ('left', 'right')),
          f"GOAL_KEEPER_TEAM={R.GOAL_KEEPER_TEAM}")

    check("각 팀은 자기 진영(공략 골대 반대편)에서 시작",
          R.TEAM_SPAWN_X['blue'] > R.CENTER_X
          and R.TEAM_SPAWN_X['red'] < R.CENTER_X,
          f"spawn={R.TEAM_SPAWN_X}, center_x={R.CENTER_X}")

    assert not FAILURES, f"실패: {FAILURES}"


if __name__ == '__main__':
    try:
        test_physics()
    except AssertionError as e:
        print(f"\n{e}")
        sys.exit(1)
    print("\n모든 물리 검증 통과")
    sys.exit(0)
