"""예제 봇 — 사람이 손으로 치던 것을 코드로 하는 참가자.

/register(서비스)로 참가하고 → /world_state(토픽)로 공과 골대를 보고
→ /move_to(서비스)로 공 뒤에 서고 → /shoot(액션)으로 찬다. 혼자서도 여러 명을 띄워 멀티플레이를 시험할 수 있다.

    ros2 run soccer_game example_bot --name 봇1
    ros2 run soccer_game example_bot --name 봇2 --rounds 20

이 파일을 그대로 베껴 자기만의 전략을 짜면 된다 (pick_target 만 고치면 된다).
"""
import argparse
import math
import os
import re
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from soccer_msgs.msg import WorldState
from soccer_msgs.srv import MoveTo, Register
from soccer_msgs.action import Shoot

STAND_OFF = 1.05      # 공에서 이만큼 떨어져 선다(몸으로 밀지 않으면서 찰 수 있는 거리)


def node_name_for(name, suffix):
    """ROS2 노드 이름은 영숫자와 _ 만 된다 — 한글 이름도 쓸 수 있게 정제한다."""
    safe = re.sub(r'[^0-9A-Za-z_]', '', name)
    return f'bot_{safe}_{suffix}' if safe else f'bot_{suffix}'


class ExampleBot(Node):
    def __init__(self, name):
        # 같은 이름의 봇을 여러 번 띄워도 노드 이름이 겹치지 않게 pid 를 붙인다
        super().__init__(node_name_for(name, os.getpid()))
        self.name = name
        self.world = None
        self.create_subscription(WorldState, '/world_state', self._on_world, 10)
        self.reg_cli = self.create_client(Register, '/register')
        self.leave_cli = self.create_client(Register, '/leave')
        self.move_cli = self.create_client(MoveTo, '/move_to')
        self.shoot_cli = ActionClient(self, Shoot, '/shoot')

    def _on_world(self, msg):
        self.world = msg

    # ---- 유틸 ----
    def spin_until(self, pred, timeout=15.0):
        end = time.time() + timeout
        while time.time() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if pred():
                return True
        return False

    def call(self, client, req, timeout=60.0):
        if not client.wait_for_service(timeout_sec=5.0):
            return None
        fut = client.call_async(req)
        self.spin_until(lambda: fut.done(), timeout)
        return fut.result()

    # ---- 토픽에서 내 상황 읽기 ----
    def me(self):
        if self.world is None:
            return None
        return next((p for p in self.world.players if p.name == self.name), None)

    def teammates(self):
        m = self.me()
        return [p for p in self.world.players
                if p.team == m.team and p.name != m.name]

    def opponents(self):
        m = self.me()
        return [p for p in self.world.players if p.team != m.team]

    def my_goal(self):
        """우리 팀이 공략할 골대 (x, y). blue 는 왼쪽, red 는 오른쪽이다."""
        m = self.me()
        gx = (self.world.goal_left_x if m.team == 'blue'
              else self.world.goal_right_x)
        return gx, self.world.goal_center_y

    # ---- 전략: 여기만 고치면 된다 ----
    def pick_target(self):
        """공을 기준으로 골대 반대편에 설 지점을 고른다.

        차는 방향이 '나 -> 공' 이므로, 골대 반대편에 서면 골대 쪽으로 찬다.
        """
        w = self.world
        gx, gy = self.my_goal()
        dx, dy = gx - w.ball_x, gy - w.ball_y
        n = math.hypot(dx, dy) or 1.0
        return (w.ball_x - dx / n * STAND_OFF,
                w.ball_y - dy / n * STAND_OFF)

    # ---- 한 턴 ----
    def play_once(self):
        # 굴러가는 공을 쫓으면 사거리를 놓친다 — 멈출 때까지 기다린다
        self.spin_until(
            lambda: math.hypot(self.world.ball_vx, self.world.ball_vy) == 0.0, 12)

        tx, ty = self.pick_target()
        self.call(self.move_cli,
                  MoveTo.Request(name=self.name, x=float(tx), y=float(ty)))

        if not self.shoot_cli.wait_for_server(timeout_sec=5.0):
            return None
        goal = Shoot.Goal()
        goal.name = self.name
        goal.power = 1.0
        fut = self.shoot_cli.send_goal_async(goal)
        self.spin_until(lambda: fut.done(), 10)
        if fut.result() is None:
            return None
        rf = fut.result().get_result_async()
        self.spin_until(lambda: rf.done(), 25)
        return rf.result().result if rf.result() else None


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', required=True, help='참가할 이름 (필수)')
    parser.add_argument('--rounds', type=int, default=10, help='몇 번 시도할지')
    cli_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    bot = ExampleBot(cli_args.name)

    res = bot.call(bot.reg_cli, Register.Request(name=cli_args.name))
    if res is None:
        bot.get_logger().error('심판(/register)이 안 보인다. 심판을 먼저 띄우자.')
        rclpy.shutdown()
        return 1
    bot.get_logger().info(f'참가: ok={res.ok} — {res.message}')
    if not res.ok:
        rclpy.shutdown()
        return 1

    if not bot.spin_until(lambda: bot.me() is not None, 10):
        bot.get_logger().error('/world_state 를 못 받았다.')
        rclpy.shutdown()
        return 1

    m = bot.me()
    bot.get_logger().info(
        f'나: {m.name}({m.team}) / 아군 {len(bot.teammates())}명 '
        f'/ 적군 {len(bot.opponents())}명 / 목표 골대 x={bot.my_goal()[0]:.1f}')

    try:
        for i in range(cli_args.rounds):
            r = bot.play_once()
            if r is None:
                bot.get_logger().warn(f'[{i+1}] 응답 없음')
                continue
            tag = ('GOAL' if r.goal else 'BLOCKED' if r.blocked
                   else 'MISS' if r.kicked else 'NO-KICK')
            bot.get_logger().info(f'[{i+1}] {tag} — {r.message}')
            bot.spin_until(lambda: False, 1.2)   # 리스폰 대기
        w = bot.world
        bot.get_logger().info(
            f'끝. 스코어 blue {w.score_blue} : {w.score_red} red')
    except KeyboardInterrupt:
        pass
    finally:
        bot.call(bot.leave_cli,
                 Register.Request(name=cli_args.name), timeout=5)
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
