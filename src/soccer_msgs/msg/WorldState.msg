# 경기 상태 전체 방송 (토픽 /world_state, 20Hz).
# 계속 흘러나오는 데이터라 토픽이 맞다.
#
# 내 위치 / 우리 편 / 상대편은 players 를 이름·팀으로 걸러서 알아낸다:
#   me       = [p for p in msg.players if p.name == 내이름][0]
#   teammate = [p for p in msg.players if p.team == me.team and p.name != me.name]
#   opponent = [p for p in msg.players if p.team != me.team]
# 내가 공략할 골대는 팀에 따라 갈린다:
#   blue -> goal_right_x,  red -> goal_left_x   (둘 다 y 는 goal_center_y ± goal_half)

# 공
float32 ball_x
float32 ball_y
float32 ball_vx
float32 ball_vy

# 골대
float32 goal_right_x    # blue 가 공략 / red 가 지킴
float32 goal_left_x     # red 가 공략 / blue 가 지킴
float32 goal_center_y   # 골문 중심 y (양쪽 공통)
float32 goal_half       # 골문 반폭 → goal_center_y ± goal_half

# 경기장 경계 (가로로 긴 축구장 모양이라 x, y 가 다르다)
float32 field_x_min
float32 field_x_max
float32 field_y_min
float32 field_y_max

# 점수
int32 score_blue
int32 score_red

# 사람이 조종하는 선수들
PlayerState[] players
# 각 골대를 지키는 자동 키퍼들
PlayerState[] keepers
