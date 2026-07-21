# 선수(또는 키퍼) 한 명의 상태. /world_state 토픽 안에 담겨 온다.
string name          # 이름 (키퍼는 'keeper_right' / 'keeper_left')
string team          # 'blue' 또는 'red'
float32 x            # 위치 x
float32 y            # 위치 y
float32 theta        # 바라보는 방향(라디안)
bool frozen          # 슛 후 정지 중인가
float32 frozen_left  # 남은 정지 시간(초)
int32 goals          # 넣은 골 수
