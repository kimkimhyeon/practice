#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile


# This program requires LEGO EV3 MicroPython v2.0 or higher.
# Click "Open user guide" on the EV3 extension tab for more information.


# Create your objects here.
ev3 = EV3Brick()


# Write your program here.
ev3.speaker.beep()

# 방향 상수
N1, E2, S3, W4 = 1, 2, 3, 4

# 센서 및 제어 상수
threshold = 15
kp = 1.2

# ✨ [1] 짧은 칸 이동 거리 상수 추가 (필수 측정 필요) ✨
# 로봇이 0.5칸을 이동해야 하는 실제 거리 (mm)를 측정하여 이 값을 설정해야 합니다.
SHORT_MOVE_DIST = 75 

left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
arm_motor = Motor(Port.A)

left_sensor = ColorSensor(Port.S1)
right_sensor = ColorSensor(Port.S2)
object_detector = ColorSensor(Port.S4)
ultra_sensor = UltrasonicSensor(Port.S3)

# 드라이브베이스 초기화
robot = DriveBase(left_motor, right_motor, 55.5, 104)

# 전역변수
red_point = (0,2)
blue_point = (0,4)


# y 한 번 이동할 때 쓸 거리 기반 라인팔로우 (X축에도 사용 가능하다고 가정)
def follow_line_distance(distance_mm):
    # EV3 드라이브베이스 거리 초기화
    robot.reset()

    speed = 100 if distance_mm > 0 else -100
    target = abs(distance_mm)

    while abs(robot.distance()) < target:
        # 지금까지 쓰던 방식 그대로: 왼쪽 센서 기준
        left_reflection = left_sensor.reflection()
        error = left_reflection - threshold
        turn_rate = kp * error
        robot.drive(speed, turn_rate)
        wait(10)

    robot.stop()
    wait(50)

def move_to_y_level(current_y, target_y):
    # Y_LEVEL_DIST 상수가 정의되어야 작동하는 함수입니다. 
    # 현재 코드에 Y_LEVEL_DIST가 없어 호출 불가.
    # 하지만 move_manhattan에서 사용되지 않으므로 그대로 유지합니다.
    if current_y == target_y:
        return current_y

    # 현재/목표 y층의 절대 거리(mm)
    # curr_dist = Y_LEVEL_DIST[current_y] # Y_LEVEL_DIST 정의 필요
    # tgt_dist = Y_LEVEL_DIST[target_y] # Y_LEVEL_DIST 정의 필요
    # diff = tgt_dist - curr_dist

    # follow_line_distance(diff)

    return target_y


def left_line_following(speed, kp) :
    threshold = 15
    left_reflection = left_sensor.reflection()
    error = left_reflection - threshold
    turn_rate = kp * error
    robot.drive(speed, turn_rate)

def right_line_following(speed, kp) :
    threshold = 15
    right_reflection = right_sensor.reflection()
    error = right_reflection - threshold
    turn_rate = -kp * error
    robot.drive(speed, turn_rate)

# n_move 함수는 짧은 칸 이동이 아닐 때만 사용됩니다.
def n_move(n, direction="right") :
    for _ in range(n) :
        if direction == "right" :
            while right_sensor.reflection() > 30 :
                left_line_following(70, 1.2)
            if right_sensor.reflection() <= 30 :     
                robot.stop()
                robot.straight(50) # 표준 1칸 이동 후 라인 건너는 고정 거리
        elif direction == "left" :
            while left_sensor.reflection() > 30 :
                right_line_following(70, 1.2)
            if left_sensor.reflection() <= 30 :
                robot.stop()
                robot.straight(50) # 표준 1칸 이동 후 라인 건너는 고정 거리
    robot.stop()

def grab_object() :
    arm_motor.run_until_stalled(200, then = Stop.COAST, duty_limit=50)

def release_object() :
    arm_motor.run_until_stalled(-200, then = Stop.COAST, duty_limit=50)

def follow_line_one_cell() :
    while True:
        left_reflection = left_sensor.reflection()
        right_reflection = right_sensor.reflection()
        if right_reflection < 30:
            robot.stop()
            break
        else:
            error=left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100, turn_rate)
        wait(10)


def turn_min(now_dir, target_dir):
    diff = (target_dir - now_dir) % 4
    angle = [0, 90, 180, -90][diff]
    print("angle = " , angle)
    robot.turn(angle)
    return target_dir

# ✨ [2] move_manhattan 함수 수정 (짧은 칸 분기 처리) ✨
def move_manhattan(start_xy, goal_xy, now_dir):

    x, y = start_xy
    gx, gy = goal_xy
    dx = gx - x
    dy = gy - y

    # 1) 가로 이동 (X축)
    if dx != 0:
        target_dir = E2 if dx > 0 else W4
        now_dir = turn_min(now_dir, target_dir)
        steps = abs(dx)
        
        for _ in range(steps):
            
            is_short_move = False
            
            # (0, y) -> (1, y) 이동 (dx > 0)
            if dx > 0 and x == 0:
                is_short_move = True
            
            # (1, y) -> (0, y) 이동 (dx < 0)
            elif dx < 0 and x == 1:
                is_short_move = True
                
            if is_short_move:
                # 짧은 칸 (0.5칸) 이동: 고정 거리 직진
                ev3.speaker.beep()
                distance_mm = SHORT_MOVE_DIST if dx > 0 else -SHORT_MOVE_DIST
                
                # 라인 팔로잉이 필요하다면 follow_line_distance(distance_mm) 사용
                # 현재는 가장 간단한 robot.straight(distance_mm) 사용
                robot.straight(distance_mm) 
                
            else:
                # 일반 1칸 이동: 기존 n_move 로직 사용 (교차점 감지)
                n_move(1, "right" if dx > 0 else "left")
                ev3.speaker.beep()
            
            # 현재 위치 업데이트
            x += 1 if target_dir == E2 or target_dir == S3 else -1 
            # Note: X축 이동 중인데 Y축 방향 S3/N1로 x가 업데이트 되는 로직은 잘못되었습니다.
            # X축 이동 시 X만 업데이트 되어야 합니다. (아래와 같이 수정)
            x += 1 if target_dir == E2 else -1

    # 2) 세로 이동 (Y축)
    if dy > 0:
        target_dir = S3
        now_dir = turn_min(now_dir, target_dir)
        steps = abs(dy)
        for _ in range(steps):
             n_move(1, "right") 
             ev3.speaker.beep()
             y += 1
             
    if dy < 0:
        target_dir = N1
        now_dir = turn_min(now_dir, target_dir)
        steps = abs(dy)
        for _ in range(steps):
             n_move(1, "left") 
             ev3.speaker.beep()
             y -= 1

    return (x, y), now_dir


def object_color_check(object_color) :
    red = object_color[0]
    blue = object_color[2]
    if red > 5 and blue < 10:
        return red_point
    elif blue > 5 and red < 10:
        return blue_point

MAP = [
    "............",
    "##.#.#.#....",
    "...#........",
    "##.#.#.#....",
    "...#........",
]
G = [[1 if c == "#" else 0 for c in r] for r in MAP]

H, W = len(G), len(G[0])

S, E = (0,0), (4,0)


def bfs(s, g) :
    dist = [[None] * W for _ in range(H)]
    prev = [[None] * W for _ in range(H)]
    q = deque([s])
    dist[s[1]][s[0]] = 0
    while q :
        x, y = q.popleft()
        if (x, y) == g :
            break
        for dx, dy in ((1,0), (-1,0), (0,1), (0,-1)) :
            nx, ny = x+dx, y+dy
            if 0 <= nx < W and 0 <= ny < H and not G[ny][nx] and dist[ny][nx] is None :
                dist[ny][nx] = dist[y][x] + 1
                prev[ny][nx] = (x, y)
                q.append((nx,ny))

    path = []
    if dist[g[1]][g[0]] is not None :
        p = g
        while p :
            path.append(p)
            p = prev[p[1]][p[0]]
        path.reverse()
    return path, dist

def show(path, dist) :
    grid = [['#' if G[y][x] else '.' for x in range(W)] for y in range(H)]
    for x, y in path :
        if (x,y) not in (S,E): grid[y][x] = '*'
    grid[S[1]][S[0]], grid[E[1]][E[0]] = 'S', 'G'
    print("=== 경로 맵 ===")
    for r in grid : print("".join(r))
    print("\n=== 거리 히트맵 ===")
    print("\n경로: ", path)
    print("길이 : ", len(path)-1 if path else "없음")

if __name__ == "__main__":
    release_object()
    path, dist = bfs(S,E)
    tmp = []
    # ✨ [3] tmp 배열 오류 수정: Indexing 대신 append 사용 ✨
    j = 0
    for i in range(len(path)) :
        if i % 2 == 0 :
            tmp.append(path[i]) # 수정됨
            j += 1
    
    print(path)
    print(tmp)
    dir = E2
    current_pos = tmp[0]
    for i in range(len(tmp)-1) :
        goal_pos = tmp[i+1]
        current_pos, dir = move_manhattan(current_pos, goal_pos, dir)
    wait(10)
    grab_object()
    wait(10)
    object_color1 = object_detector.rgb()
    print(object_color1)
    goal = []
    goal = object_color_check(object_color1)
    print(goal)
    start = []
    start = path[len(path) - 1]
    print(start)
    path, dist = bfs(start, goal)
    wait(50)
    tmp = []
    # ✨ [3] tmp 배열 오류 수정: Indexing 대신 append 사용 ✨
    j = 0
    for i in range(len(path)) :
    
        if i % 2 == 0 :
            tmp.append(path[i]) # 수정됨
            j += 1
    
    print(path)
    print(tmp)
    current_pos = tmp[0]
    for i in range(len(tmp)-1) :
        goal_pos = tmp[i+1]
        current_pos, dir = move_manhattan(current_pos, goal_pos, dir)
    release_object()
    print("===현재 MAP 출력 ===")
    for r in MAP :
        print(r)
    print()
    show(path, dist)