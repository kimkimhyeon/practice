ev3.speaker.beep()


from collections import deque
# N=1, E=2, S=3, W=4
N1, E2, S3, W4 = 1, 2, 3, 4
threshold = 50
kp = 1.2

left_motor = Motor(Port.C)
right_motor = Motor(Port.B)
arm_motor = Motor(Port.A)

left_sensor = ColorSensor(Port. S1)
right_sensor = ColorSensor(Port. S2)
object_detector = ColorSensor(Port. S4)
ultra_sensor = UltrasonicSensor(Port. S3)

# 드라이브베이스 초기화
robot = DriveBase(left_motor, right_motor, 55.5, 104)

#전역변수
red_point = (0,2)
blue_point = (0,4)


# y 한 번 이동할 때 쓸 거리 기반 라인팔로우
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
    if current_y == target_y:
        return current_y

    # 현재/목표 y층의 절대 거리(mm)
    curr_dist = Y_LEVEL_DIST[current_y]
    tgt_dist = Y_LEVEL_DIST[target_y]

    diff = tgt_dist - curr_dist  # >0면 아래로, <0면 위로

    # 이미 move_manhattan 안에서 N/S 방향은 돌려놓는다고 가정하고
    follow_line_distance(diff)

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

def n_move(n, direction="right") :
    for _ in range(n) :
        if direction == "right" :
            while right_sensor.reflection() > 30 :
                left_line_following(70, 1.2)
            if right_sensor.reflection() <= 30 :    
                robot.stop()
                robot.straight(50)
        elif direction == "left" :
            while left_sensor.reflection() > 30 :
                right_line_following(70, 1.2)
            if left_sensor.reflection() <= 30 :
                robot.stop()
                robot.straight(50)
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

#variables

def turn_min(now_dir, target_dir):
    diff = (target_dir - now_dir) % 4
    angle = [0, 90, 180, -90][diff]
    print("angle = " , angle)
    robot.turn(angle)
    return target_dir

def move_manhattan(start_xy, goal_xy, now_dir):

    x, y = start_xy
    gx, gy = goal_xy
    dx = gx - x
    dy = gy - y

    # 1) 가로 이동은 그대로 (한 칸씩)
    if dx > 0:
        target_dir = E2
        now_dir = turn_min(now_dir, target_dir)
        steps = abs(dx)
        for _ in range(steps):
            n_move(1, "right")  # 현재 잘 된다면 그대로 유지
            ev3.speaker.beep()
            x += 1 if target_dir == E2 else -1

    if dx < 0:
        target_dir = W4
        now_dir = turn_min(now_dir, target_dir)
        steps = abs(dx)
        for _ in range(steps):
            n_move(1, "left")  # 현재 잘 된다면 그대로 유지
            ev3.speaker.beep()
            x += 1 if target_dir == W4 else -1

    if dy > 0:
        target_dir = S3
        now_dir = turn_min(now_dir, target_dir)
        steps = abs(dy)
        for _ in range(steps):
            n_move(1, "right")  # 현재 잘 된다면 그대로 유지
            ev3.speaker.beep()
            x += 1 if target_dir == S3 else -1

    if dy < 0:
        target_dir = N1
        now_dir = turn_min(now_dir, target_dir)
        steps = abs(dy)
        for _ in range(steps):
            n_move(1, "left")  # 현재 잘 된다면 그대로 유지
            ev3.speaker.beep()
            x += 1 if target_dir == N1 else -1

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

H, W  = len(G), len(G[0])

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
    j = 0
    for i in range(len(path)) :
        if i % 2 == 0 :
            tmp[j] = path[i]
            j += 1
    
    print(path)
    print(tmp)
    dir = E2
    for i in range(len(tmp)-1) :
        t = []
        t, dir = move_manhattan(tmp[i], tmp[i+1], dir)
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
    j = 0
    for i in range(len(path)) :
    
        if i % 2 == 0 :
            tmp[j] = path[i]
            j += 1
    
    print(path)
    print(tmp)
    for i in range(len(tmp)-1) :
        t = []
        t, dir = move_manhattan(tmp[i], tmp[i+1], dir)
    release_object()
    print("===현재 MAP 출력 ===")
    for r in MAP :
        print(r)
    print()
    show(path, dist)