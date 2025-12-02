# 필요한 모듈을 가져옵니다. (Pybricks 환경 가정)
# from pybricks.ev3devices import Motor, ColorSensor
# from pybricks.parameters import Port
# from pybricks.robotics import DriveBase
# from pybricks.tools import wait
# from pybricks.hubs import EV3Brick
# ev3 = EV3Brick()

ev3.speaker.beep() # 로봇 시작/특정 동작 시작을 알리는 경고음 발생


# --- 1. 초기 설정 (모터 및 센서 정의) ---
left_motor = Motor(Port.B) # 왼쪽 주행 모터를 Port B에 연결
right_motor = Motor(Port.C) # 오른쪽 주행 모터를 Port C에 연결
line_sensor = ColorSensor(Port.S2) # 라인 트레이싱용 왼쪽 컬러 센서 (S2 포트)
line_sensor2 = ColorSensor(Port.S1) # 라인 트레이싱용 오른쪽 컬러 센서 및 정지 감지 (S1 포트)

# 로봇 구동부 설정: 바퀴 지름 55.5mm, 트랙 폭 104mm (실제 로봇 규격에 맞게 설정)
robot = DriveBase(left_motor, right_motor, 55.5, 104)


# --- 2. 라인 트레이싱 제어 변수 ---
threshold = 10 # 라인(검은색)과 바닥(흰색)을 구분하는 기준 반사율 (매우 낮은 값으로 설정됨)
kp = 1.2       # P(비례) 제어 상수: 라인을 벗어났을 때 회전율을 결정하는 민감도


# --- 3. 첫 번째 이동 및 교차로 통과 (1번 영역에서 2번 영역으로 이동) ---
for i in range(1): # 루프를 한 번만 실행 (특정 경로의 시작)
    # [3-1] 교차점(검은 선)을 만날 때까지 라인 트레이싱
    while True:
        left_reflection = line_sensor.reflection()
        right_reflection = line_sensor2.reflection()
        
        # **정지 조건 1**: 오른쪽 센서(line_sensor2)가 검은 선(반사율 < 25)을 감지하면 정지
        if right_reflection < 25:
            robot.stop()
            break
        else:
            # P-제어 로직: 왼쪽 센서 값을 기준으로 오차(error) 계산
            error = left_reflection - threshold 
            turn_rate = kp * error # 오차에 비례하여 회전 속도 결정
            robot.drive(100,turn_rate) # 속도 100, 계산된 turn_rate로 주행
        wait(10) # 10ms 대기 (제어 주기)

    # [3-2] 교차점을 완전히 통과하여 흰색 바닥으로 나올 때까지 라인 트레이싱
    while True:
        left_reflection = line_sensor.reflection()
        right_reflection = line_sensor2.reflection()
        
        # **정지 조건 2**: 오른쪽 센서(line_sensor2)가 흰색 영역(반사율 > 25)을 감지하면 정지
        if right_reflection > 25:
            robot.stop()
            break
        else:
            # 교차로를 완전히 통과하기 위해 계속 라인 트레이싱 유지
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)

robot.straight(5) # 교차로를 완전히 벗어나기 위해 5mm 추가 전진


# --- 4. 첫 번째 방향 전환 (하드코딩된 턴) ---
now_dir = 1       # 현재 방향을 1 (예: 북쪽)로 설정
target_dir = 2    # 목표 방향을 2 (예: 동쪽)로 설정 (이미지상 90도 우회전)

ev3.speaker.beep()
direction = (target_dir - now_dir) % 4 # 방향 차이를 0, 1, 2, 3 중 하나로 계산
turn_table = [0, 90, 180, -90] # 방향 차이(0, 1, 2, 3)에 따른 회전 각도표
angle = turn_table[direction] # 계산된 각도 (여기서는 90도)
robot.turn(angle) # 로봇 회전 실행


# --- 5. 두 번째 이동 (2개의 교차로 통과) ---
for i in range(2): # 라인 트레이싱/교차점 통과 루프를 2회 반복 (2개의 교차로 통과)
    # [5-1] 교차점 만날 때까지 이동 (주석 3-1과 동일 로직)
    while True:
        left_reflection = line_sensor.reflection()
        right_reflection = line_sensor2.reflection()
        if right_reflection < 25:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)

    # [5-2] 교차점 완전히 통과 (주석 3-2와 동일 로직)
    while True:
        left_reflection = line_sensor.reflection()
        right_reflection = line_sensor2.reflection()
        if right_reflection > 25:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)

robot.straight(5) # 교차로를 완전히 벗어나기 위해 5mm 추가 전진


# --- 6. 두 번째 방향 전환 (하드코딩된 턴) ---
now_dir = 1       # 현재 방향을 1로 재설정 (주의: 이전 턴으로 인해 실제 방향과 다를 수 있음)
target_dir = 4    # 목표 방향을 4 (예: 서쪽)로 설정

ev3.speaker.beep()
direction = (target_dir - now_dir) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction] # (4 - 1) % 4 = 3, turn_table[3] = -90도 회전
robot.turn(angle) 


# --- 7. 세 번째 이동 (2개의 교차로 통과) ---
for i in range(2): # 루프 2회 반복 (2개의 교차로 통과)
    # [7-1] 교차점 만날 때까지 이동 (센서 위치 반전됨)
    # **주의**: 여기서는 센서 역할이 바뀜. (line_sensor2가 왼쪽, line_sensor가 오른쪽을 감지)
    while True:
        left_reflection = line_sensor2.reflection() # 왼쪽 반사율을 S1(line_sensor2)로 읽음
        right_reflection = line_sensor.reflection() # 오른쪽 반사율을 S2(line_sensor)로 읽음
        
        # **정지 조건**: 오른쪽 센서(line_sensor)가 검은 선(반사율 < 25)을 감지하면 정지
        if right_reflection < 25: 
            robot.stop()
            break
        else:
            # P-제어: 왼쪽 센서 값(line_sensor2)을 기준으로 오차 계산
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)

    # [7-2] 교차점 완전히 통과 (센서 위치 반전됨)
    while True:
        left_reflection = line_sensor2.reflection()
        right_reflection = line_sensor.reflection()
        
        # **정지 조건**: 오른쪽 센서(line_sensor)가 흰색 영역(반사율 > 25)을 감지하면 정지
        if right_reflection > 25:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)

robot.straight(5) # 5mm 추가 전진


# --- 8. 세 번째 방향 전환 (하드코딩된 턴) ---
now_dir = 1
target_dir = 4

ev3.speaker.beep()
direction = (target_dir - now_dir) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction] # -90도 회전
robot.turn(angle)


# --- 9. 네 번째 이동 (2개의 교차로 통과) ---
# **주의**: 센서 역할은 7번 섹션과 동일하게 유지됨
for i in range(2):
    # [9-1] 교차점 만날 때까지 이동
    while True:
        left_reflection = line_sensor2.reflection()
        right_reflection = line_sensor.reflection()
        if right_reflection < 25:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)

    # [9-2] 교차점 완전히 통과
    while True:
        left_reflection = line_sensor2.reflection()
        right_reflection = line_sensor.reflection()
        if right_reflection > 25:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)

robot.straight(5) # 5mm 추가 전진


# --- 10. 네 번째 방향 전환 (하드코딩된 턴) ---
now_dir = 1
target_dir = 2

ev3.speaker.beep()
direction = (target_dir - now_dir) % 4
turn_table = [0, 90, 180, -90]
angle = turn_table[direction] # 90도 회전
robot.turn(angle)
wait(10)


# --- 11. 마지막 이동 (1개의 교차로 통과) ---
for i in range(1): # 루프 1회 반복
    # [11-1] 교차점 만날 때까지 이동 (센서 역할은 3번 섹션과 동일하게 복구됨)
    while True:
        left_reflection = line_sensor.reflection()
        right_reflection = line_sensor2.reflection()
        if right_reflection < 25:
            robot.stop()
            break
        else:
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)

    # [11-2] 교차점 완전히 통과 (주의: 마지막 while문은 line_sensor2와 line_sensor의 역할이 바뀌어 있음)
    while True:
        left_reflection = line_sensor2.reflection() # 왼쪽 반사율을 S1(line_sensor2)로 읽음
        right_reflection = line_sensor.reflection() # 오른쪽 반사율을 S2(line_sensor)로 읽음
        
        # **정지 조건**: 오른쪽 센서(line_sensor)가 흰색 영역(반사율 > 25)을 감지하면 정지
        if right_reflection > 25:
            robot.stop()    
            break
        else:
            # P-제어: 왼쪽 센서 값(line_sensor2)을 기준으로 오차 계산
            error = left_reflection - threshold
            turn_rate = kp * error
            robot.drive(100,turn_rate)
        wait(10)

# 이 지점에서 로봇은 모든 하드코딩된 경로 이동을 완료하고 멈춥니다.