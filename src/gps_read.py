import serial

# 시리얼 포트와 속도 설정
port = "/dev/ttyS0"  # 포트 이름
baud = 9600           # 속도
timeout = 1           # 타임아웃 시간 (초)

# 시리얼 객체 생성
ser = serial.Serial(port, baud, timeout=timeout)

# 시리얼 포트 열기
print("Serial port opened successfully.")

# 무한 루프를 돌며 GPS 데이터를 출력합니다.
while True:
    # 시리얼 데이터가 수신되었을 때
    if ser.in_waiting > 0:
        data = ser.readline()  # 데이터 읽기
        print(data.decode("utf-8").strip())  # 수신된 데이터를 출력 (디코딩하여 출력)
