# robot_badook

#ssh 서버 연결
ssh sunmaan@172.20.10.6
qwer1234

#rviz, rqt 사용을 위한 디스플레이 설정
ssh -X sunmaan@172.20.10.6
export DISPLAY=172.20.10.2:0.0

#ttyS0 UART 포트에 권한주기
sudo chmod 666 /dev/ttyS0

#minicom으로 ttyS0에 들어오는 gps 정보 확인
minicom -D /dev/ttyS0 -b 9600

#ttyS0를 쓰고 있는 프로그램 모두 종료
sudo systemctl stop serial-getty@ttyS0.service
sudo systemctl disable serial-getty@ttyS0.service

#roscore 연결 - 다른 터미널
roscore

#코드에 권한주기
chmod +x ~/catkin_ws/src/gps_pkg/scripts/gps_publisher.py

#코드 실행
rosrun gps_pkg gps_publisher.py
