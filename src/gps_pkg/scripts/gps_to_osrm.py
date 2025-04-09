#!/usr/bin/env python3
import rospy
import requests
import math
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# 전역 변수
destination_lat = None
destination_lon = None
route_coords = []
steps = []
has_route = False
off_route_counter = 0
OFF_ROUTE_THRESHOLD = 0.0005
OFF_ROUTE_CONFIRM_COUNT = 3
path_pub = None

def distance(lat1, lon1, lat2, lon2):
    return math.sqrt((lat1 - lat2)**2 + (lon1 - lon2)**2)

def min_distance_to_route(current_lat, current_lon, route_coords):
    return min(distance(current_lat, current_lon, lat, lon) for lon, lat in route_coords)

def geocode_address(address):
    url = "https://nominatim.openstreetmap.org/search"
    params = {'q': address, 'format': 'json', 'limit': 1}
    headers = {'User-Agent': 'ros-gps-route-test'}
    try:
        res = requests.get(url, params=params, headers=headers)
        if res.status_code == 200 and res.json():
            lat = float(res.json()[0]['lat'])
            lon = float(res.json()[0]['lon'])
            rospy.loginfo(f"✅ 주소 '{address}' → 위도: {lat}, 경도: {lon}")
            return lat, lon
    except Exception as e:
        rospy.logwarn(f"주소 변환 실패: {e}")
    return None, None

def get_osrm_route(start_lat, start_lon, end_lat, end_lon):
    url = f"http://router.project-osrm.org/route/v1/walking/{start_lon},{start_lat};{end_lon},{end_lat}"
    params = {"overview": "full", "geometries": "geojson", "steps": "true"}
    try:
        res = requests.get(url, params=params)
        if res.status_code == 200:
            route = res.json()['routes'][0]
            return route['geometry']['coordinates'], route['legs'][0]['steps']
    except Exception as e:
        rospy.logwarn(f"OSRM 요청 실패: {e}")
    return [], []

def print_turn_instructions(step_data):
    rospy.loginfo("📢 길안내 지시:")
    for i, step in enumerate(step_data):
        m = step.get('maneuver', {})
        name = step.get('name', '(도로명 없음)')
        instr = f"[{i}] {m.get('type', '')} {m.get('modifier', '')} → {name}"
        rospy.loginfo(instr)

def publish_path():
    global route_coords
    rate = rospy.Rate(0.2)  # 5초마다 (0.2Hz)
    while not rospy.is_shutdown():
        if has_route and route_coords:
            path_msg = Path()
            path_msg.header.stamp = rospy.Time.now()
            path_msg.header.frame_id = "map"

            for lon, lat in route_coords:
                pose = PoseStamped()
                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "map"
                pose.pose.position.x = lon
                pose.pose.position.y = lat
                pose.pose.orientation.w = 1.0
                path_msg.poses.append(pose)

            path_pub.publish(path_msg)
            rospy.loginfo("🗺️ 경로 퍼블리시 완료 (/gps/route)")

        rate.sleep()

def gps_callback(msg):
    global has_route, route_coords, steps, off_route_counter
    current_lat = msg.latitude
    current_lon = msg.longitude

    if not has_route:
        return

    min_dist = min_distance_to_route(current_lat, current_lon, route_coords)

    if min_dist > OFF_ROUTE_THRESHOLD:
        off_route_counter += 1
        rospy.logwarn(f"📏 경로에서 거리: {min_dist:.6f} → 이탈 카운트: {off_route_counter}")
    else:
        off_route_counter = 0

    if off_route_counter >= OFF_ROUTE_CONFIRM_COUNT:
        rospy.logwarn("⚠️ 경로 이탈 감지됨. 경로 재계산 중...")
        route_coords, steps = get_osrm_route(current_lat, current_lon, destination_lat, destination_lon)
        print_turn_instructions(steps)
        off_route_counter = 0

def main():
    global destination_lat, destination_lon
    global route_coords, steps, has_route, path_pub

    rospy.init_node('osrm_route_planner')
    path_pub = rospy.Publisher("/gps/route", Path, queue_size=1)

    while not rospy.is_shutdown():
        dest = input("🎯 목적지 주소 입력 (예: 광교중앙역): ").strip()
        lat, lon = geocode_address(dest)
        if lat and lon:
            destination_lat = lat
            destination_lon = lon
            break

    rospy.Subscriber("/gps/fix", NavSatFix, gps_callback)
    rospy.loginfo("🛰️ GPS 위치 수신 대기 중...")

    while not rospy.is_shutdown():
        try:
            data = rospy.wait_for_message("/gps/fix", NavSatFix, timeout=5)
            current_lat = data.latitude
            current_lon = data.longitude
            route_coords, steps = get_osrm_route(current_lat, current_lon, destination_lat, destination_lon)
            if route_coords:
                has_route = True
                print_turn_instructions(steps)
                break
        except rospy.ROSException:
            rospy.logwarn("GPS 수신 실패, 재시도 중...")

    # 5초마다 퍼블리시 루프 실행
    publish_path()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
