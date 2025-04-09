#!/usr/bin/env python3
import rospy
import requests
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

def osrm_get_route(start, end):
    """
    OSRM 공식 서버를 사용하여 경로 좌표 리스트를 반환합니다.
    start, end는 (lon, lat) 튜플
    """
    base_url = "https://router.project-osrm.org"
    coords = f"{start[0]},{start[1]};{end[0]},{end[1]}"
    url = f"{base_url}/route/v1/driving/{coords}?overview=full&geometries=geojson"

    try:
        response = requests.get(url)
        response.raise_for_status()
        data = response.json()
        route_coords = data['routes'][0]['geometry']['coordinates']
        return route_coords
    except Exception as e:
        rospy.logerr(f"OSRM 요청 실패: {e}")
        return []

def publish_path(route_coords, frame_id="map"):
    """
    받은 좌표 리스트를 nav_msgs/Path로 퍼블리시
    """
    pub = rospy.Publisher('/osrm_path', Path, queue_size=10)
    rospy.init_node('osrm_path_publisher', anonymous=True)
    rate = rospy.Rate(1)

    path_msg = Path()
    path_msg.header.frame_id = frame_id

    for lon, lat in route_coords:
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = lon  # 경도 → x
        pose.pose.position.y = lat  # 위도 → y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # 기본 방향
        path_msg.poses.append(pose)

    rospy.loginfo(f"총 {len(path_msg.poses)}개의 경로 포인트 퍼블리시")

    while not rospy.is_shutdown():
        path_msg.header.stamp = rospy.Time.now()
        pub.publish(path_msg)
        rate.sleep()

if __name__ == "__main__":
    # 예시: 서울역 → 시청역
    start = (126.9706, 37.5547)   # (lon, lat)
    end = (126.9770, 37.5663)     # (lon, lat)

    try:
        coords = osrm_get_route(start, end)
        if coords:
            publish_path(coords)
        else:
            rospy.logwarn("경로를 받아오지 못했습니다.")
    except rospy.ROSInterruptException:
        pass
