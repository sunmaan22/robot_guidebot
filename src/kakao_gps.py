import requests

def get_location_from_kakao(lat, lon):
    # 카카오맵 API URL
    url = "https://dapi.kakao.com/v2/local/geo/coord2address.json"
    
    # 카카오 API 키 (발급받은 키로 변경)
    api_key = 'e68a76ce44a72c8327a4835ef38b42af'
    
    # 요청 헤더에 API 키 추가
    headers = {
        'Authorization': f'KakaoAK {api_key}'
    }
    
    # 요청 파라미터
    params = {
        'x': lon,  # 경도
        'y': lat   # 위도
    }
    
    # API 호출
    response = requests.get(url, headers=headers, params=params)
    
    # 결과 확인
    if response.status_code == 200:
        data = response.json()
        if 'documents' in data and data['documents']:
            address = data['documents'][0]['address']
            print(f"Latitude: {lat}, Longitude: {lon}")
            print(f"Address: {address['address_name']}")
        else:
            print("No address found.")
    else:
        print(f"Error: {response.status_code}")

# GPS 좌표 예시
lat = 37.1698694
lon = 127.0265502

get_location_from_kakao(lat, lon)
