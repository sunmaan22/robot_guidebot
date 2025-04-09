import serial
import time

port = "/dev/ttyS0"
baud = 9600
timeout = 1

ser = serial.Serial(port, baud, timeout=timeout)
print("Serial port opened successfully.")

def parse_gpgga(line):
    try:
        parts = line.split(',')
        if len(parts) >= 6 and parts[2] and parts[4]:
            lat_raw = float(parts[2])
            lon_raw = float(parts[4])

            lat_deg = int(lat_raw / 100)
            lat_min = lat_raw - (lat_deg * 100)
            lat = lat_deg + (lat_min / 60)

            lon_deg = int(lon_raw / 100)
            lon_min = lon_raw - (lon_deg * 100)
            lon = lon_deg + (lon_min / 60)

            if parts[3] == 'S':
                lat = -lat
            if parts[5] == 'W':
                lon = -lon

            return lat, lon
    except ValueError as e:
        print(f"Error parsing GPGGA: {e}")
    return None, None

def main():
    try:
        while True:
            try:
                # 데이터가 있을 때만 읽음
                if ser.in_waiting:
                    line = ser.readline().decode('ascii', errors='ignore').strip()

                    if not line or line.startswith('$GPTXT'):
                        continue

                    if line.startswith('$GPGGA'):
                        lat, lon = parse_gpgga(line)
                        if lat and lon:
                            print(f"Latitude: {lat:.7f}, Longitude: {lon:.7f}")
                else:
                    time.sleep(0.05)  # CPU 과점유 방지
            except serial.SerialException as e:
                print(f"SerialException: {e}")
                break
            except Exception as e:
                print(f"Read error: {e}")
    except KeyboardInterrupt:
        print("Exiting program.")
    finally:
        ser.close()
        print("Serial port closed.")

if __name__ == "__main__":
    main()
