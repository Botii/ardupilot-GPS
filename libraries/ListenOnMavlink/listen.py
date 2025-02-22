from pymavlink import mavutil
import time

# Connect to MAVLink (UDP)
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')  # Use new port

# Wait for heartbeat
master.wait_heartbeat()
print("Connected to MAVLink!")

# Fake GPS coordinates (start position)
lat = -35.3632608 * 1e7
lon = 149.1652352 * 1e7
alt = 100 * 1000  # 100 meters in mm

while True:
    # Move latitude slightly every second (simulating movement)
    lat += 10  # Move 1mm per loop

    # Send fake GPS data
    master.mav.gps_raw_int_send(
        int(time.time() * 1e6),  # Timestamp in microseconds
        3,  # Fix type (3 = 3D fix)
        int(lat),  # Latitude
        int(lon),  # Longitude
        int(alt),  # Altitude (mm)
        100,  # HDOP
        100,  # VDOP
        10,  # Velocity (cm/s)
        0,  # Heading (degrees * 100)
        11,  # Satellites visible
        alt,  # Ellipsoid height in mm
        500,  # Horizontal accuracy in mm
        500,  # Vertical accuracy in mm
        500,  # Velocity accuracy in mm/s
        36000,  # Heading accuracy * 100
        18000  # GPS Yaw * 100
    )

    print(f"Sent Moving GPS: Lat={lat / 1e7}, Lon={lon / 1e7}, Alt={alt / 1000}m")

    time.sleep(1)  # Send every second
