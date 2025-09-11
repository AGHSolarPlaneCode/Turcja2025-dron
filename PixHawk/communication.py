from pymavlink import mavutil
import time
from typing import Tuple, List, Dict, Optional


class PixHawkService:
    """
    Example usage:
    # Initialize connection\n
    drone = PixHawkService()

    waypoints = [
        {"lat": -35.363261, "lon": 149.165230, "alt": 20},
        {"lat": -35.363500, "lon": 149.165500, "alt": 25},
        {"lat": -35.363800, "lon": 149.165800, "alt": 20}
    ]

    try:
        # Upload mission\n
        if drone.set_waypoints(waypoints):
            print("Mission uploaded successfully")
            # Start mission\n
            if drone.start_mission():
                print("Mission started - monitoring progress...")
                # Monitor for 60 seconds\n
                for _ in range(30):
                    drone.monitor_mission()
                    time.sleep(2)
        else:
            print("Failed to upload mission")
    except KeyboardInterrupt:
        print("\nMission monitoring stopped by user")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        drone.disarm()
        drone.close()
    """

    def __init__(self, device: str = "/dev/ttyAMA0", baud: int = 57600):
        """
        Connects to Pixhawk via MAVLink.

        :param device: string to device (eg. '/dev/ttyAMA0')
        :param baud: baud rate (default 57600)
        """
        try:
            self.master = mavutil.mavlink_connection(device, baud=baud)
            self.master.wait_heartbeat()
            print(f"Connected to system {self.master.target_system}, component {self.master.target_component}")
        except Exception as e:
            print(f"Connection failed: {e}")
            raise

    def get_attitude(self, timeout=1):
        msg_att = self.master.recv_match(type='ATTITUDE', blocking=True, timeout=timeout)
        if msg_att is None:
            return None
        roll  = msg_att.roll    # rad +prawo -lewo
        pitch = msg_att.pitch   # rad +góra  -dół
        yaw   = msg_att.yaw     # rad +prawo -lewo
        return roll, pitch, yaw
    
    def get_current_coordinates(self) -> Optional[Tuple[float, float, float]]:
        """
        Reads current GPS position (lat, lon, alt).

        :return: Tuple of (latitude, longitude, altitude) or None if no GPS data
        """
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if not msg:
            print("No GPS data received.")
            return None
        return msg.lat / 1e7, msg.lon / 1e7, msg.alt / 1000.0

    def is_armed(self) -> bool:
        """
        Checks if the drone is armed (returns True/False).

        :return: True if armed, False if disarmed or no heartbeat
        """
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if msg:
            return (msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) != 0
        return False

    def get_current_mode(self) -> str:
        """
        Gets the current flight mode.

        :return: current flight mode or 'UNKNOWN'
        """
        msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
        if msg:
            # Convert mode number to mode name
            mode_mapping = self.master.mode_mapping()
            for mode_name, mode_id in mode_mapping.items():
                if mode_id == msg.custom_mode:
                    return mode_name
        return "UNKNOWN"

    def get_mission(self) -> List[Dict[str, float]]:
        """
        Downloads mission as list of dicts with lat/lon/alt.

        :return: List of dicts with seq/lat/lon/alt/command
        """
        mission = []
        self.master.mav.mission_request_list_send(self.master.target_system, self.master.target_component)
        msg = self.master.recv_match(type='MISSION_COUNT', blocking=True, timeout=5)
        if not msg:
            print("No mission count received")
            return mission

        count = msg.count
        print(f"Mission has {count} waypoints")

        for i in range(count):
            self.master.mav.mission_request_send(self.master.target_system, self.master.target_component, i)
            item = self.master.recv_match(type='MISSION_ITEM', blocking=True, timeout=5)
            if item:
                mission.append({
                    "seq": item.seq,
                    "lat": item.x,
                    "lon": item.y,
                    "alt": item.z,
                    "command": item.command
                })
        return mission

    def set_waypoints(self, waypoints: List[Dict[str, float]]) -> bool:
        """
        Uploads a mission (clears old waypoints).

        :param waypoints: New waypoints to upload
        :return: True if mission was uploaded successfully, False otherwise
        """
        # Clear existing mission
        #self.master.mav.mission_clear_all_send(self.master.target_system, self.master.target_component)

        # Wait for clear acknowledgment
        #clear_ack = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        #if not clear_ack or clear_ack.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
        #    print("Failed to clear mission")
        #    return False

        # Send mission count
    # Wyślij ilość elementów w misji
        self.master.mav.mission_count_send(
            self.master.target_system,
            self.master.target_component,
            len(waypoints)
        )

        for i, wp in enumerate(waypoints):
            # Czekamy na żądanie przesłania waypointa
            req = self.master.recv_match(type='MISSION_REQUEST', blocking=True, timeout=5)
            if not req:
                print(f"No mission request received for waypoint {i}")
                return False

            if wp["cmd"] == "NAV":
                self.master.mav.mission_item_send(
                    self.master.target_system,
                    self.master.target_component,
                    i,  # sequence number
                    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    1 if i == 0 else 0,  # current
                    1,  # autocontinue
                    0, wp["acr"], 0, 0,  # param1-4 (hold time, acceptance radius, etc.)
                    wp["lat"],
                    wp["lon"],
                    wp["alt"]
                )

            elif wp["cmd"] == "SET_SERVO":
                self.master.mav.mission_item_send(
                    self.master.target_system,
                    self.master.target_component,
                    i,
                    mavutil.mavlink.MAV_FRAME_MISSION,  # frame for DO commands
                    mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                    0,  # current = 0 (nie ustawia aktualnej pozycji)
                    1,  # autocontinue
                    wp["ch"],   # param1 = channel
                    wp["pwm"],       # param2 = PWM
                    0, 0, 0,         # param3-5 (unused)
                    0, 0, 0          # x, y, z not used for this command
                )

            else:
                print(f"Unknown command in waypoint {i}: {wp['command']}")
                return False

        # Czekamy na potwierdzenie
        ack = self.master.recv_match(type='MISSION_ACK', blocking=True, timeout=5)
        if ack and ack.type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            print(f"Mission upload successful: {len(waypoints)} items")
            return True
        else:
            print(f"Mission upload failed: {ack.type if ack else 'timeout'}")
            return False

    def append_waypoints(self, new_wps: List[Dict[str, float]]) -> bool:
        """
        Appends new waypoints to existing mission.

        :param new_wps: List of waypoint dicts to append
        :return: True if mission was updated successfully, False otherwise
        """
        mission = self.get_mission()
        mission.extend(new_wps)
        print(f"Appending {len(new_wps)} waypoints (new total {len(mission)})")
        return self.set_waypoints(mission)

    def prepend_waypoint(self, new_wp: Dict[str, float]) -> bool:
        """
        Prepends a waypoint at the beginning of mission.

        :param new_wp: Waypoint dict to prepend
        :return: True if mission was updated successfully, False otherwise
        """
        mission = self.get_mission()
        mission.insert(0, new_wp)
        print(f"Prepended waypoint, new total {len(mission)}")
        return self.set_waypoints(mission)

    def set_current_waypoint(self, index: int) -> bool:
        """
        Sets current waypoint index (0-based).

        :param index: Waypoint index to set as current
        :return: True if waypoint was set successfully, False otherwise
        """
        self.master.mav.mission_set_current_send(self.master.target_system, self.master.target_component, index)

        # Wait for acknowledgment
        ack = self.master.recv_match(type='MISSION_CURRENT', blocking=True, timeout=5)
        if ack and ack.seq == index:
            print(f"Successfully set current waypoint to index {index}")
            return True
        else:
            print(f"Failed to set current waypoint to index {index}")
            return False

    def arm(self) -> bool:
        """
        Arms the drone (if not armed).

        :return: True if drone is armed successfully, False otherwise
        """
        if self.is_armed():
            print("Drone already armed")
            return True

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            1,  # arm (1=arm, 0=disarm)
            0, 0, 0, 0, 0, 0  # unused parameters
        )

        # Wait for acknowledgment
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Arm command accepted")
            # Wait a moment and check if actually armed
            time.sleep(1)
            if self.is_armed():
                print("Drone successfully armed")
                return True
            else:
                print("Arm command sent but drone not armed")
                return False
        else:
            print(f"Arm command failed: {ack.result if ack else 'timeout'}")
            return False

    def disarm(self) -> bool:
        """
        Disarms the drone (if armed).

        :return: True if drone is disarmed successfully, False otherwise
        """
        if not self.is_armed():
            print("Drone already disarmed")
            return True

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            0,  # disarm (1=arm, 0=disarm)
            0, 0, 0, 0, 0, 0  # unused parameters
        )

        # Wait for acknowledgment
        ack = self.master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("Disarm command accepted")
            time.sleep(1)
            if not self.is_armed():
                print("Drone successfully disarmed")
                return True
            else:
                print("Disarm command sent but drone still armed")
                return False
        else:
            print(f"Disarm command failed: {ack.result if ack else 'timeout'}")
            return False

    def set_mode(self, mode_name: str) -> bool:
        """
        Sets the flight mode (e.g., 'AUTO', 'GUIDED').

        :param mode_name: Flight mode name to set
        :return: True if mode was set successfully, False otherwise
        """
        mode_id = self.master.mode_mapping().get(mode_name)
        if mode_id is None:
            print(f"Unknown mode: {mode_name}")
            available_modes = list(self.master.mode_mapping().keys())
            print(f"Available modes: {available_modes}")
            return False

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )

        # Verify mode change
        time.sleep(0.5)
        current_mode = self.get_current_mode()
        if current_mode == mode_name:
            print(f"Successfully set mode to {mode_name}")
            return True
        else:
            print(f"Mode change failed. Current mode: {current_mode}")
            return False

    def check_prearm_status(self) -> bool:
        """
        Checks GPS, EKF, and system status before arming.

        :return: True if drone is ready for arming, False otherwise
        """
        checks_passed = True

        # GPS check
        gps = self.master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
        if not gps or gps.fix_type < 3:
            print("GPS not ready (need 3D fix)")
            checks_passed = False
        else:
            print(f"GPS ready (fix type: {gps.fix_type}, satellites: {gps.satellites_visible})")

        # System status
        sys_status = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
        if sys_status:
            # Check individual sensor health bits
            sensors = sys_status.onboard_control_sensors_health
            if sensors & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO:
                print("Gyroscope healthy")
            else:
                print("Gyroscope not healthy")
                checks_passed = False

            if sensors & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL:
                print("Accelerometer healthy")
            else:
                print("Accelerometer not healthy")
                checks_passed = False
        else:
            print("No system status received")
            checks_passed = False

        # EKF check (if available)
        ekf = self.master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=2)
        if ekf:
            if ekf.flags & 0x01:  # EKF solution OK bit
                print("EKF stable")
            else:
                print("EKF not stable")
                checks_passed = False
        else:
            print("EKF status not available (may be normal)")

        # Battery check
        battery = self.master.recv_match(type='SYS_STATUS', blocking=True, timeout=2)
        if battery and hasattr(battery, 'voltage_battery'):
            voltage = battery.voltage_battery / 1000.0  # Convert from mV to V
            if voltage > 10.5:  # Minimum safe voltage for most setups
                print(f"Battery voltage OK: {voltage:.1f}V")
            else:
                print(f"Battery voltage low: {voltage:.1f}V")
                checks_passed = False

        if checks_passed:
            print("All pre-arm checks passed")
        else:
            print("Some pre-arm checks failed")

        return checks_passed

    def start_mission(self, start_index: int = 0) -> bool:
        """
        Runs pre-arm checks, arms drone, sets AUTO mode,
        and starts the mission from the given waypoint index.

        :param start_index: Waypoint index to start from (0-based)
        :return: True if mission started successfully, False otherwise
        """
        print("Starting mission sequence...")

        if not self.check_prearm_status():
            print("Pre-arm checks failed - mission not started")
            return False

        # Check if mission exists
        mission = self.get_mission()
        if not mission:
            print("No mission loaded")
            return False

        if start_index >= len(mission):
            print(f"Start index {start_index} exceeds mission length {len(mission)}")
            return False

        # Arm the drone
        if not self.arm():
            print("Failed to arm drone")
            return False

        # Set AUTO mode
        if not self.set_mode("AUTO"):
            print("Failed to set AUTO mode")
            return False

        # Set starting waypoint
        if not self.set_current_waypoint(start_index):
            print(f"Failed to set starting waypoint {start_index}")
            return False

        print(f"Mission started successfully from waypoint {start_index}")
        return True


    def emergency_stop(self) -> None:
        """
        Emergency stop - disarm immediately.

        Forces immediate disarm without waiting for acknowledgment.
        """
        print("EMERGENCY STOP - Disarming drone")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # confirmation
            0,  # disarm
            21196,  # force disarm magic number
            0, 0, 0, 0, 0
        )

    def get_mission_status(self) -> Optional[Dict[str, int]]:
        """
        Gets current mission progress.

        Returns:
            list with [ current_waypoint, total_waypoints] None if no data
        """
        current = self.master.recv_match(type='MISSION_CURRENT', blocking=True, timeout=3)
        print(current)
        if current:
            return (
                current.seq,
                current.total
            )
        return None

    def wait_for_command_ack(self, command: int, timeout: int = 5) -> bool:
        """
        Waits for command acknowledgment.

        :param command: MAVLink command ID to wait for
        :param timeout: Timeout in seconds
        :return: True if command was acknowledged successfully, False otherwise
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            ack = self.master.recv_match(type='COMMAND_ACK', blocking=False)
            if ack and ack.command == command:
                return ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED
            time.sleep(0.1)
        return False

    def monitor_mission(self, update_interval: int = 2) -> Dict:
        """
        Monitors mission progress in real-time.
        Call this in a loop to track mission status.

        :param update_interval: Update interval in seconds (not used in current implementation)
        :return: Dict with coordinates, mission_status, mode, and armed status
        """
        coords = self.get_current_coordinates()
        status = self.get_mission_status()
        mode = self.get_current_mode()
        armed = self.is_armed()

        if coords and status:
            print(f"Position: {coords[0]:.6f}, {coords[1]:.6f}, Alt: {coords[2]:.1f}m")
            print(
                f"Mission: {status['current_waypoint']}/{status['total_waypoints'] - 1} | Mode: {mode} | Armed: {armed}")

        return {
            "coordinates": coords,
            "mission_status": status,
            "mode": mode,
            "armed": armed
        }

    def close(self) -> None:
        """
        Closes the MAVLink connection.

        Should be called when finished to properly cleanup resources.
        """
        if hasattr(self, 'master'):
            self.master.close()
            print("MAVLink connection closed")

if __name__ == "__main__": 
    pix = PixHawkService()
    t=time.time()
    ret = pix.get_mission_status()
    print(time.time()-t)
    print('gioakjsfcnSKIEJFVWkf')