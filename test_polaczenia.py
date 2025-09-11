from pymavlink import mavutil, mavwp
tcp_ip = "192.168.2.108"  # Mission Planner host address
tcp_port = 14550
master = mavutil.mavlink_connection(f'tcp:{tcp_ip}:{tcp_port}')
master.wait_heartbeat(blocking=True)                                       
wp = mavwp.MAVWPLoader()                                                    
seq = 1
frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
radius = 10
for i in range(4):                  
            wp.add(mavutil.mavlink.MAVLink_mission_item_message(master.target_system,
                         master.target_component,
                         seq,
                         frame,
                         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                         0, 0, 0, radius, 0, 0,
                         lat[i],lon[i],alt[i]))
            seq += 1                                                                       

master.waypoint_clear_all_send()                                     
master.waypoint_count_send(wp.count())                          

for i in range(wp.count()):
            msg = master.recv_match(type=['MISSION_REQUEST'],blocking=True)             
            master.mav.send(wp.wp(msg.seq))                                                                      
            print ('Sending waypoint {0}').format(msg.seq)             