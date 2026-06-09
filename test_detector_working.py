#!/usr/bin/env python3
"""
Working detector test - launches SITL, MAVProxy, and tests GPS spoofing detection
"""

import time
import sys
import subprocess
import os
from pymavlink import mavutil

def test_detector():
    # Connect to SITL through MAVProxy (MAVProxy on port 14550, we connect to its forwarded port 14551)
    print("Connecting to SITL through MAVProxy on 127.0.0.1:14551...")
    mav = mavutil.mavlink_connection('127.0.0.1:14551', baud=115200, timeout=30)

    # Wait for heartbeat
    print("Waiting for heartbeat...")
    try:
        mav.wait_heartbeat(timeout=10)
    except Exception as e:
        print(f"ERROR: No heartbeat received: {e}")
        return 3

    print(f"Connected to vehicle {mav.sysid}")
    time.sleep(1)

    # Set parameters to clear any previous spoofing
    print("\n[1] Clearing any previous spoofing parameters...")
    mav.param_set_send('SIM_GPS1_VERR_X', 0)
    mav.param_set_send('SIM_GPS1_VERR_Y', 0)
    time.sleep(2)

    # Set mode to LOITER (mode 5 = LOITER)
    print("[2] Setting mode to LOITER...")
    mav.mav.set_mode_send(mav.target_system, 1, 5)
    time.sleep(2)

    # Arm using MAV_CMD_COMPONENT_ARM_DISARM (command 400)
    print("[3] Arming...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        400,  # MAV_CMD_COMPONENT_ARM_DISARM
        0,    # confirmation
        1,    # param1: 1=arm, 0=disarm
        0, 0, 0, 0, 0, 0
    )
    time.sleep(3)

    # Takeoff using MAV_CMD_NAV_TAKEOFF (command 22)
    print("[4] Taking off to 15m...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        22,   # MAV_CMD_NAV_TAKEOFF
        0,    # confirmation
        0,    # param1: pitch (0=default)
        0,    # param2: empty
        0,    # param3: empty
        0,    # param4: yaw (0=default)
        0,    # param5: latitude (0=current)
        0,    # param6: longitude (0=current)
        15    # param7: altitude in meters
    )
    time.sleep(5)

    # Wait for hover
    print("[5] Waiting for hover to stabilize (20 seconds)...")
    for i in range(20):
        msg = mav.recv_match(type='STATUSTEXT', blocking=False)
        if msg:
            text = msg.text.decode() if isinstance(msg.text, bytes) else msg.text
            if 'GSPF' in text:
                print(f"  {text}")
        time.sleep(1)

    # Inject spoofing
    print("\n[6] INJECTING SPOOFING...")
    print("  Setting SIM_GPS1_VERR_X = 1.5 m/s")
    print("  Setting SIM_GPS1_VERR_Y = 0.75 m/s")
    mav.param_set_send('SIM_GPS1_VERR_X', 1.5)
    time.sleep(1)
    mav.param_set_send('SIM_GPS1_VERR_Y', 0.75)
    time.sleep(1)

    # Monitor for GSPF messages
    print("\n[7] Monitoring for detector messages (60 seconds)...")
    gspf_messages = []
    start_time = time.time()

    while time.time() - start_time < 60:
        msg = mav.recv_match(type='STATUSTEXT', blocking=False)
        if msg:
            text = msg.text.decode() if isinstance(msg.text, bytes) else msg.text
            if 'GSPF' in text or 'AP:' in text:
                elapsed = time.time() - start_time
                print(f"[{elapsed:.1f}s] {text}")
                if "GSPF" in text:
                    gspf_messages.append((elapsed, text))
        time.sleep(0.1)

    # Remove spoofing
    print("\n[8] REMOVING SPOOFING...")
    mav.param_set_send('SIM_GPS1_VERR_X', 0)
    time.sleep(1)
    mav.param_set_send('SIM_GPS1_VERR_Y', 0)
    time.sleep(2)

    # Land
    print("[9] Landing...")
    mav.mav.command_long_send(
        mav.target_system,
        mav.target_component,
        22,   # MAV_CMD_NAV_LAND
        0, 0, 0, 0, 0, 0, 0
    )
    time.sleep(30)

    # Summary
    print("\n" + "="*60)
    print("DETECTOR MESSAGES RECEIVED:")
    print("="*60)
    if gspf_messages:
        for time_s, msg in gspf_messages:
            print(f"  {time_s:.1f}s: {msg}")

        # Check for states
        has_suspicious = any("SUSPICIOUS" in m for _, m in gspf_messages)
        has_confirmed = any("CONFIRMED" in m for _, m in gspf_messages)

        print("\nDetection Results:")
        print(f"  SUSPICIOUS detected: {has_suspicious}")
        print(f"  CONFIRMED detected: {has_confirmed}")

        if has_suspicious and has_confirmed:
            print("\n✓ TEST PASSED - Detector working correctly!")
            return 0
        elif has_suspicious:
            print("\n⚠ PARTIAL - SUSPICIOUS detected but not CONFIRMED")
            return 1
        else:
            print("\n✗ TEST FAILED - No GSPF messages detected")
            return 2
    else:
        print("  No GSPF messages received")
        print("\n✗ TEST FAILED - Detector not responding")
        return 2

if __name__ == '__main__':
    # Launch SITL in background
    sitl_bin = "./build/sitl/bin/arducopter"
    if not os.path.exists(sitl_bin):
        print(f"ERROR: SITL binary not found at {sitl_bin}")
        sys.exit(1)

    print("Launching SITL...")
    sitl_log = open('/tmp/sitl_working_test.log', 'w')
    sitl_proc = subprocess.Popen(
        [sitl_bin, '-S', '-I0',
         '--home=-35.362938,149.165085,584.1,0',
         '--model=quad', '--speedup=1'],
        stdout=sitl_log,
        stderr=subprocess.STDOUT
    )

    # Give SITL time to bind to port
    time.sleep(2)

    # Launch MAVProxy to bridge SITL and our script
    print("Launching MAVProxy bridge...")
    mavproxy_log = open('/tmp/mavproxy_working_test.log', 'w')
    mavproxy_proc = subprocess.Popen(
        ['python3', '-m', 'MAVProxy.mavproxy',
         '--master=127.0.0.1:14550',
         '--out=127.0.0.1:14551',
         '--logfile=/tmp/mavproxy_bridge.log'],
        stdout=mavproxy_log,
        stderr=subprocess.STDOUT
    )

    # Wait for MAVProxy to connect to SITL
    time.sleep(3)

    try:
        result = test_detector()
        sys.exit(result)
    except Exception as e:
        print(f"\n✗ ERROR: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(3)
    finally:
        # Kill processes
        print("\nCleaning up...")
        for proc in [mavproxy_proc, sitl_proc]:
            try:
                proc.terminate()
                proc.wait(timeout=3)
            except:
                try:
                    proc.kill()
                except:
                    pass
        sitl_log.close()
        mavproxy_log.close()
        print("SITL log: /tmp/sitl_working_test.log")
        print("MAVProxy log: /tmp/mavproxy_working_test.log")
        print("Bridge log: /tmp/mavproxy_bridge.log")
