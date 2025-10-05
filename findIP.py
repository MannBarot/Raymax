import subprocess
import os

def get_reachy_ip(interface='wlp0s20f3', set_env=True):
    try:
        # Run the ifconfig command and extract the IP
        result = subprocess.run(
            f"ifconfig {interface} 2>/dev/null | grep -oP 'inet \\K\\d+\\.\\d+\\.\\d+\\.\\d+'",
            shell=True, capture_output=True, text=True
        )
        
        ip_address = result.stdout.strip()
        
        if not ip_address:
            # Check if interface exists
            check_interface = subprocess.run(
                f"ifconfig | grep -q {interface}",
                shell=True
            )
            if check_interface.returncode == 0:
                print(f"{interface} exists but has no IP address assigned.")
            else:
                print(f"{interface} not found.")
            return None
        
        print(f"Reachy's IP address: {ip_address}")
        
        if set_env:
            os.environ['REACHY_IP'] = ip_address
            print("Environment variable REACHY_IP set.")
        
        return ip_address
    
    except Exception as e:
        print(f"Error: {e}")
        return None
