import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/leo11dk/Desktop/drobot/install/drobot_scan_3d'
