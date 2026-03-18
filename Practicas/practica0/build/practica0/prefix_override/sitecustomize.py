import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/arrgusr/ROS2Dev/ws_sem26_2/src/practica0/install/practica0'
