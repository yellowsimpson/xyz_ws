import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/deepet/Desktop/autofuel_system/doosan-robot2/install/dsr_visualservoing'
