import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/rayy2/ECE470Labs/lab_files/install/ece470labs'
