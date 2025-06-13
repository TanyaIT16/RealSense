import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tanya/Escritorio/Real_sense/RealSense/install/realsense_image_saver'
