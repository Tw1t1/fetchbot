import launch
from launch import LaunchDescription
from launch_ros.actions import Node
import subprocess
import glob

def get_device_name(device_path):
    try:
        result = subprocess.run(['v4l2-ctl', '--device', device_path, '--all'], 
                                capture_output=True, text=True)
        for line in result.stdout.split('\n'):
            if 'Card type' in line:
                return line.split(':')[1].strip()
    except:
        pass
    return "Unknown"

def find_usb_camera():
    video_devices = glob.glob('/dev/video*')
    for device in video_devices:
        name = get_device_name(device)
        if "GENERAL WEBCAM" in name:
            return device
    return None

def generate_launch_description():
    usb_camera_device = find_usb_camera()
    
    if usb_camera_device is None:
        print("No USB camera found!!")
        return LaunchDescription()
    
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='camera_sensor',
        parameters=[{
            'device': usb_camera_device,
            'image_size': [640, 480],
            'time_per_frame': [1, 6],
            'camera_frame_id': 'camera_link_optical',
        }],
        output='screen'
    )
    
    return LaunchDescription([
        camera_node
    ])