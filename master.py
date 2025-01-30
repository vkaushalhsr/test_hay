#!/usr/bin/env python3
import os
import subprocess
import rospy
from std_msgs.msg import Bool, String
import time
import roslaunch
from serial.tools import list_ports
import dynamic_reconfigure.client
import signal
import configparser

ROBOT = "VIOLET"
config_path = "/haystack_disinfect_report/robot_config.ini"
HAYSTACK_DIR = "/haystack_disinfect_report/"
if not os.path.exists(HAYSTACK_DIR):
    os.makedirs(HAYSTACK_DIR)
if os.path.exists(config_path):
    config = configparser.ConfigParser()
    config.read(config_path)
    if config.has_option('ROBOT', 'SERIAL_NO'):
        serial_no = config.get('ROBOT', 'SERIAL_NO')
        if "HSRUVC" in serial_no:
            ROBOT = "VIOLET"
        if "HSRPRPL" in serial_no:
            ROBOT = "PURPLE"
if ROBOT == "PURPLE":
    CAMERA_1 = 'CH1T831001V'
    CAMERA_2 = 'CH1T831006N'
    REEMAN = "NEW"
    if os.path.exists(config_path):
        config = configparser.ConfigParser()
        config.read(config_path)
        CAMERA_1 = config.get('ROBOT', 'DEPTH_CAMERA_1')
        CAMERA_2 = config.get('ROBOT', 'DEPTH_CAMERA_2')
        if config.has_option('ROBOT', 'REEMAN'):
            REEMAN = config.get('ROBOT', 'REEMAN')

SHUTDOWN_TOPIC = "/shutdown"
DOSIMETER_CONFIG_TOPIC = "/haystack/dosimeter_config_data"
LOCK_CONFIG_TOPIC = "/haystack/lock_config_data"
SHUTDOWN_STATUS = "/haystack/shutdown_status"
DEVICE_STATUS = "/haystack/device_status"
LAUNCH_STATUS = "/haystack/launch_status"
INITIALIZATION_STATUS = "/init_pub"

TAG_PARAM_NAME = '/image_tag'
TIMEZONE_TOPIC = "/haystack/timezone"
if ROBOT == "VIOLET":
    PERIPHERAL_INFO = '/jetson_info'
    ROBOT_INFO = '/violet_info'
    PERIPHERAL_IP = {
        "jetson": "10.42.0.1",
        "rp": "10.42.0.2",
    }
    PERIPHERAL = "JETSON"
elif ROBOT == "PURPLE":
    PERIPHERAL_INFO = '/intel_info'
    ROBOT_INFO = '/purple_info'
    PERIPHERAL_IP = {
        "intel": "192.168.10.1",
        "jetson": "192.168.10.104",
        "rp": "192.168.10.103",
        "lidar": "192.168.10.160"
    }
    PERIPHERAL = "INTEL"
ERROR_CODE = {"NULL": "000", "LAUNCH_FAILED": "001", "DEVICE_NOT_FOUND": "002"}

CAMERA_TOPIC_LIST = [['/camera/realsense2_camera_manager/bond', 'bond/Status'],
                     ['/camera/stereo_module/parameter_descriptions', 'dynamic_reconfigure/ConfigDescription'],
                     ['/camera/stereo_module/parameter_updates', 'dynamic_reconfigure/Config'],
                     ['/camera/rgb_camera/parameter_descriptions', 'dynamic_reconfigure/ConfigDescription'],
                     ['/camera/rgb_camera/parameter_updates', 'dynamic_reconfigure/Config'],
                     ['/camera/motion_module/parameter_descriptions', 'dynamic_reconfigure/ConfigDescription'],
                     ['/camera/motion_module/parameter_updates', 'dynamic_reconfigure/Config'],
                     ['/camera/depth/image_rect_raw', 'sensor_msgs/Image'],
                     ['/camera/depth/camera_info', 'sensor_msgs/CameraInfo'],
                     ['/camera/depth/metadata', 'realsense2_camera/Metadata'],
                     ['/camera/color/image_raw', 'sensor_msgs/Image'],
                     ['/camera/color/camera_info', 'sensor_msgs/CameraInfo'],
                     ['/camera/color/metadata', 'realsense2_camera/Metadata'],
                     ['/camera/extrinsics/depth_to_color', 'realsense2_camera/Extrinsics'],
                     ['/camera/stereo_module/auto_exposure_roi/parameter_descriptions',
                      'dynamic_reconfigure/ConfigDescription'],
                     ['/camera/stereo_module/auto_exposure_roi/parameter_updates', 'dynamic_reconfigure/Config'],
                     ['/camera/rgb_camera/auto_exposure_roi/parameter_descriptions',
                      'dynamic_reconfigure/ConfigDescription'],
                     ['/camera/rgb_camera/auto_exposure_roi/parameter_updates', 'dynamic_reconfigure/Config']]

UBIQUITY_TOPIC_LIST = [['/ubiquity_velocity_controller/parameter_descriptions',
                        'dynamic_reconfigure/ConfigDescription'],
                       ['/ubiquity_velocity_controller/parameter_updates', 'dynamic_reconfigure/Config'],
                       ['/motor_node/parameter_descriptions', 'dynamic_reconfigure/ConfigDescription'],
                       ['/motor_node/parameter_updates', 'dynamic_reconfigure/Config']]

REEMAN_MOTOR_TOPIC_LIST = [['/tf', 'tf2_msgs/TFMessage'],
                           ['/tf_static', 'tf2_msgs/TFMessage'],
                           ['/odom_combined', 'geometry_msgs/PoseWithCovarianceStamped'],
                           ['/yoyo_vel_smoother/parameter_descriptions', 'dynamic_reconfigure/ConfigDescription'],
                           ['/yoyo_vel_smoother/parameter_updates', 'dynamic_reconfigure/Config'],
                           ['/input/nav_cmd_vel', 'geometry_msgs/Twist'],
                           ['/mobile_base_nodelet_manager/bond', 'bond/Status'],
                           ['/yoyo_cmd_vel_mux/parameter_descriptions', 'dynamic_reconfigure/ConfigDescription'],
                           ['/yoyo_cmd_vel_mux/parameter_updates', 'dynamic_reconfigure/Config'],
                           ['/mobile_base/commands/velocity', 'geometry_msgs/Twist'],
                           ['/yoyo_cmd_vel_mux/active', 'std_msgs/String'],
                           ['/joint_states', 'sensor_msgs/JointState'],
                           ['/mobile_base/version_info', 'driver_msgs/VersionInfo'],
                           ['/mobile_base/events/button', 'driver_msgs/ButtonEvent'],
                           ['/mobile_base/events/bumper', 'driver_msgs/BumperEvent'],
                           ['/mobile_base/events/cliff', 'driver_msgs/CliffEvent'],
                           ['/mobile_base/events/charger', 'driver_msgs/PowerSystemEvent'],
                           ['/mobile_base/events/battery', 'driver_msgs/PowerSystemEvent'],
                           ['/mobile_base/events/digital_input', 'driver_msgs/DigitalInputEvent'],
                           ['/mobile_base/events/online', 'driver_msgs/RobotStateEvent'],
                           ['/mobile_base/sensors/core', 'driver_msgs/SensorState'],
                           ['/mobile_base/sensors/dock_ir', 'driver_msgs/DockInfraRed'],
                           ['/mobile_base/debug/raw_data_command', 'std_msgs/String'],
                           ['/mobile_base/debug/raw_data_stream', 'std_msgs/String'],
                           ['/mobile_base/debug/raw_control_command', 'std_msgs/Int16MultiArray'],
                           ['/mobile_base/upgrade_result', 'std_msgs/String'],
                           ['/mobile_base/events/power_off', 'std_msgs/Int32'],
                           ['/mobile_base/events/battery_info', 'driver_msgs/BatteryInfo'],
                           ['/mobile_base/events/current_info', 'driver_msgs/CurrentInfo'],
                           ['/mobile_base/events/auth_info', 'std_msgs/Int32'],
                           ['/mobile_base/events/wheel_status', 'driver_msgs/WheelStatus'],
                           ['/mobile_base/events/range_sensor', 'driver_msgs/RangeSensorEvent'],
                           ['/mobile_base/debug/send_data_app', 'std_msgs/String'],
                           ['/mobile/odom', 'nav_msgs/Odometry'],
                           ['/odom', 'nav_msgs/Odometry'],
                           ['/input/safety_controller', 'geometry_msgs/Twist'],
                           ['/mobile_base/commands/force_stop', 'std_msgs/Int32']]

DEPTH_CAMERA_TOPIC_LIST = [['/camera/scan', 'sensor_msgs/LaserScan'],
                           ['/filter/voxel', 'sensor_msgs/PointCloud2'],
                           ['/filter/ground', 'sensor_msgs/PointCloud2'],
                           ['/filter/merge', 'sensor_msgs/PointCloud2'],
                           ['/camera_02/depth/image_raw', 'sensor_msgs/Image'],
                           ['/camera_02/depth/camera_info', 'sensor_msgs/CameraInfo'],
                           ['/camera_02/extrinsic/depth_to_color', 'astra_camera/Extrinsics'],
                           ['/camera_02/depth/points', 'sensor_msgs/PointCloud2'],
                           ['/camera_02/depth_registered/points', 'sensor_msgs/PointCloud2'],
                           ['/camera_01/depth/image_raw', 'sensor_msgs/Image'],
                           ['/camera_01/depth/camera_info', 'sensor_msgs/CameraInfo'],
                           ['/camera_01/extrinsic/depth_to_color', 'astra_camera/Extrinsics'],
                           ['/camera_01/depth/points', 'sensor_msgs/PointCloud2'],
                           ['/camera_01/depth_registered/points', 'sensor_msgs/PointCloud2']]

LIDAR_TOPIC_LIST = [['/scan', 'sensor_msgs/LaserScan']]

UBIQUITY_VENDOR_ID = 0x0403
UBIQUITY_PRODUCT_ID = 0x6001

LIDAR_VENDOR_ID = 0x10c4
LIDAR_PRODUCT_ID = 0xea60

ARDUINO_VENDOR_ID = 0x2341
ARDUINO_PRODUCT_ID = 0x0043

BATTERY_VENDOR_ID = 0x1a86
BATTERY_PRODUCT_ID = 0x7523

CAMERA_VENDOR_ID = '8086'
CAMERA_PRODUCT_ID = '0b5c'

DEPTH_CAMERA_PRODUCT_ID = '2bc5'
DEPTH_CAMERA_VENDOR_ID = '065a'

PERIPHERAL_WAIT_TIME = 240
TOPIC_WAIT_TIME = 10
if ROBOT == "VIOLET":
    TOPIC_WAIT_TIME = 20

if ROBOT == "VIOLET":
    LAUNCHES = {"CAMERA": [['haystack', 'camera.launch'], CAMERA_TOPIC_LIST],
                "PERSON_DETECTION": [['haystack', 'haystack-person-follower.launch'], []],
                # "OBJECT_DETECTION": [['haystack', 'haystack-object-detection.launch'], []],
                "MOTOR": [['magni_bringup', 'base.launch'], UBIQUITY_TOPIC_LIST],
                "LIDAR": [['rplidar_ros', 'rplidar.launch'], LIDAR_TOPIC_LIST],
                "BATTERY": [['haystack', 'haystack-battery.launch'], []],
                "ARDUINO": [['cliff_controller', 'cliff_controller.launch'], []],
                "VELOCITY_SMOOTHER": [['yocs_velocity_smoother', 'standalone.launch'], []],
                "LASER_FILTER": [['haystack', 'laser_scan_filter.launch'], []],
                "DEPTH_FILTER": [['haystack', 'haystack-depth-filter.launch'], []],
                "RANGE_SCAN": [['haystack', 'laser_scan_from_range.launch'], []],
                "ROBOT_SAFETY": [['haystack', 'haystack-safety.launch'], []],
                "JOYSTICK": [['haystack', 'haystack-joystick.launch'], []],
                "MODE_SWITCHER": [['haystack', 'haystack-mode.launch'], []],
                "BLUETOOTH": [['haystack', 'haystack-bluetooth.launch'], []]
                }
elif ROBOT == "PURPLE":
    LAUNCHES = {"BATTERY": [['haystack', 'haystack-battery.launch'], []],
                "MOTOR": [['yoyo_bringup', 'robot_with_tf.launch'], REEMAN_MOTOR_TOPIC_LIST],
                "LIDAR": [['yoyo_bringup', 'ltme.launch'], LIDAR_TOPIC_LIST],
                "DEPTH_CAMERAS": [['yoyo_bringup', 'pointcloud_dabai_dabai.launch'], DEPTH_CAMERA_TOPIC_LIST],
                "MODE_SWITCHER": [['haystack', 'haystack-mode.launch'], []],
                "JOYSTICK": [['haystack', 'haystack-joystick.launch'], []],
                "ROBOT_SAFETY": [['haystack', 'haystack-safety.launch'], []],
                "PERSON_DETECTION": [['haystack', 'haystack-person-detection.launch'], []],
                "PERSON_FOLLOWER": [['haystack', 'haystack-person-follower.launch'], []],
                "BLUETOOTH": [['haystack', 'haystack-bluetooth.launch'], []],
                "SPEAKER": [['haystack', 'haystack-speaker.launch'], []]
                }


def check_topic_available(topic_list):
    available_topics = rospy.get_published_topics("/")
    for topic in topic_list:
        if topic not in available_topics:
            return False
    return True


def find_camera_device(product_id, vendor_id):
    command = 'lsusb'
    output = subprocess.check_output(command).decode('utf-8').split('\n')
    count = 0
    for line in output:
        if product_id in line and vendor_id in line:
            count += 1
    return count


def is_ip_reachable(ip, attempts=5, PING_COUNT=1):
    for attempt in range(attempts):
        try:
            response = os.system(f"ping -c {PING_COUNT} {ip}")
            if response == 0:
                return True
            else:
                print(f"Attempt {attempt + 1} failed: IP '{ip}' is not reachable.")
        except Exception as e:
            print(f"Error pinging IP '{ip}': {e}")
    return False


class Master:
    def __init__(self):
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)
        robot_version = os.getenv("ROBOT_VERSION", "0.0.0")
        rospy.set_param(TAG_PARAM_NAME, robot_version)
        try:
            output = subprocess.check_output("timedatectl show --property=Timezone --value", shell=True, text=True)
            current_time_zone = output.strip("\n")
        except:
            current_time_zone = "Asia/Kolkata"
        rospy.set_param(TIMEZONE_TOPIC, current_time_zone)
        dosimeter_value = ""
        pin = "0000"
        screen_lock = "0"
        lock = "0"
        max_lamp_hours = "9000"
        last_lamp_reset_hour = "0"
        disinfect_wait_time_multiplier = "10"
        self.missed_devices = []
        self.missed_peripherals = []
        self.info = {}
        self.set_info()
        self.config = configparser.ConfigParser()
        if os.path.exists(config_path):
            self.config.read(config_path)
            if self.config.has_option('ROBOT', 'DOSIMETER'):
                dosimeter_value = self.config.get('ROBOT', 'DOSIMETER')
            if not self.config.has_option('ROBOT', 'PIN'):
                self.config.set('ROBOT', 'PIN', pin)
                with open(config_path, 'w') as configfile:
                    self.config.write(configfile)
            else:
                pin = self.config.get('ROBOT', 'PIN')

            if not self.config.has_option('ROBOT', 'LOCK_SLEEP'):
                self.config.set('ROBOT', 'LOCK_SLEEP', screen_lock)
                with open(config_path, 'w') as configfile:
                    self.config.write(configfile)
            else:
                screen_lock = self.config.get('ROBOT', 'LOCK_SLEEP')

            if not self.config.has_option('ROBOT', 'LOCK'):
                self.config.set('ROBOT', 'LOCK', lock)
                with open(config_path, 'w') as configfile:
                    self.config.write(configfile)
            else:
                lock = self.config.get('ROBOT', 'LOCK')

            if not self.config.has_option('ROBOT', 'MAX_LAMP_HOURS'):
                self.config.set('ROBOT', 'MAX_LAMP_HOURS', max_lamp_hours)
                with open(config_path, 'w') as configfile:
                    self.config.write(configfile)
            else:
                max_lamp_hours = self.config.get('ROBOT', 'MAX_LAMP_HOURS')

            if not self.config.has_option('ROBOT', 'LAST_LAMP_RESET_HOUR'):
                self.config.set('ROBOT', 'LAST_LAMP_RESET_HOUR', last_lamp_reset_hour)
                with open(config_path, 'w') as configfile:
                    self.config.write(configfile)
            else:
                last_lamp_reset_hour = self.config.get('ROBOT', 'LAST_LAMP_RESET_HOUR')

            if not self.config.has_option('ROBOT', 'DISINFECT_WAIT_TIME_MULTIPLIER'):
                self.config.set('ROBOT', 'DISINFECT_WAIT_TIME_MULTIPLIER', disinfect_wait_time_multiplier)
                with open(config_path, 'w') as configfile:
                    self.config.write(configfile)
            else:
                disinfect_wait_time_multiplier = self.config.get('ROBOT', 'DISINFECT_WAIT_TIME_MULTIPLIER')

        rospy.set_param("/haystack/dosimeters", dosimeter_value)
        rospy.set_param("/haystack/firmware_version", robot_version)
        rospy.set_param("/haystack/pin", pin)
        rospy.set_param("/haystack/lock_sleep", screen_lock)
        rospy.set_param("/haystack/lock", lock)
        rospy.set_param("/haystack/robot", ROBOT)
        rospy.set_param("/haystack/max_lamp_hours", max_lamp_hours)
        rospy.set_param("/haystack/last_lamp_reset_hour", last_lamp_reset_hour)
        rospy.set_param("/haystack/disinfect_wait_time_multiplier", disinfect_wait_time_multiplier)
        self.running = True
        if ROBOT == "VIOLET":
            self.devices_check = {"MOTOR": False, "LIDAR": False, "BLUETOOTH": False, "ARDUINO": False, "CAMERA": False,
                                  "BATTERY": False, "DISPLAY": True, "SOS": True}
            self.device_topic_status = {"MOTOR": False, "LIDAR": False, "CAMERA": False}
        elif ROBOT == "PURPLE":
            self.devices_check = {"MOTOR": False, "LIDAR": False, "BLUETOOTH": False, "DEPTH_CAMERAS": False,
                                  "BATTERY": False, "DISPLAY": True, "SOS": True, "AI_PERIPHERAL": False,
                                  "SYSTEM_PERIPHERAL": False}
            self.device_topic_status = {"MOTOR": False, "LIDAR": False, "DEPTH_CAMERAS": False}
        self.device_name = {}
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self.launch = {}
        if ROBOT == "VIOLET":
            for key, [package, topic] in LAUNCHES.items():
                path = roslaunch.rlutil.resolve_launch_arguments(package)
                self.launch[key] = [roslaunch.parent.ROSLaunchParent(uuid, path), topic]
        elif ROBOT == "PURPLE":
            for key, [package, topic] in LAUNCHES.items():
                if key == "DEPTH_CAMERAS":
                    path = [(roslaunch.rlutil.resolve_launch_arguments(
                        ['yoyo_bringup', 'pointcloud_dabai_dabai.launch'])[0],
                             ['device_1_id:=' + CAMERA_1, 'device_2_id:=' + CAMERA_2])]
                else:
                    path = roslaunch.rlutil.resolve_launch_arguments(package)
                if key != "PERSON_DETECTION" and key != "SPEAKER":
                    self.launch[key] = [roslaunch.parent.ROSLaunchParent(uuid, path), topic]

                if key == "SPEAKER" and REEMAN == "OLD":
                    self.launch[key] = [roslaunch.parent.ROSLaunchParent(uuid, path), topic]
                    self.devices_check["SPEAKER"] = True

                if key == "PERSON_DETECTION":
                    if is_ip_reachable(PERIPHERAL_IP["jetson"]):
                        self.devices_check["AI_PERIPHERAL"] = True
                        self.devices_check["FRONT_CAMERA"] = False
                        self.wait_for_param("/peripheral_status/jetson", PERIPHERAL_WAIT_TIME)
                        if not rospy.has_param("/peripheral_status/jetson"):
                            self.missed_peripherals.append("AI_PERIPHERAL")
                        elif not rospy.get_param("/peripheral_status/jetson", True):
                            rospy.loginfo("Launching PERSON_DETECTION...")
                            self.launch[key] = [roslaunch.parent.ROSLaunchParent(uuid, path), topic]
                        else:
                            rospy.loginfo(
                                "Skipping launch of PERSON_DETECTION because /peripheral_status/jetson is 'found'")

            if is_ip_reachable(PERIPHERAL_IP["rp"]):
                self.devices_check["SYSTEM_PERIPHERAL"] = True
                self.devices_check["BACK_CAMERA"] = False
                self.devices_check["LEFT_CAMERA"] = False
                self.devices_check["RIGHT_CAMERA"] = False
                self.devices_check["DOSIMETER_BLUETOOTH"] = False
                self.wait_for_param("/peripheral_status/raspberry_pi", PERIPHERAL_WAIT_TIME)
                if not rospy.has_param("/peripheral_status/raspberry_pi"):
                    self.missed_peripherals.append("SYSTEM_PERIPHERAL")

        self.init_pub = rospy.Publisher(INITIALIZATION_STATUS, String, queue_size=10)
        rospy.Subscriber(SHUTDOWN_TOPIC, Bool, self.shutdown_callback)
        rospy.Subscriber(DOSIMETER_CONFIG_TOPIC, String, self.dosimeter_config_callback)
        rospy.Subscriber(LOCK_CONFIG_TOPIC, String, self.lock_config_callback)

    def signal_handler(self, *args):
        self.running = False
        self.shutdown_launch()

    def wait_for_param(self, param, timeout):
        count = 0
        while not rospy.has_param(param) and self.running:
            if count > timeout:
                rospy.loginfo(f"{param} not available...")
                break
            rospy.loginfo(f"Waiting for {param} parameter to be available...{count}")
            time.sleep(1)
            count += 1

    def shutdown_launch(self):
        for key, [launch, topics] in self.launch.items():
            launch.shutdown()
        rospy.set_param(SHUTDOWN_STATUS, 1)
        rospy.signal_shutdown("shutdown")

    def dosimeter_config_callback(self, data):
        dosimeter = data.data
        self.config.set('ROBOT', 'DOSIMETER', dosimeter)
        with open(config_path, 'w') as configfile:
            self.config.write(configfile)
        rospy.set_param('/haystack/dosimeters', dosimeter)

    def lock_config_callback(self, data):
        screen_lock = data.data
        self.config.set('ROBOT', 'LOCK_SLEEP', screen_lock)
        with open(config_path, 'w') as configfile:
            self.config.write(configfile)
        rospy.set_param('/haystack/lock_sleep', screen_lock)

    def shutdown_callback(self, data):
        if data.data:
            self.shutdown_launch()

    def check_device(self):
        device = find_camera_device(CAMERA_PRODUCT_ID, CAMERA_VENDOR_ID)
        if device == 1:
            self.devices_check["CAMERA"] = True
            print("camera found.")

        device = find_camera_device(DEPTH_CAMERA_PRODUCT_ID, DEPTH_CAMERA_VENDOR_ID)
        if device == 2:
            self.devices_check["DEPTH_CAMERAS"] = True
            print("camera found.")

        if ROBOT == "PURPLE":
            if is_ip_reachable(PERIPHERAL_IP["lidar"]):
                self.devices_check["LIDAR"] = True
            if os.path.exists("/dev/ttyS0"):
                self.devices_check["MOTOR"] = True
            if self.devices_check["AI_PERIPHERAL"] and rospy.has_param("/peripheral_status/jetson"):
                if rospy.get_param("/jetson_info/RS_CAMERA/Status", False):
                    self.devices_check["FRONT_CAMERA"] = True
            if self.devices_check["SYSTEM_PERIPHERAL"] and rospy.has_param("/peripheral_status/raspberry_pi"):
                self.wait_for_param("/raspberry_pi_info/BACK_CAMERA/Status", TOPIC_WAIT_TIME)
                if rospy.get_param("/raspberry_pi_info/BACK_CAMERA/Status", False):
                    self.devices_check["BACK_CAMERA"] = True
                self.wait_for_param("/raspberry_pi_info/LEFT_CAMERA/Status", TOPIC_WAIT_TIME)
                if rospy.get_param("/raspberry_pi_info/LEFT_CAMERA/Status", False):
                    self.devices_check["LEFT_CAMERA"] = True
                self.wait_for_param("/raspberry_pi_info/RIGHT_CAMERA/Status", TOPIC_WAIT_TIME)
                if rospy.get_param("/raspberry_pi_info/RIGHT_CAMERA/Status", False):
                    self.devices_check["RIGHT_CAMERA"] = True
                self.wait_for_param("/raspberry_pi_info/DOSIMETER/Status", TOPIC_WAIT_TIME)
                if rospy.get_param("/raspberry_pi_info/DOSIMETER/Status", False):
                    self.devices_check["DOSIMETER_BLUETOOTH"] = True

        device_list = list_ports.comports()
        for cfg in device_list:
            if cfg.vid is not None:
                if int(cfg.vid) == int(UBIQUITY_VENDOR_ID) and int(cfg.pid) == int(UBIQUITY_PRODUCT_ID):
                    self.devices_check["MOTOR"] = True
                    self.device_name["MOTOR"] = cfg.device
                if int(cfg.vid) == int(LIDAR_VENDOR_ID) and int(cfg.pid) == int(LIDAR_PRODUCT_ID):
                    self.devices_check["LIDAR"] = True
                    self.device_name["LIDAR"] = cfg.device
                if int(cfg.vid) == int(ARDUINO_VENDOR_ID) and int(cfg.pid) == int(ARDUINO_PRODUCT_ID):
                    self.devices_check["ARDUINO"] = True
                    self.device_name["ARDUINO"] = cfg.device
                if int(cfg.vid) == int(BATTERY_VENDOR_ID) and int(cfg.pid) == int(BATTERY_PRODUCT_ID):
                    self.devices_check["BATTERY"] = True
                    self.device_name["BATTERY"] = cfg.device

        try:
            ble_out = subprocess.check_output("hciconfig", shell=True)
            if len(ble_out) > 0:
                self.devices_check["BLUETOOTH"] = True
        except subprocess.CalledProcessError as e:
            print("bluetooth not detected")

        if ROBOT == "PURPLE" and REEMAN == "OLD":
            self.devices_check["BATTERY"] = True
            print("battery found.")

        ret = True
        for key, value in self.devices_check.items():
            if value:
                print(key + " is available")
                self.info[key] = {"Status": True}
            else:
                print(key + " is not available")
                self.missed_devices.append(key)
                self.info[key] = {"Status": False}
                ret = False
            self.set_info()
        if self.missed_peripherals:
            ret = False

        return ret

    def set_info(self):
        peripheral_info = rospy.get_param(ROBOT_INFO, {})
        peripheral_info[PERIPHERAL] = self.info
        rospy.set_param(ROBOT_INFO, peripheral_info)
        rospy.set_param(PERIPHERAL_INFO, self.info)

    def link_devices(self):
        print("Checking usb link status")

        try:
            subprocess.check_output("ls /dev | grep ubiquity", shell=True)
            print("ubiquity is linked")
        except subprocess.CalledProcessError as e:
            print("Initiating Ubiquity Manual Linking")
            os.system("ln -s " + self.device_name["MOTOR"] + " /dev/ubiquity")

        try:
            subprocess.check_output("ls /dev | grep lidar", shell=True)
            print("lidar is linked")
        except subprocess.CalledProcessError as e:
            print("Initiating Lidar Manual Linking")
            os.system("ln -s " + self.device_name["LIDAR"] + " /dev/lidar")

        try:
            subprocess.check_output("ls /dev | grep battery", shell=True)
            print("battery is linked")
        except subprocess.CalledProcessError as e:
            print("Initiating Battery Manual Linking")
            os.system("ln -s " + self.device_name["BATTERY"] + " /dev/battery")

        try:
            subprocess.check_output("ls /dev | grep arduino", shell=True)
            print("arduino is linked")
        except subprocess.CalledProcessError as e:
            print("Initiating Arduino Manual Linking")
            os.system("ln -s " + self.device_name["ARDUINO"] + " /dev/arduino")

    def run(self):
        if not self.check_device():
            if self.missed_peripherals:
                rospy.set_param(DEVICE_STATUS, ", ".join(self.missed_peripherals) + " NOT LAUNCHED")
            else:
                rospy.set_param(DEVICE_STATUS, ", ".join(self.missed_devices) + " NOT CONNECTED")
            rospy.set_param(SHUTDOWN_STATUS, 1)
            try:
                rospy.set_param(LAUNCH_STATUS, False)
                rospy.spin()
            finally:
                print("Exited Haystack Master Launch")

        else:
            print("All device are available")
            if ROBOT == "VIOLET":
                self.link_devices()
            print("Starting Haystack Master Launch")
            init_window_count = 0
            for key, [launch, topics] in self.launch.items():
                launch.start()
                count = 0
                self.device_topic_status[key] = True
                if not self.running:
                    break
                while not check_topic_available(topics):
                    if not self.running:
                        break
                    if count > TOPIC_WAIT_TIME:
                        self.device_topic_status[key] = False
                        break
                    rospy.sleep(duration=1.0)
                    count += 1
                if self.device_topic_status[key]:
                    init_window_count += 1
                    init_window = int(init_window_count / (len(self.launch.keys()) / 10))
                    self.init_pub.publish(str(init_window))
                    if key in self.devices_check:
                        self.info[key]["Active"] = True
                        self.set_info()
                    time.sleep(0.1)
            if ROBOT == "VIOLET":
                try:
                    if ROBOT == "VIOLET" and self.device_topic_status["CAMERA"]:
                        client = dynamic_reconfigure.client.Client("/camera/rgb_camera", timeout=30)
                        client.update_configuration({"enable_auto_exposure": True})
                        client.close()
                except Exception as e:
                    print("dynamic reconfigure error ", e)

            for key, value in self.device_topic_status.items():
                if not value:
                    print(key + " is not available")
                    self.missed_devices += ", " + key
            try:
                if any(not value for value in self.device_topic_status.values()):
                    rospy.set_param(DEVICE_STATUS, "SENSOR ERROR in " + ", ".join(self.missed_devices))
                    self.shutdown_launch()
                    rospy.set_param(LAUNCH_STATUS, False)
                else:
                    rospy.set_param(LAUNCH_STATUS, True)
                rospy.spin()
            finally:
                print("Exited Haystack Master Launch")


if __name__ == '__main__':
    rospy.init_node('Master')
    master = Master()
    master.run()