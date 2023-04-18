class SensorData(object):
    def __init__(self):
        self.robot = None

    def init_robot(self, robot):
        print("Initialized")
        self.robot = robot

    def get_data(self):
        imu_data = self.robot.get_imu_data()
        ret_dict = {"battery_level": 78, "imu_data": imu_data,
                    "tf_luna_data": (self.robot.get_tf_luna_data()), "pitch_roll_yaw": (self.robot.get_filtered_data(imu_data)),
                    "husky_lens_data": (self.robot.get_husky_lens_data())}
        # ret_dict = {"battery_level": (self.robot.read_battery_level()), "imu_data": (imu_data),
        #             "tf_luna_data": (self.robot.get_tf_luna_data()),
        #             "pitch_roll_yaw": (self.robot.get_filtered_data(imu_data)),
        #             "husky_lens_data": (self.robot.get_husky_lens_data())}
        # ret_dict = {"battery_level": (self.robot.read_battery_level())}
        print(ret_dict)
        return ret_dict
