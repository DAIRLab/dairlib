import numpy as np
from selenium import webdriver
from pydrake.all import RotationMatrix, RigidTransform

class ChromeCapture:
    def __init__(self, url, window_size):
        options = webdriver.ChromeOptions()
        options.add_argument('--headless')
        options.add_argument('--no-sandbox')
        self.driver = webdriver.Chrome(options=options)
        self.driver.set_window_size(window_size[0], window_size[1])
        self.url = url
        self.driver.get(url)

    def __del__(self):
        self.driver.quit()

    def grab(self, save_file_name: str) -> None:
        print(f'saving page to {save_file_name}')
        self.driver.save_screenshot(save_file_name)

    def look_at(self, meshcat, point_of_interest, cam_pos_local):
        # point the camera at the poit of interest
        meshcat.SetCameraPose(
            point_of_interest + cam_pos_local, point_of_interest)

        # Set the lighting positions
        meshcat.SetTransform(
            "/Lights/SpotLight/<object>",
            RigidTransform(
                RotationMatrix(), point_of_interest + np.array([0.0, -5.0, 1.0])))
        meshcat.SetTransform(
            "/Lights/PointLightPositiveX/<object>",
            RigidTransform(
                RotationMatrix(), point_of_interest + np.array([2.0, 0.0, 2.0])))
        meshcat.SetTransform(
            "/Lights/PointLightNegativeX/<object>",
            RigidTransform(
                RotationMatrix(), point_of_interest + np.array([-2.0, 0.0, 2.0])))
