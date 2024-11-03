import numpy as np

from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC

from pydrake.all import RotationMatrix, RigidTransform


class MeshcatChromeCapture:
    def __init__(self, url, window_size):
        options = webdriver.ChromeOptions()
        options.add_argument('--headless')
        options.add_argument('--no-sandbox')
        self.driver = webdriver.Chrome(options=options)
        self.driver.set_window_size(window_size[0], window_size[1])
        self.url = url
        self.driver.get(url)
        self.remove_meshcat_panels()

    def remove_meshcat_panels(self):
        # Wait for the stats panel to be present
        wait = WebDriverWait(self.driver, 10)
        stats_panel = wait.until(EC.presence_of_element_located((By.ID, "stats-plot")))

        # remove the real time rate panel
        self.driver.execute_script("""
            var element = document.getElementById('stats-plot');
            if (element) {
                element.remove();
            }
        """)

        # remove the controls GUI -
        # Three.JS is often used in tandem with Dat.GUI (https://sbcode.net/threejs/dat-gui/),
        # so I asked an LLM for a script to hide Dat.GUI elements
        self.driver.execute_script("""
            // Remove by class name (most common)
            var datGuis = document.getElementsByClassName('dg main');
            while(datGuis.length > 0) {
                datGuis[0].remove();
            }
            
            // Alternative: remove by container ID if it exists
            var datContainer = document.getElementById('dat-gui-container');
            if (datContainer) {
                datContainer.remove();
            }
            
            // Hide any remaining Dat.GUI elements
            var datElements = document.querySelectorAll('.dg');
            datElements.forEach(function(element) {
                element.style.display = 'none';
            });
        """)

    def __del__(self):
        self.driver.quit()

    def grab(self, save_file_name: str) -> None:
        print(f'saving page to {save_file_name}')
        self.driver.save_screenshot(save_file_name)

    @staticmethod
    def look_at(meshcat, point_of_interest, cam_pos_local):
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
