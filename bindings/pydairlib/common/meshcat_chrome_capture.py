import numpy as np

from selenium import webdriver
from selenium.webdriver.common.by import By
from selenium.webdriver.support.ui import WebDriverWait
from selenium.webdriver.support import expected_conditions as EC

from pydrake.all import RotationMatrix, RigidTransform, Meshcat

from typing import Tuple


class MeshcatChromeCapture:
    """
    Helper class for generating clean screenshots of drake Meshcat scenes.
    The workflow looks like this:

        meshcat = Meshcat()
        capture = MeshcatChromeCapture(meshcat, (1080, 720))

        # setup the scene here

        capture.grab('filename.png')

    """
    def __init__(self, meshcat: Meshcat, window_size: Tuple[int, int], silent: bool = False):
        options = webdriver.ChromeOptions()
        options.add_argument('--headless')
        options.add_argument('--no-sandbox')
        self.url = meshcat.web_url()

        self._silent = silent
        self._meshcat = meshcat
        self._driver = webdriver.Chrome(options=options)
        self._driver.set_window_size(window_size[0], window_size[1])
        self._driver.get(self.url)
        self._set_pretty_lighting()
        self._remove_meshcat_panels()

    def _set_pretty_lighting(self):
        self._meshcat.SetProperty("/Lights/PointLightPositiveX/<object>", "castShadow", True)
        self._meshcat.SetProperty("/Lights/SpotLight/<object>", "castShadow", True)
        self._meshcat.SetProperty("/Lights/PointLightPositiveX/<object>", "intensity", 100.0)
        self._meshcat.SetProperty("/Lights/SpotLight/<object>", "intensity", 40.0)

    def _remove_meshcat_panels(self):
        assert(self._driver.current_url == self._meshcat.web_url())
        wait = WebDriverWait(self._driver, 10)
        _ = wait.until(EC.presence_of_element_located((By.ID, "stats-plot")))

        # remove the real time rate panel
        self._driver.execute_script("""
            var element = document.getElementById('stats-plot');
            if (element) {
                element.remove();
            }
        """)

        # remove the controls GUI
        # The control panel is made with Dat.GUI (https://sbcode.net/threejs/dat-gui/),
        # so I asked an LLM for a script to hide Dat.GUI elements
        #
        # Maybe need updated for future meshcat versions
        self._driver.execute_script("""
            var datGuis = document.getElementsByClassName('dg main');
            while(datGuis.length > 0) {
                datGuis[0].remove();
            }
            
            var datContainer = document.getElementById('dat-gui-container');
            if (datContainer) {
                datContainer.remove();
            }
            
            var datElements = document.querySelectorAll('.dg');
            datElements.forEach(function(element) {
                element.style.display = 'none';
            });
        """)

    def __del__(self):
        self._driver.quit()

    def grab(self, save_file_name: str) -> None:
        self._driver.save_screenshot(save_file_name)
        if not self._silent:
            print(f'saved page to {save_file_name}')

    def look_at(self, point_of_interest, cam_pos_local):
        # point the camera at the poit of interest
        self._meshcat.SetCameraPose(
            point_of_interest + cam_pos_local, point_of_interest)

        # Set the lighting positions
        self._meshcat.SetTransform(
            "/Lights/SpotLight/<object>",
            RigidTransform(
                RotationMatrix(), point_of_interest + np.array([0.0, -5.0, 1.0])))
        self._meshcat.SetTransform(
            "/Lights/PointLightPositiveX/<object>",
            RigidTransform(
                RotationMatrix(), point_of_interest + np.array([2.0, 0.0, 2.0])))
        self._meshcat.SetTransform(
            "/Lights/PointLightNegativeX/<object>",
            RigidTransform(
                RotationMatrix(), point_of_interest + np.array([-2.0, 0.0, 2.0])))
