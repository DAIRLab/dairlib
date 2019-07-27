import dairlib.lcmt_iiwa_command
import dairlib.lcmt_iiwa_status

import director.openscope as scope
import subprocess

view = applogic.getMainWindow()
applogic.addShortcut(view, 'Ctrl+I', scope.startSignalScope)
