This documentation explains the different parts of the JSON file needed for the
visualization of data during a simulation:

"model_file": This is the directory containing the file for the robot/plant
              description (like an urdf file).

"weld-body": This is whether the user wants "WeldFrames()" to be called on the
             robot/plant. If this property does not exist then the "WeldFrames()"
             function will not be called

"data": This will be a list of all the different visualizations/massages that
        the user wants to be displayed. The 3 types of visualizations will be
        kinematic data, the center of mass, and an LCM message. So far there
        are 2 types of visualizations that can be shown both of which are
        kinematic data:

        1) A trace line which has the following properties:

        "name": The line's name to be displayed in the GUI
        "frame": The name of the body part/context on which the line would be
        "point": The location within the body part/context
        "color": The line's color in the format [Red, Green, Blue]
        "alpha": The transparency as a value from 0 to 1
        "type": "line",
        "thickness": The thickness of the line
        "history": The time (in seconds) that the trace line will cover. Note
                   that this assumes that there is a constant rate of messages
                   coming through. However, if the messages are not coming at a
                   constant rate then the displayed line might be of varying
                   length.

        2) A point which has the following properties:

        "name": The point's name to be displayed in the GUI
        "frame": The name of the body part/context on which the line would be
        "point": The location within the body part/context
        "color": The line's color in the format [Red, Green, Blue]
        "alpha": The transparency as a value from 0 to 1
        "type": "point",
        "radius": The radius of the sphere representing the point
