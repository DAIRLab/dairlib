This documentation explains the different parts of the JSON file needed for the
visualization of data during a simulation:


* "model_file": This is the directory containing the file for the robot/plant
              description (like an urdf file).

* "weld_body": This is whether the user wants "WeldFrames()" to be called on the
             robot/plant. If this property does not exist then the "WeldFrames()"
             function will not be called

* "channel_name": Name of the LCM channel containing state information as lcmt_robot_output

* "data": This will be a list of all the different shapes/objects that
        the user wants to be displayed. Each shape is a separate JSON object
        consisting of a name and an info JSON. The info consists of 2 pieces of
        information: 1) the source data, which has information about how to draw
        the particular shape and 2) the type data, which contains information about
        the what is to be drawn. The source data can be one of 3 categories:
        kinematic, center of mass, and LCM message. Also in terms of the type
        data there are 3 types as well: a point/sphere, a line, and a set of 3
        axes. The following are the specific formats of the 3 source type
        categories:

        1) Kinematic: Uses robot state information to draw the position of a particular point fixed in a given frame.

        "category":"kinematic"
        "frame": The name of the given frame.
        "point": The constant position of the point in the given frame.


        2) LCM: While kinematic data is drawn from the robot state, LCM data comes directly from a specific LCM message.
                For example, this might be used to visualize the target position of some kinematic value.

        "category":"lcm",
        "abstract_channel" : The channel from where the user wants to listen to
                             messages

        "abstract_type" : Type/Class of message from the secondary channel

        "abstract_field" : The specific field from where the information is
                           to be extracted. This needs to contain the entire
                           path to reach the field. It can contain both field
                           names as well as arrays with the index number to be
                           selected. If this contains an array and the needed
                           index is unknown, but the name of the element is
                           known, then the symbol %d can be provided inside
                           the brackets which will be followed by the two
                           additional JSON attributes below.

       "index_field": In the case the symbol %d is present in the abstract_field
                      this will contain the path to an array with the names of
                      all the objects/elements present in the lcm message

       "index_element": In the case the symbol %d is present in the abstract_field
                        this will contain the name of the specific object/element
                        to be searched in the "index_field" array


        If this is intended to be used to draw a point or a line then it
        also contains:
        "x_index" : The index of the x element of the location where the
                    line/point it so be drawn

        The y and z indices are assumed to be sequential.

        If this is intended to be used to draw a set of axes then it also
        contains:
        "quaternion_index" : The first index of the 4D array which will contain
                             the quaternion from where the rotation matrix will
                             be extracted. The other 3 indices will be the ones
                             that follow this in sequance. For example, if this
                             was 0 then the others will be 1, 2, 3.


         While the specified LCM message determines the orientation of the axes
         to draw, the location of the axes needs to also be known. The origin of
         the axes is determined in the same manner as a kinematic point, by
         specifying "frame" and "point" fields.

        "frame": The particular body part on which the user wants to draw the
                 axis
        "point": The location within the body part's context




        3) Center of Mass (CoM):

        "category":"com"

     The 3 types of shapes that can be drawn will have the following type
        data:

        1) A trace line:

        "color": The line's color in the format [Red, Green, Blue]
        "alpha": The transparency as a value from 0 to 1
        "type": "line",
        "thickness": The thickness of the line
        "history": The time (in seconds) that the trace line will cover. Note
                   that this assumes that there is a constant rate of messages
                   coming through. However, if the messages are not coming at a
                   constant rate then the displayed line might be of varying
                   length. Also note that if the history is negative then the
                   line will be infinitely long (i.e. it will be constantly
                   drawn as long as the simulation is running)

        2) A point:

        "color": The line's color in the format [Red, Green, Blue]
        "alpha": The transparency as a value from 0 to 1
        "type": "point",
        "radius": The radius of the sphere representing the point


        3) A set of 3 axes:

        "alpha": The transparency as a value from 0 to 1
        "type": "axes",
        "thickness": The thickness of each arrow
        "length": The length of each arrow

        Note that the axes will be color coded with red being the x-axis, green
        the y-axis, and blue the z-axis.

