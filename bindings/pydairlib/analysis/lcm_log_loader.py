import argparse
import os
from lcm import EventLog
import dairlib
from pydrake.trajectories import PiecewiseQuaternionSlerp
from pydrake.common.eigen_geometry import Quaternion
from pydrake.common.eigen_geometry import AngleAxis
from pydrake.math import RotationMatrix
import matplotlib.pyplot as plt
import numpy as np


def process_lcm_log(log_file_path, target_channels_and_message_types,
                    start_time=0.0, end_time=9999999999999):
    # Open the LCM log file
    log_file = EventLog(log_file_path, 'r')
    
    # Store the messages and timestamps for state messages
    franka_utimes = {}
    object_utimes = {}
    tracking_msg_utimes = {}
    osc_debug_msg_utimes = {}

    # Read through the log file
    first_event = log_file.read_next_event()
    initial_utimestamp = first_event.timestamp
    # DELETE TEMPORARY: Set the initial timestamp to 0
    # initial_utimestamp = 0
    # get the initial message's utime
    if first_event.channel in target_channels_and_message_types:
        msg_contents = target_channels_and_message_types[event.channel].decode(event.data)
        initial_utime = msg_contents.utime
        # DELETE TEMPORARY: Set the initial utime to 0
        # initial_utime = 0
    else:
        initial_utime = 0

    print("start time is: ", start_time)
    print("end time is: ", end_time)
    event = log_file.seek_to_timestamp(initial_utimestamp + int(start_time*1e6))
    event = log_file.read_next_event()

    c3_actual_times = []
    c3_actual_states = []

    c3_target_times = []
    c3_target_states = []

    c3_final_target_times = []
    c3_final_target_states = []

    while event is not None:
        # Check if the message is from the target channel
        adjusted_event_timestamp = (event.timestamp - initial_utimestamp) * 1e-6
        if adjusted_event_timestamp > end_time:
            break

        if event.channel in target_channels_and_message_types:
            msg_contents = target_channels_and_message_types[event.channel].decode(event.data)
            # adjusted_msg_timestamp = (msg_contents.utime - initial_utime)*1e-6
            adjusted_msg_timestamp = (msg_contents.utime)*1e-6

            # Process FRANKA_STATE_SIMULATION and OBJECT_STATE_SIMULATION messages
            if event.channel == "FRANKA_STATE_SIMULATION":
                franka_utimes[adjusted_msg_timestamp] = adjusted_msg_timestamp
            elif event.channel == "OBJECT_STATE_SIMULATION":
                object_utimes[adjusted_msg_timestamp] = adjusted_msg_timestamp
            elif event.channel == "TRACKING_TRAJECTORY_ACTOR":
                tracking_msg_utimes[adjusted_msg_timestamp] = adjusted_msg_timestamp
            elif event.channel == "OSC_DEBUG_FRANKA":
                osc_debug_msg_utimes[adjusted_msg_timestamp] = adjusted_msg_timestamp

            # Process TRACKING_TRAJECTORY_ACTOR messages
            # elif event.channel == "TRACKING_TRAJECTORY_ACTOR":
            #     tracking_msg_utime = adjusted_msg_timestamp
            #     timestamp = adjusted_msg_timestamp
            #     tracking_msg_utimes[adjusted_msg_timestamp] = adjusted_msg_timestamp

            #     if tracking_msg_utime in franka_utimes:
            #         print(f"\tReceived FRANKA_STATE_SIMULATION message at: {franka_utimes[tracking_msg_utime]*1e-6}, utime: {tracking_msg_utime*1e-6}\n"
            #               f"\tReceived TRACKING_TRAJECTORY_ACTOR message at : {timestamp*1e-6}, utime: {tracking_msg_utime*1e-6}")
            #         # print(f"Received FRANKA_STATE_SIMULATION message at: {franka_utimes[tracking_msg_utime]}, utime: {tracking_msg_utime}\n"
            #         #       f"Received TRACKING_TRAJECTORY_ACTOR message at : {timestamp}, utime: {tracking_msg_utime}")

            #     elif tracking_msg_utime in object_utimes:
            #         print(f"\tReceived OBJECT_STATE_SIMULATION message at: {object_utimes[tracking_msg_utime]*1e-6}, utime: {tracking_msg_utime*1e-6}\n"
            #               f"\tReceived TRACKING_TRAJECTORY_ACTOR msg at : {timestamp*1e-6}, utime: {tracking_msg_utime*1e-6} \n")
            #         # print(f"Received OBJECT_STATE_SIMULATION message at: {object_utimes[tracking_msg_utime]}, utime: {tracking_msg_utime}\n"
            #         #       f"Received TRACKING_TRAJECTORY_ACTOR msg at : {timestamp}, utime: {tracking_msg_utime} \n")

            #     else:
            #         # print the franka state message with the closest utime to the tracking message right before the tracking message
            #         print(f"Received FRANKA_STATE_SIMULATION message at: {min(franka_utimes, key=lambda x:abs(x-tracking_msg_utime))*1e-6}, utime: {tracking_msg_utime*1e-6}\n")
            #         print(f"\t\tReceived TRACKING_TRAJECTORY_ACTOR message at : {timestamp*1e-6}, utime: {tracking_msg_utime*1e-6}")
            #         # print(f"\tReceived TRACKING_TRAJECTORY_ACTOR message at : {timestamp}, utime: {tracking_msg_utime}")
           
            # Process C3_DEBUG_CURR messages
            elif event.channel == "C3_ACTUAL":
                c3_actual_times.append(adjusted_msg_timestamp)
                c3_actual_states.append(msg_contents.state)
                print(f"Received C3_ACTUAL message at : {adjusted_msg_timestamp}")

            elif event.channel == "C3_TARGET":
                c3_target_times.append(adjusted_msg_timestamp)
                c3_target_states.append(msg_contents.state)
                print(f"Received C3_TARGET message at : {adjusted_msg_timestamp}")

            elif event.channel == "C3_FINAL_TARGET":
                c3_final_target_times.append(adjusted_msg_timestamp)
                c3_final_target_states.append(msg_contents.state)
                print(f"Received C3_FINAL_TARGET message at : {adjusted_msg_timestamp}")
                # if(c3_actual_states != [] and c3_target_states != []):
                #     break

            #     if c3_actual_msg_utime in franka_utimes:
            #         ## print(f"Received FRANKA_STATE_SIMULATION message at: {franka_utimes[c3_actual_msg_utime]*1e-6}, utime: {c3_actual_msg_utime*1e-6}\n"
            #         #       f"Received C3_ACTUAL message at : {timestamp*1e-6}, utime: {c3_debug_msg_utime*1e-6} \n")
            #         print(f"\tReceived C3_ACTUAL message at : {timestamp*1e-6}, utime: {c3_actual_msg_utime*1e-6}\n")
            #         # print(f"Received C3_ACTUAL message at : {timestamp}, utime: {c3_debug_msg_utime}\n")

            #     elif c3_actual_msg_utime in object_utimes:
            #         print(f"\tReceived OBJECT_STATE_SIMULATION message at: {object_utimes[c3_actual_msg_utime]*1e-6}, utime: {c3_actual_msg_utime*1e-6}\n"
            #               f"\tReceived C3_ACTUAL message at : {timestamp*1e-6}, utime: {c3_actual_msg_utime*1e-6} \n")
            #         # print(f"Received OBJECT_STATE_SIMULATION message at: {object_utimes[c3_actual_msg_utime]}, utime: {c3_actual_msg_utime}\n"
            #         #       f"Received C3_ACTUAL message at : {timestamp}, utime: {c3_actual_msg_utime} \n")

            #     else:
            #         print(f"\t\tReceived C3_ACTUAL message at : {timestamp*1e-6}, utime: {c3_actual_msg_utime*1e-6}\n")
            #         # print(f"\tReceived C3_ACTUAL message at : {timestamp}, utime: {c3_actual_msg_utime}\n")

            # # Process C3_DEBUG_CURR messages
            # elif event.channel == "C3_DEBUG_CURR":
            #     c3_debug_msg_utime = adjusted_msg_timestamp
            #     timestamp = adjusted_msg_timestamp

            #     if c3_debug_msg_utime in franka_utimes:
            #        # # print(f"Received FRANKA_STATE_SIMULATION message at: {franka_utimes[c3_debug_msg_utime]*1e-6}, utime: {c3_debug_msg_utime*1e-6}\n"
            #         ##       f"Received C3_DEBUG_CURR message at : {timestamp*1e-6}, utime: {c3_debug_msg_utime*1e-6} \n")
            #         print(f"\tReceived C3_DEBUG_CURR message at : {timestamp*1e-6}, utime: {c3_debug_msg_utime*1e-6}")
            #         # print(f"Received C3_DEBUG_CURR message at : {timestamp}, utime: {c3_debug_msg_utime}")

            #     elif c3_debug_msg_utime in object_utimes:
            #         print(f"\tReceived OBJECT_STATE_SIMULATION message at: {object_utimes[c3_debug_msg_utime]*1e-6}, utime: {c3_debug_msg_utime*1e-6}\n"
            #               f"\tReceived C3_DEBUG_CURR message at : {timestamp*1e-6}, utime: {c3_debug_msg_utime*1e-6} \n")
            #         # print(f"Received OBJECT_STATE_SIMULATION message at: {object_utimes[c3_debug_msg_utime]}, utime: {c3_debug_msg_utime}\n"
            #         #       f"Received C3_DEBUG_CURR message at : {timestamp}, utime: {c3_debug_msg_utime} \n")

            #     else:
            #         print(f"\t\tReceived C3_DEBUG_CURR message at : {timestamp*1e-6}, utime: {c3_debug_msg_utime*1e-6}")
            
            # # Process OSC_DEBUG_FRANKA messages
            # elif event.channel == "OSC_DEBUG_FRANKA":
            #     osc_debug_msg_utime = adjusted_msg_timestamp
            #     timestamp = adjusted_msg_timestamp
            #     if osc_debug_msg_utime in tracking_msg_utimes:
            #         print(f"Received OSC_DEBUG_FRANKA message at : {timestamp*1e-6}, utime: {osc_debug_msg_utime*1e-6}")
                

        event = log_file.read_next_event()

    print("Finished processing log file.")

    assert c3_target_times == c3_actual_times == c3_final_target_times
    c3_actual_states = np.array(c3_actual_states)
    c3_final_target_states = np.array(c3_final_target_states)
    c3_target_states = np.array(c3_target_states)

    # Loop through the data.
    target_quats = []
    angle_diffs = []
    for i in range(len(c3_actual_states)):
        t = c3_actual_times[i]

        wxyz_actual = c3_actual_states[i, 3:7]
        wxyz_final = c3_final_target_states[i, 3:7]

        wxyz_actual = wxyz_actual / np.linalg.norm(wxyz_actual)
        wxyz_final = wxyz_final / np.linalg.norm(wxyz_final)

        actual_quat = Quaternion(wxyz=wxyz_actual)
        final_quat = Quaternion(wxyz=wxyz_final)

        quat_slerp = PiecewiseQuaternionSlerp([0,1], [actual_quat, final_quat])
        radian_lookahead = 0.698
        radian_total = AngleAxis(final_quat.multiply(actual_quat.inverse()))
        angle_diffs.append(radian_total.angle())
        lookahead_fraction = min(radian_lookahead/radian_total.angle(), 1.0)

        target_quat_2 = quat_slerp.orientation(lookahead_fraction)
        target_quats.append(target_quat_2.wxyz())

    target_quats = np.array(target_quats)

    # Plot the quaternions.
    fig, axs = plt.subplots(5, 1, figsize=(10, 10), sharex='all')
    axs[0].plot(c3_actual_times, c3_actual_states[:, 3], label='w')
    axs[0].plot(c3_actual_times, c3_actual_states[:, 4], label='x')
    axs[0].plot(c3_actual_times, c3_actual_states[:, 5], label='y')
    axs[0].plot(c3_actual_times, c3_actual_states[:, 6], label='z')
    axs[1].plot(c3_actual_times, c3_final_target_states[:, 3], label='w')
    axs[1].plot(c3_actual_times, c3_final_target_states[:, 4], label='x')
    axs[1].plot(c3_actual_times, c3_final_target_states[:, 5], label='y')
    axs[1].plot(c3_actual_times, c3_final_target_states[:, 6], label='z')
    axs[2].plot(c3_actual_times, c3_target_states[:, 3], label='w')
    axs[2].plot(c3_actual_times, c3_target_states[:, 4], label='x')
    axs[2].plot(c3_actual_times, c3_target_states[:, 5], label='y')
    axs[2].plot(c3_actual_times, c3_target_states[:, 6], label='z')
    axs[3].plot(c3_actual_times, target_quats[:, 0], label='w')
    axs[3].plot(c3_actual_times, target_quats[:, 1], label='x')
    axs[3].plot(c3_actual_times, target_quats[:, 2], label='y')
    axs[3].plot(c3_actual_times, target_quats[:, 3], label='z')
    plt.legend()
    axs[4].plot(c3_actual_times, angle_diffs, label='Angle Diff')
    axs[4].axhline(y=np.pi, color='r', linestyle='--', label='pi')
    axs[0].set_title('C3 Actual States')
    axs[1].set_title('C3 Final Target States')
    axs[2].set_title('C3 Target States')
    axs[2].set_title('Reconstructed Targets')
    fig.suptitle('Quaternion Trajectories')
    plt.legend()
    plt.show()

    '''
    prev_quat = c3_actual_states[0][3:7]
    prev_quat = prev_quat / np.linalg.norm(prev_quat)
    prev_quat = Quaternion(prev_quat)

    for i in range(len(c3_actual_states)):
        y_quat = c3_actual_states[i][3:7]
        # print("y_quat extracted ", y_quat)
        y_quat = y_quat / np.linalg.norm(y_quat)
        y_quat = Quaternion(y_quat)

        # compute angle axis diff between current and previous quaternion
        change_in_current_quat = AngleAxis(y_quat.multiply(prev_quat.inverse()))
        print("change_in_current_quat ", change_in_current_quat.angle())
        print("change in axis ", change_in_current_quat.axis())


        print("y_quat normalized ", y_quat.w(), y_quat.x(), y_quat.y(), y_quat.z())

        y_quat_des = c3_target_states[i][3:7]
        y_quat_des = y_quat_des / np.linalg.norm(y_quat_des)
        y_quat_des = Quaternion(y_quat_des)
        print("y_quat_des normalized ", y_quat_des.w(), y_quat_des.x(), y_quat_des.y(), y_quat_des.z())

        y_quat_final = c3_final_target_states[i][3:7]
        y_quat_final = y_quat_final / np.linalg.norm(y_quat_final)
        y_quat_final = Quaternion(y_quat_final)
        print("y_quat_final normalized ", y_quat_final.w(), y_quat_final.x(), y_quat_final.y(), y_quat_final.z())

        orientation_traj = PiecewiseQuaternionSlerp([0,1], [y_quat, y_quat_des])
        angle_axis_diff = AngleAxis(y_quat_des.multiply(y_quat.inverse()))
        lookahead_fraction = min(0.698/angle_axis_diff.angle(), 1.0)
        y_quat_lookahead = orientation_traj.orientation(lookahead_fraction)
        print("y_quat_lookahead ", y_quat_lookahead.w(), y_quat_lookahead.x(), y_quat_lookahead.y(), y_quat_lookahead.z(), "\n")


    # plot_utimes(franka_utimes, object_utimes, tracking_msg_utimes, osc_debug_msg_utimes)

    # TODO: Make the function definition be more general. But this should work
    # plot_utime_differences(franka_utimes, tracking_msg_utimes)
    # plot_utime_differences(osc_debug_msg_utimes, tracking_msg_utimes)
    # plot_controller_freq(tracking_msg_utimes)
    '''

def plot_controller_freq(tracking_msg_utimes):
    # Sort tracking utimes
    tracking_utimes = sorted(tracking_msg_utimes.keys())
    
    # Shift the tracking_utimes by one position
    tracking_utimes_shifted = [0] + tracking_utimes[:-1]  # Add a 0 at the start and remove the last element
    
    # Calculate the difference between current and last tracking_utimes
    diff = np.array(tracking_utimes) - np.array(tracking_utimes_shifted)

    # Plot the differences
    plt.figure()
    plt.plot(tracking_utimes, diff, label='Time Difference (Current - Last Tracking Msg)', marker='o')

    plt.xlabel('Tracking Message Utime')
    plt.ylabel('Difference in Utime (Current Tracking Msg - Last Tracking Msg)')
    plt.title('Controller Loop Time')
    plt.legend()
    plt.grid(True)
    plt.show()

def plot_utimes(franka_utimes, object_utimes, tracking_msg_utimes, osc_debug_msg_utimes):
    franka_utimes_list = franka_utimes.keys()
    object_utimes_list = object_utimes.keys()
    tracking_msg_utimes_list = tracking_msg_utimes.keys()
    osc_debug_msg_utimes_list = osc_debug_msg_utimes.keys()

    plt.figure()

    plt.plot(franka_utimes_list, [franka_utimes[u] for u in franka_utimes_list], label='Franka State Simulation', marker='o')
    plt.plot(object_utimes_list, [object_utimes[u] for u in object_utimes_list], label='Object State Simulation', marker='x')
    plt.plot(tracking_msg_utimes_list, [tracking_msg_utimes[u] for u in tracking_msg_utimes_list], label='Tracking Trajectory Actor', marker='s')
    plt.plot(osc_debug_msg_utimes_list, [osc_debug_msg_utimes[u] for u in osc_debug_msg_utimes_list], label='OSC Debug Actor', marker='d')

    plt.xlabel('Utime')
    plt.ylabel('Adjusted Timestamp')
    plt.title('Utimes of Different Message Types')
    plt.legend()
    plt.grid(True)
    plt.show()

    # Plot the differences between the utimes of the various message types
    # plot_utime_differences(franka_utimes_list, osc_utimes_list, tracking_utimes_list)

def plot_utime_differences(osc_utimes_dict, tracking_utimes_dict):
    # Get the sorted list of utimes from the keys of the dictionaries
    osc_utimes = sorted(osc_utimes_dict.keys())
    tracking_utimes = sorted(tracking_utimes_dict.keys())
    print("tracking_utimes = ")
    print(tracking_utimes)
    
    # Calculate differences between OSC utimes and the last tracking message for every OSC message
    osc_tracking_diff = []
    valid_osc_utimes = []  # Store valid OSC utimes to keep x and y dimensions aligned
    last_tracking_utime = None
    
    tracking_index = 0

    for osc_utime in osc_utimes:
        # Move the tracking_index to the most recent tracking_utime <= osc_utime
        while tracking_index < len(tracking_utimes) and tracking_utimes[tracking_index] <= osc_utime:
            last_tracking_utime = tracking_utimes[tracking_index]
            tracking_index += 1
        
        # If there is a valid last_tracking_utime, calculate the difference
        if last_tracking_utime is not None:
            osc_tracking_diff.append(osc_utime - last_tracking_utime)
            valid_osc_utimes.append(osc_utime)  # Add corresponding OSC utime to the valid list

    # print(valid_osc_utimes)
    # Plot the differences
    plt.figure()

    # plt.plot(valid_osc_utimes, osc_tracking_diff, label='OSC - Last Tracking', marker='o')

    # plt.xlabel('OSC Utime')
    # plt.ylabel('Difference in Utime (OSC - Last Tracking)')
    # plt.title('Difference Between OSC Utime and Last Tracking Utime')
    # plt.legend()
    # plt.grid(True)
    # plt.show()


    plt.plot(valid_osc_utimes, osc_tracking_diff, label='franka state msg - Last Tracking', marker='o')

    plt.xlabel('Franka Utime')
    plt.ylabel('Difference in Utime (franka msg utime - Last Tracking msg utime)')
    plt.title('Difference Between franka msg Utime and Last Tracking msg Utime')
    plt.legend()
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    # Set up command line argument parsing
    parser = argparse.ArgumentParser(description='Process an LCM log file.')
    parser.add_argument('log_folder', type=str,
                        help='Path to the folder containing the log file')
    parser.add_argument('start_time', type=float, default=0.0,
                        help='Start time into the log to begin parsing')
    # optional end time argument
    parser.add_argument('end_time', type=float, default=9999999999999,
                        help='End time into the log to stop parsing')

    args = parser.parse_args()
    log_folder = args.log_folder
    start_time = args.start_time
    end_time = args.end_time

    # Turn the folder into a file path.
    # Extract the last part of the folder name.
    log_number = log_folder.split("/")[-1][:6]
    # Construct the log file path.
    log_filepath = os.path.join(log_folder, f"simlog-{log_number}")
    print(f"Parsing log at: {log_filepath}")

    # List of channels to care about.
    target_channels_and_message_types = {
        "TRACKING_TRAJECTORY_ACTOR": dairlib.lcmt_timestamped_saved_traj,
        "FRANKA_STATE_SIMULATION": dairlib.lcmt_robot_output,
        "OBJECT_STATE_SIMULATION": dairlib.lcmt_object_state,
        "C3_DEBUG_CURR": dairlib.lcmt_c3_output,
        "C3_ACTUAL": dairlib.lcmt_c3_state,
        "OSC_DEBUG_FRANKA": dairlib.lcmt_osc_output,
        "C3_TARGET": dairlib.lcmt_c3_state,
        "C3_FINAL_TARGET": dairlib.lcmt_c3_state,
    }
    
    # Call the function and get the data and timestamps
    process_lcm_log(log_filepath, target_channels_and_message_types,
                    start_time=start_time, end_time=end_time)
