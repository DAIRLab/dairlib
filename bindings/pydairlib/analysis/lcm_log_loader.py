import argparse
import os
from lcm import EventLog
import dairlib
import matplotlib.pyplot as plt
import numpy as np

def process_lcm_log(log_file_path, target_channels_and_message_types):
    # Open the LCM log file
    log_file = EventLog(log_file_path, 'r')
    
    # Store the messages and timestamps for state messages
    franka_utimes = {}
    object_utimes = {}
    tracking_msg_utimes = {}
    osc_debug_msg_utimes = {}

    # Read through the log file
    event = log_file.read_next_event()
    initial_timestamp = event.timestamp
    # DELETE TEMPORARY: Set the initial timestamp to 0
    # initial_timestamp = 0
    # get the initial message's utime
    if event.channel in target_channels_and_message_types:
            msg_type = target_channels_and_message_types[event.channel].decode(event.data)
            initial_utime = msg_type.utime
            # DELETE TEMPORARY: Set the initial utime to 0
            # initial_utime = 0
    else:
        initial_utime = 0

    while event is not None:
        # Check if the message is from the target channel
        adjusted_timestamp = event.timestamp - initial_timestamp
        if event.channel in target_channels_and_message_types:
            msg_type = target_channels_and_message_types[event.channel].decode(event.data)
            adjusted_utime = (msg_type.utime - initial_utime)*1e-6

            # Process FRANKA_STATE_SIMULATION and OBJECT_STATE_SIMULATION messages
            if event.channel == "FRANKA_STATE_SIMULATION":
                franka_utimes[adjusted_utime] = adjusted_timestamp
            elif event.channel == "OBJECT_STATE_SIMULATION":
                object_utimes[adjusted_utime] = adjusted_timestamp
            elif event.channel == "TRACKING_TRAJECTORY_ACTOR":
                tracking_msg_utimes[adjusted_utime] = adjusted_timestamp
            elif event.channel == "OSC_DEBUG_FRANKA":
                osc_debug_msg_utimes[adjusted_utime] = adjusted_timestamp

            # Process TRACKING_TRAJECTORY_ACTOR messages
            # elif event.channel == "TRACKING_TRAJECTORY_ACTOR":
            #     tracking_msg_utime = adjusted_utime
            #     timestamp = adjusted_timestamp
            #     tracking_msg_utimes[adjusted_utime] = adjusted_timestamp

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
           
            # # Process C3_DEBUG_CURR messages
            # elif event.channel == "C3_ACTUAL":
            #     c3_actual_msg_utime = adjusted_utime
            #     timestamp = adjusted_timestamp

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
            #     c3_debug_msg_utime = adjusted_utime
            #     timestamp = adjusted_timestamp

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
            #     osc_debug_msg_utime = adjusted_utime
            #     timestamp = adjusted_timestamp
            #     if osc_debug_msg_utime in tracking_msg_utimes:
            #         print(f"Received OSC_DEBUG_FRANKA message at : {timestamp*1e-6}, utime: {osc_debug_msg_utime*1e-6}")
                

        event = log_file.read_next_event()

    # plot_utimes(franka_utimes, object_utimes, tracking_msg_utimes, osc_debug_msg_utimes)

    # TODO: Make the function definition be more general. But this should work
    plot_utime_differences(franka_utimes, tracking_msg_utimes)
    # plot_utime_differences(osc_debug_msg_utimes, tracking_msg_utimes)
    plot_controller_freq(tracking_msg_utimes)

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
    parser.add_argument('log_folder', type=str, help='Path to the folder containing the log file')
    
    args = parser.parse_args()
    log_folder = args.log_folder

    # Turn the folder into a file path
    log_number = log_folder.split("/")[-1][:6]  # Extract the last part of the folder name
    log_filepath = os.path.join(log_folder, f"simlog-{log_number}")  # Construct the log file path
    print(f"Parsing log at: {log_filepath}")

    # List of channels to care about.
    target_channels_and_message_types = {
        "TRACKING_TRAJECTORY_ACTOR": dairlib.lcmt_timestamped_saved_traj,
        "FRANKA_STATE_SIMULATION": dairlib.lcmt_robot_output,
        "OBJECT_STATE_SIMULATION": dairlib.lcmt_object_state,
        "C3_DEBUG_CURR": dairlib.lcmt_c3_output,
        "C3_ACTUAL": dairlib.lcmt_c3_state,
        "OSC_DEBUG_FRANKA": dairlib.lcmt_osc_output
    }
    
    # Call the function and get the data and timestamps
    process_lcm_log(log_filepath, target_channels_and_message_types)
