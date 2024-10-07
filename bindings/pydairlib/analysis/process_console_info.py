import re

# Received OSC_DEBUG_FRANKA message at : 5.935, utime: 5.914
# 	Received C3_DEBUG_CURR message at : 5.935454, utime: 5.701
# Received OSC_DEBUG_FRANKA message at : 5.93547, utime: 5.915

console_info = """

OSC:
	
	Time of receipt of robot state: 1728079175652461 at utime: 5.633
	Time of publish: 1728079175652680
	
	Time of receipt of robot state: 1728079175659450 at utime: 5.64
	TRACKING_TRAJECTORY_ACTOR message subscriber received msg at : 1728079175659463 with context time = 5.639
	Time of publish: 1728079175659604

	Time of receipt of robot state at : 1728079175811419 at utime: 5.792
	Time of publish at : 1728079175811607
	Time of receipt of robot state: 1728079175812418 at utime: 5.793
	TRACKING_TRAJECTORY_ACTOR message subscriber received msg at : 1728079175812438 with context time = 5.792
	Time of publish at : 1728079175812628


FRANKA C3 CONTROLLER: 
	Time of receipt of robot state: 1728079175659913 at utime: 5.633
		Compute plan begins: 1728079175660055
		utime is 5.633
		Context time is 5.633
		Compute plan ends: 1728079175811614
		Traj output send begins: 1728079175811988
		Traj output send ends: 1728079175811992
	TRACKING_TRAJECTORY_ACTOR message publisher published at : 1728079175812000 with context time = 5.633
	Time of publish: 1728079175812094

	Time of receipt of robot state at : 1728079175812260 at utime: 5.792
		Compute plan begins at : 1728079175812332
		utime is 5.792
		Context time is 5.792
		Compute plan ends at : 1728079175968018
		Traj output send begins at : 1728079175968544
		Traj output send ends at : 1728079175968549
	TRACKING_TRAJECTORY_ACTOR message publisher published at : 1728079175968562 with context time = 5.792
	Time of publish: 1728079175968705

OSC:
	Time of receipt of robot state at : 1728079175969431 at utime: 5.95
	TRACKING_TRAJECTORY_ACTOR message subscriber received msg at : 1728079175969448 with context time = 5.949
	Time of publish at : 1728079175969632

"""

real_sequence = """

OSC:

	Time of receipt of robot state: 1728079175652461 at utime: 5.633
	Time of publish: 1728079175652680
	
		
	Time of receipt of robot state: 1728079175659450 at utime: 5.64
	TRACKING_TRAJECTORY_ACTOR message subscriber received msg at : 1728079175659463 with context time = 5.639
	Time of publish: 1728079175659604


FRANKA C3 CONTROLLER: 
	Time of receipt of robot state: 1728079175659913 at utime: 5.633
		Compute plan begins: 1728079175660055
		utime is 5.633
		Context time is 5.633
		Compute plan ends: 1728079175811614
		Traj output send begins: 1728079175811988
		Traj output send ends: 1728079175811992
	TRACKING_TRAJECTORY_ACTOR message publisher published at : 1728079175812000 with context time = 5.633
	Time of publish: 1728079175812094

OSC:
	Time of receipt of robot state at : 1728079175811419 at utime: 5.792
	Time of publish at : 1728079175811607
	Time of receipt of robot state: 1728079175812418 at utime: 5.793
	TRACKING_TRAJECTORY_ACTOR message subscriber received msg at : 1728079175812438 with context time = 5.792
	Time of publish at : 1728079175812628
	
FRANKA C3 CONTROLLER: 
	Time of receipt of robot state at : 1728079175812260 at utime: 5.792
		Compute plan begins at : 1728079175812332
		utime is 5.792
		Context time is 5.792
		Compute plan ends at : 1728079175968018
		Traj output send begins at : 1728079175968544
		Traj output send ends at : 1728079175968549
	TRACKING_TRAJECTORY_ACTOR message publisher published at : 1728079175968562 with context time = 5.792
	Time of publish: 1728079175968705

OSC:
	Time of receipt of robot state at : 1728079175969431 at utime: 5.95
	TRACKING_TRAJECTORY_ACTOR message subscriber received msg at : 1728079175969448 with context time = 5.949
	Time of publish at : 1728079175969632

"""

# Extract all large integer timestamps (but not the decimal utimes) using regex
timestamps = re.findall(r'(\d{13,})', console_info)  # Extract timestamps with 13+ digits

# Convert timestamps to integers
timestamps = [int(t) for t in timestamps]

# Get the base time for offsetting (first timestamp in the log)
base_time = timestamps[0]
offset_timestamps = [(t - base_time)/ 1e6 for t in timestamps]

# Function to replace original timestamps with offset values in the log
def replace_timestamp(match):
    global offset_index
    new_timestamp = offset_timestamps[offset_index]
    offset_index += 1
    return f'{new_timestamp:.6f}'

# Initialize the offset index
offset_index = 0

# Replace timestamps (13+ digits) in the original log with offset values
new_log = re.sub(r'\d{13,}', replace_timestamp, console_info)

# Print the updated log with offsets applied
print(new_log)


# # Extract all timestamps (both message times and utimes) using regex
# timestamps = re.findall(r'at : (\d+\.\d+)', console_info)  # Extract 'message at' times
# utimes = re.findall(r'utime: (\d+\.\d+)', console_info)    # Extract 'utime'

# # Convert to floats
# timestamps = [float(t) for t in timestamps]
# utimes = [float(u) for u in utimes]

# # Base (first) time for both message timestamps and utimes
# base_time = timestamps[0]
# base_utime = utimes[0]

# # Offset the times relative to the first message time
# offset_timestamps = [(t - base_time) for t in timestamps]
# offset_utimes = [(u - base_utime) for u in utimes]

# # Function to replace the original timestamps with the offset ones
# def replace_timestamp(match):
#     global timestamp_index
#     new_time = offset_timestamps[timestamp_index]
#     timestamp_index += 1
#     return f'at : {new_time:.6f}'

# # Function to replace the original utimes with the offset ones
# def replace_utime(match):
#     global utime_index
#     new_utime = offset_utimes[utime_index]
#     utime_index += 1
#     return f'utime: {new_utime:.6f}'

# # Replace all timestamps and utimes in the log
# timestamp_index = 0
# updated_log = re.sub(r'at : \d+\.\d+', replace_timestamp, console_info)

# utime_index = 0
# updated_log = re.sub(r'utime: \d+\.\d+', replace_utime, updated_log)

# # Print the updated log with offsets applied to all timestamps and utimes
# print(updated_log)

# # Extract all timestamps
# timestamps = re.findall(r'(\d+)', console_info)

# # Convert timestamps to integers
# timestamps = [int(t) for t in timestamps]

# # Offset the timestamps so the first one is 0
# base_time = timestamps[0]
# offset_timestamps = [(t - base_time) for t in timestamps]

# # Replace old timestamps with the offset timestamps in the log
# def replace_timestamp(match):
#     global offset_index
#     new_timestamp = offset_timestamps[offset_index]
#     offset_index += 1
#     return f'{new_timestamp}'

# offset_index = 0
# new_log = re.sub(r'\d+', replace_timestamp, console_info)

# print(new_log)