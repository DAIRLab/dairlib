import os
import cv2
import warnings


def extract_frames(start_time, end_time, num_frames, video_filename, save_folder):
    """
    Extracts frames from a video file at evenly spaced timestamps and saves them
    as JPEG images.

    Args:
        start_time (float): The starting timestamp in seconds from which to
                            extract frames.
        end_time (float): The ending timestamp in seconds until which to
                          extract frames.
                          Set end_time to -1 to use the whole video.
                          If end_time is greater than the video duration, it
                          will be automatically set to the video duration.
        num_frames (int): The number of frames to extract.
        video_filename (str): The filename of the input video file.
        save_folder (str): The folder to save the extracted frames.

    Returns:
        None
    """

    os.makedirs(save_folder, exist_ok=True)

    # Open the video file
    video = cv2.VideoCapture(video_filename)

    # Get video properties
    fps = video.get(cv2.CAP_PROP_FPS)
    print(f'fps: {fps}')
    total_frames = int(video.get(cv2.CAP_PROP_FRAME_COUNT))
    video_duration = total_frames / fps

    # Check if end_time is greater than the video duration
    if end_time < 0:
        end_time = video_duration
    if end_time > video_duration:
        warnings.warn("Specified end_time is greater than the video duration.")
        end_time = video_duration

    # Calculate the time interval between frames
    interval = (end_time - start_time) / (num_frames - 1)

    # Calculate the starting and ending frame indices
    start_frame = int(start_time * fps)
    end_frame = int(end_time * fps)

    # Set the initial frame index
    frame_index = start_frame

    # Set the initial timestamp
    timestamp = start_time

    # Iterate over frames and save them at evenly spaced timestamps
    for i in range(num_frames):
        # Set the frame index and timestamp
        video.set(cv2.CAP_PROP_POS_FRAMES, frame_index)
        ret, frame = video.read()

        if ret:
            # Save the frame as a JPEG image
            frame_filename = os.path.join(save_folder, f"{i}.jpg")
            cv2.imwrite(frame_filename, frame)
            frame_index += int(interval * fps)
            timestamp += interval

        # Break the loop if we reach the end frame or exceed the video duration
        if frame_index > end_frame or timestamp > video_duration:
            break

    # Release the video file
    video.release()