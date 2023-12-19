import os
import cv2
import warnings

import numpy as np


def extract_frames(start_time, end_time, num_frames, video_filename,
                   save_folder, prefix='', frame_edits=None):
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
        prefix (str): A prefix before the frame number in the filename

    Returns:
        None
    """

    os.makedirs(save_folder, exist_ok=True)

    # Open the video file
    video = cv2.VideoCapture(video_filename)

    # Get video properties
    fps = video.get(cv2.CAP_PROP_FPS)
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

        if frame_edits is not None:
            aspect = frame_edits['aspect']
            yshift = -frame_edits['yshift']
            h = frame.shape[0]
            w = frame.shape[1]
            h_new = w / aspect
            center_y = int(h / 2)
            dy = int(h_new / 2)
            offset = int(yshift * h)
            frame = frame[offset + center_y - dy: offset + center_y + dy, :]

            if frame_edits['mirror']:
                frame = np.fliplr(frame)

        if ret:
            # Save the frame as a JPEG image
            frame_filename = os.path.join(save_folder, f"{prefix}-{i}.jpg")
            cv2.imwrite(frame_filename, frame)
            frame_index += int(interval * fps)
            timestamp += interval

        # Break the loop if we reach the end frame or exceed the video duration
        if frame_index > end_frame or timestamp > video_duration:
            break

    # Release the video file
    video.release()
