import os
import cv2
import warnings
import numpy as np
from dataclasses import dataclass


@dataclass
class FrameEdits:
    aspect: float
    yshift: int
    mirror: bool


def extract_frames(start_time: float, end_time: float, num_frames: int,
                   video_filename: str, save_folder: str, prefix: str = '',
                   frame_edits: FrameEdits = None):
    """
    Extracts frames from a video file at evenly spaced timestamps and saves them
    as JPEG images.

    Args:
        start_time: The starting timestamp in seconds from which to extract
        frames.
        end_time: The ending timestamp in seconds until which to extract frames.
        Set end_time to -1 to use the whole video. If end_time is greater
        than the video duration, it will be automatically set to the video
        duration.
        num_frames: The number of frames to extract, evenly spaced from
        start_time to end_time
        video_filename: The filename of the input video file.
        save_folder: The folder to save the extracted frames.
        prefix: A prefix before the frame number in the save filename of the
        extracted frames
        frame_edits: Transformation to apply to each video frame - see
        FrameEdits dataclass

    Returns: Nothing

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
            aspect = frame_edits.aspect
            yshift = -frame_edits.yshift
            h = frame.shape[0]
            w = frame.shape[1]
            h_new = w / aspect
            center_y = int(h / 2)
            dy = int(h_new / 2)
            offset = int(yshift * h)
            frame = frame[offset + center_y - dy: offset + center_y + dy, :]

            if frame_edits.mirror:
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
