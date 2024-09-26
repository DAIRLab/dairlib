import os
import sys
import glob
import subprocess
from pathlib import Path

def convert_mp4(file_path):
    temp_file = file_path + '.temp.mp4'
    
    # Command to convert the video using ffmpeg
    command = [
        'ffmpeg',
        '-i', file_path,
        '-vcodec', 'libx264',
        '-b:a', '12000k',
        '-maxrate', '16000k',
        '-bufsize', '32000k',
        temp_file
    ]
    
    try:
        # Run the ffmpeg command
        subprocess.run(command, check=True)
        subprocess.run(['rm', file_path])

        new_file = file_path.split('.')[0] + '.mp4'
        subprocess.run(['mv', temp_file, new_file])
        
        # Remove the original file and rename the temp file
        print(f"Successfully converted and replaced: {file_path}")
        
    except subprocess.CalledProcessError as e:
        print(f"Error converting {file_path}: {e}")

def process_directory(root_dir):
    # Use Path object to walk through the directory
    file_extensions = ['mp4', 'MP4', 'MTS']
    
    for e in file_extensions:
        for filename in glob.glob(root_dir + f'/**/*.{e}', recursive=True):
            print(filename)
            convert_mp4(filename)
    

if __name__ == "__main__":
    # Set the directory you want to start from
    root_directory = sys.argv[1].strip()
    if not os.path.isdir(root_directory):
        print("Invalid directory. Please check the path and try again.")
    else:
        process_directory(root_directory)
