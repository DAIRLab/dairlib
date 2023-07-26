import subprocess
import os
import sys
from franka_logging_utils import get_most_recent_logs

def main():
  ''' set up log directory paths '''
  if len(sys.argv) == 1:
    logdir, log_num = get_most_recent_logs()
  else:
    # process frames in directory passed in through cmd line
    filename = sys.argv[1]
    path_components = os.path.normpath(filename).split(os.sep)
    log_num = path_components[-2]
    logdir = ''
    for comp in path_components[1:]:
      if comp == log_num:
        break
      logdir += '/{}'.format(comp)
  framesdir = "{}/{}/frames".format(logdir, log_num)

  if log_num is None:
      print("Did not find logs in {}".format(logdir))
      return

  print("Processing frames in " + framesdir)
  fps = 30
  os.chdir(framesdir)
  subprocess.call(['ffmpeg', '-r', str(fps), '-f', 'image2', \
    '-s', '640x480', '-i', 'frame%04d.jpg', '-vcodec', 'libx264', \
    '-crf', '25', '-pix_fmt', 'yuv420p', '../video{}.mp4'.format(log_num)])
  print("Finished processing video")

if __name__ == "__main__":
  main()