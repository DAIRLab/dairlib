import subprocess
import os
import glob
import codecs
import sys
from datetime import date

def main():
  ''' set up log directory paths '''
  if len(sys.argv) == 1:
    curr_date = date.today().strftime("%m_%d_%y")
    year = date.today().strftime("%Y")
    logdir = "{}/adam_ws/logs/{}/{}".format(os.getenv('HOME'), year, curr_date)

    os.chdir(logdir)
    current_logs = sorted(glob.glob('*'))
    if current_logs:
        if current_logs[-1] == 'log_descriptions.txt':
            last_log = int(current_logs[-2])
        else:
            last_log = int(current_logs[-1])
        log_num = "{:02}".format(last_log)
    else:
        log_num = '00'
  else:
    filename = sys.argv[1]
    path_components = os.path.normpath(filename).split(os.sep)
    log_num = path_components[-2]
    logdir = ''
    for comp in path_components[1:]:
      if comp == log_num:
        break
      logdir += '/{}'.format(comp)
  framesdir = "{}/{}/frames".format(logdir, log_num)

  print("Processing frames in " + framesdir)
  fps = 30
  os.chdir(framesdir)
  subprocess.Popen(['ffmpeg', '-r', str(fps), '-f', 'image2', \
    '-s', '640x480', '-i', 'frame%04d.jpg', '-vcodec', 'libx264', \
    '-crf', '25', '-pix_fmt', 'yuv420p', '../video{}.mp4'.format(log_num)])
  print("Finished processing video")

if __name__ == "__main__":
  main()