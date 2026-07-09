#!/usr/bin/python3

import glob, os

for file in glob.glob("*.bin"):
    filename, file_extension = os.path.splitext(file)
    out=filename+'.mm'
    print('Processing: ' + file + ' name:' + filename)
    os.system('kitti2mm -i ' + file + ' -o ' + out + ' --id ' + filename)
