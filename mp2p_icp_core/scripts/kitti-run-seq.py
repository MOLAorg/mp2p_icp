#!/usr/bin/python3

import glob, os
import sys

dir = sys.argv[1]
cfg = sys.argv[2]
print('Processing directory      : ' + dir)
print('Using ICP pipeline config : ' + cfg)

for i in range(0,1000):
    f =  dir + '/{:06d}.mm'.format(i)
    fnext =  dir + '/{:06d}.mm'.format(i+1)
    cmd = 'icp-run --input-global ' + f + ' --input-local ' + fnext + ' --config-filters-local ' + cfg + ' -c ' + cfg + ' -d'
    print(cmd)
    os.system(cmd)
