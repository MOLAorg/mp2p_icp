#!/usr/bin/python3
# Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
#                         and individual contributors.
# SPDX-License-Identifier: BSD-3-Clause

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
