#!/usr/bin/python3
# Copyright (C) 2018-2026 Jose Luis Blanco, University of Almeria,
#                         and individual contributors.
# SPDX-License-Identifier: BSD-3-Clause

import glob, os

for file in glob.glob("*.bin"):
    filename, file_extension = os.path.splitext(file)
    out=filename+'.mm'
    print('Processing: ' + file + ' name:' + filename)
    os.system('kitti2mm -i ' + file + ' -o ' + out + ' --id ' + filename)
