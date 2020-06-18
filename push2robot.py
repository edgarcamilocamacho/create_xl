#!/usr/bin/python

import os
import sys

if 'force_all' in sys.argv:
    force_all = True
else:
    force_all = False

ROBOT_IP="192.168.20.24"
ROBOT_USER="up"
ROBOT_PASS="up"
TARGET_FOLDER="/home/"+ROBOT_USER+"/ros/create_ws/src/create_xl"

dir_path = os.path.dirname(os.path.realpath(__file__))
dummy_path = dir_path + '/.push2robot_dummy'
try:
    dummy_date = os.path.getmtime(dummy_path)
except:
    dummy_date = 0.0

packages = next(os.walk(dir_path))[1]
for package in packages:
    if not package.startswith('.') and package!='doc':
        files_in_package = [os.path.join(dp, f) for dp, dn, filenames in os.walk(dir_path+'/'+package) for f in filenames]
        date_of_package = max([os.path.getmtime(file_) for file_ in files_in_package])
        if date_of_package>dummy_date or force_all:
            print('Updating "' + package + '" package...')
            comm = 'sshpass -p {} ssh {}@{} "rm -rf {}/{}"'.format(ROBOT_PASS, ROBOT_USER, ROBOT_IP, TARGET_FOLDER, package) 
            # print(comm)
            os.system(comm)
            comm = 'sshpass -p {} scp -r {} {}@{}:{}'.format(ROBOT_PASS, dir_path+'/'+package, ROBOT_USER, ROBOT_IP, TARGET_FOLDER)
            # print(comm)
            os.system(comm)

os.system('touch ' + dummy_path)

print('Updating finished')