#!/usr/bin/env python
import subprocess

'''
import argparse

def remove_launchfile_generated_args(self, arg_strings):
    new_arg_strings = []
    for arg_string in arg_strings:
        if not arg_string.startswith('__name:=') and not arg_string.startswith('__log:='):
            new_arg_strings.append(arg_string)
    return new_arg_strings



parser = argparse.ArgumentParser(description='Launch webots nao simulator');
parser.add_argument('webots_directory', action="store",
                help='string containing the webots directory');

args = parser.parse_args();
'''
import sys
webotsDirectory = sys.argv[1]

args = (webotsDirectory + "/webots-bin", "--minimize", webotsDirectory 
+'/projects/robots/nao/worlds/nao.wbt')
popen = subprocess.Popen(args, stdout=subprocess.PIPE)
popen.wait()
output = popen.stdout.read()

print('-------------------------------------------------------------- ASDFASD')

import time
time.sleep(15)
