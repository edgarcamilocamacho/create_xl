#!/usr/local/bin/python3

import subprocess
import sys
import signal
import psutil
import threading
import subprocess

console_attr = {'PURPLE': '\033[95m',
                'CYAN' : '\033[96m',
                'DARKCYAN': '\033[36m',
                'BLUE':'\033[94m',
                'GREEN': '\033[92m',
                'YELLOW':'\033[93m',
                'RED':'\033[91m',
                'BOLD': '\033[1m',
                'UNDERLINE' : '\033[4m',
                'END' :'\033[0m',
                'OTHER1': '\033]2;',
                'OTHER2': '\033[31m',
                'OTHER3': '\033[33m',} 

def kill_child_processes(parent_pid, sig=signal.SIGTERM):
    try:
        parent = psutil.Process(parent_pid)
        print(parent)
    except psutil.NoSuchProcess:
        print("parent process not existing")
        return
    children = parent.children(recursive=True)
    print(children)
    for process in children:
        print("try to kill child: " + str(process))
        process.send_signal(sig)

def remove_console_attr(text):
    for attr in console_attr.values():
        text = text.replace(attr, '')
    return text

class Roslaunch(object):
    """
    Roslaunch wrapped into a subprocess.
    Singleton implementation prevents from creating more than one instance.
    """
    def __init__(self, args):
        self.args = args
        self.log = ""

    def run(self):
        try:
            self.process = subprocess.Popen(self.args,
                                            stdout=subprocess.PIPE, 
                                            stderr=subprocess.PIPE, 
                                            shell=True,
                                            encoding='utf-8',
                                            errors='replace'
                                            )
            self.pid = self.process.pid
        except subprocess.CalledProcessError as err:
              raise err
        else:
            self.thread = threading.Thread(target=self.check_output)
            self.thread.start()
            self.thread2 = threading.Thread(target=self.check_errors)
            self.thread2.start()        
    
    def check_output(self):
        while True:
            realtime_output = self.process.stdout.readline()
            realtime_output = remove_console_attr(realtime_output)
            
            if realtime_output == '' and self.process.poll() is not None:
                break
            if realtime_output:
                print(realtime_output.strip())
            
    def check_errors(self):
        while True:
            realtime_output = self.process.stderr.readline()
            realtime_output = remove_console_attr(realtime_output)
            if realtime_output == '' and self.process.poll() is not None:
                break
            if realtime_output:
                sys.stderr.write(realtime_output.strip())
    
    def __del__(self):
        self.terminate()
    
    def terminate(self):
        print("try to kill child pids of " + str(self.args) + " with pid: " + str(self.pid))
        kill_child_processes(self.pid)
        self.process.terminate()
        self.process.wait()  # important to prevent from zombie process
