import subprocess
import sys
import os

'''
    Wrapper for Komondor simulator
'''
class Komondor:

    def __init__(self, sim_time=100):
        self.sim_time = sim_time

    def run(self):
        try:
            try:
                os.remove("statistics.cfg")
            except:
                pass

            p = subprocess.Popen(
                '../Code/build/komondor_main -t ' + str(self.sim_time) + ' -s 112 cfg/telegraph_gen_network.cfg' + ' --stats=./',
                shell=True,
                stdout=subprocess.PIPE,
                stderr = subprocess.PIPE
            )
        except FileNotFoundError as ferr:
            print("File not found error: {0}".format(ferr), file=sys.stderr)
        except:
            print("Unexpected error:", sys.exc_info()[0], file=sys.stderr)
        else:
            stdout = p.stdout.read()
            print(stdout)
            stderr = p.stderr.read()
            print(stderr)
            pass