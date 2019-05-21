import subprocess
import sys
import os

'''
    Wrapper for Komondor simulator
'''
class Komondor:

    def __init__(self, cfg_fname, res_fname, sim_time=100, debug=False):
        self.cfg_fname = cfg_fname
        self.res_fname = res_fname
        self.sim_time = sim_time
        self.debug = debug

    def run(self):
        try:
            try:
                os.remove(self.res_fname)
            except:
                pass

            p = subprocess.Popen(
                '../Code/build/komondor_main -t ' + str(self.sim_time) + ' -s 112 ' + self.cfg_fname + ' --stats=' + self.res_fname,
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
            if self.debug:
                print(stdout)
            stderr = p.stderr.read()
            if self.debug:
                print(stderr)
            pass