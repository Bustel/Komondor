import subprocess
import sys

'''
    Wrapper for Komondor simulator
'''
class Komondor:

    def __init__(self):
        pass

    def run(self):
        try:
            p = subprocess.Popen(
                '../Code/build/komondor_main -t 10 -s 112 cfg/telegraph_gen_network.cfg',
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