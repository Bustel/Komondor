import configparser
import random

'''
    Parsing the results
'''
class SimResult:

    def __init__(self, res_fname):
        self.res_fname = res_fname
        self.config = configparser.ConfigParser()

    def parse_results(self):
        self.config.read(self.res_fname)

        print('Loaded results file: %s' % self.res_fname)

        # TBD
        # JUST MOCKUP
        res = {}
        res['FLOW1'] = {
            'src': 'Node_AP_A',
            'dst': 'Node_STA_A1',
            'throughput': random.randint(1e6,5e6),
            'delay': 0.01
        }

        res['FLOW2'] = {
            'src': 'Node_AP_A',
            'dst': 'Node_STA_A2',
            'throughput': random.randint(1e6,5e6),
            'delay': 0.01
        }

        res['FLOW3'] = {
            'src': 'Node_AP_B',
            'dst': 'Node_STA_B1',
            'throughput': random.randint(1e6,5e6),
            'delay': 0.01
        }

        return res