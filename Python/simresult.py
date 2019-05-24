import configparser
import os

'''
    Parsing the results
'''
class SimResult:

    def __init__(self, res_fname):
        self.res_fname = res_fname
        self.config = configparser.ConfigParser()

    def parse_results(self):
        self.config.read(self.res_fname)

#        print('Loaded results file: %s' % self.res_fname)

        res = {}
        flow_id = 1
        for sta_id in self.config.sections():
            thr_sta = float(self.config[sta_id]['throughput'])
            res['FLOW' + str(flow_id)] = {
            'src': 'TBD',
            'dst': 'TBD',
            'throughput': thr_sta,
            'delay': -42
            }
            flow_id = flow_id + 1

        return res
