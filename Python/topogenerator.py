'''
    Topology generator
'''
class TopoGenerator:

    def __init__(self):
        self.aps = {}
        self.stas = {}
        self.bss = {}

    def gen_fixed_placement(self, sta_ap_distance, sta_sta_distance=10, ap_ap_distance=20):
        '''
        Placement:
            STA     STA      STA
              \    /       /
               AP        AP
        '''
        self.num_aps = 2
        self.num_stas_per_ap = [2, 1]

        self.aps[0] = [0, 0, 0]
        self.aps[1] = [ap_ap_distance, 0, 0]

        sta_id = 0
        for ap_i in range(0, self.num_aps):
            self.bss[ap_i] = []
            for sta_i in range(0, self.num_stas_per_ap[ap_i]):
                self.stas[sta_id] = [sta_id*sta_sta_distance, sta_ap_distance, 0]
                self.bss[ap_i].append(sta_id)
                sta_id = sta_id + 1

