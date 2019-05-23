'''
    Topology generator
'''
class TopoGenerator:

    def __init__(self):
        self.aps = {}
        self.stas = {}
        self.bss = {}

        self.num_aps = None
        self.num_stas_per_ap = None

    def gen_fixed_placement(self, sta_ap_distance, sta_sta_distance=10, ap_ap_distance=20):
        '''
        Placement:
            STA  STA      STA
            |    /         |
            |   /          |
            |  /           |
            | /            |
            AP             AP 
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


    def placement_from_vector(self, ap_vector, sta_ap_distance, sta_sta_distance=10,ap_ap_distance=20):

        # TODO Shouldn't we place the APs in the middle of the stations (on the
        # x-axis)  

        self.num_aps = len(ap_vector)
        self.num_stas_per_ap = ap_vector
        
        sta_id = 0
        for i, num_stas in enumerate(ap_vector):
            self.bss[i] = []
            self.aps[i] = [i*ap_ap_distance, 0 ,0]

            for _ in range(0,num_stas):
                self.stas[sta_id] = [sta_id*sta_sta_distance, sta_ap_distance, 0]
                self.bss[i].append(sta_id)
                sta_id += 1

    def __repr__(self):
        return "AP: {0} STAs: {1} BSS: {2} num_aps:{3} num_stas_per_ap: {4}".format(self.aps, self.stas, self.bss, self.num_aps, self.num_stas_per_ap)
