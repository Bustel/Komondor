from dot80211params import Dot80211Params
from komondor import Komondor
from simconfig import SimConfig
from topogenerator import TopoGenerator

if __name__ == "__main__":
    # create topo
    tg = TopoGenerator()

    sta_ap_distance = 25
    tg.gen_fixed_placement(sta_ap_distance)

    # create sim params
    d11p = Dot80211Params()

    # export as cfg file
    out_fname = 'cfg/telegraph_gen_network.cfg'
    sc = SimConfig(out_fname)
    sc.create(d11p, tg)

    # execute komondor simulator
    sim = Komondor()
    sim.run()