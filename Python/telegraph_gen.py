from dot80211params import Dot80211Params
from komondor import Komondor
from simconfig import SimConfig
from simresult import SimResult
from topogenerator import TopoGenerator
from visualize import Visualizer
import time
import sys

'''
Placement:
    STA     STA      STA
      \    /       /
       AP        AP
'''
if __name__ == "__main__":

    channel_cfg = int(sys.argv[1])
    print('Running w/ channel config = %d' % channel_cfg)

    # create sim params for each AP
    if channel_cfg == 0:
        # AP1: 20 MHz, CH:0
        d11p_1 = Dot80211Params(num_channels=2, min_channel_allowed=0, max_channel_allowed=0)
        # AP2: 20 MHz, CH:1
        d11p_2 = Dot80211Params(num_channels=2, min_channel_allowed=1, max_channel_allowed=1)
    elif channel_cfg == 1:
        # AP1: 40 MHz, DCB
        d11p_1 = Dot80211Params(num_channels=2, min_channel_allowed=0, max_channel_allowed=1)
        # AP2: 40 MHz, DCB
        d11p_2 = Dot80211Params(num_channels=2, min_channel_allowed=0, max_channel_allowed=1)
    elif channel_cfg == 2:
        # AP1: 40 MHz, CH:1-2
        d11p_1 = Dot80211Params(num_channels=3, min_channel_allowed=1, max_channel_allowed=2)
        # AP2: 20 MHz, CH:0
        d11p_2 = Dot80211Params(num_channels=1, min_channel_allowed=0, max_channel_allowed=0)
    elif channel_cfg == 3:
        # AP1: 80 MHz, DCB
        d11p_1 = Dot80211Params(num_channels=4, min_channel_allowed=0, max_channel_allowed=3)
        # AP2: 80 MHz, DCB
        d11p_2 = Dot80211Params(num_channels=4, min_channel_allowed=0, max_channel_allowed=3)
    else:
        assert False

    # create topology
    tg = TopoGenerator()
    # visualize results
    vis = Visualizer(channel_cfg)

    # export as cfg file
    cfg_fname = 'cfg/telegraph_gen_network_' + str(channel_cfg) + '.cfg'
    sc = SimConfig(cfg_fname)
    sc.create([d11p_1, d11p_2], tg)

    # for different distances
    sta_ap_distances = range(10, 100, 2)
    all_res = {}
    for sta_ap_distance in sta_ap_distances:
        print('... place nodes')
        tg.gen_fixed_placement(sta_ap_distance)

        # show node placement
        #vis.show_node_placement(tg)

        # execute komondor simulator
        print('... simulate for d=%f' % sta_ap_distance)
        res_fname = 'res/telegraph_statistics_' + str(channel_cfg) + '.cfg'
        sim = Komondor(cfg_fname, res_fname, sim_time=100)
        sim.run()

        print('... parse results')
        sr = SimResult(res_fname)
        sim_res = sr.parse_results()
        all_res[sta_ap_distance] = sim_res

    # show final res
    vis.plot_thr_vs_distance_bar_chart(sta_ap_distances, all_res)

