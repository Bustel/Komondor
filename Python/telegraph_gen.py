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
    sim_time = 30
    step_sz = 1
    print('Running w/ channel config = %d' % channel_cfg)

    # create sim params for each AP
    if channel_cfg == 0:
        # AP1: 160 MHz, CH:0-7 DCB: ALWAYS_MAX
        d11p_1 = Dot80211Params(num_channels=8, min_channel_allowed=0, max_channel_allowed=7)
        
        # AP2: 160 MHz, CH:0-7
        d11p_2 = Dot80211Params(num_channels=8, min_channel_allowed=0, max_channel_allowed=7)

        # AP3: 160 MHz, CH:0-7
        d11p_3 = Dot80211Params(num_channels=8, min_channel_allowed=0, max_channel_allowed=7)
    elif channel_cfg == 1:

        # AP1: 160 MHz, CH:0-7 DCB: STATIC
        d11p_1 = Dot80211Params(num_channels=8, min_channel_allowed=0,max_channel_allowed=7, channel_bonding_mode=1)
        
        # AP2: 160 MHz, CH:0-7 
        d11p_2 = Dot80211Params(num_channels=8, min_channel_allowed=0, max_channel_allowed=7, channel_bonding_mode=1)

        # AP3: 160 MHz, CH:0-7
        d11p_3 = Dot80211Params(num_channels=8, min_channel_allowed=0, max_channel_allowed=7, channel_bonding_mode=1)

    elif channel_cfg == 2:

        # AP1: 80 MHz, CH:0-3 DCB: ALWAYS_MAX
        d11p_1 = Dot80211Params(num_channels=8, min_channel_allowed=0,max_channel_allowed=3, channel_bonding_mode=3)
        
        # AP2: 40 MHz, CH:6-7 DCB: ALWAYS_MAX
        d11p_2 = Dot80211Params(num_channels=8, min_channel_allowed=6, max_channel_allowed=7, channel_bonding_mode=3)

        # AP3: 80 MHz, CH:4-7 DCB: ALWAYS_MAX
        d11p_3 = Dot80211Params(num_channels=8, min_channel_allowed=4, max_channel_allowed=7, channel_bonding_mode=3)

    elif channel_cfg == 3:

        # AP1: 80 MHz, CH:0-3 DCB: STATIC 
        d11p_1 = Dot80211Params(num_channels=8, min_channel_allowed=0,max_channel_allowed=3, channel_bonding_mode=1)
        
        # AP2: 40 MHz, CH:6-7 DCB: STATIC 
        d11p_2 = Dot80211Params(num_channels=8, min_channel_allowed=6, max_channel_allowed=7, channel_bonding_mode=1)

        # AP3: 40 MHz, CH:4-5 DCB: STATIC 
        d11p_3 = Dot80211Params(num_channels=8, min_channel_allowed=4, max_channel_allowed=5, channel_bonding_mode=1)


    else:
        assert False

    # create topology
    tg = TopoGenerator()
    # visualize results
    vis = Visualizer(channel_cfg)

    # for different distances
    sta_ap_distances = range(10, 100, step_sz)
    all_res = {}
    for sta_ap_distance in sta_ap_distances:
        print('... place nodes')
        tg.placement_from_vector([4, 1, 2], sta_ap_distance)

        # export as cfg file
        cfg_fname = 'cfg/telegraph_gen_network_' + str(channel_cfg) + '.cfg'
        sc = SimConfig(cfg_fname)
        sc.create([d11p_1, d11p_2, d11p_3], tg)

        # show node placement
        #vis.show_node_placement(tg)

        # execute komondor simulator
        print('... simulate for d=%f' % sta_ap_distance)
        res_fname = 'res/telegraph_statistics_' + str(channel_cfg) + '.cfg'
        sim = Komondor(cfg_fname, res_fname, sim_time=sim_time)
        sim.run()

        print('... parse results')
        sr = SimResult(res_fname)
        sim_res = sr.parse_results()
        all_res[sta_ap_distance] = sim_res

    # show final res
    vis.plot_thr_vs_distance_bar_chart(sta_ap_distances, all_res)

