from dot80211params import Dot80211Params
from komondor import Komondor
from simconfig import SimConfig
from simresult import SimResult
from topogenerator import TopoGenerator
from visualize import Visualizer
import time
import sys

if __name__ == "__main__":

    num_channels = int(sys.argv[1])
    print('Running w/ %d channels' % num_channels)

    # create topo
    tg = TopoGenerator()
    # create sim params
    d11p = Dot80211Params(num_channels=num_channels)
    # visualize results
    vis = Visualizer(d11p)

    # for different distances
    sta_ap_distances = range(10, 100, 20)
    all_res = {}
    for sta_ap_distance in sta_ap_distances:
        tg.gen_fixed_placement(sta_ap_distance)

        # show node placement
        #vis.show_node_placement(tg)

        # export as cfg file
        out_fname = 'cfg/telegraph_gen_network.cfg'
        sc = SimConfig(out_fname)
        sc.create(d11p, tg)

        # execute komondor simulator
        print('Simulate for d=%f' % sta_ap_distance)
        if True:
            sim = Komondor()
            sim.run()
        else:
            print('Simulator mockup')
            time.sleep(1)

        # parse results
        res_fname = 'statistics.cfg'
        sr = SimResult(res_fname)
        sim_res = sr.parse_results()
        all_res[sta_ap_distance] = sim_res

    # show final res
    vis.plot_thr_vs_distance_bar_chart(sta_ap_distances, all_res)

