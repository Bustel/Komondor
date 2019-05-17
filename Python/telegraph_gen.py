from dot80211params import Dot80211Params
from komondor import Komondor
from simconfig import SimConfig
from simresult import SimResult
from topogenerator import TopoGenerator
from visualize import Visualizer

if __name__ == "__main__":
    # create topo
    tg = TopoGenerator()

    # for different distances
    sta_ap_distances = range(10, 50, 10)
    all_res = {}
    for sta_ap_distance in sta_ap_distances:
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

        # parse results
        res_fname = 'res/telegraph_gen_network_res.cfg'
        sr = SimResult(res_fname)
        sim_res = sr.parse_results()
        all_res[sta_ap_distance] = sim_res

    # visualize results
    vis = Visualizer(sta_ap_distances, all_res)
    vis.plot_thr_vs_distance_bar_chart()
