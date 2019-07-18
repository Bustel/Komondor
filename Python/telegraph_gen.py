from dot80211params import Dot80211Params
from komondor import Komondor
from simconfig import SimConfig
from simresult import SimResult
from topogenerator import TopoGenerator
from visualize import Visualizer


import multiprocessing
import functools
import math
import statistics
import os
from progress.bar import Bar

from operator import itemgetter


def sim_worker( tupel_num_param_comb, sta_ap_distances, ap_vector):
    global queue
    channel_cfg, param_comb = tupel_num_param_comb
    all_res = {}
    for sta_ap_distance in sta_ap_distances:
        tg.placement_from_vector(ap_vector, sta_ap_distance)

        # export as cfg file
        cfg_fname = 'cfg/telegraph_gen_network_' + str(channel_cfg) + '.cfg'
        sc = SimConfig(cfg_fname)
        sc.create(param_comb, tg)

        # execute komondor simulator
        res_fname = 'res/telegraph_statistics_{:d}_{:d}m.cfg'.format(channel_cfg,
                                                         sta_ap_distance)

        sim = Komondor(cfg_fname, res_fname, sim_time=sim_time)
        sim.run()

        sr = SimResult(res_fname)
        sim_res = sr.parse_results()
        all_res[sta_ap_distance] = sim_res

    queue.put(1)
    return all_res
   

def janes_fairness(sim_result):
    def square(a):
        return math.pow(a,2)

    flow_vals = list(map(lambda flow: sim_result[flow]['throughput'],
                         sim_result.keys())) 
    a = square(sum(flow_vals))
    b = sum(map(square, flow_vals)) * len(flow_vals)
    return a/b

    
def mean_janes_fairness(sim_result_dict):
    return statistics.mean(map(lambda dist: janes_fairness(sim_result_dict[dist]),
                        sim_result_dict.keys()))


def min_flow_rate(sim_result):
    flow_vals = list(map(lambda flow: sim_result[flow]['throughput'],
                         sim_result.keys())) 
    return min(flow_vals)


def mean_min_flow_rate(sim_result_dict):
    return statistics.mean(map(lambda dist: min_flow_rate(sim_result_dict[dist]),
                        sim_result_dict.keys()))


def total_flow(sim_result):
    flow_vals = list(map(lambda flow: sim_result[flow]['throughput'],
                         sim_result.keys())) 
    return sum(flow_vals)


def mean_total_flow(sim_result_dict):
    return statistics.mean(map(lambda dist: total_flow(sim_result_dict[dist]),
                        sim_result_dict.keys()))


def init(q):
    global queue
    queue = q


if __name__ == "__main__":

    sim_time = 1
    step_sz = 10
    max_dist = 51
    num_processes = os.cpu_count()


    # create topology
    tg = TopoGenerator()
    #vis = Visualizer(channel_cfg)

    ap_vector = [4,1,2]
    param_combinations = list(Dot80211Params.get_all_combinations(num_aps=len(ap_vector),
                                             dcb_modes=[1]))

    #SHORTEN FOR DEBUG
    param_combinations = param_combinations[0:len(param_combinations)//100]

    print('Generated {0} parameter combinations.'.format(len(param_combinations)))
    bar = Bar('Simulating combinations', max=len(param_combinations))
    q = multiprocessing.Queue()


    mean_min_flow_rates = []
    mean_fairness = []


    with multiprocessing.Pool(processes=num_processes, initializer=init,
                              initargs=(q,)) as pool:
        worker = functools.partial(sim_worker, 
                                   sta_ap_distances=range(10, max_dist,step_sz),
                                   ap_vector=ap_vector,
                                   )



        async_res = pool.map_async(worker, enumerate(param_combinations)) 
        while not async_res.ready():
            try: 
                q.get(timeout=5.0)
                bar.next()
            except:
                # Race condition
                # async might not be ready yet
                # but queue is already empty
                pass
        while not q.empty():
            try:
                q.get(False)
                bar.next()
            except:
                print('Timeout while waiting.')
                pass

        combined_results = async_res.get()
        bar.finish()
        print('Simulations completed. Processing results.')

        mean_fairness = pool.map(mean_janes_fairness,combined_results)
        mean_min_flow_rates = pool.map(mean_min_flow_rate, combined_results)
        mean_total_flow = pool.map(mean_total_flow, combined_results)
        
    counter = 1
    for comb, mean_min_rate, fairness, total_flow in sorted(zip(param_combinations,
                                                    mean_min_flow_rates,
                                                    mean_fairness,
                                                    mean_total_flow),
                                      key=itemgetter(1), reverse=True):

        if counter > 10:
            break
        print('### {0} ####'.format(counter))
        print('Mean minimum flow rate: {:.2f} Mbps'.format(mean_min_rate/1e6))
        print('Total flow rate: {:.2f} Mbps'.format(total_flow/1e6))
        print('Jane\'s fairness: {:.2f}'.format(fairness))
        print('AP1: {0}'.format(comb[0].to_string()))
        print('AP2: {0}'.format(comb[1].to_string()))
        print('AP3: {0}'.format(comb[2].to_string()))

        counter += 1
        
    print('Done')
