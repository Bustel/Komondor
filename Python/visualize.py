import matplotlib.pyplot as plt
import numpy as np

'''
    Visualize the results
'''
class Visualizer:

    def __init__(self, sta_ap_distances, all_sim_res):
        self.sta_ap_distances = sta_ap_distances
        self.all_sim_res = all_sim_res

    def plot_thr_vs_distance_bar_chart(self):

        # convert sim_res into numpy matrix
        #data = np.zeros(shape=(len(self.all_sim_res.keys()), len(next(iter(self.all_sim_res.values())))))
        data = np.empty((0,3), float)

        for dist_k in self.all_sim_res.keys():
            row = []
            for flow_k in self.all_sim_res[dist_k]:
                thr = self.all_sim_res[dist_k][flow_k]['throughput']
                row.append(thr)
            data = np.vstack([data, row])

        if False:
            plt.imshow(data);
            plt.colorbar()
            plt.show()

        for i in range(data.shape[1]):
            plt.plot(self.sta_ap_distances, data[:,i] / 1e6, '.-', label='Flow: ' + str(i))

        plt.xlabel('Distance AP to STA [m]')
        plt.ylabel('Throughput [Mbps]')
        plt.grid()
        plt.title('Scenario')
        plt.legend(loc='upper right')
        plt.show()