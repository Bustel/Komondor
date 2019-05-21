import matplotlib.pyplot as plt
import numpy as np
import networkx as nx

'''
    Visualize the results
'''
class Visualizer:

    def __init__(self, channel_cfg, folder='./plots/'):
        self.channel_cfg = channel_cfg
        self.folder = folder

    def show_node_placement(self, tg):

        pos = {}
        edge_lst = []
        sta_id = 0
        val_map = {}
        for ap in tg.bss:
            ap_name = 'AP_' + chr(65 + ap)
            pos[ap_name] = (tg.aps[ap][0], tg.aps[ap][1])
            val_map[ap_name]  = 0.2
            #for sta in tg.bss[ap]:
            for sta in range(0, tg.num_stas_per_ap[ap]):
                sta_name = 'STA_' + chr(65 + ap) + '' + str(sta)
                val_map[sta_name] = 0.3
                edge_lst.append((ap_name, sta_name))
                pos[sta_name] = (tg.stas[sta_id][0], tg.stas[sta_id][1])
                sta_id = sta_id + 1

        G = nx.DiGraph()
        G.add_edges_from(edge_lst)

        values = [val_map.get(node, 0.25) for node in G.nodes()]
        black_edges = [edge for edge in G.edges()]

        nx.draw_networkx_nodes(G, pos, cmap=plt.get_cmap('summer', 16), node_color=values, node_size=1500)
        nx.draw_networkx_labels(G, pos)
        nx.draw_networkx_edges(G, pos, edgelist=black_edges, arrows=False)
        plt.xlabel('x-axis [m]')
        plt.xlabel('y-axis [m]')
        #plt.grid()
        plt.title('Node location')
        plt.show()

    def plot_thr_vs_distance_bar_chart(self, sta_ap_distances, all_sim_res):

        # convert sim_res into numpy matrix
        data = np.empty((0,3), float)

        for dist_k in sorted(all_sim_res):
            row = []
            for flow_k in all_sim_res[dist_k]:
                thr = all_sim_res[dist_k][flow_k]['throughput']
                row.append(thr)
            data = np.vstack([data, row])

        if False:
            plt.imshow(data)
            plt.colorbar()
            plt.show()

        for i in range(data.shape[1]):
            plt.plot(sta_ap_distances, data[:,i] / 1e6, '.-', label='Flow: ' + str(i))

        plt.xlabel('Distance AP to STA [m]')
        plt.ylabel('Throughput [Mbps]')
        plt.grid()
        plt.title(str(self.channel_cfg))
        plt.legend(loc='upper right')
        #plt.show()

        plt.savefig(self.folder + 'plot_' + str(self.channel_cfg) + '.png')