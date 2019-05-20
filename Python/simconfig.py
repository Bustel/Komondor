import configparser

'''
    Create config file which is used by the komondor simulator
'''
class SimConfig:

    def __init__(self, out_fname):
        self.config = configparser.ConfigParser()
        self.config.optionxform = str
        self.out_fname = out_fname

    def create(self, d11ps, tg):
        # create global system conf
        self.create_system(d11ps[0])

        # create BSSs
        # create APs
        for ap_i in range(0, tg.num_aps):
            ap_name = chr(65 + ap_i)
            self.create_ap(d11ps[ap_i], ap_name, tg.aps[ap_i][0], tg.aps[ap_i][1], tg.aps[ap_i][2])
        # create STAs + association
        sta_id = 0
        for ap_i in range(0, tg.num_aps):
            ap_name = chr(65 + ap_i)
            for sta_i in range(0, tg.num_stas_per_ap[ap_i]):
                self.create_sta(d11ps[ap_i], ap_name, str(sta_i+1), tg.stas[sta_id][0], tg.stas[sta_id][1], tg.stas[sta_id][2])
                sta_id = sta_id + 1

        # write to file
        with open(self.out_fname, 'w') as configfile:
            self.config.write(configfile)

        print('Exported cfg file to: %s' % self.out_fname)

    def create_system(self, d11p):
        self.config['System'] = {
            'num_channels': d11p.num_channels,
            'basic_channel_bandwidth': 20,
            'pdf_backoff': 0,
            'pdf_tx_time': 1,
            'packet_length': 12000,
            'num_packets_aggregated': 64,
            'path_loss_model_default': 5,
            'path_loss_model_indoor_indoor': 5,
            'path_loss_model_indoor_outdoor': 8,
            'path_loss_model_outdoor_outdoor': 7,
            'capture_effect': 20,
            'noise_level': -95,
            'adjacent_channel_model': 0,
            'collisions_model': -0,
            'constant_PER': 0,
            'traffic_model': 99,
            'backoff_type': 1,
            'cw_adaptation': 'false',
            'pifs_activated': 'false',
            'capture_effect_model': 1
        }

    def create_ap(self, d11p, ap_name, pos_x, pos_y, pos_z, tx_power=30):
        self.config['Node_AP_' + ap_name] = {
            'type': 0,
            'wlan_code': ap_name,
            'destination_id': -1,
            'x': pos_x,
            'y': pos_y,
            'z': pos_z,
            'primary_channel': d11p.min_channel_allowed,
            'min_channel_allowed': d11p.min_channel_allowed,
            'max_channel_allowed': d11p.max_channel_allowed,
            'cw': 16,
            'cw_stage': 5,
            'tpc_min': tx_power,
            'tpc_default': tx_power,
            'tpc_max': tx_power,
            'cca_min': -82,
            'cca_default': -82,
            'cca_max': -82,
            'tx_antenna_gain': 0,
            'rx_antenna_gain': 0,
            'channel_bonding_model': d11p.channel_bonding_mode,
            'modulation_default': 0,
            'central_freq': 5,
            'lambda': 10000,
            'ieee_protocol': 1,
            'traffic_load': 1000,
            'node_env': 'outdoor'
        }

    def create_sta(self, d11p, ap_name, sta_name, pos_x, pos_y, pos_z, tx_power=30):
        self.config['Node_STA_' + ap_name + '' + sta_name] = {
            'type': 1,
            'wlan_code': ap_name,
            'destination_id': -1,
            'x': pos_x,
            'y': pos_y,
            'z': pos_z,
            'primary_channel': d11p.min_channel_allowed,
            'min_channel_allowed': d11p.min_channel_allowed,
            'max_channel_allowed': d11p.max_channel_allowed,
            'cw': 16,
            'cw_stage': 5,
            'tpc_min': tx_power,
            'tpc_default': tx_power,
            'tpc_max': tx_power,
            'cca_min': -82,
            'cca_default': -82,
            'cca_max': -82,
            'tx_antenna_gain': 0,
            'rx_antenna_gain': 0,
            'channel_bonding_model': d11p.channel_bonding_mode,
            'modulation_default': 0,
            'central_freq': 5,
            'lambda': 10000,
            'ieee_protocol': 1,
            'traffic_load': 0,
            'node_env': 'indoor'
        }
