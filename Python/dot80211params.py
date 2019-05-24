'''
    IEEE 802.11 parameters
'''

import itertools

class Dot80211Params:

    def __init__(self, num_channels=8, min_channel_allowed=0, max_channel_allowed=8, tx_power=30, channel_bonding_mode=3):
        self.num_channels = num_channels
        self.min_channel_allowed = min_channel_allowed
        self.max_channel_allowed = max_channel_allowed
        self.tx_power = tx_power
        self.channel_bonding_mode = channel_bonding_mode

    def get_id(self):
        return str(self.num_channels) \
               + '_' + str(self.min_channel_allowed) \
               + '_' + str(self.max_channel_allowed) \
               + '_' + str(self.channel_bonding_mode)

    def to_string(self):
        s = '802.11ax,'
        s = s + 'BW:' + str(self.num_channels * 20) + ' MHz,'
        s = s + 'CH:' + str(self.min_channel_allowed) + ' - ' + str(self.max_channel_allowed) + ' ,'
        s = s + 'TxP:' + str(self.tx_power) + ' dBm,'
        s = s + 'Bmode:' + str(self.channel_bonding_mode)
        return s

    @classmethod
    def param_iter(cls, tx_power=30, num_channels=8, dcb_modes=[1, 3]):
        for chan_width in [8,4,2,1]:
            for chan_base in range(0,num_channels):
                if chan_width + chan_base > num_channels:
                   break

                for cb_mode in dcb_modes:
                    yield cls(num_channels, min_channel_allowed=chan_base,
                              max_channel_allowed=chan_base+chan_width-1,
                              tx_power=tx_power, channel_bonding_mode=cb_mode)

    @classmethod
    def get_all_combinations(cls, num_aps, num_channels=8, dcb_modes=[1, 3],
                             tx_power=30):
        all_params = list(cls.param_iter(tx_power=tx_power,
                                         num_channels=num_channels,
                                         dcb_modes=dcb_modes))
       

        return itertools.product(*(num_aps*[all_params]))


if __name__ == '__main__':

    all_combinations= list(Dot80211Params.get_all_combinations(3,
                           dcb_modes=[1]))
    for p in all_combinations:
        print('')
        print(p[0].to_string())
        print(p[1].to_string())
        print(p[2].to_string())
        print('--------------------------------------')
    
    print("{0} combinations.".format(len(all_combinations)))
