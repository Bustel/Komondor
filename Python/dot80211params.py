'''
    IEEE 802.11 parameters
'''
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