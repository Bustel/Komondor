'''
    IEEE 802.11 parameters
'''
class Dot80211Params:

    def __init__(self, num_channels=8, tx_power=30, channel_bonding_mode=3):
        self.num_channels = num_channels
        self.tx_power = tx_power
        self.channel_bonding_mode = channel_bonding_mode

    def to_string(self):
        s = '802.11ax,'
        s = s + 'BW:' + str(self.num_channels * 20) + ' MHz,'
        s = s + 'TxP:' + str(self.tx_power) + ' dBm,'
        s = s + 'Bmode:' + str(self.channel_bonding_mode)
        return s