'''
    IEEE 802.11 parameters
'''
class Dot80211Params:

    def __init__(self, num_channels=8, tx_power=30, channel_bonding_mode=3):
        self.num_channels = num_channels
        self.tx_power = tx_power
        self.channel_bonding_mode = channel_bonding_mode