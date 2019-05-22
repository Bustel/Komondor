#ifndef TIME_METHODS_HPP
#define TIME_METHODS_HPP

// Exponential redefinition
double	Random(double v=1.0);
int	Random(int v);
double	Exponential(double mean);

/*
 * findMaximumPacketsAggregated: computes the minimum number of packets to be transmitted
 * within the maximum PPDU time (IEEE_AX_MAX_PPDU_DURATION)
 **/
int findMaximumPacketsAggregated(int num_packets_aggregated, int data_packet_length, double bits_ofdm_sym);

/*
 * ComputeTxTime(): computes the transmission time (just link rate) according to the number of channels used and packet lenght
 **/
double ComputeTxTime(int total_bits, double data_rate, int pdf_tx_time);

/*
 * computeRtsTxTimeIeee80211ax: computes RTS transmission time
 **/
double computeRtsTxTime80211ax(double bits_ofdm_sym_legacy);

/*
 * computeCtsTxTimeIeee80211ax: computes CTS transmission time
 **/
double computeCtsTxTime80211ax(double bits_ofdm_sym_legacy);

/*
 * computeDataTxTimeIeee80211ax: computes data transmission time
 **/
//double computeDataTxTime80211ax(int num_packets_aggregated, int data_packet_length, double bits_ofdm_sym){

double computeDataTxTime80211ax(int num_packets_aggregated, int data_packet_length, double bits_ofdm_sym);

/*
 * computeAckTxTimeIeee80211ax: computes ACK transmission time
 **/
double computeAckTxTime80211ax(int num_packets_aggregated, double bits_ofdm_sym_legacy);

/*
 * ComputeNavTime: computes the NAV time for the RTS and CTS packets.
 **/
double ComputeNavTime(int node_state, double rts_duration, double cts_duration, double data_duration,
		double ack_duration, double sifs);

/*
 * ComputeNavTime: computes the NAV time for the RTS and CTS packets.
 **/
void ComputeFramesDuration(double *rts_duration, double *cts_duration,
		double *data_duration, double *ack_duration, int ieee_protocol,
		int num_channels_tx, int current_modulation, int num_packets_aggregated,
		int data_packet_length, int bits_ofdm_sym);
#endif 
