#ifndef NOTIFICATION_METHODS_HPP
#define NOTIFICATION_METHODS_HPP


#include <notification.hpp>
#include <logical_nack.hpp>
#include <logger.hpp>

/*
 * GenerateLogicalNack: generates a logical NACK
 **/
LogicalNack GenerateLogicalNack(int packet_type, int packet_id, int node_id,
		int node_id_a, int node_id_b, int loss_reason, double ber, double sinr);

/*
 * ProcessNack(): processes a NACK notification.
 **/
int ProcessNack(LogicalNack logical_nack, int node_id, Logger node_logger, int node_state,
		int save_node_logs,	double sim_time, int *nacks_received,
		int total_nodes_number, int *nodes_transmitting);

/*
 * CleanNack(LogicalNack nack): re-initializes the Nack info.
 */
void CleanNack(LogicalNack *nack);

/*
 * handlePacketLoss(): handles a packet loss.
 */
void handlePacketLoss(int type, double *total_time_lost_in_num_channels, double *total_time_lost_per_channel,
		int &packets_lost, int &rts_cts_lost, int current_right_channel, int current_left_channel,
		double current_tx_duration);

/*
 * AttemptToDecodePacket(): attempts to decode incoming packet according to SINR and the capture effect (CE)
 **/
int AttemptToDecodePacket(double sinr, double capture_effect, double cca,
		double power_rx_interest, double constant_per, int node_id, int packet_type,
		int destination_id);

/*
 * IsPacketLost(): computes notification loss according to SINR received
 **/
int IsPacketLost(int primary_channel, Notification incoming_notification, Notification new_notification,
		double sinr, double capture_effect, double cca, double power_rx_interest, double constant_per,
		int node_id, int capture_effect_model);

/*
 * GenerateTxInfo: generates a TxInfo
 **/
TxInfo GenerateTxInfo(int num_packets_aggregated, double data_duration,	double ack_duration,
		double rts_duration, double cts_duration, double current_tpc, int num_channels_tx,
		double tx_gain,	int bits_ofdm_sym, double x, double y, double z);
#endif 
