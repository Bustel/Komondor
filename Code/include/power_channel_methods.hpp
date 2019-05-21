#ifndef POWER_CHANNEL_METHODS_HPP
#define POWER_CHANNEL_METHODS_HPP

#include <map>
#include "notification.hpp"
#include "logger.hpp"

/***********************/
/***********************/
/*  POWER MANAGEMENT   */
/***********************/
/***********************/

/*
 * ConvertPower(): convert power units
 **/
double ConvertPower(int conversion_type, double power_magnitude_in);

/*
 * ComputeDistance(): returns the distance between 2 points
 **/
double ComputeDistance(double x1, double y1, double z1, double x2, double y2, double z2);

/*
 * ComputePowerReceived() returns the power received in a given distance from the transmitter depending on the path loss model
 **/
double ComputePowerReceived(double distance, double tx_power, double tx_gain, double rx_gain,
		double central_frequency, int path_loss_model);

/*
 * ComputeTxPowerPerChannel(): computes power sent per channel
 **/
double ComputeTxPowerPerChannel(double current_tpc, int num_channels_tx);

/***********************/
/***********************/
/* CHANNELS MANAGEMENT */
/***********************/
/***********************/

/*
 * GetChannelOccupancyByCCA(): indicates the channels occupied and free in a binary way
 */
void GetChannelOccupancyByCCA(int primary_channel, int pifs_activated, int *channels_free, int min_channel_allowed,
		int max_channel_allowed, double **channel_power, double cca, double *timestampt_channel_becomes_free,
		double sim_time, double pifs);

/*
 * UpdatePowerSensedPerNode: updates the power sensed comming from each node in the primary channel
 * Sergio on 22/09/2017: power of interest counted only if transmission implies the primary channel
 **/
void UpdatePowerSensedPerNode(int primary_channel, std::map<int,double> &power_received_per_node,
	Notification notification, double rx_gain, double central_frequency, int path_loss_model,
	double pw_received, int start_or_finish);

/*
 * ApplyAdjacentChannelInterferenceModel: applies a cochannel interference model
 **/
void ApplyAdjacentChannelInterferenceModel(int adjacent_channel_model, double total_power[],
	Notification notification, int num_channels_komondor, double rx_gain,
	double central_frequency, double pw_received, int path_loss_model);

/*
 * UpdateChannelsPower: updates the aggregated power sensed by the node in every channel
 **/
void UpdateChannelsPower(double **channel_power, Notification notification,
    int update_type, double central_frequency, int num_channels_komondor, int path_loss_model,
	double rx_gain, int adjacent_channel_model, double pw_received, int node_id);

/*
 * UpdateSINR(): Updates the SINR
 **/
double UpdateSINR(double pw_received_interest, double noise_level, double max_pw_interference);

/*
 * ComputeMaxInterference(): computes the maximum interference perceived in the channels of interest
 **/
void ComputeMaxInterference(double *max_pw_interference, int *channel_max_intereference,
	Notification notification_interest, int node_state, std::map<int,double> &power_received_per_node,
	double **channel_power);

/*
 * GetTxChannelsByChannelBonding: identifies the channels to TX in depending on the channel_bonding scheme
 * and channel_power state.
 **/
void GetTxChannelsByChannelBonding(int *channels_for_tx, int channel_bonding_model, int *channels_free,
    int min_channel_allowed, int max_channel_allowed, int primary_channel, int **mcs_per_node,
	int ix_mcs_per_node, int num_channels_system);

/*
 * UpdateTimestamptChannelFreeAgain: updates the timestamp at which channels became free again
 **/
void UpdateTimestamptChannelFreeAgain(double *timestampt_channel_becomes_free, double **channel_power,
		double current_cca, int num_channels_komondor, double sim_time);

/**********************/
/**********************/
/* PRINTING FUNCTIONS */
/**********************/
/**********************/

/*
 * PrintOrWriteChannelPower: prints (or writes) the channel_power array representing the power sensed by
 * the node in each subchannel.
 */
void PrintOrWriteChannelPower(int write_or_print, int save_node_logs, Logger node_logger,
	int print_node_logs, double **channel_power, int num_channels_komondor);

/*
 * printOrWriteChannelsFree: prints (or writes) the channels_free array representing the channels that are free.
 */
void PrintOrWriteChannelsFree(int write_or_print,
		int save_node_logs, int print_node_logs, Logger node_logger,
		int num_channels_komondor, int *channels_free);

/*
 * printOrWriteNodesTransmitting: prints (or writes) the array representing the transmitting nodes.
 */
void PrintOrWriteNodesTransmitting(int write_or_print,
		int save_node_logs, int print_node_logs, Logger node_logger, int total_nodes_number,
		int *nodes_transmitting);


/*
 * printOrWriteChannelForTx: prints (or writes) the channels_for_tx array representing the channels used for TX
 */
void PrintOrWriteChannelForTx(int write_or_print,
		int save_node_logs, int print_node_logs, Logger node_logger,
		int num_channels_komondor, int *channels_for_tx);
#endif 
