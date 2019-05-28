#ifndef NOTIFICATION_HPP
#define NOTIFICATION_HPP
// Notification specific info (may be not checked by the other nodes)
#include <node_env.h>

struct TxInfo
{

	int num_packets_aggregated;				// Number of frames aggregated
	int *list_id_aggregated;				// List of frame IDs aggregated
	double *timestamp_frames_aggregated;	// List of timestamps of the frames aggregated

	// For RTS/CTS management
	double data_duration;
	double ack_duration;
	double rts_duration;
	double cts_duration;

	double preoccupancy_duration;
	double tx_power;				// Transmission power in [pW]
	double tx_gain;					// Transmission gain [linear ratio]
	double bits_ofdm_sym; 			// Bits per OFDM symbol
	double data_rate; 				// Rate at which data is transmitted
	int modulation_schemes[4];		// Modulation scheme used
	double x;						// X position of source node
	double y;						// Y position of source node
	double z;						// Z position of source node
	double nav_time;				// RTS/CTS NAV time

	void PrintTxInfo(int packet_id, int destination_id, double tx_duration);
	/*
	 * SetSizeOfIdsArray(): sets the size of the array list_id_aggregated
	 */
	void SetSizeOfIdsAggregatedArray(int num_packets_aggregated);

	/*
	 * SetSizeOfTimestampAggregatedArray(): sets the size of the array timestamp_frames_aggregated
	 */
	void SetSizeOfTimestampAggregatedArray(int num_packets_aggregated);
	/*
	 * SetSizeOfMCS(): sets the size of the array modulation_schemes
	 */
	void SetSizeOfMCS(int channels_groups);
};

// Notification info
struct Notification
{
	// Always read in destination
	int packet_id;				// Packet identifier of the first frame
	int packet_type;			// Type of packet: Data, ACK, etc.
	int source_id;				// Node id of the source
	int destination_id;			// Destination node of the transmission
	double tx_duration;			// Duration of the transmission
	int left_channel;			// Left channel used in the transmission
	int right_channel;			// Right channel used in the transmission
	int frame_length;			// Size of the packet to transmit
	int modulation_id;			// Modulation being used during the transmission
	double timestamp;			// Timestamp when notification is sent
	double timestamp_generated;	// Timestamp when notification was generated

	// Specific transmission info (may not be checked by the others nodes)
	TxInfo tx_info;

    node_env_type src_node_env; 

	void PrintNotification(void);
};
#endif
