#ifndef NODE_CONFIGURATION_HPP
#define NODE_CONFIGURATION_HPP
struct Capabilities
{
	int node_id;				// Node id
	double x;					// X position coordinate
	double y;					// Y position coordinate
	double z;					// Z position coordinate
	int node_type;				// Node type (e.g., AP, STA, ...)
	int destination_id;			// Destination node id (for nodes not belonging to any WLAN)
	double lambda;				// Average notification generation rate (related to exponential BO) [notification/s]
	double traffic_load;		// Average traffic load of the AP [packets/s]
	int ieee_protocol;			// IEEE protocol type
	int primary_channel;		// Primary channel
	int min_channel_allowed;	// Min. allowed channel
	int max_channel_allowed;	// Max. allowed channel
	int num_channels_allowed;	// Maximum number of channels allowed to TX in
	double tpc_min;				// Min. power transmission [pW]
	double tpc_default;			// Default power transmission [pW]
	double tpc_max;				// Max. power transmission [pW]
	double cca_min;				// Min. CCA	("sensitivity" threshold) [pW]
	double cca_default;			// Default CCA	("sensitivity" threshold) [pW]
	double cca_max;				// Max. CCA ("sensitivity" threshold)
	double tx_gain;				// Antenna transmission gain [linear]
	double rx_gain;				// Antenna reception gain [linear]
	int dcb_policy;	// Selected DCB policy
	int modulation_default;		// Default modulation

	// Function to print the node's capabilities
	void PrintCapabilities();
};

// Node's configuration
struct Configuration
{
	double timestamp;

	int selected_primary_channel;		// Selected primary channel
	double selected_cca;		// Selected CCA ("sensitivity" threshold) [pW]
	double selected_tx_power;	// Selected Tx Power [pW]
	int selected_dcb_policy;	// Selected DCB policy

	Capabilities capabilities;

	// Function to print the node's configuration
	void PrintConfiguration(int origin);
};


#endif
