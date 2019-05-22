#ifndef _AUX_PERFORMANCE_
#define _AUX_PERFORMANCE_

struct Performance
{

	double last_time_measured;

	double throughput;
	double max_bound_throughput;
	int data_packets_sent;
	int data_packets_lost;
	int rts_cts_packets_sent;
	int rts_cts_packets_lost;
	double num_packets_generated;
	double num_packets_dropped;

	double *rssi_list;

	// Function to print the node's report
	void PrintReport(void);

	/*
	 * SetSizeOfRssiList(): sets the size of the array list_id_aggregated
	 */
	void SetSizeOfRssiList(int total_wlans_number);

};

#endif
