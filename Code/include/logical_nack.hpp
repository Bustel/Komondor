#ifndef LOGICAL_NACK_HPP
#define LOGICAL_NACK_HPP

struct LogicalNack
{
	int source_id;		// Node sending the NACK
	int packet_type;	// Type of packet
	int packet_id;		// Packet_id
	int loss_reason;	// Loss reason. Why the packet has been lost? (Look for possible reasons in file ListOfDefines.h)
	int node_id_a;		// TODO: update definition - Old comment: "(other uses may be) Hidden node that started the transmission"
	int node_id_b;		// TODO: update definition - Old comment: "Hidden node causing the collision"

	double ber;			// Bit error rate
	double sinr;		// Signal to noise plus interference ratio

	void PrintNackInfo(void);
};

#endif
