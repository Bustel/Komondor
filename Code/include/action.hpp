#ifndef ACTION_HPP
#define ACTION_HPP

// Action info
struct Action
{
	int channel;	// Channel selected
	double cca;		// CCA level
	double tx_power;	// Tx Power
	int dcb_policy;

	/*
	 * PrintStaIds(): prints the list of STAs IDs belonging to the WLAN
	 */
	void PrintAction();
};


#endif 
