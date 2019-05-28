#ifndef WLAN_HPP
#define WLAN_HPP 


#include "logger.hpp"

// WLAN info
struct Wlan
{
	int wlan_id;			// WLAN ID
	std::string wlan_code;	// Code of the WLAN (string)
	int num_stas;			// Number of STAs in the WLAN (AP not included)
	int ap_id;				// Id of the Access Point
	int *list_sta_id;		// List of STAs IDs belonging to the WLAN

	/*
	 * SetSizeOfSTAsArray(): sets the size of the array list_sta_id
	 */
	void SetSizeOfSTAsArray(int num_stas);

};



#endif
