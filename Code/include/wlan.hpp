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

	/*
	 * PrintStaIds(): prints the list of STAs IDs belonging to the WLAN
	 */
	void PrintStaIds();

	/*
	 * WriteStaIds(): writes STAs list of IDs in a given file
	 * Input arguments:
	 * - logger: logger containing the file to write on
	 */
	void WriteStaIds(Logger logger);

	/*
	 * PrintWlanInfo(): prints general WLAN info
	 */
	void PrintWlanInfo();

	/*
	 * WriteWlanInfo(): writes general WLAN info in a given file
	 * Input arguments:
	 * - logger: logger containing the file to write on
	 * - header_string: header string
	 */
	void WriteWlanInfo(Logger logger, std::string header_str);

};



#endif
