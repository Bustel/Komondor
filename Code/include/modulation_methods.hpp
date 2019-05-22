#ifndef MODULATION_METHODS_HPP
#define MODULATION_METHODS_HPP
/*
 * SelectMCSResponse(): select the proper MCS of transmitter per number of channels
 **/
void SelectMCSResponse(int *mcs_response, double power_rx_interest);

/*
 * ComputeEbToNoise():
 * "Energy per bit to noise power spectral density ratio"
 * (normalized SNR to compare the BER performance of a digital modulation scheme)
 * Arguments:
 * - SINR: SINR received (pW)
 * - bit_rate: bit rate (bps)
 * - B: channel bandwidth (Hz)
 * - M: number of alternative modulation symbols
 * Output:
 * - Eb_to_N0: Eb_to_N0 value in linear
 */
double ComputeEbToNoise(double sinr, double bit_rate, int bandwidth, int modulation_type);
#endif
