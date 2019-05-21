#ifndef MODULATION_HPP
#define MODULATION_HPP


struct Mcs_array {
   static const double mcs_array[4][12];
   static const double coding_rate_array[12];
   static const int bits_per_symbol_modulation_array[12];
   static const int modulation_bits[12];
   static const double coding_rates[12];
};



// Sergio on 5 Oct 2017: include number of subcarriers in the IEEE 802.11ax
int getNumberSubcarriers(int num_channels);
#endif 
