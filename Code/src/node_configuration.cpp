/* Komondor IEEE 802.11ax Simulator
 *
 * Copyright (c) 2017, Universitat Pompeu Fabra.
 * GNU GENERAL PUBLIC LICENSE
 * Version 3, 29 June 2007

 * Copyright (C) 2007 Free Software Foundation, Inc. <http://fsf.org/>
 * Everyone is permitted to copy and distribute verbatim copies
 * of this license document, but changing it is not allowed.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *
 * -----------------------------------------------------------------
 *
 * Author  : Sergio Barrachina-Muñoz and Francesc Wilhelmi
 * Created : 2016-12-05
 * Updated : $Date: 2017/03/20 10:32:36 $
 *           $Revision: 1.0 $
 *
 * -----------------------------------------------------------------
 * File description: this is the main Komondor file
 *
 * - This file defines a NOTIFICATION and provides basic displaying methods
 */
#include "node_configuration.hpp"
#include "power_channel_methods.hpp"
#include "macros.h"
#include <cstdio>


void Capabilities::PrintCapabilities(){

	printf("%s Capabilities of node %d:\n", LOG_LVL3, node_id);
	printf("%s node_type = %d\n", LOG_LVL4, node_type);
	printf("%s position = (%.2f, %.2f, %.2f)\n", LOG_LVL4, x, y, z);
	printf("%s primary_channel = %d\n", LOG_LVL4, primary_channel);
	printf("%s min_channel_allowed = %d\n", LOG_LVL4, min_channel_allowed);
	printf("%s max_channel_allowed = %d\n", LOG_LVL4, max_channel_allowed);
	printf("%s dcb_policy = %d\n", LOG_LVL4, dcb_policy);
	printf("%s lambda = %f packets/s\n", LOG_LVL4, lambda);
	printf("%s traffic_load = %.2f packets/s\n", LOG_LVL4, traffic_load);
	printf("%s destination_id = %d\n", LOG_LVL4, destination_id);
	printf("%s tpc_min = %f pW (%f dBm)\n", LOG_LVL4, tpc_min, ConvertPower(PW_TO_DBM, tpc_min));
	printf("%s tpc_default = %f pW (%f dBm)\n", LOG_LVL4, tpc_default, ConvertPower(PW_TO_DBM, tpc_default));
	printf("%s tpc_max = %f pW (%f dBm)\n", LOG_LVL4, tpc_max, ConvertPower(PW_TO_DBM, tpc_max));
	printf("%s cca_min = %f pW (%f dBm)\n", LOG_LVL4, cca_min, ConvertPower(PW_TO_DBM, cca_min));
	printf("%s cca_default = %f pW (%f dBm)\n", LOG_LVL4, cca_default, ConvertPower(PW_TO_DBM, cca_default));
	printf("%s cca_max = %f pW (%f dBm)\n", LOG_LVL4, cca_max, ConvertPower(PW_TO_DBM, cca_max));
	printf("%s tx_gain = %f (%f dBi)\n", LOG_LVL4, tx_gain, ConvertPower(LINEAR_TO_DB, tx_gain));
	printf("%s rx_gain = %f (%f dBi)\n", LOG_LVL4, rx_gain, ConvertPower(LINEAR_TO_DB, rx_gain));
	printf("%s modulation_default = %d\n", LOG_LVL4, modulation_default);

	printf("\n");

}

void Configuration::PrintConfiguration(int origin){

	if (origin == ORIGIN_AGENT) {
		printf("%s Recommended configuration by the agent:\n", LOG_LVL3);
	} else if (origin == ORIGIN_AP) {
		printf("%s Current configuration of the WLAN:\n", LOG_LVL3);
	} else {
		printf("ERROR: bad origin\n");
	}

	printf("%s selected_primary = %d\n", LOG_LVL4, selected_primary_channel);
	printf("%s cca_default = %f pW (%f dBm)\n", LOG_LVL4, selected_cca, ConvertPower(PW_TO_DBM, selected_cca));
	printf("%s tpc_default = %f pW (%f dBm)\n", LOG_LVL4, selected_tx_power, ConvertPower(PW_TO_DBM, selected_tx_power));
	printf("%s selected_dcb_policy = %d\n", LOG_LVL4, selected_dcb_policy);
	printf("\n");

}

