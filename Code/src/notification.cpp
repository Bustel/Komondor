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

#include <cstdio>

#include "notification.hpp"
#include "macros.h"



void TxInfo::PrintTxInfo(int packet_id, int destination_id, double tx_duration){
	printf("packet_id = %d - destination_id = %d - tx_duration = %f - tx_power = %f pw"
		" - position = (%.2f, %.2f, %.2f)\n",
		packet_id, destination_id, tx_duration, tx_power, x, y, z);
}



/*
 * SetSizeOfIdsArray(): sets the size of the array list_id_aggregated
 */
void TxInfo::SetSizeOfIdsAggregatedArray(int num_packets_aggregated){
	list_id_aggregated = new int[num_packets_aggregated];
	for(int t = 0; t < num_packets_aggregated; ++t){
		list_id_aggregated[t] = 0;
	}
}


/*
 * SetSizeOfTimestampAggregatedArray(): sets the size of the array timestamp_frames_aggregated
 */
void TxInfo::SetSizeOfTimestampAggregatedArray(int num_packets_aggregated){
	timestamp_frames_aggregated = new double[num_packets_aggregated];
	for(int t = 0; t < num_packets_aggregated; ++t){
		timestamp_frames_aggregated[t] = 0;
	}
}


/*
 * SetSizeOfMCS(): sets the size of the array modulation_schemes
 */
void TxInfo::SetSizeOfMCS(int channels_groups){
	//modulation_schemes = new int[channels_groups];
	for(int s = 0; s < channels_groups; ++s){
		modulation_schemes[s] = MODULATION_NONE;
	}
}


void Notification::PrintNotification(void){
	printf("source_id = %d - packet_type = %d - left_channel = %d - right_channel = %d - pkt_length = %d -",
		source_id, packet_type, left_channel, right_channel, frame_length);
	printf("tx_info: ");
	tx_info.PrintTxInfo(packet_id, destination_id, tx_duration);
}

