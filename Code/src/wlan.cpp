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
 * - This file defines a WLAN and provides basic displaying methods
 */


#define    LOGS(f,...)    if (f!=NULL){fprintf(f, ##__VA_ARGS__);}
#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>

#include "macros.h"
#include "logger.hpp"

#include "wlan.hpp"

void Wlan::SetSizeOfSTAsArray(int num_stas){
    list_sta_id = new int[num_stas];
    for(int s = 0; s < num_stas; ++s){
        list_sta_id[s] = NODE_ID_NONE;
    }
}

void Wlan::PrintStaIds(){
    for(int s = 0; s < num_stas; s++){
        LOGS(stderr, "%d  ", list_sta_id[s]);
    }
    LOGS(stderr, "\n");
}

void Wlan::WriteStaIds(Logger logger){
    if (logger.save_logs){
        for(int s = 0; s < num_stas; s++){
            LOGS(logger.file, "%d  ", list_sta_id[s]);
        }
    }
}

void Wlan::PrintWlanInfo(){
    LOGS(stderr, "%s WLAN %s:\n", LOG_LVL3, wlan_code.c_str());
    LOGS(stderr, "%s wlan_id: %d\n", LOG_LVL4, wlan_id);
    LOGS(stderr, "%s num_stas: %d\n", LOG_LVL4, num_stas);
    LOGS(stderr, "%s ap_id: %d\n", LOG_LVL4, ap_id);
    LOGS(stderr, "%s list of STAs IDs: ", LOG_LVL4);
    PrintStaIds();
}

void Wlan::WriteWlanInfo(Logger logger, std::string header_str){
    if (logger.save_logs){
        LOGS(logger.file, "%s WLAN %s:\n", header_str.c_str(), wlan_code.c_str());
        LOGS(logger.file, "%s - wlan_id: %d\n", header_str.c_str(), wlan_id);
        LOGS(logger.file, "%s - num_stas: %d\n", header_str.c_str(), num_stas);
        LOGS(logger.file, "%s - ap_id: %d\n", header_str.c_str(), ap_id);
        LOGS(logger.file, "%s - list of STAs IDs: ", header_str.c_str());
        WriteStaIds(logger);
        LOGS(logger.file, "\n");
    }
}
