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
 * - This file defines a LOGGER to generate logs
 */
#include <logger.hpp>

#include <glib.h>
#include <glib/gprintf.h>

void Logger::init(int l, FILE* f, const char* p){
	this->lvl = l;
	this->file = (f != NULL) ? f : stderr;
	this->prefix =  (p != NULL) ? g_strdup(p) : "";
}

void Logger::log(int l, const char* format_key, ...){
    va_list args;
    va_start(args, format_key);

    this->v_log(l, format_key, args);
}

void Logger::v_log(int l, const char* format_key, va_list args){
    if (this->lvl > l){
        return;
    } 

    gchar* text = g_strdup_vprintf(format_key, args);	
    gchar* expanded = g_strdup_printf("%s: %s",this->prefix, text);
    g_fprintf(this->file,"%s", expanded);
    
    g_free(expanded);
    g_free(text);

}

void Logger::debug(const char* format_key, ...){
    va_list args;
    va_start(args, format_key);
    this->v_log(LOG_LVL_DEBUG, format_key, args);
}


void Logger::info(const char* format_key, ...){
    va_list args;
    va_start(args, format_key);
    this->v_log(LOG_LVL_INFO, format_key, args);
}

void Logger::warn(const char* format_key, ...){
    va_list args;
    va_start(args, format_key);
    this->v_log(LOG_LVL_WARN, format_key, args);
}

void Logger::error(const char* format_key, ...){
    va_list args;
    va_start(args, format_key);
    this->v_log(LOG_LVL_ERROR, format_key, args);
}









