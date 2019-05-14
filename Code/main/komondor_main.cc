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
 * - This file generates the wireless network according to the input files.
 * Then, it initiates nodes to start sending packets until the simulation
 * time is over. Finally, it processes the results.
 */

#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <vector>
#include <map>
#include <unistd.h>
#include <errno.h>

#include ".././COST/cost.h"
#include "../list_of_macros.h"
#include "../structures/logical_nack.h"
#include "../structures/notification.h"
#include "../structures/wlan.h"
#include "node.h"
#include "traffic_generator.h"
#include "agent.h"
#include "central_controller.h"

#include <glib.h>
#include <glib/gprintf.h>

int total_nodes_number;         // Total number of nodes
char* tmp_nodes;

/* Sequential simulation engine from where the system to be simulated is derived. */
component Komondor : public CostSimEng {

    // Methods
    public:

        void Setup(double simulation_time_komondor, int save_system_logs, int save_node_logs, int save_agent_logs,
            int print_node_logs, int print_system_logs, int print_agent_logs, const char *config_filename, const char *script_filename, const char *simulation_code, int seed_console,
            int agents_enabled, const char *agents_filename);
        void Stop();
        void Start();
        void InputChecker();

        void SetupEnvironmentByReadingInputFile(const char *system_filename);
        void GenerateNodes(const char *nodes_filename);
        void GenerateNodesByReadingNodesInputFile(const char *nodes_filename);

        void ParseNodes(const char* nodes_filename);

        void GenerateAgents(const char *agents_filename);
        void GenerateCentralController(const char *agents_filename);

        int GetNumOfLines(const char *nodes_filename);
        int GetNumOfNodes(const char *nodes_filename, int node_type, std::string wlan_code);

        void printSystemInfo();
        void PrintAllWlansInfo();
        void PrintAllAgentsInfo();
        void PrintCentralControllerInfo();
        void PrintAllNodesInfo(int info_detail_level);
        void WriteSystemInfo(Logger logger);
        void WriteAllWlansInfo(Logger logger, std::string header_str);
        void WriteAllNodesInfo(Logger logger, int info_detail_level,  std::string header_str);

    // Public items (to shared with the nodes)
    public:

        Node[] node_container;          // Container of nodes (i.e., APs, STAs, ...)
        Wlan *wlan_container;           // Container of WLANs
        TrafficGenerator[] traffic_generator_container; // Container of traffic generators (associated to nodes)

        int total_wlans_number;             // Total number of WLANs
        int total_agents_number;            // Total number of agents
        int total_controlled_agents_number; // Total number of agents attached to the central controller

        // Parameters entered per console
        int save_node_logs;                 // Flag for activating the log writting of nodes
        int print_node_logs;                // Flag for activating the printing of node logs
        int save_agent_logs;                // Flag for activating the log writting of agents
        int print_agent_logs;               // Flag for activating the printing of agent logs
        double simulation_time_komondor;    // Simulation time [s]

        // Parameters entered via system file
        int num_channels_komondor;      // Number of subchannels composing the whole channel
        double basic_channel_bandwidth; // Basic channel bandwidth [Mbps]
        int pdf_backoff;                // Probability distribution type of the backoff (0: exponential, 1: deterministic)
        int pdf_tx_time;                // Probability distribution type of the transmission time (0: exponential, 1: deterministic)
        int frame_length;               // Packet length [bits]
        int ack_length;                 // ACK length [bits]
        int rts_length;                 // RTS length [bits]
        int cts_length;                 // CTS length [bits]
        int max_num_packets_aggregated; // Number of packets aggregated in one transmission
        int path_loss_model;            // Path loss model (0: free-space, 1: Okumura-Hata model - Uban areas)
        double capture_effect;          // Capture effect threshold [linear ratio]
        double noise_level;             // Environment noise [pW]
        int adjacent_channel_model;     // Co-channel interference model
        int collisions_model;           // Collisions model
        double constant_per;            // Constant PER for successful transmissions
        int traffic_model;              // Traffic model (0: full buffer, 1: poisson, 2: deterministic)
        int backoff_type;               // Type of Backoff (0: Slotted 1: Continuous)
        int cw_adaptation;              // CW adaptation (0: constant, 1: bineary exponential backoff)
        int pifs_activated;             // PIFS mechanism activation
        int capture_effect_model;       // Capture Effect model (default or IEEE 802.11-based)

        int agents_enabled;             // Determined according to the input (for generating agents or not)

        // Public items (to shared with the agents)
        public:
        // Agents info
        Agent[] agent_container;
        int num_actions_channel;
        int num_actions_cca;
        int num_actions_tx_power;
        int num_actions_dcb_policy;

        double *actions_cca;
        double *actions_tx_power;

        // Central controller info
        CentralController[] central_controller;

    // Private items
    private:

        int seed;                       // Simulation seed number
        int save_system_logs;           // Flag for activating the log writting of the Komondor system
        int print_system_logs;          // Flag for activating the printing of system logs
        std::string simulation_code;            // Komondor simulation code
        const char *nodes_input_filename;       // Filename of the nodes (AP or Deterministic Nodes) input CSV
        const char *agents_input_filename;
        FILE *simulation_output_file;   // File for the output logs (including statistics)
        FILE *script_output_file;       // File for the whole input files included in the script TODO
        FILE *script_output_file_csv;   // File for the CSV script output
        Logger logger_simulation;       // Logger for the simulation output file
        Logger logger_script;           // Logger for the script file (containing 1+ simulations) Readable version
        Logger logger_script_csv;       // Logger for the script file in CSV format

        // Auxiliar variables
        int first_line_skiped_flag;     // Flag for skipping first informative line of input file
        int central_controller_flag;    // In order to allow the generation of the central controller

};

/*
 * Setup()
 * Input arguments:
 * - sim_time_console: simulation observation time [s]
 * - save_system_logs_console: flag for activating system logs
 * - save_node_logs_console: flag for activating nodes logs
 * - print_system_logs_console: flag for activating system prints
 * - print_node_logs_console: flag for activating nodes prints
 * - config_filename: filename of the system input and node configuration file
 * - script_output_filename: filename of the output file generated by the script of multiple simulations
 * - simulation_code_console: simulation code assigned to current simulation (it is an string)
 */
void Komondor :: Setup(double sim_time_console, int save_system_logs_console, int save_node_logs_console,
        int save_agent_logs_console, int print_system_logs_console, int print_node_logs_console,
        int print_agent_logs_console, const char *config_filename,
        const char *script_output_filename, const char *simulation_code_console, int seed_console,
        int agents_enabled_console, const char *agents_input_filename_console){

    simulation_time_komondor = sim_time_console;
    save_node_logs = save_node_logs_console;
    save_system_logs = save_system_logs_console;
    save_agent_logs = save_agent_logs_console;
    print_node_logs = print_node_logs_console;
    print_system_logs = print_system_logs_console;
    print_agent_logs = print_agent_logs_console;
    nodes_input_filename = config_filename;
    agents_input_filename = agents_input_filename_console;

    std::string simulation_code;
    simulation_code.append(ToString(simulation_code_console));

    seed = seed_console;
    agents_enabled = agents_enabled_console;
    total_wlans_number = 0;

    // Generate output files

    if (print_system_logs) printf("%s Creating output files\n", LOG_LVL1);

    std::string simulation_filename_remove;
    simulation_filename_remove.append("output/simulation_output_").append(simulation_code);

    std::string simulation_filename_fopen;
    simulation_filename_fopen.append("../").append(simulation_filename_remove);

    if(remove(simulation_filename_remove.c_str()) == 0){
        if (print_system_logs) printf("%s Simulation output file '%s' found and removed. New one created!\n",
                LOG_LVL2, simulation_filename_remove.c_str());
    } else {
        if (print_system_logs) printf("%s Simulation output file '%s' created!\n",
                            LOG_LVL2, simulation_filename_remove.c_str());
    }

    // Get loggers to write in output files
    simulation_output_file = fopen(simulation_filename_fopen.c_str(),"at");
    if (simulation_output_file == NULL){
        printf("Failed to open file: %s\n", simulation_filename_fopen.c_str());
        exit(1);
    }
    logger_simulation.save_logs = SAVE_LOG;
    logger_simulation.file = simulation_output_file;

    // Script output (Readable)
    script_output_file = fopen(script_output_filename,"at");    // Script output is removed when script is executed

    if (script_output_file == NULL){
        printf("Failed to open file: %s\n", script_output_filename);
        exit(1);
    }

    logger_script.save_logs = SAVE_LOG;
    logger_script.file = script_output_file;

    std::string script_output_csv_filename;
    gchar *script_output_filename_root = g_strdup(script_output_filename);
    script_output_filename_root[strlen(script_output_filename_root)-4] = 0;
    std::string script_output_filename_root_str(script_output_filename_root);
    script_output_csv_filename.append(script_output_filename_root_str).append("_csv.csv");

    g_free(script_output_filename_root);
    printf("%s\n", script_output_csv_filename.c_str());

    // Script output (CSV format)
    script_output_file_csv = fopen(script_output_csv_filename.c_str(),"at");    // Script output is removed when script is executed


    if (script_output_file_csv == NULL){
        printf("Failed to open file: %s\n", script_output_csv_filename.c_str());
        exit(1);
    }

    logger_script_csv.save_logs = SAVE_LOG;
    logger_script_csv.file = script_output_file_csv;

    // fprintf(logger_script.file, "------------------------------------\n");
    fprintf(logger_script.file, "%s KOMONDOR SIMULATION '%s' (seed %d)", LOG_LVL1, simulation_code.c_str(), seed);
    // Read system (environment) file

    SetupEnvironmentByReadingInputFile(config_filename);

    // Generate nodes
    GenerateNodes(nodes_input_filename);

    // Compute distance of each pair of nodes
    for(int i = 0; i < total_nodes_number; ++i) {
        node_container[i].distances_array = new double[total_nodes_number];
        node_container[i].received_power_array = new double[total_nodes_number];
        for(int j = 0; j < total_nodes_number; ++j) {
            // Compute and assign distances for each other node
            node_container[i].distances_array[j] = ComputeDistance(node_container[i].x,node_container[i].y,
                node_container[i].z,node_container[j].x,node_container[j].y,node_container[j].z);
            // Compute and assign the received power from each other node
            if(i == j) {
                node_container[i].received_power_array[j] = 0;
            } else {
                node_container[i].received_power_array[j] = ComputePowerReceived(node_container[i].distances_array[j],
                    node_container[i].tpc_default, node_container[i].tx_gain, node_container[i].rx_gain,
                    node_container[i].central_frequency, path_loss_model);
            }
        }
    }

    // Compute the maximum power received from each WLAN
    for(int i = 0; i < total_nodes_number; ++i) {
        double max_power_received_per_wlan;
        if (node_container[i].node_type == NODE_TYPE_AP) {
            node_container[i].max_received_power_in_ap_per_wlan = new double[total_wlans_number];
            for(int j = 0; j < total_wlans_number; ++j) {
                if (strcmp(node_container[i].wlan_code.c_str(),wlan_container[j].wlan_code.c_str()) == 0) {
                    // Same WLAN
                    node_container[i].max_received_power_in_ap_per_wlan[j] = 0;
                } else {
                    // Different WLAN
                    max_power_received_per_wlan = -1000;
                    for (int k = 0; k < total_nodes_number; ++k) {
                        // Check only nodes in WLAN "j"
                        if(strcmp(node_container[k].wlan_code.c_str(),wlan_container[j].wlan_code.c_str()) == 0) {
                            if (node_container[i].received_power_array[k] > max_power_received_per_wlan) {
                                max_power_received_per_wlan = node_container[i].received_power_array[k];
                            }
                        }
                    }
                    node_container[i].max_received_power_in_ap_per_wlan[j] = max_power_received_per_wlan;
                }
            }
        }
    }

    // Generate agents
    central_controller_flag = 0;
    if (agents_enabled) { GenerateAgents(agents_input_filename); }

    if (agents_enabled && central_controller_flag) { GenerateCentralController(agents_input_filename); }

    if (print_system_logs) {
        printf("%s System configuration: \n", LOG_LVL2);
        printSystemInfo();
        printf("%s Wlans generated!\n", LOG_LVL2);
        PrintAllWlansInfo();
        if (print_system_logs) printf("\n");
        printf("%s Nodes generated!\n", LOG_LVL2);
        PrintAllNodesInfo(INFO_DETAIL_LEVEL_2);
        if (print_system_logs) printf("\n");
        if (agents_enabled) {
            printf("%s Agents generated!\n\n", LOG_LVL2);
            PrintAllAgentsInfo();
            if (central_controller_flag) {
                printf("%s Central Controller generated!\n\n", LOG_LVL2);
                central_controller[0].PrintCentralControllerInfo();
            }
            printf("\n");
        }
    }

    InputChecker();

    fprintf(logger_simulation.file, "------------------------------------\n");

    if (save_system_logs){

        fprintf(logger_simulation.file, "%s System configuration: \n", LOG_LVL2);
        WriteSystemInfo(logger_simulation);
        fprintf(logger_script.file, "%s System configuration: \n", LOG_LVL2);
        WriteSystemInfo(logger_script);

        std::string header_str;
        header_str.append(ToString(LOG_LVL3));

        fprintf(logger_simulation.file, "%s Wlans generated!\n", LOG_LVL2);
        WriteAllWlansInfo(logger_simulation, header_str);
        fprintf(logger_script.file, "%s Wlans generated!\n", LOG_LVL2);
        WriteAllWlansInfo(logger_script, header_str);

        fprintf(logger_simulation.file, "%s Nodes generated!\n", LOG_LVL2);
        WriteAllNodesInfo(logger_simulation, INFO_DETAIL_LEVEL_0, header_str);
        fprintf(logger_script.file, "%s Nodes generated!\n", LOG_LVL2);
        WriteAllNodesInfo(logger_script, INFO_DETAIL_LEVEL_0, header_str);

    }

    // Set connections among nodes
    for(int n = 0; n < total_nodes_number; ++n){

        connect traffic_generator_container[n].outportNewPacketGenerated,node_container[n].InportNewPacketGenerated;

        for(int m=0; m < total_nodes_number; ++m) {

            connect node_container[n].outportSelfStartTX,node_container[m].InportSomeNodeStartTX;
            connect node_container[n].outportSelfFinishTX,node_container[m].InportSomeNodeFinishTX;
            connect node_container[n].outportSendLogicalNack,node_container[m].InportNackReceived;

            if(strcmp(node_container[n].wlan_code.c_str(),node_container[m].wlan_code.c_str()) == 0 && n!=m) {
                // Connections regarding MCS
                connect node_container[n].outportAskForTxModulation,node_container[m].InportMCSRequestReceived;
                connect node_container[n].outportAnswerTxModulation,node_container[m].InportMCSResponseReceived;
                // Connections regarding changes in the WLAN
                connect node_container[n].outportSetNewWlanConfiguration,node_container[m].InportNewWlanConfigurationReceived;
            }
        }

        if (agents_enabled) {
            // Set connections among APs and Agents
            if ( node_container[n].node_type == NODE_TYPE_AP ) {
                for(int w = 0; w < total_agents_number; ++w){
                    // Connect the agent to the corresponding AP, according to "wlan_code"
                    if (strcmp(node_container[n].wlan_code.c_str(), agent_container[w].wlan_code.c_str()) == 0) {
                        connect agent_container[w].outportRequestInformationToAp,node_container[n].InportReceivingRequestFromAgent;
                        connect node_container[n].outportAnswerToAgent,agent_container[w].InportReceivingInformationFromAp;
                        connect agent_container[w].outportSendConfigurationToAp,node_container[n].InportReceiveConfigurationFromAgent;
                    }
                }
            }
        }
    }

    // Connect the agents to the central controller, if applicable
    if (agents_enabled) {
        for(int w = 0; w < total_agents_number; ++w){
            if (agent_container[w].centralized_flag) {
                connect central_controller[0].outportRequestInformationToAgent,agent_container[w].InportReceivingRequestFromController;
                connect agent_container[w].outportAnswerToController,central_controller[0].InportReceivingInformationFromAgent;
                connect central_controller[0].outportSendConfigurationToAgent,agent_container[w].InportReceiveConfigurationFromController;
            }
        }
    }

};

/*
 * Start()
 */
void Komondor :: Start(){
    // intialize variables
    // total_nodes_number = 0;
};

/*
 * Stop(): called when the simulation is done to  collect and display statistics.
 */
void Komondor :: Stop(){

    printf("%s KOMONDOR SIMULATION '%s' (seed %d)", LOG_LVL1, simulation_code.c_str(), seed);

    int total_data_packets_sent (0);
    double total_num_packets_generated (0);
    double total_throughput (0);
    double min_throughput (999999999999999999);
    double max_throughput (0);
    double proportional_fairness(0);
    double jains_fairness (0);
    double jains_fairness_aux (0);
    int total_rts_lost_slotted_bo (0);
    int total_rts_cts_sent (0);
    double total_prob_slotted_bo_collision (0);
    int total_num_tx_init_not_possible (0);
    double total_delay (0);
    double max_delay (0);
    double min_delay (9999999999);  // Index of the WLAN experiencing less throughput
    int ix_wlan_min_throughput (99999); // Index of the WLAN experiencing less throughput
    double total_bandiwdth_tx (0);
    double av_expected_backoff (0);
    double av_expected_waiting_time (0);

    for(int m=0; m < total_nodes_number; ++m){

        if( node_container[m].node_type == NODE_TYPE_AP ){
            total_data_packets_sent = total_data_packets_sent + node_container[m].data_packets_sent;
            total_throughput = total_throughput + node_container[m].throughput;
            total_num_packets_generated = total_num_packets_generated + node_container[m].num_packets_generated;

            total_rts_lost_slotted_bo = total_rts_lost_slotted_bo + node_container[m].rts_lost_slotted_bo;
            total_rts_cts_sent = total_rts_cts_sent + node_container[m].rts_cts_sent;
            total_prob_slotted_bo_collision = total_prob_slotted_bo_collision + node_container[m].prob_slotted_bo_collision;
            total_num_tx_init_not_possible = total_num_tx_init_not_possible + node_container[m].num_tx_init_not_possible;
            proportional_fairness = proportional_fairness + log10(node_container[m].throughput);
            jains_fairness_aux = jains_fairness_aux + pow(node_container[m].throughput, 2);
            total_delay = total_delay + node_container[m].average_delay;
            if(node_container[m].average_delay > max_delay) max_delay = node_container[m].average_delay;
            if(node_container[m].average_delay < min_delay) min_delay = node_container[m].average_delay;
            av_expected_backoff = av_expected_backoff + node_container[m].expected_backoff;
            av_expected_waiting_time = av_expected_waiting_time + node_container[m].average_waiting_time;

            total_bandiwdth_tx = total_bandiwdth_tx + node_container[m].bandwidth_used_txing;

            if(node_container[m].throughput < min_throughput) {
                ix_wlan_min_throughput = m;
                min_throughput = node_container[m].throughput;
            }

            if(node_container[m].throughput > max_throughput) max_throughput = node_container[m].throughput;

        }
    }

    av_expected_backoff = av_expected_backoff / total_wlans_number;
    av_expected_waiting_time = av_expected_waiting_time / total_wlans_number;

    // Supposing that number_aps = number_nodes/2
    jains_fairness = pow(total_throughput, 2) /
            (total_nodes_number/2 * jains_fairness_aux);

    if (print_system_logs) {

        printf("\n%s General Statistics:\n", LOG_LVL1);
        printf("%s Average throughput per WLAN = %.3f Mbps (%.2f pkt/s)\n",
                LOG_LVL2, (total_throughput * pow(10,-6)/total_wlans_number),
                (total_throughput / (double) frame_length) /total_wlans_number);
        printf("%s Min. throughput = %.2f Mbps (%.2f pkt/s)\n",
                LOG_LVL3, min_throughput * pow(10,-6), min_throughput / (frame_length * max_num_packets_aggregated));
        printf("%s Max. throughput = %.2f Mbps (%.2f pkt/s)\n",
                        LOG_LVL3, max_throughput * pow(10,-6), max_throughput / (frame_length * max_num_packets_aggregated));
        printf("%s Total throughput = %.2f Mbps\n", LOG_LVL3, total_throughput * pow(10,-6));
        printf("%s Total number of packets sent = %d\n", LOG_LVL3, total_data_packets_sent);
        printf("%s Average number of data packets successfully sent per WLAN = %.2f\n",
                LOG_LVL4, ( (double) total_data_packets_sent/ (double) total_wlans_number));
        printf("%s Average number of RTS packets lost due to slotted BO = %f (%.3f %% loss)\n",
                LOG_LVL4,
                (double) total_rts_lost_slotted_bo/(double) total_wlans_number,
                ((double) total_rts_lost_slotted_bo *100/ (double) total_rts_cts_sent));
        printf("%s Average number of packets sent per WLAN = %d\n", LOG_LVL3, (total_data_packets_sent/total_wlans_number));
        printf("%s Proportional Fairness = %.2f\n", LOG_LVL2, proportional_fairness);
        printf("%s Jain's Fairness = %.2f\n",  LOG_LVL2, jains_fairness);
        printf("%s Prob. collision by slotted BO = %.3f\n", LOG_LVL2, total_prob_slotted_bo_collision / total_wlans_number);
        printf("%s Av. delay = %.2f ms\n", LOG_LVL2, total_delay * pow(10,3) / total_wlans_number);
        printf("%s Max. delay = %.2f ms\n", LOG_LVL3, max_delay * pow(10,3));
        printf("%s Av. expected waiting time = %.2f ms\n", LOG_LVL3, av_expected_waiting_time * pow(10,3));
        printf("%s Average bandwidth used for transmitting = %.2f MHz\n",
            LOG_LVL2, total_bandiwdth_tx / (double) total_wlans_number);
        printf("%s Time channel was idle = %.2f s (%f%%)\n",  LOG_LVL2,
            node_container[0].sum_time_channel_idle, (100*node_container[0].sum_time_channel_idle/simulation_time_komondor));
        printf("\n\n");
    }

    printf("\n");

    if (save_system_logs) {

        // Simulation log file
        fprintf(logger_simulation.file,"%s Average throughput per WLAN = %.2f Mbps\n", LOG_LVL2, (total_throughput * pow(10,-6)/total_wlans_number));
        fprintf(logger_simulation.file,"%s Total throughput = %.2f Mbps\n", LOG_LVL3, total_throughput * pow(10,-6));
        fprintf(logger_simulation.file,"%s Total number of packets sent = %d\n", LOG_LVL3, total_data_packets_sent);
        fprintf(logger_simulation.file,"%s Average number of data packets successfully sent per WLAN = %.2f\n",
                LOG_LVL4, ( (double) total_data_packets_sent/ (double) total_wlans_number));
        fprintf(logger_simulation.file,"%s Average number of RTS packets lost due to slotted BO = %.2f (%.2f %% loss)\n",
                LOG_LVL4,
                (double) total_rts_lost_slotted_bo/(double) total_wlans_number,
                ((double) total_rts_lost_slotted_bo *100/ (double) total_rts_cts_sent));
        fprintf(logger_simulation.file,"%s Average number of packets sent per WLAN = %d\n", LOG_LVL3, (total_data_packets_sent/total_wlans_number));
        fprintf(logger_simulation.file,"%s Proportional Fairness = %.2f\n", LOG_LVL2, proportional_fairness);
        fprintf(logger_simulation.file,"%s Jain's Fairness = %.2f\n",  LOG_LVL2, jains_fairness);
        fprintf(logger_simulation.file,"%s Jain's Fairness = %.2f\n",  LOG_LVL2, jains_fairness);
        fprintf(logger_simulation.file,"\n");

        // If csv file is empty, add header
        fseek(logger_script_csv.file, 0, SEEK_END);
        unsigned long len = (unsigned long)ftell(logger_script_csv.file);
        printf("length = %lu", len);
        if (len == 0) {
            fprintf(logger_script_csv.file, "filename;sim_code;wlan_id;wlan_code;node_id;node_code;throughput[Mbps];"
                                            "data_packets_sent;data_packets_lost;rts_cts_sent;rts_cts_lost\n");
        }

        for(int m=0; m < total_nodes_number; ++m){
            fprintf(logger_script.file, "%s Node #%d (%s) Throughput = %f\n", LOG_LVL2, m,
                node_container[m].node_code.c_str(), node_container[m].throughput);

            if(node_container[m].node_type == NODE_TYPE_AP){
                // Fill CSV script output
                fprintf(logger_script_csv.file, "%s;", nodes_input_filename);               // Smiluation code
                fprintf(logger_script_csv.file, "%s;", simulation_code.c_str());            // Smiluation code
                fprintf(logger_script_csv.file, "%d;", node_container[m].wlan.wlan_id);     // WLAN ID
                fprintf(logger_script_csv.file, "%s;", node_container[m].wlan.wlan_code.c_str());   // WLAN code
                fprintf(logger_script_csv.file, "%d;", node_container[m].node_id);          // Node ID
                fprintf(logger_script_csv.file, "%s;", node_container[m].node_code.c_str());        // Node code
                fprintf(logger_script_csv.file, "%f;", node_container[m].throughput * pow(10,-6));  // Throughput [Mbps]
                fprintf(logger_script_csv.file, "%d;", node_container[m].data_packets_sent);        // Packets sent
                fprintf(logger_script_csv.file, "%d;", node_container[m].data_packets_lost);        // Packets lost
                fprintf(logger_script_csv.file, "%d;", node_container[m].rts_cts_sent);     // RTS packets sent
                fprintf(logger_script_csv.file, "%d", node_container[m].rts_cts_lost);      // RTS packets lost
                fprintf(logger_script_csv.file, "\n");                                      // End of line
            }
        }
    }

    int simulation_index (1);

    switch(simulation_index){

        case 0:{
            // For toy scenarios
            fprintf(logger_script.file, ";%.2f;%.2f;%f;%f\n",
                node_container[0].throughput * pow(10,-6),
                node_container[2].throughput * pow(10,-6),
                node_container[0].prob_slotted_bo_collision,
                node_container[2].prob_slotted_bo_collision);
            break;
        }

        case 1:{
            // For large scenarios (Node density vs. throughput)
            fprintf(logger_script.file, ";%.2f;%.2f;%f;%.2f;%d;%.2f\n",
                (total_throughput * pow(10,-6)/total_wlans_number),
                proportional_fairness,
                jains_fairness,
                min_throughput * pow(10,-6),
                ix_wlan_min_throughput,
                total_bandiwdth_tx / (double) total_wlans_number);
            break;
        }

        case 2:{
            // Sergio logs for central WLAN scenario
            fprintf(logger_script.file, ";%.1f;%d;%d;%d;%d;%d;%d;%d\n",
                node_container[0].throughput * pow(10,-6),
                node_container[0].rts_cts_sent,
                node_container[0].rts_cts_lost,
                node_container[0].rts_lost_slotted_bo,
                node_container[0].data_packets_sent,
                node_container[0].data_packets_lost,
                node_container[0].num_tx_init_tried,
                node_container[0].num_tx_init_not_possible);
            break;
        }

        case 3:{
            // Biancci multiple WLANs
            fprintf(logger_script.file, ";%.2f;%.3f;%.5f\n",
                av_expected_backoff / SLOT_TIME,
                (total_throughput * pow(10,-6)/total_wlans_number),
                total_prob_slotted_bo_collision / total_wlans_number);
            break;
        }

        case 4:{
            // DCB validation
            fprintf(logger_script.file, ";%.5f",
                total_prob_slotted_bo_collision / total_wlans_number);
            for(int w = 0; w < total_wlans_number; ++w) {
                fprintf(logger_script.file, ";%.3f", node_container[w*2].throughput * pow(10,-6));
            }
            fprintf(logger_script.file, "\n");
            break;
        }

        case 5:{
            // Variability of optimal policies
            for(int w = 0; w < total_wlans_number; ++w) {
                fprintf(logger_script.file, ";%.5f", node_container[w*2].prob_slotted_bo_collision);
            }
            for(int w = 0; w < total_wlans_number; ++w) {
                fprintf(logger_script.file, ";%.3f", node_container[w*2].throughput * pow(10,-6));
            }
            fprintf(logger_script.file, "\n");
            break;
        }

        case 6:{
            // Sergio logs for Paper #5 Toy Scenario I and II: 2 WLANs overlap scenario, and 3 line scenario
            fprintf(logger_script.file, ";%d;%d;%.0f;%.0f;%.0f;%.0f;%.2f;%.2f;"
                "%.4f;%.4f;%.2f;%.2f;%.4f;%.4f;%.4f;%.4f;%.4f;%.4f;%.2f;%.2f\n",
                node_container[0].current_dcb_policy,
                node_container[2].current_dcb_policy,
                node_container[0].num_packets_generated,
                node_container[2].num_packets_generated,
                node_container[0].throughput / (frame_length * max_num_packets_aggregated),
                node_container[2].throughput / (frame_length * max_num_packets_aggregated),
                node_container[0].average_rho,
                node_container[2].average_rho,
                node_container[0].average_delay * pow(10,3),
                node_container[2].average_delay * pow(10,3),
                node_container[0].average_utilization,
                node_container[2].average_utilization,
                node_container[0].prob_slotted_bo_collision,
                node_container[2].prob_slotted_bo_collision,
                node_container[0].average_waiting_time / SLOT_TIME,
                node_container[2].average_waiting_time / SLOT_TIME,
                node_container[0].num_packets_dropped * 100/ node_container[0].num_packets_generated,
                node_container[2].num_packets_dropped * 100/ node_container[2].num_packets_generated,
                 (double) node_container[0].data_frames_acked / node_container[0].data_packets_acked,
                 (double) node_container[2].data_frames_acked / node_container[2].data_packets_acked
                );
            break;
        }

        case 7:{
            // Sergio logs for Paper #5: central WLAN scenario
            fprintf(logger_script.file, ";%.0f;%d;%d;%d;%d;%d;%d;%d;%f;%f;%f;%f;%f\n",
                node_container[0].throughput / (frame_length * max_num_packets_aggregated),
                node_container[0].rts_cts_sent,
                node_container[0].rts_cts_lost,
                node_container[0].rts_lost_slotted_bo,
                node_container[0].data_packets_sent,
                node_container[0].data_packets_lost,
                node_container[0].num_tx_init_tried,
                node_container[0].num_tx_init_not_possible,
                node_container[0].average_delay * pow(10,3),
                node_container[0].average_waiting_time / SLOT_TIME,
                node_container[0].prob_slotted_bo_collision,
                node_container[0].num_packets_dropped * 100/ node_container[0].num_packets_generated,
                (double) node_container[0].data_frames_acked / node_container[0].data_packets_acked);
            break;
        }

        case 8:{
            // Sergio logs for Paper #5: Central WLAN scenario
            fprintf(logger_script.file, ";%d;%.0f;%.0f;%.2f;"
                "%.4f;%.2f;%.4f;%.4f;%.4f;%.4f\n",
                node_container[0].current_dcb_policy,
                node_container[0].num_packets_generated,
                node_container[0].throughput / (frame_length * max_num_packets_aggregated),
                node_container[0].average_rho,
                node_container[0].average_delay * pow(10,3),
                node_container[0].average_utilization,
                node_container[0].prob_slotted_bo_collision,
                node_container[0].average_waiting_time / SLOT_TIME,
                node_container[0].num_packets_dropped * 100/ node_container[0].num_packets_generated,
                 (double) node_container[0].data_frames_acked / node_container[0].data_packets_acked
                );
            break;
        }

        case 9:{
            // Sergio logs for Paper #5: 6 WLAN random
            fprintf(logger_script.file, ";%.2f;%.2f;%.2f;%d;%.4f;%.4f;%.4f;%.2f;%.2f;%.2f;%f;%f;%f\n",
                total_throughput/(frame_length * max_num_packets_aggregated * total_wlans_number),
                (total_throughput * pow(10,-6)/total_wlans_number),
                min_throughput/(frame_length * max_num_packets_aggregated),
                ix_wlan_min_throughput,
                proportional_fairness,
                jains_fairness,
                total_prob_slotted_bo_collision / total_wlans_number,
                total_delay * pow(10,3) / total_wlans_number,
                max_delay * pow(10,3),
                total_bandiwdth_tx / (double) total_wlans_number,
                av_expected_waiting_time * pow(10,3),
                min_delay * pow(10,3),
                max_throughput/(frame_length * max_num_packets_aggregated)
                );
            break;
        }

        // Validation scenarios
        case 10:{
            if (total_nodes_number == 2 || total_nodes_number == 3) {
                // Basic scenarios
                fprintf(logger_script.file, ";%.2f\n",
                    node_container[0].throughput * pow(10,-6));
            } else if (total_nodes_number == 6) {
                // Complex scenarios
                fprintf(logger_script.file, ";%.2f;%.2f;%.2f\n",
                    node_container[0].throughput * pow(10,-6),
                    node_container[2].throughput * pow(10,-6),
                    node_container[4].throughput * pow(10,-6));
            } else {
                printf("Error in Komondor :: Stop(): be care of the desired generated logs (script)\n");
            }
            break;
        }

        default:{
          printf("No simulation type found\n");
          break;
        }

    }

    // End of logs
    fclose(simulation_output_file);
    fclose(script_output_file);

    printf("%s SIMULATION '%s' FINISHED\n", LOG_LVL1, simulation_code.c_str());
    printf("------------------------------------------\n");

};

/*
 * InputChecker(): checks that input is set in proper format and values are acceptable
 */
void Komondor :: InputChecker(){

    // TODO: system channels vs. WLANs channels must match

    // Auxiliary arrays
    int nodes_ids[total_nodes_number];
    double nodes_x[total_nodes_number];
    double nodes_y[total_nodes_number];
    double nodes_z[total_nodes_number];
    for(int i = 0; i<total_nodes_number;++i){
        nodes_ids[i] = 0;
        nodes_x[i] = 0;
        nodes_y[i] = 0;
        nodes_z[i] = 0;
    }

    if (print_system_logs) printf("%s Validating input files...\n", LOG_LVL2);

    for (int i = 0; i < total_nodes_number; ++i) {

        nodes_ids[i] = node_container[i].node_id;
        nodes_x[i] = node_container[i].x;
        nodes_y[i] = node_container[i].y;
        nodes_z[i] = node_container[i].z;

        // TPC values (min <= defalut <= max)
        if (node_container[i].tpc_min > node_container[i].tpc_max
                || node_container[i].tpc_default > node_container[i].tpc_max
                || node_container[i].tpc_default < node_container[i].tpc_min) {
            printf("\nERROR: TPC values are not properly configured at node in line %d\n"
                    "node_container[i].tpc_min = %f\n"
                    "node_container[i].tpc_default = %f\n"
                    "node_container[i].tpc_max = %f\n\n",
                    i+2, node_container[i].tpc_min, node_container[i].tpc_default, node_container[i].tpc_max);
            exit(-1);
        }

        // CCA values (min <= defalut <= max)
        if (node_container[i].cca_min > node_container[i].cca_max
                || node_container[i].cca_default > node_container[i].cca_max
                || node_container[i].cca_default < node_container[i].cca_min) {
            printf("\nERROR: CCA values are not properly configured at node in line %d\n\n",i+2);
            exit(-1);
        }

        // Channel values (min <= primary <= max)
        if (node_container[i].current_primary_channel > node_container[i].max_channel_allowed
                || node_container[i].current_primary_channel < node_container[i].min_channel_allowed
                || node_container[i].min_channel_allowed > node_container[i].max_channel_allowed
                || node_container[i].current_primary_channel > num_channels_komondor
                || node_container[i].min_channel_allowed > (num_channels_komondor-1)
                || node_container[i].max_channel_allowed > (num_channels_komondor-1)) {
            printf("\nERROR: Channels are not properly configured at node in line %d\n\n",i+2);
            exit(-1);
        }
    }

    for (int i = 0; i < total_nodes_number; ++i) {
        for (int j = 0; j < total_nodes_number; ++j) {

            // Node IDs must be different
            if(i!=j && nodes_ids[i] == nodes_ids[j] && i < j) {
                printf("\nERROR: Nodes in lines %d and %d have the same ID\n\n",i+2,j+2);
                exit(-1);
            }

            // Nodes position may be required to be different
            if(i!=j && nodes_x[i] == nodes_x[j] && nodes_y[i] == nodes_y[j] && nodes_z[i] == nodes_z[j] && i < j) {
                printf("%s nERROR: Nodes in lines %d and %d are exactly at the same position\n\n", LOG_LVL2, i+2,j+2);
                exit(-1);
            }
        }
    }

    if (print_system_logs) printf("%s Input files validated!\n", LOG_LVL3);

}

/*
 * SetupEnvironmentByReadingInputFile(): sets up the Komondor environment
 * Input arguments:
 * - system_filename: system input filename
 */
void Komondor :: SetupEnvironmentByReadingInputFile(const char *system_filename) {

    if (print_system_logs) printf("%s Reading system configuration file '%s'...\n", LOG_LVL1, system_filename);
    fprintf(simulation_output_file, "%s KOMONDOR SIMULATION '%s' (seed %d)", LOG_LVL1, simulation_code.c_str(), seed);

    GError* error = NULL;
    GKeyFile* key_file = g_key_file_new();

    if (!g_key_file_load_from_file(key_file, system_filename, G_KEY_FILE_NONE, &error)){
        printf("Error loading configuration file\n");
        exit(-1);
    }

    gint val_int = 0;
    const gchar* key = "num_channels"; 
    val_int = g_key_file_get_integer(key_file, "System", key, &error);
    if (error != NULL){
        printf("Unable to parse %s parameter in config file!\n",key);
        exit(-1);
    }
    num_channels_komondor = val_int;

    key = "basic_channel_bandwidth";
    val_int = g_key_file_get_integer(key_file, "System",key, &error);
    if (error != NULL){
        printf("Unable to parse %s parameter in config file!\n", key);
        exit(-1);
    }
    basic_channel_bandwidth = val_int;

    key = "pdf_backoff";
    val_int = g_key_file_get_integer(key_file, "System",key, &error);
    if (error != NULL){
        printf("Unable to parse %s parameter in config file!\n", key);
        exit(-1);
    }
    pdf_backoff = val_int;

    key = "pdf_tx_time";
    val_int = g_key_file_get_integer(key_file, "System",key, &error);
    if (error != NULL){
        printf("Unable to parse %s parameter in config file!\n", key);
        exit(-1);
    }
    pdf_tx_time = val_int;

    key = "packet_length";
    val_int = g_key_file_get_integer(key_file, "System",key, &error);
    if (error != NULL){
        printf("Unable to parse %s parameter in config file!\n", key);
        exit(-1);
    }
    frame_length = val_int; //WHY THE HELL IS A PACKET NOW A FRAME??

    key = "num_packets_aggregated";
    val_int = g_key_file_get_integer(key_file, "System",key, &error);
    if (error != NULL){
        printf("Unable to parse %s parameter in config file!\n", key);
        exit(-1);
    }
    max_num_packets_aggregated = val_int;

    key = "path_loss_model";
    val_int = g_key_file_get_integer(key_file, "System",key, &error);
    if (error != NULL){
        printf("Unable to parse %s parameter in config file!\n", key);
        exit(-1);
    }
    path_loss_model = val_int;

    key = "capture_effect";
    val_int = g_key_file_get_integer(key_file, "System",key, &error);
    if (error != NULL){
        printf("Unable to parse %s parameter in config file!\n", key);
        exit(-1);
    }
    capture_effect = ConvertPower(DB_TO_LINEAR, val_int);

    key = "noise_level";
    val_int = g_key_file_get_integer(key_file, "System",key, &error);
    if (error != NULL){
        printf("Unable to parse %s parameter in config file!\n", key);
        exit(-1);
    }
    noise_level = ConvertPower(DBM_TO_PW, val_int);

    key = "adjacent_channel_model";
    val_int = g_key_file_get_integer(key_file, "System",key, &error);
    if (error != NULL){
        printf("Unable to parse %s parameter in config file!\n", key);
        exit(-1);
    }
    adjacent_channel_model = val_int;

    key = "collisions_model";
    val_int = g_key_file_get_integer(key_file, "System",key, &error);
    if (error != NULL){
        printf("Unable to parse %s parameter in config file!\n", key);
        exit(-1);
    }
    collisions_model = val_int;

    key = "constant_PER";
    val_int = g_key_file_get_integer(key_file, "System",key, &error);
    if (error != NULL){
        printf("Unable to parse %s parameter in config file!\n", key);
        exit(-1);
    }
    constant_per = val_int;

    key = "traffic_model";
    val_int = g_key_file_get_integer(key_file, "System",key, &error);
    if (error != NULL){
        printf("Unable to parse %s parameter in config file!\n", key);
        exit(-1);
    }
    traffic_model = val_int;

    key = "backoff_type";
    val_int = g_key_file_get_integer(key_file, "System",key, &error);
    if (error != NULL){
        printf("Unable to parse %s parameter in config file!\n", key);
        exit(-1);
    }
    backoff_type = val_int;

    key = "capture_effect_model";
    val_int = g_key_file_get_integer(key_file, "System",key, &error);
    if (error != NULL){
        printf("Unable to parse %s parameter in config file!\n", key);
        exit(-1);
    }
    capture_effect_model = val_int;

    gboolean val_bool = FALSE; 
    key = "cw_adaptation";
    val_bool = g_key_file_get_boolean(key_file, "System", key, &error);
    if (error != NULL){
        printf("Unable to parse %s parameter in config file!\n", key);
        exit(-1);
    }
    cw_adaptation = val_bool ? 1 : 0; //Backwards compatability in the code...

    key = "pifs_activated";
    val_bool = g_key_file_get_boolean(key_file, "System", key, &error);
    if (error != NULL){
        printf("Unable to parse %s parameter in config file!\n", key);
        exit(-1);
    }
    pifs_activated = val_bool ? 1 : 0; //Backwards compatability in the code...

    g_key_file_free(key_file);

}

/* *******************
 * * NODE GENERATION *
 * *******************
 */

/*
 * GenerateNodes(): generates the nodes randomely if AP file is used, or deterministically if NODE file is used.
 * Input arguments:
 * - nodes_filename: AP or nodes filename
 */
void Komondor :: GenerateNodes(const char *nodes_filename) {

    if (print_system_logs) printf("%s Generating nodes...\n", LOG_LVL1);
    fprintf(simulation_output_file, "%s Generating nodes...\n", LOG_LVL1);

    if (print_system_logs) printf("%s Generating nodes DETERMINISTICALLY through NODES input file...\n", LOG_LVL2);
    if (save_system_logs) fprintf(simulation_output_file, "%s Generating nodes DETERMINISTICALLY...\n", LOG_LVL2);
    ParseNodes(nodes_filename);

}

/*
 * GenerateAgents(): generates the agents according to the information in the input file.
 * Input arguments:
 * - agents_filename: agents filename
 */
void Komondor :: GenerateAgents(const char *agents_filename) {

    if (print_system_logs) printf("%s Generating agents...\n", LOG_LVL1);

    if (print_system_logs) printf("%s Reading agents input file '%s'...\n", LOG_LVL2, agents_filename);

    // STEP 1: set size of the agents container
    total_agents_number = GetNumOfLines(agents_filename);
    agent_container.SetSize(total_agents_number);

    if (print_system_logs) printf("%s Num. of agents (WLANs): %d/%d\n", LOG_LVL3, total_agents_number, total_wlans_number);

    // STEP 2: read the input file to determine the action space
    if (print_system_logs) printf("%s Setting action space...\n", LOG_LVL4);
    FILE* stream_agents = fopen(agents_filename, "r");
    char line_agents[CHAR_BUFFER_SIZE];
    first_line_skiped_flag = 0; // Flag for skipping first informative line of input file

    int agent_ix (0);   // Auxiliar wlan index

    while (fgets(line_agents, CHAR_BUFFER_SIZE, stream_agents)){

        if(!first_line_skiped_flag){

            first_line_skiped_flag = 1;

        } else{

            char* tmp_agents = strdup(line_agents);

            // Find the length of the channel actions array
            tmp_agents = strdup(line_agents);
            const char *channel_values_aux (GetField(tmp_agents, IX_AGENT_CHANNEL_VALUES));
            std::string channel_values_text;
            channel_values_text.append(ToString(channel_values_aux));
            const char *channel_aux;
            channel_aux = strtok ((char*)channel_values_text.c_str(),",");
            num_actions_channel = 0;
            while (channel_aux != NULL) {
                channel_aux = strtok (NULL, ",");
                ++ num_actions_channel;
            }
            // Set the length of channel actions to agent's field
            agent_container[agent_ix].num_actions_channel = num_actions_channel;

            // Find the length of the CCA actions array
            tmp_agents = strdup(line_agents);
            const char *cca_values_aux (GetField(tmp_agents, IX_AGENT_CCA_VALUES));
            std::string cca_values_text;
            cca_values_text.append(ToString(cca_values_aux));
            const char *cca_aux;
            cca_aux = strtok ((char*)cca_values_text.c_str(),",");
            num_actions_cca = 0;
            while (cca_aux != NULL) {
                cca_aux = strtok (NULL, ",");
                ++ num_actions_cca;
            }

            // Set the length of CCA actions to agent's field
            agent_container[agent_ix].num_actions_cca = num_actions_cca;

            // Find the length of the Tx power actions array
            tmp_agents = strdup(line_agents);
            const char *tx_power_values_aux (GetField(tmp_agents, IX_AGENT_TX_POWER_VALUES));
            std::string tx_power_values_text;
            tx_power_values_text.append(ToString(tx_power_values_aux));
            const char *tx_power_aux;
            tx_power_aux = strtok ((char*)tx_power_values_text.c_str(),",");
            num_actions_tx_power = 0;
            while (tx_power_aux != NULL) {
                tx_power_aux = strtok (NULL, ",");
                ++ num_actions_tx_power;
            }

            // Set the length of Tx power actions to agent's field
            agent_container[agent_ix].num_actions_tx_power = num_actions_tx_power;

            // Find the length of the DCB actions actions array
            tmp_agents = strdup(line_agents);
            const char *policy_values_aux (GetField(tmp_agents, IX_AGENT_DCB_POLICY));
            std::string policy_values_text;
            policy_values_text.append(ToString(policy_values_aux));
            const char *policy_aux;
            policy_aux = strtok ((char*)policy_values_text.c_str(),",");
            num_actions_dcb_policy = 0;
            while (policy_aux != NULL) {
                policy_aux = strtok (NULL, ",");
                ++num_actions_dcb_policy;
            }

            // Set the length of DCB actions to agent's field
            agent_container[agent_ix].num_actions_dcb_policy = num_actions_dcb_policy;

            ++agent_ix;
            free(tmp_agents);

        }
    }

    if (print_system_logs) printf("%s Action space set!\n", LOG_LVL4);

    // STEP 3: set agents parameters
    if (print_system_logs) printf("%s Setting agents parameters...\n", LOG_LVL4);
    stream_agents = fopen(agents_filename, "r");
    first_line_skiped_flag = 0;     // Flag for skipping first informative line of input file

    agent_ix = 0;   // Auxiliar wlan index

    while (fgets(line_agents, CHAR_BUFFER_SIZE, stream_agents)){

        if(!first_line_skiped_flag){

            first_line_skiped_flag = 1;

        } else{

            // Initialize actions and arrays in agents
            agent_container[agent_ix].InitializeAgent();

            // Agent ID
            agent_container[agent_ix].agent_id = agent_ix;

            // WLAN code
            char* tmp_agents (strdup(line_agents));
            const char *wlan_code_aux (GetField(tmp_agents, IX_AGENT_WLAN_CODE));
            std::string wlan_code;
            wlan_code.append(ToString(wlan_code_aux));
            agent_container[agent_ix].wlan_code = wlan_code.c_str();

            //  Centralized flag
            tmp_agents = strdup(line_agents);
            int centralized_flag (atoi(GetField(tmp_agents, IX_CENTRALIZED_FLAG)));
            agent_container[agent_ix].centralized_flag = centralized_flag;
            if(centralized_flag) {
                ++total_controlled_agents_number;
                central_controller_flag = 1;
            }

            // Time between requests
            tmp_agents = strdup(line_agents);
            double time_between_requests (atof(GetField(tmp_agents, IX_AGENT_TIME_BW_REQUESTS)));
            agent_container[agent_ix].time_between_requests = time_between_requests;
            // Channel values
            tmp_agents = strdup(line_agents);
            std::string channel_values_text = ToString(GetField(tmp_agents, IX_AGENT_CHANNEL_VALUES));

            // Fill the channel actions array
            char *channel_aux_2;
            char *channel_values_text_char = new char[channel_values_text.length() + 1];
            strcpy(channel_values_text_char, channel_values_text.c_str());
            channel_aux_2 = strtok (channel_values_text_char,",");

            int ix (0);
            while (channel_aux_2 != NULL) {
                int a (atoi(channel_aux_2));
                agent_container[agent_ix].list_of_channels[ix] = a;
                channel_aux_2 = strtok (NULL, ",");
                ++ix;
            }

            // CCA values
            tmp_agents = strdup(line_agents);
            std::string cca_values_text = ToString(GetField(tmp_agents, IX_AGENT_CCA_VALUES));

            // Fill the CCA actions array
            char *cca_aux_2;
            char *cca_values_text_char = new char[cca_values_text.length() + 1];
            strcpy(cca_values_text_char, cca_values_text.c_str());
            cca_aux_2 = strtok (cca_values_text_char,",");

            ix = 0;
            while (cca_aux_2 != NULL) {
                int a = atoi(cca_aux_2);
                agent_container[agent_ix].list_of_cca_values[ix] = ConvertPower(DBM_TO_PW, a);
                cca_aux_2 = strtok (NULL, ",");
                ++ix;
            }

            // Tx Power values
            tmp_agents = strdup(line_agents);
            std::string tx_power_values_text = ToString(GetField(tmp_agents, IX_AGENT_TX_POWER_VALUES));

            // Fill the TX power actions array
            char *tx_power_aux_2;
            char *tx_power_values_text_char = new char[tx_power_values_text.length() + 1];
            strcpy(tx_power_values_text_char, tx_power_values_text.c_str());
            tx_power_aux_2 = strtok (tx_power_values_text_char,",");

            ix = 0;
            while (tx_power_aux_2 != NULL) {
                int a (atoi(tx_power_aux_2));
                agent_container[agent_ix].list_of_tx_power_values[ix] = ConvertPower(DBM_TO_PW, a);
                tx_power_aux_2 = strtok (NULL, ",");
                ++ix;
            }

            // DCB policy values
            tmp_agents = strdup(line_agents);
            std::string dcb_policy_values_text = ToString(GetField(tmp_agents, IX_AGENT_DCB_POLICY));

            // Fill the DCB policy actions array
            char *policy_aux_2;
            char *dcb_policy_values_text_char = new char[dcb_policy_values_text.length() + 1];
            strcpy(dcb_policy_values_text_char, dcb_policy_values_text.c_str());
            policy_aux_2 = strtok (dcb_policy_values_text_char,",");

            ix = 0;
            while (policy_aux_2 != NULL) {
                int a (atoi(policy_aux_2));
                agent_container[agent_ix].list_of_dcb_policy[ix] = a;
                policy_aux_2 = strtok (NULL, ",");
                ++ix;
            }

            // Type of reward
            tmp_agents = strdup(line_agents);
            int type_of_reward (atoi(GetField(tmp_agents, IX_AGENT_TYPE_OF_REWARD)));
            agent_container[agent_ix].type_of_reward = type_of_reward;

            // Learning mechanism
            tmp_agents = strdup(line_agents);
            int learning_mechanism (atoi(GetField(tmp_agents, IX_AGENT_LEARNING_MECHANISM)));
            agent_container[agent_ix].learning_mechanism = learning_mechanism;

            // Selected strategy
            tmp_agents = strdup(line_agents);
            int selected_strategy (atoi(GetField(tmp_agents, IX_AGENT_SELECTED_STRATEGY)));
            agent_container[agent_ix].selected_strategy = selected_strategy;

            // System
            agent_container[agent_ix].save_agent_logs = save_agent_logs;
            agent_container[agent_ix].print_agent_logs = print_agent_logs;

            // Initialize learning algorithm in agent
            agent_container[agent_ix].InitializeLearningAlgorithm();

            ++agent_ix;
            free(tmp_agents);

        }
    }

    if (print_system_logs) printf("%s Agents parameters set!\n", LOG_LVL4);

}

/*
 * GenerateCentralController(): generates the central controller (if applicable)
 * Input arguments:
 * -
 */
void Komondor :: GenerateCentralController(const char *agents_filename) {

    if (print_system_logs) printf("%s Generating the Central Controller...\n", LOG_LVL1);

    // Despite we only have a single controller, it must be declared as an array,
    // in order to properly perform inport & outport connections
    central_controller.SetSize(1);

    if (total_controlled_agents_number > 0) {

        central_controller[0].agents_number = total_controlled_agents_number;
        central_controller[0].wlans_number = total_wlans_number;
        central_controller[0].InitializeCentralController();

        int *agents_list;
        agents_list = new int[total_controlled_agents_number];
        int agent_list_ix (0);                  // Index considering the agents attached to the central entity
        double max_time_between_requests (0);   // To determine the maximum time between requests for agents

        for (int agent_ix = 0; agent_ix < total_controlled_agents_number; ++agent_ix) {
            if(agent_container[agent_ix].centralized_flag) {
                agents_list[agent_list_ix] = agent_container[agent_ix].agent_id;
                double agent_time_between_requests (agent_container[agent_list_ix].time_between_requests);
                if (agent_time_between_requests > max_time_between_requests) {
                    central_controller[0].time_between_requests = agent_time_between_requests;
                }
                ++agent_list_ix;
            }
        }

        // The overall "time between requests" is set to the maximum among all the agents
        central_controller[0].list_of_agents = agents_list;

        // Initialize the CC with parameters from the agents input file
        FILE* stream_cc = fopen(agents_filename, "r");
        char line_agents[CHAR_BUFFER_SIZE];
        char* tmp_agents (strdup(line_agents));
        first_line_skiped_flag = 0;     // Flag for skipping first informative line of input file

        while (fgets(line_agents, CHAR_BUFFER_SIZE, stream_cc)){
            if(!first_line_skiped_flag){
                first_line_skiped_flag = 1;
            } else{

                // Type OF reward
                tmp_agents = strdup(line_agents);
                int type_of_reward (atoi(GetField(tmp_agents, IX_AGENT_TYPE_OF_REWARD)));
                central_controller[0].type_of_reward = type_of_reward;
                // Learning mechanism
                tmp_agents = strdup(line_agents);
                int learning_mechanism (atoi(GetField(tmp_agents, IX_AGENT_LEARNING_MECHANISM)));
                central_controller[0].learning_mechanism = learning_mechanism;
                // Selected strategy
                tmp_agents = strdup(line_agents);
                int selected_strategy (atoi(GetField(tmp_agents, IX_AGENT_SELECTED_STRATEGY)));
                central_controller[0].selected_strategy = selected_strategy;

                // Find the length of the channel actions array
                tmp_agents = strdup(line_agents);
                const char *channel_values_aux (GetField(tmp_agents, IX_AGENT_CHANNEL_VALUES));
                std::string channel_values_text;
                channel_values_text.append(ToString(channel_values_aux));
                const char *channels_aux;
                channels_aux = strtok ((char*)channel_values_text.c_str(),",");
                int num_actions_channels = 0;
                while (channels_aux != NULL) {
                    channels_aux = strtok (NULL, ",");
                    ++ num_actions_channels;
                }
                central_controller[0].num_channels = num_actions_channels;

                free(tmp_agents);

            }
        }

        // System logs
        central_controller[0].save_controller_logs = save_agent_logs;
        central_controller[0].print_controller_logs = print_agent_logs;

        central_controller[0].total_nodes_number = total_nodes_number;

//      // Initialize learning algorithm in the CC
//      central_controller[0].InitializeLearningAlgorithm();

        // Print CC's information
        central_controller[0].PrintCentralControllerInfo();

    } else {
        printf("%s WARNING: THE CENTRAL CONTROLLER DOES NOT HAVE ANY ATTACHED AGENT! CHECK YOUR AGENTS' INPUT FILE\n", LOG_LVL2);
    }

}

void Komondor :: ParseNodes(const char* nodes_filename){
    GKeyFile* keyfile = g_key_file_new();
    GError* error = NULL;

    if (!g_key_file_load_from_file(keyfile, nodes_filename, G_KEY_FILE_NONE, &error)){
        printf("Error loading configuration file %s \n", nodes_filename);
        exit(-1);
    }


    gchar** groups = g_key_file_get_groups(keyfile, NULL); 
    gchar** g_ptr;


    gint num_nodes = 0;
    gint num_aps = 0;
    GSList* nodes = NULL;

    for (g_ptr=groups; *g_ptr != NULL; g_ptr++){
        gchar* g = *g_ptr;
        if (!g_str_has_prefix(g,"Node_")){
            continue;
        }

        printf("Parsing %s\n", g);
       
        gchar** split_strings; 
        gchar* node_name; 
        split_strings = g_strsplit(g, "Node_", 2);
        node_name = g_strdup(split_strings[1]);
        g_strfreev(split_strings);

        const gchar* key;

        key = "wlan_code";
        gchar* wlan_code = g_key_file_get_string(keyfile, g, key, &error);
        if (wlan_code == NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "type";
        gint node_type = g_key_file_get_integer(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "destination_id";
        gint destination_id = g_key_file_get_integer(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "x";
        gdouble pos_x = g_key_file_get_double(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "y";
        gdouble pos_y = g_key_file_get_double(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "z";
        gdouble pos_z = g_key_file_get_double(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "primary_channel";
        gint primary_channel = g_key_file_get_integer(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "min_channel_allowed";
        gint min_channel_allowed = g_key_file_get_integer(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }
           
        key = "max_channel_allowed";
        gint max_channel_allowed = g_key_file_get_integer(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "cw";
        gint cw = g_key_file_get_integer(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "cw_stage";
        gint cw_stage = g_key_file_get_integer(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "tpc_min";
        gdouble tpc_min_dbm = g_key_file_get_double(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "tpc_default";
        gdouble tpc_default_dbm = g_key_file_get_double(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "tpc_max";
        gdouble tpc_max_dbm = g_key_file_get_double(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "cca_min";
        gdouble cca_min_dbm = g_key_file_get_double(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "cca_default";
        gdouble cca_default_dbm = g_key_file_get_double(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "cca_max";
        gdouble cca_max_dbm = g_key_file_get_integer(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "tx_antenna_gain";
        gint tx_gain_db = g_key_file_get_integer(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "rx_antenna_gain";
        gint rx_gain_db = g_key_file_get_integer(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "channel_bonding_model";
        gint channel_bonding_model = g_key_file_get_integer(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "modulation_default";
        gint modulation_default = g_key_file_get_integer(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "central_freq";
        gdouble central_frequency = g_key_file_get_double(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "lambda";
        gdouble lambda = g_key_file_get_double(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "ieee_protocol";
        gint ieee_protocol = g_key_file_get_integer(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }

        key = "traffic_load";
        gdouble traffic_load = g_key_file_get_double(keyfile, g, key, &error);
        if (error != NULL){
            printf("Error parsing key %s of node %s in configuration file!\n", key, g);
            exit(-1); 
        }



        struct NodeConfig* n = (struct NodeConfig*) g_malloc(sizeof(struct NodeConfig));
        n->code = node_name;
        n->type = node_type;

        if (node_type == NODE_TYPE_AP){
            num_aps++;
        }

        n->wlan = wlan_code; 
        n->destination_id = destination_id;
        n->x = pos_x;
        n->y = pos_y;
        n->z = pos_z;
        n->primary_channel = primary_channel;
        n->min_channel_allowed = min_channel_allowed;
        n->max_channel_allowed = max_channel_allowed;
        n->cw = cw;
        n->cw_stage = cw_stage;
        n->tpc_min = ConvertPower(DBM_TO_PW, tpc_min_dbm);
        n->tpc_default = ConvertPower(DBM_TO_PW, tpc_default_dbm);
        n->tpc_max = ConvertPower(DBM_TO_PW, tpc_max_dbm);
        n->cca_min = ConvertPower(DBM_TO_PW, cca_min_dbm);
        n->cca_default = ConvertPower(DBM_TO_PW, cca_default_dbm);
        n->cca_max = ConvertPower(DBM_TO_PW, cca_max_dbm);
        n->tx_antenna_gain = ConvertPower(DB_TO_LINEAR, tx_gain_db);
        n->rx_antenna_gain = ConvertPower(DB_TO_LINEAR, rx_gain_db);
        n->channel_bonding_model =  channel_bonding_model;
        n->modulation_default = modulation_default;
        n->central_frequency = central_frequency;
        n->lambda = lambda;
        n->ieee_protocol = ieee_protocol;
        n->traffic_load = traffic_load;

        nodes = g_slist_append(nodes, n);
        num_nodes++;


    }

    if (num_nodes == 0){
        printf("No nodes found in configuration file!\n");
        exit(-1);
    }



    node_container.SetSize(num_nodes);
    traffic_generator_container.SetSize(num_nodes);

    wlan_container = new Wlan[num_aps];

    GSList* cur;
    gint i;
    gint wlan_index = 0;
    for (cur=nodes, i=0; cur!=NULL; cur=cur->next,i++){
        struct NodeConfig* n = (struct NodeConfig*) cur->data;
        node_container[i].node_id = i;
        node_container[i].node_type = n->type;

        node_container[i].node_code = ToString(n->code);
        node_container[i].wlan_code = ToString(n->wlan); 
        node_container[i].destination_id = n->destination_id; 

        node_container[i].x = n->x;
        node_container[i].y = n->y;
        node_container[i].z = n->z;

        node_container[i].current_primary_channel = n->primary_channel; 
        node_container[i].min_channel_allowed = n->min_channel_allowed; 
        node_container[i].max_channel_allowed = n->max_channel_allowed; 

        node_container[i].cw_min = n->cw; 
        node_container[i].cw_stage_max = n->cw_stage; 

        node_container[i].tpc_min = n->tpc_min; 
        node_container[i].tpc_default = n->tpc_default; 
        node_container[i].tpc_max = n->tpc_max; 

        node_container[i].cca_min = n->cca_min; 
        node_container[i].cca_default = n->cca_default; 
        node_container[i].cca_max = n->cca_max; 

        node_container[i].tx_gain = n->tx_antenna_gain; 
        node_container[i].rx_gain = n->rx_antenna_gain; 
        
        node_container[i].current_dcb_policy = n->channel_bonding_model; 
        node_container[i].modulation_default = n->modulation_default; 
        node_container[i].central_frequency = n->central_frequency; 
        
        node_container[i].central_frequency = n->central_frequency * pow(10,9);  //WHY???
        node_container[i].ieee_protocol = n->ieee_protocol; 

        traffic_generator_container[i].node_type = n->type;
        traffic_generator_container[i].node_id = i;
        traffic_generator_container[i].traffic_model = traffic_model;
        traffic_generator_container[i].traffic_load = n->traffic_load;
        traffic_generator_container[i].lambda = n->lambda; //FIXME Why do we need this twice??
        traffic_generator_container[i].burst_rate = n->lambda;

        
        if (n->type == NODE_TYPE_AP){
            wlan_container[wlan_index].ap_id = i;
            wlan_container[wlan_index].wlan_id = wlan_index;
            wlan_container[wlan_index].wlan_code = n->wlan;
        }

        // System
        node_container[i].simulation_time_komondor = simulation_time_komondor;
        node_container[i].total_wlans_number = total_wlans_number;
        node_container[i].total_nodes_number = total_nodes_number;
        node_container[i].collisions_model = collisions_model;
        node_container[i].capture_effect = capture_effect;
        node_container[i].save_node_logs = save_node_logs;
        node_container[i].print_node_logs = print_node_logs;
        node_container[i].basic_channel_bandwidth = basic_channel_bandwidth;
        node_container[i].num_channels_komondor = num_channels_komondor;
        node_container[i].adjacent_channel_model = adjacent_channel_model;
        node_container[i].default_destination_id = NODE_ID_NONE;
        node_container[i].noise_level = noise_level;
        node_container[i].constant_per = constant_per;
        node_container[i].pdf_backoff = pdf_backoff;
        node_container[i].path_loss_model = path_loss_model;
        node_container[i].pdf_tx_time = pdf_tx_time;
        node_container[i].frame_length = frame_length;
        node_container[i].max_num_packets_aggregated = max_num_packets_aggregated;
        node_container[i].ack_length = ack_length;
        node_container[i].rts_length = rts_length;
        node_container[i].cts_length = cts_length;
        node_container[i].traffic_model = traffic_model;
        node_container[i].backoff_type = backoff_type;
        node_container[i].cw_adaptation = cw_adaptation;
        node_container[i].pifs_activated = pifs_activated;
        node_container[i].capture_effect_model = capture_effect_model;
        node_container[i].simulation_code = simulation_code;

        g_free(n->code);
        g_free(n->wlan);
        g_free(n);

    }
    g_slist_free(nodes);

    //Obscure global vars ftw!        
    total_wlans_number = num_aps;
    total_nodes_number = num_nodes;


    // I hate this stupid code so much...
    gint n;
    for (i=0; i<num_aps; i++){
        gint sta_cnt = 0;

        for(n=0; n<num_nodes; n++){
            if (node_container[n].node_type != NODE_TYPE_STA ||
                g_strcmp0(node_container[n].wlan_code.c_str(), wlan_container[i].wlan_code.c_str()) != 0){
                continue;
            }

            sta_cnt++;
        } 

        wlan_container[i].SetSizeOfSTAsArray(sta_cnt); //Seriously, who names your functions??
        wlan_container[i].num_stas = sta_cnt; //Another obscure unnecessary variable
        //Now we have to do this stupid shit again...
        //
        gint sta_index = 0;
        for(n=0; n<num_nodes; n++){

            if (node_container[n].node_type != NODE_TYPE_STA ||
                g_strcmp0(node_container[n].wlan_code.c_str(), wlan_container[i].wlan_code.c_str()) != 0){
                continue;
            }
            printf("Added Station %s to WLAN %s\n", node_container[n].node_code.c_str(), wlan_container[i].wlan_code.c_str());
            wlan_container[i].list_sta_id[sta_index] = node_container[n].node_id;
            node_container[n].wlan = wlan_container[i];
            sta_index++;

        }

        //Because the original authors do not understand the concept of pointers we have to do this:
        //Otherwise the the APs doesnt know which stations he has...
        node_container[wlan_container[i].ap_id].wlan = wlan_container[i];
        gint st; 
        for (st=0; st<wlan_container[i].num_stas; st++){
            gint sta_id = wlan_container[i].list_sta_id[st];
            node_container[sta_id].wlan = wlan_container[i];
        }

    }

    g_strfreev(groups);
    g_key_file_free(keyfile);

}

/***************************/
/* LOG AND DEBUG FUNCTIONS */
/***************************/

/*
 * printSystemInfo(): prints the Komondor environment general info
 */
void Komondor :: printSystemInfo(){

    if (print_system_logs){
        printf("%s total_nodes_number = %d\n", LOG_LVL3, total_nodes_number);
        printf("%s num_channels_komondor = %d\n", LOG_LVL3, num_channels_komondor);
        printf("%s basic_channel_bandwidth = %f MHz\n", LOG_LVL3, basic_channel_bandwidth);
        printf("%s pdf_backoff = %d\n", LOG_LVL3, pdf_backoff);
        printf("%s pdf_tx_time = %d\n", LOG_LVL3, pdf_tx_time);
        printf("%s frame_length = %d bits\n", LOG_LVL3, frame_length);
        printf("%s traffic_model = %d\n", LOG_LVL3, traffic_model);
        printf("%s backoff_type = %d\n", LOG_LVL3, backoff_type);
        printf("%s cw_adaptation = %d\n", LOG_LVL3, cw_adaptation);
        printf("%s pifs_activated = %d\n", LOG_LVL3, pifs_activated);
        printf("%s capture_effect_model = %d\n", LOG_LVL3, capture_effect_model);
        printf("%s max_num_packets_aggregated = %d\n", LOG_LVL3, max_num_packets_aggregated);
        printf("%s path_loss_model = %d\n", LOG_LVL3, path_loss_model);
        printf("%s capture_effect = %f [linear] (%f dB)\n", LOG_LVL3, capture_effect, ConvertPower(LINEAR_TO_DB, capture_effect));
        printf("%s noise_level = %f pW (%f dBm)\n",
                LOG_LVL3, noise_level, ConvertPower(PW_TO_DBM, noise_level));
        printf("%s adjacent_channel_model = %d\n", LOG_LVL3, adjacent_channel_model);
        printf("%s collisions_model = %d\n", LOG_LVL3, collisions_model);
        printf("%s Constant PER = %f\n", LOG_LVL3, constant_per);
        printf("\n");
    }
}

/*
 * WriteSystemInfo(): writes the Komondor environment general info
 * Input arguments:
 * - logger: AP or nodes filename
 */
void Komondor :: WriteSystemInfo(Logger logger){

    fprintf(logger.file, "%s total_nodes_number = %d\n", LOG_LVL3, total_nodes_number);
    fprintf(logger.file, "%s num_channels_komondor = %d\n", LOG_LVL3, num_channels_komondor);
    fprintf(logger.file, "%s basic_channel_bandwidth = %f\n", LOG_LVL3, basic_channel_bandwidth);
    fprintf(logger.file, "%s pdf_backoff = %d\n", LOG_LVL3, pdf_backoff);
    fprintf(logger.file, "%s pdf_tx_time = %d\n", LOG_LVL3, pdf_tx_time);
    fprintf(logger.file, "%s frame_length = %d bits\n", LOG_LVL3, frame_length);
    fprintf(logger.file, "%s ack_length = %d bits\n", LOG_LVL3, ack_length);
    fprintf(logger.file, "%s max_num_packets_aggregated = %d\n", LOG_LVL3, max_num_packets_aggregated);
    fprintf(logger.file, "%s path_loss_model = %d\n", LOG_LVL3, path_loss_model);
    fprintf(logger.file, "%s capture_effect = %f\n", LOG_LVL3, capture_effect);
    fprintf(logger.file, "%s noise_level = %f dBm\n", LOG_LVL3, noise_level);
    fprintf(logger.file, "%s adjacent_channel_model = %d\n", LOG_LVL3, adjacent_channel_model);
    fprintf(logger.file, "%s collisions_model = %d\n", LOG_LVL3, collisions_model);
}

/*
 * PrintAllNodesInfo(): prints the nodes info
 * Input arguments:
 * - info_detail_level: level of detail of the written logs
 */
void Komondor :: PrintAllNodesInfo(int info_detail_level){
    for(int n = 0; n < total_nodes_number; ++n ){
        node_container[n].PrintNodeInfo(info_detail_level);
    }
}

/*
 * PrintAllWlansInfo(): prints the WLANs info
 */
void Komondor :: PrintAllWlansInfo(){
    for(int w = 0; w < total_wlans_number; ++w){
        wlan_container[w].PrintWlanInfo();
    }
}

/*
 * PrintAgentsInfo(): prints the Agents info
 */
void Komondor :: PrintAllAgentsInfo(){
    for(int a = 0; a < total_agents_number; ++a ){
        agent_container[a].PrintAgentInfo();
    }
}

/*
 * WriteAllNodesInfo(): writes the WLANs info in a file
 * Input arguments:
 * - logger: logger containing the file to write on
 */
void Komondor :: WriteAllWlansInfo(Logger logger, std::string header_str){
    for(int w = 0; w < total_wlans_number; ++w){
        wlan_container[w].WriteWlanInfo(logger, header_str.c_str());
    }
}

/*
 * WriteAllNodesInfo(): writes the nodes info in a file
 * Input arguments:
 * - logger: logger containing the file to write on
 * - info_detail_level: level of detail of the written logs
 */
void Komondor :: WriteAllNodesInfo(Logger logger, int info_detail_level, std::string header_str){
    for(int n = 0; n < total_nodes_number; ++n){
        node_container[n].WriteNodeInfo(logger, info_detail_level, header_str.c_str());
    }
}

/*******************/
/* FILES FUNCTIONS */
/*******************/

/*
 * GetField(): returns a field corresponding to a given index from a CSV file
 * Input arguments:
 * - line: line of the CSV
 * - num: field number (index)
 */
const char* GetField(char* line, int num){
    const char* tok;
    for (tok = strtok(line, ";");
            tok && *tok;
            tok = strtok(NULL, ";\n"))
    {
        if (!--num)
            return tok;
    }
    return NULL;
}

/*
 * GetNumOfLines(): returns the number of lines of a csv file
 * Input arguments:
 * - filename: CSV filename
 */
int Komondor :: GetNumOfLines(const char *filename){
    int num_lines (0);
    // Nodes file
    FILE* stream = fopen(filename, "r");
    if (!stream){
        printf("Nodes configuration file %s not found!\n", filename);
        exit(-1);
    }
    char line[CHAR_BUFFER_SIZE];
    while (fgets(line, CHAR_BUFFER_SIZE, stream))
    {
        ++num_lines;
    }
    num_lines--;
    fclose(stream);
    return num_lines;
}

void show_usage(GOptionContext* context){
    gchar* help = g_option_context_get_help(context, TRUE, NULL);
    printf(help);
    g_free(help);
}

int main(int argc, char *argv[]){

    //TODO These options should be adjustable via args but are kind of dumb anyways...
    const gchar* script_output_filename = DEFAULT_SCRIPT_FILENAME; 
    const gchar* simulation_code = DEFAULT_SIMULATION_CODE;
    int save_system_logs = 1;
    int save_node_logs = 1;
    int save_agent_logs = 1;
    int print_system_logs = 1;
    int print_node_logs = 1;
    int print_agent_logs = 1;
    

    gchar** config = NULL;
    gchar* agents = NULL;
    gint  seed = -1; 
    gdouble sim_time = 60.0;
    static GOptionEntry options[] = 
    {
        {"agents" , 'a', 0, G_OPTION_ARG_FILENAME, &agents, "Path to agent configuration", "agent_path"},
        {"seed", 's', 0 , G_OPTION_ARG_INT, &seed, "Seed for pseudo-random number generators", "S"},
        {"time", 't', 0 , G_OPTION_ARG_DOUBLE, &sim_time, "Simulation Time", "T"},
        {G_OPTION_REMAINING, 0, 0,G_OPTION_ARG_FILENAME_ARRAY,&config, "Path to system and node configuration", "config_path"},
        { NULL } 
        
    };

    GError* error = NULL;
    GOptionContext* context;
    context = g_option_context_new(NULL);
    g_option_context_add_main_entries(context, options, NULL);

    if (!g_option_context_parse(context, &argc, &argv, &error)){
        printf("option parsing failed: %s\n", error->message);
        exit(1);
    }

    if (seed == -1){
        printf("No seed was provided!\n");
        show_usage(context);
        exit(1);
    }	 

    if (config == NULL){
        printf("Missing configuration file!\n"); 
        show_usage(context);
        exit(1);
    }


    int agents_enabled = 1 ? agents != NULL : 0;

    total_nodes_number = 0;

   
    // Generate Komondor component
    Komondor test;
    test.Seed = seed;
    srand(seed); // Needed for ensuring randomness dependency on seed
    test.StopTime(sim_time);
    test.Setup(sim_time, save_system_logs, save_node_logs, save_agent_logs, print_system_logs, print_node_logs, print_agent_logs,
        config[0], script_output_filename, simulation_code, seed,
        agents_enabled, agents);

    printf("------------------------------------------\n");
    printf("%s SIMULATION '%s' STARTED\n", LOG_LVL1, simulation_code);

    test.Run();

    return(0);
};
