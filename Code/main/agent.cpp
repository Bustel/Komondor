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
 * File description: defines the agent component
 *
 * - This file contains the instructions to be followed by an agent, as well
 * as the communication with the APs of the controlled networks
 */

#include <math.h>
#include <algorithm>
#include <stddef.h>
#include <iostream>
#include <stdlib.h>

#include <macros.h>
#include <node_configuration.hpp>
#include <auxiliary_methods.hpp>
#include <performance_metrics.hpp>
#include <agent_methods.hpp>
#include <action.hpp>


#include "../learning_modules/multi_armed_bandits/multi_armed_bandits.h"

// Agent component: "TypeII" represents components that are aware of the existence of the simulated time.
component Agent : public TypeII{

	// Methods
	public:

		// COST
		void Setup();
		void Start();
		void Stop();

		// Generic
		void InitializeAgent();
		void InitializeLearningAlgorithm();

		// Communication with AP
		void RequestInformationToAp();
		void ComputeNewConfiguration();
		void SendNewConfigurationToAp(Configuration &configuration_to_send);

		// Communication with other Agents (distributed methods)
		// ... To be completed

		// Print methods
		void PrintAgentInfo();
		void WriteConfiguration(Configuration configuration_to_write);

	// Public items (entered by agents constructor in komondor_main)
	public:

		// Specific to each agent
		int agent_id; 			// Node identifier
		int centralized_flag;		// Indicates whether the node is controlled by a central entity or not
		std::string wlan_code;	// WLAN code to which the agent belongs

		// Learning mechanism
		int learning_mechanism;
		int selected_strategy;

		// Actions management (tunable parameters)
		int *list_of_channels; 				// List of channels
		double *list_of_cca_values;			// List of CCA values
		double *list_of_tx_power_values;	// List of tx power values
		int *list_of_dcb_policy;			// List of DCB policies
		Action *actions;					// List of actions
		int num_actions_channel;			// Number of channels available
		int num_actions_cca;				// Number of CCA levels available
		int num_actions_tx_power;			// Number of TX power levels available
		int num_actions_dcb_policy;			// Number of DCB policies available

		// Other input parameters
		int type_of_reward;				// Type of reward
		double time_between_requests; 	// Time between two information requests to the AP (for a given measurement)

    	std::string simulation_code;		// Simulation code

	// Private items (just for node operation)
	private:

		//
		int num_actions;					// Number of actions (depends on the configuration parameters - CCA, TPC, channels, etc.)
		int num_requests;					// Number of requests made by the agent to the AP
		int ix_selected_arm; 				// Index of the current selected arm
		double initial_reward;				// Initial reward assigned to each arm
		double *reward_per_arm;				// Reward experienced after playing each arm
		double *average_reward_per_arm;		// Average reward experienced for each arm
		double *cumulative_reward_per_arm;	// Cumulative reward experienced for each arm
		int *times_arm_has_been_selected; 	// Number of times an arm has been played

		// Variables to store performance and configuration reports
		Performance performance;
		Configuration configuration;
		Configuration new_configuration;
		Configuration configuration_from_controller;

    	Logger logger;				// struct containing the attributes needed for writting logs in a file

		MultiArmedBandit mab_agent;
//		bool not_initialized;				// Boolean to determine whether the learning alg. has been initialized or not

	// Connections and timers
	public:

		// INPORT connections for receiving notifications
		inport void inline InportReceivingInformationFromAp(Configuration &configuration, Performance &performance);

		// INPORT (centralized system only)
		inport void inline InportReceivingRequestFromController(int destination_agent_id);
		inport void inline InportReceiveConfigurationFromController(int destination_agent_id, Configuration &new_configuration);

		// OUTPORT connections for sending notifications
		outport void outportRequestInformationToAp();
		outport void outportSendConfigurationToAp(Configuration &new_configuration);

		// OUTPORT (centralized system only)
		outport void outportAnswerToController(Configuration &configuration, Performance &performance, int agent_id);

		// Triggers
		Timer <trigger_t> trigger_request_information_to_ap; // Timer for requesting information to the AP

		// Every time the timer expires execute this
		inport inline void RequestInformationToAp(trigger_t& t1);

		// Connect timers to methods
		Agent () {
			connect trigger_request_information_to_ap.to_component,RequestInformationToAp;
		}

};

/*
 * Setup()
 */
void Agent :: Setup(){
	// Do nothing
};

/*
 * Start()
 */
void Agent :: Start(){
    logger.init(LOG_LVL_INFO, NULL, "Agent Logger");	

	if(centralized_flag) {
		// --- Do nothing ---
		// In case of being centralized, wait for a request from the controller
	} else {
		// Generate the first request, to be triggered after "time_between_requests"
		// *** We generate here the first request in order to obtain the AP's configuration
		double extra_wait_test = 0.005 * (double) agent_id;
		trigger_request_information_to_ap.Set(fix_time_offset(SimTime() + time_between_requests + extra_wait_test,13,12));
	}

};

/*
 * Stop()
 */
void Agent :: Stop(){

	logger.info( "%.15f;A%d;%s;%s Agent Stop()\n",
		SimTime(), agent_id, LOG_C00, LOG_LVL1);

};

/***************************/
/***************************/
/*  AP-AGENT COMMUNICATION */
/***************************/
/***************************/

/*
 * RequestInformationToAp(): method where an information retrieval request is performed to the AP.
 * This method is triggered by a process that decides the time between requests.
 */
void Agent :: RequestInformationToAp(trigger_t &){


    logger.debug( "%.15f;A%d;%s;%s RequestInformationToAp() (request #%d)\n",
			SimTime(), agent_id, LOG_F00, LOG_LVL1, num_requests);

	logger.debug("%.15f;A%d;%s;%s Requesting information to AP\n",
			SimTime(), agent_id, LOG_C00, LOG_LVL2);

	outportRequestInformationToAp();

	++ num_requests;

};

/*
 * InportReceivingInformationFromAp(): called when some node (this one included) starts a TX
 * INPUT:
 * - received_configuration: configuration report that is received from to the corresponding AP
 * - received_performance: performance report that is received from to the corresponding AP
 */
void Agent :: InportReceivingInformationFromAp(Configuration &received_configuration,
		Performance &received_performance){


	logger.debug( "%.15f;A%d;%s;%s InportReceivingInformationFromAp()\n",
			SimTime(), agent_id, LOG_F00, LOG_LVL1);

	logger.debug( "%.15f;A%d;%s;%s New information has been received from the AP\n",
			SimTime(), agent_id, LOG_C00, LOG_LVL2);

	configuration = received_configuration;

    WriteConfiguration(configuration);

	performance = received_performance;

	if (centralized_flag) {
		// Forward the information to the controller
		logger.debug( "%.15f;A%d;%s;%s Answering to the controller with current information\n",
			SimTime(), agent_id, LOG_F02, LOG_LVL2);
		outportAnswerToController(configuration, performance, agent_id);
	} else {
		// Compute a new configuration according to the updated rewards
		ComputeNewConfiguration();
	}

};

/*
 * SendNewConfigurationToAp():
 * INPUT:
 * - configuration_to_send: configuration report that is sent to the corresponding AP
 */
void Agent :: SendNewConfigurationToAp(Configuration &configuration_to_send){

//	printf("%s Agent #%d: Sending new configuration to AP\n", LOG_LVL1, agent_id);

	logger.debug( "%.15f;A%d;%s;%s SendNewConfigurationToAp()\n",
			SimTime(), agent_id, LOG_F00, LOG_LVL1);

	logger.debug( "%.15f;A%d;%s;%s Sending a new configuration to the AP\n",
			SimTime(), agent_id, LOG_C00, LOG_LVL2);

    WriteConfiguration(configuration_to_send);

	// TODO (LOW PRIORITY): generate a trigger to simulate delays in the agent-node communication
	outportSendConfigurationToAp(configuration_to_send);

	// Set trigger for next request in case of being an independent agent (not controlled by a central entity)
	if (!centralized_flag) {
		logger.debug( "%.15f;A%d;%s;%s Next request to be sent at %f\n",
			SimTime(), agent_id, LOG_C00, LOG_LVL2, fix_time_offset(SimTime() + time_between_requests,13,12));
		trigger_request_information_to_ap.Set(fix_time_offset(SimTime() + time_between_requests,13,12));
	} else {
		logger.debug( "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
	}

};

/***************************/
/***************************/
/*  AGENT-CC COMMUNICATION */
/***************************/
/***************************/

/*
 * InportReceivingRequestFromAgent(): called when some agent answers for information to the AP
 * INPUT:
 * - destination_agent_id: id of the agent that is requested by the central controller (CC)
 */
void Agent :: InportReceivingRequestFromController(int destination_agent_id) {

	if(agent_id == destination_agent_id) {

//		printf("%s Agent #%d: New information request received from the Controller\n", LOG_LVL1, agent_id);

		logger.debug( "+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n");

		logger.debug( "%.15f;A%d;%s;%s New information request received from the Controller for Agent %d\n",
			SimTime(), agent_id, LOG_F02, LOG_LVL2, destination_agent_id);

		/* Once the CC requests to retrieve information from the AP, a trigger is set, which determines
		 * the delay experienced during the CC-Agent and the Agent-AP communication
		 */
		trigger_request_information_to_ap.Set(fix_time_offset(SimTime(),13,12));

	}

}

/*
 * InportReceiveConfigurationFromAgent(): called when some agent sends instructions to the AP
 * INPUT:
 * - destination_agent_id: id of the agent to which the information is delivered
 * - received_configuration: reference of the configuration received from the controller
 */
void Agent :: InportReceiveConfigurationFromController(int destination_agent_id,
		Configuration &received_configuration) {

	if(agent_id == destination_agent_id) {

		logger.debug( "%.15f;A%d;%s;%s New configuration received from the Controller to Agent %d\n",
			SimTime(), agent_id, LOG_F02, LOG_LVL2, destination_agent_id);

		// Update the received configuration
		configuration_from_controller = received_configuration;
		// Forward the configuration to the AP
		SendNewConfigurationToAp(configuration_from_controller);

	}

}

/***************************/
/***************************/
/*  GENERATE A NEW CONFIG. */
/***************************/
/***************************/

/*
 * ComputeNewConfiguration(): computes a new configuration to be sent to the AP.
 * Different mechanisms can be used, which are expected to return the new configuration.
 */
void Agent :: ComputeNewConfiguration(){

	logger.debug( "%.15f;A%d;%s;%s ComputeNewConfiguration()\n",
			SimTime(), agent_id, LOG_F00, LOG_LVL1);

	// Update the current configuration according to the selected learning method
	switch(learning_mechanism) {

		/* Multi-Armed Bandits:
		 *
		 */
		case MULTI_ARMED_BANDITS:{

			// Update the configuration according to the MABs operation
			new_configuration = mab_agent.UpdateConfiguration(configuration, performance, logger, SimTime());
			break;

		}

		default:{
			logger.error("ERROR: %d is not a correct learning mechanism\n", learning_mechanism);
			exit(EXIT_FAILURE);
			break;
		}

	}

	// Send the configuration to the AP
	SendNewConfigurationToAp(new_configuration);

}

/*****************************/
/*****************************/
/*  VARIABLE INITIALIZATION  */
/*****************************/
/*****************************/

/*
 * InitializeVariables(): initializes all the necessary variables
 */
void Agent :: InitializeAgent() {


	num_requests = 0;

	list_of_channels = new int[num_actions_channel];
	list_of_cca_values = new double[num_actions_cca];
	list_of_tx_power_values = new double[num_actions_tx_power];
	list_of_dcb_policy = new int[num_actions_dcb_policy];

	num_actions = num_actions_channel * num_actions_cca * num_actions_tx_power * num_actions_dcb_policy;

	// Generate actions
	actions = new Action[num_actions];

//	// Specify that the learning mechanism has not been initialized (to be done for the first AP-agent interaction)
//	not_initialized = true;

}

/*
 * InitializeLearningAlgorithm(): initializes all the necessary variables of the chosen learning alg.
 */
void Agent :: InitializeLearningAlgorithm() {

	if (centralized_flag) { // Learning operation managed by the CC

		logger.info( "%s Agent %d: Learning operation managed by the CC\n", LOG_LVL5, agent_id);

	} else  { // Learning operation managed by the agent

		switch(learning_mechanism) {

			/* Multi-Armed Bandits:
			 *
			 */
			case MULTI_ARMED_BANDITS:{

				mab_agent.agent_id = agent_id;
				mab_agent.save_agent_logs = 0;
				mab_agent.print_agent_logs = 0;

				mab_agent.selected_strategy = selected_strategy;

				mab_agent.type_of_reward = type_of_reward;

				mab_agent.num_actions = num_actions;
				mab_agent.num_actions_channel = num_actions_channel;
				mab_agent.num_actions_cca = num_actions_cca;
				mab_agent.num_actions_tx_power = num_actions_tx_power;
				mab_agent.num_actions_dcb_policy = num_actions_dcb_policy;

				mab_agent.InitializeVariables();

				mab_agent.list_of_channels = list_of_channels;
				mab_agent.list_of_cca_values = list_of_cca_values;
				mab_agent.list_of_tx_power_values = list_of_tx_power_values;
				mab_agent.list_of_dcb_policy = list_of_dcb_policy;

				break;
			}

			//  TODO: provide more learning mechanisms
			// case Q_LEARNING:
			// ...

			default:{
				logger.error("ERROR: %d is not a correct learning mechanism\n", learning_mechanism);
				exit(EXIT_FAILURE);
				break;
			}

		}

	}
}

/******************************/
/******************************/
/*  PRINT/WRITE INFORMATION   */
/******************************/
/******************************/

/*
 * PrintAgentInfo(): prints Agent info
 */
void Agent :: PrintAgentInfo(){

	logger.info( "%s Agent %d info:\n", LOG_LVL3, agent_id);
	logger.info( "%s wlan_code = %s\n", LOG_LVL4, wlan_code.c_str());
	logger.info( "%s centralized_flag = %d\n", LOG_LVL4, centralized_flag);
	logger.info( "%s time_between_requests = %f\n", LOG_LVL4, time_between_requests);
	logger.info( "%s type_of_reward = %d\n", LOG_LVL4, type_of_reward);
	logger.info( "%s initial_reward = %f\n", LOG_LVL4, initial_reward);
	logger.info( "%s list_of_channels: ", LOG_LVL4);
	for (int i = 0; i < num_actions_channel; ++i) {
		logger.info( "%d  ", list_of_channels[i]);
	}
	logger.info( "\n");

	logger.info( "%s list_of_cca_values: ", LOG_LVL4);
	for (int i = 0; i < num_actions_cca; ++i) {
		logger.info( "%f pW (%f dBm)  ", list_of_cca_values[i], ConvertPower(PW_TO_DBM, list_of_cca_values[i]));
	}
	logger.info( "\n");

	logger.info( "%s list_of_tx_power_values: ", LOG_LVL4);
	for (int i = 0; i < num_actions_tx_power; ++i) {
		logger.info("%f pW (%f dBm)  ", list_of_tx_power_values[i], ConvertPower(PW_TO_DBM, list_of_tx_power_values[i]));
	}
	logger.info( "\n");

    logger.info( "%s list_of_dcb_policy: ", LOG_LVL4);
	for (int i = 0; i < num_actions_dcb_policy; ++i) {
		logger.info( "%d  ", list_of_dcb_policy[i]);
	}
	logger.info( "\n");

	logger.info( "%s learning_mechanism: %d\n", LOG_LVL4, learning_mechanism);
	logger.info( "%s selected_strategy: %d\n", LOG_LVL4, selected_strategy);

	logger.info( "\n");

}

/*
 * WriteConfiguration(): writes Agent info
 */
void Agent :: WriteConfiguration(Configuration configuration_to_write) {

	logger.debug( "%.15f;A%d;%s;%s Configuration:\n", SimTime(), agent_id, LOG_C03, LOG_LVL2);

	logger.debug( "%.15f;A%d;%s;%s selected_primary_channel = %d\n", SimTime(), agent_id, LOG_C03, LOG_LVL3,
			configuration_to_write.selected_primary_channel);
	logger.debug( "%.15f;A%d;%s;%s selected_cca = %f dBm\n", SimTime(), agent_id, LOG_C03, LOG_LVL3,
			ConvertPower(PW_TO_DBM,configuration_to_write.selected_cca));
	logger.debug( "%.15f;A%d;%s;%s selected_tx_power = %f dBm\n", SimTime(), agent_id, LOG_C03, LOG_LVL3,
			ConvertPower(PW_TO_DBM,configuration_to_write.selected_tx_power));
	logger.debug( "%.15f;A%d;%s;%s selected_dcb_policy = %d\n", SimTime(), agent_id, LOG_C03, LOG_LVL3,
			configuration_to_write.selected_dcb_policy);

}
