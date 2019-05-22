#ifndef AGENT_METHODS_HPP
#define AGENT_METHODS_HPP


#include <performance_metrics.hpp>

/*
 * RestartPerformanceMetrics(): restarts the performance metrics being tracked by a given AP-agent pair
 **/
void RestartPerformanceMetrics(Performance *current_performance, double sim_time);

double GenerateReward(int type_of_reward, Performance performance);

#endif
