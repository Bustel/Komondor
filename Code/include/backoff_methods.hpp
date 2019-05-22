#ifndef BACKOFF_METHODS_HPP
#define BACKOFF_METHODS_HPP
/*
 * ComputeBackoff(): computes a new backoff
 * */
double ComputeBackoff(int pdf_backoff, int cw, int backoff_type);

/*
 * computeRemainingBackoff(): computes the remaining backoff after some even happens
 * */
double ComputeRemainingBackoff(int backoff_type, double remaining_backoff);
/*
 * HandleBackoff(): handles the backoff. It is called when backoff may be paused or resumed.
 * */
int HandleBackoff(int pause_or_resume, double **channel_power, int primary_channel, double cca,
	int packets_in_buffer);

/*
 * HandleCongestionWindow(): increase or decrease the contention window.
 **/
void HandleContentionWindow(int cw_adaptation, int increase_or_reset, int* cw_current, int cw_min,
		int *cw_stage_current, int cw_stage_max);
#endif 
