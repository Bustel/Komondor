#include <cstdio>
#include <logical_nack.hpp>
void LogicalNack::PrintNackInfo(void){
	printf("source_id = %d - packet_id = %d - loss_reason = %d - node_id_a = %d - node_id_b = %d\n",
		source_id, packet_id, loss_reason, node_id_a, node_id_b);
}

