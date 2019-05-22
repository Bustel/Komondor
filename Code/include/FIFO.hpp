#ifndef FIFO_HPP
#define FIFO_HPP
#include "notification.hpp"
#include <deque>
/*
	FIFO Class
*/

struct FIFO
{	
	std::deque <Notification> m_queue;
	int queue_size;
	Notification GetFirstPacket();
	Notification GetPacketAt(int n);
	void DelFirstPacket();		
	void DeletePacketIn(int i);
	void PutPacket(Notification &packet);
	void PutPacketFront(Notification &packet);
	void PutPacketIn(Notification &packet, int);
	int QueueSize();
};

#endif
