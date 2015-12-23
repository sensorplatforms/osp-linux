#ifndef _OSPDAEMON_QUEUE_H_
#define _OSPDAEMON_QUEUE_H_	1

#define QUEUE_LEN	1024
#include "osp-api.h"
struct queue {
	int write;
	int read;
	OSP_InputSensorData_t data[QUEUE_LEN];
};

int OSPDaemon_queue_init(struct queue *q);

/*
 * Get the data from the queue
 * On Success -
 * 0 is returned and the data is copied to the structure passed  
 * On Failure -
 * If the queue is empty or on error, -1 is returned and the structure sent 
 * is not modified.
 */
int OSPDaemon_queue_get(struct queue *q, OSP_InputSensorData_t *out_data);
/* Pointer to data only needs to be valid during this call. */
int OSPDaemon_queue_put(struct queue *q, OSP_InputSensorData_t *d);
int OSPDaemon_queue_isempty(struct queue *q);
int OSPDaemon_queue_clear(struct queue *q);

#endif
