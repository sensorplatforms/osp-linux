#ifndef _OSPDAEMON_QUEUE_H_
#define _OSPDAEMON_QUEUE_H_	1

#define QUEUE_LEN	25
#include "osp-api.h"
struct queue {
	int write;
	int read;
	OSP_InputSensorData_t data[QUEUE_LEN];
};

int OSPDaemon_queue_init(struct queue *q);

/* Pointer to data is valid til the next queue op */
OSP_InputSensorData_t *OSPDaemon_queue_get(struct queue *q);
/* Pointer to data only needs to be valid during this call. */
int OSPDaemon_queue_put(struct queue *q, OSP_InputSensorData_t *d);
int OSPDaemon_queue_isempty(struct queue *q);

#endif
