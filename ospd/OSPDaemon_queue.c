/*
 * (C) Copyright 2015 HY Research LLC for Audience.
 * Licensed under Apache Public License
 */

/*
 * Queue management routes.
 */

#include <stdio.h>
#include <string.h>

#include "OSPDaemon.h"
#include "OSPDaemon_queue.h"
#include "osp-api.h"
int OSPDaemon_queue_init(struct queue *q)
{
	if (q == NULL)
		return -1;

	q->write = 0; q->read = 0;

	return 0;
}

/* Data valid til next enqueue op */
OSP_InputSensorData_t *OSPDaemon_queue_get(struct queue *q)
{
	OSP_InputSensorData_t *d;

	if (q == NULL) return NULL;

	if (q->write == q->read) return NULL;
	d = &(q->data[q->read]);
	q->read++;
	q->read %= QUEUE_LEN;
	return d;
}
/* d can be destory after this call */
int OSPDaemon_queue_put(struct queue *q, OSP_InputSensorData_t *d)
{
	if ((q->write+1)%QUEUE_LEN == q->read) return -1;
	memcpy(&(q->data[q->write]), d, sizeof(OSP_InputSensorData_t));
	q->write++;
	q->write %= QUEUE_LEN;

	return 0;
}


/* Checks a queue for emptyness.
 *  1 - empty
 *  0 - there is data.
 */
int OSPDaemon_queue_isempty(struct queue *q)
{
	return (q->write == q->read)?1:0;
}
