#include <semaphore.h>

#include "OSPDaemon.h"
#include "OSPDaemon_driver.h"
#include "OSPDaemon_queue.h"

#define DBGOUT(x...) DBG(DEBUG_OUTDRIVER, "Queue: "x)

extern sem_t osp_sync;

static int OSPDaemon_queue_driver_setup(struct OSPDaemon_output *out, int count)
{
	int i = 0;
	int ret = 0;
	for (i = 0; i < count; i++) {
		int pfd[2];
		if (DRIVER_QUEUE != out[i].driver)
			continue;

		ret = pipe(pfd);
		if (0 != ret) {
			DBGOUT("QUEUE: Failed to create the pipe");
			return ret;
		}

		out->fd = pfd[1];
		DBGOUT("%s out->fd is %d", __func__, out->fd);
	}

	return ret;
}

static int OSPDaemon_queue_driver_send(struct OSPDaemon_output *out) 
{
	DBGOUT("Queue Driver send");
	int err = 0;

	if (0 == OSPDaemon_queue_isempty(&out->q)) {
		err = sem_post(&osp_sync);
		if (-1 == err) {
			DBGOUT("Sem post failed error - %s", strerror(errno));
			return err;
		}
	}

	return 0;
}

static struct OSPDaemon_driver QueueDriver = {
	.drvtype = DRIVER_TYPE_OUTPUT,
	.driver = DRIVER_QUEUE,
	.send = OSPDaemon_queue_driver_send,
	.setup_out = OSPDaemon_queue_driver_setup,
};

void OSPDaemon_queue_driver_init(void)
{
	DBGOUT("Queue Driver init has been called");
	OSPDaemon_driver_register(&QueueDriver);
}
