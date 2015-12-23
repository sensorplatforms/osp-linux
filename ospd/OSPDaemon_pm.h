#ifndef _OSPDAEMON_PM_H_
#define _OSPDAEMON_PM_H_

int OSPDaemon_power_init(struct OSPDaemon_SensorDescription *);
int OSPDaemon_power_process(struct OSPDaemon_SensorDescription *, int);
void OSPDaemon_power_control(struct OSPDaemon_output *out , int enable);
#endif
