#ifndef _OSPDAEMON_DRIVER_H_
#define _OSPDAEMON_DRIVER_H_

struct OSPDaemon_driver {
    int drvtype;
    int driver;
    int (*setup_in)(struct OSPDaemon_SensorDetail *, int count);
    int (*setup_out)(struct OSPDaemon_output *, int count);
    int (*send)(struct OSPDaemon_output *);
    int (*read)(struct OSPDaemon_SensorDetail *s);
    int (*enable_out)(struct OSPDaemon_output *);
    int (*enable_in)(struct OSPDaemon_SensorDetail *);
    int (*disable_out)(struct OSPDaemon_output *);
    int (*disable_in)(struct OSPDaemon_SensorDetail *);
};

int OSPDaemon_driver_setup_out(struct OSPDaemon_output *s, int count);
int OSPDaemon_driver_setup_in(struct OSPDaemon_SensorDetail *s, int count);
int OSPDaemon_driver_send(struct OSPDaemon_output *);
int OSPDaemon_driver_read(struct OSPDaemon_SensorDetail *);
int OSPDaemon_driver_register(struct OSPDaemon_driver *);
int OSPDaemon_driver_enable_out(struct OSPDaemon_output *s);
int OSPDaemon_driver_disable_out(struct OSPDaemon_output *s);
int OSPDaemon_driver_enable_in(struct OSPDaemon_SensorDetail *s);
int OSPDaemon_driver_disable_in(struct OSPDaemon_SensorDetail *s);
#endif
