#ifndef _OSP_SH_H_
#define _OSP_SH_H_

union OSP_SensorData {
	struct {
		u32 x;
		u32 y;
		u32 z;
		u64 ts;
	} xyz;
	struct {
		u32 x;
		u32 y;
		u32 z;
		u32 r;
		u64 ts;
	} quat;
};

/* Register for sensor data. Only one call back per sensor.*/
int OSP_Sensor_Register(int sensor,	/* Sensor name and space */
			int private,	
        		void (*dataready)(int sensor, int prv,
                			void *private, long long ts,
					union OSP_SensorData *sensordata),
        		void *prv	/* Private data for callback use */
			);

int OSP_Sensor_UnRegister(int sensor, int private);

int OSP_Sensor_State(int sensor, int private, int state);

#endif
