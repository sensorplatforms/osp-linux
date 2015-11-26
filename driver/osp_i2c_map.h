#ifndef OSP_NIOBE_H
#define OSP_NIOBE_H

/* I2C address map */

					/* .. [76543210] */
#define OSP_WHOAMI          0x00    	/* RO */
#define OSP_VERSION0        0x01 	/* RO */
#define OSP_VERSION1	    0x02	/* RO */
#define OSP_INT_REASON      0x03    	/* RO [-------1] */
#define	OSP_INT_NONE		0x00
#define OSP_INT_DRDY		0x01
#define OSP_INT_OVER		0x02
#define OSP_DATA_LEN        0x04    	/* RO */
#define OSP_CONFIG          0x05    	/* RW [-------1] */
#define OSP_DATA_LEN_L		0x06
#define OSP_DATA_LEN_H		0x07
#define OSP_INT_LEN		0x08

/* These registers are mainly for debugging. */
#define OSP_ENABLE0         0x10    	/* RW, shadow enable */
#define OSP_ENABLE1         0x11    	/* RW, shadow enable */
#define OSP_RESULT          0x40    	/* Shadow data */
#define OSP_RESULT_END      0x7f

#define OSP_DATA_OUT        0x80
#define OSP_DATA_OUT_END    0xbf
#define OSP_DATA_IN         0x0c
#define OSP_DATA_IN_END     0xff

/* Get cause and config command registers*/
#define OSP_GET_CAUSE 0x00 /*RO to read GET Cause*/
#define OSP_SET_CONFIG 0x03 /*RW to send Setconfig command*/
#define OSP_GET_DATA 0x48 /*RO to Read sensor data*/
#define OSP_RESET_REG 0x7A /*reset register*/
/*
 * General protocol notes:
 *	1. Reads of OSP_INT_REASON has side effects. Host should
 * cache values and NOT read OSP_INT_REASON multiple times unless
 * needed.
 *	2. A read of OSP_INT_REASON will cause the rest of the
 * registers to be updated with a new set of results. 
 *	3. Writes to OSP_DATA_IN should be done in one burst. 
 * OSP will process the OSP_DATA_IN when the burst is done.
 *	4. Interrupts are LEVEL triggered.
 * 	5. Expected host protocol is:
 *		a. Wait for interrupt.
 *		b. When an interrupt happens, read OSP_INT_REASON.
 *		c. If OSP_INT_REASON has a valid reason for interrupt,
 *		process it. Otherwise, goto (a).
 *		d. Do not read OSP_INT_REASON again. Rest of the
 *		registers are held static til the next read of 
 *		OSP_INT_REASON.
 *		e. Upon completion, read OSP_INT_REASON again. Goto (c).
 *	6. Interrupt will remain asserted as long as data is pending.
 *	7. Interrupt may deassert as soon as OSP_INT_REASON is done.
 *	8. Upon driver start, host should write a 1 to OSP_CONFIG.0.
 * This is to clear any pending data.
 * 
 * Alternative non interrupt based processing:
 * 	1. Host can periodically poll the OSP_INT_REASON register.
 * Polling should be at least as frequent as data is expected to be
 * generated. It need not be regular but should not be delayed
 * longer then there is FIFO space.
 *	2. If OSP_INT_REASON shows a pending event, process
 * as in the interrupt case.
 *
 */
#endif
