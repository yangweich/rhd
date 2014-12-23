
#ifndef SMR_H
#define SMR_H

/*
 * For C++ compilers, use extern "C"
 */
 
#ifdef __cplusplus
extern "C" {
#endif


#include <stdint.h>
#include <stdio.h>

#define ENET_BYTES_MAX 32
#define ENET_TYPE_485     (0 << 5)
#define ENET_TYPE_232     (1 << 5)
#define ENET_TYPE_SIM     (3 << 5)
#define ENET_TYPE_CONTROL (7 << 5)
#define ENET_TYPE(p)      ((*p) & (7 << 5))
#define ENET_BYTES(p)     (((*p) & 0x1f) + 1)

enum { /* client control protocol */
  ENET_CONTROL_START,
  ENET_CONTROL_END,
  ENET_CONTROL_SEND,
};

#define SMR_IR_N 6
#define SMR_LS_N 8
#define SMR_AD_N 5

#define SMR_AD_BATTERY  0
#define SMR_AD_EXTERNAL 1

#define SMR_STATUS_POWER_ON     1
#define SMR_STATUS_BUMPER_FRONT 2
#define SMR_STATUS_BUMPER_LEFT  4
#define SMR_STATUS_BUMPER_RIGHT 8

#define SMR_FLAG_LE 1		/* left encoder */
#define SMR_FLAG_RE 2		/* right encoder */
#define SMR_FLAG_IR 4		/* proximity sensors */
#define SMR_FLAG_LS 8		/* line sensor */
#define SMR_FLAG_LV 16		/* left speed */
#define SMR_FLAG_RV 32		/* right speed */
#define SMR_FLAG_AM 64		/* async message */
#define SMR_FLAG_PW 128		/* power module message */
#define SMR_FLAG_LPS 256	/* left motor pwm and status */
#define SMR_FLAG_RPS 512	/* left motor pwm and status */

#define SMR_PORT 24901

struct smr
{
  unsigned int read_flags;	/* which sensors were read successfully */
  unsigned int wait_flags;	/* smr_read() waits for these sensor values */
  unsigned int write_flags;

  struct {
    uint16_t encoder;
    int8_t speed;
    int pwm;
    int status;
  }
  left, right;

  uint8_t ir[SMR_IR_N];
  uint8_t ls[32];

  int ad[SMR_AD_N];
  int status;

  unsigned char msg[ENET_BYTES_MAX];
  void (*recv_hook)(struct smr *);

  uint32_t tick;		/* sample number (10ms) */
  uint32_t ts;			/* time stamp, cpu cycles (2ns) */

  int enet;			/* socket filedescriptor to server */

  int reverse_right;		/* default: 1. change to 0 for
				   compatibility mode */
};

extern struct smr *smr_connect(char *hostname, int port);
extern void smr_disconnect(struct smr *);
extern int smr_read(struct smr *);
extern int smr_write(struct smr *);

#define SMR_MSG_LEFT_SPEED(x)  2,0x11,(x)
#define SMR_MSG_RIGHT_SPEED(x) 2,0x12,(x)

#define SMR_LEFT_ENCODER_RET  0xa1
#define SMR_RIGHT_ENCODER_RET 0xa2

#define SMR_LS_RET            0x17
#define SMR_IR_RET            0x88

#define SMR_POWER_RET         0x19

//Beck's gyro
#define SMR_GYRO_POS_RET      0x1A
#define SMR_GYRO_RATE_RET     0x2A
#define SMR_GYRO_TEMP_RET     0x2A

#define ENET_MSG_CONTROL_SEND (ENET_TYPE_CONTROL|1),ENET_CONTROL_SEND

extern int enet_connect(char *hostname, int port);
extern int enet_read(int fd, unsigned char *p);
extern int enet_write(int fd, unsigned char *p);
extern void enet_print(unsigned char *);
extern void enet_fprint(FILE *, unsigned char *);


/*
 * end block for C++
 */

#ifdef __cplusplus
}
#endif

#endif /* SMR_H */
