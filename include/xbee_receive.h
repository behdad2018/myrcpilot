/**
 * <xbee_receive.h>
 *
 * @brief       Functions for connecting to and recieving xbee messages
 *
 * @addtogroup  XbeeReceive
 * @{
 */

#ifndef __XBEE_RECEIVE__
#define __XBEE_RECEIVE__

#include <stdint.h>

/**
 * @brief       Possition and orientation data sent/received from xbee
 */
typedef struct __attribute__((packed)) xbee_packet_t
{
    uint32_t time;  ///< Unique id for the rigid body being described
    float x;        ///< x-position in the Optitrack frame
    float y;        ///< y-position in the Optitrack frame
    float z;        ///< z-position in the Optitrack frame
    float qx;       ///< qx of quaternion
    float qy;       ///< qy of quaternion
    float qz;       ///< qz of quaternion
    float qw;       ///< qw of quaternion
    // float roll;              ///< roll
    // float pitch;             ///< pitch
    // float yaw;               ///< yaw
    uint8_t trackingValid;  ///< (bool) of whether or not tracking was valid (0 or 1)
    uint16_t sm_event;      ///< event (or input) for state machine
    uint8_t deadByte;       ///< This byte doesn't do anything. It's a place holder to preserve backward compatibility (when 'trackingValid' was a 4-byte value)
} xbee_packet_t;

#define NUM_FRAMING_BYTES 4                     ///< 2 START bytes + 2 Fletcher-16 checksum bytes
#define OPTI_DATA_LENGTH sizeof(xbee_packet_t)  ///< Actual Packet Being Sent
#define OPTI_PACKET_LENGTH OPTI_DATA_LENGTH + NUM_FRAMING_BYTES

extern xbee_packet_t xbeeMsg;
extern int xbee_portID;

/**
 * @brief       Xbee initialization function
 *
 * @return      0 on success, -1 on failure
 */
int XBEE_init(const char *xbee_port);

/**
 * @brief       Read message recieved from XBee
 *
 * @return      0 on success, -1 on failure
 */
int XBEE_getData();

/**
 * @brief       Print current XBee message to stdout
 */
void XBEE_printData();

#endif /*__XBEE_RECEIVE__ */

/* @} end group XbeeReceive */