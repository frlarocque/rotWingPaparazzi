/** @file
 *  @brief PPRZLink message header for AIRSPEED_WIND_ESTIMATOR_EKF in class telemetry
 *
 *  
 *  @see http://paparazziuav.org
 */

#ifndef _VAR_MESSAGES_telemetry_AIRSPEED_WIND_ESTIMATOR_EKF_H_
#define _VAR_MESSAGES_telemetry_AIRSPEED_WIND_ESTIMATOR_EKF_H_


#include "pprzlink/pprzlink_device.h"
#include "pprzlink/pprzlink_transport.h"
#include "pprzlink/pprzlink_utils.h"
#include "pprzlink/pprzlink_message.h"


#ifdef __cplusplus
extern "C" {
#endif

#if DOWNLINK

#define DL_AIRSPEED_WIND_ESTIMATOR_EKF 199
#define PPRZ_MSG_ID_AIRSPEED_WIND_ESTIMATOR_EKF 199

/**
 * Macro that redirect calls to the default version of pprzlink API
 * Used for compatibility between versions.
 */
#define pprzlink_msg_send_AIRSPEED_WIND_ESTIMATOR_EKF _send_msg(AIRSPEED_WIND_ESTIMATOR_EKF,PPRZLINK_DEFAULT_VER)

/**
 * Sends a AIRSPEED_WIND_ESTIMATOR_EKF message (API V2.0 version)
 *
 * @param msg the pprzlink_msg structure for this message
 * @param _u_est 
 * @param _v_est 
 * @param _w_est 
 * @param _mu_N 
 * @param _mu_E 
 * @param _mu_D 
 * @param _offset_x 
 * @param _offset_y 
 * @param _offset_z 
 * @param _debug_1 
 * @param _debug_2 
 * @param _debug_3 
 */
static inline void pprzlink_msg_v2_send_AIRSPEED_WIND_ESTIMATOR_EKF(struct pprzlink_msg * msg, float *_u_est, float *_v_est, float *_w_est, float *_mu_N, float *_mu_E, float *_mu_D, float *_offset_x, float *_offset_y, float *_offset_z, float *_debug_1, float *_debug_2, float *_debug_3) {
#if PPRZLINK_ENABLE_FD
  long _FD = 0; /* can be an address, an index, a file descriptor, ... */
#endif
  const uint8_t size = msg->trans->size_of(msg, /* msg header overhead */4+4+4+4+4+4+4+4+4+4+4+4+4);
  if (msg->trans->check_available_space(msg, _FD_ADDR, size)) {
    msg->trans->count_bytes(msg, size);
    msg->trans->start_message(msg, _FD, /* msg header overhead */4+4+4+4+4+4+4+4+4+4+4+4+4);
    msg->trans->put_bytes(msg, _FD, DL_TYPE_UINT8, DL_FORMAT_SCALAR, &(msg->sender_id), 1);
    msg->trans->put_named_byte(msg, _FD, DL_TYPE_UINT8, DL_FORMAT_SCALAR, msg->receiver_id, NULL);
    uint8_t comp_class = (msg->component_id & 0x0F) << 4 | (1 & 0x0F);
    msg->trans->put_named_byte(msg, _FD, DL_TYPE_UINT8, DL_FORMAT_SCALAR, comp_class, NULL);
    msg->trans->put_named_byte(msg, _FD, DL_TYPE_UINT8, DL_FORMAT_SCALAR, DL_AIRSPEED_WIND_ESTIMATOR_EKF, "AIRSPEED_WIND_ESTIMATOR_EKF");
    msg->trans->put_bytes(msg, _FD, DL_TYPE_FLOAT, DL_FORMAT_SCALAR, (void *) _u_est, 4);
    msg->trans->put_bytes(msg, _FD, DL_TYPE_FLOAT, DL_FORMAT_SCALAR, (void *) _v_est, 4);
    msg->trans->put_bytes(msg, _FD, DL_TYPE_FLOAT, DL_FORMAT_SCALAR, (void *) _w_est, 4);
    msg->trans->put_bytes(msg, _FD, DL_TYPE_FLOAT, DL_FORMAT_SCALAR, (void *) _mu_N, 4);
    msg->trans->put_bytes(msg, _FD, DL_TYPE_FLOAT, DL_FORMAT_SCALAR, (void *) _mu_E, 4);
    msg->trans->put_bytes(msg, _FD, DL_TYPE_FLOAT, DL_FORMAT_SCALAR, (void *) _mu_D, 4);
    msg->trans->put_bytes(msg, _FD, DL_TYPE_FLOAT, DL_FORMAT_SCALAR, (void *) _offset_x, 4);
    msg->trans->put_bytes(msg, _FD, DL_TYPE_FLOAT, DL_FORMAT_SCALAR, (void *) _offset_y, 4);
    msg->trans->put_bytes(msg, _FD, DL_TYPE_FLOAT, DL_FORMAT_SCALAR, (void *) _offset_z, 4);
    msg->trans->put_bytes(msg, _FD, DL_TYPE_FLOAT, DL_FORMAT_SCALAR, (void *) _debug_1, 4);
    msg->trans->put_bytes(msg, _FD, DL_TYPE_FLOAT, DL_FORMAT_SCALAR, (void *) _debug_2, 4);
    msg->trans->put_bytes(msg, _FD, DL_TYPE_FLOAT, DL_FORMAT_SCALAR, (void *) _debug_3, 4);
    msg->trans->end_message(msg, _FD);
  } else
        msg->trans->overrun(msg);
}

// Compatibility with the protocol v1.0 API
#define pprzlink_msg_v1_send_AIRSPEED_WIND_ESTIMATOR_EKF pprz_msg_send_AIRSPEED_WIND_ESTIMATOR_EKF
#define DOWNLINK_SEND_AIRSPEED_WIND_ESTIMATOR_EKF(_trans, _dev, u_est, v_est, w_est, mu_N, mu_E, mu_D, offset_x, offset_y, offset_z, debug_1, debug_2, debug_3) pprz_msg_send_AIRSPEED_WIND_ESTIMATOR_EKF(&((_trans).trans_tx), &((_dev).device), AC_ID, u_est, v_est, w_est, mu_N, mu_E, mu_D, offset_x, offset_y, offset_z, debug_1, debug_2, debug_3)
/**
 * Sends a AIRSPEED_WIND_ESTIMATOR_EKF message (API V1.0 version)
 *
 * @param trans A pointer to the transport_tx structure used for sending the message
 * @param dev A pointer to the link_device structure through which the message will be sent
 * @param ac_id The id of the sender of the message
 * @param _u_est 
 * @param _v_est 
 * @param _w_est 
 * @param _mu_N 
 * @param _mu_E 
 * @param _mu_D 
 * @param _offset_x 
 * @param _offset_y 
 * @param _offset_z 
 * @param _debug_1 
 * @param _debug_2 
 * @param _debug_3 
 */
static inline void pprz_msg_send_AIRSPEED_WIND_ESTIMATOR_EKF(struct transport_tx *trans, struct link_device *dev, uint8_t ac_id, float *_u_est, float *_v_est, float *_w_est, float *_mu_N, float *_mu_E, float *_mu_D, float *_offset_x, float *_offset_y, float *_offset_z, float *_debug_1, float *_debug_2, float *_debug_3) {
    struct pprzlink_msg msg;
    msg.trans = trans;
    msg.dev = dev;
    msg.sender_id = ac_id;
    msg.receiver_id = 0;
    msg.component_id = 0;
    pprzlink_msg_v2_send_AIRSPEED_WIND_ESTIMATOR_EKF(&msg,_u_est,_v_est,_w_est,_mu_N,_mu_E,_mu_D,_offset_x,_offset_y,_offset_z,_debug_1,_debug_2,_debug_3);
}


#else // DOWNLINK

#define DOWNLINK_SEND_AIRSPEED_WIND_ESTIMATOR_EKF(_trans, _dev, u_est, v_est, w_est, mu_N, mu_E, mu_D, offset_x, offset_y, offset_z, debug_1, debug_2, debug_3) {}
static inline void pprz_send_msg_AIRSPEED_WIND_ESTIMATOR_EKF(struct transport_tx *trans __attribute__((unused)), struct link_device *dev __attribute__((unused)), uint8_t ac_id __attribute__((unused)), float *_u_est __attribute__((unused)), float *_v_est __attribute__((unused)), float *_w_est __attribute__((unused)), float *_mu_N __attribute__((unused)), float *_mu_E __attribute__((unused)), float *_mu_D __attribute__((unused)), float *_offset_x __attribute__((unused)), float *_offset_y __attribute__((unused)), float *_offset_z __attribute__((unused)), float *_debug_1 __attribute__((unused)), float *_debug_2 __attribute__((unused)), float *_debug_3 __attribute__((unused))) {}

#endif // DOWNLINK


/** Getter for field u_est in message AIRSPEED_WIND_ESTIMATOR_EKF
  *
  * @param _payload : a pointer to the AIRSPEED_WIND_ESTIMATOR_EKF message
  * @return 
  */
static inline float pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_u_est(uint8_t * _payload __attribute__((unused)))
{
    return _PPRZ_VAL_float(_payload, 4);
}


/** Getter for field v_est in message AIRSPEED_WIND_ESTIMATOR_EKF
  *
  * @param _payload : a pointer to the AIRSPEED_WIND_ESTIMATOR_EKF message
  * @return 
  */
static inline float pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_v_est(uint8_t * _payload __attribute__((unused)))
{
    return _PPRZ_VAL_float(_payload, 8);
}


/** Getter for field w_est in message AIRSPEED_WIND_ESTIMATOR_EKF
  *
  * @param _payload : a pointer to the AIRSPEED_WIND_ESTIMATOR_EKF message
  * @return 
  */
static inline float pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_w_est(uint8_t * _payload __attribute__((unused)))
{
    return _PPRZ_VAL_float(_payload, 12);
}


/** Getter for field mu_N in message AIRSPEED_WIND_ESTIMATOR_EKF
  *
  * @param _payload : a pointer to the AIRSPEED_WIND_ESTIMATOR_EKF message
  * @return 
  */
static inline float pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_mu_N(uint8_t * _payload __attribute__((unused)))
{
    return _PPRZ_VAL_float(_payload, 16);
}


/** Getter for field mu_E in message AIRSPEED_WIND_ESTIMATOR_EKF
  *
  * @param _payload : a pointer to the AIRSPEED_WIND_ESTIMATOR_EKF message
  * @return 
  */
static inline float pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_mu_E(uint8_t * _payload __attribute__((unused)))
{
    return _PPRZ_VAL_float(_payload, 20);
}


/** Getter for field mu_D in message AIRSPEED_WIND_ESTIMATOR_EKF
  *
  * @param _payload : a pointer to the AIRSPEED_WIND_ESTIMATOR_EKF message
  * @return 
  */
static inline float pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_mu_D(uint8_t * _payload __attribute__((unused)))
{
    return _PPRZ_VAL_float(_payload, 24);
}


/** Getter for field offset_x in message AIRSPEED_WIND_ESTIMATOR_EKF
  *
  * @param _payload : a pointer to the AIRSPEED_WIND_ESTIMATOR_EKF message
  * @return 
  */
static inline float pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_offset_x(uint8_t * _payload __attribute__((unused)))
{
    return _PPRZ_VAL_float(_payload, 28);
}


/** Getter for field offset_y in message AIRSPEED_WIND_ESTIMATOR_EKF
  *
  * @param _payload : a pointer to the AIRSPEED_WIND_ESTIMATOR_EKF message
  * @return 
  */
static inline float pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_offset_y(uint8_t * _payload __attribute__((unused)))
{
    return _PPRZ_VAL_float(_payload, 32);
}


/** Getter for field offset_z in message AIRSPEED_WIND_ESTIMATOR_EKF
  *
  * @param _payload : a pointer to the AIRSPEED_WIND_ESTIMATOR_EKF message
  * @return 
  */
static inline float pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_offset_z(uint8_t * _payload __attribute__((unused)))
{
    return _PPRZ_VAL_float(_payload, 36);
}


/** Getter for field debug_1 in message AIRSPEED_WIND_ESTIMATOR_EKF
  *
  * @param _payload : a pointer to the AIRSPEED_WIND_ESTIMATOR_EKF message
  * @return 
  */
static inline float pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_debug_1(uint8_t * _payload __attribute__((unused)))
{
    return _PPRZ_VAL_float(_payload, 40);
}


/** Getter for field debug_2 in message AIRSPEED_WIND_ESTIMATOR_EKF
  *
  * @param _payload : a pointer to the AIRSPEED_WIND_ESTIMATOR_EKF message
  * @return 
  */
static inline float pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_debug_2(uint8_t * _payload __attribute__((unused)))
{
    return _PPRZ_VAL_float(_payload, 44);
}


/** Getter for field debug_3 in message AIRSPEED_WIND_ESTIMATOR_EKF
  *
  * @param _payload : a pointer to the AIRSPEED_WIND_ESTIMATOR_EKF message
  * @return 
  */
static inline float pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_debug_3(uint8_t * _payload __attribute__((unused)))
{
    return _PPRZ_VAL_float(_payload, 48);
}


/* Compatibility macros */
#define DL_AIRSPEED_WIND_ESTIMATOR_EKF_u_est(_payload) pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_u_est(_payload)
#define DL_AIRSPEED_WIND_ESTIMATOR_EKF_v_est(_payload) pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_v_est(_payload)
#define DL_AIRSPEED_WIND_ESTIMATOR_EKF_w_est(_payload) pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_w_est(_payload)
#define DL_AIRSPEED_WIND_ESTIMATOR_EKF_mu_N(_payload) pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_mu_N(_payload)
#define DL_AIRSPEED_WIND_ESTIMATOR_EKF_mu_E(_payload) pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_mu_E(_payload)
#define DL_AIRSPEED_WIND_ESTIMATOR_EKF_mu_D(_payload) pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_mu_D(_payload)
#define DL_AIRSPEED_WIND_ESTIMATOR_EKF_offset_x(_payload) pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_offset_x(_payload)
#define DL_AIRSPEED_WIND_ESTIMATOR_EKF_offset_y(_payload) pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_offset_y(_payload)
#define DL_AIRSPEED_WIND_ESTIMATOR_EKF_offset_z(_payload) pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_offset_z(_payload)
#define DL_AIRSPEED_WIND_ESTIMATOR_EKF_debug_1(_payload) pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_debug_1(_payload)
#define DL_AIRSPEED_WIND_ESTIMATOR_EKF_debug_2(_payload) pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_debug_2(_payload)
#define DL_AIRSPEED_WIND_ESTIMATOR_EKF_debug_3(_payload) pprzlink_get_DL_AIRSPEED_WIND_ESTIMATOR_EKF_debug_3(_payload)



#ifdef __cplusplus
}
#endif // __cplusplus

#endif // _VAR_MESSAGES_telemetry_AIRSPEED_WIND_ESTIMATOR_EKF_H_

