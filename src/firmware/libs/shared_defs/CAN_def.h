/*
 * CAN_def.h
 *
 *  Created on: 22 paź 2026
 *      Author: Tomasz Pniewski
 */

#ifndef CAN_DEF_H_
#define CAN_DEF_H_

#define MAX_CAN_TX_TRIES 3
#define CAN_TX_TIMEOUT_MS 30
#define CAN_ABORT_TIMEOUT_MS 10

#define CAN_VENTS_ID 0xA
#define CAN_WATER_ID 0xB

#define CAN_VENTS_DLC FDCAN_DLC_BYTES_1
#define CAN_WATER_DLC FDCAN_DLC_BYTES_0

#endif /* CAN_DEF_H_ */
