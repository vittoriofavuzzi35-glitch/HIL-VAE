/**
  ******************************************************************************
  * @file    network.h
  * @author  STEdgeAI
  * @date    2026-10-02 12:58:15
  * @brief   Minimal description of the generated c-implemention of the network
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  ******************************************************************************
  */
#ifndef LL_ATON_NETWORK_H
#define LL_ATON_NETWORK_H

/******************************************************************************/
#define LL_ATON_NETWORK_C_MODEL_NAME        "network"
#define LL_ATON_NETWORK_ORIGIN_MODEL_NAME   "vae_unito_3in_int8"

/************************** USER ALLOCATED IOs ********************************/
// No user allocated inputs
// No user allocated outputs

/************************** INPUTS ********************************************/
#define LL_ATON_NETWORK_IN_NUM        (3)    // Total number of input buffers
// Input buffer 1 -- Input_26_out_0
#define LL_ATON_NETWORK_IN_1_ALIGNMENT   (32)
#define LL_ATON_NETWORK_IN_1_SIZE_BYTES  (4536)
// Input buffer 2 -- Input_24_out_0
#define LL_ATON_NETWORK_IN_2_ALIGNMENT   (32)
#define LL_ATON_NETWORK_IN_2_SIZE_BYTES  (60)
// Input buffer 3 -- Input_1_out_0
#define LL_ATON_NETWORK_IN_3_ALIGNMENT   (32)
#define LL_ATON_NETWORK_IN_3_SIZE_BYTES  (1024)

/************************** OUTPUTS *******************************************/
#define LL_ATON_NETWORK_OUT_NUM        (1)    // Total number of output buffers
// Output buffer 1 -- Quantize_64_out_0
#define LL_ATON_NETWORK_OUT_1_ALIGNMENT   (32)
#define LL_ATON_NETWORK_OUT_1_SIZE_BYTES  (4)

#endif /* LL_ATON_NETWORK_H */
