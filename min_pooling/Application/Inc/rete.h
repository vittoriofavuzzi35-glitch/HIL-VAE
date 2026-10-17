/**
  ******************************************************************************
  * @file    rete.h
  * @author  STEdgeAI
  * @date    2026-10-16 17:16:26
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
#ifndef LL_ATON_RETE_H
#define LL_ATON_RETE_H

/******************************************************************************/
#define LL_ATON_RETE_C_MODEL_NAME        "rete"
#define LL_ATON_RETE_ORIGIN_MODEL_NAME   "rete_di_bordo"

/************************** USER ALLOCATED IOs ********************************/
// No user allocated inputs
// No user allocated outputs

/************************** INPUTS ********************************************/
#define LL_ATON_RETE_IN_NUM        (2)    // Total number of input buffers
// Input buffer 1 -- Input_23_out_0
#define LL_ATON_RETE_IN_1_ALIGNMENT   (32)
#define LL_ATON_RETE_IN_1_SIZE_BYTES  (60)
// Input buffer 2 -- Input_0_out_0
#define LL_ATON_RETE_IN_2_ALIGNMENT   (32)
#define LL_ATON_RETE_IN_2_SIZE_BYTES  (1024)

/************************** OUTPUTS *******************************************/
#define LL_ATON_RETE_OUT_NUM        (1)    // Total number of output buffers
// Output buffer 1 -- Quantize_39_out_0
#define LL_ATON_RETE_OUT_1_ALIGNMENT   (32)
#define LL_ATON_RETE_OUT_1_SIZE_BYTES  (4)

#endif /* LL_ATON_RETE_H */
