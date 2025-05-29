/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "hvcb.h"
#include "usart.h"
#include "mcb.h"
#include "gpio.h"
#include <string.h>
#include <stdio.h>

extern volatile uint8_t error_code;
volatile uint8_t can_data_updated = 0;
static uint8_t CAN1_CLK_RefCount=0;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */
  

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_12TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = ENABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  //AIRS CMD can message filter
CAN_FilterTypeDef filterCmd;
filterCmd.FilterActivation = ENABLE;
filterCmd.FilterBank = 0u;
filterCmd.FilterFIFOAssignment = CAN_FILTER_FIFO0;
filterCmd.FilterIdHigh = (HVCB_HVB_TX_VCU_CMD_FRAME_ID << 5);
filterCmd.FilterIdLow = 0x0000;
filterCmd.FilterMaskIdHigh = 0x0000;
filterCmd.FilterMaskIdLow = 0x0000;
filterCmd.FilterMode = CAN_FILTERMODE_IDLIST;
filterCmd.FilterScale = CAN_FILTERSCALE_32BIT;

HAL_CAN_ConfigFilter(&hcan1, &filterCmd);

//CAN FILTERS//

//AMS IMD error filter
CAN_FilterTypeDef filterTlbSignals;
filterTlbSignals.FilterActivation = ENABLE;
filterTlbSignals.FilterBank = 1;
filterTlbSignals.FilterFIFOAssignment = CAN_FILTER_FIFO0;
filterTlbSignals.FilterIdHigh = (MCB_TLB_BAT_SIGNALS_STATUS_FRAME_ID << 5);
filterTlbSignals.FilterIdLow = 0x0000;
filterTlbSignals.FilterMaskIdHigh = 0x0000;
filterTlbSignals.FilterMaskIdLow = 0x0000;
filterTlbSignals.FilterMode = CAN_FILTERMODE_IDLIST;
filterTlbSignals.FilterScale = CAN_FILTERSCALE_32BIT;

HAL_CAN_ConfigFilter(&hcan1, &filterTlbSignals);

//can filter for debugging
  /*CAN_FilterTypeDef filter;
  filter.FilterActivation = ENABLE;
  filter.FilterBank = 0;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = 0x0000;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  HAL_StatusTypeDef filter_status = HAL_CAN_ConfigFilter(&hcan1, &filter);
  
  if (filter_status != HAL_OK) {
      
      Error_Handler();
  } */





//notifications interrupt RX//
  
  if (HAL_CAN_ActivateNotification(&hcan1,
      CAN_IT_RX_FIFO0_MSG_PENDING |
      CAN_IT_RX_FIFO1_MSG_PENDING |
      CAN_IT_ERROR_WARNING        |
      CAN_IT_ERROR_PASSIVE        |
      CAN_IT_BUSOFF               |
      CAN_IT_LAST_ERROR_CODE      |
      CAN_IT_ERROR                |
      CAN_IT_TX_MAILBOX_EMPTY 
  ) != HAL_OK) {
      Error_Handler();
  } 

  
  HAL_StatusTypeDef start_status = HAL_CAN_Start(&hcan1);
  

  if (start_status != HAL_OK) {
      
      Error_Handler();
  } 


  /* USER CODE END CAN1_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */
  if (CAN1_CLK_RefCount++==0){
    __HAL_RCC_CAN1_CLK_ENABLE();
  }

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = R_CAN1_Pin|T_CAN1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_SetPriority(CAN1_RX1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */
  if (CAN1_CLK_RefCount > 0 && --CAN1_CLK_RefCount == 0) {
      __HAL_RCC_CAN1_CLK_DISABLE();
  }
  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, R_CAN1_Pin|T_CAN1_Pin);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_RX1_IRQn);
    HAL_NVIC_DisableIRQ(CAN1_SCE_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
uint8_t ams_error_active=0;
uint8_t imd_error_active=0;
uint8_t airs_command=0;


//wait for the CAN Tx to be ready
HAL_StatusTypeDef can_wait(CAN_HandleTypeDef *hcan, uint8_t timeout) {
    uint32_t tick = HAL_GetTick();
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcan) == 0) {
        if (HAL_GetTick() - tick > timeout)
            return HAL_TIMEOUT;
    }
    return HAL_OK;
}

// send messages on the CAN bus
HAL_StatusTypeDef can_send(CAN_HandleTypeDef *hcan, uint8_t *buffer, CAN_TxHeaderTypeDef *header, uint32_t mailbox) {
    if (can_wait(hcan, 1) != HAL_OK)
        return HAL_TIMEOUT;

    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(hcan, header, buffer, &mailbox);
    
    return status;
}





//BMS messages sent to scarrellino
void BMS_send_msg(uint32_t id) {
    uint8_t buffer_tx[8] = {0};

    CAN_TxHeaderTypeDef TxHeader = {
        .IDE = CAN_ID_STD,
        .RTR = CAN_RTR_DATA,
        .StdId = id,
        .ExtId = 0,
        .TransmitGlobalTime = DISABLE
    };

    union {
        struct hvcb_hvb_rx_v_cell_t v_cell;
        struct hvcb_hvb_rx_t_cell_t charge_temp_struct;
        struct hvcb_hvb_rx_soc_t SOC_struct;
    } msg;

    switch (id) {
        case HVCB_HVB_RX_V_CELL_FRAME_ID:{
          double v_max    = 1; //4.6
          double v_min    = 1; //3.2
          double v_mean   = 1; //3.4
          double v_max_id = 1u; //24u
          double v_min_id = 1u; //70u

          hvcb_hvb_rx_v_cell_init(&msg.v_cell);

          msg.v_cell.hvb_idx_cell_u_max = hvcb_hvb_rx_v_cell_hvb_idx_cell_u_max_encode(v_max_id);
          msg.v_cell.hvb_idx_cell_u_min = hvcb_hvb_rx_v_cell_hvb_idx_cell_u_min_encode(v_min_id);
          msg.v_cell.hvb_u_cell_max     = hvcb_hvb_rx_v_cell_hvb_u_cell_max_encode(v_max);
          msg.v_cell.hvb_u_cell_min     = hvcb_hvb_rx_v_cell_hvb_u_cell_min_encode(v_min);
          msg.v_cell.hvb_u_cell_mean    = hvcb_hvb_rx_v_cell_hvb_u_cell_mean_encode(v_mean);

          hvcb_hvb_rx_v_cell_pack((uint8_t *)&buffer_tx, &msg.v_cell, sizeof(buffer_tx));

          TxHeader.DLC   = sizeof(buffer_tx);

          break;
        }
            
        case HVCB_HVB_RX_T_CELL_FRAME_ID: {
          double charge_temp = 44; //44

          hvcb_hvb_rx_t_cell_init(&msg.charge_temp_struct);
          msg.charge_temp_struct.hvb_t_cell_max = hvcb_hvb_rx_t_cell_hvb_t_cell_max_encode(charge_temp);

          hvcb_hvb_rx_t_cell_pack((uint8_t *)&buffer_tx, &msg.charge_temp_struct, 8);

          TxHeader.DLC   = sizeof(buffer_tx);

          break;
        }
          

        /*case HVCB_HVB_RX_SOC_FRAME_ID: {
          double SOC = 0;

          hvcb_hvb_rx_soc_init(&msg.SOC_struct);
          msg.SOC_struct.hvb_r_so_c_hvb_u_cell_min = hvcb_hvb_rx_soc_hvb_r_so_c_hvb_u_cell_min_encode(SOC);

          hvcb_hvb_rx_soc_pack((uint8_t *)&buffer_tx, &msg.SOC_struct, HVCB_HVB_RX_SOC_LENGTH);

          TxHeader.DLC   = sizeof(buffer_tx);
            break;
        }
        */

        default:
            return;
    }
    
    HAL_StatusTypeDef result = can_send(&hcan1, buffer_tx, &TxHeader, CAN_TX_MAILBOX0);
    if (result != HAL_OK) {
      Error_Handler();
    }


    
}
//bms send message routine
 void BMS_SendMessageRoutine(void) {
    static volatile uint32_t routine_10ms_tim = 0U;
    if (HAL_GetTick()>= routine_10ms_tim){
      routine_10ms_tim = HAL_GetTick() + 10U;
      BMS_send_msg(HVCB_HVB_RX_V_CELL_FRAME_ID);
      BMS_send_msg(HVCB_HVB_RX_T_CELL_FRAME_ID);
    }
    
    
    //BMS_send_msg(HVCB_HVB_RX_SOC_FRAME_ID);
    
    
 }


 //RECEIVED messages from scarrellino

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    

    //make sure it's from CAN1
    if (hcan->Instance != CAN1) return;
    

    CAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];

    
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK)
      return;



    switch (rxHeader.StdId) {

      case MCB_TLB_BAT_SIGNALS_STATUS_FRAME_ID: {
        struct mcb_tlb_bat_signals_status_t msg;
        mcb_tlb_bat_signals_status_unpack(&msg, rxData, MCB_TLB_BAT_SIGNALS_STATUS_LENGTH);

        ams_error_active = mcb_tlb_bat_signals_status_ams_err_is_active_decode(msg.ams_err_is_active);
        imd_error_active = mcb_tlb_bat_signals_status_imd_err_is_active_decode(msg.imd_err_is_active);
        can_data_updated=1;
        break;
      }

      case HVCB_HVB_TX_VCU_CMD_FRAME_ID: {
        struct hvcb_hvb_tx_vcu_cmd_t cmd;
        hvcb_hvb_tx_vcu_cmd_unpack(&cmd, rxData, HVCB_HVB_TX_VCU_CMD_LENGTH);

        airs_command = hvcb_hvb_tx_vcu_cmd_vcu_b_hvb_inv_req_decode(cmd.vcu_b_hvb_inv_req);
        can_data_updated=1;
        break;
      }

      default:
        break; // ignore frame
    }
}


//toggle GPIO based on RECEIVED messages from scarrellino
void BMS_ReceiveRoutine(void) {
  //ams error: active = drive GPIO low
  HAL_GPIO_WritePin(AMS_ERR_GPIO_Port, AMS_ERR_Pin, ams_error_active ? GPIO_PIN_RESET : GPIO_PIN_SET);

  //imd error: active = drive GPIO low
    HAL_GPIO_WritePin(IMD_ERR_GPIO_Port, IMD_ERR_Pin, imd_error_active ? GPIO_PIN_RESET : GPIO_PIN_SET);

  //AIRs command: 1 = close AIRs = drive GPIO low
    HAL_GPIO_WritePin(AIR_CMND_GPIO_Port, AIR_CMND_Pin, airs_command ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

/* USER CODE END 1 */
