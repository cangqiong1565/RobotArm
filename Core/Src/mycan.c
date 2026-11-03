//
// #include "mycan.h"
// #include <string.h>
//
// /*
//  * 初始化 FDCAN1
//  * 接收所有标准 CAN ID，拒绝扩展帧和远程帧。
//  */
// HAL_StatusTypeDef mycan_init(void)
// {
//     FDCAN_FilterTypeDef filter = {0};
//
//     filter.IdType = FDCAN_STANDARD_ID;
//     filter.FilterIndex = 0;
//     filter.FilterType = FDCAN_FILTER_MASK;
//     filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
//
//     /* 掩码为 0：接收所有标准 ID */
//     filter.FilterID1 = 0x000;
//     filter.FilterID2 = 0x000;
//
//     if (HAL_FDCAN_ConfigFilter(
//             &hfdcan1, &filter) != HAL_OK)
//     {
//         return HAL_ERROR;
//     }
//
//     /* 未匹配帧拒绝，远程帧拒绝 */
//     if (HAL_FDCAN_ConfigGlobalFilter(
//             &hfdcan1,
//             FDCAN_REJECT,
//             FDCAN_REJECT,
//             FDCAN_REJECT_REMOTE,
//             FDCAN_REJECT_REMOTE) != HAL_OK)
//     {
//         return HAL_ERROR;
//     }
//
//     /* 启动 FDCAN1 */
//     if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
//     {
//         return HAL_ERROR;
//     }
//
//     return HAL_OK;
// }
//
//
// /*
//  * 发送经典 CAN 标准数据帧
//  *
//  * id     : 11 位标准 CAN ID
//  * txdata : 发送数据
//  * len    : 数据长度，0~8
//  */
// HAL_StatusTypeDef mycan_send(
//     FDCAN_HandleTypeDef *hfdcan,
//     uint16_t id,
//     const uint8_t *txdata,
//     uint8_t len
// )
// {
//     FDCAN_TxHeaderTypeDef txheader = {0};
//
//     if (hfdcan == NULL || id > 0x7FF || len > 8)
//     {
//         return HAL_ERROR;
//     }
//
//     if (len > 0 && txdata == NULL)
//     {
//         return HAL_ERROR;
//     }
//
//     /* 检查发送 FIFO 是否还有空间 */
//     if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) == 0)
//     {
//         return HAL_BUSY;
//     }
//
//     txheader.Identifier = id;
//     txheader.IdType = FDCAN_STANDARD_ID;
//     txheader.TxFrameType = FDCAN_DATA_FRAME;
//
//     /* 经典 CAN DLC */
//     static const uint32_t dlc_table[9] =
//     {
//         FDCAN_DLC_BYTES_0,
//         FDCAN_DLC_BYTES_1,
//         FDCAN_DLC_BYTES_2,
//         FDCAN_DLC_BYTES_3,
//         FDCAN_DLC_BYTES_4,
//         FDCAN_DLC_BYTES_5,
//         FDCAN_DLC_BYTES_6,
//         FDCAN_DLC_BYTES_7,
//         FDCAN_DLC_BYTES_8
//     };
//
//     txheader.DataLength = dlc_table[len];
//
//     txheader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
//     txheader.BitRateSwitch = FDCAN_BRS_OFF;
//     txheader.FDFormat = FDCAN_CLASSIC_CAN;
//     txheader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
//     txheader.MessageMarker = 0;
//
//     /* HAL 接口要求提供数据缓冲区 */
//     uint8_t buffer[8] = {0};
//
//     if (len > 0)
//     {
//         memcpy(buffer, txdata, len);
//     }
//
//     return HAL_FDCAN_AddMessageToTxFifoQ(
//         hfdcan,
//         &txheader,
//         buffer
//     );
// }
//
//
// /*
//  * 轮询接收 FIFO0
//  *
//  * timeout : 最长等待时间，单位 ms
//  *
//  * 返回值：
//  * HAL_OK      收到数据
//  * HAL_TIMEOUT 等待超时
//  * HAL_ERROR   接收失败
//  */
// HAL_StatusTypeDef mycan_receive(
//     FDCAN_HandleTypeDef *hfdcan,
//     uint8_t *rxdata,
//     uint32_t timeout,
//     FDCAN_RxHeaderTypeDef *rxheader
// )
// {
//     if (hfdcan == NULL ||
//         rxdata == NULL ||
//         rxheader == NULL)
//     {
//         return HAL_ERROR;
//     }
//
//     uint32_t start_tick = HAL_GetTick();
//
//     while (HAL_FDCAN_GetRxFifoFillLevel(
//                hfdcan, FDCAN_RX_FIFO0) == 0U)
//     {
//         if ((HAL_GetTick() - start_tick) >= timeout)
//         {
//             return HAL_TIMEOUT;
//         }
//     }
//
//     return HAL_FDCAN_GetRxMessage(
//         hfdcan,
//         FDCAN_RX_FIFO0,
//         rxheader,
//         rxdata
//     );
// }
