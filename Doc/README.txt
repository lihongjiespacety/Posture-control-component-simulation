数据流:

1. PC->转接板
   pc_protocal.c                                                                   dev_xxx.c
   uarttask->pc_protocol_handle->pc_protocol_handle_data/pc_protocol_handle_cmd -> dev_xxxdata_handle
   一般pc_protocal.c不需要修改,除非协议结构变了。
   修改数据内容处理,只需要对应的dev_xxx.c下的dev_xxxdata_handle函数即可。
   
2. OBC->转接板->PC
              ->OBC
   cantxrxtask.c          can.c                       obc_protocal.c               dev_xxx.c                                                
   can_rx_handle     ->   can_process_frame    ->     obc_protocol_handle_data  -> dev_xxxtel_handle -> driver_uart_send
                                                                                                     -> can_tx_raw_data
   一般obc_protocal.c不需要修改,除非协议结构变了。
   修改数据内容处理,只需要对应的dev_xxx.c下的dev_xxxtel_handle函数即可。                                                                             
   

修改说明:

1.增加一个CAN设备
  1).在can.c中s_canid_au8数组中增加一个CANID;并在can.h中gom_obc_can_id枚举添加新增的CANID,修改CANDEV_NUM.
  2).在app/dev下 仿照其他设备添加一个.c 和.h文件 实现以下三个接口函数
     int32_t dev_gpsdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
     int32_t dev_gpsdata_get(GPS_Data_t* buff,uint8_t subtype)
     int32_t dev_gpstel_handle(uint8_t* buff, uint8_t size)
  3).在obc_protocal中obc_protocol_handle_data函数中新增一路case 调用上述的接口函数
     dev_gpstel_handle 处理PC发过来的数据
  4).在pc_protocal中pc_protocol_handle_data函数中新增一路case 调用上述的接口函数
     dev_gpsdata_handle 处理OBC发过来的数据
   

2. 修改CAN设备的遥测包
   修改app/dev下对应dev_xxx.c文件的dev_xxxtel_handle函数


3. 修改PC与转接板通讯的子块内容
   比如需要修改:磁强计子块内容
   1).修改dev_magtm.h 的数据结构体 MAGTM_Data_t,该结构体与子块内容(见概要设计文档)对应(注意子块内容是按大端模式,arm环境结构体成员内容是小端模式)
   2).修改dev_xxx.c dev_xxxdata_handle函数 PC发给转接板的子块内容解析.
   3).如果对应的OBC通过转接板返回给PC的数据也需要修改 则修改相应的dev_xxxtel_handle函数