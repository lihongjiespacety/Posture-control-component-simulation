������:

1. PC->ת�Ӱ�
   pc_protocal.c                                                                   dev_xxx.c
   uarttask->pc_protocol_handle->pc_protocol_handle_data/pc_protocol_handle_cmd -> dev_xxxdata_handle
   һ��pc_protocal.c����Ҫ�޸�,����Э��ṹ���ˡ�
   �޸��������ݴ���,ֻ��Ҫ��Ӧ��dev_xxx.c�µ�dev_xxxdata_handle�������ɡ�
   
2. OBC->ת�Ӱ�->PC
              ->OBC
   cantxrxtask.c          can.c                       obc_protocal.c               dev_xxx.c                                                
   can_rx_handle     ->   can_process_frame    ->     obc_protocol_handle_data  -> dev_xxxtel_handle -> driver_uart_send
                                                                                                     -> can_tx_raw_data
   һ��obc_protocal.c����Ҫ�޸�,����Э��ṹ���ˡ�
   �޸��������ݴ���,ֻ��Ҫ��Ӧ��dev_xxx.c�µ�dev_xxxtel_handle�������ɡ�                                                                             
   

�޸�˵��:

1.����һ��CAN�豸
  1).��can.c��s_canid_au8����������һ��CANID;����can.h��gom_obc_can_idö�����������CANID,�޸�CANDEV_NUM.
  2).��app/dev�� ���������豸���һ��.c ��.h�ļ� ʵ�����������ӿں���
     int32_t dev_gpsdata_handle(uint8_t* buff, uint8_t subtype, uint8_t size)
     int32_t dev_gpsdata_get(GPS_Data_t* buff,uint8_t subtype)
     int32_t dev_gpstel_handle(uint8_t* buff, uint8_t size)
  3).��obc_protocal��obc_protocol_handle_data����������һ·case ���������Ľӿں���
     dev_gpstel_handle ����PC������������
  4).��pc_protocal��pc_protocol_handle_data����������һ·case ���������Ľӿں���
     dev_gpsdata_handle ����OBC������������
   

2. �޸�CAN�豸��ң���
   �޸�app/dev�¶�Ӧdev_xxx.c�ļ���dev_xxxtel_handle����


3. �޸�PC��ת�Ӱ�ͨѶ���ӿ�����
   ������Ҫ�޸�:��ǿ���ӿ�����
   1).�޸�dev_magtm.h �����ݽṹ�� MAGTM_Data_t,�ýṹ�����ӿ�����(����Ҫ����ĵ�)��Ӧ(ע���ӿ������ǰ����ģʽ,arm�����ṹ���Ա������С��ģʽ)
   2).�޸�dev_xxx.c dev_xxxdata_handle���� PC����ת�Ӱ���ӿ����ݽ���.
   3).�����Ӧ��OBCͨ��ת�Ӱ巵�ظ�PC������Ҳ��Ҫ�޸� ���޸���Ӧ��dev_xxxtel_handle����