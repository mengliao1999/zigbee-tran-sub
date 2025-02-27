#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_key.h"
#include "MT_UART.h"
#include "MT_APP.h"
#include "MT.h"

#include "DHT11.h" //添加温度传感器头文件

/*********************************************************************
 * MACROS
 */

/*********************************************************************
 * CONSTANTS
 */

/*********************************************************************
 * TYPEDEFS
 */

/*********************************************************************
 * GLOBAL VARIABLES
 */

// This list should be filled with Application specific Cluster IDs.
const cId_t SampleApp_ClusterList[SAMPLEAPP_MAX_CLUSTERS] =
{
  SAMPLEAPP_PERIODIC_CLUSTERID,
  SAMPLEAPP_FLASH_CLUSTERID
};

const SimpleDescriptionFormat_t SampleApp_SimpleDesc =
{
  SAMPLEAPP_ENDPOINT,              //  int Endpoint;
  SAMPLEAPP_PROFID,                //  uint16 AppProfId[2];
  SAMPLEAPP_DEVICEID,              //  uint16 AppDeviceId[2];
  SAMPLEAPP_DEVICE_VERSION,        //  int   AppDevVer:4;
  SAMPLEAPP_FLAGS,                 //  int   AppFlags:4;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList,  //  uint8 *pAppInClusterList;
  SAMPLEAPP_MAX_CLUSTERS,          //  uint8  AppNumInClusters;
  (cId_t *)SampleApp_ClusterList   //  uint8 *pAppInClusterList;
};

// This is the Endpoint/Interface description.  It is defined here, but
// filled-in in SampleApp_Init().  Another way to go would be to fill
// in the structure here and make it a "const" (in code space).  The
// way it's defined in this sample app it is define in RAM.
endPointDesc_t SampleApp_epDesc;

/*********************************************************************
 * EXTERNAL VARIABLES
 */

/*********************************************************************
 * EXTERNAL FUNCTIONS
 */

/*********************************************************************
 * LOCAL VARIABLES
 */
uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr;
afAddrType_t SampleApp_Flash_DstAddr;

afAddrType_t Point_To_Point_DstAddr;//网蜂点对点通信定义

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0;
uint8 SampleAppFlashCounter = 0;

/*********************************************************************
 * LOCAL FUNCTIONS
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendPointToPointMessage(void); //网蜂点对点通讯定义
void SampleApp_SendFlashMessage( uint16 flashTime );
void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg);

/*********************************************************************
 * NETWORK LAYER CALLBACKS
 */

/*********************************************************************
 * PUBLIC FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_Init
 *
 * @brief   Initialization function for the Generic App Task.
 *          This is called during initialization and should contain
 *          any application specific initialization (ie. hardware
 *          initialization/setup, table initialization, power up
 *          notificaiton ... ).
 *
 * @param   task_id - the ID assigned by OSAL.  This ID should be
 *                    used to send messages and set timers.
 *
 * @return  none
 */
void SampleApp_Init( uint8 task_id )
{
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  
 /***********串口初始化************/
  MT_UartInit();//初始化
  MT_UartRegisterTaskID(task_id);//登记任务号
  HalUARTWrite(0,"Hello World\n",12);
  
 // 温度传感器初始化  
  P0SEL &= 0xbf;         //DS18B20的io口初始化

  // Device hardware initialization can be added here or in main() (Zmain.c).
  // If the hardware is application specific - add it here.
  // If the hardware is other parts of the device add it in main().

 #if defined ( BUILD_ALL_DEVICES )
  // The "Demo" target is setup to have BUILD_ALL_DEVICES and HOLD_AUTO_START
  // We are looking at a jumper (defined in SampleAppHw.c) to be jumpered
  // together - if they are - we will start up a coordinator. Otherwise,
  // the device will start as a router.
  if ( readCoordinatorJumper() )
    zgDeviceLogicalType = ZG_DEVICETYPE_COORDINATOR;
  else
    zgDeviceLogicalType = ZG_DEVICETYPE_ROUTER;
#endif // BUILD_ALL_DEVICES

#if defined ( HOLD_AUTO_START )
  // HOLD_AUTO_START is a compile option that will surpress ZDApp
  //  from starting the device and wait for the application to
  //  start the device.
  ZDOInitDevice(0);
#endif

  // Setup for the periodic message's destination address
  // Broadcast to everyone
  SampleApp_Periodic_DstAddr.addrMode = (afAddrMode_t)AddrBroadcast;
  SampleApp_Periodic_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Periodic_DstAddr.addr.shortAddr = 0xFFFF;

  // Setup for the flash command's destination address - Group 1
  SampleApp_Flash_DstAddr.addrMode = (afAddrMode_t)afAddrGroup;
  SampleApp_Flash_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_Flash_DstAddr.addr.shortAddr = SAMPLEAPP_FLASH_GROUP;
  
    // 网蜂点对点通讯定义
  Point_To_Point_DstAddr.addrMode = (afAddrMode_t)Addr16Bit; //点播
  Point_To_Point_DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
  Point_To_Point_DstAddr.addr.shortAddr = 0x0000;//发给协调器
  

  // Fill out the endpoint description.
  SampleApp_epDesc.endPoint = SAMPLEAPP_ENDPOINT;
  SampleApp_epDesc.task_id = &SampleApp_TaskID;
  SampleApp_epDesc.simpleDesc
            = (SimpleDescriptionFormat_t *)&SampleApp_SimpleDesc;
  SampleApp_epDesc.latencyReq = noLatencyReqs;

  // Register the endpoint description with the AF
  afRegister( &SampleApp_epDesc );

  // Register for all key events - This app will handle all key events
  RegisterForKeys( SampleApp_TaskID );

  // By default, all devices start out in Group 1
  SampleApp_Group.ID = 0x0001;
  osal_memcpy( SampleApp_Group.name, "Group 1", 7  );
  aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );

#if defined ( LCD_SUPPORTED )
  HalLcdWriteString( "SampleApp", HAL_LCD_LINE_1 );
#endif
}

/*********************************************************************
 * @fn      SampleApp_ProcessEvent
 *
 * @brief   Generic Application Task event processor.  This function
 *          is called to process all events for the task.  Events
 *          include timers, messages and any other user defined events.
 *
 * @param   task_id  - The OSAL assigned task ID.
 * @param   events - events to process.  This is a bit map and can
 *                   contain more than one event.
 *
 * @return  none
 */
uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {
        
        case CMD_SERIAL_MSG:  //串口收到数据后由MT_UART层传递过来的数据，编译时不定义MT_TASK，则由MT_UART层直接传递到此应用层
       // 如果是由MT_UART层传过来的数据，则上述例子中29 00 14 31都是普通数据，串口控制时候用的。   
        SampleApp_SerialCMD((mtOSALSerialData_t *)MSGpkt);
        break;
        
        // Received when a key is pressed
        case KEY_CHANGE:
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;
        
        // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if ( //(SampleApp_NwkState == DEV_ZB_COORD)协调器不能给自己点播
                (SampleApp_NwkState == DEV_ROUTER)
             || (SampleApp_NwkState == DEV_END_DEVICE) )
          {
            // Start sending the periodic message in a regular interval.
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
          }
          else
          {
            // Device is no longer in the network
          }
          break;

        default:
          break;
      }

      // Release the memory
      osal_msg_deallocate( (uint8 *)MSGpkt );

      // Next - if one is available
      MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    }

    // return unprocessed events
    return (events ^ SYS_EVENT_MSG);
  }

  // Send a message out - This event is generated by a timer
  //  (setup in SampleApp_Init()).
  if ( events & SAMPLEAPP_SEND_PERIODIC_MSG_EVT )
  {
    uint8 T[8];   //温度+提示符     
    DHT11_TEST();   //温度检测    
    T[0]=wendu_shi+48;
    T[1]=wendu_ge+48;
    T[2]=' ';
    T[3]=shidu_shi+48;
    T[4]=shidu_ge+48;
    T[5]=' ';
    T[6]=' ';
    T[7]=' ';
    /*******串口打印 WEBEE*********/
    HalUARTWrite(0,"temp=",5);
    HalUARTWrite(0,T,2);
    HalUARTWrite(0,"\n",1);
    
    HalUARTWrite(0,"humidity=",9);
    HalUARTWrite(0,T+3,2);
    HalUARTWrite(0,"\n",1);
    
    /*******LCD显示  WEBEE*********/
    HalLcdWriteString("Temp: humidity:", HAL_LCD_LINE_3 );//LCD显示
    HalLcdWriteString( T, HAL_LCD_LINE_4 );//LCD显示    
    //替换成点对点通讯的程序
    SampleApp_SendPointToPointMessage();
    
    // Setup to send message again in normal period (+ a little jitter)
   
    osal_start_timerEx( SampleApp_TaskID, SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
        (SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT + (osal_rand() & 0x00FF)) );

    // return unprocessed events
    return (events ^ SAMPLEAPP_SEND_PERIODIC_MSG_EVT);
  }

  // Discard unknown events
  return 0;
}

/*********************************************************************
 * Event Generation Functions
 */
/*********************************************************************
 * @fn      SampleApp_HandleKeys
 *
 * @brief   Handles all key events for this device.
 *
 * @param   shift - true if in shift/alt.
 * @param   keys - bit field for key events. Valid entries:
 *                 HAL_KEY_SW_2
 *                 HAL_KEY_SW_1
 *
 * @return  none
 */
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
  (void)shift;  // Intentionally unreferenced parameter
  
  if ( keys & HAL_KEY_SW_1 )
  {
    /* This key sends the Flash Command is sent to Group 1.
     * This device will not receive the Flash Command from this
     * device (even if it belongs to group 1).
     */
    SampleApp_SendFlashMessage( SAMPLEAPP_FLASH_DURATION );
  }

  if ( keys & HAL_KEY_SW_2 )
  {
    /* The Flashr Command is sent to Group 1.
     * This key toggles this device in and out of group 1.
     * If this device doesn't belong to group 1, this application
     * will not receive the Flash command sent to group 1.
     */
    aps_Group_t *grp;
    grp = aps_FindGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    if ( grp )
    {
      // Remove from the group
      aps_RemoveGroup( SAMPLEAPP_ENDPOINT, SAMPLEAPP_FLASH_GROUP );
    }
    else
    {
      // Add to the flash group
      aps_AddGroup( SAMPLEAPP_ENDPOINT, &SampleApp_Group );
    }
  }
}

/*********************************************************************
 * LOCAL FUNCTIONS
 */

/*********************************************************************
 * @fn      SampleApp_MessageMSGCB
 *
 * @brief   Data message processor callback.  This function processes
 *          any incoming data - probably from other devices.  So, based
 *          on cluster ID, perform the intended action.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt )
{
  uint16 flashTime;
  switch ( pkt->clusterId )
  {
    case SAMPLEAPP_POINT_TO_POINT_CLUSTERID:
      /***********温度打印***************/
      HalUARTWrite(0,"Temp is:",8);        //提示接收到数据
      HalUARTWrite(0,&pkt->cmd.Data[0],2); //温度
      HalUARTWrite(0,"\n",1);              // 回车换行
      
     /***************湿度打印****************/
      HalUARTWrite(0,"Humidity is:",12);    //提示接收到数据
      HalUARTWrite(0,&pkt->cmd.Data[2],2); //湿度
      HalUARTWrite(0,"\n",1);              // 回车换行      
      break;

    case SAMPLEAPP_FLASH_CLUSTERID:
      flashTime = BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2] );
      HalLedBlink( HAL_LED_4, 4, 50, (flashTime / 4) );
      break;
  }
}

/*********************************************************************
 * @fn      SampleApp_SendPeriodicMessage
 *
 * @brief   Send the periodic message.
 *
 * @param   none
 *
 * @return  none
 */
void SampleApp_SendPeriodicMessage( void )
{
  uint8 data[10]={0,1,2,3,4,5,6,7,8,9};
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       10,
                       data,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

void SampleApp_SendPointToPointMessage( void )
{
  uint8 T_H[4];//温湿度
  T_H[0]=wendu_shi+48;
  T_H[1]=wendu_ge%10+48;
  
  T_H[2]=shidu_shi+48;
  T_H[3]=shidu_ge%10+48;
  
  if ( AF_DataRequest( &Point_To_Point_DstAddr,
                       &SampleApp_epDesc,
                       SAMPLEAPP_POINT_TO_POINT_CLUSTERID,
                       4,
                       T_H,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

/*********************************************************************
 * @fn      SampleApp_SendFlashMessage
 *
 * @brief   Send the flash message to group 1.
 *
 * @param   flashTime - in milliseconds
 *
 * @return  none
 */
void SampleApp_SendFlashMessage( uint16 flashTime )
{
  uint8 buffer[3];
  buffer[0] = (uint8)(SampleAppFlashCounter++);
  buffer[1] = LO_UINT16( flashTime );
  buffer[2] = HI_UINT16( flashTime );

  if ( AF_DataRequest( &SampleApp_Flash_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_FLASH_CLUSTERID,
                       3,
                       buffer,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}
void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg)//发送 FE 02 01 F1  ,则返回01 F1
{
 uint8 i,len,*str=NULL;
 str=cmdMsg->msg;
 len=*str; //msg里的第1个字节代表后面的数据长度
 
 for(i=1;i<=len;i++)
 HalUARTWrite(0,str+i,1 ); 
 HalUARTWrite(0,"\n",1 );//换行  

  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_COM_CLUSTERID,
                       len,// 数据长度         
                       str+1,//数据内容
                       &SampleApp_TransID,//  簇ID  ??
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
 
}
/*********************************************************************
*********************************************************************/


#include "OSAL.h"
#include "ZGlobals.h"
#include "AF.h"
#include "aps_groups.h"
#include "ZDApp.h"

#include "SampleApp.h"
#include "SampleAppHw.h"

#include "OnBoard.h"

/* HAL */
#include "hal_lcd.h"
#include "hal_led.h"
#include "hal_adc.h"
#include "hal_key.h"
#include "MT_UART.h"
#include "MT_APP.h"
#include "MT.h"
#include "DHT11.h
void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );
void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg);
void SendMsgToShortAddr(uint8* buff, uint8 len, uint16 addr);
void SendShortAddrToCoor();

uint8 SampleApp_TaskID;   // Task ID for internal task/event processing
                          // This variable will be received when
                          // SampleApp_Init() is called.
devStates_t SampleApp_NwkState;

uint8 SampleApp_TransID;  // This is the unique message ID (counter)

afAddrType_t SampleApp_Periodic_DstAddr;
afAddrType_t SampleApp_Flash_DstAddr;

aps_Group_t SampleApp_Group;

uint8 SampleAppPeriodicCounter = 0

uint8 SampleAppFlashCounter = 0;

//重要,定时终端ID
//范围:1~MAX_DEVICE,不同的终端或者协调器，要修改此值
uint8 deviceId=1;
uint8 led_state=0;
DEVICES mDevice[MAX_DEVICE];

void SampleApp_HandleKeys( uint8 shift, uint8 keys );
void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pckt );
void SampleApp_SendPeriodicMessage( void );
void SampleApp_SendFlashMessage( uint16 flashTime );
void SampleApp_SerialCMD(mtOSALSerialData_t *cmdMsg);
void SendMsgToShortAddr(uint8* buff, uint8 len, uint16 addr);
void SendShortAddrToCoor();

void SampleApp_Init( uint8 task_id )
{
  uint8 i=0;
  SampleApp_TaskID = task_id;
  SampleApp_NwkState = DEV_INIT;
  SampleApp_TransID = 0;
  
  MT_UartInit();                  //串口初始化
  MT_UartRegisterTaskID(task_id); //注册串口任务
  HalUARTWrite(0,"UartInit OK\n", sizeof("UartInit OK\n"));//提示信息

  //协调器使用
  //初始化终端ID
  for(i=0; i<MAX_DEVICE; i++)
  {
    mDevice[i].id=i+1;//1~MAX_DEVICE
    mDevice[i].shortAddr=0;//短地址清0
  }  
void SampleApp_HandleKeys( uint8 shift, uint8 keys )
{
    (void)shift;  // Intentionally unreferenced parameter

#if defined( ZDO_COORDINATOR )

    if ( keys & HAL_KEY_SW_6 )//按下协调器key1
    {
        //点播发给终端1（或者路由器1）
        SendMsgToShortAddr("D1", 2, mDevice[0].shortAddr);        
    }

    if ( keys & HAL_KEY_SW_1 )//按下协调器key2
    {
        //点播发给终端2（或者路由器2）
        SendMsgToShortAddr("D2", 2, mDevice[1].shortAddr);
}

uint16 SampleApp_ProcessEvent( uint8 task_id, uint16 events )
{
  afIncomingMSGPacket_t *MSGpkt;
  (void)task_id;  // Intentionally unreferenced parameter

  if ( events & SYS_EVENT_MSG )
  {
    MSGpkt = (afIncomingMSGPacket_t *)osal_msg_receive( SampleApp_TaskID );
    while ( MSGpkt )
    {
      switch ( MSGpkt->hdr.event )
      {        
        // Received when a key is pressed
        case KEY_CHANGE:
          SampleApp_HandleKeys( ((keyChange_t *)MSGpkt)->state, ((keyChange_t *)MSGpkt)->keys );
          break;

        // Received when a messages is received (OTA) for this endpoint
        case AF_INCOMING_MSG_CMD:
          SampleApp_MessageMSGCB( MSGpkt );
          break;

        // Received whenever the device changes state in the network
        case ZDO_STATE_CHANGE:
          SampleApp_NwkState = (devStates_t)(MSGpkt->hdr.status);
          if (// (SampleApp_NwkState == DEV_ZB_COORD) ||
                 (SampleApp_NwkState == DEV_ROUTER)
              || (SampleApp_NwkState == DEV_END_DEVICE) )
          {
            // Start sending the periodic message in a regular interval.
            //联网成功把自己的ID发给协调器
            SendShortAddrToCoor();
            
            //周期采集定时器
            osal_start_timerEx( SampleApp_TaskID,
                              SAMPLEAPP_SEND_PERIODIC_MSG_EVT,
                              SAMPLEAPP_SEND_PERIODIC_MSG_TIMEOUT );
          }
          else
          {
            // Device is no longer in the network
          }
          break;

        default:
          break;
      }

void SampleApp_MessageMSGCB( afIncomingMSGPacket_t *pkt ) //接收数据
{
  uint16 flashTime;
  uint8 i=0;
  static uint8 j=0,j1=0,j2=0,j3=0,j4=0;
  static float value_v_temp = 0,value_i_temp = 0,
  value_v_temp1 = 0,value_i_temp1 = 0,
  value_v_temp2 = 0,value_i_temp2 = 0,
  value_v_temp3 = 0,value_i_temp3 = 0,
  value_v_temp4 = 0,value_i_temp4 = 0;
  uint8 str_adc[20]={0};
  uint8 str_adc1[20]={0};
  switch ( pkt->clusterId )
  {
    case SAMPLEAPP_PERIODIC_CLUSTERID:
      {
    	uint8 id=pkt->cmd.Data[0];
    	uint8 buff[20]={0};
        
        sprintf(buff, "NO:%d,H:%d,T:%d", id,  pkt->cmd.Data[3], pkt->cmd.Data[2]);
        
        if(id==1)//1号节点上传数据
        {
          j++;
          if((pkt->cmd.Data[4])<23)
            pkt->cmd.Data[4]=0;
          if((pkt->cmd.Data[5])<23)
            pkt->cmd.Data[5]=0;
          value_v_temp += (float)(pkt->cmd.Data[4])/100;
          value_i_temp += (float)(pkt->cmd.Data[5])/100;
          if(j==10)
          {
            j=0;
            sprintf(str_adc, "U:%.02f V", value_v_temp/10); 
            sprintf(str_adc1, "I:%.02f A", value_i_temp/10); 
            HalUARTWrite(0, buff, osal_strlen(buff));
            HalUARTWrite(0, str_adc, osal_strlen(str_adc));
            HalUARTWrite(0, str_adc1, osal_strlen(str_adc1));
            HalUARTWrite(0, "\n", 1); 
            value_v_temp = 0;
            value_i_temp = 0;
           }
        }
    	else if(id==2)//2号节点上传数据
        {
           j1++;
          if((pkt->cmd.Data[4])<23)
            pkt->cmd.Data[4]=0;
          if((pkt->cmd.Data[5])<23)
            pkt->cmd.Data[5]=0;
          value_v_temp1 += (float)(pkt->cmd.Data[4])/100;
          value_i_temp1 += (float)(pkt->cmd.Data[5])/100;
          if(j1==10)
          {
            j1=0;
            sprintf(str_adc, "U:%.02f V", value_v_temp1/10); 
            sprintf(str_adc1, "I:%.02f A", value_i_temp1/10); 
            HalUARTWrite(0, buff, osal_strlen(buff));
            HalUARTWrite(0, str_adc, osal_strlen(str_adc));
            HalUARTWrite(0, str_adc1, osal_strlen(str_adc1));
            HalUARTWrite(0, "\n", 1); 
            value_v_temp1 = 0;
            value_i_temp1 = 0;
           }
          HalUARTWrite(0, "\n", 1); 
        }
 	else if(id==3)//3号节点上传数据
        {
           j2++;
          if((pkt->cmd.Data[4])<23)
            pkt->cmd.Data[4]=0;
          if((pkt->cmd.Data[5])<23)
            pkt->cmd.Data[5]=0;
          value_v_temp2 += (float)(pkt->cmd.Data[4])/100;
          value_i_temp2 += (float)(pkt->cmd.Data[5])/100;
          if(j2==10)
          {
            j2=0;
            sprintf(str_adc, "U:%.02f V", value_v_temp2/10); 
            sprintf(str_adc1, "I:%.02f A", value_i_temp2/10); 
            HalUARTWrite(0, buff, osal_strlen(buff));
            HalUARTWrite(0, str_adc, osal_strlen(str_adc));
            HalUARTWrite(0, str_adc1, osal_strlen(str_adc1));
            HalUARTWrite(0, "\n", 1); 
            value_v_temp2 = 0;
            value_i_temp2 = 0;
           }
        }
        else if(id==4)//4号节点上传数据
        {
           j3++;
          if((pkt->cmd.Data[4])<23)
            pkt->cmd.Data[4]=0;
          if((pkt->cmd.Data[5])<23)
            pkt->cmd.Data[5]=0;
          value_v_temp3 += (float)(pkt->cmd.Data[4])/100;
          value_i_temp3 += (float)(pkt->cmd.Data[5])/100;
          if(j3==10)
          {
            j3=0;
            sprintf(str_adc, "U:%.02f V", value_v_temp3/10); 
            sprintf(str_adc1, "I:%.02f A", value_i_temp3/10); 
            HalUARTWrite(0, buff, osal_strlen(buff));
            HalUARTWrite(0, str_adc, osal_strlen(str_adc));
            HalUARTWrite(0, str_adc1, osal_strlen(str_adc1));
            HalUARTWrite(0, "\n", 1); 
            value_v_temp3 = 0;
            value_i_temp3 = 0;
           }
        }
        else if(id==5)//5号节点上传数据
        {
           j4++;
          if((pkt->cmd.Data[4])<23)
            pkt->cmd.Data[4]=0;
          if((pkt->cmd.Data[5])<23)
            pkt->cmd.Data[5]=0;
          value_v_temp4 += (float)(pkt->cmd.Data[4])/100;
          value_i_temp4 += (float)(pkt->cmd.Data[5])/100;
          if(j4==10)
          {
            j4=0;
            sprintf(str_adc, "U:%.02f V", value_v_temp4/10); 
            sprintf(str_adc1, "I:%.02f A", value_i_temp4/10); 
            HalUARTWrite(0, buff, osal_strlen(buff));
            HalUARTWrite(0, str_adc, osal_strlen(str_adc));
            HalUARTWrite(0, str_adc1, osal_strlen(str_adc1));
            HalUARTWrite(0, "\n", 1); 
            value_v_temp4 = 0;
            value_i_temp4 = 0;
           } 
        }
  		
//        HalUARTWrite(0, buff, osal_strlen(buff)); //输出接收到的数据
//        HalUARTWrite(0, "\n", 1);         //回车换行
      }
      break;

    //终端或者路由器收到此消息点亮相应的灯
    case SAMPLEAPP_FLASH_CLUSTERID:  
      
      break;

    //保存终端短地址
    case SAMPLEAPP_ADDR_CLUSTERID:
      for(i=0; i<MAX_DEVICE; i++)
      {
        if(mDevice[i].id==pkt->cmd.Data[0])
        {
            mDevice[i].shortAddr=BUILD_UINT16(pkt->cmd.Data[1], pkt->cmd.Data[2]);
        }
      }
      break;
  }
}


void SampleApp_SendPeriodicMessage( void )
{
  uint8 data[50]={0};
  float vol=0.0; //adc采样电压   
  float vol1=0.0; 
//  byte len=19;
//  byte lens=19;
  uint16 adc= HalAdcRead(HAL_ADC_CHANNEL_4, HAL_ADC_RESOLUTION_14);
  uint16 adc1= HalAdcRead(HAL_ADC_CHANNEL_6, HAL_ADC_RESOLUTION_14);
  
  DHT11();             //获取温湿度

  vol=(float)((float)(adc*3.3))/8192.0;
  vol1=(float)((float)(adc1*3.3))/8192.0;
  vol=vol*100;
  vol1=vol1*100;
  
//  len=osal_strlen(str_adc);//计算长度
//  lens=osal_strlen(str_adc1);//计算长度
  
  data[0]=deviceId;//终端id
  data[1]=(SampleApp_NwkState == DEV_ROUTER)?1:0;//路由1   终端0
  data[2]=wendu_shi*10+wendu_ge;//温度
  data[3]=shidu_shi*10+shidu_ge;//湿度
  data[4]=vol;// 电压
  data[5]=vol1;//电流
  
  
  if ( AF_DataRequest( &SampleApp_Periodic_DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_PERIODIC_CLUSTERID,
                       6,
                       data,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
  {
  }
  else
  {
    // Error occurred in request to send.
  }
}

void SendShortAddrToCoor()
{
    char buff[5]={0};
    afAddrType_t DstAddr;
    uint16 addr=NLME_GetShortAddr();


    //点播,协调器的短地址为0
    DstAddr.addrMode = (afAddrMode_t)Addr16Bit; 
    DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
    DstAddr.addr.shortAddr = 0x0;

    //填发送的数据
    buff[0]=deviceId;//第一个字节，终端编号
    buff[1]=LO_UINT16(addr);//第二个字节，终端短地址低字节
    buff[2]=HI_UINT16(addr);//第三个字节，终端编号
    //
    
    //发送
    if ( AF_DataRequest( &DstAddr, &SampleApp_epDesc,
                       SAMPLEAPP_ADDR_CLUSTERID,
                       3,
                       buff,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    }
    else
    {
    // Error occurred in request to send.
    }
}

//点播通信
//buff:发送内容指针
//len :发送内容长度
//addr:终端地址
void SendMsgToShortAddr(uint8* buff, uint8 len, uint16 addr)
{
    afAddrType_t DstAddr;
    
    //点播
    DstAddr.addrMode = (afAddrMode_t)Addr16Bit;
    DstAddr.endPoint = SAMPLEAPP_ENDPOINT;
    DstAddr.addr.shortAddr = addr;


    //发送
if ( AF_DataRequest( &DstAddr, &SampleApp_epDesc,         
SAMPLEAPP_ADDR_CLUSTERID,
                       3,
                       buff,
                       &SampleApp_TransID,
                       AF_DISCV_ROUTE,
                       AF_DEFAULT_RADIUS ) == afStatus_SUCCESS )
    {
    }
    else
    {
    // Error occurred in request to send.
    }
}
#include"iocc2530.h"
#include"OnBoard.h"

#define uint unsigned int 
#define uchar unsigned char

#define Ds18b20Data P0_6 //温度传感器引脚

#define ON 0x01  //读取成功返回0x00，失败返回0x01
#define OFF 0x00

void Ds18b20Delay(uint k);
void Ds18b20InputInitial(void);//设置端口为输入
void Ds18b20OutputInitial(void);//设置端口为输出
uchar Ds18b20Initial(void);
void Ds18b20Write(uchar infor);
uchar Ds18b20Read(void);
void Temp_test(void); //温度读取函数



unsigned char temp,test1,test2; //储存温度信息

//时钟频率为32M
void Ds18b20Delay(uint k) //调用函数，延时us
{
  MicroWait(k);
}

void Ds18b20InputInitial(void)//设置端口为输入
{
  P0DIR &= 0xbf;
}

void Ds18b20OutputInitial(void)//设置端口为输出
{
   P0DIR |= 0x40;
}

//ds18b20初始化
//初始化成功返回0x00，失败返回0x01
uchar Ds18b20Initial(void)
{
  uchar Status = 0x00;
  uint CONT_1 = 0;
  uchar Flag_1 = ON;
  Ds18b20OutputInitial();
  Ds18b20Data = 1;
  Ds18b20Delay(260);
  Ds18b20Data = 0;
  Ds18b20Delay(750);
  Ds18b20Data = 1;
  Ds18b20InputInitial();
  while((Ds18b20Data != 0)&&(Flag_1 == ON))//等待ds18b20响应，具有防止超时功能
  {                                        //等待约60ms左右
    CONT_1++;
    Ds18b20Delay(10);
    if(CONT_1 > 8000)Flag_1 = OFF;
    Status = Ds18b20Data;
  }
  Ds18b20OutputInitial();
  Ds18b20Data = 1;
  Ds18b20Delay(100);
  return Status;
}


void Ds18b20Write(uchar infor)
{
  uint i;
  Ds18b20OutputInitial();
  for(i=0;i<8;i++)
  {
  if((infor & 0x01))
  {
  Ds18b20Data = 0;
  Ds18b20Delay(6);
  Ds18b20Data = 1;
  Ds18b20Delay(50);
  }
  else
  {
  Ds18b20Data = 0;
  Ds18b20Delay(50);
  Ds18b20Data = 1;
  Ds18b20Delay(6);
  }
  infor >>= 1;
  }
}

uchar Ds18b20Read(void)
{
  uchar Value = 0x00;
  uint i;
  Ds18b20OutputInitial();
  Ds18b20Data = 1;
  Ds18b20Delay(10);
  for(i=0;i<8;i++)
  {
  Value >>= 1; 
  Ds18b20OutputInitial();
  Ds18b20Data = 0;
  Ds18b20Delay(3);
  Ds18b20Data = 1;
  Ds18b20Delay(3);
  Ds18b20InputInitial();
  if(Ds18b20Data == 1) Value |= 0x80;
  Ds18b20Delay(15);
  } 
  return Value;
}


void Temp_test(void) //温度读取函数
{
  uchar V1,V2;
  
  test1=Ds18b20Initial();
  Ds18b20Write(0xcc);
  Ds18b20Write(0x44);
  
  test2=Ds18b20Initial();
  Ds18b20Write(0xcc);
  Ds18b20Write(0xbe);
  
  V1 = Ds18b20Read();
  V2 = Ds18b20Read();
  temp = ((V1 >> 4)+((V2 & 0x07)*16));  

}

#include <ioCC2530.h>
#include "OnBoard.h"

#define uint unsigned int
#define uchar unsigned char

#define wenshi P0_6

/*******函数声明*********/
void Delay_us(void); //1 us延时
void Delay_10us(void); //10 us延时
void Delay_ms(uint Time);//n ms延时
void COM(void);	// 温湿写入
void DHT11_TEST(void) ;  //温湿传感启动


//温湿度定义
uchar ucharFLAG,uchartemp;
uchar shidu_shi,shidu_ge,wendu_shi,wendu_ge=4;
uchar ucharT_data_H,ucharT_data_L,ucharRH_data_H,ucharRH_data_L,ucharcheckdata;
uchar ucharT_data_H_temp,ucharT_data_L_temp,ucharRH_data_H_temp,ucharRH_data_L_temp,ucharcheckdata_temp;
uchar ucharcomdata;

uchar temp[2]={0,0}; 
uchar temp1[5]="temp=";
uchar humidity[2]={0,0};
uchar humidity1[9]="humidity=";

/****************************
//延时函数
*****************************/
void Delay_us(void) //1 us延时

{
    MicroWait(1);   
}

void Delay_10us(void) //10 us延时
{
   MicroWait(10);
}

void Delay_ms(uint Time)//n ms延时
{
  unsigned char i;
  while(Time--)
  {
    for(i=0;i<100;i++)
     Delay_10us();
  }
}


/***********************
   温湿度传感
***********************/
void COM(void)	// 温湿写入
{     
    uchar i;         
    for(i=0;i<8;i++)    
    {
     ucharFLAG=2; 
     while((!wenshi)&&ucharFLAG++);
     Delay_10us();
     Delay_10us();
     Delay_10us();
     uchartemp=0;
     if(wenshi)uchartemp=1;
     ucharFLAG=2;
     while((wenshi)&&ucharFLAG++);   
     if(ucharFLAG==1)break;    
     ucharcomdata<<=1;
     ucharcomdata|=uchartemp; 
     }    
}

void DHT11_TEST(void)   //温湿传感启动
{
    wenshi=0;
    Delay_ms(19);  //>18MS
    wenshi=1; 
    P0DIR &= ~0x40; //重新配置IO口方向
    Delay_10us();
    Delay_10us();						
    Delay_10us();
    Delay_10us();  
     if(!wenshi) 
     {
      ucharFLAG=2; 
      while((!wenshi)&&ucharFLAG++);
      ucharFLAG=2;
      while((wenshi)&&ucharFLAG++); 
      COM();
      ucharRH_data_H_temp=ucharcomdata;
      COM();
      ucharRH_data_L_temp=ucharcomdata;
      COM();
      ucharT_data_H_temp=ucharcomdata;
      COM();
      ucharT_data_L_temp=ucharcomdata;
      COM();
      ucharcheckdata_temp=ucharcomdata;
      wenshi=1; 
      uchartemp=(ucharT_data_H_temp+ucharT_data_L_temp+ucharRH_data_H_temp+ucharRH_data_L_temp);
       if(uchartemp==ucharcheckdata_temp)
      {
          ucharRH_data_H=ucharRH_data_H_temp;
          ucharRH_data_L=ucharRH_data_L_temp;
          ucharT_data_H=ucharT_data_H_temp;
          ucharT_data_L=ucharT_data_L_temp;
          ucharcheckdata=ucharcheckdata_temp;
       }
         wendu_shi=ucharT_data_H/10; 
         wendu_ge=ucharT_data_H%10;
	 
         shidu_shi=ucharRH_data_H/10; 
         shidu_ge=ucharRH_data_H%10;        
    } 
    else //没用成功读取，返回0
    {
         wendu_shi=0; 
         wendu_ge=0;
	 
         shidu_shi=0; 
         shidu_ge=0;  
    } 
     P0DIR |= 0x40; //IO口需要重新配置
 }
