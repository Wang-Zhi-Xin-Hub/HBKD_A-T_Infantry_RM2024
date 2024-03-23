#include "Task_Remote.h"

/* 遥控器接收任务 */
void Task_Remote_Rx(void *pvParameters)
{
    for(;;)
    {
        if(osThreadFlagsWait(0x01, osFlagsWaitAny, osWaitForever))
        {
            if(Remote_flag)
                Remote_Rx(Usart1_Remote_Dma[0]);
            else
                Remote_Rx(Usart1_Remote_Dma[1]);
        }
    }
}

/* 3个遥控器数据处理函数 */
void RemoteControlProcess(Remote *rc)
{
    RemoteMode=REMOTE_INPUT;
	
	Key_ch[0] =(float )(rc->ch0 -1024)/660;
	Key_ch[1] =(float )(rc->ch1 -1024)/660;
	Key_ch[2] =(float )(rc->ch2 -1024)/660;
	Key_ch[3] =(float )(rc->ch3 -1024)/660;
	
	deadline_limit(Key_ch[0],0.1f);
	deadline_limit(Key_ch[1],0.1f);
	deadline_limit(Key_ch[2],0.1f);
    deadline_limit(Key_ch[3],0.1f);
}

void STOPControlProcess()
{
	RemoteMode=STOP;
}

void MouseKeyControlProcess(Mouse *mouse, Key_t key, Key_t Lastkey) 
{
	RemoteMode=KEY_MOUSE_INPUT;
	
	limit (mouse ->x,100,-100);
	limit (mouse ->y,100,-100);
	limit (mouse ->z,100,-100);
	
	Mouse_ch[0]=(float)(mouse ->x)/100;
	Mouse_ch[1]=(float)(mouse ->y)/100;
	Mouse_ch[2]=(float)(mouse ->z)/100;
	
	deadline_limit(Mouse_ch[0],0.01f);
	deadline_limit(Mouse_ch[1],0.01f);
	deadline_limit(Mouse_ch[2],0.01f);
}
