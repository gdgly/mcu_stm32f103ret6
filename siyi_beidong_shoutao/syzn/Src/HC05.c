/**
  ******************************************************************************
  * �ļ�����: bsp_usartx.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2015-10-04
  * ��    ��: ���ڵײ���������
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F1Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "HC05.h"
#include <string.h>
#include <stdio.h>
#include "thread_of_hc05.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
extern  BLTDev bltDevList;   //�����豸�б���main�ļ��ж���

/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/


/**
  * ��������: ��HC05ģ�鷢��������OK��ֻ�����ھ���OKӦ�������
  * �������: cmd������������
  *           clean��1��������ջ���������
  *                  0���������ջ���������
  * �� �� ֵ: ����Ӧ��״̬��1����OKӦ��
  *                         0���ɹ����Ͳ����յ�OKӦ��
  * ˵    ������
  */
char* HC05_Send_CMD(char* cmd,uint8_t clean)
{	 		 
	uint8_t retry=5;
	uint8_t i;
	
	while(retry--)
	{
		HC05_EN_HIGHT();
		HAL_Delay(10);
		//HAL_UART_Transmit(&husartx_rs485,(uint8_t *)cmd,strlen(cmd),1000);
		Usart_SendString(2,(uint8_t *)cmd);
    for(i=0;i<2;i++)
    { 
      char* redata;
//      HAL_Delay(50);      
      redata = Getstrfromhc05(1000); 
      if(redata!=0)
      {
        if(redata[0]!=0)
        {
          HC05_DEBUG("send CMD: %s",cmd);
          HC05_DEBUG("receive %s",redata);
        }
        if(strstr(redata,"OK"))				
        {          
          if(clean==1)
            clean_rebuff();
          return redata;
        }
		if(strstr(redata,"FAIL"))
		{
          if(clean==1)
            clean_rebuff();	
		  return 0;
		}
      }
      else
      {					
        HAL_Delay(200);
      }		
    }
    HC05_DEBUG("HC05 send CMD fail %d times",retry);
  }	
	HC05_DEBUG("HC05 send CMD fail ");
	if(clean==1)
		clean_rebuff();
	return 0 ;
}

/**
  * ��������: ʹ��HC05͸���ַ�������
  * �������: str,Ҫ������ַ���
  * �� �� ֵ: ��
  * ˵    ������
  */
void HC05_SendString(char* str)
{
	Usart_SendString(2,(uint8_t *)str);
//HAL_UART_Transmit(&husartx_rs485,(uint8_t *)str,strlen(str),1000);
}

/**
  * ��������: ��ʼ��GPIO�����HC05ģ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void HC05_Init(void)
{
	uint8_t i;	
//	HC05_GPIO_Config();
//	HC05_USARTx_Init();	
	for(i=0;i<BLTDEV_MAX_NUM;i++)
	{
		sprintf(bltDevList.unpraseAddr[i]," ");
		sprintf(bltDevList.name[i]," ");
	}	
	bltDevList.num = 0;
}

/**
  * ��������: �ѽ��յ����ַ���ת����16������ʽ�����ֱ���(��Ҫ����ת��������ַ)
  * �������: str����ת���ַ���
  * �� �� ֵ: ��
  * ˵    ������
  */
unsigned long htoul(const char *str)
{
  long result = 0;

  if (!str)
    return 0;

  while (*str)
  {
    uint8_t value;

    if (*str >= 'a' && *str <= 'f')
      value = (*str - 'a') + 10;
    else if (*str >= 'A' && *str <= 'F')
      value = (*str - 'A') + 10;
    else if (*str >= '0' && *str <= '9')
      value = *str - '0';
    else
      break;

    result = (result * 16) + value;
    ++str;
  }
  return result;
}


/**
  * ��������: ��str�У�������ǰ���prefix�ַ���,
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
char *skipPrefix(char *str, size_t str_length, const char *prefix)
{
  uint16_t prefix_length = strlen(prefix);
  if (!str || str_length == 0 || !prefix)
    return 0;
  if (str_length >= prefix_length && strncmp(str, prefix, prefix_length) == 0)
    return str + prefix_length;
  return 0;
}

/**
  * ��������: ��stream�л�ȡһ���ַ�����line��
  * �������: line,�洢����е��ַ�������
  *           stream��ԭ�ַ���������       max_size��stream�Ĵ�С   
  * �� �� ֵ: line�ĳ��ȣ���stream��û�С�\0����'\r'��'\n'���򷵻�0
  * ˵    ������
  */
int get_line(char* line, char* stream ,int max_size)  
{  
  char *p;	
  int len = 0;  
  p=stream;
  while( *p != '\0' && len < max_size )
  {  
    line[len++] = *p;  
    p++;
    if('\n' == *p || '\r'==*p)  
        break;  
  }
  if(*p != '\0' && *p != '\n' && *p != '\r')
    return 0;
  line[len] = '\0';  
  return len;  
} 

/**
  * ��������: ��HC05д����������ģ�����Ӧ
  * �������: arg�����������Ϊ0ʱ������������commandҲΪ0ʱ������"AT"����
  * �� �� ֵ: ��
  * ˵    ������
  */
void writeCommand(const char *command, const char *arg)
{
  char str_buf[50];

  HC05_EN_HIGHT();
  HAL_Delay(10);

  if (arg && arg[0] != 0)
    sprintf(str_buf,"AT+%s%s\r\n",command,arg);
  else if (command && command[0] != 0)
  {
    sprintf(str_buf,"AT+%s\r\n",command);
  }
  else
    sprintf(str_buf,"AT\r\n");
  
  HC05_DEBUG("CMD send:%s",str_buf);
  Usart_SendString(4,(uint8_t *)str_buf);
 // HAL_UART_Transmit(&husartx_rs485,(uint8_t *)str_buf,strlen(str_buf),1000);
}


/**
  * ��������: ɨ���ܱߵ������豸�����洢���豸�б���
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
uint8_t parseBluetoothAddress(BLTDev *bltDev)
{
  /* Address should look like "+ADDR:<NAP>:<UAP>:<LAP>",
   * where actual address will look like "1234:56:abcdef".
   */
	char* redata;
	char linebuff[50];
	uint16_t linelen;
	uint16_t getlen=0;
	uint8_t linenum=0;	
	uint8_t i;
	char *p;

	redata = HC05_Send_CMD("AT+INQ\r\n",0);
	if(redata[0] != 0 && strstr(redata, "+INQ:") != 0)
	{
		HC05_DEBUG("rebuf =%s",redata);

getNewLine:
//		while(getlen < len-2*linenum )
//		{	
			linelen = get_line(linebuff,redata+getlen+2*linenum,512);
			if(linelen>50 && linelen != 0)
			{
				HC05_Send_CMD("AT+INQC\r\n",1);//�˳�ǰ�жϲ�ѯ
				return 1;
			}
			getlen += linelen;
			linenum++;			
			p = skipPrefix(linebuff,linelen,"+INQ:");
			if(p!=0)
			{
				uint8_t num ;
				num = bltDev->num;
				strBLTAddr(bltDev,':');
				for(i=0;i<=num;i++)
				{
					if(strstr(linebuff,bltDev->unpraseAddr[i]) != NULL)	
					{
						goto getNewLine;	//!=nullʱ����ʾ�õ�ַ��������ĵ�ַ��ͬ
					}
				}							
				/*�������豸�����б��У��Ե�ַ���н���*/	
				bltDev->addr[num].NAP = htoul(p);			
				p = strchr(p,':');

				if (p == 0)
				{
					HC05_Send_CMD("AT+INQC\r\n",1);//�˳�ǰ�жϲ�ѯ
					return 1;
				}
				bltDev->addr[num].UAP = htoul(++p);
				p = strchr(p,':');
				if (p == 0)
				{
					HC05_Send_CMD("AT+INQC\r\n",1);//�˳�ǰ�жϲ�ѯ
					return 1;
				}
				bltDev->addr[num].LAP = htoul(++p);
				/*�洢������ַ(�ַ�����ʽ)*/
				sprintf(bltDev->unpraseAddr[num],"%X:%X:%X",bltDev->addr[num].NAP,bltDev->addr[num].UAP,bltDev->addr[num].LAP);
				bltDev->num++;
			}
//		}
		clean_rebuff();
		HC05_Send_CMD("AT+INQC\r\n",1);//�˳�ǰ�жϲ�ѯ
		return 0;
	}	
	else
	{
		clean_rebuff();
		HC05_Send_CMD("AT+INQC\r\n",1);//�˳�ǰ�жϲ�ѯ
		return 1;	
	}
}

/**
  * ��������: ��������ַת�����ַ�����ʽ
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void strBLTAddr(BLTDev *bltDev,char delimiter)  
{
	uint8_t i;
	
	if(bltDev->num==0)
	{
		HC05_DEBUG("/*******No other BLT Device********/");
	}
	else
	{
		for(i=0;i<bltDev->num;i++)
		{
			sprintf(bltDev->unpraseAddr[i],"%X%c%X%c%X",bltDev->addr[i].NAP,delimiter,bltDev->addr[i].UAP,delimiter,bltDev->addr[i].LAP);
		}
	}
}

/**
  * ��������: ��ȡԶ�������豸������
  * �������: bltDev �������豸�б�ָ��
  * �� �� ֵ: 0��ȡ�ɹ�����0���ɹ�
  * ˵    ������
  */
uint8_t getRemoteDeviceName(BLTDev *bltDev)
{
	uint8_t i;
	char *redata;
	
	char linebuff[50];
	uint16_t linelen;
	char *p;
	
	char cmdbuff[100];
	
	strBLTAddr(bltDev,',');

	HC05_DEBUG("device num =%d",bltDev->num);
	
	for(i=0;i<bltDev->num;i++)
	{
		if(strstr(bltDev->unpraseAddr[i],"2017"))
		{
		sprintf(cmdbuff,"AT+RNAME?%s\r\n",bltDev->unpraseAddr[i]);
		redata = HC05_Send_CMD(cmdbuff,0);
		if(redata[0] != 0 && strstr(redata, "OK") != 0)
		{
			linelen = get_line(linebuff,redata,512);
			if(linelen>50 && linelen !=0 ) linebuff[linelen]='\0';	//�����ض�
					
			p = skipPrefix(linebuff,linelen,"+RNAME:");
			if(p!=0)
			{
				strcpy(bltDev->name[i],p);
			}
		}
		else
		{
			clean_rebuff();
			return 1;	
		}
		clean_rebuff();
		}
	}
	return 0;	
}

/**
  * ��������: ��������豸�б�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void printBLTInfo(BLTDev *bltDev)  
{
	uint8_t i;
	if(bltDev->num==0)
	{
		HC05_DEBUG("/*******No remote BLT Device or in SLAVE mode********/");
	}
	else
	{
		HC05_DEBUG("ɨ�赽 %d �������豸",bltDev->num);

		for(i=0;i<bltDev->num;i++)
		{
			HC05_INFO("/*******Device[%d]********/",i);	
			HC05_INFO("Device Addr: %s",bltDev->unpraseAddr[i]);
			HC05_INFO("Device name: %s",bltDev->name[i]);
		}
	}
}

/**
  * ��������: ɨ�������豸�������������к���"HC05"���豸
  * �������: ��
  * �� �� ֵ: 0��ȡ�ɹ�����0���ɹ�
  * ˵    ������
  */
uint8_t linkHC05(void)
{
	uint8_t i=0;
	char cmdbuff[100];
	char* redata;
getNewLine:
	parseBluetoothAddress(&bltDevList);
	getRemoteDeviceName(&bltDevList);
	printBLTInfo(&bltDevList);
	
	for(i=0;i<=bltDevList.num;i++)
	{
		if(strstr(bltDevList.name[i],"HC05") != NULL) //��NULL��ʾ�ҵ������Ʋ���ΪHC05���豸
		{
			HC05_INFO("������Զ��HC05ģ�飬���������������...");
			strBLTAddr(&bltDevList,',');		
			//���
			sprintf(cmdbuff,"AT+PAIR=%s,20\r\n",bltDevList.unpraseAddr[i]);			
			while(1)
			{
			Usart_SendString(2,(uint8_t *)cmdbuff);
			redata=Getstrfromhc05(1000);
			if(redata!=0)
			{
				if(strstr(redata,"RNAME"))
				{
									
				}
				else if(strstr(redata,"OK"))
				{
					break;
				}
				else if(strstr(redata,"FAIL"))
				{
					goto getNewLine;
				}
			}			
			
			}

//				if(HC05_Send_CMD(cmdbuff,0)==0)
//				{
//					goto getNewLine;			
//				}
			//����	
			sprintf(cmdbuff,"AT+LINK=%s\r\n",bltDevList.unpraseAddr[i]);
			if(HC05_Send_CMD(cmdbuff,0)!=0)
			{
				return 0;
			}
			return 1;	
		}
	}
	return 1;
}


void atlinkhc05(void)
{
	uint8_t i=0;
	char cmdbuff[100];
	HC05_EN_LOW();
	HAL_Delay(100);
	HC05_EN_HIGHT(); 
	HC05_Send_CMD("AT+INIT\r\n",1);
	HC05_Send_CMD("AT+iac=9e8b3f\r\n",1);
	HC05_Send_CMD("at+inqm=1,9,48\r\n",1);
	for(i=0;i<=bltDevList.num;i++)
	{
		if(strstr(bltDevList.name[i],"HC05") != NULL) //��NULL��ʾ�ҵ������Ʋ���ΪHC05���豸
		{
			sprintf(cmdbuff,"AT+LINK=%s\r\n",bltDevList.unpraseAddr[i]);	
			HC05_Send_CMD(cmdbuff,0);
		}
		 
	}

}
