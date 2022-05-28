#include "rlc_windows.h"
#include "stm32f1xx.h"
#include <math.h>

extern FontInfo font6x8;
extern measureParams mParams;
extern float freqList[4];
	
Window DisplayMainWnd;
Window DisplaySecondWnd;
pWindow CurrentWnd;
Window MenuWnd;
Window SetupWnds[4];
	
Data rlcData = {&mParams,0,{50,10,4},0,{4.2f,250.0f,25.0f},0,0,0,0,0,0,0};

int DisplayMainWindow(pWindow wnd, pData data, Action item_action, Action value_action)
{
    char param_str[14] = {0}, r_str[14] = {0}, x_str[14] = {0}, menu_str[13] = "    Ìåíþ    ";
    float C = 0, L = 0;
		
		if(data->param_vals->isAutoSet)
		{
			sprintf(param_str,"Auto R%d F%d G%d",rlcData.param_vals->R_sense,rlcData.param_vals->testSignalFreq,rlcData.param_vals->uGain);
		}
		else
		{
			sprintf(param_str,"R%d F%d G%d",data->param_vals->R_sense,data->param_vals->testSignalFreq,data->param_vals->uGain);
		}
	
		if(data->param_vals->measureType == 3)
		{
			C = 1/(2*M_PI*freqList[getFrequency()]*fabs(data->X));
			if(C < 1e-11)
			{
				sprintf(x_str, "C = %0.3f ïÔ", C*1e12);
			}
			if(C >= 1e-11 && C < 1e-10)
			{
				sprintf(x_str, "C = %0.2f ïÔ", C*1e12);
			}
			if(C >= 1e-10 && C < 1e-9)
			{
				sprintf(x_str, "C = %0.1f ïÔ", C*1e12);
			}
			if(C >= 1e-9 && C < 1e-8)
			{
				sprintf(x_str, "C = %0.3f íÔ", C*1e9);
			}
			if(C >= 1e-8 && C < 1e-7)
			{
				sprintf(x_str, "C = %0.2f íÔ", C*1e9);
			}
			if(C >= 1e-7 && C < 1e-6)
			{
				sprintf(x_str, "C = %0.1f íÔ", C*1e9);
			}
			if(C >= 1e-6 && C < 1e-5)
			{
				sprintf(x_str, "C=%0.3fìêÔ", C*1e6);
			}
			if(C >= 1e-5 && C < 1e-4)
			{
				sprintf(x_str, "C=%0.2fìêÔ", C*1e6);
			}
			if(C >= 1e-4 && C < 1e-3)
			{
				sprintf(x_str, "C=%0.1fìêÔ", C*1e6);
			}
			if(C >= 1e-3)
			{
				sprintf(x_str, "C = %0.2f ìÔ", C*1e3);
			}
		}
		if(data->param_vals->measureType == 2)
		{
			L = data->X/(2*M_PI*freqList[getFrequency()]);
			if(L >= 1e-9 && L < 1e-6)
			{
				sprintf(x_str, "L = %0.1f íÃí", L*1e9);
			}
			if(L >= 1e-6 && L < 1e-5)
			{
				sprintf(x_str, "L=%0.3fìêÃí", L*1e6);
			}			
			if(L >= 1e-5 && L < 1e-4)
			{
				sprintf(x_str, "L=%0.2fìêÃí", L*1e6);
			}			
			if(L >= 1e-4 && L < 1e-3)
			{
				sprintf(x_str, "L=%0.1fìêÃí", L*1e6);
			}
			if(L >= 1e-3 && L < 1e-2)
			{
				sprintf(x_str, "L = %0.3f ìÃí", L*1e3);
			}
			if(L >= 1e-2 && L < 1e-1)
			{
				sprintf(x_str, "L = %0.2f ìÃí", L*1e3);
			}			
			if(L >= 1e-1 && L < 1)
			{
				sprintf(x_str, "L = %0.1f ìÃí", L*1e3);
			}
			if(L >= 1)
			{
				sprintf(x_str, "L = %0.2f Ãí", L);
			}
		}
		if(data->param_vals->measureType == 0)
		{
			if(data->X < 0)
			{
					C = 1/(2*M_PI*freqList[getFrequency()]*fabs(data->X));
					if(C < 1e-11)
					{
						sprintf(x_str, "C = %0.3f ïÔ", C*1e12);
					}
					if(C >= 1e-11 && C < 1e-10)
					{
						sprintf(x_str, "C = %0.2f ïÔ", C*1e12);
					}
					if(C >= 1e-10 && C < 1e-9)
					{
						sprintf(x_str, "C = %0.1f ïÔ", C*1e12);
					}
					if(C >= 1e-9 && C < 1e-8)
					{
						sprintf(x_str, "C = %0.3f íÔ", C*1e9);
					}
					if(C >= 1e-8 && C < 1e-7)
					{
						sprintf(x_str, "C = %0.2f íÔ", C*1e9);
					}
					if(C >= 1e-7 && C < 1e-6)
					{
						sprintf(x_str, "C = %0.1f íÔ", C*1e9);
					}
					if(C >= 1e-6 && C < 1e-5)
					{
						sprintf(x_str, "C=%0.3fìêÔ", C*1e6);
					}
					if(C >= 1e-5 && C < 1e-4)
					{
						sprintf(x_str, "C=%0.2fìêÔ", C*1e6);
					}
					if(C >= 1e-4 && C < 1e-3)
					{
						sprintf(x_str, "C=%0.1fìêÔ", C*1e6);
					}
					if(C >= 1e-3)
					{
						sprintf(x_str, "C = %0.2f ìÔ", C*1e3);
					}
			}
			else
			{
					L = data->X/(2*M_PI*freqList[getFrequency()]);
					if(L >= 1e-9 && L < 1e-6)
					{
						sprintf(x_str, "L = %0.1f íÃí", L*1e9);
					}
					if(L >= 1e-6 && L < 1e-5)
					{
						sprintf(x_str, "L=%0.3fìêÃí", L*1e6);
					}			
					if(L >= 1e-5 && L < 1e-4)
					{
						sprintf(x_str, "L=%0.2fìêÃí", L*1e6);
					}			
					if(L >= 1e-4 && L < 1e-3)
					{
						sprintf(x_str, "L=%0.1fìêÃí", L*1e6);
					}
					if(L >= 1e-3 && L < 1e-2)
					{
						sprintf(x_str, "L = %0.3f ìÃí", L*1e3);
					}
					if(L >= 1e-2 && L < 1e-1)
					{
						sprintf(x_str, "L = %0.2f ìÃí", L*1e3);
					}			
					if(L >= 1e-1 && L < 1)
					{
						sprintf(x_str, "L = %0.1f ìÃí", L*1e3);
					}
					if(L >= 1)
					{
						sprintf(x_str, "L = %0.2f Ãí", L);
					}
			}
		}
		
		data->R = fabs(data->R);
		if(data->R < 1)
		{
			sprintf(r_str, "R = %0.3f Îì", data->R);
		}
		if(data->R >= 1 && data->R < 100)
		{
			sprintf(r_str, "R = %0.2f Îì", data->R);
		}
		if(data->R >= 100 && data->R < 1000)
		{
			sprintf(r_str, "R = %0.1f Îì", data->R);
		}		
		if(data->R >= 1e3 && data->R < 1e4)
		{
			sprintf(r_str, "R = %0.3f êÎì", data->R/1e3);
		}
		if(data->R >= 1e4 && data->R < 1e5)
		{
			sprintf(r_str, "R = %0.2f êÎì", data->R/1e3);
		}
		if(data->R >= 1e5 && data->R < 1e6)
		{
			sprintf(r_str, "R = %0.1f êÎì", data->R/1e3);
		}
		if(data->R >= 1e6 && data->R < 1e7)
		{
			sprintf(r_str, "R = %0.3f ÌÎì", data->R/1e6);
		}
		if(data->R >= 1e7)
		{
			sprintf(r_str, "R = %0.2f ÌÎì", data->R/1e6);
		}
		
		String str1 = {0,9,AlignCenter,font6x8,(const char*)param_str,NotInverted};
		String str2 = {0,18,AlignCenter,font6x8,(const char*)r_str,NotInverted};
		String str3 = {0,38,AlignCenter,font6x8,(const char*)menu_str,Inverted};
		
    wnd->strings[0] = str1;
    wnd->strings[1] = str2;
    wnd->strings[2] = str3;
		
		if(getMeasureType() != 1)
		{
				String str4 = {0,27,AlignCenter,font6x8,(const char*)x_str,NotInverted};
				wnd->strings[3] = str4;
				wnd->StringsQuantity = 4;
		}
		else
		{
				wnd->StringsQuantity = 3;
		}

    SetWindow(wnd);
    return 1;
}

int DisplaySecondWindow(pWindow wnd, pData data,Action item_action, Action value_action)
{
	char Ux_str[14] = {0}, r_str[14] = {0}, Ur_str[14] = {0}, fi_str[14] = {0};
	
	if(data->Z < 1)
	{
		sprintf(r_str, "Z = %0.3f Îì", data->Z);
	}
	if(data->Z >= 1 && data->Z < 100)
	{
		sprintf(r_str, "Z = %0.2f Îì", data->Z);
	}
	if(data->Z >= 100 && data->Z < 1000)
	{
		sprintf(r_str, "Z = %0.1f Îì", data->Z);
	}		
	if(data->Z >= 1e3 && data->Z < 1e4)
	{
		sprintf(r_str, "Z = %0.3f êÎì", data->Z/1e3);
	}
	if(data->Z >= 1e4 && data->Z < 1e5)
	{
		sprintf(r_str, "Z = %0.2f êÎì", data->Z/1e3);
	}
	if(data->Z >= 1e5 && data->Z < 1e6)
	{
		sprintf(r_str, "Z = %0.1f êÎì", data->Z/1e3);
	}
	if(data->Z >= 1e6 && data->Z < 1e7)
	{
		sprintf(r_str, "Z = %0.3f ÌÎì", data->Z/1e6);
	}
	if(data->Z >= 1e7)
	{
		sprintf(r_str, "Z = %0.2f ÌÎì", data->Z/1e6);
	}
	
	sprintf(Ur_str, "Ur = %0.3fÂ", data->Ur);
	sprintf(Ux_str, "Ux = %0.3fÂ", data->Ux);
	sprintf(fi_str, "fi = %0.2f°", data->fi);
	
	String str1 = {0,9,AlignCenter,font6x8,(const char*)r_str,NotInverted};
	String str2 = {0,18,AlignCenter,font6x8,(const char*)Ur_str,NotInverted};
	String str3 = {0,27,AlignCenter,font6x8,(const char*)Ux_str,NotInverted};
	String str4 = {0,38,AlignCenter,font6x8,(const char*)fi_str,NotInverted};
	
	
	wnd->strings[0] = str1;
	wnd->strings[1] = str2;
	wnd->strings[2] = str3;
	wnd->strings[3] = str4;
	wnd->StringsQuantity = 4;	
			
	SetWindow(wnd);
	return 1;
}

int SetupParametersWindow(pWindow wnd, pData data, Action item_action, Action value_action)
{
		uint8_t param_current_index_limit = 4;
	
		if(data->param_vals->isAutoSet)
		{
			param_current_index_limit = 2;
		}
		
    if(item_action == Next)
    {
        if(++data->param_current_index >= param_current_index_limit)
        {
            data->param_current_index = 0;
            return 0;
        }
    }

    if(value_action == Next)
    {
        switch(data->param_current_index)
        {
					 case 0:
                if(++data->param_vals->isAutoSet >= 2)
                {
                    data->param_vals->isAutoSet = 0;
                }
               break;
								
           case 1:
                if(++data->param_vals->testSignalFreq >= 4)
                {
                    data->param_vals->testSignalFreq = 0;
                }
               break;

           case 2:
                if(++data->param_vals->uGain >= 4)
                {
                    data->param_vals->uGain = 0;
                }
               break;

           case 3:
								if(++data->param_vals->R_sense >= 5)
								{
										data->param_vals->R_sense = 0;
								}
               break;														
       }
				
			 setFrequency(data->param_vals->testSignalFreq);
			 setUGain(data->param_vals->uGain);
			 setRsense(data->param_vals->R_sense);
			 setAutoSetParams(data->param_vals->isAutoSet);
    }
    if(value_action == Prev)
    {
        switch(data->param_current_index)
        {
           case 0:
                if(--data->param_vals->isAutoSet >= 128)
                {
                    data->param_vals->isAutoSet = 1;
                }
               break;
								
           case 1:
                if(--data->param_vals->testSignalFreq >= 128)
                {
                    data->param_vals->testSignalFreq = 3;
                }
               break;

           case 2:
                if(--data->param_vals->uGain >= 128)
                {
                    data->param_vals->uGain = 3;
                }
               break;

           case 3:
                if(--data->param_vals->R_sense >= 128)
                {
                    data->param_vals->R_sense = 4;
                }
               break;
       }
				
			 setFrequency(data->param_vals->testSignalFreq);
			 setUGain(data->param_vals->uGain);
			 setRsense(data->param_vals->R_sense);
			 setAutoSetParams(data->param_vals->isAutoSet);
    }
		
		char freq_str[4][8] = {"120Ãö\0","1êÃö\0","16êÃö\0","94êÃö"};
		char gain_str[4][5] = {"2\0","5\0","13\0","34\0"};
		char rs_str[5][7] = {"100Îì\0","1ê\0","10ê\0","100ê\0","10Îì\0"};
		char automode_str[2][6] = {"Âûêë.\0","Âêë.\0"};
		char low_str1[12] = "-  Äàëåå  +";
		char low_str2[12] = "-    OK   +";
		
		if(data->param_vals->isAutoSet)
		{
				String str1 = {1,9,AlignLeft,font6x8,"Àâòî:",NotInverted}; //seconds
				String str2 = {0,9,AlignRight,font6x8,(const char*)automode_str[data->param_vals->isAutoSet],(data->param_current_index == 0)? (Inverted):(NotInverted)};
				
				wnd->strings[0] = str1;
				wnd->strings[1] = str2;
				
				if(data->param_current_index == 1)
				{
					String str5 = {0,38,AlignCenter,font6x8,(const char*)low_str2,NotInverted};
					wnd->strings[2] = str5;
				}
				else
				{
					String str5 = {0,38,AlignCenter,font6x8,(const char*)low_str1,NotInverted};
					wnd->strings[2] = str5;
				}
				wnd->StringsQuantity = 3;
		}
		else
		{
				if(data->param_current_index < 3)
				{
					String str1 = {1,9,AlignLeft,font6x8,"Àâòî:",NotInverted}; //seconds
					String str2 = {0,9,AlignRight,font6x8,(const char*)automode_str[data->param_vals->isAutoSet],(data->param_current_index == 0)? (Inverted):(NotInverted)};
					String str3 = {0,18,AlignLeft,font6x8,"×àñòîòà: ",NotInverted};
					String str4 = {0,18,AlignRight,font6x8,(const char*)freq_str[data->param_vals->testSignalFreq],(data->param_current_index == 1)? (Inverted):(NotInverted)};
					String str5 = {1,27,AlignLeft,font6x8,"Êîýô.óñèë:",NotInverted}; //hours
					String str6 = {0,27,AlignRight,font6x8,(const char*)gain_str[data->param_vals->uGain],(data->param_current_index == 2)? (Inverted):(NotInverted)}; //week day
					String str7 = {0,38,AlignCenter,font6x8,(const char*)low_str1,NotInverted};
							
					wnd->strings[0] = str1;
					wnd->strings[1] = str2;
					wnd->strings[2] = str3;
					wnd->strings[3] = str4;
					wnd->strings[4] = str5;
					wnd->strings[5] = str6;
					wnd->strings[6] = str7;
				}
				else
				{		
							String str1 = {0,9,AlignLeft,font6x8,"×àñòîòà: ",NotInverted};
							String str2 = {0,9,AlignRight,font6x8,(const char*)freq_str[data->param_vals->testSignalFreq],(data->param_current_index == 1)? (Inverted):(NotInverted)}; //minutes
							String str3 = {1,18,AlignLeft,font6x8,"Ê-íò.óñèë:",NotInverted}; //hours
							String str4 = {0,18,AlignRight,font6x8,(const char*)gain_str[data->param_vals->uGain],(data->param_current_index == 2)? (Inverted):(NotInverted)}; //week day
							String str5 = {1,27,AlignLeft,font6x8,"Rsense:",NotInverted}; //seconds
							String str6 = {0,27,AlignRight,font6x8,(const char*)rs_str[data->param_vals->R_sense],(data->param_current_index == 3)? (Inverted):(NotInverted)}; //minute
							String str7 = {0,38,AlignCenter,font6x8,(const char*)low_str2,NotInverted};
							
							wnd->strings[0] = str1;
							wnd->strings[1] = str2;
							wnd->strings[2] = str3;
							wnd->strings[3] = str4;
							wnd->strings[4] = str5;
							wnd->strings[5] = str6;
							wnd->strings[6] = str7;
				}
			wnd->StringsQuantity = 7;
	  }
		
		DrawLine(0,37,83,37);
		
    SetWindow(wnd);
    return 1;
}

int SetupModeWindow(pWindow wnd, pData data,Action item_action, Action value_action)
{
	  if(item_action == NoAction)
    {
				if(value_action == Next)
				{
						if(++data->param_vals->measureType > 3)
						{
							data->param_vals->measureType = 0;
						}
						setMeasureType(data->param_vals->measureType);
				}
				if(value_action == Prev)
				{
						if(--data->param_vals->measureType >= 128)
						{
							data->param_vals->measureType = 3;
						}
						setMeasureType(data->param_vals->measureType);
				}
				
				char low_str[13] = "<    OK    >\0", s[13] = {0};
				const char* m_str[4] = {"Àâòî", "R", "L", "C"};
				
				sprintf(s,"Ðåæèì: %s",m_str[data->param_vals->measureType]);
				
				String str1 = {0,9,AlignLeft,font6x8,s,NotInverted}; //brightness
				String str2 = {0,38,AlignCenter,font6x8,(const char*)low_str,NotInverted};	
				
				DrawLine(0,37,83,37);

				wnd->strings[0] = str1;
				wnd->strings[1] = str2;

				wnd->StringsQuantity = 2;
				SetWindow(wnd);
    }
		else
		{
			return 0;
		}
		return 1;
}

int SetupDisplayWindow(pWindow wnd, pData data, Action item_action, Action value_action)
{
    if(item_action == Next)
    {
        if(++data->display_current_index >= sizeof(data->display_vals))
        {
            data->display_current_index = 0;
            return 0;
        }
    }
    if(value_action == Next)
    {
       switch(data->display_current_index)
       {
           case 0:
							  if(data->display_vals[data->display_current_index] < 100)
                {
                    data->display_vals[data->display_current_index] += 5;
                }
								else
								{
									  data->display_vals[data->display_current_index] = 0;
								}
               break;

           case 1:
              if(data->display_vals[data->display_current_index] < 60)
              {
                  data->display_vals[data->display_current_index] += 10;
              }
							else
							{
								  data->display_vals[data->display_current_index] = 0;
							}
              break;
							
					 case 2:
						  if(data->display_vals[data->display_current_index] < 7)
              {
                  data->display_vals[data->display_current_index] += 1;
              }
							else
							{
								  data->display_vals[data->display_current_index] = 0;
							}
						  break;
       }
    }
    if(value_action == Prev)
    {
        switch(data->display_current_index)
        {
            case 0:
                if(data->display_vals[data->display_current_index] > 0)
                {
                    data->display_vals[data->display_current_index] -= 5;
                }
							  else
							  {
							 		data->display_vals[data->display_current_index] = 100;
							  }
                break;

            case 1:
               if(data->display_vals[data->display_current_index] > 0)
               {
                   data->display_vals[data->display_current_index] -= 10;
               }
							 else
							 {
								   data->display_vals[data->display_current_index] = 60;
							 }						
               break;
							 
						case 2:
               if(data->display_vals[data->display_current_index] > 0)
               {
                   data->display_vals[data->display_current_index] -= 1;
               }
							 else
							 {
							 	  data->display_vals[data->display_current_index] = 7;
							 }						 
							 break;
        }
				
    }
		//set brightness
		TIM4->CCR2 = data->display_vals[0]/5;
		//set contrast
		GPIOA->BRR = CE;
		Write_Cmd(0x21);
		Write_Cmd(0x10+(data->display_vals[2]));
		Write_Cmd(0x20);
		Write_Cmd(0x0C);
		GPIOA->BSRR = CE;

    char bright_str[5] = "";
    char cont_str[2] = "";
		char light_time[5] = "";
		char low_str1[12] = "-  Äàëåå  +";
		char low_str2[12] = "-    OK   +";

    sprintf(bright_str,"%d%%",data->display_vals[0]);
		sprintf(light_time,"%02d c",data->display_vals[1]);
    sprintf(cont_str,"%d",data->display_vals[2]);
		
		String str1 = {1,9,AlignLeft,font6x8,"ßðêîñòü:",NotInverted}; //brightness
		String str2 = {0,9,AlignRight,font6x8,(const char*)bright_str,(data->display_current_index == 0)? (Inverted):(NotInverted)}; //brightness
		String str3 = {1,18,AlignLeft,font6x8,"Âðåìÿ:", NotInverted}; //time
		String str4 = {0,18,AlignRight,font6x8,(const char*)light_time,(data->display_current_index == 1)? (Inverted):(NotInverted)}; //time
		String str5 = {1,27,AlignLeft,font6x8,"Êîíòðàñò:", NotInverted}; //contrast
		String str6 = {0,27,AlignRight,font6x8,(const char*)cont_str,(data->display_current_index == 2)? (Inverted):(NotInverted)}; //contrast
		
		if(data->display_current_index < 2)
		{
			String str7 = {0,38,AlignCenter,font6x8,(const char*)low_str1,NotInverted};
			wnd->strings[6] = str7;
		}
		else
		{
			String str7 = {0,38,AlignCenter,font6x8,(const char*)low_str2,NotInverted};	
			wnd->strings[6] = str7;
		}
		
		DrawLine(0,37,83,37);

		wnd->strings[0] = str1;
		wnd->strings[1] = str2;
		wnd->strings[2] = str3;
		wnd->strings[3] = str4;
		wnd->strings[4] = str5;
		wnd->strings[5] = str6;

    wnd->StringsQuantity = 7;
    SetWindow(wnd);
    return 1;
}

int SetBatteryStateWindow(pWindow wnd, pData data, Action item_action, Action action)
{
    char volt_str[10] = {0}, temp_str[11] = {0}, i_str[11] = {0};
    if(item_action == NoAction)
    {
       sprintf(volt_str,"U = %.2fÂ",data->batADC_data[0]);
			
			 if(USB_ON())
			 {
					 sprintf(temp_str,"T = %0.1f°C",data->batADC_data[2]);
					 sprintf(i_str,"I = %0.0f ìÀ",data->batADC_data[1]);
				 
				 	 String str1 = {0,9,AlignCenter,font6x8,(const char*)volt_str,NotInverted}; //voltage
					 String str2 = {0,18,AlignCenter,font6x8,(const char*)i_str,NotInverted}; // current
					 String str3 = {0,27,AlignCenter,font6x8,(const char*)temp_str,NotInverted}; //temperature
					 String str4 = {0,38,AlignCenter,font6x8,"OK",NotInverted};

					 wnd->strings[0] = str1;
					 wnd->strings[1] = str2;
					 wnd->strings[2] = str3;
					 wnd->strings[3] = str4;

           wnd->StringsQuantity = 4;
			 }
			 else
			 {
				 	 String str1 = {0,9,AlignCenter,font6x8,(const char*)volt_str,NotInverted}; //voltage
					 String str2 = {0,38,AlignCenter,font6x8,"OK",NotInverted};
					 
					 wnd->strings[0] = str1;
					 wnd->strings[1] = str2;
					 
					 wnd->StringsQuantity = 2;
			 }
			 
			 DrawLine(0,37,83,37);
       SetWindow(wnd);
       return 1;
    }
    else
    {
        return 0;
    }
}

int SetMenuWindow(pWindow wnd, pData data, Action item_action, Action action)
{
	char title[10] = "Íàñòðîéêè";
	String MenuTitle = {0,9,AlignCenter,font6x8,(const char*)title,NotInverted};
	
	const char* Items[5] = {"Ðåæèì","Ïàðàìåòðû","Áàòàðåÿ","Äèñïëåé","Âûõîä"};
	uint8_t ItemsQuantity = 5;

	if(item_action == Next)
	{
			if(++data->current_item >= ItemsQuantity)
			{
					data->current_item = 0;
			}
  }
	if(item_action == Prev)
	{
			if(--data->current_item > 128)
			{
					data->current_item = ItemsQuantity-1;
			}
	}
	
	for(uint8_t i = 0;(i < ItemsQuantity)&&(i < 3);i++)
	{
		if(data->current_item < 3)
		{
			String item = {5,19+9*i,AlignLeft,font6x8,Items[i],(i == data->current_item)?(Inverted):(NotInverted)};
			wnd->strings[i] = item;
	  }
		else
		{
			String item =	{5,19+9*i,AlignLeft,font6x8,Items[data->current_item+i-2],(i == 2)?(Inverted):(NotInverted)};
			wnd->strings[i] = item;
		}
	}	
	
	wnd->strings[3] = MenuTitle;
	wnd->StringsQuantity = 4;
  SetWindow(wnd);
	
	return 0;
}

void LightEnable()
{
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M_2 |TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_0);
	TIM4->CCMR1 |= TIM_CCMR1_OC2M_2 |TIM_CCMR1_OC2M_1;//// PWM mode 1
	TIM4->CCER |= TIM_CCER_CC2E;
}

void LightDisable()
{
	TIM4->CCMR1 &= ~(TIM_CCMR1_OC2M_2 |TIM_CCMR1_OC2M_1|TIM_CCMR1_OC2M_0);
	TIM4->CCMR1 |= TIM_CCMR1_OC2M_2;//force output to low
	TIM4->CCER &= ~TIM_CCER_CC2E;
}

void WindowsInit()
{
	DisplayMainWnd.callback = &DisplayMainWindow;
	DisplaySecondWnd.callback = &DisplaySecondWindow;
	
	DisplayMainWnd.next = &DisplaySecondWnd; DisplayMainWnd.prev = &DisplaySecondWnd;
	DisplaySecondWnd.next = &DisplayMainWnd; DisplaySecondWnd.prev = &DisplayMainWnd;
	
	MenuWnd.callback = &SetMenuWindow;
	
	SetupWnds[0].callback = &SetupModeWindow;
	SetupWnds[1].callback = &SetupParametersWindow;
	SetupWnds[2].callback = &SetBatteryStateWindow;
	SetupWnds[3].callback = &SetupDisplayWindow;
	
	CurrentWnd = &DisplayMainWnd;
}
