#include "rlc_windows.h"
#include "stm32f1xx.h"
#include <math.h>
#include "usbd_custom_hid_if.h"

#define USBD_DFU_BOOT_DEFAULT_ADD 0x0800A000

typedef  void (*pFunction)(void);

extern FontInfo font6x8, MSSanSerif_6pt;
extern measureParams mParams;
extern Stabilization rlcStabilzation;
extern CalibrationVals calibrationValues;
extern float freqList[4];
extern USBD_HandleTypeDef hUsbDeviceFS;
	
Window DisplayMainWnd;
Window DisplaySecondWnd;
pWindow CurrentWnd;
Window MenuWnd;
Window SetupWnds[6];
	
Data rlcData = {&mParams,{50,10,4},{4.2f,250.0f,25.0f},0,0,0,0,0,0,0,0};

int DisplayMainWindow(pWindow wnd, pData data, Action item_action, Action value_action)
{
    char param_str[14] = {0}, r_str[14] = {0}, x_str[14] = {0}, qd_str[14] = {0}, menu_str[13] = "    ћеню    ";
		const char* Rsp[2] = {"Rs","Rp"};
		const char* Lsp[2] = {"Ls","Lp"};
		const char* Csp[2] = {"Cs","Cp"};
		const char* measureTypesStr[4] = {"", "R", "L", "C"};
		
    float C = 0, L = 0, R = 0, D = 0, Q = 0;
		//current measure parameters string
		if(data->param_vals->isAutoSet)
		{
			sprintf(param_str,"Auto F%d %s", rlcData.param_vals->testSignalFreq, measureTypesStr[rlcData.param_vals->measureType]);
		}
		else
		{
			sprintf(param_str,"F%d %s", rlcData.param_vals->testSignalFreq, measureTypesStr[rlcData.param_vals->measureType]);
		}
		//capacity string
		if((data->param_vals->measureType == 3) || ((data->param_vals->measureType == 0)&&(data->X < 0)))
		{
			C = fabs(data->X);
			D = fabs(data->R)/C;
			C = 1/(2*M_PI*freqList[getFrequency()]*C);
			
			if(data->rsrp == 1)
			{
				C /= (1+D*D);
			}
			sprintf(qd_str, "D = %0.3g", D);
			
			if(C < 1e-11)
			{
				sprintf(x_str, "%s = %0.3f п‘", Csp[data->rsrp], C*1e12);
			}
			if(C >= 1e-11 && C < 1e-10)
			{
				sprintf(x_str, "%s = %0.2f п‘", Csp[data->rsrp], C*1e12);
			}
			if(C >= 1e-10 && C < 1e-9)
			{
				sprintf(x_str, "%s = %0.1f п‘", Csp[data->rsrp], C*1e12);
			}
			if(C >= 1e-9 && C < 1e-8)
			{
				sprintf(x_str, "%s = %0.3f н‘", Csp[data->rsrp], C*1e9);
			}
			if(C >= 1e-8 && C < 1e-7)
			{
				sprintf(x_str, "%s = %0.2f н‘", Csp[data->rsrp], C*1e9);
			}
			if(C >= 1e-7 && C < 1e-6)
			{
				sprintf(x_str, "%s = %0.1f н‘", Csp[data->rsrp], C*1e9);
			}
			if(C >= 1e-6 && C < 1e-5)
			{
				sprintf(x_str, "%s = %0.3fмк‘", Csp[data->rsrp], C*1e6);
			}
			if(C >= 1e-5 && C < 1e-4)
			{
				sprintf(x_str, "%s = %0.2fмк‘", Csp[data->rsrp], C*1e6);
			}
			if(C >= 1e-4 && C < 1e-3)
			{
				sprintf(x_str, "%s = %0.1fмк‘", Csp[data->rsrp], C*1e6);
			}
			if(C >= 1e-3 && C < 1e-2)
			{
				sprintf(x_str, "%s = %0.3f м‘", Csp[data->rsrp], C*1e3);
			}
			if(C >= 1e-2 && C < 1e-1)
			{
				sprintf(x_str, "%s = %0.2f м‘", Csp[data->rsrp], C*1e3);
			}
			if(C >= 1e-1 && C < 1)
			{
				sprintf(x_str, "%s = %0.1f м‘", Csp[data->rsrp], C*1e3);
			}
			if(C >= 1)
			{
				sprintf(x_str, "%s = %0.1f ‘", Csp[data->rsrp], C);
			}
		}
		// inductivity string
		if((data->param_vals->measureType == 2) || ((data->param_vals->measureType == 0)&&(data->X >= 0)))
		{
			L = fabs(data->X);
			D = fabs(data->R)/L;
			L = L/(2*M_PI*freqList[getFrequency()]);
			Q = 1/D;
			
			if(data->rsrp == 1)
			{
				L *= (1+D*D);
			}
			
			sprintf(qd_str, "Q = %0.3g", Q);
			
			if(L >= 1e-9 && L < 1e-6)
			{
				sprintf(x_str, "%s = %0.1f н√н", Lsp[data->rsrp], L*1e9);
			}
			if(L >= 1e-6 && L < 1e-5)
			{
				sprintf(x_str, "%s = %0.3fмк√н", Lsp[data->rsrp], L*1e6);
			}			
			if(L >= 1e-5 && L < 1e-4)
			{
				sprintf(x_str, "%s = %0.2fмк√н", Lsp[data->rsrp], L*1e6);
			}			
			if(L >= 1e-4 && L < 1e-3)
			{
				sprintf(x_str, "%s = %0.1fмк√н", Lsp[data->rsrp], L*1e6);
			}
			if(L >= 1e-3 && L < 1e-2)
			{
				sprintf(x_str, "%s = %0.3f м√н", Lsp[data->rsrp], L*1e3);
			}
			if(L >= 1e-2 && L < 1e-1)
			{
				sprintf(x_str, "%s = %0.2f м√н", Lsp[data->rsrp], L*1e3);
			}			
			if(L >= 1e-1 && L < 1)
			{
				sprintf(x_str, "%s = %0.1f м√н", Lsp[data->rsrp], L*1e3);
			}
			if(L >= 1 && L < 10)
			{
				sprintf(x_str, "%s = %0.3f √н", Lsp[data->rsrp], L);
			}
			if(L >= 10 && L < 100)
			{
				sprintf(x_str, "%s = %0.2f √н", Lsp[data->rsrp], L);
			}
			if(L >= 100 && L < 1e3)
			{
				sprintf(x_str, "%s = %0.1f √н", Lsp[data->rsrp], L);
			}
			if(L >= 1e3)
			{
				sprintf(x_str, "%s = %0.1f к√н", Lsp[data->rsrp], L/1e3);
			}
		}
		// resistance string
		R = fabs(data->R);
		if(data->rsrp == 1)
		{
			R *= (1+D*D)/(D*D);
		}
			
		if(R < 1)
		{
			sprintf(r_str, "%s = %0.3f ќм", Rsp[data->rsrp], R);
		}
		if(R >= 1 && R < 100)
		{
			sprintf(r_str, "%s = %0.2f ќм", Rsp[data->rsrp], R);
		}
		if(R >= 100 && R < 1000)
		{
			sprintf(r_str, "%s = %0.1f ќм", Rsp[data->rsrp], R);
		}		
		if(R >= 1e3 && R < 1e4)
		{
			sprintf(r_str, "%s = %0.3f кќм", Rsp[data->rsrp], R/1e3);
		}
		if(R >= 1e4 && R < 1e5)
		{
			sprintf(r_str, "%s = %0.2f кќм", Rsp[data->rsrp], R/1e3);
		}
		if(R >= 1e5 && R < 1e6)
		{
			sprintf(r_str, "%s = %0.1f кќм", Rsp[data->rsrp], R/1e3);
		}
		if(R >= 1e6 && R < 1e7)
		{
			sprintf(r_str, "%s = %0.3f ћќм", Rsp[data->rsrp], R/1e6);
		}
		if(R >= 1e7 && R < 1e8)
		{
			sprintf(r_str, "%s = %0.2f ћќм", Rsp[data->rsrp], R/1e6);
		}
		if(R >= 1e8 && R < 1e9)
		{
			sprintf(r_str, "%s = %0.1f ћќм", Rsp[data->rsrp], R/1e6);
		}
		if(R >= 1e9 && R < 1e10)
		{
			sprintf(r_str, "%s = %0.3f √ќм", Rsp[data->rsrp], R/1e9);
		}
		if(R >= 1e10)
		{
			sprintf(r_str, "%s = %0.1f √ќм", Rsp[data->rsrp], R/1e9);
		}
		//copy strings in display buffer
		String str1 = {0,0,AlignCenter,MSSanSerif_6pt,(const char*)param_str,NotInverted};
		String str3 = {0,39,AlignCenter,font6x8,(const char*)menu_str,Inverted};
		
    wnd->strings[0] = str1;
    wnd->strings[2] = str3;
		
		if(data->Z > 5e7)
		{
				String str2 = {0,19,AlignCenter,font6x8,"Z = inf",NotInverted};
				wnd->strings[1] = str2;
				wnd->StringsQuantity = 3;
		}
		else
		{
			if(getMeasureType() == 1)
			{
				String str2 = {0,19,AlignCenter,font6x8,(const char*)r_str,NotInverted};
				wnd->strings[1] = str2;
				wnd->StringsQuantity = 3;
			}
			else
			{
					String str2 = {0,10,AlignCenter,font6x8,(const char*)r_str,NotInverted};
					String str4 = {0,19,AlignCenter,font6x8,(const char*)x_str,NotInverted};
					String str5 = {0,28,AlignCenter,font6x8,(const char*)qd_str,NotInverted};
					wnd->strings[1] = str2;
					wnd->strings[3] = str4;
					wnd->strings[4] = str5;
					wnd->StringsQuantity = 5;
			}
		}
		DrawLine(0, 8, 83, 8);
    SetWindow(wnd);
    return 1;
}

int DisplaySecondWindow(pWindow wnd, pData data,Action item_action, Action value_action)
{
	char Ux_str[14] = {0}, r_str[14] = {0}, Ur_str[14] = {0}, fi_str[14] = {0};
	
	if(data->Z < 1)
	{
		sprintf(r_str, "Z = %0.3f ќм", data->Z);
	}
	if(data->Z >= 1 && data->Z < 100)
	{
		sprintf(r_str, "Z = %0.2f ќм", data->Z);
	}
	if(data->Z >= 100 && data->Z < 1000)
	{
		sprintf(r_str, "Z = %0.1f ќм", data->Z);
	}		
	if(data->Z >= 1e3 && data->Z < 1e4)
	{
		sprintf(r_str, "Z = %0.3f кќм", data->Z/1e3);
	}
	if(data->Z >= 1e4 && data->Z < 1e5)
	{
		sprintf(r_str, "Z = %0.2f кќм", data->Z/1e3);
	}
	if(data->Z >= 1e5 && data->Z < 1e6)
	{
		sprintf(r_str, "Z = %0.1f кќм", data->Z/1e3);
	}
	if(data->Z >= 1e6 && data->Z < 1e7)
	{
		sprintf(r_str, "Z = %0.3f ћќм", data->Z/1e6);
	}
	if(data->Z >= 1e7)
	{
		sprintf(r_str, "Z = %0.2f ћќм", data->Z/1e6);
	}
	
	sprintf(Ur_str, "Ur = %0.3f¬", data->Ur);
	sprintf(Ux_str, "Ux = %0.3f¬", data->Ux);
	sprintf(fi_str, "fi = %0.2f∞", data->fi);
	
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
		static uint8_t param_current_index;
		uint8_t param_current_index_limit = 3;
	
		if(data->param_vals->isAutoSet)
		{
			param_current_index_limit = 2;
		}
		
    if(item_action == Next)
    {
        if(++param_current_index >= param_current_index_limit)
        {
            param_current_index = 0;
            return 0;
        }
    }

    if(value_action == Next)
    {
        switch(param_current_index)
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

          /* case 2:
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
               break;	*/													
       }
    }
    if(value_action == Prev)
    {
        switch(param_current_index)
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

           /*case 2:
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
               break;*/
       }
    }
		
			 setFrequency(data->param_vals->testSignalFreq);
			 //setUGain(data->param_vals->uGain);
			 //setRsense(data->param_vals->R_sense);
			 setAutoSetParams(data->param_vals->isAutoSet);
		
		char freq_str[4][8] = {"120√ц\0","1к√ц\0","8к√ц\0","62к√ц"};
		//char gain_str[4][5] = {"2\0","5\0","13\0","34\0"};
		//char rs_str[5][7] = {"100ќм\0","1к\0","10к\0","100к\0","10ќм\0"};
		char automode_str[2][6] = {"¬ыкл.\0","¬кл.\0"};
		char low_str1[12] = "-  ƒалее  +";
		
		if(data->param_vals->isAutoSet)
		{
				String str1 = {1,9,AlignLeft,font6x8,"јвто:",NotInverted}; //seconds
				String str2 = {0,9,AlignRight,font6x8,(const char*)automode_str[data->param_vals->isAutoSet],(param_current_index == 0)? (Inverted):(NotInverted)};
				
				wnd->strings[0] = str1;
				wnd->strings[1] = str2;
				
				if(param_current_index == 1)
				{
					String str5 = {0,38,AlignCenter,font6x8,"OK",Inverted};
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
				//if(data->param_current_index < 3)
				//{
					String str1 = {1,9,AlignLeft,font6x8,"јвто:",NotInverted}; //seconds
					String str2 = {0,9,AlignRight,font6x8,(const char*)automode_str[data->param_vals->isAutoSet],(param_current_index == 0)? (Inverted):(NotInverted)};
					String str3 = {0,18,AlignLeft,font6x8,"„астота: ",NotInverted};
					String str4 = {0,18,AlignRight,font6x8,(const char*)freq_str[data->param_vals->testSignalFreq],(param_current_index == 1)? (Inverted):(NotInverted)};
					//String str5 = {1,27,AlignLeft,font6x8," оэф.усил:",NotInverted}; //hours
					//String str6 = {0,27,AlignRight,font6x8,(const char*)gain_str[data->param_vals->uGain],(data->param_current_index == 2)? (Inverted):(NotInverted)}; //week day
					
					wnd->strings[0] = str1;
					wnd->strings[1] = str2;
					wnd->strings[2] = str3;
					wnd->strings[3] = str4;
					
					if(param_current_index == 2)
					{
						String str5 = {0,38,AlignCenter,font6x8,"OK",Inverted};
						wnd->strings[4] = str5;
					}
					else
					{
						String str5 = {0,38,AlignCenter,font6x8,(const char*)low_str1,NotInverted};
						wnd->strings[4] = str5;
					}
					
					wnd->StringsQuantity = 5;
					//wnd->strings[4] = str5;
					//wnd->strings[5] = str6;
				//}
				/*else
				{		
							String str1 = {0,9,AlignLeft,font6x8,"„астота: ",NotInverted};
							String str2 = {0,9,AlignRight,font6x8,(const char*)freq_str[data->param_vals->testSignalFreq],(data->param_current_index == 1)? (Inverted):(NotInverted)}; //minutes
							String str3 = {1,18,AlignLeft,font6x8," -нт.усил:",NotInverted}; //hours
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
			wnd->StringsQuantity = 5;*/
	  }
		
		DrawLine(0,36,83,36);
		
    SetWindow(wnd);
    return 1;
}

int SetupModeWindow(pWindow wnd, pData data,Action item_action, Action value_action)
{
	  static uint8_t mode_current_index;
	
	  if(item_action == Next)
    {
			  if(++mode_current_index >= 3)
        {
            mode_current_index = 0;
            return 0;
        }
		}
		
		if(value_action == Next)
		{
			switch(mode_current_index)
			{
				case 0:
					if(++data->param_vals->measureType > 3)
					{
						data->param_vals->measureType = 0;
					}
					setMeasureType(data->param_vals->measureType);
					break;
					
				case 1:
					data->rsrp++;
					data->rsrp &= 0x01;
					break;
			}
		}
		if(value_action == Prev)
		{
			switch(mode_current_index)
			{
				case 0:
					if(--data->param_vals->measureType >= 128)
					{
						data->param_vals->measureType = 3;
					}
					setMeasureType(data->param_vals->measureType);
					break;
					
				case 1:
					data->rsrp--;
					data->rsrp &= 0x01;
					break;
			}
		}
		
		char low_str1[13] = "<   ƒалее  >\0";
		const char* m_str[4] = {"јвто", "R", "L", "C"};
		
		String str1 = {0,9,AlignLeft,font6x8,"–ежим:",NotInverted};
		String str2 = {39,9,AlignLeft,font6x8, m_str[data->param_vals->measureType],(mode_current_index == 0) ? (Inverted):(NotInverted)};
		String str3 = {0,18,AlignLeft,font6x8,"—хема:",NotInverted};
		String str4 = {0,18,AlignRight,font6x8,(data->rsrp == 0) ? ("ѕослед."):("ѕаралл."), (mode_current_index == 1) ? (Inverted):(NotInverted)};
		
		if(mode_current_index == 2)
		{
			String str5 = {0,38,AlignCenter,font6x8,"OK",Inverted};
			wnd->strings[4] = str5;
		}
		else
		{
			String str5 = {0,38,AlignCenter,font6x8,(const char*)low_str1,NotInverted};
			wnd->strings[4] = str5;
		}

		wnd->strings[0] = str1;
		wnd->strings[1] = str2;
		wnd->strings[2] = str3;
		wnd->strings[3] = str4;

		wnd->StringsQuantity = 5;	
		
		DrawLine(0,36,83,36);

		SetWindow(wnd);

		return 1;
}

int SetupDisplayWindow(pWindow wnd, pData data, Action item_action, Action value_action)
{
		static uint8_t display_current_index;
	
    if(item_action == Next)
    {
        if(++display_current_index > sizeof(data->display_vals))
        {
            display_current_index = 0;
						if(*((uint32_t*)(CALIBRATION_DATA_ADDR+sizeof(calibrationValues))) != ((data->display_vals[0]<<16)|(data->display_vals[0]<<8)|(data->display_vals[2])))
						{
							WriteCalibrationDataToFlash();
						}
            return 0;
        }
    }
    if(value_action == Next)
    {
       switch(display_current_index)
       {
           case 0:
							  if(data->display_vals[display_current_index] < 100)
                {
                    data->display_vals[display_current_index] += 5;
                }
								else
								{
									  data->display_vals[display_current_index] = 0;
								}
               break;

           case 1:
              if(data->display_vals[display_current_index] < 60)
              {
                  data->display_vals[display_current_index] += 10;
              }
							else
							{
								  data->display_vals[display_current_index] = 0;
							}
              break;
							
					 case 2:
						  if(data->display_vals[display_current_index] < 7)
              {
                  data->display_vals[display_current_index] += 1;
              }
							else
							{
								  data->display_vals[display_current_index] = 0;
							}
						  break;
       }
    }
    if(value_action == Prev)
    {
        switch(display_current_index)
        {
            case 0:
                if(data->display_vals[display_current_index] > 0)
                {
                    data->display_vals[display_current_index] -= 5;
                }
							  else
							  {
							 		data->display_vals[display_current_index] = 100;
							  }
                break;

            case 1:
               if(data->display_vals[display_current_index] > 0)
               {
                   data->display_vals[display_current_index] -= 10;
               }
							 else
							 {
								   data->display_vals[display_current_index] = 60;
							 }						
               break;
							 
						case 2:
               if(data->display_vals[display_current_index] > 0)
               {
                   data->display_vals[display_current_index] -= 1;
               }
							 else
							 {
							 	  data->display_vals[display_current_index] = 7;
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
		char low_str1[12] = "-  ƒалее  +";

    sprintf(bright_str,"%d%%",data->display_vals[0]);
		sprintf(light_time,"%02d c",data->display_vals[1]);
    sprintf(cont_str,"%d",data->display_vals[2]);
		
		String str1 = {1,9,AlignLeft,font6x8,"яркость:",NotInverted}; //brightness
		String str2 = {0,9,AlignRight,font6x8,(const char*)bright_str,(display_current_index == 0)? (Inverted):(NotInverted)}; //brightness
		String str3 = {1,18,AlignLeft,font6x8,"¬рем€:", NotInverted}; //time
		String str4 = {0,18,AlignRight,font6x8,(const char*)light_time,(display_current_index == 1)? (Inverted):(NotInverted)}; //time
		String str5 = {1,27,AlignLeft,font6x8," онтраст:", NotInverted}; //contrast
		String str6 = {0,27,AlignRight,font6x8,(const char*)cont_str,(display_current_index == 2)? (Inverted):(NotInverted)}; //contrast
		
		if(display_current_index < 3)
		{
			String str7 = {0,38,AlignCenter,font6x8,(const char*)low_str1,NotInverted};
			wnd->strings[6] = str7;
		}
		else
		{
			String str7 = {0,38,AlignCenter,font6x8,"OK",Inverted};	
			wnd->strings[6] = str7;
		}
		
		DrawLine(0,36,83,36);

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
       sprintf(volt_str,"U = %.2f¬",data->batADC_data[0]);
			
			 if(USB_ON())
			 {
					 sprintf(temp_str,"T = %0.1f∞C",data->batADC_data[2]);
					 sprintf(i_str,"I = %0.0f мј",data->batADC_data[1]);
				 
				 	 String str1 = {0,9,AlignCenter,font6x8,(const char*)volt_str,NotInverted}; //voltage
					 String str2 = {0,18,AlignCenter,font6x8,(const char*)i_str,NotInverted}; // current
					 String str3 = {0,27,AlignCenter,font6x8,(const char*)temp_str,NotInverted}; //temperature
					 String str4 = {0,38,AlignCenter,font6x8,"OK",Inverted};

					 wnd->strings[0] = str1;
					 wnd->strings[1] = str2;
					 wnd->strings[2] = str3;
					 wnd->strings[3] = str4;

           wnd->StringsQuantity = 4;
			 }
			 else
			 {
				 	 String str1 = {0,9,AlignCenter,font6x8,(const char*)volt_str,NotInverted}; //voltage
					 String str2 = {0,38,AlignCenter,font6x8,"OK",Inverted};
					 
					 wnd->strings[0] = str1;
					 wnd->strings[1] = str2;
					 
					 wnd->StringsQuantity = 2;
			 }
			 
			 DrawLine(0,36,83,36);
       SetWindow(wnd);
       return 1;
    }
    else
    {
        return 0;
    }
}

int CalibrationWindow(pWindow wnd, pData data, Action item_action, Action action)
{
	static uint8_t freqIndex, dataIndex, isFirst, calibrationType, isChecked, isEnded;
	static CalibrationVals tempCalVals;
	
	  if(item_action == Next)
    {
			 isFirst = 0;
			 freqIndex = 0;
			 isChecked = 0;
			 calibrationType = 0;
			 if(isEnded)
			 {
					WriteCalibrationDataToFlash();//write calibration data to flash
			 }
			 memset(&tempCalVals, 0, sizeof(tempCalVals));
			 setAutoSetParams(1); //enable autoset params
       return 0;
    }
		else
		{
			if(!isFirst)
			{
				isEnded = 0;
				
				String str1 = {0,18,AlignCenter,font6x8,"–азомкните",NotInverted};
				String str2 = {0, 27,AlignCenter,font6x8,"щупы",NotInverted};
				String str3 = {0, 38,AlignCenter,font6x8,"ќтмена", Inverted};
				
				wnd->strings[0] = str1;
				wnd->strings[1] = str2;
				wnd->strings[2] = str3;
					 
				wnd->StringsQuantity = 3;
					
				setAutoSetParams(0); //disable autoset params
				setMeasureType(0);
				isFirst++;
				return 1;
			}
			
			if(calibrationType == 0) //open probes
			{
				if((data->Z < 5e5) && (!isChecked))
				{
					String str1 = {0,9,AlignCenter,font6x8,"–азомкните",NotInverted};
					String str2 = {0,18,AlignCenter,font6x8,"щупы",NotInverted};
					String str3 = {0, 38,AlignCenter,font6x8,"ќтмена", Inverted};
					
					wnd->strings[0] = str1;
					wnd->strings[1] = str2;
					wnd->strings[2] = str3;
						 
					wnd->StringsQuantity = 3;
				}
				else
				{			
					isChecked = 1;
					if(freqIndex < 4)
					{
						setFrequency(freqIndex);
					}
					
					if(!rlcStabilzation.isStable)
					{
						String str1 = {0,14,AlignCenter,font6x8,"—табилизаци€",NotInverted};
						String str2 = {0,38,AlignCenter,font6x8,"ќтмена", Inverted};
						
						wnd->strings[0] = str1;
						wnd->strings[1] = str2;
						
						wnd->StringsQuantity = 2;
					}
					else
					{
						if(dataIndex < 20 && freqIndex < 4)
						{
							String str1 = {0,9,AlignCenter,font6x8," алибровка:",NotInverted};
							char stepStr[12] = {0};
							sprintf(stepStr,"шаг %d из 4",freqIndex+1);
							String str2 = {0,18,AlignCenter,font6x8, (const char*)stepStr,NotInverted};
							String str3 = {0, 38,AlignCenter,font6x8,"ќтмена", Inverted};
							
							wnd->strings[0] = str1;
							wnd->strings[1] = str2;
							wnd->strings[2] = str3;
							wnd->StringsQuantity = 3;
							
							tempCalVals.Zo[freqIndex].Re += fabs(data->R);
							tempCalVals.Zo[freqIndex].Im += data->X;
							dataIndex++;
						}
						else
						{						
							tempCalVals.Zo[freqIndex].Re /= 20;
							tempCalVals.Zo[freqIndex].Im /= 20;
							
							if(freqIndex < 4)
							{
								freqIndex++;
								dataIndex = 0;
							}
							if(freqIndex == 4)
							{
								freqIndex = 0;
								isChecked = 0;
								dataIndex = 0;
								calibrationType++; // to next calibration type
								setFrequency(0);
								rlcStabilzation.isStable = 0;
							}
						}
					}
				}
			}
			if(calibrationType == 1) //closed probes
			{
				if((data->Z > 0.1) && (!isChecked))
				{
					String str1 = {0,9,AlignCenter,font6x8,"«амкните",NotInverted};
					String str2 = {0,18,AlignCenter,font6x8,"щупы",NotInverted};
					String str3 = {0,38,AlignCenter,font6x8,"ќтмена", Inverted};
					
					wnd->strings[0] = str1;
					wnd->strings[1] = str2;
					wnd->strings[2] = str3;
						 
					wnd->StringsQuantity = 3;
				}
				else
				{			
					isChecked = 1;
					if(freqIndex < 4)
					{
						setFrequency(freqIndex);
					}
					
					if(!rlcStabilzation.isStable)
					{
						String str1 = {0,14,AlignCenter,font6x8,"—табилизаци€",NotInverted};
						String str2 = {0,38,AlignCenter,font6x8,"ќтмена", Inverted};
						
						wnd->strings[0] = str1;
						wnd->strings[1] = str2;
						
						wnd->StringsQuantity = 2;
					}
					else
					{
						if(dataIndex < 20 && freqIndex < 4)
						{
							String str1 = {0,9,AlignCenter,font6x8," алибровка:",NotInverted};
							char stepStr[12] = {0};
							sprintf(stepStr,"шаг %d из 4",freqIndex+1);
							String str2 = {0,18,AlignCenter,font6x8, (const char*)stepStr,NotInverted};
							String str3 = {0, 38,AlignCenter,font6x8,"ќтмена", Inverted};
							
							wnd->strings[0] = str1;
							wnd->strings[1] = str2;
							wnd->strings[2] = str3;
							wnd->StringsQuantity = 3;
							
							tempCalVals.Zc[freqIndex].Re += fabs(data->R);
							tempCalVals.Zc[freqIndex].Im += data->X;
							dataIndex++;
						}
						else
						{						
							tempCalVals.Zc[freqIndex].Re /= 20;
							tempCalVals.Zc[freqIndex].Im /= 20;						
							
							if(freqIndex < 4)
							{
								freqIndex++;
								dataIndex = 0;
							}
							if(freqIndex == 4) //end of calibration
							{
								memcpy(&calibrationValues, &tempCalVals, sizeof(calibrationValues));
								calibrationValues.isCalibrated = 1;
								isEnded = 1;
								
								String str1 = {0,9,AlignCenter,font6x8," алибровка",NotInverted};
								String str2 = {0,18,AlignCenter,font6x8,"завершена!",NotInverted};
								String str3 = {0,38, AlignCenter, font6x8, "—охранить", Inverted};
								DrawLine(0,36,83,36);
								
								wnd->strings[0] = str1;
								wnd->strings[1] = str2;
								wnd->strings[2] = str3;
									 
								wnd->StringsQuantity = 3;
							}
						}
					}
				}
			}
		}
		DrawLine(0,36,83,36);
		SetWindow(wnd);
		return 1;
}

int UpdateFirmwareWindow(pWindow wnd, pData data,Action item_action, Action value_action)
{
	  pFunction JumpToApplication;
		uint32_t JumpAddress = 0;
	
		if(item_action == Next)
		{
			return 0;
		}
	
		if(USB_ON())
		{
			//enableUSB_PullUp(0);
			USBD_Stop(&hUsbDeviceFS);
			USBD_DeInit(&hUsbDeviceFS);
			
			JumpAddress = *(__IO uint32_t*) (USBD_DFU_BOOT_DEFAULT_ADD + 4);
			JumpToApplication = (pFunction) JumpAddress;
			
			/* Initialize user application's Stack Pointer */
			__set_MSP(*(__IO uint32_t*) USBD_DFU_BOOT_DEFAULT_ADD);
			JumpToApplication();
		}
		else
		{
			String str1 = {0,9,AlignCenter,font6x8,"ѕодключите",NotInverted};
			String str2 = {0,18,AlignCenter,font6x8,"USB",NotInverted};
			String str3 = {0,38,AlignCenter,font6x8,"Ќазад", Inverted};
			
			wnd->strings[0] = str1;
			wnd->strings[1] = str2;
			wnd->strings[2] = str3;
				 
			wnd->StringsQuantity = 3;
		}
		DrawLine(0,36,83,36);
		SetWindow(wnd);
		return 1;
}

int SetMenuWindow(pWindow wnd, pData data, Action item_action, Action action)
{
	char title[10] = "Ќастройки";
	String MenuTitle = {0,9,AlignCenter,font6x8,(const char*)title,NotInverted};
	
	const char* Items[7] = {"–ежим","ѕараметры"," алибровка","Ѕатаре€","ƒисплей","ќбновление ѕќ","¬ыход"};
	uint8_t ItemsQuantity = 7;

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

void WindowsInit()
{
	DisplayMainWnd.callback = &DisplayMainWindow;
	DisplaySecondWnd.callback = &DisplaySecondWindow;
	
	DisplayMainWnd.next = &DisplaySecondWnd; DisplayMainWnd.prev = &DisplaySecondWnd;
	DisplaySecondWnd.next = &DisplayMainWnd; DisplaySecondWnd.prev = &DisplayMainWnd;
	
	MenuWnd.callback = &SetMenuWindow;
	
	SetupWnds[0].callback = &SetupModeWindow;
	SetupWnds[1].callback = &SetupParametersWindow;
	SetupWnds[2].callback = &CalibrationWindow;
	SetupWnds[3].callback = &SetBatteryStateWindow;
	SetupWnds[4].callback = &SetupDisplayWindow;
	SetupWnds[5].callback = &UpdateFirmwareWindow;
	
	CurrentWnd = &DisplayMainWnd;
}
