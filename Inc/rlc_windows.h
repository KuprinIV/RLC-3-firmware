#ifndef __RLC_WINDOWS_H
#define __RLC_WINDOWS_H

#include "nokia_5110_lib.h"
#include "rlc.h"

int SetupModeWindow(pWindow wnd, pData data,Action item_action, Action value_action);
int SetupParametersWindow(pWindow wnd, pData data, Action item_action, Action value_action);
int SetupDisplayWindow(pWindow wnd, pData data, Action item_action, Action action);
int SetBatteryStateWindow(pWindow wnd, pData data, Action item_action, Action action);

int SetMenuWindow(pWindow wnd, pData data, Action item_action, Action action);
int CalibrationWindow(pWindow wnd, pData data, Action item_action, Action action);

int DisplayMainWindow(pWindow wnd, pData data,Action item_action, Action value_action);
int DisplaySecondWindow(pWindow wnd, pData data,Action item_action, Action value_action);
int UpdateFirmwareWindow(pWindow wnd, pData data,Action item_action, Action value_action);

void WindowsInit(void);
#endif
