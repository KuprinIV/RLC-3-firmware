#ifndef __RLC_WINDOWS_H
#define __RLC_WINDOWS_H

#include "nokia_5110_lib.h"
#include "rlc.h"

void WindowsInit(void);
void goToNextWindowOrItem(void);
void goToPrevWindowOrItem(void);
void confirmWindowOrItem(void);
void refreshWindow(void);

int DisplayMainWindow(pWindow wnd, pData data,Action item_action, Action value_action);
int DisplaySecondWindow(pWindow wnd, pData data,Action item_action, Action value_action);
int SetMenuWindow(pWindow wnd, pData data, Action item_action, Action action);

int SetupModeWindow(pWindow wnd, pData data,Action item_action, Action value_action);
int SetupParametersWindow(pWindow wnd, pData data, Action item_action, Action value_action);
int CalibrationWindow(pWindow wnd, pData data, Action item_action, Action action);
int SetBatteryStateWindow(pWindow wnd, pData data, Action item_action, Action action);
int SetupDisplayWindow(pWindow wnd, pData data, Action item_action, Action action);
int UpdateFirmwareWindow(pWindow wnd, pData data,Action item_action, Action value_action);

#endif
