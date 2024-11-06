#ifndef __RLC_WINDOWS_H
#define __RLC_WINDOWS_H

#include "nokia_5110_lib.h"
#include "rlc.h"

void WindowsInit(void);
void goToNextWindowOrItem(void);
void goToPrevWindowOrItem(void);
void confirmWindowOrItem(void);
void refreshWindow(void);

static int DisplayMainWindow(pWindow wnd, pData data,Action item_action, Action value_action);
static int DisplaySecondWindow(pWindow wnd, pData data,Action item_action, Action value_action);
int SetMenuWindow(pWindow wnd, pData data, Action item_action, Action action);

static int SetupModeWindow(pWindow wnd, pData data,Action item_action, Action value_action);
static int SetupParametersWindow(pWindow wnd, pData data, Action item_action, Action value_action);
static int CalibrationWindow(pWindow wnd, pData data, Action item_action, Action value_action);
static int SetBatteryStateWindow(pWindow wnd, pData data, Action item_action, Action value_action);
static int SetupDisplayWindow(pWindow wnd, pData data, Action item_action, Action value_action);
static int UpdateFirmwareWindow(pWindow wnd, pData data,Action item_action, Action value_action);
static int FirmwareVersionWindow(pWindow wnd, pData data,Action item_action, Action value_action);
static int CalibrationProbesWindow(pWindow wnd, pData data, Action item_action, Action value_action);
static int CalibrationRsenseWindow(pWindow wnd, pData data, Action item_action, Action value_action);

#endif
