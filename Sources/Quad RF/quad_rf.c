#include "quad_rf.h"
#include "quad_rf_disp.h"
#include "nRF24L01+.h"
#include "LCD.h"
#include "fjoy.h"
#include "rti.h"

#define QRF_INVALID_BATT_LEVEL 127

void qrf_fjoyUpdateCallback (void);
void qrf_nrfCallback (bool success, u8 *payloadData, u8 length);
void PeriodicPrint (void *data, rti_time period, rti_id id);

typedef enum
{
	AXIS,
	COMM_AND_BATT
} QRF_SCREEN;

struct 
{
	QRF_SCREEN currScreen;
	u8 fjoyNRFData[4];
	u16 lostPacketsCount;
	u8 battALevel;
	u8 battBLevel;
} qrf_data;

void qrf_Init (void)
{
	nrf_Init(PTX);
	lcd_Init(LCD_2004);
	fjoy_Init();
	
	qrf_data.currScreen = AXIS;
	qrf_data.lostPacketsCount = 0;
	qrf_data.battALevel = QRF_INVALID_BATT_LEVEL;
	qrf_data.battBLevel = QRF_INVALID_BATT_LEVEL;
}

void qrf_PrintCommInfo(void)
{		
	rti_Init();
	rti_Register(PeriodicPrint, NULL, RTI_MS_TO_TICKS(QUAD_RF_DISP_REFRESH_PERIOD_MS), RTI_NOW);
}

void qrf_SendJoyMeasurements(void)
{
	fjoy_CallOnUpdate(qrf_fjoyUpdateCallback);
}

void qrf_fjoyUpdateCallback (void)
{
	qrf_data.fjoyNRFData[0] = fjoy_status.yaw;
	qrf_data.fjoyNRFData[1] = fjoy_status.pitch;
	qrf_data.fjoyNRFData[2] = fjoy_status.roll;
	qrf_data.fjoyNRFData[3] = fjoy_status.elev;
	
	nrf_Transmit(qrf_data.fjoyNRFData, 4, qrf_nrfCallback);
}

void PeriodicPrint (void *data, rti_time period, rti_id id)
{
	switch (qrf_data.currScreen)
	{
		case AXIS:
			qrf_disp_PrintAxes();
			
			break;
			
		case COMM_AND_BATT:
			//qrf_disp_PrintCommAndBatt(qrf_data.lostPacketsCount, qrf_data.battALevel, qrf_data.battBLevel);
			break;
	}
}

void qrf_nrfCallback (bool success, u8 *payloadData, u8 length)
{
	if (success != _TRUE)
		qrf_data.lostPacketsCount++;
			
	if (payloadData != NULL)
	{
		qrf_data.battALevel = payloadData[0];
		qrf_data.battBLevel = payloadData[1];
	}
}