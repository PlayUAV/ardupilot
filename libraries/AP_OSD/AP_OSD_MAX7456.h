/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifndef __AP_OSD_MAX7456_H__
#define __AP_OSD_MAX7456_H__

#include <AP_HAL.h>
#include <AP_Param.h>
#include "AP_OSD_Stream.h"

#define HORIZON_LEN	11

class AP_OSD_MAX7456 : public AP_OSD_Stream
{
public:
	AP_OSD_MAX7456();

	bool            init();

	//update charsets
	void			write_NVM(uint32_t font_count, uint8_t *character_bitmap);
	//void			read_one_char_from_NVM(uint32_t font_count);

	//update the screen
	void			updateScreen();

	//clear the display screen
	void			clear(void);

	void			setPanel(uint8_t start_col, uint8_t start_row);
	void			openPanel(void);
	void			closePanel(void);
	void			openSingle(uint8_t x, uint8_t y);

	

	//the implementations of AP_OSD_STREAM::write
	virtual size_t write(uint8_t c);

	static const struct AP_Param::GroupInfo var_info[];
	uint8_t  osd_frame_type;	//0:copter 1:plan. This value be set at sys.init()
public:
	// screen display variable
	float		_groundSpeed;
	uint16_t	_throttle;
	float		_altitude;
	int8_t		_pitch;
	int8_t		_roll;
	int32_t		_homeDirection;	// Arrow direction pointing to home (1-16 to CW loop)
	int32_t		_homeDistance;
	uint8_t		_flyMode;
	float		_startTime;
	uint8_t		_GPSSats;
	float		_GPSLongitude;
	float		_GPSLatitude;
	float		_GPSLongitudePrint;
	float		_GPSLatitudePrint;
	float		_BatteryVol;
	float		_BatteryCurrent;
	uint8_t		_BatteryPercent;
	float		_BatteryConsum;
	int32_t		_WPDirection;
	int32_t		_WPDistance;
	float		_heading;
	uint8_t		_iMotorArmed;
	uint8_t		_iGPSStatus;
	uint8_t		_iRSSI;
protected:
	//parameters which can be set from the ground station
	AP_Int8		_bEnableSpeed;				//if show the speed
	AP_Int8     _iSpdX;
	AP_Int8     _iSpdY;

	AP_Int8		_bEnableAlt;	
	AP_Int8		_iAltX;
	AP_Int8		_iAltY;

	AP_Int8		_bEnableThrottle;
	AP_Int8		_iThrotX;
	AP_Int8		_iThrotY;

	AP_Int8		_bEnablePitch;
	AP_Int8		_iPichX;
	AP_Int8		_iPichY;

	AP_Int8		_bEnableRoll;
	AP_Int8		_iRolX;
	AP_Int8		_iRolY;

	AP_Int8		_bEnableHome;
	AP_Int8		_iHomeX;
	AP_Int8		_iHomeY;

	AP_Int8		_bEnableMode;
	AP_Int8		_iModX;
	AP_Int8		_iModY;

	AP_Int8		_bEnableTime;
	AP_Int8		_iTimeX;
	AP_Int8		_iTimeY;

	AP_Int8		_bEnableHorizon;
	AP_Int8		_iHoriX;
	AP_Int8		_iHoriY;

	AP_Int8		_bEnableGPSSats;
	AP_Int8		_iGPSSatsX;
	AP_Int8		_iGPSSatsY;

	AP_Int8		_bEnableGPSCoord;
	AP_Int8     _iGPSCoordX;
	AP_Int8     _iGPSCoordY;

	AP_Int8		_bEnableBattVol;
	AP_Int8     _iBatVolX;
	AP_Int8     _iBatVolY;

	AP_Int8		_bEnableBattCur;
	AP_Int8     _iBatCurX;
	AP_Int8     _iBatCurY;

	AP_Int8		_bEnableBattPercent;
	AP_Int8     _iBatPerX;
	AP_Int8     _iBatPerY;

	AP_Int8		_bEnableWP;
	AP_Int8     _iWPX;
	AP_Int8     _iWPY;

	AP_Int8		_bEnableHead;
	AP_Int8     _iHeadX;
	AP_Int8     _iHeadY;

	AP_Int8		_bEnableHeadRose;
	AP_Int8     _iHeadRoseX;
	AP_Int8     _iHeadRoseY;

	AP_Int8		_iMode;
	AP_Int8     _iModeX;
	AP_Int8     _iModeY;

	
	AP_Int8		_iEnableCurConsume;
	AP_Int8		_iCurCsmX;
	AP_Int8		_iCurCsmY;

	AP_Int8		_iEnableRSSI;
	AP_Int8		_iRSSIX;
	AP_Int8		_iRSSIY;
	AP_Int8		_iRSSIRaw;
	AP_Int8		_iRSSIMin;
	AP_Int16	_iRSSIMax;
	AP_Int8		_iLanguage;			//0:English 1:Chinese

private:
	void showArrow(uint8_t rotate_arrow, uint8_t mode);	
	void showHorizon(uint8_t start_col, uint8_t start_row);
	void showWarning();
	void printHit(uint8_t col, uint8_t row, uint8_t subval);
	void showAt10HZ();
	void showAt3HZ();
	void showAt1HZ();

	AP_HAL::SPIDeviceDriver *_spi;
	AP_HAL::Semaphore *_spi_sem;
	
	uint8_t start_col, start_row, col, row, _video_mode;
	uint32_t		_lastUpdate10HZ;
	uint32_t		_lastUpdate3HZ;
	uint32_t		_lastUpdate1HZ;
	uint8_t			_lastHorizonColHit[HORIZON_LEN];
	uint8_t			_lastHorizonRowHit[HORIZON_LEN];
	uint8_t			_HorizonHitIndex;

	
};

#endif //  __AP_OSD_MAX7456_H__
