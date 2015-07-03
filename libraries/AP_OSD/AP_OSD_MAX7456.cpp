/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include <AP_HAL.h>
#include "AP_OSD_MAX7456.h"
#include <AP_Math.h>

extern const AP_HAL::HAL& hal;


/******* MAX7456 DATASHEET BEGIN*******/
#define NTSC 0
#define PAL 1
#define MAX7456_MODE_MASK_PAL 0x40 //PAL mask 01000000
#define MAX7456_CENTER_PAL 0x8

#define MAX7456_MODE_MASK_NTCS 0x00 //NTSC mask 00000000 ("|" will do nothing)
#define MAX7456_CENTER_NTSC 0x6

//MAX7456 reg read addresses
#define MAX7456_OSDBL_reg_read 0xec //black level
#define MAX7456_STAT_reg_read  0xa0 //0xa[X] Status

//MAX7456 reg write addresses
#define MAX7456_VM0_reg   0x00
#define MAX7456_VM1_reg   0x01
#define MAX7456_DMM_reg   0x04
#define MAX7456_DMAH_reg  0x05
#define MAX7456_DMAL_reg  0x06
#define MAX7456_DMDI_reg  0x07
#define MAX7456_OSDM_reg  0x0c //not used. Is to set mix
#define MAX7456_OSDBL_reg 0x6c //black level

//MAX7456 reg write addresses to recording NVM process
#define MAX7456_CMM_reg   0x08
#define MAX7456_CMAH_reg  0x09
#define MAX7456_CMAL_reg  0x0a
#define MAX7456_CMDI_reg  0x0b
#define MAX7456_CMDO_reg  0xc0

//DMM commands
#define MAX7456_CLEAR_display 0x04
#define MAX7456_CLEAR_display_vert 0x06

#define MAX7456_INCREMENT_auto 0x03
#define MAX7456_SETBG_local 0x20 //00100000 force local BG to defined brightness level VM1[6:4]

#define MAX7456_END_string 0xff

//VM0 commands mixed with mode NTSC or PAL mode
#define MAX7456_ENABLE_display_vert 0x0c //mask with NTSC/PAL
#define MAX7456_RESET 0x02 //mask with NTSC/PAL
#define MAX7456_DISABLE_display 0x00 //mask with NTSC/PAL

//VM0 command modifiers
#define MAX7456_SYNC_autosync 0x10
#define MAX7456_SYNC_internal 0x30
#define MAX7456_SYNC_external 0x20
//VM1 command modifiers
#define MAX7456_WHITE_level_80 0x03
#define MAX7456_WHITE_level_90 0x02
#define MAX7456_WHITE_level_100 0x01
#define MAX7456_WHITE_level_120 0x00

#define NVM_ram_size 0x36
#define WRITE_nvr 0xa0
#define STATUS_reg_nvr_busy 0x20

//#define isPAL

////If PAL
//#ifdef isPAL
//#define MAX7456_screen_size 480 //16x30
//#define MAX7456_screen_rows 15
//#else
//#define MAX7456_screen_size 390 //13x30
//#define MAX7456_screen_rows 12
//#endif

/******* MAX7456 DATASHEET END*******/

const AP_Param::GroupInfo AP_OSD_MAX7456::var_info[] PROGMEM = {

	AP_GROUPINFO("SPEED", 0, AP_OSD_MAX7456, _bEnableSpeed, 1),
	AP_GROUPINFO("SPD_X", 1, AP_OSD_MAX7456, _iSpdX, 1),
	AP_GROUPINFO("SPD_Y", 2, AP_OSD_MAX7456, _iSpdY, 8),

	AP_GROUPINFO("ALTITUDE", 3, AP_OSD_MAX7456, _bEnableAlt, 1),
	AP_GROUPINFO("ALT_X", 4, AP_OSD_MAX7456, _iAltX, 21),
	AP_GROUPINFO("ALT_Y", 5, AP_OSD_MAX7456, _iAltY, 8),

	AP_GROUPINFO("THROTTLE", 6, AP_OSD_MAX7456, _bEnableThrottle, 1),
	AP_GROUPINFO("THROTTLE_X", 7, AP_OSD_MAX7456, _iThrotX, 1),
	AP_GROUPINFO("THROTTLE_Y", 8, AP_OSD_MAX7456, _iThrotY, 2),

	AP_GROUPINFO("PITCH", 9, AP_OSD_MAX7456, _bEnablePitch, 1),
	AP_GROUPINFO("PITCH_X", 10, AP_OSD_MAX7456, _iPichX, 8),
	AP_GROUPINFO("PITCH_Y", 11, AP_OSD_MAX7456, _iPichY, 2),

	AP_GROUPINFO("ROLL", 12, AP_OSD_MAX7456, _bEnableRoll, 1),
	AP_GROUPINFO("ROLL_X", 13, AP_OSD_MAX7456, _iRolX, 14),
	AP_GROUPINFO("ROLL_Y", 14, AP_OSD_MAX7456, _iRolY, 2),

	AP_GROUPINFO("HOME", 15, AP_OSD_MAX7456, _bEnableHome, 1),
	AP_GROUPINFO("HOME_X", 16, AP_OSD_MAX7456, _iHomeX, 22),
	AP_GROUPINFO("HOME_Y", 17, AP_OSD_MAX7456, _iHomeY, 3),

	AP_GROUPINFO("MODE", 18, AP_OSD_MAX7456, _bEnableMode, 1),
	AP_GROUPINFO("MODE_X", 19, AP_OSD_MAX7456, _iModX, 21),
	AP_GROUPINFO("MODE_Y", 20, AP_OSD_MAX7456, _iModY, 5),

	AP_GROUPINFO("TIME", 21, AP_OSD_MAX7456, _bEnableTime, 1),
	AP_GROUPINFO("TIME_X", 22, AP_OSD_MAX7456, _iTimeX, 22),
	AP_GROUPINFO("TIME_Y", 23, AP_OSD_MAX7456, _iTimeY, 6),

	AP_GROUPINFO("HORIZON", 24, AP_OSD_MAX7456, _bEnableHorizon, 1),
	AP_GROUPINFO("HORIZON_X", 25, AP_OSD_MAX7456, _iHoriX, 8),
	AP_GROUPINFO("HORIZON_Y", 26, AP_OSD_MAX7456, _iHoriY, 6),

	AP_GROUPINFO("GPS_SATS", 27, AP_OSD_MAX7456, _bEnableGPSSats, 1),
	AP_GROUPINFO("GPS_SATS_X", 28, AP_OSD_MAX7456, _iGPSSatsX, 1),
	AP_GROUPINFO("GPS_SATS_Y", 29, AP_OSD_MAX7456, _iGPSSatsY, 10),

	AP_GROUPINFO("GPS_COORD", 30, AP_OSD_MAX7456, _bEnableGPSCoord, 1),
	AP_GROUPINFO("GPS_COORD_X", 31, AP_OSD_MAX7456, _iGPSCoordX, 1),
	AP_GROUPINFO("GPS_COORD_Y", 32, AP_OSD_MAX7456, _iGPSCoordY, 11),

	AP_GROUPINFO("BATT_VOL", 33, AP_OSD_MAX7456, _bEnableBattVol, 1),
	AP_GROUPINFO("BATT_VOL_X", 34, AP_OSD_MAX7456, _iBatVolX, 21),
	AP_GROUPINFO("BATT_VOL_Y", 35, AP_OSD_MAX7456, _iBatVolY, 9),

	AP_GROUPINFO("BATT_CUR", 36, AP_OSD_MAX7456, _bEnableBattCur, 1),
	AP_GROUPINFO("BATT_CUR_X", 37, AP_OSD_MAX7456, _iBatCurX, 22),
	AP_GROUPINFO("BATT_CUR_Y", 38, AP_OSD_MAX7456, _iBatCurY, 10),

	AP_GROUPINFO("BATT_PER", 39, AP_OSD_MAX7456, _bEnableBattPercent, 1),
	AP_GROUPINFO("BATT_PER_X", 40, AP_OSD_MAX7456, _iBatPerX, 24),
	AP_GROUPINFO("BATT_PER_Y", 41, AP_OSD_MAX7456, _iBatPerY, 11),

	AP_GROUPINFO("WP", 42, AP_OSD_MAX7456, _bEnableWP, 1),
	AP_GROUPINFO("WP_X", 43, AP_OSD_MAX7456, _iWPX, 1),
	AP_GROUPINFO("WP_Y", 44, AP_OSD_MAX7456, _iWPY, 4),

	AP_GROUPINFO("HEAD", 45, AP_OSD_MAX7456, _bEnableHead, 1),
	AP_GROUPINFO("HEAD_ROSE", 46, AP_OSD_MAX7456, _bEnableHeadRose, 1),

	AP_GROUPINFO("BATT_CON", 47, AP_OSD_MAX7456, _iEnableCurConsume, 1),
	AP_GROUPINFO("BATT_CON_X", 48, AP_OSD_MAX7456, _iCurCsmX, 21),
	AP_GROUPINFO("BATT_CON_Y", 49, AP_OSD_MAX7456, _iCurCsmY, 12),

	AP_GROUPINFO("VIDEO_MODE", 50, AP_OSD_MAX7456, _iMode, 1),
	//AP_GROUPINFO("RSSI", 18, AP_OSD_MAX7456, _iEnableRSSI, 1),
	AP_GROUPINFO("LANGUAGE", 51, AP_OSD_MAX7456, _iLanguage, 1),

	AP_GROUPEND
};

AP_OSD_MAX7456::AP_OSD_MAX7456()
:_spi(NULL),
_spi_sem(NULL),
_groundSpeed(0.0),
_throttle(0),
_altitude(0.0),
_pitch(0),
_roll(0),
_homeDirection(0),
_homeDistance(0),
_flyMode(0),
_startTime(0),
_GPSSats(0),
_GPSLongitude(0.0),
_GPSLatitude(0.0),
_GPSLongitudePrint(0.0),
_GPSLatitudePrint(0.0),
_BatteryVol(0.0),
_BatteryCurrent(0.0),
_BatteryPercent(0),
_WPDirection(0),
_WPDistance(0),
_iMotorArmed(0),
_iGPSStatus(0)
{
	AP_Param::setup_object_defaults(this, var_info);

	//default
	_video_mode = MAX7456_MODE_MASK_PAL;

	_heading = 0.0;
}

// SPI should be initialized externally
bool AP_OSD_MAX7456::init()
{
	_spi = hal.spi->device(AP_HAL::SPIDevice_MAX7456Onboard);
	_spi_sem = _spi->get_semaphore();

	//Test
	//read_one_char_from_NVM(1);

	if (!_spi_sem->take(100)){
		hal.scheduler->panic(PSTR("PANIC: AP_OSD_MAX7456: failed to take "
			"serial semaphore for init"));
		return false; /* never reached */
	}


	// set mode. we will do auto bottom-align, because NTCS mode only has 12 rows.
	if(_iMode ==0)
	{
		_video_mode = MAX7456_MODE_MASK_NTCS;
	}
	else
	{
		_video_mode = MAX7456_MODE_MASK_PAL;
	}

	_spi->cs_assert();
	{
		//read black level register
		_spi->transfer(MAX7456_OSDBL_reg_read);
		uint8_t osdbl_r = _spi->transfer(0xff);

		_spi->transfer(MAX7456_VM0_reg);
		_spi->transfer(MAX7456_RESET | _video_mode);
		hal.scheduler->delay(50);

		//set black level
		uint8_t osdbl_w = (osdbl_r & 0xef);	//Set bit 4 to zero 11101111
		_spi->transfer(MAX7456_OSDBL_reg); //black level write register
		_spi->transfer(osdbl_w);

		// set all rows to same charactor white level, 90%
		// no matter what the video mode, we just assume the max rows is 15(PAL mode)
		for (uint8_t x = 0; x < 15; x++)
		{
			_spi->transfer(x + 0x10);
			_spi->transfer(MAX7456_WHITE_level_120);
		}
	}
	_spi->cs_release();

	_spi->cs_assert();
	{
		// define sync (auto,int,ext) and making sure the Max7456 is enabled
		_spi->transfer(MAX7456_VM0_reg);
		_spi->transfer((MAX7456_ENABLE_display_vert | _video_mode) | MAX7456_SYNC_autosync);
	}
	_spi->cs_release();

	// now that we have initialised, we set the SPI bus speed to high
	// (8MHz on APM2)
	//_spi->set_bus_speed(AP_HAL::SPIDeviceDriver::SPI_SPEED_HIGH);

	_spi_sem->give();

	for(_HorizonHitIndex=0;_HorizonHitIndex < HORIZON_LEN; _HorizonHitIndex++)
	{
		_lastHorizonColHit[_HorizonHitIndex] = _iHoriX + 1;
		_lastHorizonRowHit[_HorizonHitIndex] = _iHoriY;
	}

	clear();

	uint32_t nowtime = hal.scheduler->millis();
	_lastUpdate10HZ = nowtime;
	_lastUpdate3HZ = nowtime;
	_lastUpdate1HZ = nowtime;

	return true;
}

// clear the screen
void AP_OSD_MAX7456::clear()
{
	if (!_spi_sem->take(10)){
		hal.console->printf_P(PSTR("AP_OSD_MAX7456::clear() can not get sem\n"));
		return ;
	}

	_spi->cs_assert();
	{
		_spi->transfer(MAX7456_DMM_reg);
		_spi->transfer(MAX7456_CLEAR_display);
	}
	_spi->cs_release();

	//hal.scheduler->delay(50);

	setPanel(_iHoriX, _iHoriY);
	openPanel();
	printf_P(PSTR("\xDA\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xDB|"));
	printf_P(PSTR("\xDA\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xDB|"));
	printf_P(PSTR("\xD8\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xD9|"));
	printf_P(PSTR("\xDA\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xDB|"));
	printf_P(PSTR("\xDA\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\xDB"));
	closePanel();

	_spi_sem->give();
}

void AP_OSD_MAX7456::setPanel(uint8_t st_col, uint8_t st_row)
{
	start_col = st_col;
	start_row = st_row;
	col = st_col;
	row = st_row;
}

void AP_OSD_MAX7456::openPanel(void)
{
	unsigned int linepos;
	uint8_t settings, char_address_hi, char_address_lo;

	//find [start address] position
	linepos = row*30+col;

	// divide 16 bits into hi & lo uint8_t
	char_address_hi = linepos >> 8;
	char_address_lo = linepos;

	//Auto increment turn writing fast (less SPI commands).
	//No need to set next char address. Just send them
	settings = MAX7456_INCREMENT_auto; //To Enable DMM Auto Increment

	_spi->cs_assert();

	_spi->transfer(MAX7456_DMM_reg); //dmm
	_spi->transfer(settings);

	_spi->transfer(MAX7456_DMAH_reg); // set start address high
	_spi->transfer(char_address_hi);

	_spi->transfer(MAX7456_DMAL_reg); // set start address low
	_spi->transfer(char_address_lo);
}

void AP_OSD_MAX7456::closePanel(void){  
	_spi->transfer(MAX7456_DMDI_reg);
	_spi->transfer(MAX7456_END_string); //This is needed "trick" to finish auto increment

	_spi->cs_release();

	row++; //only after finish the auto increment the new row will really act as desired
}

void AP_OSD_MAX7456::openSingle(uint8_t x, uint8_t y){
	unsigned int linepos;
	uint8_t char_address_hi, char_address_lo;

	//find [start address] position
	linepos = y*30+x;

	// divide 16 bits into hi & lo uint8_t
	char_address_hi = linepos >> 8;
	char_address_lo = linepos;

	_spi->cs_assert();

	_spi->transfer(MAX7456_DMAH_reg); // set start address high
	_spi->transfer(char_address_hi);

	_spi->transfer(MAX7456_DMAL_reg); // set start address low
	_spi->transfer(char_address_lo);
}

size_t AP_OSD_MAX7456::write(uint8_t c)
{
	if(c == '|'){
		closePanel(); //It does all needed to finish auto increment and change current row
		openPanel(); //It does all needed to re-enable auto increment
	}
	else{
		_spi->transfer(MAX7456_DMDI_reg);
		_spi->transfer(c);
	}
	return 1;
}

void AP_OSD_MAX7456::updateScreen()
{
	uint32_t nowTime = hal.scheduler->millis();

	if((nowTime - _lastUpdate10HZ) > 100)
	{
		showAt10HZ();
		_lastUpdate10HZ = nowTime;
	}

	if((nowTime - _lastUpdate3HZ) > 330)
	{
		showAt3HZ();
		_lastUpdate3HZ = nowTime;
	}

	if((nowTime - _lastUpdate1HZ) > 1000)
	{
		showAt1HZ();
		_lastUpdate1HZ = nowTime;
	}
}

void AP_OSD_MAX7456::showAt10HZ()
{
	// SPI select max7456 
	if (!_spi_sem->take_nonblocking()){
		hal.console->printf_P(PSTR("AP_OSD_MAX7456::showAt10HZ() can not get sem\n"));
		return ;
	}

	// heading degree
	if(_bEnableHead)
	{
		setPanel(12, 5);
		openPanel();
		printf("%4.0f%c", (double)_heading, 0xb0);
		closePanel();
	}

	//heading direction
	static char buf_show[12];
	static const char buf_Rule[36] = { 0xf2,0xf0,0xf0,0xf1,0xf0,0xf0,0xf1,0xf0,0xf0,
		0xf4,0xf0,0xf0,0xf1,0xf0,0xf0,0xf1,0xf0,0xf0,
		0xf3,0xf0,0xf0,0xf1,0xf0,0xf0,0xf1,0xf0,0xf0,
		0xf5,0xf0,0xf0,0xf1,0xf0,0xf0,0xf1,0xf0,0xf0};
	static const char buf_Rule_en[36] = { 0x2A,0xf0,0xf0,0xf1,0xf0,0xf0,0xf1,0xf0,0xf0,
		0x2B,0xf0,0xf0,0xf1,0xf0,0xf0,0xf1,0xf0,0xf0,
		0x2C,0xf0,0xf0,0xf1,0xf0,0xf0,0xf1,0xf0,0xf0,
		0x26,0xf0,0xf0,0xf1,0xf0,0xf0,0xf1,0xf0,0xf0};


	if(_bEnableHeadRose)
	{
		int8_t start;
		start = round((_heading * 36)/360);
		start -= 5;
		if(start < 0) start += 36;
		for(int8_t x=0; x <= 10; x++)
		{
			if(_iLanguage == 0){
				buf_show[x] = buf_Rule_en[start];
			}
			else{
				buf_show[x] = buf_Rule[start];
			}

			if(++start > 35) start = 0;
		}
		buf_show[11] = '\0';
		setPanel(8, 3);
		openPanel();
		printf("%s|%c%s%c", "\x20\xf0\xf0\xf0\xf0\xf0\xf7\xf0\xf0\xf0\xf0\xf0\x20", 0xf8, buf_show, 0xf9);
		closePanel();
	}

	// pitch, size 1 x 6
	// -+ value of current Pitch from vehicle with degree symbols and pitch symbol
	if(_bEnablePitch)
	{
		setPanel(_iPichX, _iPichY);
		openPanel();
		printf("%4i%c%c",_pitch,0xb0,0xb1);
		closePanel();
	}

	// roll, size 1 x 6
	// -+ value of current Roll from vehicle with degree symbols and roll symbol
	if(_bEnableRoll)
	{
		setPanel(_iRolX, _iRolY);
		openPanel();
		printf("%4i%c%c",_roll, 0xb0, 0xb2);
		closePanel();
	}

	if(_bEnableHorizon)
	{
		showHorizon(_iHoriX + 1, _iHoriY);
		_spi->cs_release();
	}

	//SPI release
	_spi_sem->give();
}


void AP_OSD_MAX7456::showAt3HZ()
{
	// SPI select max7456 
	if (!_spi_sem->take_nonblocking()){
		hal.console->printf_P(PSTR("AP_OSD_MAX7456::showAt3HZ() can not get sem\n"));
		return ;
	}

	// velocity, size 1 x 7
	if(_bEnableSpeed)
	{
		setPanel(_iSpdX, _iSpdY);
		openPanel();
		if(_iLanguage == 0){
			printf("%c%3.0f%c", 0x86, (double)(_groundSpeed * 3.6), 0x81);
		}
		else{
			printf("%c%3.0f%c",0xBC, (double)(_groundSpeed * 3.6), 0x81);
		}
		closePanel();
	}

	// throttle, size 1 x 7
	if(_bEnableThrottle)
	{
		setPanel(_iThrotX, _iThrotY);
		openPanel();
		printf("%c%3.0i%c", 0xE1, _throttle, 0x25);
		closePanel();
	}

	// altitude, size 1 x 7
	if(_bEnableAlt)
	{
		setPanel(_iAltX, _iAltY);
		openPanel();
		if(_iLanguage == 0){
			printf("%c%4.0f%c", 0x85, (double)_altitude, 0x8D);
		}
		else{
			printf("%c%4.0f%c", 0xC0, (double)_altitude, 0x8D);
		}
		closePanel();
	}

	// number of locked satellites, size 1 x 5
	if(_bEnableGPSSats)
	{
		setPanel(_iGPSSatsX, _iGPSSatsY);
		openPanel();
		printf("%c%2i", 0x0f, _GPSSats);
		closePanel();
	}

	// GPS Longitude and Latitude, size 2 x 12
	if(_bEnableGPSCoord)
	{
		setPanel(_iGPSCoordX, _iGPSCoordY);
		openPanel();
		if(_iLanguage == 0){
			printf("%c%11.6f|%c%11.6f", 0x84, (double)_GPSLongitudePrint*1000, 0x83, (double)_GPSLatitudePrint*1000);
		}
		else{
			printf("%c%11.6f|%c%11.6f", 0xC1, (double)_GPSLongitudePrint*1000, 0xC2, (double)_GPSLatitudePrint*1000);
		}
		closePanel();
	}


	//SPI release
	_spi_sem->give();
}

void AP_OSD_MAX7456::showAt1HZ()
{
	// SPI select max7456 
	if (!_spi_sem->take_nonblocking()){
		hal.console->printf_P(PSTR("AP_OSD_MAX7456::showAt1HZ() can not get sem\n"));
		return ;
	}

	//showWarning();

	if(_bEnableHome)
	{
		// home direction, size 1 x 5
		setPanel(_iHomeX, _iHomeY);
		openPanel();
		uint8_t home_bearing = round(((float)_homeDirection - _heading)/360.0 * 16.0) + 1; //Convert to int 0-16 
		if(home_bearing < 0 ) home_bearing += 16; //normalize
		showArrow((uint8_t)home_bearing,0);
		closePanel();

		// home distance, size 1 x 5
		setPanel(_iHomeX, _iHomeY+1);
		openPanel();
		printf("%5.0f%c",(double)(_homeDistance*0.01f), 0x8D);
		closePanel();
	}

	// flight mode 
	// TODO - ugly? fixme!
	setPanel(_iModX, _iModY);
	openPanel();
	char* mode_str = "";
	if(osd_frame_type == 1)		//is plan
	{
		if (_flyMode == 0)       mode_str = "manu"; //Manual
		else if (_flyMode == 1)  mode_str = "circ"; //Circle
		else if (_flyMode == 2)  mode_str = "stab"; //Stabilize
		else if (_flyMode == 3)  mode_str = "trng"; //Training
		else if (_flyMode == 4)  mode_str = "acro"; //Acro
		else if (_flyMode == 5)  mode_str = "fbwa"; //Fly_By_Wire_A
		else if (_flyMode == 6)  mode_str = "fbwb"; //Fly_By_Wire_B
		else if (_flyMode == 7)  mode_str = "crui"; //Cruise
		else if (_flyMode == 8)  mode_str = "atun"; //Auto Tune
		else if (_flyMode == 10) mode_str = "auto"; //Auto
		else if (_flyMode == 11) mode_str = "retl"; //Return to Launch
		else if (_flyMode == 12) mode_str = "loit"; //Loiter
		else if (_flyMode == 15) mode_str = "guid"; //Guided
		else if (_flyMode == 16) mode_str = "init"; //Initializing
	}
	else
	{
		if (_flyMode == 0)       mode_str = "stab"; //Stabilize: hold level position
		else if (_flyMode == 1)  mode_str = "acro"; //Acrobatic: rate control
		else if (_flyMode == 2)  mode_str = "alth"; //Altitude Hold: auto control
		else if (_flyMode == 3)  mode_str = "auto"; //Auto: auto control
		else if (_flyMode == 4)  mode_str = "guid"; //Guided: auto control
		else if (_flyMode == 5)  mode_str = "loit"; //Loiter: hold a single location
		else if (_flyMode == 6)  mode_str = "retl"; //Return to Launch: auto control
		else if (_flyMode == 7)  mode_str = "circ"; //Circle: auto control
		else if (_flyMode == 8)  mode_str = "posi"; //Position: auto control
		else if (_flyMode == 9)  mode_str = "land"; //Land:: auto control
		else if (_flyMode == 10) mode_str = "oflo"; //OF_Loiter: hold a single location using optical flow sensor
		else if (_flyMode == 11) mode_str = "drif"; //Drift mode: 
		else if (_flyMode == 13) mode_str = "sprt"; //Sport: earth frame rate control
		else if (_flyMode == 14) mode_str = "flip"; //Flip: flip the vehicle on the roll axis
		else if (_flyMode == 15) mode_str = "atun"; //Auto Tune: autotune the vehicle's roll and pitch gains
		else if (_flyMode == 16) mode_str = "posh"; //Hybrid: position hold with manual override
	}

	if(_iLanguage == 0){
		printf("%c%s", 0xE0, mode_str);
	}
	else{
		printf("%c%c%s", 0xC7, 0xC8, mode_str);
	}


	closePanel();

	//  flight time from start
	if(_bEnableTime)
	{
		setPanel(_iTimeX, _iTimeY);
		openPanel();
		_startTime = hal.scheduler->millis()*0.001f;
		printf("%c%2i%c%02i", 0xB3, ((int)_startTime/60)%60,0x3A,(int)_startTime%60);
		closePanel();
	}

	// Total battery current consume since start up in amp/h
	if(_iEnableCurConsume)
	{
		setPanel(_iCurCsmX, _iCurCsmY);
		openPanel();
		printf("%5i%c", (int)_BatteryConsum, 0x82);

		closePanel();
	}

	// battery voltage, size 1 x 8
	if(_bEnableBattVol)
	{
		setPanel(_iBatVolX, _iBatVolY);
		openPanel();
		if(_iLanguage == 0){
			printf("%c%5.2f%c", 0xB8, (double)_BatteryVol, 0x8e);
		}
		else{
			printf("%c%5.2f%c", 0xCB, (double)_BatteryVol, 0x8e);
		}

		closePanel();
	}

	// battery current, size 1 x 8
	if(_bEnableBattCur)
	{
		setPanel(_iBatCurX, _iBatCurY);
		openPanel();
		printf("%5.2f%c", (float(_BatteryCurrent) * .01), 0x8F);
		closePanel();
	}

	// battery percent, size 1 x 8
	if(_bEnableBattPercent)
	{
		setPanel(_iBatPerX, _iBatPerY);
		openPanel();
		printf("%3.0i%c", _BatteryPercent, 0x25);
		closePanel();
	}

	if(_bEnableWP)
	{
		// waypoint direction, size 1 x 5
		setPanel(_iWPX, _iWPY);
		openPanel();
		uint8_t wp_target_bearing = round(((float)_WPDirection - _heading)/360.0 * 16.0) + 1; //Convert to int 0-16 
		if(wp_target_bearing < 0 ) wp_target_bearing += 16; //normalize
		showArrow((uint8_t)wp_target_bearing,1);
		closePanel();

		// waypoint distance, size 1 x 5
		setPanel(_iWPX, _iWPY+1);
		openPanel();
		printf("%5.0f%c",(double)(_WPDistance*0.01f), 0x8D);
		closePanel();
	}

	//SPI release
	_spi_sem->give();
}

void AP_OSD_MAX7456::showWarning()
{
	setPanel(6, 1);
	openPanel();

	char* warning_string;
	warning_string = "\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20\x20";

	if(_iGPSStatus < 2)
	{
		//No GPS Fix
		warning_string = "\x20\x4E\x6F\x20\x47\x50\x53\x20\x66\x69\x78\x21";
	}
	else if(_iMotorArmed == 0)
	{
		//no armed
		warning_string = "\x20\x20\x44\x49\x53\x41\x52\x4d\x45\x44\x20\x20";
	}

	printf("%s",warning_string);
	closePanel();
}

void AP_OSD_MAX7456::showArrow(uint8_t rotate_arrow, uint8_t mode) 
{ 
	char arrow_set1 = 0x90;
	char arrow_set2 = 0x91;
	if((1< rotate_arrow) && (rotate_arrow < 17))
	{
		arrow_set1 = 0x90+(uint8_t)((rotate_arrow-1)*2);
		arrow_set2 = arrow_set1+0x1;
	}

	if(mode == 0){
		//home icon
		printf("%c%c%c", 0x1F, arrow_set1, arrow_set2);			
	}
	else if(mode == 1){
		//waypoint
		if(_iLanguage == 0){
			printf("%s%c%c", "WP",arrow_set1, arrow_set2);	
		}
		else{
			printf("%c%c%c%c", 0xCF, 0xD0, arrow_set1, arrow_set2);	
		}
	}
	else if(mode == 2){
		//heading
		printf("%c%c", arrow_set1, arrow_set2);	
	}
}

// Calculate and shows Artificial Horizon
void AP_OSD_MAX7456::showHorizon(uint8_t start_col, uint8_t start_row) 
{ 
	int x, nose, row, minval, hit, subval = 0;
	const int cols = HORIZON_LEN;
	const int rows = 5;
	int col_hit[cols];
	float  pitch, roll;

	(abs(_pitch) == 90)?pitch = 89.99 * (90/_pitch) * -0.017453293:pitch = _pitch * -0.017453293;
	(abs(_roll) == 90)?roll = 89.99 * (90/_roll) * 0.017453293:roll = _roll * 0.017453293;

	nose = round(tan(pitch) * (rows*9));
	for(int col=1;col <= cols;col++){
		x = (col * HORIZON_LEN) - (cols * 6) - 6;//center X point at middle of each col
		col_hit[col-1] = (tan(roll) * x) + nose + (rows*9) - 1;//calculating hit point on Y plus offset to eliminate negative values
		//col_hit[(col-1)] = nose + (rows * 9);
	}

	//clear the last display
	for(_HorizonHitIndex=0;_HorizonHitIndex < cols; _HorizonHitIndex++)
	{
		openSingle(_lastHorizonColHit[_HorizonHitIndex], _lastHorizonRowHit[_HorizonHitIndex]);
		printf("%c", 0x20);
	}
	_HorizonHitIndex = 0;
	for(int col=0;col < cols; col++){
		hit = col_hit[col];
		if(hit > 0 && hit < (rows * 18)){
			row = rows - ((hit-1)/18);
			minval = rows*18 - row*18 + 1;
			subval = hit - minval;
			subval = round((subval*9)/18);
			if(subval == 0) subval = 1;
			printHit(start_col + col, start_row + row - 1, subval);
		}
	}
}

void AP_OSD_MAX7456::printHit(uint8_t col, uint8_t row, uint8_t subval)
{
	_lastHorizonColHit[_HorizonHitIndex] = col;
	_lastHorizonRowHit[_HorizonHitIndex] = row;
	_HorizonHitIndex++;

	openSingle(col, row);
	char subval_char = 0x05 + subval;
	printf("%c", subval_char);

}

void AP_OSD_MAX7456::write_NVM(uint32_t font_count, uint8_t *character_bitmap)
{
	uint8_t x;
	uint8_t char_address_hi, char_address_lo;
	uint8_t screen_char;

	char_address_hi = font_count;
	char_address_lo = 0;

	//if (!_spi_sem->take_nonblocking()) {
	if (!_spi_sem->take(3000)) {
		hal.console->printf_P(PSTR("AP_OSD_MAX7456::write_NVM() can not get sem\n"));
		return;
	}

	// disable display
	_spi->cs_assert();
	_spi->transfer(MAX7456_VM0_reg); 
	_spi->transfer(MAX7456_DISABLE_display);

	_spi->transfer(MAX7456_CMAH_reg); // set start address high
	_spi->transfer(char_address_hi);

	for(x = 0; x < NVM_ram_size; x++) // write out 54 (out of 64) uint8_ts of character to shadow ram
	{
		screen_char = character_bitmap[x];
		_spi->transfer(MAX7456_CMAL_reg); // set start address low
		_spi->transfer(x);
		_spi->transfer(MAX7456_CMDI_reg);
		_spi->transfer(screen_char);
	}

	// transfer a 54 uint8_ts from shadow ram to NVM
	_spi->transfer(MAX7456_CMM_reg);
	_spi->transfer(WRITE_nvr);

	// wait until bit 5 in the status register returns to 0 (12ms)
	while ((_spi->transfer(MAX7456_STAT_reg_read) & STATUS_reg_nvr_busy) != 0x00);

	_spi->transfer(MAX7456_VM0_reg); // turn on screen next vertical
	_spi->transfer(MAX7456_ENABLE_display_vert);
	_spi->cs_release(); 

	_spi_sem->give();
}


//void AP_OSD_MAX7456::read_one_char_from_NVM(uint32_t font_count)
//{
//	uint8_t x;
//	uint8_t character_bitmap[NVM_ram_size];
//	uint8_t char_address_hi, char_address_lo;
//
//	char_address_hi = font_count;
//	char_address_lo = 0;  
//
//	if (!_spi_sem->take_nonblocking()) {
//		hal.console->printf_P(PSTR("AP_OSD_MAX7456::read_one_char_from_NVM() can not get sem\n"));
//		return;
//	}
//
//	// disable display
//	_spi->cs_assert();
//	{
//		_spi->transfer(MAX7456_VM0_reg); 
//		_spi->transfer(MAX7456_DISABLE_display);
//
//		_spi->transfer(MAX7456_CMAH_reg); // set start address high
//		_spi->transfer(char_address_hi);
//
//		_spi->transfer(MAX7456_CMM_reg); // set start address low
//		_spi->transfer(0x50);
//	}
//	_spi->cs_release();
//
//	// wait until bit 5 in the status register returns to 0 (12ms)
//	while ((_spi->transfer(MAX7456_STAT_reg_read) & STATUS_reg_nvr_busy) != 0x00);
//
//	for(x = 0; x < NVM_ram_size; x++) // write out 54 (out of 64) uint8_ts of character to shadow ram
//	{  
//		_spi->cs_assert();
//		{
//			_spi->transfer(MAX7456_CMAL_reg); // set start address low
//			_spi->transfer(x);
//
//			_spi->transfer(MAX7456_CMDO_reg);
//			character_bitmap[x] = _spi->transfer(0xff);
//		}
//		_spi->cs_release();
//	}
//
//	_spi->cs_assert();
//	{
//		_spi->transfer(MAX7456_VM0_reg); // turn on screen next vertical
//		_spi->transfer(MAX7456_ENABLE_display_vert);
//	}
//	_spi->cs_release();
//
//
//	_spi_sem->give();
//
//	//for testing
//	for(x = 0; x < NVM_ram_size; x++)
//	{
//		hal.console->printf_P(PSTR("ReadFont - 4-pixel data: %u\n"), character_bitmap[x]);
//	}
//	
//}