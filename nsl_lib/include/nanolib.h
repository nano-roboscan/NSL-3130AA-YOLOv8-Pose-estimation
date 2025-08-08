#ifndef __NANOLIB_H__
#define __NANOLIB_H__

#ifndef NSLTOF_API
#define NSLTOF_API
#endif

#include <vector>

#define NSL_MAX_HANDLE_SIZE			10

#define NSL_LIDAR_TYPE_A_WIDTH 		320
#define NSL_LIDAR_TYPE_A_HEIGHT		240
#define NSL_LIDAR_TYPE_B_WIDTH 		800
#define NSL_LIDAR_TYPE_B_HEIGHT		600
#define NSL_LIDAR_IMAGE_BPP 		2

#define NSL_RGB_IMAGE_WIDTH 		1920
#define NSL_RGB_IMAGE_HEIGHT		1080
#define NSL_RGB_IMAGE_BPP 			2

#define MAX_GRAYSCALE_VALUE			2897

#define	MAX_DISTANCE_24MHZ			6250	// mm
#define	MAX_DISTANCE_12MHZ			12500	// mm
#define	MAX_DISTANCE_6MHZ			25000	// mm
#define	MAX_DISTANCE_3MHZ			50000	// mm 

//Special codes for pixels without valid data
#define NSL_LIMIT_FOR_VALID_DATA 	64000
#define NSL_LOW_AMPLITUDE			64001
#define NSL_ADC_OVERFLOW			64002
#define NSL_SATURATION				64003
#define NSL_BAD_PIXEL 				64004
#define NSL_LOW_DCS					64005
#define NSL_INTERFERENCE			64007
#define NSL_EDGE_DETECTED			64008

#define OUT_X						0
#define OUT_Y						1
#define OUT_Z						2
#define MAX_OUT						3


namespace  NslOption {

	enum class FUNCTION_OPTIONS
	{
		FUNC_OFF = 0,
		FUNC_ON  = 1
	};

	enum class HDR_OPTIONS
	{
		HDR_NONE_MODE = 0,
		HDR_SPATIAL_MODE = 1,
		HDR_TEMPORAL_MODE =2
	};

	enum class UDP_SPEED_OPTIONS
	{
		NET_100Mbps = 0,
		NET_1000Mbps = 1
	};

	enum class DUALBEAM_MOD_OPTIONS
	{
		DB_OFF = 0,
		DB_6MHZ = 1,
		DB_3MHZ = 2
	};

	enum class DUALBEAM_OPERATION_OPTIONS
	{
		DB_AVOIDANCE = 0,
		DB_CORRECTION = 1,
		DB_FULL_CORRECTION = 2
	};

	enum class MODULATION_OPTIONS
	{
		MOD_12Mhz = 0,
		MOD_24Mhz = 1,
		MOD_6Mhz = 2,
		MOD_3Mhz = 3
	};

	enum class MODULATION_CH_OPTIONS
	{
		MOD_CH0 = 0,
		MOD_CH1 = 1,
		MOD_CH2 = 2,
		MOD_CH3 = 3,
		MOD_CH4 = 4,
		MOD_CH5 = 5,
		MOD_CH6 = 6,
		MOD_CH7 = 7,
		MOD_CH8 = 8,
		MOD_CH9 = 9,
		MOD_CH10 = 10,
		MOD_CH11 = 11,
		MOD_CH12 = 12,
		MOD_CH13 = 13,
		MOD_CH14 = 14,
		MOD_CH15 = 15
	};

	enum class FRAME_RATE_OPTIONS
	{
		FRAME_5FPS = 5,
		FRAME_10FPS = 10,
		FRAME_15FPS = 15,
		FRAME_20FPS = 20,
		FRAME_25FPS = 25,
		FRAME_30FPS = 30
	};

	enum class LENS_TYPE
	{
		LENS_NF = 0,
		LENS_SF = 1,
		LENS_WF = 2,
	};

	enum class LIDAR_TYPE_OPTIONS
	{
		TYPE_A = 0, // 320x240 lidar type 
		TYPE_B = 1  // 800x600 lidar type
	};

	enum class OPERATION_MODE_OPTIONS
	{
		NONE_MODE = 0,
		DISTANCE_MODE = 1,
		GRAYSCALE_MODE = 2,
		DISTANCE_AMPLITUDE_MODE = 3,
		DISTANCE_GRAYSCALE_MODE = 4,
		RGB_MODE = 5,
		RGB_DISTANCE_MODE = 6,
		RGB_DISTANCE_AMPLITUDE_MODE = 7,
		RGB_DISTANCE_GRAYSCALE_MODE = 8
	};

	enum class NSL_DATA_TYPE
	{
		NONE_DATA_TYPE = 0,
		DISTANCE_DATA_TYPE = 1,
		GRAYSCALE_DATA_TYPE = 2,
		DISTANCE_AMPLITUDE_DATA_TYPE = 3,
		DISTANCE_GRAYSCALE_DATA_TYPE = 4,
		RGB_DATA_TYPE = 5
	};

	enum class NSL_ERROR_TYPE
	{
		NSL_SUCCESS = 0,
		NSL_INVALID_HANDLE = -1,
		NSL_NOT_OPENED = -2,
		NSL_NOT_READY = -3,
		NSL_IP_DUPLICATED = -4,
		NSL_HANDLE_OVERFLOW = -5,
		NSL_DISCONNECTED_SOCKET = -6,
		NSL_ANSWER_ERROR = -7,
		NSL_INVALID_PARAMETER = -8
	};

	
	
	typedef struct NslVec3b_ {
		unsigned char b, g, r;
	
		NslVec3b_() : b(0), g(0), r(0) {}
		NslVec3b_(unsigned char b, unsigned char g, unsigned char r) : b(b), g(g), r(r) {}
	
		unsigned char& operator[](int index) {
			if( index == 0 ) return b;
			else if( index == 1 ) return g;
	
			return r;
		}
	
		const unsigned char& operator[](int  index) const {
			if( index == 0 ) return b;
			else if( index == 1 ) return g;
	
			return r;
		}
	
	
	}NslVec3b;

	inline const char* toString(FUNCTION_OPTIONS c) {
        switch (c) {
            case FUNCTION_OPTIONS::FUNC_OFF:   return "FUNC_OFF";
            case FUNCTION_OPTIONS::FUNC_ON: return "FUNC_ON";
        }
		return "Unknown";
    }

	inline const char* toString(HDR_OPTIONS c) {
        switch (c) {
            case HDR_OPTIONS::HDR_NONE_MODE:   return "HDR_NONE_MODE";
            case HDR_OPTIONS::HDR_SPATIAL_MODE: return "HDR_SPATIAL_MODE";
            case HDR_OPTIONS::HDR_TEMPORAL_MODE: return "HDR_TEMPORAL_MODE";
        }
		return "Unknown";
    }

	inline const char* toString(UDP_SPEED_OPTIONS c) {
        switch (c) {
            case UDP_SPEED_OPTIONS::NET_100Mbps:   return "NET_100Mbps";
            case UDP_SPEED_OPTIONS::NET_1000Mbps: return "NET_1000Mbps";
        }

		return "Unknown";
    }

	inline const char* toString(DUALBEAM_MOD_OPTIONS c) {
        switch (c) {
            case DUALBEAM_MOD_OPTIONS::DB_OFF:   	return "DB_OFF";
            case DUALBEAM_MOD_OPTIONS::DB_6MHZ: 	return "DB_6MHZ";
            case DUALBEAM_MOD_OPTIONS::DB_3MHZ: 	return "DB_3MHZ";
        }

		return "Unknown";
    }

	inline const char* toString(DUALBEAM_OPERATION_OPTIONS c) {
        switch (c) {
            case DUALBEAM_OPERATION_OPTIONS::DB_AVOIDANCE:   		return "DB_AVOIDANCE";
            case DUALBEAM_OPERATION_OPTIONS::DB_CORRECTION: 		return "DB_CORRECTION";
            case DUALBEAM_OPERATION_OPTIONS::DB_FULL_CORRECTION: 	return "DB_FULL_CORRECTION";
        }

		return "Unknown";
    }

	inline const char* toString(MODULATION_OPTIONS c) {
        switch (c) {
            case MODULATION_OPTIONS::MOD_12Mhz:   		return "MOD_12Mhz";
            case MODULATION_OPTIONS::MOD_24Mhz: 		return "MOD_24Mhz";
            case MODULATION_OPTIONS::MOD_6Mhz: 			return "MOD_6Mhz";
            case MODULATION_OPTIONS::MOD_3Mhz: 			return "MOD_3Mhz";
        }

		return "Unknown";
    }

	inline const char* toString(MODULATION_CH_OPTIONS c) {
        switch (c) {
            case MODULATION_CH_OPTIONS::MOD_CH0:   	return "MOD_CH0";
            case MODULATION_CH_OPTIONS::MOD_CH1: 		return "MOD_CH1";
            case MODULATION_CH_OPTIONS::MOD_CH2: 		return "MOD_CH2";
            case MODULATION_CH_OPTIONS::MOD_CH3: 		return "MOD_CH3";
            case MODULATION_CH_OPTIONS::MOD_CH4: 		return "MOD_CH4";
            case MODULATION_CH_OPTIONS::MOD_CH5: 		return "MOD_CH5";
            case MODULATION_CH_OPTIONS::MOD_CH6: 		return "MOD_CH6";
            case MODULATION_CH_OPTIONS::MOD_CH7: 		return "MOD_CH7";
            case MODULATION_CH_OPTIONS::MOD_CH8: 		return "MOD_CH8";
            case MODULATION_CH_OPTIONS::MOD_CH9: 		return "MOD_CH9";
            case MODULATION_CH_OPTIONS::MOD_CH10: 		return "MOD_CH10";
            case MODULATION_CH_OPTIONS::MOD_CH11: 		return "MOD_CH11";
            case MODULATION_CH_OPTIONS::MOD_CH12: 		return "MOD_CH12";
            case MODULATION_CH_OPTIONS::MOD_CH13: 		return "MOD_CH13";
            case MODULATION_CH_OPTIONS::MOD_CH14: 		return "MOD_CH14";
            case MODULATION_CH_OPTIONS::MOD_CH15: 		return "MOD_CH15";
        }

		return "Unknown";
    }

	inline const char* toString(FRAME_RATE_OPTIONS c) {
        switch (c) {
            case FRAME_RATE_OPTIONS::FRAME_5FPS:   		return "FRAME_5FPS";
            case FRAME_RATE_OPTIONS::FRAME_10FPS: 		return "FRAME_10FPS";
            case FRAME_RATE_OPTIONS::FRAME_15FPS: 		return "FRAME_15FPS";
            case FRAME_RATE_OPTIONS::FRAME_20FPS: 		return "FRAME_20FPS";
            case FRAME_RATE_OPTIONS::FRAME_25FPS: 		return "FRAME_25FPS";
            case FRAME_RATE_OPTIONS::FRAME_30FPS: 		return "FRAME_30FPS";
        }

		return "Unknown";
    }

	inline const char* toString(LIDAR_TYPE_OPTIONS c) {
        switch (c) {
            case LIDAR_TYPE_OPTIONS::TYPE_A:   		return "TYPE_A";
            case LIDAR_TYPE_OPTIONS::TYPE_B: 		return "TYPE_B";
        }

		return "Unknown";
    }

	inline const char* toString(OPERATION_MODE_OPTIONS c) {
        switch (c) {
            case OPERATION_MODE_OPTIONS::NONE_MODE:   					return "NONE_MODE";
            case OPERATION_MODE_OPTIONS::DISTANCE_MODE: 				return "DISTANCE_MODE";
            case OPERATION_MODE_OPTIONS::GRAYSCALE_MODE: 				return "GRAYSCALE_MODE";
            case OPERATION_MODE_OPTIONS::DISTANCE_AMPLITUDE_MODE: 		return "DISTANCE_AMPLITUDE_MODE";
            case OPERATION_MODE_OPTIONS::DISTANCE_GRAYSCALE_MODE: 		return "DISTANCE_GRAYSCALE_MODE";
            case OPERATION_MODE_OPTIONS::RGB_MODE: 						return "RGB_MODE";
            case OPERATION_MODE_OPTIONS::RGB_DISTANCE_MODE: 			return "RGB_DISTANCE_MODE";
            case OPERATION_MODE_OPTIONS::RGB_DISTANCE_AMPLITUDE_MODE: 	return "RGB_DISTANCE_AMPLITUDE_MODE";
			case OPERATION_MODE_OPTIONS::RGB_DISTANCE_GRAYSCALE_MODE: 	return "RGB_DISTANCE_GRAYSCALE_MODE";
        }

		return "Unknown";
    }

	inline const char* toString(NSL_DATA_TYPE c) {
        switch (c) {
            case NSL_DATA_TYPE::NONE_DATA_TYPE:   				return "NONE_DATA_TYPE";
            case NSL_DATA_TYPE::DISTANCE_DATA_TYPE: 			return "DISTANCE_DATA_TYPE";
            case NSL_DATA_TYPE::DISTANCE_AMPLITUDE_DATA_TYPE:  	return "DISTANCE_AMPLITUDE_DATA_TYPE";
            case NSL_DATA_TYPE::DISTANCE_GRAYSCALE_DATA_TYPE:  	return "DISTANCE_GRAYSCALE_DATA_TYPE";
            case NSL_DATA_TYPE::GRAYSCALE_DATA_TYPE:			return "GRAYSCALE_DATA_TYPE";
            case NSL_DATA_TYPE::RGB_DATA_TYPE:			 		return "RGB_DATA_TYPE";
        }

		return "Unknown";
    }

	inline const char* toString(NSL_ERROR_TYPE c) {
        switch (c) {
            case NSL_ERROR_TYPE::NSL_SUCCESS:   				return "NSL_SUCCESS";
            case NSL_ERROR_TYPE::NSL_INVALID_HANDLE: 			return "NSL_INVALID_HANDLE";
            case NSL_ERROR_TYPE::NSL_NOT_OPENED:  				return "NSL_NOT_OPENED";
            case NSL_ERROR_TYPE::NSL_NOT_READY:			 		return "NSL_NOT_READY";
            case NSL_ERROR_TYPE::NSL_IP_DUPLICATED:			 	return "NSL_IP_DUPLICATED";
            case NSL_ERROR_TYPE::NSL_HANDLE_OVERFLOW:			return "NSL_HANDLE_OVERFLOW";
            case NSL_ERROR_TYPE::NSL_DISCONNECTED_SOCKET:		return "NSL_DISCONNECTED_SOCKET";
            case NSL_ERROR_TYPE::NSL_ANSWER_ERROR:			 	return "NSL_ANSWER_ERROR";
            case NSL_ERROR_TYPE::NSL_INVALID_PARAMETER:			return "NSL_INVALID_PARAMETER";
        }

		return "Unknown";
    }
};


typedef struct NslConfig_
{	
	//Integration time
	int			integrationTime3D;
	int			integrationTime3DHdr1;
	int			integrationTime3DHdr2;
	int			integrationTimeGrayScale;
	
	// ROI
	int			roiXMin;
	int			roiXMax;
	int			roiYMin;
	int			roiYMax;

	// distance offset
	int			currentOffset;
	
	// min amplitude
	int			minAmplitude;

	// release info
	int 		firmware_release;
	int 		chipID;
	int 		waferID;

	// UDP data port
	int 		udpDataPort;

	// LED Seqment Mask
	int 		ledMask;

	// installed lidar angle(-90 ~ 90)
	double		lidarAngle;

	NslOption::LIDAR_TYPE_OPTIONS		lidarType;

	// Lens type
	NslOption::LENS_TYPE				lensType;
	
	// tofcam mode
	NslOption::OPERATION_MODE_OPTIONS	operationModeOpt;

	// HDR
	NslOption::HDR_OPTIONS				hdrOpt;

	// modulation
	NslOption::MODULATION_OPTIONS		mod_frequencyOpt;
	NslOption::MODULATION_CH_OPTIONS	mod_channelOpt;
	NslOption::FUNCTION_OPTIONS			mod_enabledAutoChannelOpt;

	// DualBeam
	NslOption::DUALBEAM_MOD_OPTIONS			dbModOpt;
	NslOption::DUALBEAM_OPERATION_OPTIONS	dbOpsOpt;

	//Binning
	NslOption::FUNCTION_OPTIONS			ver_binningOpt;
	NslOption::FUNCTION_OPTIONS			horiz_binningOpt;

	// adc overflow & saturation
	NslOption::FUNCTION_OPTIONS			overflowOpt;
	NslOption::FUNCTION_OPTIONS			saturationOpt;

	// drnu compensation
	NslOption::FUNCTION_OPTIONS 	drnuOpt;
	NslOption::FUNCTION_OPTIONS 	temperatureOpt;
	NslOption::FUNCTION_OPTIONS 	grayscaleOpt;
	NslOption::FUNCTION_OPTIONS 	ambientlightOpt;

	// Filter
	NslOption::FUNCTION_OPTIONS 	medianOpt;
	NslOption::FUNCTION_OPTIONS 	gaussOpt;
	int 							temporalFactorValue;
	int 							temporalThresholdValue;
	int 							edgeThresholdValue;
	int 							interferenceDetectionLimitValue;
	NslOption::FUNCTION_OPTIONS 	interferenceDetectionLastValueOpt;
	int 							edgeThresholdValue3D;

	// UDP speed
	NslOption::UDP_SPEED_OPTIONS	udpSpeedOpt;

	// frame Rate
	NslOption::FRAME_RATE_OPTIONS 	frameRateOpt;
	
	//grayscale illumination mode
	NslOption::FUNCTION_OPTIONS		grayscaleIlluminationOpt;
}NslConfig, *LP_NslConfig;



typedef struct NslPCD_{
	NslOption::OPERATION_MODE_OPTIONS 	operationMode; // DISTANCE_MODE, DISTANCE_AMPLITUDE_MODE, DISTANCE_GRAYSCALE_MODE, RGB_MODE, RGB_DISTANCE_MODE, RGB_DISTANCE_AMPLITUDE_MODE, RGB_DISTANCE_GRAYSCALE_MODE
	NslOption::LIDAR_TYPE_OPTIONS		lidarType; // TYPE_A(320x240), TYPE_B(800x600)
	double								temperature;	// temperature

	bool 				includeRgb;			// true : include RGB, false : not include RGB
	bool 				includeLidar;		// tru : include Lidar Data, false : not include Lidar Data
	
	int 				width;				// Width of lidar or rgb image
	int 				height;				// height of lidar or rgb image
	int 				roiXMin;			// Starting point of horizontal line
	int 				roiYMin;			// Starting point of vertical line
	int 				roiXMax;			// end point of horizontal line
	int 				roiYMax;			// end point of vertical line
	int 				binning_h;			// 1 : normal operation, 2 : horizontal binning
	int 				binning_v;			// 1 : normal operation, 2 : vertical binning
	int 				amplitude[NSL_LIDAR_TYPE_B_HEIGHT][NSL_LIDAR_TYPE_B_WIDTH];	// amplitude data
	int 				distance2D[NSL_LIDAR_TYPE_B_HEIGHT][NSL_LIDAR_TYPE_B_WIDTH];	// distance data
	double				distance3D[MAX_OUT][NSL_LIDAR_TYPE_B_HEIGHT][NSL_LIDAR_TYPE_B_WIDTH]; // point clouds 3D data
	NslOption::NslVec3b	rgb[NSL_RGB_IMAGE_HEIGHT][NSL_RGB_IMAGE_WIDTH]; // RGB data
}NslPCD;

#if defined(__cplusplus)
//extern "C" {
#endif


/******************* Lidar Interface SDK *****************************/

/**
 * @brief Create a handle for the lidar.
 * 
 * @param ipaddr : "192.168.0.220" < Connectable IP Address string >
 * @param ptNslConfig : when opened normally, basic information is read.
 * 	- angle<write param> : -90 ~ 90 < Vertical installation angle of lidar >
 * 	- lensType<write param> : NF(50), SF(90), WF(110)
 * @param enabledDebug : 0, 1 <0:off, 1:enable library debug print>
 * 
 * @return created handle value
 */
NSLTOF_API int nsl_open(const char *ipaddr, NslConfig *ptNslConfig, NslOption::FUNCTION_OPTIONS enabledDebug = NslOption::FUNCTION_OPTIONS::FUNC_OFF);


/**
 * @brief Close the lidar handle.
 * 
 * @param handle : handle value by nsl_open()
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_closeHandle(int handle);


/**
 * @brief Closes all handles created by the library.
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_close();


/**
 * @brief Turn on lidar streaming.
 * 
 * @param handle : handle value by nsl_open()
 * @param type : lidar or rgb data type < OPERATION_MODE_OPTIONS >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_streamingOn(int handle, NslOption::OPERATION_MODE_OPTIONS type);


/**
 * @brief Turn off lidar streaming.
 * 
 * @param handle : handle value by nsl_open()
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_streamingOff(int handle);



/**
 * @brief Read lidar pointcloud data
 * 
 * @param handle : handle value by nsl_open()
 * @param pcdData : lidar data info pointer < NslPCD * >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_getPointCloudData(int handle, NslPCD *pcdData, int waitTimeMs = 0);


/**
 * @brief Sets the lidar frame rate.
 * 
 * @param handle : handle value by nsl_open()
 * @param rate : 5 ~ 30 < FRAME_RATE_OPTIONS >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_setFrameRate(int handle, NslOption::FRAME_RATE_OPTIONS rate);


/**
 * @brief read the lidar frame rate.
 * 
 * @param handle : handle value by nsl_open()
 * @param *rate : 5 ~ 30 < FRAME_RATE_OPTIONS >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_getFrameRate(int handle, NslOption::FRAME_RATE_OPTIONS *rate);


/**
 * @brief Sets the udp speed of the lidar.
 * 
 * @param handle : handle value by nsl_open()
 * @param speed : 0, 1 < 0:100Mbps, 1:1000Mbps >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_setUdpSpeed(int handle, NslOption::UDP_SPEED_OPTIONS speed);


/**
 * @brief read the udp speed of the lidar.
 * 
 * @param handle : handle value by nsl_open()
 * @param *speed : UDP_SPEED_OPTIONS pointer < 0:100Mbps, 1:1000Mbps >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_getUdpSpeed(int handle, NslOption::UDP_SPEED_OPTIONS *speed);


/**
 * @brief Sets the minimum amplitude of the lidar.
 * 
 * @param handle : handle value by nsl_open()
 * @param minAmplitude : 0 ~ 500
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_setMinAmplitude(int handle, int minAmplitude);

/**
 * @brief read the minimum amplitude of the lidar.
 * 
 * @param handle : handle value by nsl_open()
 * @param *minAmplitude : minimum amplitude
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_getMinAmplitude(int handle, int *minAmplitude);


/**
 * @brief Sets the integration time of the lidar.
 * 
 * @param handle : handle value by nsl_open()
 * @param intTime : 0 ~ 2000 < Set the default integration time >
 * @param intTimeHdr1 : 0 ~ 2000 < Set the spatial / temporal integration time >
 * @param intTimeHdr2 : 0 ~ 2000 < Set the temporal integration time >
 * @param intGray : 0 ~ 40000 < Set the grayscale integration time >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_setIntegrationTime(int handle, int intTime, int intTimeHdr1, int intTimeHdr2, int intGray);


/**
 * @brief Read the integration time of the lidar.
 * 
 * @param handle : handle value by nsl_open()
 * @param *intTime : 0 ~ 2000 < Read the default integration time >
 * @param *intTimeHdr1 : 0 ~ 2000 < Read the spatial / temporal integration time >
 * @param *intTimeHdr2 : 0 ~ 2000 < Read the temporal integration time >
 * @param *intGray : 0 ~ 40000 < Read the grayscale integration time >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_getIntegrationTime(int handle, int *intTime, int *intTimeHdr1, int *intTimeHdr2, int *intGray);


/**
 * @brief Sets the HDR mode of the lidar.
 * 
 * @param handle : handle value by nsl_open()
 * @param mode : 0, 1, 2 < 0:HDR off, 1:spatial hdr, 2:temporal hdr >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_setHdrMode(int handle, NslOption::HDR_OPTIONS mode);


/**
 * @brief Read the HDR mode of the lidar.
 * 
 * @param handle : handle value by nsl_open()
 * @param *mode : 0, 1, 2 < 0:HDR off, 1:spatial hdr, 2:temporal hdr >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_getHdrMode(int handle, NslOption::HDR_OPTIONS *mode);


/**
 * @brief Sets illumination in grayscale mode
 * 
 * @param handle : handle value by nsl_open()
 * @param OnOff : ON, OFF
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_setGrayscaleillumination(int handle, NslOption::FUNCTION_OPTIONS OnOff);


/**
 * @brief Read the illumination settings in grayscale mode
 * 
 * @param handle : handle value by nsl_open()
 * @param *mode : ON, OFF
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_getGrayscaleillumination(int handle, NslOption::FUNCTION_OPTIONS *OnOff);


/**
 * @brief Sets the overflow & saturation of the lidar.
 * 
 * @param handle : handle value by nsl_open()
 * @param enableOverFlow : 0, 1 < 0:off, 1:on >
 * @param enableSaturation : 0, 1 < 0:off, 1:on >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_setAdcOverflowSaturation(int handle, NslOption::FUNCTION_OPTIONS enableOverFlow, NslOption::FUNCTION_OPTIONS enableSaturation);


/**
 * @brief Read the overflow & saturation of the lidar.
 * 
 * @param handle : handle value by nsl_open()
 * @param *enableOverFlow : 0, 1 < 0:off, 1:on >
 * @param *enableSaturation : 0, 1 < 0:off, 1:on >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_getAdcOverflowSaturation(int handle, NslOption::FUNCTION_OPTIONS *enableOverFlow, NslOption::FUNCTION_OPTIONS *enableSaturation);


/**
 * @brief Sets the dualbeam mode of the lidar.
 * 
 * @param handle : handle value by nsl_open()
 * @param dualModulation : 0, 1, 2 < 0:off, 1:6Mhz, 2:3Mhz >
 * @param dualOption : 0, 1, 2 < 0:avoidance, 1:correction, 2:full conrrection >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_setDualBeam(int handle, NslOption::DUALBEAM_MOD_OPTIONS dualModulation, NslOption::DUALBEAM_OPERATION_OPTIONS dualOption);


/**
 * @brief Read the dualbeam mode of the lidar.
 * 
 * @param handle : handle value by nsl_open()
 * @param *dualModulation : 0, 1, 2 < 0:off, 1:6Mhz, 2:3Mhz >
 * @param *dualOption : 0, 1, 2 < 0:avoidance, 1:correction, 2:full conrrection >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_getDualBeam(int handle, NslOption::DUALBEAM_MOD_OPTIONS *dualModulation, NslOption::DUALBEAM_OPERATION_OPTIONS *dualOption);


/**
 * @brief Sets the modulation & channel of the lidar.
 * 
 * @param handle : handle value by nsl_open()
 * @param modulation : 0~3 < 0:12Mhz, 1:24Mhz, 2:6Mhz, 3:3Mhz >
 * @param ch : 0~15 < channel number >
 * @param enableAutoChannel : 0, 1 < 0:off, 1:on >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_setModulation(int handle, NslOption::MODULATION_OPTIONS modulation, NslOption::MODULATION_CH_OPTIONS ch, NslOption::FUNCTION_OPTIONS enableAutoChannel);


/**
 * @brief Read the modulation & channel of the lidar.
 * 
 * @param handle : handle value by nsl_open()
 * @param *modulation : 0~3 < 0:12Mhz, 1:24Mhz, 2:6Mhz, 3:3Mhz >
 * @param *ch : 0~15 < channel number >
 * @param *enableAutoChannel : 0, 1 < 0:off, 1:on >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_getModulation(int handle, NslOption::MODULATION_OPTIONS *modulation, NslOption::MODULATION_CH_OPTIONS *ch, NslOption::FUNCTION_OPTIONS *enableAutoChannel);


/**
 * @brief Sets the filter of the lidar.
 * 
 * @param handle : handle value by nsl_open()
 * @param *enableMedian : 0, 1 < 0 : off, 1 : enable median filter> 
 * @param *enableGauss : 0, 1 < 0 : off, 1 : enable gauss filter> 
 * @param *temporalFactor : 0 ~ 1000 <0, 1000 : off, 1 ~ 999 temporal factor value>
 * @param temporalThreshold : 0 ~ 1000 <0 : off, 1 ~ 1000 temporal factor value>
 * @param *edgeThreshold : 0 ~ 5000 <0 : off, 1 ~ 5000 : edge threshold value>
 * @param *interferenceDetectionLimit : 0 ~ 10000 <0 : off, 1 ~ 10000 : interference detection limit value>
 * @param *enableInterferenceDetectionLastValue : 0, 1 < 0 : off, 1 : used last value>
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_setFilter(int handle, NslOption::FUNCTION_OPTIONS enableMedian, NslOption::FUNCTION_OPTIONS enableGauss, int temporalFactor, int temporalThreshold, int edgeThreshold, int interferenceDetectionLimit, NslOption::FUNCTION_OPTIONS enableInterferenceDetectionLastValue);


/**
 * @brief Read the filter of the lidar.
 * 
 * @param handle : handle value by nsl_open()
 * @param *enableMedian : 0, 1 < 0 : off, 1 : enable median filter> 
 * @param *enableGauss : 0, 1 < 0 : off, 1 : enable gauss filter> 
 * @param *temporalFactor : 0 ~ 1000 <0, 1000 : off, 1 ~ 999 temporal factor value>
 * @param temporalThreshold : 0 ~ 1000 <0 : off, 1 ~ 1000 temporal factor value>
 * @param *edgeThreshold : 0 ~ 5000 <0 : off, 1 ~ 5000 : edge threshold value>
 * @param *interferenceDetectionLimit : 0 ~ 10000 <0 : off, 1 ~ 10000 : interference detection limit value>
 * @param *enableInterferenceDetectionLastValue : 0, 1 < 0 : off, 1 : used last value>
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_getFilter(int handle, NslOption::FUNCTION_OPTIONS *enableMedian, NslOption::FUNCTION_OPTIONS *enableGauss, int *temporalFactor, int *temporalThreshold, int *edgeThreshold, int *interferenceDetectionLimit, NslOption::FUNCTION_OPTIONS *enableInterferenceDetectionLastValue);


/**
 * @brief Sets the filter of the lidar.
 * 
 * @param handle : handle value by nsl_open()
 * @param *edgeThreshold : 0 ~ 5000 <0 : off, 1 ~ 5000 : edge threshold value>
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_set3DFilter(int handle, int edgeThreshold);


/**
 * @brief Read the filter of the lidar.
 * 
 * @param handle : handle value by nsl_open()
 * @param *edgeThreshold : 0 ~ 5000 <0 : off, 1 ~ 5000 : edge threshold value>
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_get3DFilter(int handle, int *edgeThreshold);

/**
 * @brief Sets the binning of the lidar.
 * 
 * @param handle : handle value by nsl_open()
 * @param vertical : 0, 1 < 0:off, 1:on >
 * @param horizontal : 0, 1 < 0:off, 1:on >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_setBinning(int handle, NslOption::FUNCTION_OPTIONS vertical, NslOption::FUNCTION_OPTIONS horizontal);


/**
 * @brief Read the binning of the lidar.
 * 
 * @param handle : handle value by nsl_open()
 * @param *vertical : 0, 1 < 0:off, 1:on >
 * @param *horizontal : 0, 1 < 0:off, 1:on >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_getBinning(int handle, NslOption::FUNCTION_OPTIONS *vertical, NslOption::FUNCTION_OPTIONS *horizontal);


/**
 * @brief Sets the ROI of the lidar. <min:8x8, max:320x240>
 * 
 * Please refer to the NSL-3140AA GUI Application for further assistance.
 * 
 * @param handle : handle value by nsl_open()
 * @param minX : 0~124(120) <Can be set to multiples of 4> (for NSL-3130CV, set to multiples of 8)
 * @param minY : 0~116 <Can be set to multiples of 2>
 * @param maxX : 319~131(127) <Can be set to multiples of 4> (for NSL-3130CV, set to multiples of 8)
 * @param maxY : 239~123 <Can be set to multiples of 2>
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_setRoi(int handle, int minX, int minY, int maxX, int maxY);


/**
 * @brief Read the ROI of the lidar. <min:8x8, max:320x240>
 * 
 * Please refer to the NSL-3140AA GUI Application for further assistance.
 * 
 * @param handle : handle value by nsl_open()
 * @param *minX : 0~124(120) <Can be set to multiples of 4> (for NSL-3130CV, set to multiples of 8)
 * @param *minY : 0~116 <Can be set to multiples of 2>
 * @param *maxX : 319~131(127) <Can be set to multiples of 4> (for NSL-3130CV, set to multiples of 8)
 * @param *maxY : 239~123 <Can be set to multiples of 2>
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_getRoi(int handle, int *minX, int *minY, int *maxX, int *maxY);


/**
 * @brief Sets the sensor's correction function.
 * 
 * @param handle : handle value by nsl_open()
 * @param drnu : 0, 1 < 0:off, 1:on >
 * @param temperature : 0, 1 < 0:off, 1:on >
 * @param grayscale : 0, 1 < 0:off, 1:on >
 * @param ambientlight : 0, 1 < 0:off, 1:on >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_setCorrection(int handle, NslOption::FUNCTION_OPTIONS drnu, NslOption::FUNCTION_OPTIONS temperature, NslOption::FUNCTION_OPTIONS grayscale, NslOption::FUNCTION_OPTIONS ambientlight);


/**
 * @brief Reads the sensor's correction function.
 * 
 * @param handle : handle value by nsl_open()
 * @param *drnu : 0, 1 < 0:off, 1:on >
 * @param *temperature : 0, 1 < 0:off, 1:on >
 * @param *grayscale : 0, 1 < 0:off, 1:on >
 * @param *ambientlight : 0, 1 < 0:off, 1:on >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_getCorrection(int handle, NslOption::FUNCTION_OPTIONS *drnu, NslOption::FUNCTION_OPTIONS *temperature, NslOption::FUNCTION_OPTIONS *grayscale, NslOption::FUNCTION_OPTIONS *ambientlight);


/**
 * @brief Sets  the sensor's ip address
 * 
 * @param handle : handle value by nsl_open()
 * @param *ipAddress : "192.168.0.220"
 * @param *netMask : "255.255.255.0"
 * @param *gateYay : "192.168.0.1"
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_setIpAddress(int handle, const char *ipAddress, const char *netMask, const char *gateWay);


/**
 * @brief Reads the sensor's ip address
 * 
 * @param handle : handle value by nsl_open()
 * @param *ipAddress : char array
 * @param *netMask : char array
 * @param *gateWay : char array
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_getIpAddress(int handle, char *ipAddress, char *netMask, char *gateWay);


/**
 * @brief Sets the sensor's Led segment. (default : 15)
 * 
 * @param handle : handle value by nsl_open()
 * @param mask : 0~15 < 0:off, 1 : LEDs in the lower two rows, 4 : LEDs in the upper two rows, 15 : all LED >
 * 
 * @description (used lower 4Bits)
 *	 all off = 0,
 *	 1st line = 1,
 *	 2nd line = 2,
 *	 3rd line = 4,
 *	 4th line = 8,
 *	 all on   = 15,
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_setLedSegment(int handle, int mask);


/**
 * @brief Reads the sensor's Led segment. (default : 15)
 * 
 * @param handle : handle value by nsl_open()
 * @param *mask : 0, 1 < 0:off, 1 : LEDs in the lower two rows, 4 : LEDs in the upper two rows, 15 : all LED >
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_getLedSegment(int handle, int *mask);


/**
 * @brief Saves the parameters set so far to the device.
 * 
 * @param handle : handle value by nsl_open()
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_saveConfiguration(int handle);


/**
 * @brief Reads the parameters set on the device.
 * 
 * @param handle : handle value by nsl_open()
 * @param ptLidarConfig : lidar configuration pointer
 * 
 * @return NSL_ERROR_TYPE 
 */
NSLTOF_API NslOption::NSL_ERROR_TYPE nsl_getCurrentConfig(int handle, NslConfig *ptNslConfig);


/******************* 3D Point Cloud SDK *****************************/


/**
 * @brief Returns the point cloud distance at a pixel location.
 * 
 * @param srcX : Pixel index of width
 * @param srcY : Pixel index of height
 * @param srcZ : 2D distance to pixel
 * @param destX : catesian distance of width
 * @param destY : catesian distance of height
 * @param destZ : catesian distance to pixel
 * @param transformAngle : Vertical installation angle of lidar
 * 
 * @return void 
 */
NSLTOF_API void nsl_transformPixel(unsigned int srcX, unsigned int srcY, double srcZ, double *destX, double *destY, double *destZ);

/**
 * @brief Sets the 2D color of distance and amplitude.
 * 
 * @param maxDistance : Sets the color of the maximum distance.
 * @param maxGrayScale : Sets the color of the maximum grayscale.
 * @param isGrayscale : amplitude to grayscale
 * 
 * @return void 
 */
NSLTOF_API void nsl_setColorRange(int maxDistance, int maxGrayScale, NslOption::FUNCTION_OPTIONS isGrayscale);

/**
 * @brief Returns the distance color by distance.
 * 
 * @param value : distance value (mm)
 * 
 * @return NslVec3b 
 */
NSLTOF_API NslOption::NslVec3b nsl_getDistanceColor(int value);


/**
 * @brief Returns the amplitude color.
 * 
 * @param value : amplitude value (lsb)
 * 
 * @return NslVec3b 
 */
NSLTOF_API NslOption::NslVec3b nsl_getAmplitudeColor(int value);

#if defined(__cplusplus)
//}
#endif



#endif // __NANOLIB_H__
