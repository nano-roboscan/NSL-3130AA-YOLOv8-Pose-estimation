/*
 * Copyright (c) 2013, NANOSYSTEMS CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#ifndef __NSL3130AA_CAMERA_H__
#define __NSL3130AA_CAMERA_H__

#ifdef _WINDOWS
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/console/parse.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#endif

#include "videoSource.h"

#define NSL3130_ETH_BUFF_SIZE		1
#define NSL3130_BUFF_SIZE   		308000	// 320 x 240 x 4 + 12 (head + length + tail) +  25

#define NSL3130_IMAGE_WIDTH 		320
#define NSL3130_IMAGE_HEIGHT		240
#define NSL3130_NUM_COLORS     		30000
#define NSL3130_MARKER_SIZE 		4
#define NSL3130_NET_PORT 			50660
#define NSL3130_NET_HOST 			"192.168.0.220"

#define HEADER_PREFIX_SIZE			4
#define TAIL_PREFIX_SIZE			4
#define PACKET_LENGTH_SIZE			4
#define PACKET_INFO_SIZE			(HEADER_PREFIX_SIZE+PACKET_LENGTH_SIZE)

#define START_MARKER 				"\xff\xff\xaa\x55"
#define END_MARKER 					"\xff\xff\x55\xaa"
#define PAYLOAD_SIZE_INDEX 			4
#define PAYLOAD_LENGTH_SIZE 		4
#define NSL3130_NET_HEADER_SIZE 	(NSL3130_MARKER_SIZE + PAYLOAD_LENGTH_SIZE)
#define PROTOCOL_OVERHEAD_SIZE 		(NSL3130_MARKER_SIZE + PAYLOAD_LENGTH_SIZE + NSL3130_MARKER_SIZE)
#define DATA_HEADER_SIZE 			25
#define UDP_PACKET_HEADER_SIZE 		20
#define UART_OVERHEAD_SIZE			13

#define NSL3130_PIXEL_CODE_OFFSET		48000
//Special codes for pixels without valid data
#define NSL3130_LIMIT_FOR_VALID_DATA 	(16000 + NSL3130_PIXEL_CODE_OFFSET)
#define NSL3130_LOW_AMPLITUDE			(16001 + NSL3130_PIXEL_CODE_OFFSET)
#define NSL3130_ADC_OVERFLOW			(16002 + NSL3130_PIXEL_CODE_OFFSET)
#define NSL3130_SATURATION				(16003 + NSL3130_PIXEL_CODE_OFFSET)
#define NSL3130_BAD 					(16004 + NSL3130_PIXEL_CODE_OFFSET)
#define NSL3130_LOW_DCS					(16005 + NSL3130_PIXEL_CODE_OFFSET)
#define NSL3130_INTERFERENCE			(16007 + NSL3130_PIXEL_CODE_OFFSET)
#define NSL3130_EDGE_DETECTED			(16008 + NSL3130_PIXEL_CODE_OFFSET)


typedef struct CamAppConfig_
{	
	//Integration time
	int			integrationTime3D;
	int			integrationTime3DHdr1;
	int			integrationTime3DHdr2;
	int			integrationTimeGrayScale;

	// HDR
	int			hdr_mode;

	// modulation
	int			mod_frequency;
	int			mod_channel;
	int			mod_autoChannelEnabled;

	// ROI
	int			roi_xMin;
	int			roi_xMax;
	int			roi_yMin;
	int			roi_yMax;

	// adc overflow & saturation
	int			saturatedFlag;

	// drnu compensation
	int			compensationFlag;

	// distance offset
	int			drnuOffset[6];
	
	// min amplitude
	int			minAmplitude;

	// dual beam
	int			dualbeamState;

	// deep learning
	int			deeplearning;

	// Filter
	int			temporalFilterFactorActual;
	int			temporalFilterThreshold;
	int			medianFilterEnable;
	int			averageFilterEnable;
	int			edgeFilterThreshold;
	int			interferenceUseLashValueEnable;
	int			interferenceLimit;
	int			edgefilterThresholdLow;
	int			edgefilterThresholdHigh;

	// Data IP
	uint32_t	dataIP;
}CamAppConfig, *LP_CamAppConfig;

typedef struct RxHeader_
{
	uint16_t version;
	uint16_t dataType;
	uint16_t width;
	uint16_t height;
	uint16_t roiX0;
	uint16_t roiY0;
	uint16_t roiX1;
	uint16_t roiY1;
	uint16_t intTime0;
	uint16_t intTime1;
	uint16_t intTime2;
	uint16_t offset;
	uint16_t temperature;
}RxHeader, *LP_RxHeader;

typedef struct Tofcam660Info_{

	RxHeader header;
	CamAppConfig config;

	SOCKET control_sock;
	SOCKET data_sock;

	int tofcamModeType;
	int led_control;
	int captureNetType;
	int printBasicInfo;
	int tofcamEvent_key;
	int rotate_90;
	int rxError;
	int actualNumber;
	int receivedBytes;
	double meanAvg;
	
	int imageWidth;
	int imageHeight;

	int usedPointCloud;
}TOFCAM660_INFO, *PTOFCAM660_INFO;


typedef struct TOFCAM660_DATABUF_
{
	CRITICAL_SECTION lock;
	int			overflow;
	int			head_idx;
	int 		tail_idx;
	int		 	bufLen[NSL3130_ETH_BUFF_SIZE];
	int		 	bufGrayLen[NSL3130_ETH_BUFF_SIZE];
	uint8_t 	tofcamBuf[NSL3130_ETH_BUFF_SIZE][NSL3130_BUFF_SIZE];
	uint8_t 	tofcamGrayBuf[NSL3130_ETH_BUFF_SIZE][NSL3130_BUFF_SIZE];

		TOFCAM660_DATABUF_() {
#ifdef _WINDOWS		
			InitializeCriticalSection(&lock);
#else
			pthread_mutex_init(&lock, NULL);
#endif
		}
		
		~TOFCAM660_DATABUF_() {
#ifdef _WINDOWS		
			DeleteCriticalSection(&lock);
#else
			pthread_mutex_destroy(&lock);
#endif
		}
}TOFCAM660_DATABUF, *PTOFCAM660_DATABUF;


typedef enum
{
    TOFCAM_DATA_UNDEFINED = -1,
    TOFCAM_DATA_DISTANCE_AMPLITUDE = 0,
    TOFCAM_DATA_DISTANCE = 1,
    TOFCAM_DATA_AMPLITUDE = 2,
    TOFCAM_DATA_GRAYSCALE = 3,
    TOFCAM_DATA_DCS = 4
}TOFCAM660_DATATYPE;

typedef enum
{
  COMMAND_SET_ROI = 0,											 ///<Set the ROI
  COMMAND_SET_INT_TIMES = 1, 									 ///<Set the integration times (all at once)
  COMMAND_GET_DISTANCE_AMPLITUDE = 2,							 ///<Get distance and amplitude (single or stream)
  COMMAND_GET_DISTANCE = 3,										 ///<Get distance (single or stream)
  COMMAND_GET_GRAYSCALE = 5, 									 ///<Get grayscale (single or stream)
  COMMAND_STOP_STREAM = 6,										 ///<Stop the stream
  COMMAND_GET_DCS = 7,											 ///<Get DCS data (single or stream)
  
  COMMAND_GET_DISTANCE_GRAYSCALE = 8,							 ///<Get distance and GRAYSCALE (single or stream) :: seobi add
  COMMAND_GET_DISTANCE_AMPLITUDE_GRAYSCALE = 9,					 ///<Get distance, amplitude and GRAYSCALE (single or stream) :: seobi add
  COMMAND_SET_ADC_OVERFLOW = 10, 								  ///<Set the adc overflow, saturation(enable/disable)
  COMMAND_SET_AUTO_INTEGRATION_TIME = 11,						  ///<Set the auto integration time
  COMMAND_GET_OBJECT_DETECTION_STANDALONE = 12,					///<Get distance and amplitude (single or stream)
  COMMAND_STOP_STANDALONE = 13,									  ///<Stop stand alone
  COMMAND_SET_DEEP_LEARNING = 14,								  ///<Set the DeepLearning (enable/disable)
  COMMAND_SAVE_APP_CONFIG = 15,									  ///<save config
  COMMAND_GET_OFFSET = 16,										  ///<Get Offset
  
  COMMAND_SET_OFFSET = 20,										 ///<Set the offset
  COMMAND_SET_MIN_AMPLITUDE = 21,								 ///<Set the minimal amplitude
  COMMAND_SET_FILTER = 22,										 ///<Set the filter settings (all at once)
  COMMAND_SET_MODULATION = 23,									 ///<Set the modulation settings
  COMMAND_SET_BINNING = 24,										 ///<Set the binning settings
  COMMAND_SET_HDR = 25,											 ///<Set the HDR settings
  COMMAND_SET_SHUTTER_MODE = 26, 								 ///<Set the shutter mode
  COMMAND_SET_ABS = 27,											 ///<Set the ABS (enable/disable)
  COMMAND_SET_COMPENSATION = 28, 								 ///<Set the compensations (enable/disable)
  COMMAND_SET_DLL_STEP = 29, 									 ///<Set the DLL step
  COMMAND_SHUTTER = 100, 										 ///<Force a shutter (in case of external shutter)
  COMMAND_CALIBRATE = 30,										 ///<Calibrate DRNU
  COMMAND_CALIBRATE_PRODUCTION = 31, 							 ///<Calibrate for production
  COMMAND_DELETE_CALIBRATION = 32,								 ///<Delete the calibration data
  COMMAND_DEBUG = 33,											 ///<Debug command with different sub commands
  COMMAND_CALIBRATE_GRAYSCALE = 34,								 ///<Calibrate Grayscale
  COMMAND_CALIBRATE_AMBIENT_LIGHT = 35,							 ///<Calibrate Ambient Light
  COMMAND_READ_CHIP_INFORMATION = 36,							 ///<Read chip ID and wafer ID
  COMMAND_READ_FIRMWARE_RELEASE = 37,							 ///<Read firmware release
  COMMAND_SET_DATA_IP_ADDRESS = 38,								 ///<Set the IP address of the data
  COMMAND_SET_GRAYSCALE_ILLUMINATION = 39,						  ///<Configure illumination during grayscale acquisition
  COMMAND_SET_CAMERA_IP_SETTINGS= 40,
  COMMAND_SET_CAMERA_MAC_ADDRESS= 41,
  COMMAND_WRITE_REGISTER= 42,
  COMMAND_READ_REGISTER= 43,
  COMMAND_IO_RESET = 44, 											 ///<reset chip in case of NACK
  COMMAND_SYSTEM_RESET = 45, 										 ///<system reset
  COMMAND_PSU_5V_ENABLE = 46,										 ///<psu 5v enable
  COMMAND_PSU_5V_DISABLE = 47,										 ///<psu 5v disable
  COMMAND_GET_CAMERA_IP_ADDRESS	= 48,							 ///<Get data ip address
  COMMAND_WRITE_CALIBRATION = 49,						 ///<Command to write the calibration data
  COMMAND_READ_CALIBRATION = 50, 						 ///<Command to READ the calibration data
  COMMAND_REQ_DEVLOG = 51,								   ///<Request dev log
  COMMAND_GET_TEMPERATURE = 74,									  ///<Command to read the temperature dec 74
  COMMAND_JUMP_TO_BOOTLOADER = 111,									  /// Jump to Bootloader
  COMMAND_SEND_FIRMWARE_UPDATE = 200							 ///<Send firmware update file
}TOFCAM660_CAMMAND_LIST;



/**
 * MIPI CSI and V4L2 camera capture using GStreamer and `nvarguscamerasrc` or `v4l2src` elements.
 * gstCamera supports both MIPI CSI cameras and V4L2-compliant devices like USB webcams.
 *
 * Examples of MIPI CSI cameras that work out of the box are the OV5693 module from the
 * Jetson TX1/TX2 devkits, and the IMX219 sensor from the Raspberry Pi Camera Module v2.
 *
 * For MIPI CSI cameras, the GStreamer element `nvarguscamerasrc` will be used for capture.
 * For V4L2 devices, the GStreamer element `v4l2src` will be used for camera capture.
 *
 * gstCamera uses CUDA underneath for any necessary colorspace conversion, and provides
 * the captured image frames in CUDA device memory, or zero-copy shared CPU/GPU memory.
 *
 * @note gstCamera now implements the videoSource interface and is intended to be used through 
 * that as opposed to directly. videoSource implements additional command-line parsing of 
 * videoOptions to construct instances. Some legacy APIs of gstCamera are now marked deprecated.
 *
 * @see videoSource
 * @ingroup camera
 */
class NSL3130AA : public videoSource
{
public:
	/**
	 * Create a MIPI CSI or V4L2 camera device.
	 */
	static NSL3130AA* Create( std::string ipaddr );
	~NSL3130AA();

	virtual void closeLidar();
	virtual bool Capture( void** image, int timeout=3000 );
	virtual void startCaptureCommand( int netType, CaptureOptions &camOpt) ;
	virtual void setKey(int cmdKey);
	virtual std::string getDistanceString(int distance );
	virtual int getDLWidth();
	virtual int getDLHeight();
	virtual int getWidth();
	virtual int getHeight();
	virtual int getVideoWidth();
	virtual int getVideoHeight();
	virtual bool isRotate90();
	virtual int getWidthDiv();
	virtual int getHeightDiv();
	virtual void setCameraSize(int width, int height);
	virtual std::string getLeftViewName();
	virtual void drawPointCloud();
	/**
	 * Default camera width, unless otherwise specified during Create()
 	 */
	static const uint32_t DefaultWidth  = 320;

	/**
	 * Default camera height, unless otherwise specified during Create()
 	 */
	static const uint32_t DefaultHeight = 240;

	
private:
	NSL3130AA( std::string ipaddr );
//	image makeImage(int w, int h, int c);
//	image matToImage(cv::Mat mat);
	bool hasValidStartMarking(unsigned char *data, int data_len);
	bool hasValidEndMarking(unsigned char * data, int data_len);
	bool lengthIsCorrect(unsigned char * data, int data_len);
	bool isValidData(unsigned char *data, int data_len);
	uint16_t getUint16FromCharBuffer(const char *data);
	int16_t getInt16FromCharBuffer(const char *data);
	int getCamInfo( uint8_t *data );
	uint16_t getHeaderUint16(uint8_t *pData, const int offset);
	uint32_t getHeaderUint32(uint8_t *pData, const int offset);
	int processUpdData(uint8_t *tempBuffer, int recvedLen, int resp_idx);
	int recvFromTcp(SOCKET sock, uint8_t *total_buf);
	void setGrayScaledColor(cv::Mat &imageLidar, int x, int y, int value, double end_range );
	int setDistanceColor(cv::Mat &imageLidar, int x, int y, int value );
	void setAmplitudeColor(cv::Mat &imageLidar, int x, int y, int value );
	int getDistanceAmplitude(cv::Mat &imageDistance, cv::Mat &imageAmplitude, bool bUsedPointCloud);
	int getGrayscaled(cv::Mat &imageLidar, bool bUsedPointCloud);
	int sendToDev(SOCKET sock, uint8_t *pData, int nLen, int expectedLen = 0);
	void reqOverflow(SOCKET control_sock);
	void reqHdrMode(SOCKET control_sock);
	void reqCompensation(SOCKET control_sock);
	int reqStreamingFrame(SOCKET control_sock);
	uint8_t getCommandByType( int modeType );
	void reqSingleFrame(SOCKET control_sock, int modeType);
	void reqStopStream(SOCKET control_sock);
	void reqIntegrationTime(SOCKET control_sock);
	void reqFilterParameter(SOCKET control_sock);
	void reqMinAmplitude(SOCKET control_sock);
	void reqSetROI(SOCKET control_sock);
	void reqGrayscaleLedControl(SOCKET control_sock, int ledOnOff);
	void reqDualBeam(int control_sock);
	void reqDeepLearning(int control_sock);

	SOCKET InitializeControlsocket(void);
	SOCKET InitializeDataSocket(void);
	double interpolate( double x, double x0, double y0, double x1, double y1);
	void createColorMapPixel(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue);
	uint8_t *convertParameterFromConfig(uint8_t *pData, int nLen);
	void initializeTofcam660(SOCKET socket);
	void *rxTofcam660(void *arg);
	void rxSocket(uint8_t *socketbuff, int buffLen);
	void keyProc();

#ifdef _WINDOWS
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcbVis();
	pcl::visualization::PCLVisualizer::Ptr rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

	static UINT WINAPI rxWrapper(void* thisPtr) {
    	((NSL3130AA*) thisPtr)->rxTofcam660(NULL);
		
		_endthreadex( 0 );
	    return 0;
	}

	
	HANDLE hThread;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_ptr;
	pcl::visualization::PCLVisualizer::Ptr viewer;

	LensTransform  lensTransform;
#else
	int rxSerial(uint8_t *socketbuff, int buffLen, bool addQue);
	int flushRx(void);
	int setSerialBaudrate(void);
	static void* rxWrapper(void* thisPtr) {
		((NSL3130AA*) thisPtr)->rxTofcam660(NULL);
		return NULL;
	}

	pthread_t threadID;
#endif

	bool		 ttySerial;
	std::string  mIpaddr;
	
//	TOFCAM_635INFO tofcamInfo;	
//	uint8_t tofcambuff[4][DIST_AMPLITUDE_TOTAL_SIZE];
//	std::vector<uchar3> colorVector;	
	ImageFrame tofcamImage;
	
	
	TOFCAM660_INFO		tofcamInfo;
	TOFCAM660_DATABUF	tofcamBuff;
	int					exit_thtread;

	int 				distanceTable[NSL3130_IMAGE_WIDTH*NSL3130_IMAGE_HEIGHT];
	uint8_t 			procBuff[2][NSL3130_BUFF_SIZE];
	uint8_t 			response[2][NSL3130_BUFF_SIZE];

	std::vector<cv::Vec3b> colorVector;

	double	sin_angle;
	double	cos_angle;

};

#endif
