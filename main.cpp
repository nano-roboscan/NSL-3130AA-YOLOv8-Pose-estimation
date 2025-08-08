#include "main.h"
#include <stdio.h>
#include <signal.h>

#include "YoloDet.h"
#include "YoloPose.h"
#include "ImageTools.h"

#include "videoSource.h"
#include "timeCheck.h"


int signal_recieved = 0;

void sig_handler(int signo)
{
	if( signo == SIGINT )
	{
		printf("received SIGINT\n");
		signal_recieved = 1;
	}
}


int main(int argc, char **argv) {

	CaptureOptions camOpt;
	if( signal(SIGINT, sig_handler) == SIG_ERR )
		printf("can't catch SIGINT\n");

#ifdef HAVE_CV_CUDA
	printf("=================== YOLO CUDA MODE ===================\n");
#else
	printf("=================== YOLO CPU MODE ===================\n");
#endif

	videoSource* lidarSrc = createApp(argc, argv, camOpt);

	lidarSrc->initDeepLearning(camOpt);
	lidarSrc->setLidarOption(camOpt);
	TimeCheck tmChk;
	tmChk.setPrint(false);

	while ( !signal_recieved )
	{
		tmChk.setBegin();
		if (!lidarSrc->captureLidar(3000, camOpt)) {
			printf("capture : failed...\n");
			break;
		}

		tmChk.setEnd();
		tmChk.printTime("capture");

		tmChk.setBegin();
		camOpt.detectingCnt = lidarSrc->deepLearning(camOpt.frameMat, camOpt);
		tmChk.setEnd();
		tmChk.printTime("detection");
		
		tmChk.setBegin();
		lidarSrc->drawCaption(camOpt.frameMat, camOpt.distMat, camOpt);
		tmChk.setEnd();
		tmChk.printTime("draw");

		if (lidarSrc->prockey(camOpt) == 27) // ESC
			break;
	}

	lidarSrc->stopLidar();
	delete lidarSrc;
	lidarSrc = NULL;

    return 0;
}


