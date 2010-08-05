#include "WppCamera.h"
#include <stdio.h>

WppCapture::WppCapture(bool useVideo, const char* videoName) 
{
  mUseVideo = useVideo;
  mVideoName = videoName;
  mCapture = 0;
  mSource = 0;
}

PBYTE WppCapture::getPixels()
{
	return (PBYTE)mSource->imageData;
}

bool WppCapture::isFrameNew()
{
  if (CV::cvGrabFrame(mCapture))
  {
    mSource = CV::cvRetrieveFrame(mCapture);
    return true;
  }
  else
  {
    // In case of test with video, loop at end. FIXME is there a better way to do this?
    CV::cvReleaseCapture(&mCapture);
    mCapture = CV::cvCaptureFromAVI( "colortracking.avi" ); 
    return false;
  }

  return false;

}

void WppCapture::start(int width,int height, int framerate)
{
  if (mUseVideo) {
    mCapture = CV::cvCaptureFromAVI( mVideoName );
  }else {
    mCapture = CV::cvCaptureFromCAM(0);
  }

  CV::IplImage* img = CV::cvQueryFrame(mCapture);
  CV::CvSize s = CV::cvGetSize(img);
  camWidth = s.width;
  camHeight = s.height;
  CV::cvReleaseImage(&img);
  mSource = CV::cvCreateImage( CV::cvSize(camWidth,camHeight), 8, 3 );

}

int WppCapture::getNbChannels()
{
  if (mSource != 0) {
    return mSource->nChannels;
  }
  return 0;
}

void WppCapture::stop()
{
  if (mCapture != 0) {
    CV::cvReleaseCapture(&mCapture);
  }
  CV::cvReleaseImage(&mSource);
}

int WppCapture::getCamWidth()
{
	return camWidth;
}

int WppCapture::getCamHeight()
{
	return camHeight;
}

//Clean up
WppCapture::~WppCapture()
{
	// Stop capturing
	stop();
	Sleep(50);

}
