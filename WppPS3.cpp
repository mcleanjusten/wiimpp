#include "WppCamera.h"
#include <stdio.h>

WppPS3::WppPS3() 
{
  pCam = 0;
}

PBYTE WppPS3::getPixels()
{
	return pBuffer;
}

bool WppPS3::isFrameNew()
{
	return pCam->GetFrame(pBuffer, 24, false);
}

void WppPS3::start(int width,int height, int framerate)
{
  if (pCam == 0) {
  	pCam = IPS3EyeLib::Create();
	  pCam->SetFormat(IPS3EyeLib::GetFormatIndex(width, height, framerate));
	  pBuffer = new BYTE[(width*height*24)/8];
  }
	pCam->StartCapture();
	pCam->AutoAGC(false);
	pCam->AutoAEC(true);
	pCam->SetGain(0);
}

void WppPS3::stop()
{
  if (pCam != 0) {
  	pCam->StopCapture();
  }
}

int WppPS3::getCamWidth()
{
	return pCam->GetWidth();
}

int WppPS3::getCamHeight()
{
	return pCam->GetHeight();
}

WppPS3::~WppPS3()
{
	stop();
	Sleep(50);
	delete pCam;
	delete [] pBuffer;
}
