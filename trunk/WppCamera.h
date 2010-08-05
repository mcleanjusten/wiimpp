#ifndef WppPS3_H
#define WppPS3_H

#include "IPS3EyeLib.h"

namespace CV
{
  #include <cv.h>
  #include <highgui.h>
}

class WppCamera
{
  public:
    virtual void start(int width,int height, int framerate) = 0;
    virtual void stop() = 0;
    virtual int getCamWidth() = 0;
    virtual int getCamHeight() = 0;
    virtual int getNbChannels() = 0;
    virtual bool isFrameNew() = 0;
    virtual PBYTE getPixels() = 0;
};

class WppPS3 : public WppCamera
{
  public:
    WppPS3();
    ~WppPS3();
    void start(int width,int height, int framerate);
    void stop();
    int getCamWidth();
    int getCamHeight();
    int getNbChannels() {return 3;};
    bool isFrameNew();
    PBYTE getPixels();

    IPS3EyeLib	*pCam;
    PBYTE		pBuffer;

  private:
    // This acts as a handle to the camera.
    int camWidth;
    int camHeight;
    // Enumerate the cameras on the bus.
    static unsigned int	camNum;
};

class WppCapture : public WppCamera
{
    public:

      WppCapture(bool useVideo, const char* videoName);
      ~WppCapture();
      void start(int width,int height, int framerate);
      void stop();
      int getCamWidth();
      int getCamHeight();
      int getNbChannels();
      bool isFrameNew();
      PBYTE getPixels();

    private:

      CV::CvCapture* mCapture;
      bool mUseVideo;
      const char* mVideoName;
      int camWidth;
      int camHeight;
      CV::IplImage*	mSource;

};

#endif // WppPS3_H_
