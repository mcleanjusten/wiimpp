#pragma once

#include <Ogre.h>
#include "wiimote.h"
#include "ip/UdpSocket.h"

namespace CV
{
  #include <cv.h>
  #include <highgui.h>
}

using namespace Ogre;

struct HoughCircle
{
  unsigned short radius;
  unsigned short intensity;
  CV::CvPoint center;
};

class D6Mote : public wiimote
{
public:
  D6Mote(int width, int height, float diameter);
  ~D6Mote(void);
  bool processWiiData();
  bool processImageData(CV::IplImage * img);
  void initCalibration();
  bool isInitialized(){return mInitialized;};
  Vector3 getAngleRates(){return mAngleRates;};
  bool isCalibrating();
  bool isInFieldOfView(){return mInFoV;};
  void setFilterParams(int range1, int blur, int amp, int minRadius);
  void setTrackColor(CV::IplImage * img, int x, int y, int size = 1);
  CV::CvBox2D getContour();
  CV::CvRect getROI();
  int getID() {return id;};
  CV::IplImage *getColorMask() {return mColorMask;};
  CV::IplImage *getFgMask() {return mFgMask;};
  CV::IplImage *getProcessedImage() {return mColorImage;};
  CV::IplImage *getCalibrationImage() {return mCalibrationImage;};
  Vector3 getPosition() {return mPosition;};
  Quaternion getOrientation() {return Quaternion(mOrientation);};

protected:
  void updateCalibration(Real dt);
  void resetROI();
  void setROI(const CV::CvRect &roi);
  void processFilters();
  void updateTrackColor(CV::IplImage * img, CV::CvRect region);
  bool fitCircle(CV::IplImage *img, int minRadius, int maxRadius, CV::CvBox2D32f &blob);
  bool fitEllipse(CV::IplImage *img, int minRadius, int maxRadius, CV::CvBox2D32f &blob);
  void accum_pixel(CV::IplImage *img, const CV::CvRect &bounds, const CV::CvPoint &pt, HoughCircle &circle);

  int id;
  int mWidth;
  int mHeight;
  Vector3 mPosition;
  Matrix3 mOrientation;
  Vector3 mMaxNoise;
  Vector3 mMinNoise;
  Vector3 mCalibration;
  Real mCalibrationTimeout;

  Vector3 mAngleRates;
  Vector3 mPrevAngleRates;
  Vector3 mAcceleration;
  Vector3 mVelocity;
  bool mCalibrateYaw;
  bool mInitialized;
  bool mInFoV;
  bool mBlobFound;
  Real mTime;
  Real mPrevTime;
  Real mAccCalibrationTime;


  CV::IplImage* mColorImage;
  CV::IplImage* mGrayImage;
  CV::IplImage* mCalibrationImage;
  CV::IplImage* mBgRefImage;
  CV::IplImage* mFgMask;
  CV::IplImage* mColorMask;
  CV::CvMemStorage *mContour_storage;

  int mTrack_color[3];
  int mRange1;
  int mHueMax;
  int mHueMin;
  int mBlur;
  int mAmp;
  int mMinRadius;

  Real mTrackedObjDiam;
  CV::CvBox2D mBlob;
  Radian mCamAngle;
  Radian mYawAngle;
  bool mColorPicked;
  float mImageTime;
};
