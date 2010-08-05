// 
//	Addition to gl.tter's WiiYourself library that adds 6DOF tracking capabilities 
//  to Wiimotes with Motion Plus extention.
//
//   "contains WiiYourself! wiimote code by gl.tter
//    http://gl.tter.org"
//
//



#include "D6Mote.h"

#define NOISE_FILTER 1.5
#define EPSILON 0.0001
#define CALIB_TIME 5.0
#define MAX_CONTOURS 4
#define YAW_ZERO_ZONE 0.1 // Tolerance level when finding yaw calibration dot in sphere (%)
#define YAW_ZERO_ANGLE 1.0 // Tolerance angle when calibrating yaw 


//TODO-List:
// - Predict position by use of blob velocities, and accelerometers
// - don't force Wii calibration.
// - FG/BG segmentation statistical model??
// - Remove Ogre dependencies: Vector, Math, Quaternion functions.

#define USE_OSC

D6Mote::D6Mote(int width, int height, float diameter):mWidth(width), mHeight(height), mTrackedObjDiam(diameter)
{
    mPosition = Vector3::ZERO;
    //mOrientation = Quaternion::IDENTITY;

    CV::CvSize camSize = CV::cvSize(width, height);
    mColorImage = CV::cvCreateImage( camSize, 8, 3 );
    mGrayImage = CV::cvCreateImage( camSize, 8, 1 );
    mCalibrationImage = CV::cvCreateImage( camSize, 8, 1 );
    mFgMask = CV::cvCreateImage( camSize, 8, 1 );
    mBgRefImage = CV::cvCreateImage( camSize, 8, 1 );
    mColorMask = CV::cvCreateImage( camSize, 8, 1);
    mBlob.center.x = mBlob.center.y = 0;
    mBlob.size.width = width;
    mBlob.size.height = height;
    mBlob.angle = 0.0;

    mTrack_color[0] = 180;
    mTrack_color[1] = 256;
    mTrack_color[2] = 256;
    mRange1 = 20;
    mHueMax = mTrack_color[0] + mRange1;
    mHueMin = mTrack_color[0] - mRange1;
    mBlur = 5;
    mAmp = 128;
    mMinRadius = 5;
    mCamAngle = Radian(Degree(0.5*75)); // This is the FOV of the PS3Eye camera!!!
    mYawAngle = Radian(0.0);

    mCalibrateYaw = false;
    mInitialized = false;
    mColorPicked = false;
    mBlobFound = false;
    mInFoV = false;
    
    mTime = mImageTime = mAccCalibrationTime = Ogre::Root::getSingleton().getTimer()->getMicroseconds();
    static int wiimote_ID = 0;
    id = wiimote_ID++;
}

D6Mote::~D6Mote(void)
{
  if (IsConnected())
  {
    SetLEDs(0x08);
  	Disconnect();
    Sleep(1000);
  }
}

bool D6Mote::processWiiData()
{
    Real newTime = Ogre::Root::getSingleton().getTimer()->getMicroseconds();
    if (!mInitialized) {

      //This is the first report we get, init calibration sequence
      //initCalibration();
      mTime = newTime;
      mInitialized = true;
      return true;
    }

    Real dt = 0.000001*(newTime - mTime);

    if (Button.One()) 
    {
      initCalibration();
    } 
    else if(isCalibrating())
    {
      updateCalibration(dt);
    }


    // ANGLES
    // Filter the angle rates
    //Vector3 calib = 0.5*(mMinNoise + mMaxNoise);
    Vector3 noiselevel = 0.5*(mMaxNoise - mMinNoise);

    //Wiiyourself uses a RH y-down coordinate sys. Change to RH y-up (Ogre)
    Real yawRate =   -(MotionPlus.Speed.Yaw - mCalibration.x);
    Real pitchRate = (MotionPlus.Speed.Pitch - mCalibration.y);
    Real rollRate =  -(MotionPlus.Speed.Roll - mCalibration.z);
    if (Math::Abs(yawRate) < noiselevel.x) yawRate = 0.0;
    if (Math::Abs(pitchRate) < noiselevel.y) pitchRate = 0.0;
    if (Math::Abs(rollRate) < noiselevel.z) rollRate = 0.0;
    Vector3 newAngleRates(yawRate, pitchRate, rollRate);

    // Simpson integration of angles
    Vector3 deltaAngles = dt*(mPrevAngleRates + newAngleRates + 4*mAngleRates) / 6;

    // Trapezoidal integration
    //Vector3 deltaAngles = 0.5*dt*(mAngleRates + newAngleRates);
    
    Vector3 newAcceleration = mAcceleration;
    Vector3 newVelocity = mVelocity;

    // Update node
    if (!isCalibrating()) {

      Vector3 localAcc(Acceleration.X, -Acceleration.Z, -Acceleration.Y);

      Radian pitch_acc, roll_acc;
      Radian yaw_gyr, pitch_gyr, roll_gyr;
      bool useAcc = false;
      Matrix3 delta;
      delta.FromEulerAnglesYXZ(
        Radian(Degree(deltaAngles.x)), 
        Radian(Degree(deltaAngles.y)),
        Radian(Degree(deltaAngles.z))
        );
      Matrix3 rotGyr = mOrientation * delta;

      // At slow movement and pitch/roll angles near zero, calibrate pitch and roll according to gravity
      Real weight = 0.0;
      Real acc_tolerance = 0.02;
      Real ang_speed_tolerance = 5.0;
      //Real weight = k0- k1*Math::Abs(localAcc.length() - 1) - k2*Math::Abs(newAngleRates.length());
      Real acc = Math::Abs(localAcc.length()-1);
      Real ang_speed = newAngleRates.length();
      if (acc < acc_tolerance && ang_speed < ang_speed_tolerance && Math::Abs(Acceleration.Orientation.Pitch)<60.0) {
        // accelerometers
        // Set orientation according to accelerometers
        // Here we add some intelligent weighting. weight should depend on a) how close acc is to 1.0g and how 
        // long time it was since last recalibration.
        Real k1 = 1.0;
        Real k2 = 0.5;
        Real weight1 = k1*Math::Abs(localAcc.length() - 1);
        Real weight2 = k2*Math::Abs(0.000001*(newTime-mAccCalibrationTime));
        weight = weight1/(weight1+weight2);

        pitch_acc = Radian(Degree(Acceleration.Orientation.Pitch)); 
        roll_acc = -Radian(Degree(Acceleration.Orientation.Roll));
        mAccCalibrationTime = newTime;
        useAcc = true;
      }
      
      if (useAcc) {
        rotGyr.ToEulerAnglesYXZ(yaw_gyr, pitch_gyr, roll_gyr);
        mOrientation.FromEulerAnglesYXZ(yaw_gyr, weight * pitch_acc + (1-weight)*pitch_gyr, weight * roll_acc + (1-weight)*roll_gyr);
      } else {
        mOrientation = rotGyr;
      }

      if (mCalibrateYaw) {
        Radian yaw, pitch, roll;
        mOrientation.ToEulerAnglesYXZ(yaw, pitch, roll);
//        if (Math::Abs(yaw.valueDegrees()) > YAW_ZERO_ANGLE)
//        {
          //yaw = Radian(0.0);
          yaw = mYawAngle;
          mOrientation.FromEulerAnglesYXZ(yaw,pitch,roll); 
//        }
        mCalibrateYaw = false;

      //  // POSITION
      //  Vector3 globalAcc = mNode->getOrientation().Inverse()*localAcc;
      //  Vector3 newAcceleration = 9.82 * (globalAcc - Vector3(0,-1,0));
      //  Vector3 globalAcc = mNode->getOrientation().Inverse()*localAcc;

      //  // Trapezoidal integration
      //  Vector3 newVelocity = mVelocity + 0.5*dt*(newAcceleration + mAcceleration);
      //  Vector3 translation = 0.5*dt*(newVelocity + mVelocity);

      //  // TODO check this out. Must have calculated wrong somewhere, 
      //  // mNode->translate(translation);      // This gives wrong motion along negative y. WHY ???
      //  mNode->translate(translation.x, -translation.y, translation.z);

      //  mAcceleration = newAcceleration;
      //  mVelocity = newVelocity;
      //} else {
      //  mNode->setPosition(Vector3::ZERO);
      //  mVelocity = Vector3::ZERO;
      }
    }

    // Save states
    mPrevAngleRates = mAngleRates;
    mAngleRates = newAngleRates;
    mPrevTime = mTime;
    mTime = newTime;

    return true;
}

void D6Mote::initCalibration()
{
  mCalibrationTimeout = CALIB_TIME;
  mOrientation.FromEulerAnglesYXZ(Radian(0), Radian(0), Radian(0));
  mMaxNoise = mMinNoise = mCalibration = Vector3(MotionPlus.Speed.Yaw, MotionPlus.Speed.Pitch, MotionPlus.Speed.Roll);
}

bool D6Mote::isCalibrating()
{
  return mCalibrationTimeout > 0;
}

void D6Mote::updateCalibration(Real dt)
{
  Vector3 rates(MotionPlus.Speed.Yaw, MotionPlus.Speed.Pitch, MotionPlus.Speed.Roll);
  mCalibration = rates;
  mMaxNoise.x = MAX(mMaxNoise.x, rates.x);
  mMaxNoise.y = MAX(mMaxNoise.y, rates.y);
  mMaxNoise.z = MAX(mMaxNoise.z, rates.z);
  mMinNoise.x = MIN(mMinNoise.x, rates.x);
  mMinNoise.y = MIN(mMinNoise.y, rates.y);
  mMinNoise.z = MIN(mMinNoise.z, rates.z);

  // Check that the offset is within a given interval, else the wiimote is moving and we need to recalibrate
  //if (mMaxNoise.x - mMinNoise.x > NOISE_FILTER || mMaxNoise.y - mMinNoise.y > NOISE_FILTER || mMaxNoise.z - mMinNoise.z > NOISE_FILTER) {
  //   initCalibration();
  //}

  // If the wiimote is moving we need to recalibrate
  if ((rates - (0.5 * (mMaxNoise + mMinNoise))).length() > NOISE_FILTER) {
     initCalibration();
  }

  // Flash leds while calibrating 
  if (mCalibrationTimeout - (int) mCalibrationTimeout - dt < 0)
    SetLEDs(0x0f);
  if (Math::Abs(mCalibrationTimeout - (int) mCalibrationTimeout - 0.5) - dt < 0)
    SetLEDs(0x0);

  mCalibrationTimeout -= dt;
}

void D6Mote::setFilterParams(int range1, int blur, int amp, int minRadius)
{
  mRange1 = range1;
  mBlur = blur;
  mAmp = amp;
  mMinRadius = minRadius;
}

bool D6Mote::processImageData(CV::IplImage * srcImg)
{

  // Here we do color tracking to find the colored sphere. From blob size and camera poition 
  // we calculate its the 3D-position in space.

  // TODO maybe update track color of blob detection to make more robust tracking
  // TODO predict next ROI on blob velocity
  int maxTries = 3;
  float roiFactor = 3.0f; // Size of the roi in terms of found blob size

  for (int nTries = 0; nTries < maxTries; ++nTries)
  {
    // TODO prepareImages();
    CV::CvSize size = CV::cvGetSize(srcImg); 
    CV::CvRect roi = CV::cvRect(0,0,size.width, size.height);
    int resizeFactor = 1;

    if (mBlobFound){
      // The blob was found in the previous frame. Use it to optimize finding it in this frame.
      int x = MAX(0, mBlob.center.x-0.5*mBlob.size.width*roiFactor);
      int y = MAX(0, mBlob.center.y-0.5*mBlob.size.height*roiFactor);
      int w = MAX(mMinRadius*2, roiFactor*mBlob.size.width);
      int h = MAX(mMinRadius*2, roiFactor*mBlob.size.width);
      roi = CV::cvRect(x, y, MIN(w, size.width-x), MIN(h, size.height-y));
        
      // If the blob is expected to be small, we resize the image to get better accuracy
      resizeFactor = mBlob.size.width < 35 ? 2 : 1;
    }

    CV::CvRect dstRoi = CV::cvRect(0, 0, roi.width*resizeFactor, roi.height*resizeFactor);
    setROI(dstRoi);

    // Copy image from source
    CV::cvSetImageROI(srcImg, roi);
    if (resizeFactor == 1)
    {
      CV::cvCopy( srcImg, mColorImage, 0 );
    }
    else
    {
      CV::cvResize( srcImg, mColorImage, CV_INTER_CUBIC);
    }
    CV::cvResetImageROI(srcImg);

    // Copy to grayscale image 
    CV::cvCvtColor(mColorImage, mGrayImage, CV_BGR2GRAY);

    //if (isCalibrating()) 
    //{
    //  // Save reference image for background segmentation
    //  CV::cvCopy(mGrayImage, mBgRefImage);
    //}

    // Work in HSV color space to improve color tracking
    CV::cvCvtColor( mColorImage, mColorImage, CV_BGR2HSV );

    if (!mColorPicked)
      return false;

    // Filter image
    processFilters();

    // Try to find the blob.
    CV::CvBox2D32f blob;
    int minRadius = mMinRadius;
    int maxRadius = dstRoi.height/2;

    // Now try to find the 
    //if (!mBlobFound || nTries > 0)
    //{
    //  mBlobFound = fitEllipse(mColorMask, minRadius, maxRadius, blob);
    //}
    //else
    //{
      mBlobFound = fitCircle(mColorMask, minRadius, maxRadius, blob);
    //}

    if (mBlobFound)
    {
      // Calc position from blob properties
      // Get the blob diameter in pixels. Since blob should be circular, either w or h could be used. 
      // However, motion bluring, partly hidden blob and noise might make the blob differently shaped. 
      // For now, use the mean value (w*h/2). A more sofisticated calculation may be needed to account for special cases.
      mBlob.center.x = blob.center.x/resizeFactor + roi.x;
      mBlob.center.y = blob.center.y/resizeFactor + roi.y;
      mBlob.size.width = blob.size.width/resizeFactor;
      mBlob.size.height = blob.size.height/resizeFactor;

      Real diam = 0.5 * (mBlob.size.width + mBlob.size.height); 
      Real radius = 0.5 * diam;
      Real imgplaneW = mTrackedObjDiam * mWidth/diam;
      Real imgplaneH = mTrackedObjDiam * mHeight/diam;
      Real x = imgplaneW*(mBlob.center.x-0.5*mWidth)/mWidth;
      Real y = -imgplaneH*(mBlob.center.y-0.5*mHeight)/mHeight;

      Real z = Math::Sqrt(imgplaneW*imgplaneW + imgplaneH*imgplaneH)/(2.0*Math::Tan(mCamAngle)); // TODO convert to vertical FOV; angle_v = atan(tan(37.5) / Sqrt(1 + w*w/(h*h)))
      Vector3 spherePosition(x,y,z);
      //mPosition = spherePosition;
      // wiimote center -> sphere offset ~15 cm.
      mPosition = spherePosition - mOrientation * Vector3(0,0,-0.15);

      // See if we can find a dark blob inside the light blob. 
      // If so, use it for yaw calibration
      // Make sure the center does not include the corners of the blob box. 
      CV::CvRect centerRoi;
      centerRoi.x = CV::cvRound(blob.center.x - 0.35*blob.size.width);
      centerRoi.y = CV::cvRound(blob.center.y - 0.35*blob.size.height);
      centerRoi.width = 0.7*blob.size.width;
      centerRoi.height = 0.7*blob.size.height;

      if (centerRoi.x >= 0 && centerRoi.y >= 0 && 
        (centerRoi.height + centerRoi.y) < mWidth && 
        (centerRoi.height + centerRoi.y) < mHeight && 
        centerRoi.width > 10 && centerRoi.height > 10)
      {
        // Set roi to blob box minus corners
        CV::cvSetImageROI(mCalibrationImage, centerRoi);
        CV::CvBox2D32f center;
        if (fitEllipse(mCalibrationImage, 0, maxRadius, center)) {
          Real offCenterX = (blob.center.x - center.center.x)/resizeFactor;
          // If calibration blob is close to the center
          if (Math::Abs(offCenterX)/mBlob.size.width < YAW_ZERO_ZONE) {
            mYawAngle = Math::ATan(imgplaneW*offCenterX/(mWidth*mTrackedObjDiam*0.5));
            mCalibrateYaw = true;
          }
        }

      }

      return true;
    }
  }

  return false;   
}

CV::CvBox2D D6Mote::getContour(){
  return mBlob;
}

CV::CvRect D6Mote::getROI(){
  return CV::cvGetImageROI(mColorImage);
}

void D6Mote::resetROI()
{
    CV::cvResetImageROI(mColorImage);
    CV::cvResetImageROI(mGrayImage);
    CV::cvResetImageROI(mCalibrationImage);
    CV::cvResetImageROI(mFgMask);
    CV::cvResetImageROI(mBgRefImage);
    CV::cvResetImageROI(mColorMask);
}

void D6Mote::setROI(const CV::CvRect &roi)
{
    CV::cvSetImageROI(mColorImage, roi);
    CV::cvSetImageROI(mGrayImage, roi);
    CV::cvSetImageROI(mCalibrationImage, roi);
    CV::cvSetImageROI(mFgMask, roi);
    CV::cvSetImageROI(mBgRefImage, roi);
    CV::cvSetImageROI(mColorMask, roi);
}

void D6Mote::processFilters()
{
  //CV::cvSmooth( mColorImage, mColorImage, CV_BLUR, (mBlur * 2) + 1);

  int min_color[3] = {MAX(0, mHueMin - mRange1/5), MAX(0, mTrack_color[1] - 70), MAX(0, mTrack_color[2] - 70)};
  int max_color[3] = {MIN(256, mHueMax + mRange1/5), MIN(256, mTrack_color[1] + 70), MIN(256, mTrack_color[2] + 70)};

  CV::cvInRangeS(mColorImage,CV::cvScalar(min_color[0],min_color[1],min_color[2]),CV::cvScalar(max_color[0],max_color[1],max_color[2]), mColorMask);

  /*
  // Very basic background segmentation.
  CV::cvAbsDiff(mGrayImage, mBgRefImage, mFgMask);
  CV::cvThreshold(mFgMask, mFgMask, 15, 255, CV_THRESH_BINARY);

  // BG filter
  CV::cvAnd(mFgMask, mGrayImage, mGrayImage);
  */

  // Amplify filter
  //CV::cvMul( mGrayImage, mGrayImage, mGrayImage, mAmp/1000.0f );

  // Copy image to calibration image
  CV::cvCopy( mGrayImage, mCalibrationImage);

  // Color filter
  //CV::cvAnd(mColorMask, mGrayImage, mGrayImage);
  CV::cvOr(mColorMask, mCalibrationImage, mCalibrationImage);

  // Blur filter
  //CV::cvSmooth( mGrayImage, mGrayImage, CV_BLUR, (mBlur * 2) + 1);
  //CV::cvSmooth( mGrayImage, mGrayImage, CV_BLUR, (mBlur * 2) + 1);

  // Threshold
  //CV::cvThreshold( mColorMask, mColorMask, 125, 255, CV_THRESH_BINARY);
  CV::cvThreshold( mCalibrationImage, mCalibrationImage, 80, 255, CV_THRESH_BINARY_INV);
}

bool D6Mote::fitCircle(CV::IplImage *img, int minRadius, int maxRadius, CV::CvBox2D32f &blob)
{
  // This algorithm performs circle fitting by a hough circle transform. 
  // It assumes the circle center is inside the conture and that the diameter
  // is roughly defined by the bounding box.
  // This speeds up search and reduces the risk to find false centers. 
  // On the other hand, it gives bad results if the circles vertical and horizontal diameter is occluded

  CV::CvPoint* PointArray;
  mContour_storage = CV::cvCreateMemStorage();

  CV::CvSeq* contour_list = NULL;

  // cvFindContours corrupts the image. Use a copy instead
  CV::CvSize size = {img->width, img->height};
  CV::IplImage *imgCopy = CV::cvCreateImage( size, img->depth, img->nChannels);
  CV::CvRect roi = CV::cvGetImageROI(img);
  CV::cvSetImageROI(imgCopy, roi);
  CV::cvCopy(img, imgCopy);

	CV::cvFindContours( imgCopy, mContour_storage, &contour_list, sizeof(CV::CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	CV::CvSeq* contour_ptr = contour_list;

	int nCvSeqsFound = 0;

  bool found = false;

  // save the contour with largest area
	while( (contour_ptr != NULL) ) 
  {
    int count = contour_ptr->total; // This is number point in contour
    float area = fabs( CV::cvContourArea(contour_ptr, CV::CV_WHOLE_SEQ) );

    // Bounding box gives a rough estimation of the radius 
    CV::CvRect rect = CV::cvBoundingRect( contour_ptr, 0 );
    Real estimatedRadius = 0.5*MAX(rect.width, rect.height);
    maxRadius = MIN(maxRadius, estimatedRadius + 5);
    minRadius = MAX(minRadius, estimatedRadius - 5);

    //Area gives a much better radius estimation, but it can't handle occlusions
    Real r = sqrt(area/Math::PI);
    /*
    if (r >= minRadius && r <= maxRadius)
    {
      CV::CvMoments moments;
      Real M00, M01, M10;
      CV::cvMoments(imgCopy,&moments,1);
      M00 = CV::cvGetSpatialMoment(&moments,0,0);
      M10 = CV::cvGetSpatialMoment(&moments,1,0);
      M01 = CV::cvGetSpatialMoment(&moments,0,1);
      blob.center.x = M10/M00;
      blob.center.y = M01/M00;
      blob.size.width = 2*r;
      blob.size.height = 2*r;
      blob.angle =  0.0;
      found = true;
      contour_ptr = contour_ptr->h_next;
      continue;
    }
    */

    // If area -> radius calculation fails. Caluculate area by a hough transform.

    // Alloc memory for contour point set.    
    PointArray = (CV::CvPoint*)malloc( count*sizeof(CV::CvPoint) );
    
    // Get contour point set.
    CV::cvCvtSeqToArray(contour_ptr, PointArray, CV::CV_WHOLE_SEQ);
    
    // Loop through the image and draw circles with radiuses reaching from r_min to r_max
    std::vector<HoughCircle> savedCircles;

    CV::IplImage * houghImg = CV::cvCreateImage(CV::cvSize(rect.width, rect.height), 8, 1);
    for (int radius = minRadius; radius<=maxRadius; ++radius)
    {
      CV::cvSetZero(houghImg);

      HoughCircle circle = {0,0,{0,0}};
      circle.radius = radius;
      for(int i=0; i<count; i++)
      {
        CV::CvPoint pt = CV::cvPoint(PointArray[i].x - rect.x, PointArray[i].y - rect.y);
        //CV::cvFloodFill(mColorImage, pt, CV::cvScalar(mTrack_color[0], mTrack_color[1], mTrack_color[2]), CV::cvScalarAll(20), CV::cvScalarAll(20));
        // draw a circle round the contour point
        int f = 1 - radius;
        int ddF_x = 1;
        int ddF_y = -2 * radius;
        int x = 0;
        int y = radius;
        
        accum_pixel(houghImg, rect, CV::cvPoint(pt.x, pt.y + radius), circle);
        accum_pixel(houghImg, rect, CV::cvPoint(pt.x, pt.y - radius), circle);
        accum_pixel(houghImg, rect, CV::cvPoint(pt.x + radius, pt.y), circle);
        accum_pixel(houghImg, rect, CV::cvPoint(pt.x - radius, pt.y), circle);
        
        while(x < y)
        {
          if(f >= 0)
          {
            y--;
            ddF_y += 2;
            f += ddF_y;
          }
          
          x++;
          ddF_x += 2;
          f += ddF_x;
          
          accum_pixel(houghImg, rect, CV::cvPoint(pt.x + x, pt.y + y), circle);
          accum_pixel(houghImg, rect, CV::cvPoint(pt.x - x, pt.y + y), circle);
          accum_pixel(houghImg, rect, CV::cvPoint(pt.x + x, pt.y - y), circle);
          accum_pixel(houghImg, rect, CV::cvPoint(pt.x - x, pt.y - y), circle);
          accum_pixel(houghImg, rect, CV::cvPoint(pt.x + y, pt.y + x), circle);
          accum_pixel(houghImg, rect, CV::cvPoint(pt.x - y, pt.y + x), circle);
          accum_pixel(houghImg, rect, CV::cvPoint(pt.x + y, pt.y - x), circle);
          accum_pixel(houghImg, rect, CV::cvPoint(pt.x - y, pt.y - x), circle);
        }

      }

      // save circle estimation (sorted descending)
      std::vector<HoughCircle>::iterator iter = savedCircles.begin();
      while(iter != savedCircles.end() && iter->intensity > circle.intensity)
      {
        ++iter;
      }
      savedCircles.insert(iter, circle);
    }
    CV::cvReleaseImage(&houghImg);

    // Average the best x procent of the circles weighted by intensity
    Real x = 0;
    Real y = 0;
    r = 0;
    Real sum = 0;
    for(size_t i=0; i<savedCircles.size(); ++i)
    {
      if (savedCircles[i].intensity < 0.8 * savedCircles[0].intensity)
        break;
      x+=savedCircles[i].intensity * savedCircles[i].center.x;
      y+=savedCircles[i].intensity * savedCircles[i].center.y;
      r+=savedCircles[i].intensity * savedCircles[i].radius;
      sum+=savedCircles[i].intensity;
    }
    r/=sum;
    
    if (r >= minRadius && r <= maxRadius)
    {
      blob.center.x = (x/sum + rect.x);
      blob.center.y = (y/sum + rect.y);
      blob.size.width = 2*r;
      blob.size.height = 2*r;
      blob.angle =  0.0;
      found = true;
    }

    // Free memory.          
    free(PointArray);

		contour_ptr = contour_ptr->h_next;
  }

  // Free the storage memory.
	if( mContour_storage != NULL ) 
  { 
    cvReleaseMemStorage(&mContour_storage); 
  }

  // Release the image copy
  CV::cvReleaseImage(&imgCopy);

  return found;

}

bool D6Mote::fitEllipse(CV::IplImage *img, int minRadius, int maxRadius, CV::CvBox2D32f &blob)
{
  CV::CvPoint* PointArray;
  CV::CvPoint2D32f* PointArray2D32f;
  mContour_storage = CV::cvCreateMemStorage();

  CV::CvSeq* contour_list = NULL;

  // cvFindContours corrupts the image. Use a copy instead
  CV::CvSize size = {img->width, img->height};
  CV::IplImage *imgCopy = CV::cvCreateImage( size, img->depth, img->nChannels);
  CV::CvRect roi = CV::cvGetImageROI(img);
  CV::cvSetImageROI(imgCopy, roi);
  CV::cvCopy(img, imgCopy);

	CV::cvFindContours( imgCopy, mContour_storage, &contour_list, sizeof(CV::CvContour), CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
	CV::CvSeq* contour_ptr = contour_list;

  bool found = false;

  // save the contour with largest area
	while( (contour_ptr != NULL) ) 
  {

    float area = fabs( CV::cvContourArea(contour_ptr, CV::CV_WHOLE_SEQ) );
    int count = contour_ptr->total; // This is number point in contour

      // Number point must be more than or equal to 6 (for cvFitEllipse_32f).        
    if(count >= 6 ) 
    {

      // ellips fitting to find blob center.

      // Alloc memory for contour point set.    
      PointArray = (CV::CvPoint*)malloc( count*sizeof(CV::CvPoint) );
      PointArray2D32f= (CV::CvPoint2D32f*)malloc( count*sizeof(CV::CvPoint2D32f) );
      
      // Get contour point set.
      CV::cvCvtSeqToArray(contour_ptr, PointArray, CV::CV_WHOLE_SEQ);
      
      for(int i=0; i<count; i++)
      {
          PointArray2D32f[i].x = (float)PointArray[i].x;
          PointArray2D32f[i].y = (float)PointArray[i].y;
      }
      
      // Fits ellipse to current contour.
      CV::CvBox2D32f box;
      CV::cvFitEllipse(PointArray2D32f, count, &box);

      // Check if blob is in field of view and is roughly cirlular. Pick the largest.
      Real posx = box.center.x;
      Real posy = box.center.y;
      mInFoV = (posx + box.size.height*0.5) < mWidth && (posx - box.size.height*0.5) > 0.0
                  && (posy + box.size.width*0.5) < mHeight && (posy - box.size.width*0.5) > 0.0;
      Real radius = 0.5*MIN(box.size.width, box.size.height);
      if (radius>=minRadius && radius<=maxRadius && Math::Abs(box.size.width/box.size.height - 1.0)<0.25) 
      {
        blob.center.x =  posx;
        blob.center.y =  posy;
        blob.size.width = box.size.width;
        blob.size.height = box.size.height;
        blob.angle =  0.0;
        found = true;
      }

      // Free memory.          
      free(PointArray);
      free(PointArray2D32f);

    } // area check
		contour_ptr = contour_ptr->h_next;
  }

  // Free the storage memory.
	if( mContour_storage != NULL ) 
  { 
    cvReleaseMemStorage(&mContour_storage); 
  }

  // Release the image copy
  CV::cvReleaseImage(&imgCopy);

  return found;
}

void D6Mote::setTrackColor(CV::IplImage * img, int x, int y, int size)
{
  //TODO maybe some histogram function
  //CV::CvRect oldRoi = CV::cvGetImageROI(mColorImage);
  //// Set the image roi to the color sampling box
  //CV::CvRect colorRoi;
  //colorRoi.x = x;
  //colorRoi.y = y;
  //colorRoi.width = colorRoi.height = size;
  //CV::cvSetImageROI(mColorImage, colorRoi);
  // HISTOGRAM STUFF
  //CV::cvSetImageROI(mColorImage, oldRoi);
  
  // TODO assert color model is BGR or make more compatible
  CV::cvCvtColor(img, img, CV_BGR2HSV);

  int val1 = 0, val2 = 0, val3 = 0;

  // add up all color values in the box
  for (int i = 0; i<size; ++i)
  {
    for (int j = 0; j<size; ++j)
    {
      CV::CvPoint pt = {x + i,y + j};
      uchar* temp_ptr = &((uchar*)(img->imageData + img->widthStep*pt.y))[pt.x*3];
      val1 += temp_ptr[0];
      val2 += temp_ptr[1];
      val3 += temp_ptr[2];
    }
  }

  // Take the mean color values
  mTrack_color[0] = val1/(size*size);
  mTrack_color[1] = val2/(size*size);
  mTrack_color[2] = val3/(size*size);
  mHueMax = mTrack_color[0] + mRange1;
  mHueMin = mTrack_color[0] - mRange1;

  mColorPicked = true;

  CV::cvCvtColor(img, img, CV_HSV2BGR);
}

void D6Mote::updateTrackColor(CV::IplImage* img, CV::CvRect region)
{

  // update color values
  mHueMax = mHueMin = mTrack_color[0];
  for (int i = 0; i<region.width; ++i)
  {
      uchar* temp_ptr = &((uchar*)(img->imageData + img->widthStep*region.y))[(region.x + i)*3];
      if (temp_ptr[0] > mHueMax && Math::Abs(temp_ptr[0]-mTrack_color[0]) < mRange1 ) mHueMax = temp_ptr[0];
      if (temp_ptr[0] < mHueMin && Math::Abs(temp_ptr[0]-mTrack_color[0]) < mRange1 ) mHueMin = temp_ptr[0];

      temp_ptr = &((uchar*)(img->imageData + img->widthStep*(region.y+region.height)))[(region.x + i)*3];
      if (temp_ptr[0] > mHueMax && Math::Abs(temp_ptr[0]-mTrack_color[0]) < mRange1 ) mHueMax = temp_ptr[0];
      if (temp_ptr[0] < mHueMin && Math::Abs(temp_ptr[0]-mTrack_color[0]) < mRange1 ) mHueMin = temp_ptr[0];
  }
  for (int i = 0; i<region.height; ++i)
  {
      uchar* temp_ptr = &((uchar*)(img->imageData + img->widthStep*(region.y + i)))[region.x*3];
      if (temp_ptr[0] > mHueMax && Math::Abs(temp_ptr[0]-mTrack_color[0]) < mRange1 ) mHueMax = temp_ptr[0];
      if (temp_ptr[0] < mHueMin && Math::Abs(temp_ptr[0]-mTrack_color[0]) < mRange1 ) mHueMin = temp_ptr[0];

      temp_ptr = &((uchar*)(img->imageData + img->widthStep*(region.y + i)))[(region.x + region.width)*3];
      if (temp_ptr[0] > mHueMax && Math::Abs(temp_ptr[0]-mTrack_color[0]) < mRange1 ) mHueMax = temp_ptr[0];
      if (temp_ptr[0] < mHueMin && Math::Abs(temp_ptr[0]-mTrack_color[0]) < mRange1 ) mHueMin = temp_ptr[0];
  }

  // TODO not sure about this... PROS: may improve robustness CONS : may cause tracking to get lost 
  // Update tracking color for e.g. changed lighting conditions
  //int newHue = (mHueMax+mHueMin)/2;
  //if (Math::Abs(newHue - mTrack_color[0]) < mRange1 ) mTrack_color[0] = newHue;
}

void D6Mote::accum_pixel(CV::IplImage *img, const CV::CvRect &bounds, const CV::CvPoint &pt, HoughCircle &circle)
{
  /* bounds checking */
  if(pt.x < 0 || pt.x >= bounds.width ||
     pt.y < 0 || pt.y >= bounds.height)
  {
    return;
  }

  uchar* temp_ptr = &((uchar*)(img->imageData + img->widthStep*pt.y))[pt.x];
  temp_ptr[0]++;
  if (temp_ptr[0]>circle.intensity)
  {
    circle.intensity = temp_ptr[0];
    circle.center.x = pt.x;
    circle.center.y = pt.y;
  }
  return;
}
