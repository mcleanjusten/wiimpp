
#include "OgreWiiListener.h"
#include "osc/OscOutboundPacketStream.h"
#include <iostream>

#define OUTPUT_BUFFER_SIZE 1024

/* TODO LIST:
- Camera Calibration.

NOTES:
Coordinate system:
Uses a right-handed system with origin in camera, y-up and z-axis pointing out from the camera.

*/

bool useOsc = true;
bool usePS3 = true;
bool useWii = true;
bool dummyWii = false; // Used for dev purposes when you dont have your wiimotes but want to perform colortracking.
bool useVideo = false;
const char* videoName = "colortracking.avi";

void on_state_change (wiimote &remote, state_change_flags changed)
{

  /**
	// extension was just connected:
	// a MotionPlus was detected
	if(changed & MOTIONPLUS_DETECTED)
		{
		// enable it if there isn't a normal extension plugged into it
		// (MotionPlus devices don't report like normal extensions until
		//  enabled - and then, other extensions attached to it will no longer be
		//  reported (so disable it when you want to access to those again).
		if(remote.ExtensionType == wiimote_state::NONE) {
			bool res = remote.EnableMotionPlus();
			_ASSERT(res);
			}
		}
	else if(changed & MOTIONPLUS_EXTENSION_CONNECTED)
		{
		// an extension is connected to the MotionPlus.  We can't read it if the
		//  MotionPlus is currently enabled, so disable it:
		if(remote.MotionPlusEnabled())
			remote.DisableMotionPlus();
		}
	else if(changed & MOTIONPLUS_EXTENSION_DISCONNECTED)
		{
		// the extension plugged into the MotionPlus was removed, so enable
		//  the MotionPlus data again:
		if(remote.MotionPlusConnected())
			remote.EnableMotionPlus();
		}
	else 
  */
  if(changed & EXTENSION_CONNECTED)
	{
		// switch to a report mode that includes the extension data (we will
		//  loose the IR dot sizes)
		// note: there is no need to set report types for a Balance Board.
		if(!remote.IsBalanceBoard())
			remote.SetReportType(wiimote::IN_BUTTONS_ACCEL_IR_EXT);
	}
	// extension was just disconnected:
	else if(changed & EXTENSION_DISCONNECTED)
	{
		// use a non-extension report mode (this gives us back the IR dot sizes)
		remote.SetReportType(wiimote::IN_BUTTONS_ACCEL_IR);
	}
  else if(changed & MOTIONPLUS_SPEED_CHANGED)
	{
    D6Mote *d6 = static_cast<D6Mote*>(&remote);
    d6->processWiiData();
	}
}

OgreWiiListener::OgreWiiListener(RenderWindow *window, 
                                 SceneManager *sceneMgr, 
                                 Camera* cam, 
                                 OIS::Keyboard* keyboard, 
                                 OIS::Mouse* mouse,
                                 const char* address, 
                                 const int port, 
                                 const int width, 
                                 const int height, 
                                 const int fps,
                                 const float diameter)
:
mWindow(window), 
mSceneMgr(sceneMgr), 
mCamera(cam), 
mKeyboard(keyboard), 
mMouse(mouse),
mWidth(width), 
mHeight(height), 
mRange1(20),
mBlur(3),
mAmp(128),
mMinRadius(5),
mTransmitSocket(0), 
mTexture(0),
mStatsOn(false)
{
	CEGUI::WindowManager *wmgr = CEGUI::WindowManager::getSingletonPtr();

	mTextBoxInfo = wmgr->getWindow((CEGUI::utf8*)"Vp/TextBoxInfo");

  CEGUI::Window *firstPersonView = wmgr->getWindow((CEGUI::utf8*)"Vp/FirstPersonView");
  firstPersonView->subscribeEvent(CEGUI::Checkbox::EventCheckStateChanged,
  	CEGUI::Event::Subscriber(&OgreWiiListener::toggleFirstPersonView, this));

  CEGUI::Scrollbar *scroll = static_cast<CEGUI::Scrollbar*>(wmgr->getWindow("Vp/Scrollbar1"));
	scroll->setScrollPosition(mRange1/256.0f);
	scroll->subscribeEvent(CEGUI::Scrollbar::EventScrollPositionChanged, 
			CEGUI::Event::Subscriber(&OgreWiiListener::filterParams1Changed, this));

  scroll = static_cast<CEGUI::Scrollbar*>(wmgr->getWindow("Vp/Scrollbar2"));
	scroll->setScrollPosition(mBlur/100.0f);
	scroll->subscribeEvent(CEGUI::Scrollbar::EventScrollPositionChanged, 
			CEGUI::Event::Subscriber(&OgreWiiListener::filterParams2Changed, this));

  scroll = static_cast<CEGUI::Scrollbar*>(wmgr->getWindow("Vp/Scrollbar3"));
	scroll->setScrollPosition(mMinRadius/(float)mHeight);
	scroll->subscribeEvent(CEGUI::Scrollbar::EventScrollPositionChanged, 
			CEGUI::Event::Subscriber(&OgreWiiListener::filterParams3Changed, this));

  CEGUI::Window *reload = wmgr->getWindow((CEGUI::utf8*)"Vp/Reconnect");
  reload->subscribeEvent(CEGUI::PushButton::EventClicked,
	CEGUI::Event::Subscriber(&OgreWiiListener::connectOsc, this));

  CEGUI::Window *snap = wmgr->getWindow((CEGUI::utf8*)"Vp/SnapButton");
  snap->subscribeEvent(CEGUI::PushButton::EventClicked,
	CEGUI::Event::Subscriber(&OgreWiiListener::snapshot, this));

  CEGUI::Window *quit = wmgr->getWindow((CEGUI::utf8*)"Vp/QuitButton");
  quit->subscribeEvent(CEGUI::PushButton::EventClicked,
	CEGUI::Event::Subscriber(&OgreWiiListener::quit, this));

  // continue rendering
	mContinue = true;

	mMouse->setEventCallback(this);
	mKeyboard->setEventCallback(this);

  windowResized();
    
    try
    {
      if (usePS3)
      {
        mCam = new WppPS3();
      }
      else {
        mCam = new WppCapture(useVideo, videoName);
      }

      mCam->start(width,height,fps);
      LogManager::getSingleton().logMessage("camera initialized");
    }
    catch (Exception ex)
    {
        String msg("Failed to initialize camera. ");
        LogManager::getSingleton().logMessage(msg + ex.getFullDescription());
        throw Exception(0, msg + ex.getFullDescription(), "OgreWiiListener::OgreWiiListener");
    }

    if (dummyWii)
    {
      // Create a dummy to use for position tracking
      D6Mote *dummy = new D6Mote(mWidth, mHeight, diameter);
      mD6Wiimotes.push_back(dummy);
      SceneNode * node = mSceneMgr->getRootSceneNode()->createChildSceneNode("WiiNode" + dummy->getID(), Vector3(0,0,0));
      Entity *ent = mSceneMgr->createEntity("Wii" + dummy->getID(), "wand.mesh");
      node->attachObject(ent);
    } 
    else 
    {
      initializeWiimotes(width,height,diameter);
    }

    if (useOsc) 
    {
      initializeOsc(address, port);
    }

    mWidth = mCam->getCamWidth();
    mHeight = mCam->getCamHeight();
    mSource = CV::cvCreateImage( CV::cvSize(mWidth,mHeight), 8, mCam->getNbChannels() );
    mDisplayImage = CV::cvCreateImage( CV::cvSize(160,120), 8, mCam->getNbChannels() );

    mTextBoxInfo->setText(StringConverter::toString(width) + "x" + StringConverter::toString(height) + "@" + StringConverter::toString(fps));

    // Create video texture for background
    mTexture = TextureManager::getSingleton().createManual(
    "VideoTexture", // name
    ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
    TEX_TYPE_2D,      // type
    mDisplayImage->width, mDisplayImage->height,         // width & height
    0,                // number of mipmaps
    PF_B8G8R8,     // pixel format
    TU_DEFAULT);      // usage; should be TU_DYNAMIC_WRITE_ONLY_DISCARDABLE for
                      // textures updated very often (e.g. each frame)

    // Create a material using the texture
    MaterialPtr mat = MaterialManager::getSingleton().create(
    "VideoMaterial", // name
    ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);

    mat->getTechnique(0)->getPass(0)->createTextureUnitState("VideoTexture");
    mat->getTechnique(0)->getPass(0)->setSceneBlending(SBT_TRANSPARENT_ALPHA);

    Entity *bgEnt = mSceneMgr->createEntity("VideoBG", "plane_z_1m.mesh");
    bgEnt->setMaterial(mat);
    bgEnt->setRenderQueueGroup(RENDER_QUEUE_BACKGROUND);
    SceneNode* bgNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("Background", Vector3(0,0,11), Quaternion(Degree(180), Vector3::UNIT_Y));
    bgNode->setScale(12,9,0);
    bgNode->attachObject(bgEnt);

    setFirstPersonView(((CEGUI::Checkbox*)firstPersonView)->isSelected());
}

OgreWiiListener::~OgreWiiListener()
{
  if (mCam)
    delete mCam;

  if (mTransmitSocket)
    delete mTransmitSocket;

  for (size_t i = 0; i < mD6Wiimotes.size(); ++i)
    delete mD6Wiimotes[i];

}

void OgreWiiListener::initializeWiimotes(const int width, const int height, const float diameter)
{
    try
    {
      //Connect up to 4 Wiimotes
      for (int i = 0; i<4; i++)
      {
        bool connected = false;

        D6Mote *d6Mote = new D6Mote(width, height, diameter);
        d6Mote->ChangedCallback		= on_state_change;
        //  notify us only when something related to the extension changes
        d6Mote->CallbackTriggerFlags = EXTENSION_CHANGED;
        
        for (int nTries = 0; nTries < 4; ++nTries) 
        {
          if (d6Mote->Connect(wiimote::FIRST_AVAILABLE))
          {
            d6Mote->SetLEDs(0x0f);
            d6Mote->SetReportType(wiimote::IN_BUTTONS_ACCEL_IR);
            mD6Wiimotes.push_back(d6Mote);

            // Scene
            SceneNode * node = mSceneMgr->getRootSceneNode()->createChildSceneNode("WiiNode" + d6Mote->getID(), Vector3(0,0,0));
            Entity *ent = mSceneMgr->createEntity("Wii" + d6Mote->getID(), "wand.mesh");
            node->attachObject(ent);
            connected = true;
            break;
          }
          else
          {
            Sleep(500);
          }
	      }
        // None more found. cleanup and exit
        if (!connected)
        {
          delete d6Mote;
          break;
        }
      }
    }
    catch (Exception ex)
    {
        String msg("Failed to initialize Wiimote. ");
        LogManager::getSingleton().logMessage(msg + ex.getFullDescription());
        throw Exception(0, msg + ex.getFullDescription(), "OgreWiiListener::OgreWiiListener");
    }
}

void OgreWiiListener::initializeOsc(const char* address, const int port)
{
    try
    {
      // Osc
      LogManager::getSingleton().logMessage("Opening UDP Socket");
      mTransmitSocket = new UdpTransmitSocket( IpEndpointName( address, port ) );
      LogManager::getSingleton().logMessage("Done");
      if (mTransmitSocket == 0)
      {
          String msg("Could not init UDP!");
          LogManager::getSingleton().logMessage(msg);
          throw Exception(0, msg, "OgreWiiListener::OgreWiiListener");
      }
    }
    catch (Exception ex)
    {
        String msg("Failed to initialize OSC. ");
        LogManager::getSingleton().logMessage(msg + ex.getFullDescription());
        throw Exception(0, msg + ex.getFullDescription(), "OgreWiiListener::OgreWiiListener");
    }
}


bool OgreWiiListener::connectOsc(const CEGUI::EventArgs &e)
{
	CEGUI::WindowManager *wmgr = CEGUI::WindowManager::getSingletonPtr();
  try 
  {
    const char* address = wmgr->getWindow((CEGUI::utf8*)"Vp/OscAddress")->getText().c_str();
    int port = StringConverter::parseInt(wmgr->getWindow((CEGUI::utf8*)"Vp/OscPort")->getText().c_str());
    // Re-init osc
    if (mTransmitSocket)
      delete mTransmitSocket;
    initializeOsc(address, port);
  }
  catch (Exception e)
  {
    // To nothing, 
  }
  return true;
}

bool OgreWiiListener::toggleFirstPersonView(const CEGUI::EventArgs &e)
{
	CEGUI::WindowManager *wmgr = CEGUI::WindowManager::getSingletonPtr();
  setFirstPersonView(((CEGUI::Checkbox*)wmgr->getWindow((CEGUI::utf8*)"Vp/FirstPersonView"))->isSelected());
  return true;
}

bool OgreWiiListener::filterParams1Changed(const CEGUI::EventArgs &e)
{
  CEGUI::Scrollbar* scroll = static_cast<CEGUI::Scrollbar*>(CEGUI::WindowManager::getSingletonPtr()->getWindow("Vp/Scrollbar1"));
  mRange1 = 256 * scroll->getScrollPosition();
  return true;
}

bool OgreWiiListener::filterParams2Changed(const CEGUI::EventArgs &e)
{
  CEGUI::Scrollbar* scroll = static_cast<CEGUI::Scrollbar*>(CEGUI::WindowManager::getSingletonPtr()->getWindow("Vp/Scrollbar2"));
  mBlur = 100 * scroll->getScrollPosition();
  return true;
}

bool OgreWiiListener::filterParams3Changed(const CEGUI::EventArgs &e)
{
  CEGUI::Scrollbar* scroll = static_cast<CEGUI::Scrollbar*>(CEGUI::WindowManager::getSingletonPtr()->getWindow("Vp/Scrollbar3"));
  mMinRadius = mHeight * scroll->getScrollPosition();
  return true;
}

bool OgreWiiListener::snapshot(const CEGUI::EventArgs &e)
{
  //CV::cvSaveImage("snapshot", mSource);
  Image ogreImage;
  Ogre::PixelFormat pf = Ogre::PF_R8G8B8;
  if (mCam->getNbChannels()==1) pf = Ogre::PF_L8;
  else if (mCam->getNbChannels()==3) pf = Ogre::PF_R8G8B8;
  ogreImage.loadDynamicImage(mCam->getPixels(), 
    mCam->getCamWidth(), mCam->getCamHeight(), pf );
  ogreImage.save("snapshot.bmp");
  return true;
}

void OgreWiiListener::setFirstPersonView(bool isFirstPersonView)
{
  if (isFirstPersonView)
  {
    mCamera->setPosition(0,0,2);
    mCamera->setOrientation(Quaternion::IDENTITY);
  } else {
    Quaternion y180 = Quaternion(Degree(180), Vector3::UNIT_Y);
    mCamera->setPosition(0,0,0);
    mCamera->setOrientation(y180);
  }

  mIsFirstPersonView = isFirstPersonView;
}

bool OgreWiiListener::frameStarted(const FrameEvent& evt)
{

	if(mMouse)
		mMouse->capture();
	if(mKeyboard) 
		mKeyboard->capture();

  if(mStatsOn) 
  {
    updateStats();
  }

  Real t0 = 0.000001*Ogre::Root::getSingleton().getTimer()->getMicroseconds();

  //if (mKeyboard->isKeyDown(OIS::KC_P))
  //{
  //  return true;
  //}

  if (mCam)
  {
    if(!mCam->isFrameNew())
    {
      // Continue
      return true;
    }

    // Copy the ps3 pixels to cv frame
    memcpy(mSource->imageData, mCam->getPixels(), mSource->imageSize);
  }
  else
  {
    return true;
  }

  // Track Wiimotes
  if (useWii || dummyWii){
    trackWiimotes();
  }
  
  // Draw center of image (for color picking)
  CV::cvRectangle( mSource, CV::cvPoint(mWidth/2 - 2,mHeight/2 - 2), CV::cvPoint(mWidth/2 + 2,mHeight/2 + 2), 
  CV::cvScalar(255,255,255), 1, 8, 0 );
//  CV::cvShowImage( "SphereTrack", showImg);

  if (!mIsFirstPersonView) {

    // Show image in ogre window.
    if (mSource->width == mDisplayImage->width && mSource->height == mDisplayImage->height)
    {
      CV::cvCopy(mSource, mDisplayImage);
    }
    else
    {
      CV::cvResize(mSource, mDisplayImage);
    }
              /* Get the pixel buffer
              HardwarePixelBufferSharedPtr pixelBuffer = mTexture->getBuffer();

              // Lock the pixel buffer and get a pixel box
              pixelBuffer->lock(HardwareBuffer::HBL_NORMAL);
              const PixelBox& pixelBox = pixelBuffer->getCurrentLock();

              uint8* pDest = static_cast<uint8*>(pixelBox.data);

              // Fill in some pixel data.
              for (int i = 0; i < mDisplayImage->imageSize; i++)
              {
                *pDest++ = mDisplayImage->imageData[i];
              }

              // Unlock the pixel buffer
              pixelBuffer->unlock();
    */
    drawWiimotes(mDisplayImage);

    Image ogreImage;
    Ogre::PixelFormat pf = Ogre::PF_R8G8B8;
    if (mCam->getNbChannels()==1) pf = Ogre::PF_L8;
    else if (mCam->getNbChannels()==3) pf = Ogre::PF_R8G8B8;
    ogreImage.loadDynamicImage((unsigned char*) mDisplayImage->imageData, 
      mDisplayImage->width, mDisplayImage->height, pf );
	  Box b( 0, 0, 0, ogreImage.getWidth(), ogreImage.getHeight(), 1);
	  mTexture->getBuffer()->blitFromMemory( ogreImage.getPixelBox(), b );
    
  }
    
  return mContinue;
}

bool OgreWiiListener::quit(const CEGUI::EventArgs &e)
{
    mContinue = false;
    return true;
}

CEGUI::MouseButton OgreWiiListener::convertButton(OIS::MouseButtonID buttonID)
{
    switch (buttonID)
    {
    case OIS::MB_Left:
        return CEGUI::LeftButton;

    case OIS::MB_Right:
        return CEGUI::RightButton;

    case OIS::MB_Middle:
        return CEGUI::MiddleButton;

    default:
        return CEGUI::LeftButton;
    }
}

// MouseListener
bool OgreWiiListener::mouseMoved(const OIS::MouseEvent &e)
{
	CEGUI::System::getSingleton().injectMouseMove(e.state.X.rel, e.state.Y.rel);
	return true;
}

bool OgreWiiListener::mousePressed(const OIS::MouseEvent &e, OIS::MouseButtonID id)
{
	CEGUI::System::getSingleton().injectMouseButtonDown(convertButton(id));

  if (!useWii && e.state.buttonDown(OIS::MB_Left))
	{
    // TEMP for dev purpose. set color to track for dummy wiimote
    for (size_t i = 0; i < mD6Wiimotes.size(); ++i)
    {
      mD6Wiimotes[i]->setTrackColor(mSource,e.state.X.abs,e.state.Y.abs);
    }
	} 

  return true;
}

bool OgreWiiListener::mouseReleased(const OIS::MouseEvent &e, OIS::MouseButtonID id) 
{
    CEGUI::System::getSingleton().injectMouseButtonUp(convertButton(id));

	return true;
}

// KeyListener
bool OgreWiiListener::keyPressed(const OIS::KeyEvent &e)
{
	CEGUI::System *sys = CEGUI::System::getSingletonPtr();
    sys->injectKeyDown(e.key);
    sys->injectChar(e.text);

  // Just return if user is editing the reload file edit box.
  CEGUI::WindowManager *wmgr = CEGUI::WindowManager::getSingletonPtr();
  if (((CEGUI::Editbox*)wmgr->getWindow((CEGUI::utf8*)"Vp/OscAddress"))->hasInputFocus())
  {
    return true;
  }

  switch (e.key)
  {
    case OIS::KC_ESCAPE: 
      mContinue = false;
      break;

    case OIS::KC_S:
      mStatsOn = !mStatsOn;
      showDebugOverlay(mStatsOn);
      break;

    case OIS::KC_M:
      mTextBoxInfo->setVisible(!mTextBoxInfo->isVisible());
      break;

  }

  updateText();

	return true;
}


bool OgreWiiListener::keyReleased(const OIS::KeyEvent &e)
{
	CEGUI::System::getSingleton().injectKeyUp(e.key);
    return true;
}


//Adjust mouse clipping area
void OgreWiiListener::windowResized()
{
	unsigned int width, height, depth;
	int left, top;
	mWindow->getMetrics(width, height, depth, left, top);

	const OIS::MouseState &ms = mMouse->getMouseState();
	ms.width = width;
	ms.height = height;
}




void OgreWiiListener::drawWiimotes(CV::IplImage * showImg)
{
  for (size_t i = 0; i < mD6Wiimotes.size(); ++i)
  {
    // Choose image to show
    if (mD6Wiimotes[i]->Button.Up() || mKeyboard->isKeyDown(OIS::KC_UP))
    {
      CV::IplImage *img = mD6Wiimotes[i]->getProcessedImage();
      CV::IplImage *tmp = CV::cvCreateImage(CV::cvGetSize(showImg), img->depth, img->nChannels);
      CV::CvRect roi= CV::cvGetImageROI(img);
      CV::cvResetImageROI(img);
      CV::cvResize(img, tmp);
      CV::cvSetImageROI(img, roi);
      CV::cvCvtColor(tmp, showImg, CV_HSV2BGR);
      CV::cvReleaseImage(&tmp);
    }
    else if (mD6Wiimotes[i]->Button.Left() || mKeyboard->isKeyDown(OIS::KC_LEFT))
    {
      CV::IplImage *img = mD6Wiimotes[i]->getFgMask(); 
      CV::IplImage *tmp = CV::cvCreateImage(CV::cvGetSize(showImg), img->depth, img->nChannels);
      CV::CvRect roi= CV::cvGetImageROI(img);
      CV::cvResetImageROI(img);
      CV::cvResize(img, tmp);
      CV::cvSetImageROI(img, roi);
      CV::cvCvtColor(tmp, showImg, CV_GRAY2BGR);
      CV::cvReleaseImage(&tmp);
    }
    else if (mD6Wiimotes[i]->Button.Right() || mKeyboard->isKeyDown(OIS::KC_RIGHT))
    {
      CV::IplImage *img = mD6Wiimotes[i]->getColorMask();  
      CV::IplImage *tmp = CV::cvCreateImage(CV::cvGetSize(showImg), img->depth, img->nChannels);
      CV::CvRect roi= CV::cvGetImageROI(img);
      CV::cvResetImageROI(img);
      CV::cvResize(img, tmp);
      CV::cvSetImageROI(img, roi);
      CV::cvCvtColor(tmp, showImg, CV_GRAY2BGR);
      CV::cvReleaseImage(&tmp);
    }
    else if (mD6Wiimotes[i]->Button.Down() || mKeyboard->isKeyDown(OIS::KC_DOWN))
    {
      CV::IplImage *img = mD6Wiimotes[i]->getCalibrationImage(); 
      CV::IplImage *tmp = CV::cvCreateImage(CV::cvGetSize(showImg), img->depth, img->nChannels);
      CV::CvRect roi= CV::cvGetImageROI(img);
      CV::cvResetImageROI(img);
      CV::cvResize(img, tmp);
      CV::cvSetImageROI(img, roi);
      CV::cvCvtColor(tmp, showImg, CV_GRAY2BGR);
      CV::cvReleaseImage(&tmp);
    }

    // Draw ROI
    CV::CvRect roi = mD6Wiimotes[i]->getROI();
    CV::cvRectangle( showImg, CV::cvPoint(roi.x,roi.y), CV::cvPoint(roi.x+roi.width,roi.y+roi.height), 
         CV::cvScalar(50*i,255-50*i,50*i), 1, 8, 0 );

    // Draw blob ellipse
    // Convert ellipse data from float to integer representation.
    CV::CvBox2D blob = mD6Wiimotes[i]->getContour();
    CV::CvPoint center = {CV::cvRound(blob.center.x), CV::cvRound(blob.center.y)};
    CV::CvSize size = {CV::cvRound(blob.size.width*0.5), CV::cvRound(blob.size.height*0.5)};        
    
    // Draw ellipse.
    CV::cvEllipse(showImg, center, size,
      -blob.angle, 0, 360, CV::cvScalar(255-50*i,50*i,50*i), 1, CV_AA, 0);

    //CV::cvRectangle( mSource, CV::cvPoint(blob.x,blob.y), CV::cvPoint(blob.x+blob.width,blob.y+blob.height), 
    //     CV::cvScalar(255-50*i,50*i,50*i), 1, 8, 0 );
  }
}

void OgreWiiListener::trackWiimotes()
{

  for (size_t i = 0; i < mD6Wiimotes.size(); ++i)
  {
    mD6Wiimotes[i]->setFilterParams(mRange1, mBlur, mAmp, mMinRadius);
    bool blobFound = mD6Wiimotes[i]->processImageData(mSource);
    
    if (mD6Wiimotes[i]->Button.Two()) 
    {
      mD6Wiimotes[i]->setTrackColor(mSource, mSource->width/2, mSource->height/2, 1);
    }
  }

/*
  Real t2 = 0.000001*Ogre::Root::getSingleton().getTimer()->getMicroseconds();
  OverlayElement* imgInfo = OverlayManager::getSingleton().getOverlayElement("AR/ModelViewMatrix/Row3"); 
  String fps="Calc time: " + StringConverter::toString(t2-t1);
  imgInfo->setCaption(fps);
  */

  for (size_t i = 0; i < mD6Wiimotes.size(); ++i)
  {
    String oeName("AR/ModelViewMatrix/Row");
    OverlayElement* oe = OverlayManager::getSingleton().getOverlayElement(oeName + StringConverter::toString(i+1)); 
    Vector3 pos = mD6Wiimotes[i]->getPosition();
    String txt("position: x=");
    txt.append(StringConverter::toString(pos.x,4));
    txt.append(" y=");
    txt.append(StringConverter::toString(pos.y,4));
    txt.append(" z=");
    txt.append(StringConverter::toString(pos.z,4));
    //if (!mD6Wiimotes[i]->isInitialized())
    //  txt = "WAITING TO INITIALIZE";
    //if (mD6Wiimotes[i]->isCalibrating())
    //  txt = "CALIBRATING";
    oe->setCaption(txt);

    /* TODO temp
    oeName = "AR/ModelViewMatrix/Row3";
    oe = OverlayManager::getSingleton().getOverlayElement(oeName); 
    Vector3 rates = mD6Wiimotes[i]->getAngleRates();
    txt = "rates: yaw=";
    txt.append(StringConverter::toString(rates.x,4));
    txt.append(" pitch=");
    txt.append(StringConverter::toString(rates.y,4));
    txt.append(" roll=");
    txt.append(StringConverter::toString(rates.z,4));
    oe->setCaption(txt);
    */

    SceneNode *node = mSceneMgr->getSceneNode("WiiNode" + mD6Wiimotes[i]->getID());
    node->setPosition(mD6Wiimotes[i]->getPosition());
    node->setOrientation(node->getInitialOrientation()*mD6Wiimotes[i]->getOrientation());
    if (useOsc)
    {
      sendWii(mD6Wiimotes[i]);
    }
  }
}


void OgreWiiListener::sendWii(D6Mote *d6Mote)
{
  Ogre::String text("/Wii/");  
  text.append(Ogre::StringConverter::toString(d6Mote->getID()));

  char buffer[OUTPUT_BUFFER_SIZE];
  osc::OutboundPacketStream ps( buffer, OUTPUT_BUFFER_SIZE );

  Vector3 pos = d6Mote->getPosition();
  Quaternion quat= d6Mote->getOrientation();

  ps << osc::BeginBundleImmediate
    << osc::BeginMessage( text.c_str() )
    << (float)pos.x
    << (float)pos.y 
    << (float)pos.z 
    << (float)quat.x 
    << (float)quat.y 
    << (float)quat.z 
    << (float)quat.w 
    << (int)d6Mote->Button.A() 
    << (int)d6Mote->Button.B()
    << (int)d6Mote->Button.One() 
    << (int)d6Mote->Button.Two() 
    << (int)d6Mote->Button.Left()
    << (int)d6Mote->Button.Right() 
    << (int)d6Mote->Button.Up() 
    << (int)d6Mote->Button.Down() 
    << osc::EndMessage
    << osc::EndBundle;

  mTransmitSocket->Send( ps.Data(), ps.Size() );
}

void OgreWiiListener::updateText(void)
{
  //mTextBoxInfo->setText();
}

void OgreWiiListener::updateStats(void)
{
    static String currFps = "Current FPS: ";
    static String avgFps = "Average FPS: ";
    static String bestFps = "Best FPS: ";
    static String worstFps = "Worst FPS: ";
    static String tris = "Triangle Count: ";

    // update stats when necessary
    OverlayElement* guiAvg = OverlayManager::getSingleton().getOverlayElement("Core/AverageFps");
    OverlayElement* guiCurr = OverlayManager::getSingleton().getOverlayElement("Core/CurrFps");
    OverlayElement* guiBest = OverlayManager::getSingleton().getOverlayElement("Core/BestFps");
    OverlayElement* guiWorst = OverlayManager::getSingleton().getOverlayElement("Core/WorstFps");
    
    guiAvg->setCaption(avgFps + StringConverter::toString(mWindow->getAverageFPS()));
    guiCurr->setCaption(currFps + StringConverter::toString(mWindow->getLastFPS()));
    guiBest->setCaption(bestFps + StringConverter::toString(mWindow->getBestFPS())
        +" "+StringConverter::toString(mWindow->getBestFrameTime())+" ms");
    guiWorst->setCaption(worstFps + StringConverter::toString(mWindow->getWorstFPS())
        +" "+StringConverter::toString(mWindow->getWorstFrameTime())+" ms");
        
    OverlayElement* guiTris = OverlayManager::getSingleton().getOverlayElement("Core/NumTris");
    guiTris->setCaption(tris + StringConverter::toString(mWindow->getTriangleCount()));

//    OverlayElement* guiDbg = OverlayManager::getSingleton().getOverlayElement("Core/DebugText");
//    guiDbg->setCaption(mDebugText);
}

void OgreWiiListener::showDebugOverlay(bool show)
{   
    Overlay* o = OverlayManager::getSingleton().getByName("Core/DebugOverlay");
    if (!o)
        OGRE_EXCEPT( Exception::ERR_ITEM_NOT_FOUND, "Could not find overlay Core/DebugOverlay",
            "showDebugOverlay" );
    if (show)
        o->show();
    else
        o->hide();
}
