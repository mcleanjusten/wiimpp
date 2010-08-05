/*
*/

#ifndef __OgreWiiListener_H__
#define __OgreWiiListener_H__

#include <Ogre.h>
#include <CEGUI/CEGUI.h>
#include <OIS/OIS.h>

namespace CV
{
#include <cv.h>
#include <highgui.h>
}

#include "ExampleFrameListener.h"
#include "D6Mote.h"
#include "WppCamera.h"
#include <iostream>
#include "ip/UdpSocket.h"


class OgreWiiListener : public FrameListener, public WindowEventListener, public OIS::MouseListener, public OIS::KeyListener
{
public:
OgreWiiListener(RenderWindow *window, 
                SceneManager *sceneMgr, 
                Camera* cam, 
                OIS::Keyboard* keyboard, 
                OIS::Mouse* mouse,
                const char* address = "127.0.0.1", 
                const int port = 5000, 
                const int width = 320, 
                const int height = 240, 
                const int fps = 60,
                const float diameter = 0.05);
  ~OgreWiiListener();

  bool connectOsc(const CEGUI::EventArgs &e);
  bool toggleFirstPersonView(const CEGUI::EventArgs &e);
  bool filterParams1Changed(const CEGUI::EventArgs &e);
  bool filterParams2Changed(const CEGUI::EventArgs &e);
  bool filterParams3Changed(const CEGUI::EventArgs &e);
  bool snapshot(const CEGUI::EventArgs &e);
  bool quit(const CEGUI::EventArgs &e);
  bool frameStarted(const FrameEvent& evt);
  bool mouseMoved(const OIS::MouseEvent &e);
  bool mousePressed(const OIS::MouseEvent &e, OIS::MouseButtonID id);
  bool mouseReleased(const OIS::MouseEvent &e, OIS::MouseButtonID id);
  bool keyPressed(const OIS::KeyEvent &e);
  bool keyReleased(const OIS::KeyEvent &e);
  virtual void windowResized();

private:
	CEGUI::MouseButton convertButton(OIS::MouseButtonID buttonID);
  void setFirstPersonView(bool bVal);
  void initializeWiimotes(const int width, const int height, const float diameter);
  void initializeOsc(const char* address, const int port);
  void trackWiimotes();
  void drawWiimotes(CV::IplImage *showImage);
  void sendWii(D6Mote *d6Mote);
  void updateText(void);
  void updateStats(void);
  void showDebugOverlay(bool show);

  Ogre::Overlay *mOverlay;
  RenderWindow *mWindow;   // The current Scene Manager
  SceneManager *mSceneMgr;

  OIS::Keyboard *mKeyboard;
  OIS::Mouse *mMouse;
  Camera *mCamera;
  CEGUI::Window *mTextBoxInfo; 

	Rectangle2D *mRect;
  TexturePtr mTexture;
  bool mIsFirstPersonView;

  bool mStatsOn;       // Whether to show statistics or not
  bool mContinue;       // Whether to continue rendering or not


  // Imaging
  WppCamera * mCam;
  CV::IplImage* mSource;
  CV::IplImage* mDisplayImage;  
  int mWidth;
  int mHeight;
  int mRange1;
  int mBlur;
  int mAmp;
  int mMinRadius;

  // Wiimote
  std::vector<D6Mote*> mD6Wiimotes;

  // OSC
  UdpTransmitSocket *mTransmitSocket;
  PBYTE mBuffer;

  // Override mouse input, we want no camera movements from mouse
  virtual bool processUnbufferedMouseInput(const FrameEvent& evt){return true;};
};

#endif  // __OgreWiiListener_H__
