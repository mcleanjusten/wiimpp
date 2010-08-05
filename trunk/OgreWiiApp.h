/*
-----------------------------------------------------------------------------
Filename:    OgreWiiApp.h
-----------------------------------------------------------------------------

This source file is generated by the Ogre AppWizard.

Check out: http://conglomerate.berlios.de/wiki/doku.php?id=ogrewizards

Based on the Example Framework for OGRE
(Object-oriented Graphics Rendering Engine)

Copyright (c) 2000-2007 The OGRE Team
For the latest info, see http://www.ogre3d.org/

You may use this sample code for anything you like, it is not covered by the
LGPL like the rest of the OGRE engine.
-----------------------------------------------------------------------------
*/
#ifndef __OgreWiiApp_h_
#define __OgreWiiApp_h_


#include <iostream>
#include <Ogre.h>
#include <OIS/OIS.h>
#include <CEGUI/CEGUI.h>
#include <OgreCEGUIRenderer.h>

using namespace Ogre;
using std::map;

class OgreWiiApp
{
public:
  OgreWiiApp(const char* address, 
             const int port, 
             const int width, 
             const int height, 
             const int fps,
             const float diameter )
  :mKeyboard(0), 
  mMouse(0), 
  mInputManager(0), 
  mRenderer(0),
  mSystem(0),
	mFrameListener(0), 
  mSceneMgr(0), 
  mWindow(0),
  mCamera(0),
  mAddress(address),
  mPort(port),
  mWidth(width),
  mHeight(height),
  mFps(fps),
  mDiameter(diameter){};
	~OgreWiiApp();
	virtual void go();

private:
  Root *mRoot;
//	World *mWorld;
//	Scene *mScene;
  OIS::Keyboard *mKeyboard;
  OIS::Mouse *mMouse;
  map<String, OIS::JoyStick *> mJoys;
  OIS::InputManager *mInputManager;
  CEGUI::OgreCEGUIRenderer *mRenderer;
  CEGUI::System *mSystem;
  FrameListener *mFrameListener;
  SceneManager *mSceneMgr;
  RenderWindow *mWindow;
  Camera *mCamera;

  const char* mAddress;
  const int mPort;
  const int mWidth;
  const int mHeight;
  const int mFps;
  const float mDiameter;

  void createRoot(void);    
  void defineResources(void);
  void setupRenderSystem(void);
  void createRenderWindow(void);
  void initializeResourceGroups(void);
  void setupSceneMgr(void);
  void setupInputSystem(void);
  void setupCEGUI(void);
  void OgreWiiApp::setupControl();
  void startRenderLoop(void);
	void createFrameListener(void);

};

#endif // #ifndef __OgreWiiApp_h_