
#include "OgreWiiApp.h"
#include "OgreWiiListener.h"
#include <iostream>

using namespace std;

OgreWiiApp::~OgreWiiApp()
{
  if (mFrameListener)
    delete mFrameListener;

	if (mInputManager)
	{
		mInputManager->destroyInputObject(mKeyboard);
		mInputManager->destroyInputObject(mMouse);
    map<String, OIS::JoyStick*>::iterator iter = mJoys.begin();
    map<String, OIS::JoyStick*>::iterator end = mJoys.end();
    while(iter != end)
    {
      delete iter->second;
      ++iter;
    }
    mJoys.clear();
    
		OIS::InputManager::destroyInputSystem(mInputManager);
	}

	delete mSystem;
	delete mRenderer;
  delete mRoot;
}


void OgreWiiApp::go()
{
    createRoot();
    defineResources();
    setupRenderSystem();
    createRenderWindow();
    initializeResourceGroups();
    setupSceneMgr();
    setupInputSystem();
    setupCEGUI();
    createFrameListener();
    startRenderLoop();
}

void OgreWiiApp::createRoot()
{
    mRoot = new Root();
}

void OgreWiiApp::defineResources()
{
    String secName, typeName, archName;
    ConfigFile cf;
    cf.load("resources.cfg");

    ConfigFile::SectionIterator seci = cf.getSectionIterator();
    while (seci.hasMoreElements())
    {
        secName = seci.peekNextKey();
        ConfigFile::SettingsMultiMap *settings = seci.getNext();
        ConfigFile::SettingsMultiMap::iterator i;
        for (i = settings->begin(); i != settings->end(); ++i)
        {
            typeName = i->first;
            archName = i->second;
            ResourceGroupManager::getSingleton().addResourceLocation(archName, typeName, secName);
        }
    }
}

void OgreWiiApp::setupRenderSystem()
{
    if (!mRoot->restoreConfig() && !mRoot->showConfigDialog())
        throw Exception(52, "User canceled the config dialog!", "Application::setupRenderSystem()");

    // Place rendering info here
    //RenderSystem *rs = mRoot->getRenderSystemByName("Direct3D9 Rendering Subsystem");
    //mRoot->setRenderSystem(rs);
    //rs->setConfigOption("Full Screen", "No");
    //rs->setConfigOption("Video Mode", "800 x 600 @ 32-bit colour");
}

void OgreWiiApp::createRenderWindow()
{
  mWindow = mRoot->initialise(true, "WiiM++");
}

void OgreWiiApp::initializeResourceGroups()
{
    TextureManager::getSingleton().setDefaultNumMipmaps(5);
    ResourceGroupManager::getSingleton().initialiseAllResourceGroups();

}

void OgreWiiApp::setupSceneMgr()
{
	mSceneMgr = mRoot->createSceneManager(ST_EXTERIOR_CLOSE, "Default SceneManager");

  // Scene manager and camera setup
  mCamera = mSceneMgr->createCamera("Camera");
	mCamera->setNearClipDistance(0.1);
	mCamera->setFarClipDistance(100.0);
  mWindow->addViewport(mCamera);

    // Key light
  Ogre::Vector3 direction(0.2, -0.2, -0.2);
	direction.normalise();
	Light* mylight = mSceneMgr->createLight("KeyLight");
	mylight->setType(Light::LT_DIRECTIONAL);
	mylight->setDirection(direction);
}


void OgreWiiApp::createFrameListener()
{
  mFrameListener = new OgreWiiListener(mWindow, mSceneMgr, mCamera, mKeyboard, mMouse, mAddress, mPort, mWidth, mHeight, mFps, mDiameter);
  mRoot->addFrameListener(mFrameListener);
}


void OgreWiiApp::setupInputSystem()
{
    size_t windowHnd = 0;
    std::ostringstream windowHndStr;
    OIS::ParamList pl;
    RenderWindow *win = mRoot->getAutoCreatedWindow();

    win->getCustomAttribute("WINDOW", &windowHnd);
    windowHndStr << windowHnd;
    pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));
    mInputManager = OIS::InputManager::createInputSystem(pl);
    int nJoys = mInputManager->numJoySticks();
    try
    {
        mKeyboard = static_cast<OIS::Keyboard*>(mInputManager->createInputObject(OIS::OISKeyboard, true));
        mMouse = static_cast<OIS::Mouse*>(mInputManager->createInputObject(OIS::OISMouse, true));
        for (int i=0; i < nJoys; ++i)
        {
          OIS::JoyStick* joy = static_cast<OIS::JoyStick*>(mInputManager->createInputObject(OIS::OISJoyStick, true));
          mJoys[String(joy->vendor())] = joy;
        }
    }
    catch (const OIS::Exception &e)
    {
        throw new Exception(42, e.eText, "Application::setupInputSystem");
    }

}

void OgreWiiApp::setupCEGUI()
{
    //mWindow = mRoot->getAutoCreatedWindow();
    mRenderer = new CEGUI::OgreCEGUIRenderer(mWindow, Ogre::RENDER_QUEUE_OVERLAY, false, 3000, mSceneMgr);
    mSystem = new CEGUI::System(mRenderer);

	CEGUI::SchemeManager::getSingleton().loadScheme((CEGUI::utf8*)"TaharezLookSkin.scheme");
	mSystem->setDefaultMouseCursor((CEGUI::utf8*)"TaharezLook", (CEGUI::utf8*)"MouseArrow");
    mSystem->setDefaultFont((CEGUI::utf8*)"BlueHighway-12");

	CEGUI::WindowManager *winMgr = CEGUI::WindowManager::getSingletonPtr();
	CEGUI::Window *sheet = winMgr->createWindow("DefaultGUISheet", "Vp/Sheet");

  CEGUI::UVector2 buttonSize(CEGUI::UDim(0.15, 0), CEGUI::UDim(0.05, 0));
  CEGUI::UVector2 checkboxSize(CEGUI::UDim(0.15, 0), CEGUI::UDim(0.05, 0));
  CEGUI::UVector2 comboboxSize(CEGUI::UDim(0.15, 0), CEGUI::UDim(0.15, 0));
  CEGUI::UVector2 infoboxSize(CEGUI::UDim(0.5, 0.0), CEGUI::UDim(0.1, 0.0));
  CEGUI::UVector2 controlboxSize(CEGUI::UDim(0.5, 0.0), CEGUI::UDim(0.5, 0.0));
  CEGUI::UVector2 scrollSize(CEGUI::UDim(0.15, 0), CEGUI::UDim(0.025, 0));
  float dy = 0.06f;
  float y = 0.01f;

	CEGUI::Window *win = winMgr->createWindow("TaharezLook/Editbox", "Vp/OscAddress");
	win->setText(mAddress);
  win->setPosition( CEGUI::UVector2( CEGUI::UDim( 0.01f, 0 ), CEGUI::UDim( y, 0 ) ) );
	win->setSize(buttonSize);
	sheet->addChildWindow(win);

  y+=dy;
	win = winMgr->createWindow("TaharezLook/Editbox", "Vp/OscPort");
  win->setText(StringConverter::toString(mPort));
  win->setPosition( CEGUI::UVector2( CEGUI::UDim( 0.01f, 0 ), CEGUI::UDim( y, 0 ) ) );
	win->setSize(buttonSize);
	sheet->addChildWindow(win);

  y+=dy;
	win = winMgr->createWindow("TaharezLook/Button", "Vp/Reconnect");
	win->setText("Reconnect");
  win->setPosition( CEGUI::UVector2( CEGUI::UDim( 0.01f, 0 ), CEGUI::UDim( y, 0 ) ) );
	win->setSize(buttonSize);
	sheet->addChildWindow(win);

  y+=dy;
	win = winMgr->createWindow("TaharezLook/Checkbox", "Vp/FirstPersonView");
  win->setPosition( CEGUI::UVector2( CEGUI::UDim( 0.01f, 0 ), CEGUI::UDim( y, 0 ) ) );
  win->setText("First Person View");
	win->setSize(checkboxSize);
	sheet->addChildWindow(win);

  y+=dy;
	win = winMgr->createWindow("TaharezLook/HorizontalScrollbar", "Vp/Scrollbar1");
	win->setText("Scrollbar1");
  win->setPosition( CEGUI::UVector2( CEGUI::UDim( 0.01f, 0 ), CEGUI::UDim( y, 0 ) ) );
	win->setSize(scrollSize);
	sheet->addChildWindow(win);

  y+=dy;
	win = winMgr->createWindow("TaharezLook/HorizontalScrollbar", "Vp/Scrollbar2");
	win->setText("Scrollbar1");
  win->setPosition( CEGUI::UVector2( CEGUI::UDim( 0.01f, 0 ), CEGUI::UDim( y, 0 ) ) );
	win->setSize(scrollSize);
	sheet->addChildWindow(win);

  y+=dy;
	win = winMgr->createWindow("TaharezLook/HorizontalScrollbar", "Vp/Scrollbar3");
	win->setText("Scrollbar1");
  win->setPosition( CEGUI::UVector2( CEGUI::UDim( 0.01f, 0 ), CEGUI::UDim( y, 0 ) ) );
	win->setSize(scrollSize);
	sheet->addChildWindow(win);

  // Snapshot button
  y+=dy;
	win = winMgr->createWindow("TaharezLook/Button", "Vp/SnapButton");
	win->setText("Shapshot");
  win->setPosition( CEGUI::UVector2( CEGUI::UDim( 0.01f, 0 ), CEGUI::UDim( y, 0 ) ) );
	win->setSize(buttonSize);
	sheet->addChildWindow(win);

  // Quit button
  y+=dy;
	win = winMgr->createWindow("TaharezLook/Button", "Vp/QuitButton");
	win->setText("Quit");
  win->setPosition( CEGUI::UVector2( CEGUI::UDim( 0.01f, 0 ), CEGUI::UDim( y, 0 ) ) );
	win->setSize(buttonSize);
	sheet->addChildWindow(win);

  // Info txt box
  win = winMgr->createWindow("TaharezLook/StaticText", "Vp/TextBoxInfo");
  win->setPosition( CEGUI::UVector2( CEGUI::UDim( 0.01f, 0 ), CEGUI::UDim( 0.85f, 0 ) ) );
  win->setSize(infoboxSize);
  sheet->addChildWindow(win);

  // Info txt box
  win = winMgr->createWindow("TaharezLook/StaticText", "Vp/TextBoxAR");
  win->setPosition( CEGUI::UVector2( CEGUI::UDim( 0.6f, 0 ), CEGUI::UDim( 0.85f, 0 ) ) );
  win->setSize(infoboxSize);
  win->setVisible(true);
  sheet->addChildWindow(win);

  // Controls txt box
  /*
	win = winMgr->createWindow("TaharezLook/StaticText", "Vp/TextBoxControls");
  win->setPosition( CEGUI::UVector2( CEGUI::UDim( 0.5f, 0 ), CEGUI::UDim( 0.0f, 0 ) ) );
  win->setSize(controlboxSize);
  sheet->addChildWindow(win);
  */

  mSystem->setGUISheet(sheet);


}

void OgreWiiApp::startRenderLoop()
{
  mRoot->startRendering();
}
