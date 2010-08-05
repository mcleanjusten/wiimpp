
#include "OgreWiiApp.h"
#include <iostream>


#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
#define WIN32_LEAN_AND_MEAN
#include "windows.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
int main(int argc, char *argv[])
//INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
#else
int main(int argc, char **argv)
#endif
{
  char* address = "127.0.0.1";
  int port = 5000;
  int width = 320;
  int height = 240;
  int fps = 60;
  float diam = 0.05;
  if ( argc == 2 )
    port = atoi ( argv[1] );
  else if ( argc == 3 )
  {
    address = argv[1];
    port = atoi ( argv[2] );
  }
  else if ( argc == 7 )
  {
    address = argv[1];
    port = atoi ( argv[2] );
    width = atoi ( argv[3] );
    height = atoi ( argv[4] );
    fps = atoi ( argv[5] );
    diam = atof ( argv[6] );
  }

  // Create application object
    OgreWiiApp app(address, port, width, height, fps, diam);

    try {
        app.go();
    } catch( Exception& e ) {
#if OGRE_PLATFORM == OGRE_PLATFORM_WIN32
        MessageBox( NULL, e.getFullDescription().c_str(), "An exception has occured!", MB_OK | MB_ICONERROR | MB_TASKMODAL);
#else
        std::cerr << "An exception has occured: " << e.getFullDescription();
#endif
    }


    return 0;
}

#ifdef __cplusplus
}
#endif
