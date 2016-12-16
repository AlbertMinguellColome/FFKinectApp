//
//  ScreenSetup.h
//  ConductrEnter
//
//  Created by Miquel Àngel Soler on 24/6/15.
//
//

#pragma once

#ifndef __ConductrEnter__ScreenSetup__
#define __ConductrEnter__ScreenSetup__

#include <stdio.h>
#include "ofMain.h"

typedef enum
{
    SCREENMODE_WINDOW,
    SCREENMODE_FULL,
    SCREENMODE_DISPLAX,
    SCREENMODE_FULLWINDOW,
    SCREENMODE_NUMMODES
} ScreenMode;

class ScreenSetup
{
public:

    ScreenSetup() {};

    void setScreenMode(ScreenMode mode);
    void changeToNextMode();
    string getCurrentScreenModeString();
    ScreenMode getCurrentMode();

private:

    void setModeWindow();
    void setModeFull();
    void setModeDisplax();
    void setModeFullWindow();

    ofVec2f getMainScreenOrigin();
    ofVec2f getLeftmostMonitorCoord();

    ScreenMode currentMode;
};

#endif /* defined(__ConductrEnter__ScreenSetup__) */
