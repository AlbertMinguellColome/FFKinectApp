//
//  SettingsManager.h
//  ConductrEnter
//
//  Created by Miquel Ëngel Soler on 25/5/15.
//
//

#pragma once

#ifndef __ConductrEnter__SettingsManager__
#define __ConductrEnter__SettingsManager__

#include <stdio.h>
#include "ofxJSON.h"

class SettingsManager
{
public:
    static SettingsManager &getInstance()
    {
        static SettingsManager instance;
        return instance;
    }

    // App Restart
    unsigned int    restartDelay;
    bool            restartShowMessage;

    // Displax
    unsigned int    displaxWidth;
    unsigned int    displaxHeight;

    // Scene management
    unsigned int    sceneFirstScene;
    unsigned int    sceneNumArtists;
    unsigned int    scene1IntroLoopFrame;
    unsigned int    sceneIdleTimeToArtists;
    unsigned int    sceneIdleTimeToPlayground;

    // Ableton
    string          abletonHost;
    unsigned int    abletonSenderPort;
    unsigned int    abletonReceiverPort;
    unsigned int    abletonFirstClipIndex;
    unsigned int    abletonArtistOffset;

    // TUIO Client
    unsigned int    tuioClientPort;

    // TUIO Server
    unsigned int    tuioServerPort;

    // Color
    unsigned int    colorMinXOffset;
    unsigned int    colorCircleAlphaMin;
    unsigned int    colorCircleAlphaMax;

    // Debug Mode
    bool            debugShowGUI;
    bool            debugShowFPS;
    bool            debugShowTUIOCursors;
    unsigned int    debugS1BgMode;
    unsigned int    debugS2BgMode;
    bool            debugEnableVBO;
    bool            debugEnableVSync;
    bool            debugEnableDemoObjects;

    // Release Mode
    bool            releaseShowGUI;
    bool            releaseShowFPS;
    bool            releaseShowTUIOCursors;
    unsigned int    releaseS1BgMode;
    unsigned int    releaseS2BgMode;
    bool            releaseEnableVBO;
    bool            releaseEnableVSync;
    bool            releaseEnableDemoObjects;

private:

    SettingsManager();
    bool loadSettings();

    ofxJSONElement  json;
};

#endif /* defined(__ConductrEnter__SettingsManager__) */
