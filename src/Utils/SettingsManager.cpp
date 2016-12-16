//
//  SettingsManager.cpp
//  ConductrEnter
//
//  Created by Miquel Ëngel Soler on 25/5/15.
//
//

#include "SettingsManager.h"

const std::string FILENAME = "settings/settings.json";

///--------------------------------------------------------------
SettingsManager::SettingsManager()
{
    bool parserResult = loadSettings();
    if (!parserResult)
    {
        ofLog(OF_LOG_ERROR, "BAD FORMAT IN settings.json. Now quitting...");
        std::exit(EXIT_FAILURE);
    }
}

///--------------------------------------------------------------
bool SettingsManager::loadSettings()
{
    // JSON parse

    bool parsingSuccessful = json.open(FILENAME);
#ifdef OF_DEBUG
    cout << "PARSING RESULT: " << parsingSuccessful << endl;
#endif

    string prefix;

    // App Restart
    prefix = "App Restart";
    restartDelay = json[prefix]["Delay"].asUInt();
    restartShowMessage = json[prefix]["Show Message"].asBool();

    // Displax
    prefix = "Displax";
    displaxWidth = json[prefix]["Screen Width"].asUInt();
    displaxHeight = json[prefix]["Screen Height"].asUInt();

    // Scene management
    prefix = "Scenes";
    sceneFirstScene = json[prefix]["First Scene"].asUInt();
    sceneNumArtists = json[prefix]["Number Of Artists"].asUInt();
    scene1IntroLoopFrame = json[prefix]["Scene1 Intro Loop Frame"].asUInt();
    sceneIdleTimeToArtists = json[prefix]["Idle Time To Artists"].asUInt();
    sceneIdleTimeToPlayground = json[prefix]["Idle Time To Playground"].asUInt();

    // Ableton
    prefix = "Ableton";
    abletonHost = json[prefix]["Ableton Host"].asString();
    abletonSenderPort = json[prefix]["Ableton Sender Port"].asUInt();
    abletonReceiverPort = json[prefix]["Ableton Receiver Port"].asUInt();
    abletonFirstClipIndex = json[prefix]["First Clip Index"].asUInt();
    abletonArtistOffset = json[prefix]["Artist Offset"].asUInt();

    // TUIO Client
    prefix = "TUIO Client";
    tuioClientPort = json[prefix]["Client Port"].asUInt();

    // TUIO Server
    prefix = "TUIO Server";
    tuioServerPort = json[prefix]["Client Port"].asUInt();

    // Color
    prefix = "Color";
    colorMinXOffset = json[prefix]["Minimum X Offset"].asUInt();
    colorCircleAlphaMin = json[prefix]["Circle Alpha Min"].asUInt();
    colorCircleAlphaMax = json[prefix]["Circle Alpha Max"].asUInt();

    // Debug Mode
    prefix = "Debug Mode";
    debugShowGUI = json[prefix]["Show GUI"].asBool();
    debugShowFPS = json[prefix]["Show FPS"].asBool();
    debugShowTUIOCursors = json[prefix]["Show TUIO Cursors"].asBool();
    debugS1BgMode = json[prefix]["Scene1 Background Mode"].asUInt();
    debugS2BgMode = json[prefix]["Scene2 Background Mode"].asUInt();
    debugEnableVBO = json[prefix]["Enable VBO"].asBool();
    debugEnableVSync = json[prefix]["Enable Vertical Sync"].asBool();
    debugEnableDemoObjects = json[prefix]["Enable Demo Objects"].asBool();

    // Release mode
    prefix = "Release Mode";
    releaseShowGUI = json[prefix]["Show GUI"].asBool();
    releaseShowFPS = json[prefix]["Show FPS"].asBool();
    releaseShowTUIOCursors = json[prefix]["Show TUIO Cursors"].asBool();
    releaseS1BgMode = json[prefix]["Scene1 Background Mode"].asUInt();
    releaseS2BgMode = json[prefix]["Scene2 Background Mode"].asUInt();
    releaseEnableVBO = json[prefix]["Enable VBO"].asBool();
    releaseEnableVSync = json[prefix]["Enable Vertical Sync"].asBool();
    releaseEnableDemoObjects = json[prefix]["Enable Demo Objects"].asBool();

    return parsingSuccessful;
}
