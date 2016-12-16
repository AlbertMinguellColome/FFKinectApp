//
//  AbletonManager.h
//  ConductrEnter
//
//  Created by Albert Minguell
//
//

#pragma once

#ifndef __ConductrEnter__AbletonManager__
#define __ConductrEnter__AbletonManager__

#include <stdio.h>
#include "ofMain.h"
#include "ofxOsc.h"

class AbletonManager
{
public:

    static AbletonManager &getInstance()
    {
        static AbletonManager instance;
        return instance;
    }

    void init(string _senderHost, unsigned int _senderPort, unsigned int _receiverPort);

    void setSceneNumObjects(unsigned int numObjects);

    void playClip(int clipNumber, int trackNumber);
    void stopClip(int clipNumber, int trackNumber);
    void stopAll();
    void playScene(int sceneIndex);

    void setDeviceParameter(int track, int device, int parameter, int value);

    void requestTempo();
    void requestVolumeUpdates();
    void requestGridUpdates(); // In order to receive /live/playing_position

    void update();

    ofEvent<float>              eventTempoChanged;
    vector< ofEvent<float> >    eventsVolumeChanged;
    vector< ofEvent<float> >    eventsClipPositionChanged;

private:

    AbletonManager() {};

    void onTempoChanged(ofxOscMessage &m);
    void onTracksVolumeChanged(ofxOscMessage &m);
    void onClipPlayingPositionChanged(ofxOscMessage &m);
    void onMasterVolumeChanged(ofxOscMessage &m);

    ofxOscSender                oscSender;
    ofxOscReceiver              oscReceiver;

    unsigned int                sceneNumObjects;

    string                      senderHost;
    unsigned int                senderPort;
    unsigned int                receiverPort;
};


#endif /* defined(__ConductrEnter__AbletonListener__) */
