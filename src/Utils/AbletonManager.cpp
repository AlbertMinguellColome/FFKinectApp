//
//  AbletonManager.cpp
//  ConductrEnter
//
//  Created by Miquel Ëngel Soler on 10/6/15.
//
//

#include "AbletonManager.h"

const unsigned int MAX_TRACKS = 20;

#pragma mark - Constructors

///--------------------------------------------------------------
void AbletonManager::init(string _senderHost, unsigned int _senderPort, unsigned int _receiverPort)
{
    senderHost = _senderHost;
    senderPort = _senderPort;
    receiverPort = _receiverPort;

    oscSender.setup(senderHost, senderPort);
    oscReceiver.setup(receiverPort);
}

///--------------------------------------------------------------
void AbletonManager::setSceneNumObjects(unsigned int numObjects)
{
    sceneNumObjects = numObjects;
    eventsVolumeChanged.clear();
    eventsClipPositionChanged.clear();

    for (int i=0; i<sceneNumObjects; i++)
    {
        eventsVolumeChanged.push_back(ofEvent<float>());
        eventsClipPositionChanged.push_back(ofEvent<float>());
    }
}

#pragma mark - Send messages

///--------------------------------------------------------------
/**
 * /live/play/clip
 * (int track, int clip)
 * Launches clip number clip in track number track
 */
void AbletonManager::playClip(int clipNumber, int trackNumber)
{
    ofxOscMessage m;
//    m.setAddress("/live/play/clip");
//    m.addIntArg(trackNumber); // Session view -> col number (track)
//    m.addIntArg(clipNumber); // Session view -> row number (clip)

    m.setAddress("/live/fire/clipslot");
    m.addIntArg(trackNumber); // Session view -> col number (track)
    m.addIntArg(clipNumber); // Session view -> row number (clip)
    m.addIntArg(1); // 1=play, 0=stop

    oscSender.sendMessage(m);
}

///--------------------------------------------------------------
/**
 * /live/stop/clip
 * (int track, int clip)
 * Stops clip number clip in track number track
 */
void AbletonManager::stopClip(int clipNumber, int trackNumber)
{
    ofxOscMessage m;
//    m.setAddress("/live/stop/clip");
//    m.addIntArg(trackNumber); // Session view -> col number (track)
//    m.addIntArg(clipNumber); // Session view -> row number (clip)

    m.setAddress("/live/fire/clipslot");
    m.addIntArg(trackNumber); // Session view -> col number (track)
    m.addIntArg(clipNumber); // Session view -> row number (clip)
    m.addIntArg(0); // 1=play, 0=stop

    oscSender.sendMessage(m);
}

///--------------------------------------------------------------
/**
 * /live/stop/track
 * (int track)
 * Stops track number track
 */
void AbletonManager::stopAll()
{
    for (unsigned int i=0; i<MAX_TRACKS; i++) {
        ofxOscMessage m;
        m.setAddress("/live/stop/track");
        m.addIntArg(i);
        oscSender.sendMessage(m);
    }

//    ofxOscMessage m;
//    m.setAddress("/live/stop");
//    oscSender.sendMessage(m);
}

///--------------------------------------------------------------
/**
* /live/play/scene
* (int scene)
* Plays scene with index sceneIndex
*/
void AbletonManager::playScene(int sceneIndex)
{
    ofxOscMessage m;
    m.setAddress("/live/play/scene");
    m.addIntArg(sceneIndex);
    oscSender.sendMessage(m);
}

///--------------------------------------------------------------
/**
 * /live/master/device
 * (int device, int parameter, int value)
 * Sets parameter on device on track number track to value
 */
void AbletonManager::setDeviceParameter(int track, int device, int parameter, int value)
{
    ofxOscMessage m;
    m.setAddress("/live/device");
    m.addIntArg(track);
    m.addIntArg(device);
    m.addIntArg(parameter);
    m.addIntArg(value);
    oscSender.sendMessage(m);
#ifdef OF_DEBUG
//    cout << "Message: " << m.getAddress() << "- T:" << track << " - D:" << device << " - P:" << parameter << " - V:" << value << endl;
#endif
}

///--------------------------------------------------------------
/**
 * /live/tempo
 * Request current tempo, replies with /live/tempo (float tempo)
 */
void AbletonManager::requestTempo()
{
    ofxOscMessage m;
    m.setAddress("/live/tempo");
    oscSender.sendMessage(m);
}

///--------------------------------------------------------------
/**
 * /live/volumeupdates (int window, int min_track, int max_track
 * Request volume updates
 */
void AbletonManager::requestVolumeUpdates()
{
    ofxOscMessage m;
    m.setAddress("/live/volumeupdates");
    m.addIntArg(0); // window
    m.addIntArg(0); // min_track
    m.addIntArg(8); // max_track
    
    oscSender.sendMessage(m);
}

///--------------------------------------------------------------
/**
* /live/gridupdates
* Request grid updates in order to receive clip playing position (/clip/playing_position)
*/

void AbletonManager::requestGridUpdates()
{
    ofxOscMessage m;
    m.setAddress("/live/gridupdates");
    m.addIntArg(0); // Window id
    m.addIntArg(0); // X (from track)
    m.addIntArg(0); // Y (from clip)
    m.addIntArg(8); // Width (track width, i.e. number of tracks)
    m.addIntArg(100); // Height (clip height, i.e. number of clips)

    oscSender.sendMessage(m);
}

#pragma mark - Receive messages

///--------------------------------------------------------------
void AbletonManager::update()
{
    while(oscReceiver.hasWaitingMessages())
    {
        ofxOscMessage m;
        oscReceiver.getNextMessage(&m);

        if (m.getAddress() == "/live/tempo")
            onTempoChanged(m);
        else if (m.getAddress() == "/live/track/meterblock")
            onTracksVolumeChanged(m);
        else if (m.getAddress() == "/live/master/meterblock")
            onMasterVolumeChanged(m);
        else if (m.getAddress() == "/clip/playing_position")
            onClipPlayingPositionChanged(m);

#ifdef OF_DEBUG
        cout << "[AbletonManager] Received " << m.getAddress() << endl;
#endif
    }
}

///--------------------------------------------------------------
void AbletonManager::onTempoChanged(ofxOscMessage &m)
{
    /**
     * Response for tempo changes:
     * /live/tempo
     * (float) tempo
     */
    float newTempo = m.getArgAsFloat(0);
    ofNotifyEvent(eventTempoChanged, newTempo, this);
}

///--------------------------------------------------------------
void AbletonManager::onTracksVolumeChanged(ofxOscMessage &m)
{
    /**
     * Response for track volume (amplitude) changes:
     * /live/track/meterblock
     * N triplets with:
     *   (int) track number
     *   (int) channel (0=left, 1=right)
     *   (float) volume (0..1)
     */

    int track, channel;
    float volume;

    int numArgs = m.getNumArgs();
    for (int i=0; i<numArgs; i+=3)
    {
        track = m.getArgAsInt32(i);
        channel = m.getArgAsInt32(i+1);
        volume = m.getArgAsFloat(i+2);

        if (channel == 1) continue; // Skip right channel messages
        if (track >= sceneNumObjects) continue; // Skip if this track index is larger than the number of objects in scene

        ofNotifyEvent(eventsVolumeChanged[track], volume, this);
    }
}

///--------------------------------------------------------------
void AbletonManager::onClipPlayingPositionChanged(ofxOscMessage &m)
{
    /**
    * Response for clip position changes:
    * /clip/playing_position
    * N triplets with:
    *   (int) track
    *   (int) clip
    *   (float) track position (0..1)
    */

    int track;
    float position;

    int numArgs = m.getNumArgs();
    for (int i=0; i<numArgs; i+=3)
    {
        track = m.getArgAsInt32(i);
//        clip = m.getArgAsInt32(i+1);
        position = m.getArgAsFloat(i+2);

        if (track >= sceneNumObjects) continue;
        ofNotifyEvent(eventsClipPositionChanged[track], position, this);
    }
}

///--------------------------------------------------------------
void AbletonManager::onMasterVolumeChanged(ofxOscMessage &m)
{
    /**
    * Response for master volume (amplitude) changes:
    * /live/master/meterblock
    * (int) channel (0=left, 1=right)
    * (float) volume (0..1)
    */
}
