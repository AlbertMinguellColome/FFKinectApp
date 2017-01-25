//
//  FFKinectManager.hpp
//  FFKinectApp
//
//  Created by Albert Minguell Colome on 24/1/17.
//
//

#include "ofMain.h"
#include "ofxKinect.h"
#include "ofxMultiKinectV2.h"
#include "ofxKinectDepthUtils.h"
#include "ofxGui.h"
#include "demoParticle.h"

class FFKinectMeshManager {
    
public:
    ofxKinect kinectV1;
    ofxMultiKinectV2 kinectV2;
    ofxKinectDepthUtils  kinectUtils;
    ofxPanel gui;
    ofVboMesh mesh;
    void init();
    void setupKinectV1();
    void setupKinectV2();
    void setupGui();
    void processKinectV1Data();
    void processKinectV2Data();
    void drawMesh(bool faced);
    
private:
    vector< vector<ofVec3f> > points;
    vector< vector<ofColor> > colors;
    vector< vector<int> > indexs;
    vector <demoParticle> p;
    ofShortPixels kinectDepth;
    ofxFloatSlider nearThreshold;
    ofxFloatSlider farThreshold;
    ofxFloatSlider meshBlurRadius;
    ofxFloatSlider zAveragingMaxDepth;
    ofxFloatSlider blankDepthPixMax;
    ofxToggle isDepthSmoothingActive;
    ofxToggle isRGBMapActive;
    ofxIntSlider  displacement;
    ofxToggle activateSmooth;
    ofxIntSlider  smoothCount;
    ofxFloatSlider temporalSmoothing;
    ofxIntSlider  meshResolution;
    ofxFloatSlider fatten;
    ofxToggle activateParticles;
    ofxIntSlider  meshMode;
    ofxIntSlider  meshType;
    int  particleFrameLimiter;
    int  kinectFrameLimiter;
    void changeMeshType(int &meshTypeSelector);
    void changeMeshMode(int &meshSelector);
    void smoothArray(ofShortPixels &pix);
    void calcNormals(ofMesh &mesh);
    void drawGui(int x, int y);
    void setDepthSmoothingActive(bool &val);
    void setNearThreshold(float &val);
    void setFarThreshold(float &val);
    void setMeshBlurRadius(float &val);
    void setZAveragingMaxDepth(float &val);
    void setBlankDepthPixMax(float &val);
};
