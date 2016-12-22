#pragma once

#include "ofMain.h"
#include "ofxShadersFX.h"
#include "ofxMultiKinectV2.h"
#include "ofxCubeMap.h"
#include "ofxGui.h"
#include "MSAFluid.h"
//#include "MSATimer.h"
#include "ParticleSystem.h"
#include "demoParticle.h"
#include "SettingsManager.h"
#include "ofxOpenCv.h"
#include "ofxKinectV2.h"
#include "ofxKinect.h"

enum meshType
{
    pointCloudMesh = 1,
    cubeMapMesh=2,
    texturedMesh=3
};

class ofApp : public ofBaseApp{
    
public:
    
    ofxCvColorImage colorImg;
    ofxCvFloatImage depthFloat;
    
    ofxCvGrayscaleImage grayImage; // grayscale depth image
    ofxCvGrayscaleImage grayThreshNear; // the near thresholded image
    ofxCvGrayscaleImage grayThreshFar;
    
    ofShortPixels kinectDepth;

    ofLight pointLight;
    ofLight directionalLight;
    ofLight spotLight;
    ofLight spotLight45;
    ofLight spotLight90;
    ofLight spotLight135;
    ofLight spotLight180;
    ofLight spotLight125;
    ofLight spotLight270;
    ofLight spotLight315;
    ofLight spotLight360;
    
    ofEasyCam cam;
    ofxShadersFX::Lighting::LightingShader phong;
    ofMaterial mat;
    ofSpherePrimitive sphere;
    ofSpherePrimitive test;
    
    bool useMultikinect;
    bool bPointLight;
    bool bSpotLight;
    bool bSpotLight90;
    bool bSpotLight180;
    bool bSpotLight270;
    bool bDirLight;
    bool bShowHelp;
    bool cubeMapReflection;
    bool pointCloudMode;
    bool delayMode;
    float radius;
    float maxX;
    float minX;
    float minY;
    float maxY;
    float camCurrentX;
    float camCurrentY;
    float camCurrentAngle;
    
    ofImage tex;
    ofxMultiKinectV2 kinect0;
    vector < shared_ptr<ofxKinectV2> > kinects;
    ofxKinect kinect;
    ofVboMesh mesh;
    enum meshType meshtype;
    
    vector< vector<ofVec3f> > points;
    vector< vector<ofColor> > colors;
    vector< vector<int> > indexs;
    
    ofxCubeMap myCubeMap;
    ofShader cubeMapShader;
    
    
    int textureSelector;
    int kinectFrameLimiter;
    int particleFrameLimiter;
    
    //Gui Panel
    ofxPanel gui;
    ofxIntSlider  front;
    ofxIntSlider  back;
    ofxIntSlider  pointSize;
    ofxIntSlider  meshMode;
    ofxIntSlider  smoothCount;
    ofxIntSlider  meshResolution;
    ofxIntSlider  displacement;
    ofxIntSlider  meshType;
    ofxIntSlider  cutoff;
    ofxIntSlider  lightStrobeFrequency;
    ofxFloatSlider cameraDistance;
    ofxFloatSlider ZFilterMesh;
    ofxToggle showSolvers;
    ofxToggle cameraZoom;
    ofxToggle cameraSpin;
    ofxToggle showSolver;
    ofxToggle activateParticles;
    ofxToggle activatePointCloud;
    ofxToggle activateLightStrobe;
    ofxIntSlider  cubeMapSelector;
    ofxFloatSlider displacementAmount;
    ofxFloatSlider temporalSmoothing;
    
    //Solver
    float                   colorMult;
    float                   velocityMult;
    int                     fluidCellsX;
    bool                    resizeFluid;
    bool                    drawFluid;
    bool                    drawParticles;
    ofVec2f                 pMouse;
    msa::fluid::Solver      fluidSolver;
    msa::fluid::DrawerGl	fluidDrawer;
    
    //Particles
    ParticleSystem          particleSystem;
    particleMode currentMode;
    string currentModeStr;
    vector <demoParticle> p;
    vector <ofPoint> attractPoints;
    vector <ofPoint> attractPointsWithMovement;

    
    
    void setup();
    void update();
    void draw();
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
    //Calculates mesh normals
    void calcNormals(ofMesh &mesh);
    //Draws panel with all parameters
    void drawGui();
    //Draws pointCloud
    void drawPointCloudMode();
    //Draws cubeMap
    void drawCubeMapMode();
    //Draws texture
    void drawTexturedMode();
    //Camera Zoom Fx
    void zoomInOutCamera();
    //Camera Spin Fx
    void spinCamera();
    //Update camera positions
    void updateCamera();
    //Update Kinect mesh with Kinect
    void updateKinectMesh();
    void updateKinectV1Mesh();
    //Setup kinectV1
    void setupKinect();
    //Update CubeMap with selected one
    void updateCubeMap(int &cubeMapSelector);
    //Update mesh mode with selected parameter
    void changeMeshMode(int &meshSelector);
    //Update mesh type with selected parameter
    void changeMeshType(int &meshTypeSelector);
    //Change cubeMap image
    void changeCubeMapImages(int textureSelector, ofxCubeMap &myCubeMap);
    //Sets Position Lights
    void positionLights();
    //Setup Lights on Scene
    void setupLights();
    //Update lights
    void updateLights();
    //Activates strobe lights
    void strobeLights();
    
   // void volumeChanged(float &newVolume);
    
    //Solver
    void setupSolver();
    void fadeToColor(float r, float g, float b, float speed);
    void addToFluid(ofVec2f pos, ofVec2f vel, bool addColor, bool addForce);
    void drawSolvers();
    
    //Kinect 1
    int nearThreshold;
    int farThreshold;
    int angle;
    bool bThreshWithOpenCV;
    bool bDrawPointCloud;
};
