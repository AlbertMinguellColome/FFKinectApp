#include "ofApp.h"
#include "AbletonManager.h"
#include "ofxOpenCv.h"



char sz[] = "[Rd9?-2XaUP0QY[hO%9QTYQ`-W`QZhcccYQY[`b";



#pragma mark - Ableton

void volumeChanged(){
    
};
void setupAbleton(){
    string host = SettingsManager::getInstance().abletonHost;
    unsigned int senderPort = SettingsManager::getInstance().abletonSenderPort;
    unsigned int receiverPort = SettingsManager::getInstance().abletonReceiverPort;
    AbletonManager::getInstance().init(host, senderPort, receiverPort);
}

#pragma mark - App Cycle

void ofApp::setup() {
    
    setupGui();
    ofSetDataPathRoot("../Resources/data/");
    ofSetLogLevel(OF_LOG_VERBOSE);
    // disable vertical Sync is too bad with light sometimes
    ofSetVerticalSync(true);
    ofSetFrameRate(60);
    ofBackground(10, 10, 10);
    ofEnableAntiAliasing();
    
    // High resolution sphere
    ofSetSphereResolution(128);
    sphere.setRadius(80);
    sphere.setPosition(0, 0, -100);
    test.setPosition(0, 0, -100);
    
    // Shiny material
    mat.setSpecularColor(ofColor::white);
    mat.setShininess(120);
    tex.load("text1.jpg");
    

    kinectFrameLimiter = 2;
    particleFrameLimiter= 0;
    flashCount =0;
    int mode = 1;
    //changeMeshMode(mode);
    // Cube map setup
    textureSelector = 0;
    

    myCubeMap.loadImages(
                         "warehouse/px.jpg",
                         "warehouse/nx.jpg",
                         "warehouse/py.jpg",
                         "warehouse/ny.jpg",
                         "warehouse/pz.jpg",
                         "warehouse/nz.jpg");
    
    cubeMapShader.load("shaders/CubeMap");
    
    delayMode=false;
    radius=500;
    camCurrentX=0;
    camCurrentY=0;
    
    setupLights();
    phong.useMaterial(&mat);
    phong.useCamera(&cam);
    sphere.mapTexCoordsFromTexture(tex.getTexture());
    positionLights();
    
    
    setupSolver();
    setupAbleton();
    
    //   ofAddListener(AbletonManager::getInstance().eventsVolumeChanged[0], this, &ofApp::volumeChanged);
    //    ofAddListener(AbletonManager::getInstance().eventsVolumeChanged, this, &ofApp::volumeChanged);
    
    glShadeModel(GL_SMOOTH);
    useKinectV2 = true;
    kinectManager.init();
    
    if(useKinectV2){
        kinectManager.setupKinectV2();
    }else{
        kinectManager.setupKinectV1();
    }
    
    
    colorImg.allocate(640,480);
    grayImage.allocate(640,480);
    grayBg.allocate(640,480);
    grayDiff.allocate(640,480);
    
    bLearnBakground = true;
    threshold = 80;
}

void ofApp::setupGui(){
    
    gui.setup();
    gui.setPosition(-1000,-1000);
    gui.add(frameRate.setup("frameRate",60,1,60));
    gui.add(translateMesh.setup("translateMesh",900,0,1500));
    gui.add(meshMode.setup("meshMode",3,1,4));  // It change mesh mode POINTS, LINES ,TRIANGLES = activates delanuay, LINES_LOOP
    gui.add(meshType.setup("meshType",3,1,3));// Changes between standard pointCloud , CubeMap and Texture mode
    gui.add(displacement.setup("displacement",0,-300,300)); // adjust kinect points Z-postion
    gui.add(dummyY.setup("dummyY",1.0,-1,2));
    gui.add(dummyX.setup("dummyX",0.16,-1,1));
//    gui.add(radius.setup("radius",400,0,2000));
    gui.add(cubeMapSelector.setup("cubeMapSelector",1,1,4));  // Change cube map images use with meshType = 3
    gui.add(cameraDistance.setup("cameraDistance",450,100,2000));
    gui.add(cameraZoom.setup("cameraZoom",0,25,25)); //Zoom in-out cam.
    gui.add(drawLights.setup("drawLights",0,25,25));
    gui.add(activateLightStrobe.setup("activateLightStrobe",0,25,25));
    gui.add(lightStrobeFrequency.setup("lightStrobeFrequency",2,0,3));
    gui.add(flashSpeed.setup("flashSpeed",1,1,3));
    gui.add(cameraSpin.setup("cameraSpin",0,25,25)); // TODO : check if it's finally interesting to implement it, was just an idea.
//  gui.add(activateParticles.setup("activateParticles",0,25,25)); // test with particles to future simulate delays , Atention! Drops FPS if not set higher values of meshResolution
    gui.add(showSolvers.setup("showSolvers",0,25,25));
    cubeMapSelector.addListener(this, &ofApp::updateCubeMap);
    meshMode.addListener(this,&ofApp::changeMeshMode);
    meshType.addListener(this,&ofApp::changeMeshType);
}

//--------------------------------------------------------------
void ofApp::update() {
    ofSetFrameRate(frameRate);
    if(!cameraZoom){radius=cameraDistance;};
    if(cameraZoom){zoomInOutCamera();};
    if(cameraSpin){spinCamera();};
    if(!cameraSpin){updateCamera();};
    positionLights();
    if(useKinectV2){
        kinectManager.processKinectV2Data();
    }else{
        kinectManager.processKinectV1Data();
    }
    
    // Particle update -> TODO move to method
    for(unsigned int i = 0; i < p.size(); i++){
        p[i].update();
    }
    
    if(resizeFluid) 	{
        fluidSolver.setSize(fluidCellsX, fluidCellsX / msa::getWindowAspectRatio());
        fluidDrawer.setup(&fluidSolver);
        resizeFluid = false;
    }
    
    if (kinectManager.kinectV1.isFrameNew()){
        

        colorImg.setFromPixels(kinectManager.kinectV1.getPixels(),640,480);

        
        grayImage = colorImg;
        if (bLearnBakground == true){
            grayBg = grayImage;		// the = sign copys the pixels from grayImage into grayBg (operator overloading)
            bLearnBakground = false;
        }
        
        // take the abs value of the difference between background and incoming and then threshold:
        grayDiff.absDiff(grayBg, grayImage);
        grayDiff.threshold(threshold);
        
        // find contours which are between the size of 20 pixels and 1/3 the w*h pixels.
        // also, find holes is set to true so we will get interior contours as well....
        contourFinder.findContours(grayDiff, 80, (640*480)/1, 10, true);	// find holes

    }
    
    
    fluidSolver.update();
}


void ofApp::updateCubeMap(int &cubeMapSelector){
    changeCubeMapImages(cubeMapSelector, myCubeMap);
}

void ofApp::changeMeshMode(int &meshSelector){
    kinectManager.setMeshType(meshSelector);
}

void ofApp::changeMeshType(int &meshTypeSelector){
    if (meshType == 1){
       kinectManager.setMeshType(1);
    };
    if (meshType == 2){
        kinectManager.setMeshType(3);
    };
    if (meshType == 3){
        kinectManager.setMeshType(3);
    };
}

void ofApp::changeCubeMapImages(int textureSelector, ofxCubeMap &myCubeMap) {

    switch (textureSelector) {
        case 0:
            break;
        case 1:
            myCubeMap.loadImages(
                                 "warehouse/px.jpg",
                                 "warehouse/nx.jpg",
                                 "warehouse/py.jpg",
                                 "warehouse/ny.jpg",
                                 "warehouse/pz.jpg",
                                 "warehouse/nz.jpg");
            break;
        case 2:
            myCubeMap.loadImages(
                                 "icy/0004.png",
                                 "icy/0002.png",
                                 "icy/0006.png",
                                 "icy/0005.png",
                                 "icy/0001.png",
                                 "icy/0003.png");
            break;
  
        case 3:
            myCubeMap.loadImages(
                                 "blue/0004.png",
                                 "blue/0002.png",
                                 "blue/0006.png",
                                 "blue/0005.png",
                                 "blue/0001.png",
                                 "blue/0003.png");
            break;
        case 4:
            myCubeMap.loadImages(
                                 "volcano/0004.png",
                                 "volcano/0002.png",
                                 "volcano/0006.png",
                                 "volcano/0005.png",
                                 "volcano/0001.png",
                                 "volcano/0003.png");
            break;
            
        default:
            break;
    }
}

void ofApp::updateLights(){
    ofVec3f meshPosition = kinectManager.getMesh().getCentroid();
    pointLight.setPosition(100, 0, meshPosition.z+600);
    //spotLight.setPosition(0, 100, meshPosition.z+600);
    // spotLight90.setPosition(0, 100, meshPosition.z+600);
    spotLight.setPosition(spotLight.getPosition().x,spotLight.getPosition().y, meshPosition.z+600);
    spotLight90.setPosition(spotLight90.getPosition().x,spotLight90.getPosition().y, meshPosition.z+600-200);
    spotLight180.setPosition(spotLight180.getPosition().x,spotLight180.getPosition().y, meshPosition.z+600);
    spotLight270.setPosition(spotLight270.getPosition().x,spotLight270.getPosition().y, meshPosition.z+600+200);
    directionalLight.setPosition(-100, 0, meshPosition.z+600);
    directionalLight.setOrientation(ofVec3f(-90, 0, 0));
}

void ofApp::setupLights(){
    
    
    pointLight.setDiffuseColor(ofColor::black);
    pointLight.setSpecularColor(ofColor::black);
    pointLight.setPointLight();
    pointLight.setPosition(100, 0, -150);
    pointLight.setAttenuation(0.0, 0.001);
    
    spotLight.setSpotlight();
    spotLight.setDiffuseColor(ofColor::whiteSmoke);
    spotLight.setSpecularColor(ofColor::whiteSmoke);
    spotLight.setSpotlightCutOff(100);
    spotLight.setSpotConcentration(45);
    spotLight.setAttenuation(0.00, 0.0005);
    
    
    spotLight.setPosition(0, 200, -100);
    
    spotLight90.setSpotlight();
    spotLight90.setDiffuseColor(ofColor::white);
    spotLight90.setSpecularColor(ofColor::white);
    spotLight90.setSpotlightCutOff(100);
    spotLight90.setSpotConcentration(45);
    spotLight90.setAttenuation(0.0, 0.0001);
    
    spotLight180.setSpotlight();
    spotLight180.setDiffuseColor(ofColor::white);
    spotLight180.setSpecularColor(ofColor::white);
    spotLight180.setSpotlightCutOff(100);
    spotLight180.setSpotConcentration(45);
    spotLight180.setAttenuation(0.0, 0.0001);
    
    spotLight270.setSpotlight();
    spotLight270.setDiffuseColor(ofColor::white);
    spotLight270.setSpecularColor(ofColor::white);
    spotLight270.setSpotlightCutOff(50);
    spotLight270.setSpotConcentration(45);
    spotLight270.setAttenuation(0.0, 0.0005);
    
    directionalLight.setDiffuseColor(ofColor::black);
    directionalLight.setSpecularColor(ofColor::black);
    directionalLight.setDirectional();
    directionalLight.setPosition(-100, 0, -140);
    directionalLight.setOrientation(ofVec3f(0, 90, 0));
    
    spotLight.setOrientation(ofVec3f(-45, 90, 0));
    spotLight90.setOrientation(ofVec3f(225, 0, 0));
    spotLight180.setOrientation(ofVec3f(-45, -90, 0));
    spotLight270.setOrientation(ofVec3f(-45, 0, 0));
    
    // Activate/Deactivate all lights
    bPointLight=false;
    bSpotLight = false;
    bSpotLight90 = false;
    bSpotLight180 = false;
    bSpotLight270 = false;
    bDirLight = false;
    bShowHelp = false;
}

void ofApp::positionLights(){
    float xorig = 0;
    float zorig = 0;
    float radius= 800;
    float x;
    float z;
    float y = 200 + dummyY*400;
    
    x = xorig + radius * cos(0 * PI / 180.0) +dummyX*400;
    z = zorig + radius * -sin(0 * PI / 180.0);
    spotLight.setPosition(x, y, z);
    x = xorig + radius * cos(45 * PI / 180.0)+dummyX*400;
    z = zorig + radius * -sin(45 * PI / 180.0);
    spotLight45.setPosition(x, y, z);
    x = xorig + radius * cos(90 * PI / 180.0)+dummyX*400;
    z = zorig + radius * -sin(90 * PI / 180.0);
    spotLight90.setPosition(x, y, z);
    x = xorig + radius * cos(135 * PI / 180.0)+dummyX*400;
    z = zorig + radius * -sin(135 * PI / 180.0);
    spotLight135.setPosition(x, y, z);
    x = xorig + radius * cos(180 * PI / 180.0)+dummyX*400;
    z = zorig + radius * -sin(180 * PI / 180.0);
    spotLight180.setPosition(x, y, z);
    x = xorig + radius * cos(270 * PI / 180.0)+dummyX*400;
    z = zorig + radius * -sin(270 * PI / 180.0);
    spotLight270.setPosition(x, y, z);
}

void ofApp::strobeLights(){
    if(!activateLightStrobe){
        phong.useLight(&spotLight);
        phong.useLight(&spotLight90);
        phong.useLight(&spotLight180);
        phong.useLight(&spotLight270);
        return;
    }
    float frequency= lightStrobeFrequency;
    float angle = ofGetElapsedTimef()*2;
    float phase=0;
    float phase90=90 * M_PI/180;
    float phase180=180 * M_PI/180;
    float phase270=279 * M_PI/180;
    float tval =2*M_PI*frequency*ofGetElapsedTimef();
    
    float y = sin(tval+phase);
    float y90 = sin(tval+phase90);
    float y180 = sin(tval+phase180);
    float y270 = sin(tval+phase270);
    flashCount++;
    
    //bool flashOff = sin(tvalFlash)>0;
    int fstep = flashCount/flashSpeed;
    bool flashOff = fstep % 2;
    bSpotLight    = flashOff || y >-0.5;
    bSpotLight90  = flashOff || y90 >-0.5;
    bSpotLight180 = flashOff || y180 >-0.5;
    bSpotLight270 = flashOff || y270 >-0.5;
    
    
    if (!bSpotLight) {
        phong.useLight(&spotLight);
    }else {
        phong.removeLight(&spotLight);
        
    };
    if (!bSpotLight90) {
        phong.useLight(&spotLight90);
    }else {
        phong.removeLight(&spotLight90);
    }
    if (!bSpotLight180) {
        phong.useLight(&spotLight180);
    }else{
        phong.removeLight(&spotLight180);
        
    }
    if (!bSpotLight270) {
        phong.useLight(&spotLight270);
    }else{
        phong.removeLight(&spotLight270);
        
    }
    if(frequency==0){
        phong.removeLight(&spotLight);
        phong.removeLight(&spotLight90);
        phong.removeLight(&spotLight180);
        phong.removeLight(&spotLight270);
    }
}

void ofApp::zoomInOutCamera(){
    bool upDirection;
    if(radius>=1000){
        upDirection=false;
    }else if(radius<=500){
        upDirection=true;
    }
    if (upDirection) {
        radius= radius +0.1;
    }else{
        radius= radius -0.1;
    }
}

void ofApp::spinCamera(){
    float xorig = 0;
    float yorig = 0;
    float angle = ofGetElapsedTimef()*0.5;
    float x = xorig + radius * cos(angle);
    float y = yorig + radius * -sin(angle);
    camCurrentAngle=angle;
    camCurrentX= x;
    camCurrentY=y;
    cam.lookAt(ofVec3f(0,0,0));
    cam.setPosition(x, 0, y);
}

void ofApp::updateCamera(){
    cam.lookAt(ofVec3f(0,0,0));
    cam.setDistance(cameraDistance);
}
//--------------------------------------------------------------
void ofApp::draw() {
    ofEnableDepthTest();
    glLineWidth(int(1));
    if(meshType == pointCloudMesh){drawPointCloudMode();};
    if(meshType == cubeMapMesh){drawCubeMapMode();};
    if(meshType == texturedMesh){drawTexturedMode();};
    if(showSolvers){drawSolvers();};
    ofDisableDepthTest();
    drawGui();
    kinectManager.drawGui();
    
    ofFill();
   // ofSetHexColor(0x333333);
   // ofRect(360,540,320,240);
    ofSetHexColor(0xffffff);
    
    // we could draw the whole contour finder
  //  contourFinder.draw(0,0);
    
    float farPointX = 0;
    float farPointY = 0;
    
    for (int i = 0; i < contourFinder.nBlobs; i++){
        contourFinder.blobs[0].draw(0,0);
        
        // draw over the centroid if the blob is a hole
        
        for (int j = 0 ; j< contourFinder.blobs[i].nPts ; j++){
            
            if (contourFinder.blobs[0].pts[j].x>10 && contourFinder.blobs[i].pts[j].x< 630){
            if(contourFinder.blobs[0].pts[j].x > farPointX){
                farPointX =contourFinder.blobs[i].pts[j].x;
            }
            }
            if (contourFinder.blobs[0].pts[j].y>10 && contourFinder.blobs[i].pts[j].y< 470){
            if(contourFinder.blobs[0].pts[j].y > farPointY){
                farPointY =contourFinder.blobs[0].pts[j].y;
            }
            }
            
            
        }
    }
        ofSetColor(255);
        
//            float left =contourFinder.blobs[i].boundingRect.getLeft();
//            float right =contourFinder.blobs[i].boundingRect.getRight();
//            float up = contourFinder.blobs[i].boundingRect.getTop();
//            float down = contourFinder.blobs[i].boundingRect.getBottom();
//            if (right>previousFarpointX && i>0){
//                farPointX = right;
//            }
//            
//            if (up>previousFarpointY && i>0){
//                farPointY = up;
//            }
        
//            if(previousFarpointX==0){
//                previousFarpointX = farPointX;
//            }
//            if(previousFarpointY==0){
//                previousFarpointY = farPointY;
//            }
            if (std::abs(this->previousFarpointY-farPointY) > 40 && abs(this->previousFarpointX - farPointX) > 40){
           // printf("%f \n",previousFarpointX);
                ofVec2f vel = ofVec2f(3,3) / ofGetWindowSize();
                ofVec2f eventPos = ofVec2f(farPointX+100, farPointY+100);
                ofVec2f mouseNorm = ofVec2f(eventPos) / ofGetWindowSize();
                ofVec2f pastPoint =ofVec2f(previousFarpointX,previousFarpointY);
                ofVec2f mouseVel = ofVec2f(eventPos - pastPoint) / ofGetWindowSize();
                addToFluid(mouseNorm, mouseVel, true, true);
                previousFarpointX = farPointX;
                previousFarpointY = farPointY;
        
    }
    ofDrawBitmapString( "Fair point : " + ofToString(farPointX) + "\n",farPointX,farPointY);
}


void ofApp::drawPointCloudMode(){
      //mesh.setUsage(GL_DYNAMIC_DRAW);
        ofPushStyle();
        cam.begin();
        ofPushMatrix();
        ofRotateX(20);
        //ofRotateZ(-180);
        //ofTranslate(-kinect0.getDepthPixelsRef().getWidth()/2, -kinect0.getDepthPixelsRef().getHeight()/2, +600);
      //  ofScale(1, -1, -1);
       // ofTranslate(0, 0, -1000);
        
      //  ofTranslate(0, 0,translateMesh + displacement);
        //Refactored
        //mesh.draw();
        kinectManager.drawMesh(false);
        ofPopMatrix();
        
        if(activateParticles){
            ofPushMatrix();
            ofRotateZ(-180);
            
          //  ofTranslate(-kinect0.getDepthPixelsRef().getWidth()/2, -kinect0.getDepthPixelsRef().getHeight()/2, +600);
            for(unsigned int i = 0; i < p.size(); i++){
                //  printf("%f /n",p[i].pos.z);
                if(p[i].pos.y > 400){
                    p.erase(p.begin() + i);
                }else{
                    p[i].draw();
                }
            }
            ofPopMatrix();
        }
        cam.end();
        ofPopStyle();
}

void ofApp::drawCubeMapMode(){
    cam.begin();
    updateLights();
    strobeLights();
    myCubeMap.bind();
    cubeMapShader.begin();
    cubeMapShader.setUniform1i("envMap", 0);
    cubeMapShader.setUniform1f("reflectivity", 1);
    cubeMapShader.setUniform3f("pos_eye", cam.getPosition());
    ofPushMatrix();
    ofRotateX(20);
    //ofRotateZ(-180);
    //ofTranslate(-kinect0.getDepthPixelsRef().getWidth()/2, -kinect0.getDepthPixelsRef().getHeight()/2, +600);
   // ofScale(1, -1, -1);
  //  ofTranslate(0, 0,translateMesh + displacement);
    //Refactored
    //mesh.drawFaces();
    kinectManager.drawMesh(true);
    ofPopMatrix();
    cubeMapShader.end();
//    myCubeMap.drawSkybox(2000);
    myCubeMap.unbind();
   
    cam.end();
};

void ofApp::drawTexturedMode(){
    
    updateLights();
    strobeLights();
    //  positionLights();
    cam.begin();
    phong.begin();
    ofPushMatrix();
    ofRotateX(20);
   // ofRotateZ(-180);
    //ofTranslate(-kinect0.getDepthPixelsRef().getWidth()/2, -kinect0.getDepthPixelsRef().getHeight()/2, +600);
  //  ofScale(1, -1, -1);
 //   ofTranslate(0, 0, -1000);
 //   ofTranslate(0, 0,translateMesh + displacement);
    //mesh.drawFaces();
    kinectManager.drawMesh(true);
    ofDrawAxis(100);
    ofPopMatrix();
    phong.end();
    
    if (drawLights) {
        if (!bSpotLight){
            ofSetColor(spotLight.getDiffuseColor());
            spotLight.draw();
        }
        
        if (!bSpotLight90){
            ofSetColor(spotLight90.getDiffuseColor());
            spotLight90.draw();
        }
        if (!bSpotLight180){
            ofSetColor(spotLight180.getDiffuseColor());
            spotLight180.draw();
        }
        if (!bSpotLight270){
            ofSetColor(spotLight270.getDiffuseColor());
            spotLight270.draw();
        }
    }
    cam.end();
};

void ofApp::drawGui(){
    ofSetColor(255, 255, 255);
        ofDrawBitmapString( "FrameRate (Fr) : " + ofToString(ofGetFrameRate()) + "\n" +
                           + "Delay (d) : " + ofToString(delayMode) + "\n" ,
                           20, 20);
    
    gui.draw();
};

//--------------------------------------------------------------
void ofApp::keyPressed(int key) {
    ofVec3f pos;
    
    switch (key) {
        case OF_KEY_UP:
            cameraDistance = cameraDistance-30;
            break;
        case OF_KEY_DOWN:
            cameraDistance = cameraDistance+30;
            break;
        case OF_KEY_RIGHT:
            kinectManager.addFatten(0.2);
            break;
        case OF_KEY_LEFT:
            kinectManager.addFatten(-0.2);
            break;
        case '`':
            if (cameraSpin == 0){cameraSpin = 25;}
            else {cameraSpin=0;};
            break;
        case '1':
//            bPointLight = !bPointLight;
//            if (bPointLight) {
//                phong.useLight(&pointLight);
//            } else {
//                phong.removeLight(&pointLight);
//            }
            meshType = 1;
            
            break;
        case '2':
//            bSpotLight = !bSpotLight;
//            if (bSpotLight) {
//                phong.useLight(&spotLight);
//            } else {
//                phong.removeLight(&spotLight);
//            }
            meshType = 3;
            break;
        case '3':
//            bDirLight = !bDirLight;
//            if (bDirLight) {
//                phong.useLight(&directionalLight);
//            } else {
//                phong.removeLight(&directionalLight);
//            }
            
            meshType =2;
            break;
        case 'q':
            if (activateLightStrobe == 0){activateLightStrobe = 25;}
            else {activateLightStrobe=0;};
            
            
            break;
            
        case '/':
            if (showSolvers == 0){showSolvers = 25;}
            else {showSolvers=0;};
            
            
            break;
        
        case 'w':
            meshMode =4;
            break;
        case '4':
            textureSelector = 4;
            changeCubeMapImages(textureSelector, myCubeMap);
            break;
        case '5':
            textureSelector = 2;
            changeCubeMapImages(textureSelector, myCubeMap);
            break;
        case '6': 
            textureSelector = 1;
            changeCubeMapImages(textureSelector, myCubeMap);
            break;
            
        case '7':
            textureSelector = 3;
            changeCubeMapImages(textureSelector, myCubeMap);
            break;
        case 'a':
            angle = angle +5;
            kinectManager.kinectV1.setCameraTiltAngle(angle);
            if (angle>25){
                angle=25;};

            break;
        case 'z':
            angle = angle -5;
            kinectManager.kinectV1.setCameraTiltAngle(angle);
            if (angle<-25){
                angle=-25;};
            

            break;
            
        case 'e':
            bShowHelp = !bShowHelp;
            break;
//        case 'h':
//            if (mat.getSpecularColor() == ofFloatColor(1., 1., 1.)) {
//                mat.setSpecularColor(ofFloatColor(0., 0., 0.));
//            } else {
//                mat.setSpecularColor(ofFloatColor(1., 1., 1.));
//            }
//            break;
        case 's':
            if (phong.lightingMethod() == ofxShadersFX::Lighting::PHONG_LIGHTING) {
                phong.setLightingMethod(ofxShadersFX::Lighting::BLINN_PHONG_LIGHTING);
            } else {
                phong.setLightingMethod(ofxShadersFX::Lighting::PHONG_LIGHTING);
            }
            break;
        case 'y':
            if (phong.shadingMethod() == ofxShadersFX::Lighting::VERTEX_SHADING) {
                phong.setShadingMethod(ofxShadersFX::Lighting::PIXEL_SHADING);
            } else {
                phong.setShadingMethod(ofxShadersFX::Lighting::VERTEX_SHADING);
            }
            break;
        case 'm':
            if (cubeMapReflection == true) {
                cubeMapReflection = false;
            } else {
                cubeMapReflection = true;
            }
            break;
        case 'd':
            if (delayMode == true) {
                delayMode = false;
            } else {
                delayMode = true;
                pointCloudMode=true;
            }
            break;
        case 't':
            if (phong.texture() == NULL) {
                phong.useTexture(&tex);
            } else {
                phong.removeTexture();
            }
            break;
            
        case 'c':
            pos = sphere.getPosition();
            pos[2] += 5;
            sphere.setPosition(pos);
            break;
        case 'p':
            if (pointCloudMode == true) {
                pointCloudMode = false;
            } else {
                pointCloudMode = true;
            }
            break;
        case ',':
            ofHideCursor();
            break;
        case '.':
            ofShowCursor();
            break;
//        case 'z':
//            pos = sphere.getPosition();
//            pos[2] -= 5;
//            sphere.setPosition(pos);
//            break;
        case 32:
            ofToggleFullscreen();
            break;
        case 'j':
            gui.setPosition(-1000,-1000);
            kinectManager.setGuiPosition(-1000,-1000);
            break;
        case 'h':
            gui.setPosition(0, 40);
            kinectManager.setGuiPosition(0,450);
            break;
    }
    
}

void ofApp::setupSolver(){
    for(int i=0; i<strlen(sz); i++) sz[i] += 20;
    
    // setup fluid stuff
    fluidSolver.setup(100, 100);
    fluidSolver.enableRGB(true).setFadeSpeed(0.002).setDeltaT(0.5).setVisc(0.00015).setColorDiffusion(0);
    fluidDrawer.setup(&fluidSolver);
    
    fluidCellsX			= 150;
    
    drawFluid			= true;
    drawParticles		= true;
    resizeFluid=true;
    colorMult=100;
    velocityMult=100;
    drawFluid=true;
    
    ofSetFrameRate(60);
    ofBackground(0, 0, 0);
    ofSetVerticalSync(false);
    
    
    windowResized(ofGetWidth(), ofGetHeight());		// force this at start (cos I don't think it is called)
    pMouse = msa::getWindowCenter();
    resizeFluid			= true;
    
    fluidDrawer.setDrawMode(msa::fluid::kDrawVectors);
}

void ofApp::fadeToColor(float r, float g, float b, float speed) {
    glColor4f(r, g, b, speed);
    ofRect(0, 0, ofGetWidth(), ofGetHeight());
}


// add force and dye to fluid, and create particles
void ofApp::addToFluid(ofVec2f pos, ofVec2f vel, bool addColor, bool addForce) {
    float speed = vel.x * vel.x  + vel.y * vel.y * msa::getWindowAspectRatio() * msa::getWindowAspectRatio();    // balance the x and y components of speed with the screen aspect ratio
    if(speed > 0) {
        pos.x = ofClamp(pos.x, 0.0f, 1.0f);
        pos.y = ofClamp(pos.y, 0.0f, 1.0f);
        
        int index = fluidSolver.getIndexForPos(pos);
        
        if(addColor) {
            //			Color drawColor(CM_HSV, (getElapsedFrames() % 360) / 360.0f, 1, 1);
            ofColor drawColor;
            drawColor.setHsb((ofGetFrameNum() % 255), 255, 255);
            
            fluidSolver.addColorAtIndex(index, drawColor * colorMult);
            
            if(drawParticles)
                particleSystem.addParticles(pos * ofVec2f(ofGetWindowSize()), 10);
        }
        
        if(addForce)
            fluidSolver.addForceAtIndex(index, vel * velocityMult);
        
    }
}

void ofApp::drawSolvers(){
    
    ofEnableAlphaBlending();
    ofSetBackgroundAuto(false);
    
    if(showSolver){};
    if(drawFluid) {
        ofClear(0);
        glColor3f(1, 1, 1);
        fluidDrawer.draw(0, 0, ofGetWidth(), ofGetHeight());
    } else {
        //		if(ofGetFrameNum()%5==0)
        fadeToColor(0, 0, 0, 0.01);
    }
    if(drawParticles)
        particleSystem.updateAndDraw(fluidSolver, ofGetWindowSize(), drawFluid);
    
    ofDisableAlphaBlending();
    ofSetBackgroundAuto(true);
}


#pragma mark - camera tweens



#pragma mark - key events

//--------------------------------------------------------------
void ofApp::keyReleased(int key) {}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y) {
    ofVec2f eventPos = ofVec2f(x, y);
    ofVec2f mouseNorm = ofVec2f(eventPos) / ofGetWindowSize();
    ofVec2f mouseVel = ofVec2f(eventPos - pMouse) / ofGetWindowSize();
    addToFluid(mouseNorm, mouseVel, true, true);
    pMouse = eventPos;
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button) {
    ofVec2f eventPos = ofVec2f(x, y);
    ofVec2f mouseNorm = ofVec2f(eventPos) / ofGetWindowSize();
    ofVec2f mouseVel = ofVec2f(eventPos - pMouse) / ofGetWindowSize();
    addToFluid(mouseNorm, mouseVel, false, true);
    pMouse = eventPos;
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button) {}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button) {}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h) {}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg) {}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo) {}
