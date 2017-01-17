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
    
  
    useMultikinect = false;
    
    if(useMultikinect){
        // kinect setup
        kinect0.open(true, true, 0, 2);
        // Note :
        // Default OpenCL device might not be optimal.
        // e.g. Intel HD Graphics will be chosen instead of GeForce.
        // To avoid it, specify OpenCL device index manually like following.
        // kinect1.open(true, true, 0, 2); // GeForce on MacBookPro Retina
        kinect0.start();
        kinect0.update();
    }else{
          setupKinect();
    }
    int w = 640;
    int h = 480;
    
    kinectDepth.allocate(w, h, 1);
    for(int i = 0; i < h*w; i++){
        kinectDepth[i] = 0;
    }

}

void ofApp::setupGui(){
    
    gui.setup();
    gui.setPosition(0, 40);
    gui.add(frameRate.setup("frameRate",60,1,60));
    gui.add(isDepthSmoothingActive.setup("isDepthSmoothingActive",0,25,25));
    gui.add(isSmoothingThresholdOnly.setup("isSmoothingThresholdOnly",0,25,25));
    gui.add(isNormalMapThresholdOnly.setup("isNormalMapThresholdOnly",0,25,25));
    gui.add(nearThreshold.setup("nearThreshold",0.0,0,4000));
    gui.add(farThreshold.setup("farThreshold",4000,0,4000));
    gui.add(meshBlurRadius.setup("meshBlurRadius",1,0,10));
    gui.add(zAveragingMaxDepth.setup("zAveragingMaxDepth",195,0,200));
    gui.add(blankDepthPixMax.setup("blankDepthPixMax",7,0,10));
    gui.add(activateSmooth.setup("activateSmooth",0,25,25));
    gui.add(innerThreshold.setup("innerThreshold",0,0,10));
    gui.add(outerThreshold.setup("outerThreshold",0,0,10));
    gui.add(front.setup("frontSlider",607,0,1500));
    gui.add(back.setup("backSlider",1680,0,8000));
    gui.add(smoothCount.setup("smoothCount",2,1,10));
    gui.add(temporalSmoothing.setup("temporalSmoothing",0,0,1));
    gui.add(pointSize.setup("pointSize",2,0,100));  // Increase-decrease point size use it with meshMode = 1 (GL_POINTS)
    gui.add(meshMode.setup("meshMode",3,1,4));  // It change mesh mode POINTS, LINES ,TRIANGLES = activates delanuay, LINES_LOOP
    gui.add(meshType.setup("meshType",2,1,3));// Changes between standard pointCloud , CubeMap and Texture mode
    gui.add(dummy.setup("dummy",0.1,-1,1));
    gui.add(meshResolution.setup("meshResolutionSlider",4,1,16)); //Increase-decrease resolution, use always pair values
    gui.add(displacement.setup("displacement",6,2,8)); // adjust kinect points Z-postion
    gui.add(cubeMapSelector.setup("cubeMapSelector",1,1,4));  // Change cube map images use with meshType = 3
    gui.add(displacementAmount.setup("displacementAmount",0.0,0,0.2));
    gui.add(cameraDistance.setup("cameraDistance",500,100,2000));
    gui.add(fatten.setup("fatten",0,-1,1));
    gui.add(cameraZoom.setup("cameraZoom",0,25,25)); //Zoom in-out cam.
    gui.add(drawLights.setup("drawLights",0,25,25));
    gui.add(activateLightStrobe.setup("activateLightStrobe",0,25,25));
    gui.add(lightStrobeFrequency.setup("lightStrobeFrequency",2,0,3));
    gui.add(flashSpeed.setup("flashSpeed",1,1,3));
    gui.add(cameraSpin.setup("cameraSpin",0,25,25));
    gui.add(activateParticles.setup("activateParticles",0,25,25)); // test with particles to future simulate delays , Atention! Drops FPS if not set higher values of meshResolution
    gui.add(showSolvers.setup("showSolvers",0,25,25));
    
    
    cubeMapSelector.addListener(this, &ofApp::updateCubeMap);
    meshMode.addListener(this,&ofApp::changeMeshMode);
    meshType.addListener(this,&ofApp::changeMeshType);
    isDepthSmoothingActive.addListener(this,&ofApp::setDepthSmoothingActive);
    nearThreshold.addListener(this,&ofApp::setNearThreshold);
    farThreshold.addListener(this,&ofApp::setFarThreshold);
    meshBlurRadius.addListener(this,&ofApp::setMeshBlurRadius);
    //zAveragingMaxDepth.addListener(this,&ofApp::setDepthSmoothingActive);
    //  blankDepthPixMax.addListener(this, &ofApp::setBlankDepthPixMax);
}

void ofApp::setDepthSmoothingActive(bool &val){
    kinectUtils.setDepthSmoothingActive(val);
}

void ofApp::setNearThreshold(float &val){
    kinectUtils.setNearThreshold(val);
}

void ofApp::setFarThreshold(float &val){
    kinectUtils.setFarThreshold(val);
}

void ofApp::setMeshBlurRadius(float &val){
    kinectUtils.setMeshBlurRadius(val);
}

void ofApp::setZAveragingMaxDepth(float &val){
    kinectUtils.setZAveragingMaxDepth(val);
}

void ofApp::setBlankDepthPixMax(float &val){
    kinectUtils.setBlankDepthPixMax(val);
}


//--------------------------------------------------------------
void ofApp::update() {
    ofSetFrameRate(frameRate);
    if(!cameraZoom){radius=cameraDistance;};
    if(cameraZoom){zoomInOutCamera();};
    if(cameraSpin){spinCamera();};
    if(!cameraSpin){updateCamera();};
    
    //updateKinectMesh();
    updateKinectV1Mesh();
    
    // Particle update -> TODO move to method
    for(unsigned int i = 0; i < p.size(); i++){
        p[i].update();
    }
    
    if(resizeFluid) 	{
        fluidSolver.setSize(fluidCellsX, fluidCellsX / msa::getWindowAspectRatio());
        fluidDrawer.setup(&fluidSolver);
        resizeFluid = false;
    }
    fluidSolver.update();
}

void ofApp::calcNormals(ofMesh &mesh) {
    for (int i = 0; i < mesh.getVertices().size(); i++)
        mesh.addNormal(ofPoint(0, 0, 0));
    
    for (int i = 0; i < mesh.getIndices().size(); i += 3) {
        const int ia = mesh.getIndices()[i];
        const int ib = mesh.getIndices()[i + 1];
        const int ic = mesh.getIndices()[i + 2];
        //  cout<<"Index:"<<i<<"\n";
        // cout<<ia <<ib<<ic<<"\n";
        if (ia >= mesh.getVertices().size() || ib >= mesh.getVertices().size() ||
            ic >= mesh.getVertices().size()) {
            return;
        }
        //     cout<<mesh.getVertices()[ia] - mesh.getVertices()[ib]
        //     <<mesh.getVertices()[ic] - mesh.getVertices()[ib]<<"\n";
        ofVec3f e1 = mesh.getVertices()[ia] - mesh.getVertices()[ib];
        ofVec3f e2 = mesh.getVertices()[ic] - mesh.getVertices()[ib];
        ofVec3f no = e1.cross(e2);
        
        mesh.getNormals()[ia] += no;
        mesh.getNormals()[ib] += no;
        mesh.getNormals()[ic] += no;
        
        // depending on your clockwise / winding order, you might want to reverse
        // the e2 / e1 above if your normals are flipped.
    }
}

static void setNormals( ofMesh &mesh ){
    
    //The number of the vertices
    int nV = mesh.getNumVertices();
    
    //The number of the triangles
    int nT = mesh.getNumIndices() / 3;
    
    vector<ofPoint> norm( nV ); //Array for the normals
    
    //Scan all the triangles. For each triangle add its
    //normal to norm's vectors of triangle's vertices
    for (int t=0; t<nT; t++) {
        
        //Get indices of the triangle t
        int i1 = mesh.getIndex( 3 * t );
        int i2 = mesh.getIndex( 3 * t + 1 );
        int i3 = mesh.getIndex( 3 * t + 2 );
        
        //Get vertices of the triangle
        const ofPoint &v1 = mesh.getVertex( i1 );
        const ofPoint &v2 = mesh.getVertex( i2 );
        const ofPoint &v3 = mesh.getVertex( i3 );
        
        //Compute the triangle's normal
        ofPoint dir = ( (v2 - v1).crossed( v3 - v1 ) ).normalized();
        
        //Accumulate it to norm array for i1, i2, i3
        norm[ i1 ] += dir;
        norm[ i2 ] += dir;
        norm[ i3 ] += dir;
    }
    
    //Normalize the normal's length
    for (int i=0; i<nV; i++) {
        norm[i].normalize();
    }
    
    //Set the normals to mesh
    mesh.clearNormals();
    mesh.addNormals( norm );
}

void ofApp::updateCubeMap(int &cubeMapSelector){
    changeCubeMapImages(cubeMapSelector, myCubeMap);
}

void ofApp::changeMeshMode(int &meshSelector){
    switch (meshSelector) {
        case 1:{
            mesh.setMode(OF_PRIMITIVE_POINTS);
        }
            break;
        case 2:{
            mesh.setMode(OF_PRIMITIVE_LINES);
        }
            break;
        case 3:{
            mesh.setMode(OF_PRIMITIVE_TRIANGLES);
        }
            break;
        case 4:{
            mesh.setMode(OF_PRIMITIVE_LINE_LOOP);
        }
            break;
            
        default:
            break;
    }
}

void ofApp::changeMeshType(int &meshTypeSelector){
}

void ofApp::changeCubeMapImages(int textureSelector, ofxCubeMap &myCubeMap) {

    switch (textureSelector) {
        case 0:
            break;
        case 1:
            myCubeMap.loadImages(
                                 "ame_bluefreeze/bluefreeze_rt.tga", "ame_bluefreeze/bluefreeze_lf.tga",
                                 "ame_bluefreeze/bluefreeze_up.tga", "ame_bluefreeze/bluefreeze_dn.tga",
                                 "ame_bluefreeze/bluefreeze_ft.tga", "ame_bluefreeze/bluefreeze_bk.tga");
            break;
        case 2:
            myCubeMap.loadImages("mp_ss/ss_rt.tga", "mp_ss/ss_lf.tga",
                                 "mp_ss/ss_up.tga", "mp_ss/ss_dn.tga",
                                 "mp_ss/ss_ft.tga", "mp_ss/ss_bk.tga");
            break;
        case 3:
            myCubeMap.loadImages(
                                 "sb_iceflow/iceflow_rt.tga", "sb_iceflow/iceflow_lf.tga",
                                 "sb_iceflow/iceflow_up.tga", "sb_iceflow/iceflow_dn.tga",
                                 "sb_iceflow/iceflow_ft.tga", "sb_iceflow/iceflow_bk.tga");
            break;
        case 4:
            myCubeMap.loadImages(
                                 "sb_strato/stratosphere_rt.tga", "sb_strato/stratosphere_lf.tga",
                                 "sb_strato/stratosphere_up.tga", "sb_strato/stratosphere_dn.tga",
                                 "sb_strato/stratosphere_ft.tga", "sb_strato/stratosphere_bk.tga");
            break;
            
        default:
            break;
    }
}

void ofApp::updateLights(){
    ofVec3f meshPosition = mesh.getCentroid();
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
    pointLight.setAttenuation(0.0, 0.01);
    
    spotLight.setSpotlight();
    spotLight.setDiffuseColor(ofColor::whiteSmoke);
    spotLight.setSpecularColor(ofColor::whiteSmoke);
    spotLight.setSpotlightCutOff(90);
    spotLight.setSpotConcentration(45);
    spotLight.setAttenuation(0.00, 0.001);
    
    
    spotLight.setPosition(0, 200, -100);
    
    spotLight90.setSpotlight();
    spotLight90.setDiffuseColor(ofColor::white);
    spotLight90.setSpecularColor(ofColor::white);
    spotLight90.setSpotlightCutOff(50);
    spotLight90.setSpotConcentration(45);
    spotLight90.setAttenuation(0.0, 0.001);
    
    spotLight180.setSpotlight();
    spotLight180.setDiffuseColor(ofColor::white);
    spotLight180.setSpecularColor(ofColor::white);
    spotLight180.setSpotlightCutOff(50);
    spotLight180.setSpotConcentration(45);
    spotLight180.setAttenuation(0.0, 0.001);
    
    spotLight270.setSpotlight();
    spotLight270.setDiffuseColor(ofColor::white);
    spotLight270.setSpecularColor(ofColor::white);
    spotLight270.setSpotlightCutOff(50);
    spotLight270.setSpotConcentration(45);
    spotLight270.setAttenuation(0.0, 0.001);
    
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
    float radius= 400;
    float x;
    float z;
    float y = 200 + dummy*400;
    
    x = xorig + radius * cos(0 * PI / 180.0);
    z = zorig + radius * -sin(0 * PI / 180.0);
    spotLight.setPosition(x, y, z);
    x = xorig + radius * cos(45 * PI / 180.0);
    z = zorig + radius * -sin(45 * PI / 180.0);
    spotLight45.setPosition(x, y, z);
    x = xorig + radius * cos(90 * PI / 180.0);
    z = zorig + radius * -sin(90 * PI / 180.0);
    spotLight90.setPosition(x, y, z);
    x = xorig + radius * cos(135 * PI / 180.0);
    z = zorig + radius * -sin(135 * PI / 180.0);
    spotLight135.setPosition(x, y, z);
    x = xorig + radius * cos(180 * PI / 180.0);
    z = zorig + radius * -sin(180 * PI / 180.0);
    spotLight180.setPosition(x, y, z);
    x = xorig + radius * cos(270 * PI / 180.0);
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

void ofApp:: smoothArray(int process, ofShortPixels &pix ){
//    printf("IN SMOOTH!\n");
    int w = 640;
    int h = 480;
    ofShortPixels tempPixels;
    tempPixels.allocate(w, h, 1);
    
    for(int y = 1; y<h-1; y++ ){
        for(int x = 1; x<w-1; x++){
            int index = x + y*w;
            tempPixels[index] = 0;
            if( pix[index] != 0 ){
                int base = pix[index];
                int total = 0;
                int numAdded = 0;
                for( int ty = y-1; ty <= y+1; ty++){
                    for( int tx = x-1; tx <= x+1; tx++){
                        int index2 = tx + ty*w;
                        if( abs(pix[index2] - base) < 50 ){
                            total += pix[index2];
                            numAdded++;
                        }
                    }
                    
                }
                if( numAdded > 0){
                    tempPixels[index] = (float)total/(float)numAdded;
                }
            }
        }
    }
    for(int y = 1; y<h-1; y++ ){
        for(int x = 1; x<w-1; x++){
            int index = x + y*w;
//            pix[index] = 0;
            pix[index] = tempPixels[index];
        }
    }
}

static void medianFilter(ofShortPixels & pix,int x, int y){
    int width = pix.getWidth();
    int matrixH = 5;
    int matrixV = 5;
    //create a sliding window of size 9
    std::vector<unsigned short> window;
    if (y>=(matrixH-1)/2){
            int horizontalDisplacement = (matrixH-1)/2;
            int verticalDisplacement = (matrixV-1)/2;
            for (int h=-horizontalDisplacement ; h<=horizontalDisplacement;h++){
                for (int v=-verticalDisplacement; v<=verticalDisplacement; v++ ){
                    window.push_back(pix[(y+v)*width + x+h]);
                }
            }
            sort(window.begin(),window.end());
            for (vector<int>::size_type i = 0; i != window.size(); ++i)
            // assign the median to centered element of the matrix
            pix[y*width+x] = window[(matrixV*matrixH-1)/2];
            window.clear();
    }
}

void ofApp::updateKinectV1Mesh() {
    kinect.update();
    
    if (kinect.isFrameNew()) {
        
        kinectUtils.processKinectData();
        int w = 640;
        int h = 480;
        //    for(int i = 0; i < smoothCount; i++ ){
        //        smoothArray( pix );
        //    }
        //    if(temporalSmoothing != 0.0){
        //        for(int i = 0; i < h*w; i++){
        //            int diff =pix[i] - kinectDepth[i];
        //            int pixd = pix[i];
        //
        //            if(abs(diff) > 90){
        //                kinectDepth[i] = pixd;
        //                /*
        //                pixd = kinectDepth[i];
        //                if( diff > 0 ){
        //                    pixd += 90;
        //                }else{
        //                    pixd -= 90;
        //                }
        //                 */
        //            }
        //            kinectDepth[i] = kinectDepth[i] * temporalSmoothing + pixd * (1.0-temporalSmoothing);
        //            pix[i] = kinectDepth[i];
        //        }
        //    }
        //    }
        //ofMesh mesh;
        //  mesh.setMode(OF_PRIMITIVE_TRIANGLES);
        mesh.clear();
        points.clear();
        indexs.clear();
        int step = meshResolution;
        
        
        for(int y = 0; y < h; y += step) {
            vector<ofVec3f> temppoints;
            vector<ofColor> tempcolors;
            points.push_back(temppoints);
            colors.push_back(tempcolors);
            for(int x = 0; x < w; x += step) {
                float distance = kinect.getDistanceAt(x, y);
                mesh.addColor(ofColor::white);
                ofVec3f tempPoint;
                tempPoint = kinectUtils.getProcessedVertex(x, y);
                points[y / step].push_back(tempPoint);
            }
        }
        
        // Create Vertex Indexes
        int ind = 0;
        for (int m = 0; m < h; m += step) {
            vector<int> tempindexs;
            indexs.push_back(tempindexs);
            
            for (int n = 0; n < w; n += step) {
                if (points[m / step][n / step].z != 0) {
                    ofVec3f ptTemp= points[m / step][n / step];
                    mesh.addVertex(ptTemp);
                    indexs[m / step].push_back(ind);
                    ind++;
                } else {
                    indexs[m / step].push_back(-1);
                }
            }
        }
        
        // Delanuay triangulation
        int W = int(w / step);
        for (int b = 0; b < h - step; b += step) {
            for (int a = 0; a < w - 1; a += step) {
                if ((indexs[int(b / step)][int(a / step)] != -1 &&
                     indexs[int(b / step)][int(a / step + 1)] != -1) &&
                    (indexs[int(b / step + 1)][int(a / step + 1)] != -1 &&
                     indexs[int(b / step + 1)][int(a / step)] != -1)) {
                        
                        mesh.addTriangle(indexs[int(b / step)][int(a / step)],
                                         indexs[int(b / step)][int(a / step + 1)],
                                         indexs[int(b / step + 1)][int(a / step + 1)]);
                        mesh.addTriangle(indexs[int(b / step)][int(a / step)],
                                         indexs[int(b / step + 1)][int(a / step + 1)],
                                         indexs[int(b / step + 1)][int(a / step)]);
                    }
            }
        }
        
        // Calculate normals
        //calcNormals(mesh);
        // Faster normal calculations
        setNormals(mesh);
        //Fatten algorithm
        for (int i = 0; i < mesh.getIndices().size(); i++) {
            const int ia = mesh.getIndices()[i];
            if (ia < mesh.getVertices().size() ) {
                
                //ofVec3f e1 = mesh.getVertices()[ia];
                ofVec3f norml = mesh.getNormals()[ia];
                float hello = sqrt(4);
                
                float l = sqrt(norml[0]*norml[0]+norml[1]*norml[1]+norml[2]*norml[2]);
                
                if (l != 0.0){
                    mesh.getVertices()[ia] = mesh.getVertices()[ia] + (norml*0.9*-3*fatten)/l;
                }
                
            }
        }
        
    }
}

void ofApp::updateKinectMesh(){
    kinect0.update();
    positionLights();

    if (!useMultikinect){
    
        int w = 640;
        int h = 480;
        int total = 0;
        int step = 2;
        for(int y = 0; y < h; y += step) {
            for(int x = 0; x < w; x += step) {
                if(kinect.getDistanceAt(x, y) > 0) {
                    mesh.addColor(kinect.getColorAt(x,y));
                    mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
                    
                }else{
                    
                }
            }
        }
        
        if (delayMode) {
            if (kinectFrameLimiter > 20) {
                kinectFrameLimiter = 0;
                mesh.clear();
            }
        }
        if (kinectFrameLimiter >= 0) {
            if(!delayMode)
                mesh.clear();
            
            points.clear();
            colors.clear();
            indexs.clear();
            
            {
                vector<demoParticle> tempParticles;
                for (int j = 0; j < h; j += step) {
                    vector<ofVec3f> temppoints;
                    vector<ofColor> tempcolors;
                    
                    points.push_back(temppoints);
                    colors.push_back(tempcolors);
                    
                    for (int i = 0; i < w; i += step) {
                        float distance =kinect.getDistanceAt(i, j);
                        float previous_distance;
                        float next_distance;
                        float up_distance;
                        float down_distance;
                        float z_difference;
                        if(i>0){
                            previous_distance = kinect.getDistanceAt(i-1, j);
                            next_distance = kinect.getDistanceAt(i+1, j);
                            if(j>0){
                                up_distance = kinect.getDistanceAt(i, j);
                                down_distance = kinect.getDistanceAt(i-1, j);
                                z_difference= (std::abs(distance-previous_distance) + std::abs(distance-next_distance) + std::abs(distance-up_distance)+std::abs(distance-down_distance))/4;
                            }else{
                                z_difference= (std::abs(distance-previous_distance) + std::abs(distance-next_distance))/2;
                            }
                        }
                        if (distance > front && distance < back) {
                            
                           
                                ofVec3f tempPoint;
                                ofColor tempColor;
                                demoParticle particle;
                                
                                tempPoint = ofVec3f(i, j, distance * -1 *displacement);
                                ofColor c;
                                float h = ofMap(distance, 50, 200, 0, 255, true);
                                c.setHsb(h, 255, 255);
                                points[j / step].push_back(tempPoint);
                                colors[j / step].push_back(ofColor::white);
                                particle.setPosition(tempPoint);
                                particle.setMode(PARTICLE_MODE_NOISE);
                                particle.reset();
                                particle.addColor(c);
                                tempParticles.push_back(particle);
                                total++;
                            
                        } else {
                            ofVec3f tempPoint2;
                            ofColor tempColor2;
                            tempPoint2 = ofVec3f(i, j, 0);
                            tempColor2 = ofColor(ofColor::yellow);
                            points[j / step].push_back(tempPoint2);
                            colors[j / step].push_back(tempColor2);
                        }
                    }
                }
                
                if(particleFrameLimiter>2 && activateParticles){
                    particleFrameLimiter=0;
                    p.reserve( p.size() + tempParticles.size() );                // preallocate memory without erase
                    p.insert( p.end(), tempParticles.begin(), tempParticles.end() );
                    tempParticles.clear();
                    
                }else if(!activateParticles){
                    p.clear();
                }
                
                int ind = 0;
                for (int m = 0; m < h; m += step) {
                    vector<int> tempindexs;
                    indexs.push_back(tempindexs);
                    
                    for (int n = 0; n < w; n += step) {
                        if (points[m / step][n / step].z != 0) {
                            //   cout << points[m][n] << endl;
                            mesh.addColor(colors[m / step][n / step]);
                            ofVec3f ptTemp= points[m / step][n / step];
                            //   ofVec3f pt = kinect0.getWorldCoordinateAt(ptTemp.x, ptTemp.y, ptTemp.z);
                            mesh.addVertex(ptTemp);
                            indexs[m / step].push_back(ind);
                            ind++;
                        } else {
                            indexs[m / step].push_back(-1);
                        }
                    }
                }
                // Drops 4 fps
                if (!pointCloudMode) {
                    int W = int(w / step);
                    for (int b = 0; b < h - step; b += step) {
                        for (int a = 0; a < w - 1; a += step) {
                            if ((indexs[int(b / step)][int(a / step)] != -1 &&
                                 indexs[int(b / step)][int(a / step + 1)] != -1) &&
                                (indexs[int(b / step + 1)][int(a / step + 1)] != -1 &&
                                 indexs[int(b / step + 1)][int(a / step)] != -1)) {
                                    
                                    mesh.addTriangle(indexs[int(b / step)][int(a / step)],
                                                     indexs[int(b / step)][int(a / step + 1)],
                                                     indexs[int(b / step + 1)][int(a / step + 1)]);
                                    mesh.addTriangle(indexs[int(b / step)][int(a / step)],
                                                     indexs[int(b / step + 1)][int(a / step + 1)],
                                                     indexs[int(b / step + 1)][int(a / step)]);
                                }
                        }
                    }
                }
                calcNormals(mesh);
                //                for (int i = 0; i < mesh.getIndices().size(); i++) {
                //                    const int ia = mesh.getIndices()[i];
                //                    if (ia < mesh.getVertices().size() ) {
                //
                //                        //ofVec3f e1 = mesh.getVertices()[ia];
                //                        ofVec3f norml = mesh.getNormals()[ia];
                //                        float hello = sqrt(4);
                //
                //                        float l = sqrt(norml[0]*norml[0]+norml[1]*norml[1]+norml[2]*norml[2]);
                //
                //                        if (l != 0.0){
                //                            mesh.getVertices()[ia] = mesh.getVertices()[ia] + (norml*0.9*10.0)/l;
                //                        }
                //
                //                    }
                //                }
            }
        }
        kinectFrameLimiter++;
        particleFrameLimiter++;

        
    }else{
        
    int step = meshResolution;
    int total = 0;
    int h = kinect0.getDepthPixelsRef().getHeight();
    int w = kinect0.getDepthPixelsRef().getWidth();
    
    if (kinect0.isFrameNew()) {
        if (delayMode) {
            if (kinectFrameLimiter > 20) {
                kinectFrameLimiter = 0;
                mesh.clear();
            }
        }
        if (kinectFrameLimiter >= 0) {
            if(!delayMode)
                mesh.clear();
            
            points.clear();
            colors.clear();
            indexs.clear();
            
            {
                vector<demoParticle> tempParticles;
                for (int j = 0; j < h; j += step) {
                    vector<ofVec3f> temppoints;
                    vector<ofColor> tempcolors;
                    
                    points.push_back(temppoints);
                    colors.push_back(tempcolors);
                    
                    for (int i = 0; i < w; i += step) {
                        float distance = kinect0.getDistanceAt(i, j);
                        float previous_distance;
                        float next_distance;
                        float up_distance;
                        float down_distance;
                        float z_difference;
                        if(i>0){
                            previous_distance = kinect0.getDistanceAt(i-1, j);
                            next_distance = kinect0.getDistanceAt(i+1, j);
                            if(j>0){
                                up_distance = kinect0.getDistanceAt(i, j);
                                down_distance = kinect0.getDistanceAt(i-1, j);
                                z_difference= (std::abs(distance-previous_distance) + std::abs(distance-next_distance) + std::abs(distance-up_distance)+std::abs(distance-down_distance))/4;
                            }else{
                                z_difference= (std::abs(distance-previous_distance) + std::abs(distance-next_distance))/2;
                            }
                        }
                        if (distance > front && distance < back) {
                            

                                ofVec3f tempPoint;
                                ofColor tempColor;
                                demoParticle particle;
                                
                                tempPoint = ofVec3f(i, j, distance * -1 *displacement);
                                ofColor c;
                                float h = ofMap(distance, 50, 200, 0, 255, true);
                                c.setHsb(h, 255, 255);
                                points[j / step].push_back(tempPoint);
                                colors[j / step].push_back(ofColor::white);
                                particle.setPosition(tempPoint);
                                particle.setMode(PARTICLE_MODE_NOISE);
                                particle.reset();
                                particle.addColor(c);
                                tempParticles.push_back(particle);
                                total++;
                            
                        } else {
                            ofVec3f tempPoint2;
                            ofColor tempColor2;
                            tempPoint2 = ofVec3f(i, j, 0);
                            tempColor2 = ofColor(ofColor::yellow);
                            points[j / step].push_back(tempPoint2);
                            colors[j / step].push_back(tempColor2);
                        }
                    }
                }
                
                if(particleFrameLimiter>2 && activateParticles){
                    particleFrameLimiter=0;
                    p.reserve( p.size() + tempParticles.size() );                // preallocate memory without erase
                    p.insert( p.end(), tempParticles.begin(), tempParticles.end() );
                    tempParticles.clear();
                    
                }else if(!activateParticles){
                    p.clear();
                }
                
                int ind = 0;
                for (int m = 0; m < h; m += step) {
                    vector<int> tempindexs;
                    indexs.push_back(tempindexs);
                    
                    for (int n = 0; n < w; n += step) {
                        if (points[m / step][n / step].z != 0) {
                            //   cout << points[m][n] << endl;
                            mesh.addColor(colors[m / step][n / step]);
                            ofVec3f ptTemp= points[m / step][n / step];
                            //   ofVec3f pt = kinect0.getWorldCoordinateAt(ptTemp.x, ptTemp.y, ptTemp.z);
                            mesh.addVertex(ptTemp);
                            indexs[m / step].push_back(ind);
                            ind++;
                        } else {
                            indexs[m / step].push_back(-1);
                        }
                    }
                }
                // Drops 4 fps
                if (!pointCloudMode) {
                    int W = int(w / step);
                    for (int b = 0; b < h - step; b += step) {
                        for (int a = 0; a < w - 1; a += step) {
                            int bstep = b/step;
                            int astep = a/step;
                            if ((indexs[int(bstep)][int(astep)] != -1 &&
                                 indexs[int(bstep)][int(astep + 1)] != -1) &&
                                (indexs[int(bstep + 1)][int(astep + 1)] != -1 &&
                                 indexs[int(bstep + 1)][int(astep)] != -1)) {
                                    
                                    mesh.addTriangle(indexs[int(bstep)][int(astep)],
                                                     indexs[int(bstep)][int(astep + 1)],
                                                     indexs[int(bstep + 1)][int(astep + 1)]);
                                    mesh.addTriangle(indexs[int(bstep)][int(astep)],
                                                     indexs[int(bstep + 1)][int(astep + 1)],
                                                     indexs[int(bstep + 1)][int(astep)]);
                                }
                        }
                    }
                }
                calcNormals(mesh);
                for (int i = 0; i < mesh.getIndices().size(); i++) {
                    const int ia = mesh.getIndices()[i];
                    if (ia < mesh.getVertices().size() ) {
                        
                        //ofVec3f e1 = mesh.getVertices()[ia];
                        ofVec3f norml = mesh.getNormals()[ia];
                        float hello = sqrt(4);
                        
                        float l = sqrt(norml[0]*norml[0]+norml[1]*norml[1]+norml[2]*norml[2]);
                        
                        if (l != 0.0){
                            mesh.getVertices()[ia] += norml*fatten*3.0/l;
                        }
                    }
                }
            }
        }
        kinectFrameLimiter++;
        particleFrameLimiter++;
    }
}
}

//--------------------------------------------------------------
void ofApp::draw() {
    //  ofBackgroundGradient(ofColor(64), ofColor(0));
    //ofEnableBlendMode(OF_BLENDMODE_ADD);
    ofEnableDepthTest();
    glPointSize(pointSize);
    glLineWidth(int(1));
    if(meshType == pointCloudMesh){drawPointCloudMode();};
    if(meshType == cubeMapMesh){drawCubeMapMode();};
    if(meshType == texturedMesh){drawTexturedMode();};
    if(showSolvers){drawSolvers();};
    ofDisableDepthTest();
    drawGui();
    
   // kinectUtils.drawProcessing(0,0);
   // grayImage.draw(0,0);
}

void ofApp::drawPointCloudMode(){
    mesh.setUsage(GL_DYNAMIC_DRAW);
    if (mesh.getVertices().size()) {
        ofPushStyle();
        glPointSize(pointSize);
        cam.begin();
        ofDrawAxis(200);
        ofPushMatrix();
        //ofRotateZ(-180);
        //ofTranslate(-kinect0.getDepthPixelsRef().getWidth()/2, -kinect0.getDepthPixelsRef().getHeight()/2, +600);
      //  ofScale(1, -1, -1);
       // ofTranslate(0, 0, -1000);
        mesh.draw();
        ofPopMatrix();
        
        if(activateParticles){
            ofPushMatrix();
            ofRotateZ(-180);
            ofTranslate(-kinect0.getDepthPixelsRef().getWidth()/2, -kinect0.getDepthPixelsRef().getHeight()/2, +600);
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
}

void ofApp::drawCubeMapMode(){
    cam.begin();
    myCubeMap.bind();
    cubeMapShader.begin();
    cubeMapShader.setUniform1i("envMap", 0);
    cubeMapShader.setUniform1f("reflectivity", 1);
    cubeMapShader.setUniform3f("pos_eye", cam.getPosition());
    ofPushMatrix();
    //ofRotateZ(-180);
   // sphere.draw();
    
    //ofTranslate(-kinect0.getDepthPixelsRef().getWidth()/2, -kinect0.getDepthPixelsRef().getHeight()/2, +600);
   // ofScale(1, -1, -1);
    ofTranslate(0, 0,400);
    mesh.drawFaces();
    ofPopMatrix();
    cubeMapShader.end();
    myCubeMap.drawSkybox(2000);
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
   // ofRotateZ(-180);
    //ofTranslate(-kinect0.getDepthPixelsRef().getWidth()/2, -kinect0.getDepthPixelsRef().getHeight()/2, +600);
  //  ofScale(1, -1, -1);
 //   ofTranslate(0, 0, -1000);
    mesh.drawFaces();
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
            spotLight.setSpotlightCutOff(spotLight.getSpotlightCutOff() + 1);
            break;
        case OF_KEY_DOWN:
            spotLight.setSpotlightCutOff(spotLight.getSpotlightCutOff() - 1);
            break;
        case OF_KEY_RIGHT:
            spotLight.setSpotConcentration(spotLight.getSpotConcentration() + 1);
            break;
        case OF_KEY_LEFT:
            spotLight.setSpotConcentration(spotLight.getSpotConcentration() - 1);
            break;
        case '1':
            bPointLight = !bPointLight;
            if (bPointLight) {
                phong.useLight(&pointLight);
            } else {
                phong.removeLight(&pointLight);
            }
            break;
        case '2':
            bSpotLight = !bSpotLight;
            if (bSpotLight) {
                phong.useLight(&spotLight);
            } else {
                phong.removeLight(&spotLight);
            }
            break;
        case '3':
            bDirLight = !bDirLight;
            if (bDirLight) {
                phong.useLight(&directionalLight);
            } else {
                phong.removeLight(&directionalLight);
            }
            break;
        case '4':
            textureSelector = 1;
            changeCubeMapImages(textureSelector, myCubeMap);
            break;
        case '5':
            textureSelector = 2;
            changeCubeMapImages(textureSelector, myCubeMap);
            break;
        case '6':
            textureSelector = 3;
            changeCubeMapImages(textureSelector, myCubeMap);
            break;
            
        case 'e':
            bShowHelp = !bShowHelp;
            break;
        case 'h':
            if (mat.getSpecularColor() == ofFloatColor(1., 1., 1.)) {
                mat.setSpecularColor(ofFloatColor(0., 0., 0.));
            } else {
                mat.setSpecularColor(ofFloatColor(1., 1., 1.));
            }
            break;
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
        case 'z':
            pos = sphere.getPosition();
            pos[2] -= 5;
            sphere.setPosition(pos);
            break;
        case 32:
            ofToggleFullscreen();
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

#pragma mark - Kinect 

void ofApp:: setupKinect(){
    // enable depth->video image calibration
    kinect.setRegistration(true);
    
    kinect.init();
    //kinect.init(true); // shows infrared instead of RGB video image
    //kinect.init(false, false); // disable video image (faster fps)
    
    kinect.open();		// opens first available kinect
    //kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
    //kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
    
    // print the intrinsic IR sensor values
    if(kinect.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
    }
    
    kinectUtils.setup(&kinect);
    
    bThreshWithOpenCV = true;
    // zero the tilt on startup
    angle = 0;
    kinect.setCameraTiltAngle(angle);
    // start from the front
    bDrawPointCloud = false;

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
