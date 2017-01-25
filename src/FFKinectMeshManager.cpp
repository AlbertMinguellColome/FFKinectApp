//
//  FFKinectMeshManager.cpp
//  FFKinectApp
//
//  Created by Albert Minguell Colome on 24/1/17.
//
//

#include "FFKinectMeshManager.h"


void FFKinectMeshManager::init (){

    int w = 640;
    int h = 480;
    kinectDepth.allocate(w, h, 1);
    for(int i = 0; i < h*w; i++){
        kinectDepth[i] = 0;
    }
    setupGui();
}


void FFKinectMeshManager::setupKinectV1(){
    // enable depth->video image calibration
    kinectV1.setRegistration(true);
    kinectV1.init();
    //kinect.init(true); // shows infrared instead of RGB video image
    //kinect.init(false, false); // disable video image (faster fps)
    kinectV1.open();		// opens first available kinect
    //kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
    //kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
    // print the intrinsic IR sensor values
    if(kinectV1.isConnected()) {
        ofLogNotice() << "sensor-emitter dist: " << kinectV1.getSensorEmitterDistance() << "cm";
        ofLogNotice() << "sensor-camera dist:  " << kinectV1.getSensorCameraDistance() << "cm";
        ofLogNotice() << "zero plane pixel size: " << kinectV1.getZeroPlanePixelSize() << "mm";
        ofLogNotice() << "zero plane dist: " << kinectV1.getZeroPlaneDistance() << "mm";
    }
    kinectUtils.setup(&kinectV1);
    kinectUtils.setFarThreshold(farThreshold);
    kinectUtils.setNearThreshold(nearThreshold);
}

void FFKinectMeshManager::setupKinectV2(){
    kinectV2.open(true, true, 0, 2);
    // Note :
    // Default OpenCL device might not be optimal.
    // e.g. Intel HD Graphics will be chosen instead of GeForce.
    // To avoid it, specify OpenCL device index manually like following.
    // kinect1.open(true, true, 0, 2); // GeForce on MacBookPro Retina
    kinectV2.start();
    kinectV2.update();
}

void FFKinectMeshManager::setupGui(){
    
    gui.setup();
    gui.add(isRGBMapActive.setup("isDepthSmoothingActive",0,25,25));
    gui.add(isDepthSmoothingActive.setup("isDepthSmoothingActive",0,25,25));
    gui.add(meshBlurRadius.setup("meshBlurRadius",1,0,10));
    
    //    gui.add(isSmoothingThresholdOnly.setup("isSmoothingThresholdOnly",0,25,25)); // TODO : Test with shader
    //    gui.add(isNormalMapThresholdOnly.setup("isNormalMapThresholdOnly",0,25,25)); // TODO : Test with shader
    gui.add(nearThreshold.setup("nearThreshold",120,0,4000));
    gui.add(farThreshold.setup("farThreshold",1420,0,4000));
    //  gui.add(zAveragingMaxDepth.setup("zAveragingMaxDepth",195,0,200));  // TODO : Test with shader
    //  gui.add(blankDepthPixMax.setup("blankDepthPixMax",7,0,10)); // TODO : Test with shader
    gui.add(activateSmooth.setup("activateSmooth",0,25,25));
    gui.add(smoothCount.setup("smoothCount",2,1,10));
    gui.add(temporalSmoothing.setup("temporalSmoothing",0,0,1));
    gui.add(meshMode.setup("meshMode",3,1,4));  // It change mesh mode POINTS, LINES ,TRIANGLES = activates delanuay, LINES_LOOP
    gui.add(meshType.setup("meshType",3,1,3));// Changes between standard pointCloud , CubeMap and Texture mode
    gui.add(meshResolution.setup("meshResolutionSlider",2,1,16)); //Increase-decrease resolution, use always pair values
    gui.add(displacement.setup("displacement",0,-300,300)); // adjust kinect points Z-postion
    gui.add(fatten.setup("fatten",0,-4,4));

    //    gui.add(radius.setup("radius",400,0,2000));
    //  gui.add(displacementAmount.setup("displacementAmount",0.0,0,0.2));// TODO : Test with shader
    //  gui.add(activateParticles.setup("activateParticles",0,25,25)); // test with particles to future simulate delays , Atention! Drops FPS if not set higher values of meshResolution
    
  

    isDepthSmoothingActive.addListener(this,&FFKinectMeshManager::setDepthSmoothingActive);
    nearThreshold.addListener(this,&FFKinectMeshManager::setNearThreshold);
    farThreshold.addListener(this,&FFKinectMeshManager::setFarThreshold);
    meshBlurRadius.addListener(this,&FFKinectMeshManager::setMeshBlurRadius);
    //zAveragingMaxDepth.addListener(this,&ofApp::setDepthSmoothingActive);
    //  blankDepthPixMax.addListener(this, &ofApp::setBlankDepthPixMax);
}
void FFKinectMeshManager::drawGui(int x, int y){
    gui.setPosition(x,y);
    gui.draw();
}

void FFKinectMeshManager::drawMesh(bool faced){
    if(faced){
        mesh.draw();
    }else{
        mesh.drawFaces();
    }
}

void FFKinectMeshManager::processKinectV1Data(){
    kinectV1.update();
  //  positionLights();
    if (kinectV1.isFrameNew()) {
        kinectUtils.processKinectData();
        int w = 640;
        int h = 480;
        ofShortPixels  pix = kinectV1.getRawDepthPixels();
        if(activateSmooth){
            for(int i = 0; i < smoothCount; i++ ){
                smoothArray(pix);
            }
            if(temporalSmoothing != 0.0){
                for(int i = 0; i < h*w; i++){
                    int diff =pix[i] - kinectDepth[i];
                    int pixd = pix[i];
                    
                    if(abs(diff) > 90){
                        kinectDepth[i] = pixd;
                    }
                    kinectDepth[i] = kinectDepth[i] * temporalSmoothing + pixd * (1.0-temporalSmoothing);
                    pix[i] = kinectDepth[i];
                }
            }
        }
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
                float distance = kinectV1.getDistanceAt(x, y);
                if(isRGBMapActive){
                    ofColor vertexColor = kinectV1.getColorAt(x, y);
                    mesh.addColor(vertexColor);
                }else{
                    mesh.addColor(ofColor::white);
                }
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
        calcNormals(mesh);
        //Fatten algorithm
        for (int i = 0; i < mesh.getIndices().size(); i++) {
            const int ia = mesh.getIndices()[i];
            if (ia < mesh.getVertices().size() ) {
                //ofVec3f e1 = mesh.getVertices()[ia];
                ofVec3f norml = mesh.getNormals()[ia];
                float hello = sqrt(4);
                float l = sqrt(norml[0]*norml[0]+norml[1]*norml[1]+norml[2]*norml[2]);
                if (l != 0.0){
                    mesh.getVertices()[ia] = mesh.getVertices()[ia] + (norml*3*fatten)/l;
                }
            }
        }
    }
}

void FFKinectMeshManager::processKinectV2Data(){
    kinectV2.update();
    int step = meshResolution;
    int total = 0;
    int h = kinectV2.getDepthPixelsRef().getHeight();
    int w = kinectV2.getDepthPixelsRef().getWidth();
    
    if (kinectV2.isFrameNew()) {
        if (kinectFrameLimiter >= 0) {
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
                        float distance = kinectV2.getDistanceAt(i, j);
                        float previous_distance;
                        float next_distance;
                        float up_distance;
                        float down_distance;
                        float z_difference;
                        if(i>0){
                            previous_distance = kinectV2.getDistanceAt(i-1, j);
                            next_distance = kinectV2.getDistanceAt(i+1, j);
                            if(j>0){
                                up_distance = kinectV2.getDistanceAt(i, j);
                                down_distance = kinectV2.getDistanceAt(i-1, j);
                                z_difference= (std::abs(distance-previous_distance) + std::abs(distance-next_distance) + std::abs(distance-up_distance)+std::abs(distance-down_distance))/4;
                            }else{
                                z_difference= (std::abs(distance-previous_distance) + std::abs(distance-next_distance))/2;
                            }
                        }
                        if (distance > nearThreshold && distance < farThreshold) {
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

void FFKinectMeshManager::calcNormals(ofMesh &mesh) {
    for (int i = 0; i < mesh.getVertices().size(); i++)
        mesh.addNormal(ofPoint(0, 0, 0));
    
    for (int i = 0; i < mesh.getIndices().size(); i += 3) {
        const int ia = mesh.getIndices()[i];
        const int ib = mesh.getIndices()[i + 1];
        const int ic = mesh.getIndices()[i + 2];
        if (ia >= mesh.getVertices().size() || ib >= mesh.getVertices().size() ||
            ic >= mesh.getVertices().size()) {
            return;
        }
        ofVec3f e1 = mesh.getVertices()[ia] - mesh.getVertices()[ib];
        ofVec3f e2 = mesh.getVertices()[ic] - mesh.getVertices()[ib];
        ofVec3f no = e1.cross(e2);
        
        mesh.getNormals()[ia] += no;
        mesh.getNormals()[ib] += no;
        mesh.getNormals()[ic] += no;
    }
}

void FFKinectMeshManager::smoothArray(ofShortPixels &pix ){
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

void FFKinectMeshManager::changeMeshMode(int &meshSelector){
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

void FFKinectMeshManager::changeMeshType(int &meshTypeSelector){
    if (meshType == 1){meshMode = 1;};
    if (meshType == 2){meshMode = 3;};
    if (meshType == 3){meshMode = 3;};
}

void FFKinectMeshManager::setDepthSmoothingActive(bool &val){
    kinectUtils.setDepthSmoothingActive(val);
}

void FFKinectMeshManager::setNearThreshold(float &val){
    kinectUtils.setNearThreshold(val);
}

void FFKinectMeshManager::setFarThreshold(float &val){
    kinectUtils.setFarThreshold(val);
}

void FFKinectMeshManager::setMeshBlurRadius(float &val){
    kinectUtils.setMeshBlurRadius(val);
}

void FFKinectMeshManager::setZAveragingMaxDepth(float &val){
    kinectUtils.setZAveragingMaxDepth(val);
}

void FFKinectMeshManager::setBlankDepthPixMax(float &val){
    kinectUtils.setBlankDepthPixMax(val);
}
