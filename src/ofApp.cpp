#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

/* スタティック変数の定義 */
int markerInfo::pointSet = 0;
int markerInfo::selected = 0;
bool markerInfo::drawing = false;
ofVec2f markerInfo::mouse_position;

/* network configuration */
const char *address = "Coconuts.local";
const int port = 8000;

//--------------------------------------------------------------
void ofApp::setup(){
    ofBackground(50, 50, 50);
    
    myCam.setDeviceID(0);
    myCam.setup(camwidth, camheight);
    ofSetVerticalSync(false);
    ofSetCircleResolution(8);
    
    /* load font */
    font.load("Arial.ttf",18);
    
    /* calibrate cam */
    calib.calibration.setFillFrame(true); // true by default
    calib.calibration.load("calibration.yml");
    ofxCv::imitate(calib.undistorted, myCam);
    
    /* allocate ofImages */
    improcess.bin.allocate(camwidth, camheight, OF_IMAGE_GRAYSCALE);

    improcess.pixels_origin = calib.undistorted.getPixels().getData();
    improcess.camTexture = calib.undistorted.getTexture();
    improcess.pixels_bin = improcess.bin.getPixels().getData();
    improcess.binTexture = improcess.bin.getTexture();
    
    /* initialize FBOs */
    improcess.camFbo.allocate(camwidth,camheight,GL_RGB);
    improcess.camFbo.begin();
    {
        ofClear(255, 255, 255);
    }
    improcess.camFbo.end();
    improcess.binFbo.allocate(camwidth,camheight,GL_RGBA,4);
    improcess.binFbo.begin();
    {
        ofClear(255,255,255,0);
    }
    improcess.camFbo.end();
    
    improcess.stringFbo.allocate(camwidth,camheight * 2,GL_RGB,4);
    improcess.stringFbo.begin();
    {
        ofClear(0,0,0,0);
    }
    improcess.stringFbo.end();
    
    /* initialize OSC */
    oscSender.init(address,port);
    oscSender.start = std::chrono::system_clock::now(); //initialize time stamp
    
}

//--------------------------------------------------------------
void ofApp::update(){
    myCam.update();
    
    if (myCam.isFrameNew()){
        
        calib.calibration.undistort(ofxCv::toCv(myCam), ofxCv::toCv(calib.undistorted));
        calib.undistorted.update();
        
        for (int i = 0 ; i < camwidth; i++){
            for (int j = 0; j < camheight; j++){
                
                /* グレースケール化 */
                improcess.red = improcess.pixels_origin[j*3 * camwidth + i * 3];
                if (improcess.red > bin_threshold){
                    improcess.pixels_bin[j*camwidth + i] = 255;
                }
                else{
                    improcess.pixels_bin[j* camwidth + i] = 0;
                }
            }
        }

        /***** labeling~ *****/
        if (improcess.filter_intensity != 0){
            for (int i = 0; i < improcess.filter_intensity; i++){
                ofxCv::erode(improcess.bin);
            }
            for (int i = 0; i < improcess.filter_intensity; i++){
                ofxCv::dilate(improcess.bin);
            }
        }
        improcess.bin_mat = ofxCv::toCv(improcess.bin);
        improcess.num = labeling.labeling(improcess.bin_mat, improcess.labels);       //ラベリング実行
        
        /* 各ラベルの重心を求める */
        
        /* 初期化(前回埋めたところだけ消去して節約) */
        if (improcess.previous_num >= region){  //下で初期化するときに変な領域に入らないように制限
            improcess.previous_num = region - 1;    //最大値を超えた場合は最大値に設定
        }
        for (int i = 0; i <= improcess.previous_num; i++){
            improcess.center_point[i] = ofVec3f(0,0,0);     //重心を入れる配列をリセット
        }
        improcess.previous_num = improcess.num;
        /* ~初期化終了 */
        
        int region_number;      //ラベル番号を一時的に保存
        for (int i = 0; i < improcess.labels.rows; i++){
            for (int j = 0; j < improcess.labels.cols; j++){
                region_number = improcess.labels(i,j);          //画素アクセスの回数を減らすために退避
                if(region_number != 0){
                improcess.center_point[region_number].x += j;
                improcess.center_point[region_number].y += i;
                improcess.center_point[region_number].z += 1;   //足した回数を記憶(後で割る)
                }
            }
        }
        for (int i = 1; i < improcess.num; i++){
            if (improcess.center_point[i].z > min_region){    //簡易的なローパスフィルタ(小さい画素は無視)
                improcess.center_point[i].x /= (improcess.center_point[i].z);     // 重心を求めるために割り算
                improcess.center_point[i].y /= (improcess.center_point[i].z);
            }
            else {
                improcess.center_point[i].z = 0;    //アクティブでないものを見分ける(z = 0 : 非アクティブ)
            }
        }
        
        /***** ~labeling *****/
        
        /***** homography *****/
        if(homography.ready){
            for (int i = 1; i < improcess.num; i++){
                if (improcess.center_point[i].z != 0){
                    homography.executeTransform(improcess.center_point[i]);
                }
            }
        }
        
        /***** information about markers *****/
        oscSender.currentTime = std::chrono::system_clock::now();
        oscSender.elapsedTime = chrono::duration_cast<chrono::milliseconds>(oscSender.currentTime - oscSender.start);
        oscSender.timeStamp = static_cast<uint32_t>(oscSender.elapsedTime.count()*1000);
        
        for (int i = 0; i < 6; i++){
            if (marker[i].active){
                marker[i].update(improcess.center_point, improcess.num);    //update markers
                marker[i].calcNormalizedPoint(improcess.usingArea);  //convert to mm scale
                oscSender.sendData(i, oscSender.timeStamp, marker[i].normalized_point.x, marker[i].normalized_point.y, marker[i].angle);
            }
        }
        improcess.bin.update();
        
        camFps.getFps(oscSender.elapsedTime.count());    //カメラの実際のfpsを出力
    }
    
    /* fbo(GPU)による描画処理 */
    
    if (improcess.showImage){
        improcess.camFbo.begin();
        {
            improcess.camTexture.draw(improcess.homographyCorner[0], improcess.homographyCorner[1], improcess.homographyCorner[2], improcess.homographyCorner[3]);
            /* 使用範囲を描画 */
            if (improcess.usingAreaDecided){
                ofPushStyle();
                ofNoFill();
                ofSetColor(230, 80, 80);
                ofDrawRectangle(improcess.usingArea[0].x, improcess.usingArea[0].y, improcess.usingArea[1].x - improcess.usingArea[0].x, improcess.usingArea[1].y - improcess.usingArea[0].y);
                ofPopStyle();
            }
            else if (improcess.setCoordToggle){
                ofPushStyle();
                ofFill();
                ofSetColor(230,130,130);
                ofDrawCircle(improcess.usingArea[0],5);
                ofPopStyle();
            }
            /* ホモグラフィの基準点を描画 */
            if (!homography.srcPoints.empty()){
                homography.drawPoints(homography.srcPoints);
            }
        }
        improcess.camFbo.end();
    }
    
    //int region_number;      //ラベル番号を一時的に保存
    improcess.binFbo.begin();
    {
        improcess.binTexture.draw(improcess.homographyCorner[0], improcess.homographyCorner[1], improcess.homographyCorner[2], improcess.homographyCorner[3]);
        labeling.drawRegions(improcess.center_point,improcess.num);
        
        /* マーカーの初期化領域を描画 */
        if (markerInfo::drawing){
            for (int i = 0; i < 8; i++){
                if (marker[i].marker_initializing == true){
                    marker[i].drawRegion();
                }
            }
        }
        
        /* 認識されているマーカーをオーバーレイ */
        for (int i = 0; i < 9; i++){
            if (marker[i].active){
                marker[i].showMarker();
                marker[i].highlightFront();
            }
        }
    }
    improcess.binFbo.end();
    
    improcess.stringFbo.begin();
    {
        /* 座標を出力 */
        ofClear(0,0,0,0);
        ofSetColor(255, 255, 255);
        for (int i = 0; i < 8; i++){
            if (marker[i].active){
                marker[i].pointStr = "marker[" + ofToString(i) + "] : (" + ofToString(marker[i].normalized_point.x) + ", " + ofToString(marker[i].normalized_point.y) + ") , angle : " + ofToString(marker[i].angle);
                font.drawString(marker[i].pointStr, 30, camheight+ 20*i);
            }
        }
        
        ofPushStyle();
        ofSetColor(0, 120, 200);
        font.drawString("MRPM IRtracker", 30, 50);
        ofPopStyle();
        
        /* fps書き出し */
        improcess.camFpsString = "fps : " + ofToString(camFps.normalFps) + "       cam fps : " + ofToString(camFps.fps);
        font.drawString(improcess.camFpsString,300,110);
        
        /* filter強度 */
        improcess.filter_info = "filter intensity : " + ofToString(improcess.filter_intensity);
        font.drawString(improcess.filter_info, 30, 110);
        
        /* 選択中のマーカー */
        improcess.selectedMarker = "selected marker : " + ofToString(markerInfo::selected);
        font.drawString(improcess.selectedMarker,30,150);
        
        /* 正規化エリア */
        improcess.normalizedArea1 = "normalized point : (" + ofToString(improcess.usingArea[0].x) + ", " + ofToString(improcess.usingArea[0].y) + ")";
        improcess.normalizedArea2 = "normalized point : (" + ofToString(improcess.usingArea[1].x) + ", " + ofToString(improcess.usingArea[1].y) + ")";
        font.drawString(improcess.normalizedArea1, 30, 280);
        font.drawString(improcess.normalizedArea2, 30, 320);
        if (improcess.setCoord){
            ofPushStyle();
            ofSetColor(200, 50, 50);
            font.drawString("SETTING NORMALIZED AREA (" + ofToString(improcess.usingAreaConfig) + ")", 30, 200);
            ofPopStyle();
        }
        
        /* 重心座標を描写・書き出し */
//        improcess.writePoints();
    }
    improcess.stringFbo.end();
}

//--------------------------------------------------------------
void ofApp::draw(){
    /* 映像の描写 */
    if (improcess.showImage){
        improcess.camFbo.draw(cam_margin,cam_margin);
        improcess.binFbo.draw(cam_margin,cam_margin + camheight);
    }
    improcess.stringFbo.draw(cam_margin + camwidth,cam_margin);
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
    if (key == 's'){
        if (improcess.showImage == true){
            improcess.showImage = false;
        }
        else {
            improcess.showImage = true;
        }
    }
    
    if (key == ' '){
        improcess.setCoord = !improcess.setCoord;
        cout << "set coord" << endl;
    }
    
    if (key == 'r'){
        oscSender.start = std::chrono::system_clock::now();
    }
    
    /* 先頭選択用 */
    if (!improcess.setCoord){
        int num = key - 48;
        if (num >= 0 && num < 6){
            marker[num].marker_initializing = !marker[num].marker_initializing; //bool反転
            cout << "\ninitializing marker[" << num << "]" << endl;
            for (int i = 0; i < 8; i ++){
                if (marker[i].marker_initializing == true && i != num){     //連続で別の番号を押した時のための処理
                    marker[i].marker_initializing = false;
                }
            }
        }
        
        if (key == OF_KEY_RIGHT){
            markerInfo::selected++;
            if (markerInfo::selected == 8) marker[0].selected = 0;
        }
        if (key == OF_KEY_LEFT){
            markerInfo::selected--;
            if (markerInfo::selected == -1) marker[0].selected = 7;
        }
        if (key == OF_KEY_RETURN){
            marker[markerInfo::selected].front++;
            if (marker[markerInfo::selected].front == 3){
                marker[markerInfo::selected].front = 0;
            }
        }
        
        if (key == OF_KEY_UP){
            improcess.filter_intensity++;
        }
        if (key == OF_KEY_DOWN && (improcess.filter_intensity != 0)){
            improcess.filter_intensity--;
        }
    }
    else{
        if (key == 49){
            improcess.usingAreaConfig = false;
            cout << "setting left upper point" << endl;
        }
        else if (key == 50){
            improcess.usingAreaConfig = true;
            cout << "setting right bottom point" << endl;
        }
        
        if (!improcess.usingAreaConfig){
            switch (key) {
                case OF_KEY_UP:
                    improcess.usingArea[0].y -= 1;
                    break;
                case OF_KEY_DOWN:
                    improcess.usingArea[0].y += 1;
                    break;
                case OF_KEY_LEFT:
                    improcess.usingArea[0].x -= 1;
                    break;
                case OF_KEY_RIGHT:
                    improcess.usingArea[0].x += 1;
                    break;
                
                default:
                    break;
            }
        }
        else if (improcess.usingAreaConfig){
            switch (key) {
                case OF_KEY_UP:
                    improcess.usingArea[1].y -= 1;
                    break;
                case OF_KEY_DOWN:
                    improcess.usingArea[1].y += 1;
                    break;
                case OF_KEY_LEFT:
                    improcess.usingArea[1].x -= 1;
                    break;
                case OF_KEY_RIGHT:
                    improcess.usingArea[1].x += 1;
                    break;
                    
                default:
                    break;
            }
        }
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    if (markerInfo::drawing){
        markerInfo::mouse_position.x = x;
        markerInfo::mouse_position.y = y;
    }
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    if(homography.movingPoint) {
        homography.curPoint->x = x - cam_margin;
        homography.curPoint->y = y - cam_margin;
        
        homography.homographyMat = cv::findHomography(homography.srcPoints, homography.warpedPoints);
    }
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    cv::Point2f cur(x , y);
    if (!homography.movePoint(homography.srcPoints,cur) && y < cam_margin + camheight + 10 && !improcess.setCoord){    //10は念のために余分に取っただけ
        if (homography.srcPoints.size() < 4){   //4点未満の時しかpush_backはしない
            cv::Point2f cur(x - cam_margin, y - cam_margin);
            homography.srcPoints.push_back(cur);
        }

        if (homography.first && static_cast<int>(homography.srcPoints.size()) == 4){     //set destination points
            /* 左上から反時計回りに */
            homography.warpedPoints.push_back(cv::Point2f(0,10));
            homography.warpedPoints.push_back(cv::Point2f(0,10) + cv::Point2f(0,426));
            homography.warpedPoints.push_back(cv::Point2f(0,10) + cv::Point2f(639,426));
            homography.warpedPoints.push_back(cv::Point2f(0,10) + cv::Point2f(639,0));
            
            homography.homographyMat = cv::getPerspectiveTransform(homography.srcPoints, homography.warpedPoints);
            //変換行列の計算はループから外し、変更するときのみ更新する
            
            //入力映像の変換用
            improcess.camFbo.begin();
                ofClear(0, 0, 0);
            improcess.camFbo.end();
            improcess.binFbo.begin();
                ofClear(0,0,0);
            improcess.binFbo.end();
            for (int i = 0; i < 4; i++){
                homography.executeTransform(improcess.homographyCorner[i]);
                improcess.homographyCorner[i].z = 0;
            }
            
            homography.ready = true;     //ホモグラフィ完了フラグ
            homography.first = false;      //skip after the first loop
        }
    }
    /* マーカー認識領域の設定 */
    /* 長方形領域の二点(init_region[])を指定する、pointSet(staticメンバ)が一時的なインデックスになる */
    for (int i = 0; i < 8; i++){
        if (marker[i].marker_initializing == true){
            markerInfo::drawing = true;
            /* 画像上の座標に変換(右辺) */
            marker[i].init_region[markerInfo::pointSet] = ofVec2f(x - cam_margin,y - cam_margin - camheight);
            markerInfo::mouse_position = ofVec2f(x + 1, y + 1); //領域選択の際に指定する二点目を仮に入れておく
            if (markerInfo::pointSet == 1){
                marker[i].active = true;
                marker[i].init(improcess.center_point);
                markerInfo::pointSet = 0; //1の次は0に戻しておく(0 or 1 の２つ)
                marker[i].marker_initializing = false;  //設定終了
                markerInfo::drawing = false;
                break;
            }
            markerInfo::pointSet++;
            break;
        }
    }
    
    if (improcess.setCoord){
        if (!improcess.setCoordToggle){
            improcess.usingArea[0] = ofVec2f(x - cam_margin,y - cam_margin);
            improcess.setCoordToggle = true;
        }
        else {
            improcess.usingArea[1] = ofVec2f(x - cam_margin,y - cam_margin);
            improcess.setCoordToggle = false;
            improcess.setCoord = false;
            improcess.usingAreaDecided = true;
            improcess.areaSize.x = improcess.usingArea[1].x - improcess.usingArea[0].x;
            improcess.areaSize.y = improcess.usingArea[1].y - improcess.usingArea[0].y;
        }
    }
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    if (homography.movingPoint && homography.ready){
        homography.movingPoint = false;
        
        improcess.homographyCorner[0] = ofVec3f(0,0,0);
        improcess.homographyCorner[1] = ofVec3f(camwidth,0,0);
        improcess.homographyCorner[2] = ofVec3f(camwidth,camheight,0);
        improcess.homographyCorner[3] = ofVec3f(0,camheight,0);
        improcess.camFbo.begin();
            ofClear(0, 0, 0);
        improcess.camFbo.end();
        for (int i = 0; i < 4; i++){
            homography.executeTransform(improcess.homographyCorner[i]);
            improcess.homographyCorner[i].z = 0;
        }
    }
}
//--------------------------------------------------------------

void markerInfo::init(ofVec3f *markerPoints){    //個体を認識するため、3つの点が含まれる領域を設定
    /* 長方形の領域を設定する。引数は左上と右下の二点の座標 markerPointsは全てのledの座標(3×8個になるはず) */
    int counter = 0;    //検出された個数を保持
    for (int i = 1; i < region; i++){   //最大数までループ
        if (markerPoints[i].z < min_region){
            /* 上と同様min_region以下の大きさの領域は無視する */
            continue;
        }
        /* 領域内の点をpoint[3]に書き込む */
        if (markerPoints[i].x > init_region[0].x && markerPoints[i].x < init_region[1].x && markerPoints[i].y > init_region[0].y && markerPoints[i].y < init_region[1].y){
            point[counter] = markerPoints[i];
            cout << "point[" << counter << "]" << endl;
            counter++;
            cout << "markerPoints (x,y,z) : " << markerPoints[i].x << ", " << markerPoints[i].y << ", " << markerPoints[i].z << endl;
            
        }
        if (counter > 3){
            cout << "error : too many points" << endl;
            break;
        }
    }
    if (counter < 3) {
        cout << "error : too few points" << endl;
        for (int i = counter ; i < 3; i++){
            point[i] = ofVec2f(0,0);    //細かいエラー処理は後で追加
            active = false;
        }
    }
    cout << "init_region[0] (x,y) = (" << init_region[0].x << ", " << init_region[0].y <<")" << endl;
    cout << "init_region[1] (x,y) = (" << init_region[1].x << ", " << init_region[1].y <<")" << endl;

    marker_initializing = false;
}

void markerInfo::drawRegion(){
    ofSetColor(30,30,150,70);
    ofFill();
    ofDrawRectangle(init_region[0].x, init_region[0].y, mouse_position.x - init_region[0].x - cam_margin, mouse_position.y - init_region[0].y - camheight - cam_margin);
    ofSetColor(255);
    ofNoFill();
}

void markerInfo::update(ofVec3f *markerPoints, int array_length){
    int min_index = 1;
    double min_dif,dist;    //両方一時変数
//    for (int i = 0; i < 3; i++){
//        prev_point[i] = point[i];
//        
//        /* 一番近い点を探す */
//        min_index = 1;
//        min_dif = distance(point[i], markerPoints[1]);  //markerPointsは1以降が有効な座標(ラベリングの仕様)
//        for (int j = 2; j < array_length; j++){
//            if (markerPoints[j].z == 0) continue;
//            dist = distance(point[i], markerPoints[j]);
//            //cout << "dif : " << dist <<endl;
//            if (dist < min_dif && dist < max_velocity){ //max_velocityより進んでいる場合は間違いなので含めない
//                min_dif = dist;
//                min_index = j;
//                /*
//                //本来最初の値を最小として初期化する必要はない
//                //一個もmax_velocityの範囲内の点が見つからないならここでエラー処理して次のフレームに回すべき(後で実装)
//                */
//            }
//        }
//        point[i] = markerPoints[min_index];
//    }
    
    /* 新バージョン */
    for (int i = 0; i < 3; i++){
        bool exist = false;
        for (int j = 1; j < array_length; j++){
            if (markerPoints[j].z == 0){
                /* アクティブでない領域は無視 */
                continue;
            }
            dist = distance(point[i], markerPoints[j]);
            if (dist < max_velocity){
                if (!exist){
                    /* 条件に合うものが初めて見つかった時 */     //条件に合うものが見つかった時点で、それだと確定するマーカーのサイズ設計が理想？？
                    exist = true;
                    min_dif = dist;
                    min_index = j;
                }
                else if(exist && dist < min_dif) {
                    /* より近い物が見つかった時 */
                    min_dif = dist;
                    min_index = j;
                }
            }
        }
        if (exist){
            point[i] = markerPoints[min_index];
            prev_point[i] = point[i];
        }
        else {
            point[i] = prev_point[i];  //見つからなかった場合(前と同じ位置とする　本当は他のマーカーの進んだベクトル分足したい)
            cout << "not found" << endl;
        }
        /* debug */
        int dist[i];
        dist[0] = distance(point[0],point[1]);
        dist[1] = distance(point[1],point[2]);
        dist[2] = distance(point[2],point[0]);
        for (int i = 0; i < 3; i++){
            if (dist[i] < 1){
                cout << "** marker duplicated **" << endl;
            }
        }
    }
    
    calcCenter();  //重心と角度も更新
    calcAngle();
}

void markerInfo::showMarker(){
    ofSetColor(50, 130, 250);
    /* 表示位置の関係で色々足している */
    //ofDrawTriangle(point[0].x + cam_margin,point[0].y + cam_margin + camheight , point[1].x + cam_margin, point[1].y + cam_margin + camheight, point[2].x + cam_margin, point[2].y + cam_margin + camheight);
    ofDrawCircle(marker_center.x, marker_center.y, 18);
    ofDrawCircle(marker_center.x, marker_center.y, 9);
    ofSetColor(200, 255, 255);
    ofDrawLine(ofVec3f(marker_center.x, marker_center.y,0), ofVec3f(point[front].x, point[front].y,0));
    
    ofFill();
    ofSetColor(180,180,180,80);
    ofDrawCircle(marker_center.x, marker_center.y, 18);
    ofNoFill();
    ofSetColor(255);
}
    
void markerInfo::highlightFront(){
    if(active){
        ofFill();
        ofSetColor(100,100,230);
        ofDrawCircle(point[front],10);
        ofSetColor(255, 255, 255);
        ofNoFill();
    }
}

void homographyClass::drawPoints(vector<cv::Point2f>& points) {
    ofNoFill();
    ofPushStyle();
    ofSetColor(200, 10, 10);
    for(int i = 0; i < points.size(); i++) {
        ofDrawCircle(points[i].x,points[i].y, 10);
        ofDrawCircle(points[i].x,points[i].y, 1);
    }
    ofPopStyle();
}

bool homographyClass::movePoint(vector<cv::Point2f>& points, cv::Point2f point) {
    point.x -= cam_margin;
    point.y -= cam_margin;
    for(int i = 0; i < points.size(); i++) {
        if(distance(points[i],point) < 60) {
            movingPoint = true;
            curPoint = &points[i];
            return true;
        }
    }
    return false;
}


int labelingClass::labeling(const cv::Mat& image,cv::Mat_<int>& label){
    const int W=image.cols;
    const int H=image.rows;
        
    int index=0;
    for(int j=0;j<H;j++){
        for(int i=0;i<W;i++)
        {
            //隣接画素（４近傍）との連結チェック
            unsigned int c=getAt(image,i,j);
            bool flagA=(isIn(W,H,i-1,j  ) && c==getAt(image,i-1,j  )); //左
            bool flagB=(isIn(W,H,i  ,j-1) && c==getAt(image,i  ,j-1)); //上
            
            //着目画素と連結画素を併合
            label(j,i)=index;
            if((flagA|flagB)==true)
            {
                parents.push_back(index);
                if(flagA) label(j,i)=link(parents,label(j,i),label(j  ,i-1));
                if(flagB) label(j,i)=link(parents,label(j,i),label(j-1,i  ));
                parents.pop_back();
            }
            else{
                /* 点が多すぎる時のエラー処理(必要そうなら入れる) */
//                if (index > BUF_LABEL - 50){
//                    cout << "labeling error" << endl;
//                    return 0;
//                }
                parents.push_back(index++);
            }
        }
    }

    //再ラベリング
    int regions=relabel(parents);
    for(int j=0;j<H;j++){
        for(int i=0;i<W;i++){
            label(j,i)=parents[label(j,i)];
        }
    }
    parents.clear();    //関数外で定義してるのでリセットしないと要素が増え続けてしまう、vectorのclear()はデータを消すだけでメモリは開放しない
    return regions;
}

void labelingClass::drawRegions(ofVec3f* center_points, int num){
    ofFill();
    ofPushStyle();
    
    ofSetColor(100, 100, 230);
    for (int i = 0; i < num; i++){
        if (center_points[i].z != 0){    //アクティブでない画素を無視
            ofDrawCircle(center_points[i].x, center_points[i].y, 5);
        }
    }
    
    ofPopStyle();
    ofNoFill();
}

void imageProcess::writePoints(){      //ラベリング後に重心を求め、それを表示する
    string position;
    int limit;
    int counter = 0;
    if (num > 15){
        limit = 15;
    }
    else {
        limit = num-1;
    }
    
    int i = 1;
    while (counter < limit && i < region-10){
        if (center_point[i].z != 0){
            position = ofToString(i) + " : ";
            position += ofToString(center_point[i]);
            ofDrawBitmapString(position,30 , 15 * counter + 20 );
            counter++;
        }
        i++;
    }
    ofSetColor(255);
}

void cameraFps::getFps(double elapsedTime){
    currentTime = elapsedTime - static_cast<int>(elapsedTime);
    if (currentTime < previousTime){
        fps = frameCounter;
        normalFps = ofGetFrameRate();
        frameCounter = 0;
    }
    else {
        frameCounter++;
    }
    previousTime = currentTime;
}


void simulatorClass::markerGen(ofVec2f center){
    drawCube(ofVec2f(center.x + 5, center.y));
    drawCube(ofVec2f(center.x - 10, center.y + 5));
    drawCube(ofVec2f(center.x - 10, center.y - 5));
}
