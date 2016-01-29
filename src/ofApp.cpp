#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofBackground(50, 50, 50);
    
    myCam.initGrabber(camwidth, camheight);
    
    /* ofImageのallocate */
    improcess.grayImage.allocate(camwidth, camheight, OF_IMAGE_GRAYSCALE);
    improcess.bin.allocate(camwidth, camheight, OF_IMAGE_GRAYSCALE);
    
    improcess.pixels_origin = myCam.getPixels();
    improcess.pixels_gray = improcess.grayImage.getPixels();
    improcess.pixels_bin = improcess.bin.getPixels();
}

//--------------------------------------------------------------
void ofApp::update(){
    myCam.update();
    
    if (myCam.isFrameNew()){
        for (int i = 0 ; i < camwidth; i++){
            for (int j = 0; j < camheight; j++){
                
                /* グレースケール化 */
                improcess.red = improcess.pixels_origin[j*3 * camwidth + i * 3];
                improcess.green = improcess.pixels_origin[j*3 * camwidth + i * 3 + 1];
                improcess.blue = improcess.pixels_origin[j*3 * camwidth + i * 3 + 2];
                //gray = (11*red + 16*green + 5*blue)/32;
                improcess.gray = (improcess.red + improcess.green + improcess.blue) / 3;
                
                improcess.pixels_gray[j* camwidth + i] = improcess.gray;
                
                /* 二値化 */
            
                if (improcess.pixels_gray[j* camwidth + i] > 250){
                    improcess.pixels_bin[j* camwidth + i] = 255;
                }
                else{
                    improcess.pixels_bin[j* camwidth + i] = 0;
                }
            }
        }
        
        /***** labeling~ *****/
        improcess.bin_mat = ofxCv::toCv(improcess.bin);
        improcess.num = labeling.labeling(improcess.bin_mat, improcess.labels);       //ラベリング実行
        
        /* 各ラベルの重心を求める */
//        for (int i = 0; i < improcess.labels.rows; i++){
//            for (int j = 0; j < improcess.labels.cols; j++){
//                improcess.center_point[improcess.labels(i,j)] = ofVec3f(0,0,0);     //重心を入れる配列をリセット(改善の余地あり？)
//            }
//        } //間違ってたやつ(一応残してる)
        
        /* 初期化(前回埋めたところだけ消去して節約) */
        if (improcess.previous_num >= region){  //下で初期化するときに変な領域に入らないように制限
            improcess.previous_num = region-1;
        }
        for (int i = 0; i <= improcess.previous_num; i++){
            improcess.center_point[i] = ofVec3f(0,0,0);     //重心を入れる配列をリセット
        }
        improcess.previous_num = improcess.num;
        /* 初期化終了 */
        
        int region_number;      //ラベル番号を一時的に保存
        for (int i = 0; i < improcess.labels.rows; i++){
            for (int j = 0; j < improcess.labels.cols; j++){
                region_number = improcess.labels(i,j);          //画素アクセスの回数を減らすために退避した
                if(region_number != 0){
                improcess.center_point[region_number].x += j;
                improcess.center_point[region_number].y += i;
                improcess.center_point[region_number].z += 1;   //足した回数を記憶(後で割る)
                }
            }
        }
        for (int i = 1; i < improcess.num; i++){
            if (improcess.center_point[i].z > 10){    //簡易的なローパスフィルタ(小さい画素は無視)
                improcess.center_point[i].x /= (improcess.center_point[i].z + 1);     // 重心を求めるために割り算  +1は0から足した分を補正している
                improcess.center_point[i].y /= (improcess.center_point[i].z + 1);
            }
            else {
                improcess.center_point[i].z = 0;    //アクティブでないものを見分ける(z = 0 : 非アクティブ)
            }
        }
        
        /***** ~labeling *****/
        
        /***** homography *****/
        if(homography.homographyReady){
            for (int i = 1; i < improcess.num; i++){
                if (improcess.center_point[i].z != 0){
                    homography.executeTransform(improcess.center_point[i]);
                }
            }
        }
        improcess.grayImage.update();
        improcess.bin.update();
    }
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    /* 映像の描写 */
    myCam.draw(cam_margin,cam_margin);
    improcess.grayImage.draw(cam_margin + camwidth, cam_margin);
    improcess.bin.draw(cam_margin, cam_margin + camheight);
    
    /* 重心座標を描写・書き出し */
    labeling.drawRegions(improcess.center_point,improcess.num);
    improcess.writePoints();
    
    /* ホモグラフィの基準点を描写 */
    if (!homography.srcPoints.empty()){
        homography.drawPoints(homography.srcPoints);
    }
    /* fps書き出し */
    double fps = ofGetFrameRate();
    string fpsString = "fps : " + ofToString(fps);
    ofDrawBitmapString(fpsString, 10, 10);
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if (key > 0 && key < 9){    //機体の数なので1~8
        marker[key].marker_initializing = true;
        for (int i = 0; i < 8; i ++){
            if (marker[i].marker_initializing == true && i != key){     //連続で番号を押した時のための処理
                marker[i].marker_initializing = false;
            }
        }
    }
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    for (int i = 0; i < 8; i++){
        if (marker[i].marker_initializing == true){
            marker[i].mouse_position->x = x;
            marker[i].mouse_position->y = y;
            marker[i].drawRegion();
            break;
        }
    }
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    if(homography.movingPoint) {
        homography.curPoint->x = x - cam_margin;
        homography.curPoint->y = y - cam_margin;
        
        homography.homographyMat = cv::findHomography(cv::Mat(homography.srcPoints), cv::Mat(homography.warpedPoints));
    }
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    cv::Point2f cur(x , y);
    if (!homography.movePoint(homography.srcPoints,cur)){
        if (homography.srcPoints.size() < 4){   //4点未満の時しかpush_backはしない
            cv::Point2f cur(x - cam_margin, y - cam_margin);
            homography.srcPoints.push_back(cur);
        }
        if (homography.first && homography.srcPoints.size() == 4){     //set destination points
            
            homography.warpedPoints.push_back(cv::Point2f(50,50));
            homography.warpedPoints.push_back(cv::Point2f(50,50) + cv::Point2f(0,250));
            homography.warpedPoints.push_back(cv::Point2f(50,50) + cv::Point2f(250,250));
            homography.warpedPoints.push_back(cv::Point2f(50,50) + cv::Point2f(250,0));
            
            homography.homographyMat = cv::findHomography(cv::Mat(homography.srcPoints), cv::Mat(homography.warpedPoints));
            //変換行列の計算はループから外し、変更するときのみ更新する
            
            homography.homographyReady = true;     //ホモグラフィ完了フラグ
            homography.first = false;      //skip after the first loop
        }
    }
    for (int i = 0; i < 8; i++){
        if (marker[i].marker_initializing == true){
            marker[i].init_region[marker[i].pointSet] = ofVec2f(x - cam_margin,y - cam_margin - camheight);
            if (marker[i].pointSet == 1){
                marker[i].init(improcess.center_point);
                break;
            }
            marker[i].pointSet ++;
            break;
        }
    }
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    homography.movingPoint = false;
}
//--------------------------------------------------------------

void markerInfo::init( ofVec3f *markerPoints){    //個体を認識するため、3つの点が含まれる領域を設定
    /* 長方形の領域を設定する。引数は左上と右下の二点の座標 markerPointsは全てのledの座標(3×8個になるはず) */
    int counter = 0;    //検出された個数を保持
    /* 画像上の座標に変換 */
    for (int i = 0; i < sizeof(markerPoints) / sizeof(markerPoints[0]); i++){
        /* 領域内の点をpoint[3]に書き込む */
        if (markerPoints[i].x > init_region[0].x && markerPoints[i].x < init_region[1].x && markerPoints[i].y > init_region[0].y && markerPoints[i].y < init_region[1].y){
            point[counter] = markerPoints[i];
            counter ++;
        }
    }
    if (counter != 3){
        cout << "error" << endl;    //(仮)coutでエラー表示しておく
    }
    marker_initializing = false;
}
void markerInfo::drawRegion(){
    ofSetColor(30,30,150,70);
    ofFill();
    ofDrawRectangle(init_region[0].x, init_region[0].y, mouse_position->x - init_region[0].x, mouse_position->y - init_region[0].y);
    ofSetColor(255);
    ofNoFill();
}

void homographyClass::drawPoints(vector<cv::Point2f>& points) {
    ofNoFill();
    ofSetColor(200, 10, 10);
    for(int i = 0; i < points.size(); i++) {
        ofDrawCircle(points[i].x+cam_margin,points[i].y+cam_margin, 10);
        ofDrawCircle(points[i].x+cam_margin,points[i].y+cam_margin, 1);
    }
    ofSetColor(255);
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
                parents.push_back(index++);
            }
        }
    }
    //cout << parents.size() << endl;

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
    ofSetColor(130, 130, 230);
    ofFill();
    for (int i = 0; i < num; i++){
        if (center_points[i].z > 10){    //簡易的なローパスフィルタ(小さい画素は無視)
            ofDrawCircle(cam_margin + center_points[i].x, cam_margin + camheight + center_points[i].y, 5);
        }
    }
    ofSetColor(255, 255, 255);      //色をリセット(これをしないと画像に色が上書きされてしまう)
    ofNoFill();
}

void imageProcess::writePoints(){      //ラベリング後に重心を求め、それを表示する
    string position;
    ofSetColor(200, 200, 200);
    
    int limit;
    if (num > 15){
        limit = 15;
    }
    else {
        limit = num;
    }
    
    for (int i = 1; i < limit; i++){
        if (center_point[i].z != 0){
            position = ofToString(i) + " : ";
            position += ofToString(center_point[i]);
            ofDrawBitmapString(position,cam_margin + camwidth + 30 , cam_margin + camheight + 15 * i );
        }
    }
    ofSetColor(255);
}
