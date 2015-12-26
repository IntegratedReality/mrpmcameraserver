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

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

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
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    homography.movingPoint = false;
}
//--------------------------------------------------------------

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
    for(int i = 0; i < points.size(); i++) {
        if(distance(points[i],point) < 60) {    //本当はmargin分の調整をすべき(めんどくさい)
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
