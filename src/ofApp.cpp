#include "ofApp.h"

/* スタティック変数の定義 */
int markerInfo::pointSet = 0;
int markerInfo::selected = 0;
bool markerInfo::drawing = false;
ofVec2f markerInfo::mouse_position;

//--------------------------------------------------------------
void ofApp::setup(){
    ofBackground(50, 50, 50);
    
    myCam.setDeviceID(0);
    myCam.initGrabber(camwidth, camheight);
    ofSetFrameRate(40);
    ofSetVerticalSync(true);
    ofSetCircleResolution(12);
    
    /* ofImageのallocate */
    improcess.bin.allocate(camwidth, camheight, OF_IMAGE_GRAYSCALE);
    
    improcess.pixels_origin = myCam.getPixels();
    improcess.camTexture = myCam.getTexture();
    improcess.pixels_bin = improcess.bin.getPixels();
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
}

//--------------------------------------------------------------
void ofApp::update(){
    myCam.update();
    
    if (myCam.isFrameNew()){
        for (int i = 0 ; i < camwidth; i++){
            for (int j = 0; j < camheight; j++){
                
                /* グレースケール化 */
                improcess.red = improcess.pixels_origin[j*3 * camwidth + i * 3];    //どうせ欲しいのは赤外なので赤だけで良い？
//                improcess.green = improcess.pixels_origin[j*3 * camwidth + i * 3 + 1];
//                improcess.blue = improcess.pixels_origin[j*3 * camwidth + i * 3 + 2];
                
                /* 二値化 */
//                if ((improcess.red + improcess.green + improcess.blue) / 3 > 245){
//                    improcess.pixels_bin[j* camwidth + i] = 255;
//                }
                if (improcess.red > 245){
                    improcess.pixels_bin[j*camwidth + i] = 255;
                }
                else{
                    improcess.pixels_bin[j* camwidth + i] = 0;
                }
            }
        }

        /***** labeling~ *****/
        if (improcess.filter_intensity != 0){
            ofxCv::erode(improcess.bin,improcess.filter_intensity);
            ofxCv::dilate(improcess.bin, improcess.filter_intensity);
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
                improcess.center_point[i].x /= (improcess.center_point[i].z);     // 重心を求めるために割り算  +1は0から足した分を補正している
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
        for (int i = 0; i < 8; i++){
            if (marker[i].active){
                marker[i].update(improcess.center_point, improcess.num);
            }
        }
        improcess.bin.update();
    }
    
    /* fbo(GPU)による描画処理 */
    
    if (improcess.showImage){
        improcess.camFbo.begin();
        {
            improcess.camTexture.draw(improcess.homographyCorner[0], improcess.homographyCorner[1], improcess.homographyCorner[2], improcess.homographyCorner[3]);
            /* ホモグラフィの基準点を描写 */
            if (!homography.srcPoints.empty()){
                homography.drawPoints(homography.srcPoints);
            }
        }
        improcess.camFbo.end();
    }
    
    int region_number;      //ラベル番号を一時的に保存
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
                //cout << "show [" << i << "]" << endl;
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
                //cout << "marker[" << i << "] : (" << marker[i].marker_center.x << "," << marker[i].marker_center.y << ")" << endl;
                marker[i].pointStr = "marker[" + ofToString(i) + "] : (" + ofToString(marker[i].marker_center.x) + ", " + ofToString(marker[i].marker_center.y) + ") , angle : " + ofToString(marker[i].angle);
                ofDrawBitmapString(marker[i].pointStr, 30, camheight + 10 * i);
            }
        }
        
        string filter_info = "filter intensity : " + ofToString(improcess.filter_intensity);
        ofDrawBitmapString(filter_info, 330, 20);
        
        /* 重心座標を描写・書き出し */
        improcess.writePoints();
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
    
    /* fps書き出し */
    double fps = ofGetFrameRate();
    string fpsString = "fps : " + ofToString(fps);
    ofDrawBitmapString(fpsString, 10, 10);
    
    /* 選択中のマーカー */
    string selectedMarker = "selected marker : " + ofToString(markerInfo::selected);
    ofDrawBitmapString(selectedMarker, 150, 10);
    
//    cout << "length : " << sizeof(improcess.center_point) / sizeof(ofVec3f) << endl;
//    cout << "num : " << improcess.num << endl;
    
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    int num = key - 49;     //keyはアスキーコードなので、ずらしてやる
    if (num >= 0 && num < 8){
        marker[num].marker_initializing = !marker[num].marker_initializing; //bool反転
        cout << "\ninitializing marker[" << num << "]" << endl;
        for (int i = 0; i < 8; i ++){
            if (marker[i].marker_initializing == true && i != num){     //連続で別の番号を押した時のための処理
                marker[i].marker_initializing = false;
            }
        }
    }
    
    if (key == 's'){
        if (improcess.showImage == true){
            improcess.showImage = false;
        }
        else {
            improcess.showImage = true;
        }
    }
    
    /* 先頭選択用 */
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
    if (!homography.movePoint(homography.srcPoints,cur) && y < cam_margin + camheight + 10){    //10は念のために余分に取っただけ
        if (homography.srcPoints.size() < 4){   //4点未満の時しかpush_backはしない
            cv::Point2f cur(x - cam_margin, y - cam_margin);
            homography.srcPoints.push_back(cur);
        }

        if (homography.first && static_cast<int>(homography.srcPoints.size()) == 4){     //set destination points
            
            homography.warpedPoints.push_back(cv::Point2f(0,0));
            homography.warpedPoints.push_back(cv::Point2f(0,0) + cv::Point2f(0,camheight));
            homography.warpedPoints.push_back(cv::Point2f(0,0) + cv::Point2f(camwidth,camheight));
            homography.warpedPoints.push_back(cv::Point2f(0,0) + cv::Point2f(camwidth,0));
            
            homography.homographyMat = cv::findHomography(homography.srcPoints, homography.warpedPoints);
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
        }
    }
    
    calcCenter();  //重心と角度も更新
    calcAngle();
    calcNormalizedPoint();  //実際の座標に合わせた数値に変換
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

void simulatorClass::markerGen(ofVec2f center){
    drawCube(ofVec2f(center.x + 5, center.y));
    drawCube(ofVec2f(center.x - 10, center.y + 5));
    drawCube(ofVec2f(center.x - 10, center.y - 5));
}
