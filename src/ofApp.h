#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxOpenCv.h"

#include "Const.hpp"
#include "CameraSender.h"
#include "Simulator.hpp"
#include "Labeling.hpp"
#include "Homography.hpp"
#include "ImageProcess.hpp"

#include <cmath>
#include <vector>
#include <typeinfo>
#include <string>

#include <aruco.h>


class markerInfo{  //マーカーの座標などを保管しておく
    public :
        /* 座標関連 */
        ofVec2f point[3];   //マーカーの頂点座標(先端を0番とし、時計回りにする)
        ofVec2f prev_point[3];  //前のフレームでのマーカー位置
        ofVec2f marker_center;  //3点の重心位置
        ofVec2f prev_marker_center; //前のフレームでの重心位置
        ofVec2f velocity;   //(1フレーム辺りの)機体の速度(移動距離)
        ofVec2f normalized_point;   //実際の長さに合わせて正規化した座標
        ofVec2f prev_normalized_point = ofVec2f(0,0);
        double angle;   //マーカーの方向(rad単位)
        double prev_angle = 0;
        const double noise_floor_angle = 0.1;
        const double noise_floor_point = 0.1;
        int front;          //先頭の座標がpoint[3]の何番目か(角度算出用)
        //char IP;  //各機のIPアドレス
        
        /* 領域指定用 */
        bool marker_initializing;   //マーカーの初期化中かどうか(ドラッグで領域を選択するため、イベントを区別)
        bool active;   //初期化完了フラグ兼、生きているかどうか
        static int pointSet;   //領域指定の時の一時的な変数
        static bool drawing;    //描画中かどうかのフラグ
        static int selected;
        ofVec2f init_region[2]; //指定する領域の左上、右下の座標を保管
        static ofVec2f mouse_position;    //描画用に領域指定中のマウス位置を保管
    
        /* 表示用 */
        string pointStr;
    
        /* 関数 */
//        inline void calcAngle(){   //角度算出
//            angle = atan2(-(point[front].y - marker_center.y),point[front].x - marker_center.x);
//        }
         void calcAngle(){   //角度算出
            angle = atan2(-(point[front].x - marker_center.x),point[front].y - marker_center.y);
            angle += M_PI;
            if (abs(angle - prev_angle) < noise_floor_angle){
                angle = prev_angle;
            }
            else{
                prev_angle = angle;
            }
        }
         void calcVelocity(){  //速度算出
            velocity.x = marker_center.x - prev_marker_center.x;
            velocity.y = marker_center.y - prev_marker_center.y;
        }
         double distance(ofVec2f a,ofVec2f b){    //二点間の距離(の二乗)
            return ((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
        }
         void calcCenter(){
            marker_center = ofVec2f((point[0].x + point[1].x + point[2].x) / 3, (point[0].y + point[1].y + point[2].y) / 3);
        }
        void calcNormalizedPoint(ofVec2f *offset){
            //normalized_point = ofVec2f((-marker_center.x + offset[0].x) * field_width/2700,(marker_center.y - offset[0].y) * field_height/1800);
            normalized_point = ofVec2f(field_height - ((marker_center.y - offset[0].y) * field_height/(offset[1].y - offset[0].y)),(field_width - (-marker_center.x + offset[1].x) * field_width/(offset[1].x - offset[0].x)));
            if ((abs(normalized_point.x - prev_normalized_point.x) < noise_floor_point) && (abs(normalized_point.y - prev_normalized_point.y) < noise_floor_point)){
                normalized_point = prev_normalized_point;   //using previous point(to avoid oscillation)
            }
            else{
                prev_normalized_point = normalized_point;   //update
            }
        }
        void init(ofVec3f *markerPoints);   //個体を認識するため、3つの点が含まれる領域を設定
        void drawRegion();
        void showMarker();
        void update(ofVec3f *markerPoints, int array_length);
        void highlightFront();
        
        markerInfo(){
            marker_initializing = false;
            active = false;
        }
};

//class k_means {
//    public :
//    const int NOT_USED = 128;    //数字は適当
//    static const int NUM_OF_CLUSTERS = 8;   // 分類するクラスタ数(=ロボットの数)
//    
//    ofVec2f clusterPositions[NUM_OF_CLUSTERS];     //各クラスタの中心座標(=ロボットの座標)
//    //std::vector<ofVec2f> clusterInfo[NUM_OF_CLUSTERS];     //各点がどのクラスタに属するか
//    ofVec3f inputPoints[NUM_OF_CLUSTERS * 3];     //入力点の情報 zはどのクラスタに属するかを入れる
//    bool finished = 1;  //収束したかどうかのフラグ
//    
//    inline double distance(ofVec3f input, ofVec2f clusterPoint){
//        return sqrt((input.x - clusterPoint.x)*(input.x - clusterPoint.x) + (input.y - clusterPoint.y)*(input.y - clusterPoint.y));
//        //重かったらマンハッタン距離にするかも
//    }
//    
//    void getInputData(ofVec3f *input){
//        for (int i = 0; i < NUM_OF_CLUSTERS*3; i++){    //ロボット * 3
//            if (inputPoints[i].z == 0){  //使われていない領域は無視(z=0は非アクティブのもの)
//                inputPoints[i].z = NOT_USED;
//                break;
//            }
//            inputPoints[i] = input[i+1];  //入力はcenter_point[]なので1から始める
//            inputPoints[i].z = 0;
//        }
//    }
//    
//    /* 所属するクラスタを探す */
//    void belonging(){
//        finished = 1;   //終わったかどうかのフラグ、まだ変更された点があれば最後に0にする
//        for (int i = 0; i < NUM_OF_CLUSTERS * 3; i++){
//            /* 最短の点を探す */
//            int result = 0;
//            double current_min = distance(inputPoints[i], clusterPositions[0]);   //とりあえず0番に所属させとく
//            if (inputPoints[i].z == NOT_USED){
//                break;      //入力が3 * 8より少ない場合の処理 途中でNOT_USEDに出くわしたらbreakする
//            }
//            
//            for (int j = 1; j < NUM_OF_CLUSTERS; j++){
//                double dist = distance(inputPoints[i], clusterPositions[j]);
//                if (current_min > dist){
//                    current_min = dist;
//                    result = j;
//                }
//            }
//            /* 所属を変更 */
//            if (inputPoints[i].z != result){
//                inputPoints[i].z = result;
//                finished = 0;   //所属が変わる点がある場合はまだ終わっていない
//            }
//        }
//    }
//    
//    void update(){      //重心からクラスタの中心を変更
//        int x[NUM_OF_CLUSTERS*3] = {};
//        int y[NUM_OF_CLUSTERS*3] = {};
//        int counter[NUM_OF_CLUSTERS] = {};
//        for (int i = 0; i < NUM_OF_CLUSTERS*3; i++){
//            if (inputPoints[i].z == NOT_USED){
//                x[static_cast<int>(inputPoints[i].z)] /= i+1;
//                y[static_cast<int>(inputPoints[i].z)] /= i+1;
//                break;      //入力が3 * 8より少ない場合の処理 途中でNOT_USEDに出くわしたらbreakする
//            }
//            x[static_cast<int>(inputPoints[i].z)] += inputPoints[i].x;
//            y[static_cast<int>(inputPoints[i].z)] += inputPoints[i].y;
//            if (i == (NUM_OF_CLUSTERS*3 - 1)){
//                x[static_cast<int>(inputPoints[i].z)] /= (NUM_OF_CLUSTERS*3);
//                y[static_cast<int>(inputPoints[i].z)] /= (NUM_OF_CLUSTERS*3);
//            }
//        }
//    }
//};


class cameraFps{
  public:
    double currentTime;
    double previousTime = 0;
    int frameCounter;
    int fps;
    double normalFps;
    void getFps(double elapsedTime);
};

class camCalib{
  public:
    ofImage undistorted;
    ofxCv::Calibration calibration;
};

class ofApp : public ofBaseApp{

	public:
		void setup();
		void update();
		void draw();
        //void exit();

		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y );
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
    
        /* for cam input */
        ofVideoGrabber myCam;
    
        /* for marker */
        markerInfo marker[8];
    
        /* classes */
        imageProcess improcess;
        labelingClass labeling;
        homographyClass homography;
        cameraFps camFps;
        CameraSender oscSender;
        camCalib calib;
    
        ofTrueTypeFont font;

        aruco::MarkerDetector md;
        vector<aruco::Marker> markers;
        cv::Mat cvCamImage,imputimagee_copy;
        std::map<uint32_t, aruco::MarkerPoseTracker> MTrackr;
        const float Markersize = -1;
};

