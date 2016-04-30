#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxOpenCv.h"
#include "CameraSender.h"
#include <cmath>
#include <vector>
#include <typeinfo>
#include <string>

constexpr int camwidth = 640;   //対応解像度で使わないとPC上の座標と画像の座標がずれるようなので注意
constexpr int camheight = 480;
constexpr int cam_margin = 30;
constexpr int field_width = 2700;
constexpr int field_height = 1800;
const int BUF_LABEL= 2048;   //raspiでは領域の再確保が発生するとセグフォ起こしたので大きめに取っておく
const int region = 512;     //ラベリングから受け取る点の最大値(実際の運用時は30とか？)
const int min_region = 5;   //ラベリングの際にこの数値以下の小さい領域は無視する(ノイズ除去のため)
const int max_velocity = 300;   //1フレームで進める最大距離(後で計算して決める)
const int bin_threshold = 150;  //二値化の閾値

const int infra_cam_height = 2;     //赤外線カメラの高さ
const int robot_height = 1;     //ロボットの高さ
//const double height_compensation = (infra_cam_height -  robot_height)/infra_cam_height;     //高さ補正の係数
const double height_compensation = 1;   //仮置き

class markerInfo{  //マーカーの座標などを保管しておく
    public :
        /* 座標関連 */
        ofVec2f point[3];   //マーカーの頂点座標(先端を0番とし、時計回りにする)
        ofVec2f prev_point[3];  //前のフレームでのマーカー位置
        ofVec2f marker_center;  //3点の重心位置
        ofVec2f prev_marker_center; //前のフレームでの重心位置
        ofVec2f velocity;   //(1フレーム辺りの)機体の速度(移動距離)
        ofVec2f normalized_point;   //実際の長さに合わせて正規化した座標
        double angle;   //マーカーの方向(rad単位)
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
        inline void calcAngle(){   //角度算出
            angle = atan2(-(point[front].x - marker_center.x),point[front].y - marker_center.y);
            angle += M_PI;
        }
        inline void calcVelocity(){  //速度算出
            velocity.x = marker_center.x - prev_marker_center.x;
            velocity.y = marker_center.y - prev_marker_center.y;
        }
        inline double distance(ofVec2f a,ofVec2f b){    //二点間の距離(の二乗)
            return ((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
        }
        inline void calcCenter(){
            marker_center = ofVec2f((point[0].x + point[1].x + point[2].x) / 3, (point[0].y + point[1].y + point[2].y) / 3);
        }
        inline void calcNormalizedPoint(ofVec2f *offset){
            //normalized_point = ofVec2f((-marker_center.x + offset[0].x) * field_width/2700,(marker_center.y - offset[0].y) * field_height/1800);
            normalized_point = ofVec2f((marker_center.y - offset[0].y) * field_height/(offset[1].y - offset[0].y),(-marker_center.x + offset[1].x) * field_width/(offset[1].x - offset[0].x));
            normalized_point.x *= height_compensation;
            normalized_point.y *= height_compensation;
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

class imageProcess{
    public :
    
        /* for camera input */
        ofImage bin;
        
        /* pixel datas */
        
        unsigned char *pixels_origin;
        unsigned char *pixels_gray;
        unsigned char *pixels_bin;
        
        unsigned char red,green,blue,gray;
        
        /* for labeling */
        
        int num = 0;
        int previous_num = region;  //一個前のラベル数を保存
        int num_of_light = 0;   //min_region以上の領域の個数
        int filter_intensity = 1;   //medianフィルターの強度
        cv::Mat_<int> labels = cv::Mat_<int>::zeros(camheight,camwidth);
        cv::Mat bin_mat;    //ofとCVの変換(ラッパー)用
        ofVec3f center_point[region];       //ラベリングされた領域の重心を保管する (x座標,y座標,領域の大きさ)
        ofVec2f usingArea[2];   //使用する範囲を指定(左上と右下)
        ofVec2f areaSize;
        void writePoints();
    
        ofTexture camTexture;
        ofTexture binTexture;
        ofFbo camFbo;
        ofFbo binFbo;
        ofFbo stringFbo;
    
        ofVec3f homographyCorner[4];
    
        /* flags */
        bool isNewframe = false;
        bool showImage = true;
        bool setCoord = false;  //使用する範囲を指定して座標を確定
        bool setCoordToggle = false;    //指定する点の切り替え用
        bool usingAreaDecided = false;
    
        imageProcess(){
            homographyCorner[0] = ofVec3f(0,0,0);
            homographyCorner[1] = ofVec3f(camwidth,0,0);
            homographyCorner[2] = ofVec3f(camwidth,camheight,0);
            homographyCorner[3] = ofVec3f(0,camheight,0);
        }
};

class homographyClass{
    public :
    
        vector<cv::Point2f> srcPoints,warpedPoints;     //ホモグラフィの基準となる4点(×2)
        cv::Mat homographyMat;     //ホモグラフィ行列(変換行列)
        
        /* 基準点選択用 */
        bool movePoint(vector<cv::Point2f>& points, cv::Point2f point);
        void drawPoints(vector<cv::Point2f>& points);
        cv::Point2f* curPoint;      //一度選択した座標を変更する際に使うポインタ
        bool movingPoint = false;
        
        /* 諸々のフラグ */
        bool ready = false;
        bool first = true;
        
        /* 基準点変更時にどの点を変更するのか調べるための関数(どの点が近いのか距離を測定する) */
        inline double distance(cv::Point2f a, cv::Point2f b){
            return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
        }
        
        /* 座標に対しホモグラフィ行列を適用する関数 */
        inline void executeTransform(ofVec3f &input){
            // 変換行列をかけるベクトルは(x,y,1)の形で、3項目は(省略されているが)1をかけている
            double x = input.x; //変数を退避
            double y = input.y;
            double scale = homographyMat.at<double>(2,0) * x + homographyMat.at<double>(2,1) * y + 1;
            input.x = (homographyMat.at<double>(0,0) * x + homographyMat.at<double>(0,1) * y + homographyMat.at<double>(0,2))/scale;
            input.y = (homographyMat.at<double>(1,0) * x + homographyMat.at<double>(1,1) * y + homographyMat.at<double>(1,2))/scale;
        }
};

class labelingClass{
    public :
    
        labelingClass(){
            parents.reserve(BUF_LABEL);
        }
        
        std::vector<int> parents;
        
        void drawRegions(ofVec3f* point, int nums);
        
        inline bool isIn(int w,int h,int x,int y)
        {
            return 0<=x && x<w && 0<=y && y<h;
        }
        
        inline unsigned int getAt(const cv::Mat& img,int x,int y)
        {
            return int(img.data[y*img.step + x]);   //dataはキャストしないと取れないっぽい
        }
        
        //aの属すグループの代表に向かって経路圧縮（代表を返す）
        inline int compress(std::vector<int>& parents,int a)
        {
            while(a!=parents[a])
            {
                parents[a]=parents[parents[a]];
                a=parents[a];
            }
            return a;
        }
        
        //aの属すグループとbの属すグループを併合（併合後の代表を返す）
        inline int link(std::vector<int>& parents,int a,int b)
        {
            a=compress(parents,a);
            b=compress(parents,b);
            if(a<b)
                return parents[b]=a;
            else
                return parents[a]=b;
        }
        
        //番号とびとびなラベルを0,1,2,...に貼り替え
        inline int relabel(std::vector<int>& parents)
        {
            int index=0;
            for(int k=0;k<(int)parents.size();k++)
            {
                if(k==parents[k])
                    parents[k]=index++;
                else
                    parents[k]=parents[parents[k]];
            }
            return index;
        }
        
        int labeling(const cv::Mat& image,cv::Mat_<int>& label);
        
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

class simulatorClass{
    public :
        ofImage simulationImg;
        unsigned char *pixels_simulation;
        void markerGen(ofVec2f center);
    
        ofVec2f markerPos[3];
        int interval = 0;
        bool turn = false;
    
        inline void drawCube(ofVec2f point1){
            for (int i = point1.x; i <= point1.x + 3; i++){
                for (int j = point1.y; j <= point1.y + 3; j++){
                    pixels_simulation[j * camwidth + i ] = 255;
                }
            }
        }
        void init(){
            for (int i = 0; i < camheight; i++){
                for (int j = 0; j < camwidth; j++){
                    pixels_simulation[i * camwidth + j] = 0;
                }
            }
        }
        void movementManager(){
            if (interval == 250){
                turn = !turn;
                interval = 0;
            }
            if (!turn){
                markerPos[0].x += 1;
                
                markerPos[1].x -= 1;
                
                markerPos[2].y += 1;
            }
            else {
                markerPos[0].x -= 1;
                
                markerPos[1].x += 1;
                
                markerPos[2].y -= 1;

            }
            interval++;
        }
    
};

class cameraFps{
  public:
    double currentTime;
    double previousTime;
    int frameCounter;
    int fps;
    void getFps();
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
};

