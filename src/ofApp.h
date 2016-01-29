#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxOpenCv.h"
#include <cmath>
#include <vector>
#include <typeinfo>
#include <string>

constexpr int camwidth = 448;
constexpr int camheight = 336;
constexpr int cam_margin = 30;
const int BUF_LABEL=1024;   //raspiでは領域の再確保が発生するとセグフォ起こしたので大きめに取っておく
const int region = 512;     //ラベリングから受け取る点の最大値(実際の運用時は30とか？)
const int min_region = 10;   //ラベリングの際にこの数値以下の小さい領域は無視する(ノイズ除去のため)

class markerInfo{  //マーカーの座標などを保管しておく
    public :
    /* 座標関連 */
    ofVec2f point[3];   //マーカーの頂点座標(先端を0番とし、時計回りにする)
    ofVec2f prev_point[3];  //前のフレームでのマーカー位置
    ofVec2f marker_center;  //3点の重心位置
    ofVec2f prev_marker_center; //前のフレームでの重心位置
    ofVec2f velocity;   //(1フレーム辺りの)機体の速度(移動距離)
    double angle;   //マーカーの方向(値は tan x とする)
    //char IP;  //各機のIPアドレス
    
    /* 領域指定用 */
    bool marker_initializing;   //マーカーの初期化中かどうか(ドラッグで領域を選択するため、イベントを区別)
    static int pointSet;   //領域指定の時の一時的な変数
    static bool drawing;    //描画中かどうかのフラグ
    ofVec2f init_region[2]; //指定する領域の左上、右下の座標を保管
    static ofVec2f mouse_position;    //描画用に領域指定中のマウス位置を保管
    
    /* 関数 */
    inline void calcAngle(ofVec2f front,ofVec2f marker_center){   //角度算出
        angle = (front.x - marker_center.x) / (front.y - marker_center.y);
    }
    inline void calcVelocity(){  //速度算出
        velocity.x = marker_center.x - prev_marker_center.x;
        velocity.y = marker_center.y - prev_marker_center.y;
    }
    void init(ofVec3f *markerPoints);   //個体を認識するため、3つの点が含まれる領域を設定
    void drawRegion();
    markerInfo(){
        marker_initializing = false;
    }
};

class imageProcess{
    public :
    
    /* for camera input */
    ofImage grayImage,bin;
    
    /* pixel datas */
    
    unsigned char *pixels_origin;
    unsigned char *pixels_gray;
    unsigned char *pixels_bin;
    
    unsigned char red,green,blue,gray;
    
    /* for labeling */
    
    int num = 0;
    int previous_num = region;  //一個前のラベル数を保存
    cv::Mat_<int> labels = cv::Mat_<int>::zeros(camheight,camwidth);
    cv::Mat bin_mat;    //ofとCVの変換(ラッパー)用
    ofVec3f center_point[region];       //ラベリングされた領域の重心を保管する (x,y,アクティブor非アクティブ)
    void writePoints();
    
    /* flags */
    bool isNewframe = false;
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
    bool homographyReady = false;
    bool first = true;
    
    /* 基準点変更時にどの点を変更するのか調べるための関数(どの点が近いのか距離を測定する) */
    inline double distance(cv::Point2f a, cv::Point2f b){
        return sqrt((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y));
    }
    
    /* 座標に対しホモグラフィ行列を適用する関数 */
    inline void executeTransform(ofVec3f &input){
        // 変換行列をかけるベクトルは(x,y,1)の形で、3項目は(省略されているが)1をかけている
        input.x = homographyMat.at<double>(0,0) * input.x + homographyMat.at<double>(0,1) * input.y + homographyMat.at<double>(0,2);
        input.y = homographyMat.at<double>(1,0) * input.x + homographyMat.at<double>(1,1) * input.y + homographyMat.at<double>(1,2);
        //input.z = homographyMat.at<double>(2,0) * input.x + homographyMat.at<double>(2,1) * input.y + homographyMat.at<double>(2,2);
        //z座標は使わないし毎回初期化するので意味ない
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
};

