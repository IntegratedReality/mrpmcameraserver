#pragma once

#include "ofMain.h"
#include "ofxCv.h"
#include "ofxOpenCv.h"
#include <cmath>
#include <vector>
#include <typeinfo>

constexpr int camwidth = 448;
constexpr int camheight = 336;
constexpr int cam_margin = 30;
const int BUF_LABEL=1024;

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
    cv::Mat_<int> labels = cv::Mat_<int>::zeros(camheight,camwidth);
    cv::Mat bin_mat;    //ofとCVの変換(ラッパー)用
    ofVec3f center_point[512];       //ラベリングされた領域の重心を保管する (x,y,アクティブor非アクティブ)
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
        return int(img.data[y*img.step + x]);
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
    
        /* classes */
        imageProcess improcess;
        labelingClass labeling;
        homographyClass homography;
};

