//
//  Labeling.cpp
//  mrpmcameraserver
//
//  Created by 精密五月祭 on 2016/10/31.
//
//

#include "Labeling.hpp"


labelingClass::labelingClass(){
    parents.reserve(BUF_LABEL);
}

bool labelingClass::isIn(int w,int h,int x,int y)
{
    return 0<=x && x<w && 0<=y && y<h;
}

unsigned int labelingClass::getAt(const cv::Mat& img,int x,int y)
{
    return int(img.data[y*img.step + x]);   //dataはキャストしないと取れないっぽい
}

//aの属すグループの代表に向かって経路圧縮（代表を返す）
 int labelingClass::compress(std::vector<int>& parents,int a)
{
    while(a!=parents[a])
    {
        parents[a]=parents[parents[a]];
        a=parents[a];
    }
    return a;
}

//aの属すグループとbの属すグループを併合（併合後の代表を返す）
 int labelingClass::link(std::vector<int>& parents,int a,int b)
{
    a=compress(parents,a);
    b=compress(parents,b);
    if(a<b)
        return parents[b]=a;
    else
        return parents[a]=b;
}

//番号とびとびなラベルを0,1,2,...に貼り替え
 int labelingClass::relabel(std::vector<int>& parents)
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
            ofDrawCircle(center_points[i].x, center_points[i].y, 3);
        }
    }
    
    ofPopStyle();
    ofNoFill();
}

