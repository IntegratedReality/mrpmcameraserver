//
//  Const.hpp
//  mrpmcameraserver
//
//  Created by 精密五月祭 on 2016/10/31.
//
//
#pragma once

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
const int num_of_robot = 4;

//const int infra_cam_height = 161.5;     //赤外線カメラの高さ
//const int robot_height = 9;     //ロボットの高さ
//const double height_compensation = ((infra_cam_height -  robot_height)/double(infra_cam_height));     //高さ補正の係数
