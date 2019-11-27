// -*- C++ -*-
/*!
 * @file  Human_Tracking.cpp
 * @brief LRFを用いた複数人追跡RTC
 * @date $Date$
 *
 * @author Daiki Nomiyama, Shinichi Ishida, Shogo Namatame, Nobuto Matsuhira 
 *         SHIBAURA INSTITUTE OF TECHNOLOGY /  3-7-5 TOYOSU KOUTO-KU TOKYO 135-8548
 *　　　　 Email:md14067@shibaura-it.ac.jp
 *         Tell : 03-5859-8073
 * @author 野見山大基
 *         芝浦工業大学　工学部　機械工学専攻
 *         135-8548 東京都江東区豊洲3-7-5
 *　　　　 Email:md14067@shibaura-it.ac.jp
 *         Tell : 03-5859-8073
 * 
 * ライセンス：
 * 以下のMIT Licenceを適用します．
 * ////////////////////////////////////////////////////////////////////////////////////////////////////

	Copyright (c) 2003-2009, MIST Project, Intelligent Media Integration COE, Nagoya University
	All rights reserved.

	Redistribution and use in source and binary forms, with or without modification,
	are permitted provided that the following conditions are met:

		1. Redistributions of source code must retain the above copyright notice,
		   this list of conditions and the following disclaimer. 

		2. Redistributions in binary form must reproduce the above copyright notice,
		   this list of conditions and the following disclaimer in the documentation
		   and/or other materials provided with the distribution.

		3. Neither the name of the Nagoya University nor the names of its contributors
		   may be used to endorse or promote products derived from this software
		   without specific prior written permission. 

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
	IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
	FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
	CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
	DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
	DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
	IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
	THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

	//////////////////////////////////////////////////////////////////////////////////////////////////////
 * http://mist.suenaga.cse.nagoya-u.ac.jp/trac/wiki/License (Last accessed 2014/09/24)
 *
 *
 *
 * $Id$
 */

#ifndef HUMAN_TRACKING_H
#define HUMAN_TRACKING_H
#define max_particles 100
#define NUM_Hmax         11

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>
#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service implementation headers
// <rtc-template block="service_impl_h">

// </rtc-template>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">

// </rtc-template>

using namespace RTC;

/*!
 * @class Human_Tracking
 * @brief LRFを用いた複数人追跡RTC
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */
class Human_Tracking
  : public RTC::DataFlowComponentBase
{
 public:

	struct Data{
	double mem_data_x[1100];//生データ
	double mem_data_x2[1100];//更新用データ（外周を書くときに使う関数）
	double mem_data_x3[1100];//障害物データ
	double mem_data_x4[1100];//生データ記録用
	int m_different_search[1100];//人フラグ
	int parson_divide[NUM_Hmax][1100];	//複数人物の検出をするための変数、人数[6]とデータ[800]の配列
	int LRF_start;
	int LRF_end;
	int LRF_mid;
	int LRF_theta;
	int LRF_steps;
	};
	Data* m_data;


	
	double human_sx[1100];
	double human_sy[1100];
	double human_ex[1100];
	double human_ey[1100];
	double human_dr[1100];
	int move_trace[NUM_Hmax][3];
	int flag_check_start[NUM_Hmax];
	int flag_check_end[NUM_Hmax];
	double ppx_d;
	double ppy_d;
	double theta;
	double G_dist;
	double px_d_other[NUM_Hmax],py_d_other[NUM_Hmax];
	double theta_other[NUM_Hmax];
	double G_dist_other[NUM_Hmax];
	int G_senter_angl[NUM_Hmax];
	double distance_xy[NUM_Hmax][3];
	double distance_xy2[NUM_Hmax][3];
	double distance_xy3[NUM_Hmax][3];
	double distance_S[NUM_Hmax];
	double distance_S2[NUM_Hmax];
	double distance_S3[NUM_Hmax];
	int m_human_in;
	int m_human_out;
	int m_human_wait;
	int tmp_x[NUM_Hmax][100];
	int tmp_y[NUM_Hmax][100];
	int tmp_x2[NUM_Hmax][100];
	int tmp_y2[NUM_Hmax][100];
	int Tracking_check[NUM_Hmax][NUM_Hmax];
	int Human_check[NUM_Hmax];
	//double scale_control;//画像の尺度を変更するための変数

	struct Human{//各人データ収納構造体
		double velocity[11];//速さ
		double vector[11];//向き
		//double vector[3];//向き
		clock_t start;
		clock_t  end;
		//極座標系
		double distance[100];//半径
		double theta[100];//角度
		//直行座標系(生データ)
		double x[100];//x座標
		double y[100];//y座標
		//直行座標系(追跡処理)
		double x2[100];//x座標
		double y2[100];//y座標
		double distance2[100];//半径
		double theta2[100];
		//保存用
		double x3[100];//x座標
		double y3[100];//y座標
		//送信メッセージログ
		int Hum_ID2;
		double S_msg[20];
		int hum_count[11];
		bool count_state;
		int Msg_Lv[5];
		double xp[max_particles];
		double yp[max_particles];
		double xp2[max_particles];
		double yp2[max_particles];
		//double heading[max_particles];
		//int size;
		double  weight[max_particles];
	};
	struct Humans{
		Human human[NUM_Hmax];
	};

	Humans* m_qHuman;
	Humans* m_qHuman2;

	void strong_pass_filter(int scan_range,int filter_limit);
	void weak_add_filter(int scan_range,int filter_limit);
	void parson_devid_method(int scan_range,int filter_limit,int delsize,int LimitAveDistance);
	void trans_memdata_to_pxpy(int count);
	void LRFPocoHuman();
	void Human_Initialize();
	void TARGET_TRACKING(struct Humans *m_MAN);
	void HUMAN_STATE_CALCULATE(struct Humans *m_MAN);
	void HUMAN_MIGRATION_LENGTH(int i,struct Humans *m_MAN);
	void HUMAN_DELETE_COUNT(int i,struct Humans *m_MAN);
	void HUMAN_DELETE(struct Humans *m_MAN);
	void arrayInsert(double data[], int n, double newdata, int k);
	int PrimeNum_Search(int HUMAN_ID[],int MODE);
	int PrimeNum_KILL(int HUMAN_ID[],int KILL_ID);
	void resample( int i, struct Humans *m_MAN,struct Humans *m_MAN2);
	void predict(int i, struct Humans *m_MAN);
	void weight( int i, struct Humans *m_MAN);
	void measure(int i, struct Humans *m_MAN);
	void Data_outport();
	void LRFData_In();





  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  Human_Tracking(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~Human_Tracking();

  // <rtc-template block="public_attribute">
  
  // </rtc-template>

  // <rtc-template block="public_operation">
  
  // </rtc-template>

  /***
   *
   * The initialize action (on CREATED->ALIVE transition)
   * formaer rtc_init_entry() 
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onInitialize();

  /***
   *
   * The finalize action (on ALIVE->END transition)
   * formaer rtc_exiting_entry()
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onFinalize();

  /***
   *
   * The startup action when ExecutionContext startup
   * former rtc_starting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStartup(RTC::UniqueId ec_id);

  /***
   *
   * The shutdown action when ExecutionContext stop
   * former rtc_stopping_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onShutdown(RTC::UniqueId ec_id);

  /***
   *
   * LRFと接続し，複数人追跡を開始する．描写ウィンドウを生成．
   * The activated action (Active state entry action)
   * former rtc_active_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);

  /***
   *　複数人追跡を終了する．描写ウィンドウを削除．
   *
   * The deactivated action (Active state exit action)
   * former rtc_active_exit()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);

  /***
   * 人検知及び複数人追跡を行う．
   *
   *
   * The execution action that is invoked periodically
   * former rtc_active_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
   virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);

  /***
   *
   * The aborting action when main logic error occurred.
   * former rtc_aborting_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onAborting(RTC::UniqueId ec_id);

  /***
   *
   * The error action in ERROR state
   * former rtc_error_do()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onError(RTC::UniqueId ec_id);

  /***
   *
   * The reset action that is invoked resetting
   * This is same but different the former rtc_init_entry()
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onReset(RTC::UniqueId ec_id);
  
  /***
   *
   * The state update action that is invoked after onExecute() action
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onStateUpdate(RTC::UniqueId ec_id);

  /***
   *
   * The action that is invoked when execution context's rate is changed
   * no corresponding operation exists in OpenRTm-aist-0.2.0
   *
   * @param ec_id target ExecutionContext Id
   *
   * @return RTC::ReturnCode_t
   * 
   * 
   */
  // virtual RTC::ReturnCode_t onRateChanged(RTC::UniqueId ec_id);


 protected:
  // <rtc-template block="protected_attribute">
  
  // </rtc-template>

  // <rtc-template block="protected_operation">
  
  // </rtc-template>

  // Configuration variable declaration
  // <rtc-template block="config_declare">
 /*
  * 以下，コンフィギュレーションパラメーター
  *	詳細はユーザーマニュアルを参照してください．
  */

  /*!
   * 　検出する人数（最大10人まで）
   * - Name:  NUM_max
   * - DefaultValue: 6
   * - Unit: 人
   * - Constraint: <=10
   */
  int m_NUM_max;
  /*!
   * 　最大環境データ
   * - Name:  environment
   * - DefaultValue: 4000
   * - Unit: mm
   */
  int m_environment;
  /*!
   * 　LRFで使用するレーザの最小ステップ番号
   * - Name:  LRFdata_min
   * - DefaultValue: 44
   * - Unit: ステップ
   */
  int m_LRFdata_min;
  /*!
   * 　LRFで使用するレーザの最大ステップ番号
   * - Name:  LRFdata_max
   * - DefaultValue: 725
   * - Unit: ステップ
   */
  int m_LRFdata_max;
  /*!
   * 　LRFで使用するレーザの中間ステップ番号
   * - Name:  LRFdata_mid
   * - DefaultValue: 384
   * - Unit: ステップ
   */
   int m_LRFdata_mid;
    /*!　
   * 　LRFのトータルステップ数　
   * - Name:  LRF_steps
   * - DefaultValue: 1024
   * - Unit: ステップ
   */
   int m_LRFdata_theta;
  /*!　
   * 　走査角度（角度分解能算出に使われている角度）
   * - Name:  LRFdata_theta
   * - DefaultValue: 360
   * - Unit: deg
   */
   int m_LRF_steps;
   /*!
   *   物体検出の最小幅
   * - Name:  human_width
   * - DefaultValue: 200
   * - Unit: mm
   */
   int m_human_width;
  /*!
   *   人数カウントを行う際の入退室の向き変更用．　　IN,OUT と OUT,IN で入退室の向きを変更できる．　
   * - Name:  Count_mode
   * - DefaultValue: "IN,OUT" 
   * - Range: IN,OUT or OUT,IN
   */
  std::string m_Count_mode;
  /*!
   *   人認識を行う最小距離
   * - Name:  Human_set
   * - DefaultValue: 2800
   * - Unit: mm
   */
  int m_Human_set;
   /*!
   *   人追跡処理を行う範囲
   * - Name:  Human_TrackingData
   * - DefaultValue: 2500
   * - Unit: mm
   */
  int m_Human_TrackingData;
   /*!
   *   人数カウントを行う際のエリア距離
   * - Name:  Count_area
   * - DefaultValue: 2500
   * - Unit: mm
   */
  int m_Count_area;
  // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  //詳しくはユーザーマニュアルを参照してください．
  RangeData m_range_data;
  /*!
   * 　LRFから値を取得する．
   * - Type: RangeData
   * - Unit: mm
   */
  InPort<RangeData> m_range_dataIn;
   // </rtc-template>

  // DataInPort declaration
  // <rtc-template block="inport_declare">
  //詳しくはユーザーマニュアルを参照してください．
  TimedLongSeq m_humanX;
  /*!
   * 　各人の位置データ（Ｘ軸[mm]）を出力する．
   * - Type: TimedLongSeq
   * - Number: m_NUM_maxに依存（最大10人）
   * - Unit: mm
   */
  OutPort<TimedLongSeq> m_humanXOut;

  TimedLongSeq m_humanY;
  /*!
   * 　各人の位置データ（Ｙ軸[mm]）を出力する．
   * - Type: TimedLongSeq
   * - Number: m_NUM_maxに依存（最大10人）
   * - Unit: mm
   */
  OutPort<TimedLongSeq> m_humanYOut;

  TimedLongSeq m_humanV;
  /*!
   * 　各人の歩行データ（速さ[mm/s]）を出力する．
   * - Type: TimedLongSeq
   * - Number: m_NUM_maxに依存（最大10人）
   * - Unit: mm/s
   */
  OutPort<TimedLongSeq> m_humanVOut;

  TimedLongSeq m_humanT;
  /*!
   * 　各人の歩行データ（向き[deg]）を出力する．
   * - Type: TimedLongSeq
   * - Number: m_NUM_maxに依存（最大10人）
   * - Unit: deg
   */
  OutPort<TimedLongSeq> m_humanTOut;

  TimedLongSeq m_hcount;
  /*!
   * 　入退室及び滞在人数データを出力する．
   * - Type: TimedLongSeq
   * - Number: m_NUM_maxに依存（最大10人）
   * - Unit: deg
   */
  OutPort<TimedLongSeq> m_hcountOut;
  // </rtc-template>


  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  
  // </rtc-template>

  // Service declaration
  // <rtc-template block="service_declare">
  
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  
  // </rtc-template>

 private:
  // <rtc-template block="private_attribute">
  
  // </rtc-template>

  // <rtc-template block="private_operation">
  
  // </rtc-template>

};


extern "C"
{
  DLL_EXPORT void Human_TrackingInit(RTC::Manager* manager);
};

#endif // HUMAN_TRACKING_H
