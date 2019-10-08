#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file Human_Tracking.py
 @brief ModuleDescription
 @date $Date$


"""
import sys
import time
sys.path.append(".")
import datetime
import math
import csv
import cv2
import numpy as np
from numpy.random import*

# Import RTM module
import RTC
import OpenRTM_aist

human_tracking_spec = ["implementation_id", "Human_Tracking", 
		 "type_name",         "Human_Tracking", 
		 "description",       "ModuleDescription", 
		 "version",           "1.0.0", 
		 "vendor",            "Kusaka", 
		 "category",          "Category", 
		 "activity_type",     "STATIC", 
		 "max_instance",      "1", 
		 "language",          "Python", 
		 "lang_type",         "SCRIPT",
		 "conf.default.Human_NUM", "10",
		 "conf.default.Enviromental_maxdata", "4000",
		 "conf.default.LRFdata_min", "44",
		 "conf.default.LRFdata_max", "725",
		 "conf.default.LRFdata_mid", "384",
		 "conf.default.LRF_Total_steps", "1024",
		 "conf.default.LRFdata_theta", "360",
		 "conf.default.Human_width", "200",
		 "conf.default.Count_mode", "IN,OUT",
		 "conf.default.Count_area_distance", "2500",
		 "conf.default.Human_setdistance", "2800",
		 "conf.default.Human_TrackingData", "2500",

		 "conf.__widget__.Human_NUM", "text",
		 "conf.__widget__.Enviromental_maxdata", "text",
		 "conf.__widget__.LRFdata_min", "text",
		 "conf.__widget__.LRFdata_max", "text",
		 "conf.__widget__.LRFdata_mid", "text",
		 "conf.__widget__.LRF_Total_steps", "text",
		 "conf.__widget__.LRFdata_theta", "text",
		 "conf.__widget__.Human_width", "text",
		 "conf.__widget__.Count_mode", "text",
		 "conf.__widget__.Count_area_distance", "text",
		 "conf.__widget__.Human_setdistance", "text",
		 "conf.__widget__.Human_TrackingData", "text",

         "conf.__type__.Human_NUM", "int",
         "conf.__type__.Enviromental_maxdata", "int",
         "conf.__type__.LRFdata_min", "int",
         "conf.__type__.LRFdata_max", "int",
         "conf.__type__.LRFdata_mid", "int",
         "conf.__type__.LRF_Total_steps", "int",
         "conf.__type__.LRFdata_theta", "int",
         "conf.__type__.Human_width", "int",
         "conf.__type__.Count_mode", "string",
         "conf.__type__.Count_area_distance", "int",
         "conf.__type__.Human_setdistance", "int",
         "conf.__type__.Human_TrackingData", "int",

		 ""]

class struct_human:
	def __init__(self, manager):
		self._NUM_Hmax = 11
		self._max_particles = 100
		self.velocity = [0 for i in range(self._NUM_Hmax)] 	# 速さ
		self.vector = [0 for i in range(self._NUM_Hmax)] 	# 向き
		self.clock_start = 0.0
		self.clock_end = 0.0
		# 極座標
		self.distance = [0 for i in range(100)]		# 半径
		self.theta = [0 for i in range(100)]		# 角度
		# 直行座標系(生データ)
		self.x = [0 for i in range(100)]			# x座標
		self.y = [0 for i in range(100)]			# y座標
		# 直行座票系(追従処理)
		self.x2 = [0 for i in range(100)]			# x座標
		self.y2 = [0 for i in range(100)]			# y座標
		self.distance2 = [0 for i in range(100)]	# 半径
		self.theta2 = [0 for i in range(100)]
		# 保存用
		self.x3 = [0 for i in range(100)]			# x座標
		self.y3 = [0 for i in range(100)]			# y座標
		# 送信メッセージログ
		self.Hum_ID2 = 0
		self.S_msg = [0 for i in range(self._NUM_Hmax)]
		self.hum_count = [0 for i in range(self._NUM_Hmax)]
		self.count_state = False
		self.Msg_Lv = [0 for i in range(self._NUM_Hmax)]
		self.xp = [0 for i in range(self._max_particles)]
		self.yp = [0 for i in range(self._max_particles)]
		self.xp2 = [0 for i in range(self._max_particles)]
		self.yp2 = [0 for i in range(self._max_particles)]
		self.weight = [0 for i in range(self._max_particles)]

class struct_Data:
	def __init__(self, manager):
		self._NUM_Hmax = 11
		self._men_data_x = [0 for i in range(1100)]				# 生データ
		self._men_data_x2 = [0 for i in range(1100)]			# 更新用データ(外周を書く時に使う関数)
		self._men_data_x3 = [0 for i in range(1100)]			# 障害物データ
		self._men_data_x4 = [0 for i in range(1100)]			# 生データ記録用
		self._m_different_search = [0 for i in range(1100)]		# 人フラグ
		self._parson_divide =[[0 for i in range(1100)] for j in range(self._NUM_Hmax)]		# 複数人物の検出をするための変数、人数[10]とデータ[800]の配列
		self._xx = [0 for i in range(self._NUM_Hmax)]
		self._yy = [0 for i in range(self._NUM_Hmax)]

class Human_Tracking(OpenRTM_aist.DataFlowComponentBase):
	

	def __init__(self, manager):
		OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

		self._d_Range_data = RTC.RangeData(RTC.Time(0,0), [], RTC.RangerGeometry(RTC.Geometry3D(RTC.Pose3D(RTC.Point3D(0.0,0.0,0.0), RTC.Orientation3D(0.0,0.0,0.0)), RTC.Size3D(0.0,0.0,0.0)), []), RTC.RangerConfig(*([0.0]*7)))
		# self._d_Range_data = RTC.RangeData(RTC.Time(0,0), RTC.double[0])
		"""
		"""
		self._Range_dataIn = OpenRTM_aist.InPort("Range_data", self._d_Range_data)
		self._d_human_dataX = RTC.TimedLongSeq(RTC.Time(0,0), [0]*2)
		"""
		"""
		self._human_dataXOut = OpenRTM_aist.OutPort("human_dataX", self._d_human_dataX)
		self._d_human_dataY = RTC.TimedLongSeq(RTC.Time(0,0), [0]*2)
		"""
		"""
		self._human_dataYOut = OpenRTM_aist.OutPort("human_dataY", self._d_human_dataY)
		self._d_human_dataV = RTC.TimedLongSeq(RTC.Time(0,0), [0]*2)
		"""
		"""
		self._human_dataVOut = OpenRTM_aist.OutPort("human_dataV", self._d_human_dataV)
		self._d_human_dataT = RTC.TimedLongSeq(RTC.Time(0,0), [0]*2)
		"""
		"""
		self._human_dataTOut = OpenRTM_aist.OutPort("human_dataT", self._d_human_dataT)
		# self._d_hcount = RTC.TimedLongSeq(RTC.Time(0,0), [0]*2)
		"""
		"""
		# self._hcountOut = OpenRTM_aist.OutPort("hcount", self._d_hcount)
		self._d_human_in_data = RTC.TimedLong(RTC.Time(0,0), "null")
		"""
		"""
		self._human_in_dataOut = OpenRTM_aist.OutPort("human_in_data", self._d_human_in_data)
		self._d_human_out_data = RTC.TimedLong(RTC.Time(0,0), "null")
		"""
		"""
		self._human_out_dataOut = OpenRTM_aist.OutPort("human_out_data", self._d_human_out_data)
		self._d_human_wait_number = RTC.TimedLong(RTC.Time(0,0), "null")
		"""
		"""
		self._human_wait_numberOut = OpenRTM_aist.OutPort("Congestion_degree", self._d_human_wait_number)


		# self._men_data_x = []		# 生データ
		# self._different_search = []	# 人フラグ
		self._human_sx = []
		self._human_sy = []
		self._human_ex = []
		self._human_ey = []
		self._human_dr = []
		self._flag_check_start = []
		self._flag_check_end = []
		self._theta = 0
		self._ppx_d = 0
		self._ppy_d = 0
		self._G_dist = 0
		self._m_human_wait = 0
		self._m_human_in = 0
		self._m_human_out = 0
		self._human_all = 0

		self._first_flag = 1
		self._first_flagA = 1
		self._max_particles = 100
		self._particle_filter_cur_frame = 0

		self._m_qHuman = struct_human('qHuman1')
		self._m_qHuman2 = struct_human('qHuman2')
		self._m_data = struct_Data('data')

		self._NUM_Hmax = 11
		# self._move_trace = [[0]*self._NUM_Hmax]*self._NUM_Hmax
		self._move_trace = [[0 for i in range(self._NUM_Hmax)] for j in range(self._NUM_Hmax)]
		# self._parson_divide = [[0 for i in range(1100)] for j in range(self._NUM_Hmax)]
		self._Tracking_check = [[0 for i in range(self._NUM_Hmax)] for j in range(self._NUM_Hmax)]
		self._human = [self._m_qHuman for i in range(self._NUM_Hmax)]
		self._human2 = [self._m_qHuman2 for i in range(self._NUM_Hmax)]
		self._theta_other = [0 for i in range(self._NUM_Hmax)]
		self._distance_xy = [[0 for i in range(self._NUM_Hmax)] for j in range(self._NUM_Hmax)]
		self._distance_S = [0 for i in range(self._NUM_Hmax)]

		self._distance_S2 = [0 for i in range(self._NUM_Hmax)]

		self._tmp_x = [[0 for i in range(100)] for j in range(self._NUM_Hmax)]
		self._tmp_y = [[0 for i in range(100)] for j in range(self._NUM_Hmax)]

		"""
		
		 - Name:  Human_NUM
		 - DefaultValue: 10
		"""
		self._Human_NUM = [10]
		"""
		
		 - Name:  Enviromental_maxdata
		 - DefaultValue: 4000
		"""
		self._Enviromental_maxdata = [4000]
		"""
		
		 - Name:  LRFdata_min
		 - DefaultValue: 44
		"""
		self._LRFdata_min = [44]
		"""
		
		 - Name:  LRFdata_max
		 - DefaultValue: 725
		"""
		self._LRFdata_max = [725]
		"""
		
		 - Name:  LRFdata_mid
		 - DefaultValue: 384
		"""
		self._LRFdata_mid = [384]
		"""
		
		 - Name:  LRF_Total_steps
		 - DefaultValue: 1024
		"""
		self._LRF_Total_steps = [1024]
		"""
		
		 - Name:  LRFdata_theta
		 - DefaultValue: 360
		"""
		self._LRFdata_theta = [360]
		"""
		
		 - Name:  Human_width
		 - DefaultValue: 200
		"""
		self._Human_width = [200]
		"""
		
		 - Name:  Count_mode
		 - DefaultValue: IN,OUT
		"""
		self._Count_mode = ['IN,OUT']
		"""
		
		 - Name:  Count_area_distance
		 - DefaultValue: 2500
		"""
		self._Count_area_distance = [2500]
		"""
		
		 - Name:  Human_setdistance
		 - DefaultValue: 2800
		"""
		self._Human_setdistance = [2800]
		"""
		
		 - Name:  Human_TrackingData
		 - DefaultValue: 2500
		"""
		self._Human_TrackingData = [2500]
		
		# </rtc-template>

	def onInitialize(self):
		# Bind variables and configuration variable
		self.bindParameter("Human_NUM", self._Human_NUM, "10")
		self.bindParameter("Enviromental_maxdata", self._Enviromental_maxdata, "4000")
		self.bindParameter("LRFdata_min", self._LRFdata_min, "44")
		self.bindParameter("LRFdata_max", self._LRFdata_max, "725")
		self.bindParameter("LRFdata_mid", self._LRFdata_mid, "384")
		self.bindParameter("LRF_Total_steps", self._LRF_Total_steps, "1024")
		self.bindParameter("LRFdata_theta", self._LRFdata_theta, "360")
		self.bindParameter("Human_width", self._Human_width, "200")
		self.bindParameter("Count_mode", self._Count_mode, "IN,OUT")
		self.bindParameter("Count_area_distance", self._Count_area_distance, "2500")
		self.bindParameter("Human_setdistance", self._Human_setdistance, "2800")
		self.bindParameter("Human_TrackingData", self._Human_TrackingData, "2500")
		
		# Set InPort buffers
		self.addInPort("Range_data",self._Range_dataIn)
		
		# Set OutPort buffers
		self.addOutPort("human_dataX",self._human_dataXOut)
		self.addOutPort("human_dataY",self._human_dataYOut)
		self.addOutPort("human_dataV", self._human_dataVOut)
		self.addOutPort("human_dataT", self._human_dataTOut)
		self.addOutPort("human_in_data", self._human_in_dataOut)
		self.addOutPort("human_out_data", self._human_out_dataOut)
		self.addOutPort("Congestion_degree", self._human_wait_numberOut)
		
		return RTC.RTC_OK


	def onActivated(self, ec_id):
		self._first_flag = 1
		print("on Activate")
		return RTC.RTC_OK
	

	def onDeactivated(self, ec_id):
		del self._human		# 各人の歩行情報(構造体)データ解放
		del self._human2	# 各人の歩行情報(構造体)データ解放
		del self._m_data	# 環境データ及び生データ解放
		print("on Deactivate")
		return RTC.RTC_OK

	def onExecute(self, ec_id):
		if self._Range_dataIn.isNew():
			LRFData_IN()					# 新しいデータを取得
			strong_pass_filter(10, 6)		# 密度が低過ぎるデータの有効フラグを折る（ごみ取り）{前後のスキャン範囲,有効データ数}以下のデータが対象
			weak_add_filter(5, 5)			# 密度が高い場所にあるデータの欠落を補完（穴埋め）
			parson_devid_methon(10, 4, 3, 300)	# サブ計測結果として得られたデータをオブジジェェクトごとに分ける
			LRFPocoHuman()						# マッチングによる複数人追跡

			# 表示部分

			Data_outport()

		"""
		def is_green(region):
			return (region >= 50) | (region < 85)

		cap = cv2.VideoCapture(0)
		particles = None

		while cv2.waitKey(30) < 0:
			_, frame = cap.read()
			frame_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV_FULL)
			frame_h = frame_hsv[:, :, 0]
			_, frame_s = cv2.threshold(frame_hsv[:, :, 1], 0, 255, cv2.THRESH_BINARY|cv2.THRESH_OTSU)
			_, frame_v = cv2.threshold(frame_hsv[:, :, 2], 0, 255, cv2.THRESH_BINARY|cv2.THRESH_OTSU)
			frame_h[(frame_s == 0) | (frame_v == 0)] = 0

			particles, x, y = particle_filter(particles, is_green, frame_h)

			if particles is not None:
				valid_particles = particles[(particles[:, 0] >= 0) & (frame.shape[1]) & (particles[:, 1] >= 0) & (particles[:, 1] < frame.shape[0])]

				for i in xrange(valid_particles.shape[0]):
					frame[valid_particles[i][1], valid_particles[i][0]] = [255, 0, 0]
				p = np.array([x, y], dtype = np.int32)
				cv2.rectangle(frame, tuple(p - 15), tuple(p + 15), (0, 0, 255), thickness = 2)
		
			cv2.imshow('green', frame)
	
		cap.release()
		cv2.destroyAllWindows()	
		"""
		return RTC.RTC_OK

# 新データ整理関数
def LRFData_IN(self):
	# 描写用に値を保存
	LRF_start = self._LRFdata_min[0]
	LRF_end = self._LRFdata_max[0]
	LRF_mid = self._LRFdata_mid[0]
	LRF_theta = self._LRFdata_theta[0]
	LRF_step = self._LRF_Total_steps[0]

	count = LRF_start
	# men_data_x = [1100]		# 生データ
	# men_data_x2 = [1100]	# 更新用データ(外周を書く時に使う関数)
	# different_search = []

	self._Range_dataIn.read()
	while count <= LRF_end:
		self._m_data._men_data_x[count] = self._d_Range_data.ranges[count]
		if self._m_data._men_data_x[count] > self._Enviromental_maxdata[0]:
			self._m_data._men_data_x[count] = self._Enviromental_maxdata[0]
		elif self._m_data._men_data_x[count] < 0:
			self._m_data._men_data_x[count] = 0

		if self._m_data._men_data_x2[count] == 0:
			self._m_data._men_data_x2[count] = self._men_data_x[count]
		# 有効データが得られた場合
		elif self._m_data._men_data_x2[count] != 0:
			# 元のデータよりも距離の大きいデータが得られた場合は情報を更新する
			if self._m_data._men_data_x[count] > self._m_data._men_data_x2[count]:
				self._m_data._men_data_x2[count] = self._m_data._men_data_x[count]
			# 元データよりも9割以下の場合は人がいると判断(背景差分法)
			elif self._m_data._men_data_x[count]/self._m_data._men_data_x2[count] < 0.94:
				# データが10cm以下の場合は無視(ノイズ除去のため)
				if self._m_data._men_data_x[count] > 10:
					# データが有効な場所のフラグを立てる(1ステップ毎に行う．1:有効，0:無効)
					self._m_data._m_different_search[count] = 1
				else:
					# 無効データの場合はフラグを折る
					self._m_data._m_different_search[count] = 0
			else:
				self._m_data._m_different_search[count] = 0

		elif self._m_data._men_data_x2[count] != 0 and self._m_data._men_data_x[count] == 0:
			self._m_data._men_data_x[count] = self._m_data._men_data_x2[count]
		# 基本となるデータが0の時
		else:
			if self._m_data._men_data_x[count] == self._Enviromental_maxdata[0]:
				if self._m_data._men_data_x2[count] == self._Enviromental_maxdata[0]:
					self._m_data._m_different_search[count] =0

		count = count + 1

# 密度の低い部分を除外する関数
def strong_pass_filter(self, scan_range, filter_limit):
	filter_on = 0
	filter_count = 1
	i1 = self._LRFdata_min[0]
	# 密度の弱いデータを常にフィルタ処理
	while i1 <= self._LRFdata_max[0]:
		filter_on = 0
		if self._m_data._m_different_search[i1] == 1:
			while filter_count < scan_range:
				if self._m_data._m_different_search[i1 - filter_count] == 1:
					filter_on = filter_on + 1
				elif self._m_data._m_different_search[i1 + filter_count] == 1:
					filter_on = filter_on + 1
				filter_count = filter_count +1

			if filter_on < filter_limit:
				self._m_data._m_different_search[i1] = 0
				self._move_trace[0][0] -= 1
				self._move_trace[0][1] -= i1

# 密度の高い箇所にある欠落データを補完する関数
def weak_add_filter(self, scan_range, filter_limit):
	men_data_x_ave = 0
	i1 = self._LRFdata_min[0]
	# 距離平均値から大きく外れる値のフラグを折る
	while i1 <= self._LRFdata_max[0]:
		if self._m_data._m_different_search[i1] ==1:
			# 各データの前10カウントを検査
			filter_count = 1
			while filter_count <= scan_range:
				men_data_x_ave += self._m_data._men_data_x[i1]
				filter_count = filter_count + 1

			men_data_x_ave = men_data_x_ave / scan_range
			filter_count2 = 1
			while filter_count2 <= scan_range:
				if abs(men_data_x_ave - self._m_data._men_data_x[i1 + filter_count2]) >300:
					self._m_data._m_different_search[i1 + filter_count2] = 0
					filter_count2 = filter_count2 + 1

		i1 = i1 +1

# 複数のオブジェクトを分割する関数
def parson_devid_methon(self, scan_range, filter_limit, delsize, LimitAveDidtance):
	parson_X = 1
	# 背景差分法により得られた1ステップ毎の有効データの連続性を確認
	i1 = self._LRFdata_min[0]
	while i1 <= self._LRFdata_max[0]:
		if self._m_data._m_different_search[i1] == 1:
			human_sx = []
			human_sy = []
			human_ex = []
			human_ey = []
			trans_mendata_to_pxpy(i1)
			human_sx[i1] = self._ppx_d
			human_sy[i1] = self._ppy_d
			trans_mendata_to_pxpy(i1 + 1)
			human_ex[i1 + 1] = self._ppx_d
			humna_ey[i1 + 1] = self._ppy_d
			human_dr[i1] = math.hypot((human_sx[i1] - human_ex[i1 + 1]), (human_sy[i1] - human_ey[i1 + 1]))

			if self._m_data._m_different_search[i1] - self._m_data._m_different_search[i1 - 1] == 1:
				self._flag_check_start[parson_X] = i1
			elif self._m_data._m_different_search[i1] - self._m_data._m_different_search[i1 + 1] == 0:
				if self._human_dr[i1] > 200:
					self._flag_check_end[parson_X] = i1
				elif self._human_dr[i1 - 1] > 200 and (self._m_data._m_different_search[i1] - self._m_data._different_search[i1 - 1] == 0):
					self._flag_check_start[parson_X] = i1
			elif self._m_data._m_different_search[i1] - self._m_data._m_different_search[i1 + 1] ==1:
				if self._human_dr[i1 - 1] > 200:
					self._flag_check_start[parson_X] = 0
				else:
					self._flag_check_end[parson_X] = i1
		
		# ある程度連続した有効データに対して連続した有効ステップの幅を計算
		if self._flag_check_start[parson_X] != 0 and self._flag_check_end[parson_X] != 0 and self._flag_check_end[parson_X] != self._flag_check_start[parson_X] and self._flag_check_end[parson_X] > self._flag_check_start[parson_X] and self._flag_check_start[parson_X] > 0 and self._flag_check_end[parson_X] > 0:
			human_sx = []
			human_sy = []
			human_ex = []
			human_ey = []
			trans_mendata_to_pxpy(self._flag_check_start[parson_X])
			human_sx[parson_X] = self._ppx_d
			human_sy[parson_X] = self._ppy_d
			trans_mendata_to_pxpy(self._flag_check_end[parson_X])
			human_ex[parson_X] = self._ppx_d
			humna_ey[parson_X] = self._ppy_d
			human_dr[parson_X] = math.hypot((human_sx[parson_X] - human_ex[parson_X]), (human_sy[parson_X] - human_ey[parson_X]))
			# ある程度の幅(コンフィギュレーションパラメータで変更可能)移動で物体と認識
			if parson_X <= self._Human_NUM[0]:
				if self._human_dr[parson_X] > self._Human_width[0]:
					i = self._flag_check_start[parson_X]
					while i <= self._flag_check_end[parson_X]:
						self._m_data._parson_divide[parson_X][i] = 1
						i = i + 1
					self._move_trace[parson_X][0] = self._flag_check_start[parson_X]
					self._move_trace[parson_X][1] = self._flag_check_end[parson_X]
					self._flag_check_start[parson_X] = 0
					self._flag_check_end[parson_X] = 0
					parson_X = parson_X + 1
				
				# ある程度の幅(コンフィギュレーションパラメータで変更可能)以下であれば有効ステップデータを無効にする
				else:
					i = self._flag_check_start[parson_X]
					while i <= self._flag_check_end[parson_X]:
						self._m_data._parson_divide[parson_X][i] = 0
						self._m_data._m_different_search[i] = 0
						i = i + 1
					self._flag_check_start[parson_X] = 0
					self._flag_check_end[parson_X] = 0
					self._move_trace[parson_X][0] = 0
					self._move_trace[parson_X][1] = 0
		i = i +1

# X,Y座標に変換する関数
def trans_mendata_to_pxpy(self, count):
	# ラジアン単位の角度算出
	self._theta = (count - self._LRFdata_mid)*self._LRFdata_theta/360*2*math.pi/self._LRF_Total_steps
	# データをひとまず距離として保管
	self._G_dist = self._m_data._men_data_x[count]
	self._ppx_d = self._G_dist*math.sin(self._theta)
	self._ppy_d = self._G_dist*math.cos(self._theta)

# 複数人追跡関数"重要"
def LRFPocoHuman(self):
	# 計測データの更新　対象物1からMAX10人に対して繰り返し計算
	HUMAN_STATE_CALCULATE(self._m_qHuman)
	# 追跡処理
	TARGET_TRACKING(self._m_qHuman)
	self._m_human_wait = 0

	i = 1
	while i <= self._Human_NUM:
		# パーティクルフィルタによるロックオン処理
		resample(i, self._human, self._human2)
		predict(i, self._human)
		weight(i, self._human)
		measure(i, self._human)
		# パーティクルフィルタによるロックオン処理ここまで

		# 移動距離計算と入退室カウント
		HUMAN_MIGRATION_LENGTH(i, self._human)
		# 入退室後や長時間残っている人データを削除する
		HUMAN_DELETE_COUNT(i, self._human)
		i = i + 1

	# 入退室後や長時間残っている人データを削除するためにカウントを行う。また、追跡エリア以外で速度が小さいものも削除対象
	HUMAN_DELETE(self._human)

# 初期化関数(途中)
def Human_Initialize(self):
	self._m_qHuman = object.__new__(struct_human)
	self._m_qHuman2 = object.__new__(struct_human)
	self._m_data = object.__new__(struct_Data)

	self._ppx_d = 0
	self._ppy_d = 0
	self._theta = 0
	self._G_dist = 0

	i = 0
	while i <= self._Human_NUM:
		s2 = 0
		while s2 < self._Human_NUM:
			self._Tracking_check[i][s2] = 0
			self._m_qHuman = 0

# 更新データを出力する関数
def Data_outport(self):
	# human_dataXから出力する
	humanX = []
	for v in range(self._Human_NUM[0]):
		value_X = self._human[v + 1]._m_qHuman.y3[1]
		print("data_X:%d" %(value_X))
		humanX.append(value_X)
	self._d_human_dataX.data = humanX
	self._human_dataXOut.write()

	# human_dataYから出力する
	humanY = []
	for v in range(self._Human_NUM[0]):
		value_Y = self._human[v + 1]._m_qHuman.x3[1]
		print("data_Y:%d" %(value_Y))
		humanY.append(value_Y)
	self._d_human_dataY.data = humanY
	self._human_dataYOut.write()

	# human_dataVから出力する
	humanV = []
	for v in range(self._Human_NUM[0]):
		value_V = self._human[v + 1]._m_qHuman.velocity[1]
		print("data_V:%d" %(value_V))
		humanV.append(value_V)
	self._d_human_dataV.data = humanV
	self._human_dataVOut.write()

	# human_dataTから出力する
	humanT = []
	for v in range(self._Human_NUM[0]):
		value_T = self._human[v + 1]._m_qHuman.vector[1]
		print("data_T:%d" %(value_T))
		humanT.append(value_T)
	self._d_human_dataT.data = humanT
	self._human_dataTOut.write()

	# 右方向からの人数、左方向からの人数、合計人数の出力
	self._d_human_in_data.data = self._m_human_in				# 右からの人数
	self._d_human_out_data.data = self._m_human_out				# 左からの人数
	hcount_sum = self._m_human_in + self._m_human_out		# 左右の合計人数
	if hcount_sum > 0:
		print("右方向からの人数: %d" %self._d_human_in_data.data)
		print("左方向からの人数: %d" %self._d_human_out_data.data)
		print("合計人数: %d" %hcount_sum)
		self._human_in_dataOut.write()
		self._human_out_dataOut.write()

	elif self._first_flgA == 1:
		self._first_flgA = 0
		stime = time.time()		# 基準時間
	else:
		etime = time.time()
		dtime = etime - stime
		if dtime >= 0.2*60*1000:
			self._human_in_dataOut.write()
			self._human_out_dataOut.write()
			self._human_wait_numberOut.write()
			print("右方向からの人数: %d" %self._d_human_in_data.data)
			print("左方向からの人数: %d" %self._d_human_out_data.data)
			print("混雑度: %d" %self._d_human_wait_number.data)
			self._human_all = 0
			stime = etime

	if self._first_flg == 1:
		self._first_flg = 0
		stime = time.time()
	else:
		etime = time.time()
		dtime = etime - stime
		if dtime >= 0.2*60*1000:
			self._first_flg = 1
			self._human_all = 0
			stime = etime
	
	for i in range(self._Human_NUM[0]):
		for j in range(self._LRFdata_max[0]):
			self._m_data._parson_divide[i][j] = 0
		self._flag_check_start[i] = 0
		self._flag_check_end[i] = 0
		self._move_trace[i][0] = 0
		self._move_trace[i][1] = 0
	
	self._human_wait = 0

# 人ID振り分け関数
def TARGET_TRACKING(self, struct_MAN):
	"""
	ここでは複数人の追跡を行うための処理を行っています．LRFから得られる物体データを生データ，人認識後の位置データを人データとします．
	まず，LRFの更新によって変動する各生データと各人の人データの距離を算出します．
	次に，それぞれのデータを比較して最短距離の生データと人データの組み合わせを検索します．
	最後に，見つかった組み合わせの人データに生データの値を入れ，更新します．
	この作業を人数分行い，複数人を追跡します．
	"""
	# 設定変数
	i = self._NUM_Hmax
	HUMAN_PRIME_ID = [0,2,3,5,7,11,13,17,19,23,29]					# 各素数(HUMAN_PRIME_ID[0]は使わないので0を代入)
	dist_check1 = [[0 for j in range(i)] for k in range(i)]			# 各人と各生データとの距離記録用
	hum_comp = [0 for j in range(i)]								# マッチング前の生データの場所保存用
	hum_comp2 = 0													# マッチング前の人データの場所保存用
	c_flag = [0 for j in range(i)]									# マッチング後の生データの場所保存用
	c_flag2 = [0 for j in range(i)]									# マッチング後の人データの場所保存用
	set_flag = [0 for j in range(i)]								# マッチング後の生データの場所保存用
	set_flag2 = [0 for j in range(i)]								# マッチング後の人データの場所保存用
	Tracking_check2 = [[0 for j in range(i)] for k in range(i)]		# マッチング後の人データ及び生データの場所保存用
	Hum_ID = [1 for j in range(i)]									# どのIDを使っているか記録用(1:追跡していない　素数:追跡中)
	HumID = 0

	# 追跡している人を判別
	for t in range(self._Human_NUM[0]):
		Hum_ID[t] = self._human._m_qHuman.Hum_ID2

	"""
	生データと人データマッチング
	"""
	for t2 in range(self._Human_NUM[0]):
		if Hum_ID[t2] != 1:
			for v in range(self._Human_NUM[0]):
				if self._human[v]._m_qHuman.x[1] != 0 and self._human[v]._m_qHuman.y[1] != 0:
					dist_check1[t2][v] = math.hypot((self._human[t2]._m_qHuman.x2[1] - self._human[v]._m_qHuman.x[1]), (self._human[t2]._m_qHuman.y2[1] - self._human[v]._m_qHuman.y[1]))		# 各人と各生データとの距離を記録
	for t3 in range(self._Human_NUM[0]):
		# 回数
		dist_hum = [9999 for j in range(self._NUM_Hmax)]
		# 人
		for v2 in range(self._Human_NUM[0]):
			# 生データ
			for sti in range(self._Human_NUM[0]):
				if dist_check1[v2][sti] != 0 and set_flag[sti] != True and set_flag2[v2] != True:
					if dist_hum[v2] > dist_check1[v2][sti]:
						# 各人に対して最短の生データを検索し生データの位置を記録
						dist_hum[v2] = dist_check1[v2][sti]
						hum_comp[v2] = sti
		dist_hum2 = 9999
		for v3 in range(self._Human_NUM[0]):
			if dist_hum[v3] != 0:
				# 各人と生データが最小距離の組を検索
				if dist_hum2 > dist_hum[v3]:
					dist_hum2 = dist_hum[v3]
					hum_comp2 = v3
		if dist_hum2 != 9999:
			# マッチング後の人データ更新
			self._human[hum_comp2]._m_qHuman.xp2[1] = self._human[hum_comp[hum_comp2]].x[1]
			self._human[hum_comp2]._m_qHuman.yp2[1] = self._human[hum_comp[hum_comp2]].y[1]
			# 更新時間の保存(速度算出時に使用)
			self._human[hum_comp2]._m_qHuman.clock_end = self._human[hum_comp2]._m_qHuman.clock_start
			self._human[hum_comp2]._m_qHuman.clock_start = time.time()
			if self._human[hum_comp2]._m_qHuman.x3[1] != self._human[hum_comp2]._m_qHuman.x3[2] and self._human[hum_comp2]._m_qHuman.x3[1] != self._human[hum_comp2]._m_qHuman.x[1]:
				# マッチング後の人データを保存
				arrayInsert(self._human[hum_comp2]._m_qHuman.x3, 10, self._human[hum_comp2]._m_qHuman.x[1], 1)
				# マッチング後の人データを保存
				arrayInsert(self._human[hum_comp2]._m_qHuman.y3, 10, self._human[hum_comp2]._m_qHuman.y[1], 1)
			set_flag[hum_comp[hum_comp2]] = True	# 使用中の生データの場所更新
			set_flag2[hum_comp2] = True				# 使用中の人データの場所更新
			Tracking_check2[hum_comp2][hum_comp[hum_comp2]] = True	# 使用中のデータの場所更新
	for t4 in range(self._Human_NUM[0]):
		c_flag[t4] = set_flag[t4]
		c_flag2[t4] = set_flag[t4]
		Human_check[t4] = set_flag2[t4]
		for s in range(self._Human_NUM[0]):
			self._Tracking_check[t4][s] = Tracking_check2[t4][s]
	"""
	生データと人データマッチングここまで
	"""
	"""
	新しい値が入ってきたとき
	"""
	# 空いているIDに数値を入れていく
	for sti2 in range(self._Human_NUM[0]):
		# コンフィギュレーションパラメーターHuman_setdistanceで設定した値以上の場所で物体を検出した場合，人データの値を更新
		if self._human[sti2]._m_qHuman.y[1] != 0 and math.hypot(self._human[sti2]._m_qHuman.y[1], self._human[sti2]._m_qHuman.x[1]) > self._Human_setdistance[0] and math.hypot(self._human[sti2]._m_qHuman.y[1], self._human[sti2]._m_qHuman.x[1]) < self._Enviromental_maxdata[0] and c_flag[sti2] != True:
			# 使われていない値を探す(1以外の使われていない素数を探す
			t5 = PrimeNum_Search(Hum_ID, 0)
			if Hum_ID[t5] == 1:
				for i2 in range(self._max_particles):
					self._human[t5]._m_qHuman.xp[i2] = self._human[sti2]._m_qHuman.x[1]		# パーティクルのデータを更新
					self._human[t5]._m_qHuman.yp[i2] = self._human[sti2]._m_qHuman.y[1]		# パーティクルのデータを更新
				self._human[t5]._m_qHuman.x2[1] = self._human[sti2]._m_qHuman.x[1] + 1		# 人データを更新
				self._human[t5]._m_qHuman.y2[1] = self._human[sti2]._m_qHuman.y[1] + 1		# 人データを更新
				self._human[t5]._m_qHuman.x2[0] = self._human[sti2]._m_qHuman.x[1] 			# 初期位置x座標保管
				self._human[t5]._m_qHuman.x2[0] = self._human[sti2]._m_qHuman.y[1] 			# 初期位置y座標保管
				arrayInsert(self._human[t5]._m_qHuman.x3, 10, self._human[sti2]._m_qHuman.x[1], 1)	# 人データを保存
				arrayInsert(self._human[t5]._m_qHuman.y3, 10, self._human[sti2]._m_qHuman.y[1], 1)	# 人データを保存
				Hum_ID[t5] = PrimeNum_Search(Hum_ID, 1)		# 使っていない一番小さなID（素数）を入れる
				c_flag[sti2] = True
				c_flag2[t5] = True
	"""
	新しい値が入ってきたときここまで
	"""
	for t6 in range(self._Human_NUM[0]):
		# 追跡している人を更新
		self._human[t6]._m_qHuman.Hum_ID2 = Hum_ID[t6]

# 仮に計測データを記録する関数
def HUMAN_STATE_CALCULATE(self, struct_MAN):
	# 物体と認識された有効ステップの中間ステップでLRFに対しての距離と角度を算出
	for i in range(self._Human_NUM[0]):
		G_center_angl = [0 for j in range(self._NUM_Hmax)]
		theta_other = [0 for j in range(self._NUM_Hmax)]
		G_dist_other = [0 for j in range(self._NUM_Hmax)]
		# 目標の代表方向にあたる角度をラジアンで計算
		G_center_angl[i] = (self._move_trace[i][1] + self._move_trace[i][0]) / 2
		theta_other[i] = (G_center_angl[i] - self._LRFdata_mid[0])*self._LRFdata_theta[0]/360*2*math.pi/self._LRF_Total_steps[0]
		if G_center_angl[i] != 0:
			# 代表方向を示す線の距離を設定
			G_dist_other[i] = self._m_data._men_data_x[G_center_angl[i]]
		else:
			G_dist_other[i] = 0
	for i2 in range(self._Human_NUM[0]):
		# 算出した計測データを記録（過去データを保存可能．[1]：最新データ）
		# 極座標表示
		arrayInsert(struct_MAN[i2]._m_qHuman.theta, 100, -self._theta_other[i2]/math.pi*180, 1)
		arrayInsert(struct_MAN[i2]._m_qHuman.distance, 100, G_dist_other[i2], 1)
		# 直行座標（生データ）
		arrayInsert(struct_MAN[i2]._m_qHuman.x, 100, G_dist_other[i2]*math.cos(theta_other[i2]), 1)
		arrayInsert(struct_MAN[i2]._m_qHuman.y, 100, G_dist_other[i2]*math.sin(theta_other[i2]), 1)

# 移動距離計測および人数カウント関数
def HUMAN_MIGRATION_LENGTH(self, i, struct_MAN):
	if struct_MAN[i]._m_qHuman.x3[1] != 100000:
		# LRFから各人の距離を算出
		arrayInsert(struct_MAN[i]._m_qHuman.distance2, 100, math.hypot(struct_MAN[i]._m_qHuman.y2[1], struct_MAN[i]._m_qHuman.x2[1]), 1)
		# LRFから各人の角度を算出
		arrayInsert(struct_MAN[i]._m_qHuman.theta2, 100, -math.atan2(struct_MAN[i]._m_qHuman.y2[1], struct_MAN[i]._m_qHuman.x2[1])/math.pi*180, 1)
	# 追跡処理後の直行座標データから速さと向きを計算
	if struct_MAN[i]._m_qHuman.x3[1] != 100000 and struct_MAN[i]._m_qHuman.x3[1] < self._Enviromental_maxdata[0] and struct_MAN[i]._m_qHuman.y3[1] < self._Enviromental_maxdata[0]:
		pot = math.hypot((struct_MAN[i]._m_qHuman.x3[1] - struct_MAN[i]._m_qHuman.x3[2]) / ((struct_MAN[i]._m_qHuman.clock_start - struct_MAN[i]._m_qHuman.clock_end) / 1000), (struct_MAN[i]._m_qHuman.y3[1] - struct_MAN[i]._m_qHuman.y3[2]) / ((struct_MAN[i]._m_qHuman.clock_start - struct_MAN[i]._m_qHuman.clock_end) / 1000))
		if pot < 2000:
			arrayInsert(struct_MAN[i]._m_qHuman.velocity, 10, pot, 1)
		
		if struct_MAN[i]._m_qHuman.velocity[1] > 300 and struct_MAN[i]._m_qHuman.velocity[1] < 2000:
			arrayInsert(struct_MAN[i]._m_qHuman.vector, 10, -math.atan2((struct_MAN[i]._m_qHuman.y3[1] - struct_MAN[i]._m_qHuman.y3[2]), (struct_MAN[i]._m_qHuman.x3[1] - struct_MAN[i]._m_qHuman.x3[2])) / math.pi*180, 1)

		if struct_MAN[i]._m_qHuman.x3[1] != 100000 and struct_MAN[i]._m_qHuman.x3[1] != 0 and struct_MAN[i]._m_qHuman.x3[1] < self._Enviromental_maxdata[0] and struct_MAN[i]._m_qHuman.y3[1] < self._Enviromental_maxdata[0]:
			# x方向総移動量
			self._distance_xy[i][1] = struct_MAN[i]._m_qHuman.x3[1] - struct_MAN[i]._m_qHuman.x3[2]
			# y方向総移動量
			self._distance_xy[i][2] = struct_MAN[i]._m_qHuman.y3[1] - struct_MAN[i]._m_qHuman.y3[2]
			# 0.1秒間の移動量
			self._distance_S[i] = math.sqrt(self._distance_xy[i][1]**2 + self._distance_xy[i][2]**2)
			if struct_MAN[i]._m_qHuman.velocity[1] > 300 and struct_MAN[i]._m_qHuman.velocity[1] < 2000 and struct_MAN[i]._m_qHuman.velocity[1] != struct_MAN[i]._m_qHuman.velocity[2]:
				self._distance_S2[i] += self._distance_S[i]
	# 人数カウント部分，コンフィギュレーションパラメーターCount_modeを変更することで入退室の向きを変更可能
	if self._Count_mode[0] == "IN,OUT":
		if struct_MAN[i]._m_qHuman.distance2[1] > self._Count_area_distance[0] and struct_MAN[i]._m_qHuman.distance2[1] < self._Enviromental_maxdata[0] and struct_MAN[i]._m_qHuman.theta2[1] > 0 and struct_MAN[i]._m_qHuman.Msg_Lv[2] != 3:
			struct_MAN[i]._m_qHuman.Msg_Lv[2] = 1
		elif struct_MAN[i]._m_qHuman.distance2[1] > self._Count_area_distance[0] and struct_MAN[i]._m_qHuman.distance2[1] < self._Enviromental_maxdata[0] and struct_MAN[i]._m_qHuman.theta2[1] < 0 and struct_MAN[i]._m_qHuman.Msg_Lv[2] != 3:
			struct_MAN[i]._m_qHuman.Msg_Lv[2] = 2

		if struct_MAN[i]._m_qHuman.Msg_Lv[2] == 1 and struct_MAN[i]._m_qHuman.distance2[1] > self._Count_area_distance[0] - 500 and struct_MAN[i]._m_qHuman.distance2[1] < self._Enviromental_maxdata[0] and struct_MAN[i]._m_qHuman.theta2[1] < 0:
			struct_MAN[i]._m_qHuman.Msg_Lv[2] = 3
		elif struct_MAN[i]._m_qHuman.Msg_Lv[2] == 2 and struct_MAN[i]._m_qHuman.distance2[1] > self._Count_area_distance[0] - 500 and struct_MAN[i]._m_qHuman.distance2[1] < self._Enviromental_maxdata[0] and struct_MAN[i]._m_qHuman.theta2[1] > 0:
			struct_MAN[i]._m_qHuman.Msg_Lv[2] = 3

	elif self._Count_mode[0] == "OUT,IN":
		if struct_MAN[i]._m_qHuman.distance2[1] > self._Count_area_distance[0] and struct_MAN[i]._m_qHuman.distance2[1] < self._Enviromental_maxdata[0] and struct_MAN[i]._m_qHuman.theta2[1] < 0 and struct_MAN[i]._m_qHuman.Msg_Lv[2] != 3:
			struct_MAN[i]._m_qHuman.Msg_Lv[2] = 1
		elif struct_MAN[i]._m_qHuman.distance2[1] > self._Count_area_distance[0] and struct_MAN[i]._m_qHuman.distance2[1] < self._Enviromental_maxdata[0] and struct_MAN[i]._m_qHuman.theta2[1] > 0 and struct_MAN[i]._m_qHuman.Msg_Lv[2] != 3:
			struct_MAN[i]._m_qHuman.Msg_Lv[2] = 2

		if struct_MAN[i]._m_qHuman.Msg_Lv[2] == 1 and struct_MAN[i]._m_qHuman.distance2[1] > self._Count_area_distance[0] - 500 and struct_MAN[i]._m_qHuman.distance2[1] < self._Enviromental_maxdata[0] and struct_MAN[i]._m_qHuman.theta2[1] > 0:
			self._d_human_in_data.data = self._d_human_in_data.data + 1
			struct_MAN[i]._m_qHuman.Msg_Lv[2] = 3
		elif struct_MAN[i]._m_qHuman.Msg_Lv[2] == 2 and struct_MAN[i]._m_qHuman.distance2[1] > self._Count_area_distance[0] - 500 and struct_MAN[i]._m_qHuman.distance2[1] < self._Enviromental_maxdata[0] and struct_MAN[i]._m_qHuman.theta2[1] < 0:
			self._d_human_out_data.data = self._d_human_out_data.data + 1
			struct_MAN[i]._m_qHuman.Msg_Lv[2] = 3
		
		if struct_MAN[i]._m_qHuman.Msg_Lv[2] == 1 and struct_MAN[i]._m_qHuman.Msg_Lv[2] == 2 and struct_MAN[i]._m_qHuman.distance2[1] > self._Count_area_distance[0] - 500 and struct_MAN[i]._m_qHuman.distance2[1] < self._Enviromental_maxdata[0] and struct_MAN[i]._m_qHuman.theta2[1] < 0 and struct_MAN[i]._m_qHuman.theta2[1] > 0:
			self._human_all = self._human_all + 1
	else:
		if struct_MAN[i]._m_qHuman.distance2[1] > self._Count_area_distance[0] and struct_MAN[i]._m_qHuman.distance2[1] < self._Enviromental_maxdata[0] and struct_MAN[i]._m_qHuman.theta2[1] > 0 and struct_MAN[i]._m_qHuman.Msg_Lv[2] != 3:
			struct_MAN[i]._m_qHuman.Msg_Lv[2] = 1
		elif struct_MAN[i]._m_qHuman.distance2[1] > self._Count_area_distance[0] and struct_MAN[i]._m_qHuman.distance2[1] < self._Enviromental_maxdata[0] and struct_MAN[i]._m_qHuman.theta2[1] < 0 and struct_MAN[i]._m_qHuman.Msg_Lv[2] != 3:
			struct_MAN[i]._m_qHuman.Msg_Lv[2] = 2

		if struct_MAN[i]._m_qHuman.Msg_Lv[2] == 1 and struct_MAN[i]._m_qHuman.distance2[1] > self._Count_area_distance[0] - 500 and struct_MAN[i]._m_qHuman.distance2[1] < self._Enviromental_maxdata[0] and struct_MAN[i]._m_qHuman.theta2[1] < 0:
			self._d_human_in_data.data = self._d_human_in_data.data + 1
		elif struct_MAN[i]._m_qHuman.Msg_Lv[2] == 2 and struct_MAN[i]._m_qHuman.distance2[1] > self._Count_area_distance[0] - 500 and struct_MAN[i]._m_qHuman.distance2[1] < self._Enviromental_maxdata[0] and struct_MAN[i]._m_qHuman.theta2[1] > 0:
			self._d_human_out_data.data = self._d_human_out_data.data + 1
			struct_MAN[i]._m_qHuman.Msg_Lv[2] = 3

		if struct_MAN[i]._m_qHuman.Msg_Lv[2] == 1 and struct_MAN[i]._m_qHuman.Msg_Lv[2] == 2 and struct_MAN[i]._m_qHuman.distance2[1] > self._Count_area_distance[0] - 500 and struct_MAN[i]._m_qHuman.distance2[1] < self._Enviromental_maxdata[0] and struct_MAN[i]._m_qHuman.theta2[1] < 0 and struct_MAN[i]._m_qHuman.theta2[1] > 0:
			self._human_all = self._human_all + 1
			struct_MAN[i]._m_qHuman.Msg_Lv[2] = 3
	# 滞在人数をカウント
	if struct_MAN[i]._m_qHuman.distance2[1] > 0 and struct_MAN[i]._m_qHuman.distance2[1] < 3000:
		self._m_human_wait = self._m_human_wait + 1
	# 混雑度の定義
	if 0 <= self._m_human_wait and self._m_human_wait <= 3:
		self._d_human_wait_number.data = 1		# 混雑度：小
	elif 4 <= self._m_human_wait and self._m_human_wait <= 7:
		self._d_human_wait_number.data = 2		# 混雑度：中
	else:
		self._d_human_wait_number.data = 3		# 混雑度：大

# 入退室後や長時間残留する人データの残留時間をカウントする関数
def HUMAN_DELETE_COUNT(self, i, struct_MAN):
	if struct_MAN[i]._m_qHuman.velocity[1] == struct_MAN[i]._m_qHuman.velocity[2] and struct_MAN[i]._m_qHuman.distance2[1] < self._Enviromental_maxdata[0]:
		struct_MAN[i]._m_qHuman.hum_count[1] = struct_MAN[i]._m_qHuman.hum_count[1] + 1
	if struct_MAN[i]._m_qHuman.distance2[1] > 2500 and struct_MAN[i]._m_qHuman.distance2[1] < self._Enviromental_maxdata[0] and struct_MAN[i]._m_qHuman.velocity[1] < 180:
		struct_MAN[i]._m_qHuman.hum_count[1] = struct_MAN[i]._m_qHuman.hum_count[1] + 1
	elif struct_MAN[i]._m_qHuman.distance2[1] < 2500:
		struct_MAN[i]._m_qHuman.hum_count[1] = 0
	elif struct_MAN[i]._m_qHuman.velocity[1] > 180 and struct_MAN[i]._m_qHuman.velocity[1] != struct_MAN[i]._m_qHuman.velocity[2]:
		struct_MAN[i]._m_qHuman.hum_count[1] = 0
	
	if struct_MAN[i]._m_qHuman.distance2[1] < 2500 and struct_MAN[i]._m_qHuman.velocity[1] < 180:
		struct_MAN[i]._m_qHuman.hum_count[2] = struct_MAN[i]._m_qHuman.hum_count[2] + 1
	elif struct_MAN[i]._m_qHuman.velocity[1] > 180:
		struct_MAN[i]._m_qHuman.hum_count[2] = 0
	
	if struct_MAN[i]._m_qHuman.distance2[1] > self._Enviromental_maxdata[0]:
		struct_MAN[i]._m_qHuman.hum_count[3] = struct_MAN[i]._m_qHuman.hum_count[3] + 1
	if struct_MAN[i]._m_qHuman.velocity[1] == struct_MAN[i]._m_qHuman.velocity[2] and struct_MAN[i]._m_qHuman.distance2[1] < self._Enviromental_maxdata[0]:
		struct_MAN[i]._m_qHuman.hum_count[4] = struct_MAN[i]._m_qHuman.hum_count[4] + 1
	if struct_MAN[i]._m_qHuman.x3[1] < 0 and struct_MAN[i]._m_qHuman.velocity[1] < 180:
		struct_MAN[i]._m_qHuman.hum_count[4] = struct_MAN[i]._m_qHuman.hum_count[4] + 1
	elif struct_MAN[i]._m_qHuman.x3[1] > 0:
		struct_MAN[i]._m_qHuman.hum_count[4] = 0
	elif struct_MAN[i]._m_qHuman.velocity[1] > 180 and struct_MAN[i]._m_qHuman.velocity[1] != struct_MAN[i]._m_qHuman.velocity[2]:
		struct_MAN[i]._m_qHuman.hum_count[4] = 0
	
	if struct_MAN[i]._m_qHuman.x2[1] < 0:
		struct_MAN[i]._m_qHuman.hum_count[5] = struct_MAN[i]._m_qHuman.hum_count[5] + 1
	elif struct_MAN[i]._m_qHuman.x2[1] > 0:
		struct_MAN[i]._m_qHuman.hum_count[5] = 0
	
	if self._distance_S2[i] > 5000 and struct_MAN[i]._m_qHuman.Msg_Lv[2] != 0 and struct_MAN[i]._m_qHuman.distance2[1] > 3500:
		struct_MAN[i]._m_qHuman.hum_count[6] = struct_MAN[i]._m_qHuman.hum_count[6] + 1
	elif struct_MAN[i]._m_qHuman.distance2[1] < 4000:
		struct_MAN[i]._m_qHuman.hum_count[6] = 0

# 人データを削除する関数
def HUMAN_DELETE(self, struct_MAN):
	for i in range(self._Human_NUM):
		if struct_MAN[i]._m_qHuman.hum_count[1] >= 10 or struct_MAN[i]._m_qHuman.hum_count[3] > 10 or struct_MAN[i]._m_qHuman.hum_count[4] > 20 or struct_MAN[i]._m_qHuman.hum_count[5] > 10 or struct_MAN[i]._m_qHuman.hum_count[6] > 10:
			for s2 in range(self._Human_NUM):
				struct_MAN[i]._m_qHuman.hum_count[s2] = 0
				self._distance_S2[i] = 0
				self._flag_check_start[i] = 0
				self._flag_check_end[i] = 0
			for i2 in range(self._max_particles):
				struct_MAN[i]._m_qHuman.weight[i2] = 0
				struct_MAN[i]._m_qHuman.xp[i2] = 100000
				struct_MAN[i]._m_qHuman.yp[i2] = 100000
				struct_MAN[i]._m_qHuman.x2[i2] = 100000
				struct_MAN[i]._m_qHuman.y2[i2] = 100000
				struct_MAN[i]._m_qHuman.y3[i2] = 100000
				struct_MAN[i]._m_qHuman.distance2[i2] = 100000
				struct_MAN[i]._m_qHuman.vector[i2] = 0
				struct_MAN[i]._m_qHuman.velocity[i2] = 0
			for count in range(self._LRFdata_min[0], self._LRFdata_max[0] + 1):
				self._m_data._m_different_search[count] = 0

			struct_MAN[i]._m_qHuman.Hum_ID2 = 1
			struct_MAN[i]._m_qHuman.Msg_Lv[1] = 0
			struct_MAN[i]._m_qHuman.Msg_Lv[2] = 0
			struct_MAN[i]._m_qHuman.Msg_Lv[3] = 0
			struct_MAN[i]._m_qHuman.theta[0] = 0
			struct_MAN[i]._m_qHuman.theta2[0] = 0
			struct_MAN[i]._m_qHuman.distance[0] = 0

# 計測周期ごとのデータを残す関数/一番古いデータを消して新しい数値に更新する関数（[1]:最新データ）
def arrayInsert(data, n, newdata, k):
	i = n
	while i > k:
		data[i] = data[i - 1]
		i = i -1
	data[k] = newdata

# 未使用ID(素数)探索関数
def PrimeNum_Search(self, HUMAN_ID, MODE):
	ANS = 1
	SEARCH_NUM = [0 for i in range(self._NUM_Hmax)]
	# 検索する素数[0]は検索しない
	SEARCH_NUM = [0, 2, 3, 5, 7, 11, 13, 17, 19, 23, 29]
	# 発見した数を入れる関数
	Discovery_NUM = 0
	# 全てのIDを掛ける
	for i in range(1, self._NUM_Hmax + 1):
		if ANS * HUMAN_ID[i] != 0:
			ANS *= HUMAN_ID[i]
	# 素数検索
	for i2 in range(1, self._NUM_Hmax + 1):
		if ANS * SEARCH_NUM[i2] != 0:
			if MODE == 0:
				# 存在しない素数で一番小さい値を記録（同時に検索することも可能）
				Discovery_NUM = i2
			else:
				# 存在しない素数で一番小さい値IDを記録（同時に検索することも可能）
				Discovery_NUM = SEARCH_NUM[i2]
				break
	# 使われていないIDで一番小さいのを返す
	return Discovery_NUM

# ログ記録
# ログ入力部(ファイルに直接書き込み-汎用版)
def Logsave_Input(ID, File_Name, log1, log2, log3, log4, log5, log6, log7, log8):
	t_now = datetime.datetime.now().time()
	log_data = [t_now, log1, log2, log3, log4, log5, log6, log7, log8]
	with open(str(File_Name), 'w', newline='') as f:
		writer = csv.writer(f)
		writer.writerow(['現在時刻', 'log1', 'lod2', 'log3', 'log4', 'log5', 'log6', 'log7', 'log8'])
		for i in range(0, len(log_data)):
			writer.writerow([log_data[i], log_data[i + 1]])

# CSV読み込み　(ＯＮにするときはonActivated内の関数を使う)
def CSV_CHECK():
	with open('log_data1-3.csv', 'r') as f:
		reader = csv.reader(f)
		header = next(reader)

		for row in reader:
			print row

"""
	パーティクルフィルタ関係
"""
# パーティクルフィルタによるロックオン処理
def resample(self, i, struct_MAN, struct_MAN2):
	# 累積重みの計算
	all_weight = [0.0 for j in range(self._max_particles)]
	all_weight[0] = struct_MAN[i]._m_pHuman.weight[0]
	for i2 in range(1, self._max_particles):
		all_weight[i2] = all_weight[i2 - 1] + struct_MAN[i]._m_qHuman.weight[i2]
	
	# 重みを基準にパーティクルをリサンプリングして重みを1.0に
	for i3 in range(self._max_particles):
		struct_MAN2[i]._m_qHuman2.weight[i3] = struct_MAN[i]._m_qHuman.weight[i3]
		struct_MAN2[i]._m_qHuman2.xp[i3] = struct_MAN[i]._m_qHuman.xp[i3]
		struct_MAN2[i]._m_qHuman2.yp[i3] = struct_MAN[i]._m_qHuman.yp[i3]
	for i4 in range(self._max_particles):
		cut_rate = 0.6
		rate = randint(1, 100) / 100
		if rate < cut_rate:
			rate += (1.0 - cut_rate)
		weight = rate*all_weight[self._max_particles - 1]
		n = 0
		while all_weight[++n] < weight:
			struct_MAN[i]._m_qHuman.weight[i4] = struct_MAN2[i]._m_qHuman2.weight[n]
			struct_MAN[i]._m_qHuman.xp[i4] = struct_MAN2[i]._m_qHuman2.xp[n]
			struct_MAN[i]._m_qHuman.yp[i4] = struct_MAN2[i]._m_qHuman2.yp[n]
		struct_MAN[i]._m_qHuman.weight[i4]

def predict(self, i, struct_MAN):
	variance = 0.0
	sum = 0
	# この値を移動距離の積算と共に増大させよう
	if struct_MAN[i]._m_qHuman.Hum_ID2 != 1:
		for s in range(1, self._Human_NUM[0]):
			sum += self._Tracking_check[i][s]
		if sum == 0:
			# 位置の予測
			for i2 in range(self._max_particles):
				if struct_MAN[i]._m_qHuman.velocity[1] > 300 and struct_MAN[i]._m_qHuman.distance2[1] < self._Human_TrackingData:
					rate = 0.0
					rate = randint(1, 100) - 20
					vx2 = struct_MAN[i]._m_qHuman.velocity[1] * ((struct_MAN[i]._m_qHuman.clock_start - struct_MAN[i]._m_qHuman.clock_end) / 1000)*math.cos(self._human[i]._m_qHuman.vector[1] * math.pi / 180.0) / 20.0 * rate
					vx3 = - struct_MAN[i]._m_qHuman.velocity[1] * ((struct_MAN[i]._m_qHuman.clock_start - struct_MAN[i]._m_qHuman.clock_end) / 1000)*math.sin(self._human[i]._m_qHuman.vector[1] * math.pi / 180.0) / 20.0 * rate
				else:
					variance = 4.0
					rate = 0.0
					rate = randint(1, 200) - 100
					vx2 = variance * rate
					vx3 = variance * rate
					struct_MAN[i]._m_qHuman.xp[i2] += vx2
					struct_MAN[i]._m_qHuman.yp[i2] += vx3
		elif sum == 1:
			if struct_MAN[i]._m_qHuman.velocity[1] < 200:
				variance = 1.0
			elif 200 < struct_MAN[i]._m_qHuman.velocity[1]:
				variance = 2.0
			
			if struct_MAN[i]._m_qHuman.velocity[1] > 700 or struct_MAN[i]._m_qHuman.velocity[1] == struct_MAN[i]._m_qHuman.velocity[2]:
				variance = 4.0
			# 位置の予測
			for i3 in range(self._max_particles):
				rate = 0.0
				rate = randint(1, 200) - 100
				vx2 = variance * rate
				vx3 = variance * rate
				struct_MAN[i]._m_qHuman.xp[i3] += vx2
				struct_MAN[i]._m_qHuman.yp[i3] += vx3

def weight(self, i, struct_MAN):
	# 尤度に従いパーティクルの重みを決定する
	max_l_dist = 0.0
	heading = 9999
	lrf_weight = [0 for j in range(self._max_particles)]

	for i2 in range(self._max_particles):
		lrf_weight[i2] = 9999
		dist = 0.0
		delta_x = self._human[i]._m_qHuman.xp[1] - struct_MAN[i]._m_qHuman.xp[i2]
		delta_y = self._human[i]._m_qHuman.yp[1] - struct_MAN[i]._m_qHuman.yp[i2]
		dist = math.hypot(delta_x, delta_y)
		if lrf_weight[i2] > dist:
			lrf_weight[i2] = dist

	for i3 in range(self._max_particles):
		if lrf_weight[i3] > max_l_dist:
			max_l_dist = lrf_weight[i3]
	
	for i4 in range(self._max_particles):
		struct_MAN[i]._m_qHuman.weight[i4] = 1 - lrf_weight[i4] / max_l_dist

def measure(self, i, struct_MAN):
	x = 0.0
	y = 0.0
	weight = 0.0

	# 重み和
	for i2 in range(self._max_particles):
		x += struct_MAN[i]._m_human.xp[i2] * struct_MAN[i]._m_qHuman.weight[i2]
		y += struct_MAN[i]._m_human.yp[i2] * struct_MAN[i]._m_qHuman.weight[i2]
		weight += struct_MAN[i]._m_qHuman.weight[i2]
	
	# 正規化
	if weight != 0:
		arrayInsert(self._human[i]._m_qHuman.x2, 100, x/weight, 1)
		arrayInsert(self._human[i]._m_qHuman.y2, 100, y/weight, 1)
	for i3 in range(self._max_particles):
		self._tmp_x[i][i3] = struct_MAN[i]._m_qHuman.xp[i3]
		self._tmp_y[i][i3] = struct_MAN[i]._m_qHuman.yp[i3]


"""
パーティクルフィルタ関係保留
"""
#尤度算出関数
def likelihood(x,y,func,image, w=30,h=30):
	x1 = max(0,x - w/2)
	y1 = max(0,y - h/2)
	x2 = min(image.shape[1],x + w/2)
	y2 = min(image.shape[0],y + h/2)
	region = image[y1:y2,x1:x2]
	count = region[func(region)].size
	return (float(count)/image.size) if count >0 else 0.0001

#パーティクルの初期化関数
def init_particles(func,image):
	mask = image.copy()
	mask[func(mask) == False] = 0
	contours,_= cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
	if len(contours) <= 0:
		return None
	max_contour = max(contours, key = cv2.contourArea)
	max_rect = np.array(cv2.boundingRect(max_contour))
	max_rect = max_rect[:2] + max_rect[2:] /2
	weight = likelihood(max_rect[0], max_rect[1], func, image)
	particles = np.ndarray((500, 3), dtype = np.float32)
	particles[:] = [max_rect[0], max_rect[1], weight]
	return particles

#パーティクルのリサンプリング
def resample(particles):
	tmp_particles = particles.copy()
	weighs = particles[:, 2].cumsum()
	last_weight = weighs[weighs.shape[0] - 1]
	for i in xrange(particles.shape[0]):
		weigh = np.random.rand() * last_weight
		particles[i] = tmp_particles[(weighs > weigh).argmax()]
		particles[i][2] = 1.0

#予測
def predict(particles, variance=13.0):
	particles[:, 0] += np.random.randn((particles.shape[0])) * variance
	particles[:, 1] += np.random.randn((particles.shape[0])) * variance

#尤度(重み)判定
def weight(particles, func, image):
	for i in xrange(particles.shape[0]):
		particles[i][2] = likelihood(particles[i][0], particles[i][1],func, image)
	sum_weight = particles[:,2].sum()
	particles[:, 2] *= (particles.shape[0] / sum_weight)

#測定
def measure(particles):
	x = (particles[:, 0] * particles[:, 2]).sum()
	y = (particles[:, 1] * particles[:, 2]).sum()
	weight = particles[:, 2].sum()
	return x / weight, y / weight

#仕上げ
def particle_filter(self, partiles, func, image, max_frame = 10):
	# global particle_filter_cur_frame
	if image[func(image)].size <= 0:
		if self._particle_filter_cur_frame >= max_frame:
			return None, -1, -1
		self._particle_filter_cur_frame = min(self._particle_filter_cur_frame + 1, max_frame)
	else:
		self._particle_filter_cur_frame = 0
		if partiles is None:
			partiles = init_particles(func, image)

	if partiles is None:
		return None, -1, -1

	resample(partiles)
	predict(partiles)
	weight(partiles, func, image)
	x, y = measure(partiles)
	return partiles, x, y

def Human_TrackingInit(manager):
    profile = OpenRTM_aist.Properties(defaults_str=human_tracking_spec)
    manager.registerFactory(profile,
                            Human_Tracking,
                            OpenRTM_aist.Delete)

def MyModuleInit(manager):
    Human_TrackingInit(manager)

    # Create a component
    comp = manager.createComponent("Human_Tracking")

def main():
	mgr = OpenRTM_aist.Manager.init(sys.argv)
	mgr.setModuleInitProc(MyModuleInit)
	mgr.activateManager()
	mgr.runManager()

if __name__ == "__main__":
	main()

