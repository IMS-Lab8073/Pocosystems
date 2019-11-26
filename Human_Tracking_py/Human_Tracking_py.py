#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

"""
 @file Human_Tracking_py.py
 @brief ModuleDescription
 @date $Date$


"""
import sys
import time
sys.path.append(".")
# import pyurg
# import matplotlib.pyplot as plt
import math
import numpy as np
import pandas as pd
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore


# Import RTM module
import RTC
import OpenRTM_aist

app = QtGui.QApplication([])
pg.setConfigOptions(antialias=True)
# Import Service implementation class
# <rtc-template block="service_impl">

plot_x=np.zeros(681)
plot_y=np.zeros(681)

# </rtc-template>

# Import Service stub modules
# <rtc-template block="consumer_import">
# </rtc-template>


# This module's spesification
# <rtc-template block="module_spec">
human_tracking_py_spec = ["implementation_id", "Human_Tracking_py", 
		 "type_name",         "Human_Tracking_py", 
		 "description",       "ModuleDescription", 
		 "version",           "1.0.0", 
		 "vendor",            "NinaTajima", 
		 "category",          "Category", 
		 "activity_type",     "STATIC", 
		 "max_instance",      "1", 
		 "language",          "Python", 
		 "lang_type",         "SCRIPT",
		 "conf.default.LRF_range","4",

		 "conf.__widget__.LRF_range","text"

		 "conf.__type__.LRF_range","int",
		 ""]
# </rtc-template>

##
# @class Human_Tracking_py
# @brief ModuleDescription
# 
# 

class Human_Tracking_py(OpenRTM_aist.DataFlowComponentBase):
	##
	# @brief constructor
	# @param manager Maneger Object
	# 
	def __init__(self, manager):
		OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

		_pose3D = RTC.Pose3D(RTC.Point3D(0.0, 0.0, 0.0), RTC.Orientation3D(0.0, 0.0, 0.0))
		_size3D = RTC.Size3D(0.0, 0.0, 0.0)
		_geometry3D = RTC.Geometry3D(_pose3D, _size3D)
		_rangerConfig = RTC.RangerConfig(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
		self._d_Range_data = RTC.RangeData(RTC.Time(0,0), [], RTC.RangerGeometry(_geometry3D, []), _rangerConfig)
		"""
		"""
		self._Range_dataIn = OpenRTM_aist.InPort("Range_data", self._d_Range_data)
		self._d_human_data_XY = RTC.TimedLongSeq(RTC.Time(0,0),[0]*2)
		"""
		"""
		self._human_data_XYOut = OpenRTM_aist.OutPort("human_data_XY", self._d_human_data_XY)

		"""

		- Name:	LRF_range
		- DefaultValue:4
		"""
		self._LRF_range=[4]

		self.diff_x0 = np.zeros(681) #背景差分初期値
		self.diff_y0 = np.zeros(681)

		self.x_seq=[]
		self.y_seq=[]
		
		self.first_flag = False

		# initialize of configuration-data.
		# <rtc-template block="init_conf_param">
		
		# </rtc-template>



	##
	#
	# The initialize action (on CREATED->ALIVE transition)
	# formaer rtc_init_entry() 
	# 
	# @return RTC::ReturnCode_t
	# 
	#
	def onInitialize(self):
		# Bind variables and configuration variable
		self.bindParameter("LRF_range",self._LRF_range,"4")
		# Set InPort buffers
		self.addInPort("Range_data",self._Range_dataIn)
		
		# Set OutPort buffers
		self.addOutPort("human_data_XY",self._human_data_XYOut)
		
		# Set service provider to Ports
		
		# Set service consumers to Ports
		
		# Set CORBA Service Ports
		# PyQtGraph stuff

		return RTC.RTC_OK
	
	def onActivated(self, ec_id):

		self.first_flag = True

		return RTC.RTC_OK
	
	def onDeactivated(self, ec_id):
	
		return RTC.RTC_OK

	def onExecute(self, ec_id):
		if self._Range_dataIn.isNew():
			_rangeData = self._Range_dataIn.read()
			self.range_data = _rangeData.ranges

			beg_angle = -30 # LRFの始まりの角度
			angle_per_step=360.0/1024.0 # 角度分解能
			offset_step = 0
			n=np.arange(681) # ステップ数
			th_seq= np.deg2rad((n + offset_step) * angle_per_step + beg_angle) # ラジアン
			r_seq=np.array(self.range_data)
			
			#r_seq = np.where((r_seq > 4000) & (r_seq < 10), 0, r_seq) #10<r<4000以外は0に置換
			r_seq = np.where(r_seq > 4000, 4000, r_seq) # r<4000以上は4000に置換

			r_seq = Removalfun(r_seq)

			r_shape = r_seq[0:681].shape
			th_shape = th_seq.shape

			#print "r" + str(r_shape)
			#print "th" + str(th_shape)

			x_seq_raw = r_seq[0:681] * np.cos(th_seq)	# xに変換 #726
			y_seq_raw = r_seq[0:681] * np.sin(th_seq)	# yに変換 #726

			# x_seq_raw=r_seq[45:726]*np.cos(th_seq)	# xに変換 #726
			# y_seq_raw=r_seq[45:726]*np.sin(th_seq)	# yに変換 #726
			# print("######x_seq_raw######")
			# print(x_seq_raw)
			# print("######y_seq_raw######")
			# print(y_seq_raw)


			#self.x_seq=np.delete(x_seq_raw,np.where(x_seq_raw==0)) # xが0だったら消す
			#print("######self.x_seq######")
			#print(self.x_seq)

			#self.y_seq=np.delete(y_seq_raw,np.where(y_seq_raw==0)) # xが0だったら消す
			#print("######self.y_seq######")
			#print(self.y_seq)
			
			### 背景差分法 start #############################################################
			
			### 背景差分法用元のデータ #########
			if self.first_flag == True:
				self.diff_x0 = x_seq_raw
				#self.diff_x0 = 1000 * x_seq_raw
				self.diff_y0 = y_seq_raw
				#self.diff_y0 = 1000 * y_seq_raw
				self.first_flag == False
			##################################

			x_seq_move = np.where(x_seq_raw != self.diff_x0) # 元データじゃないなら移動体x
			y_seq_move = np.where(y_seq_raw != self.diff_y0) # 元データじゃないなら移動体y

			### 背景差分法 end ########################################################

			### plot用データに格納 ##################
			global plot_x, plot_y
			plot_x = x_seq_raw  # self.x_seq
			plot_y = y_seq_raw  # self.y_seq

			#plot_x = 1000*x_seq_raw  # self.x_seq
			#plot_y = 1000*y_seq_raw  # self.y_seq
			########################################
			
			# print plot_x
			# print plot_y
			get_pre_r_seq(r_seq)
		return RTC.RTC_OK
		
class PlotWindow:
    def __init__(self):
		#プロット初期設定
		self.win=pg.GraphicsWindow()
		self.win.setWindowTitle("real time plot")
		self.win.resize(1000, 1000)
		self.plt = self.win.addPlot()  #プロットのビジュアル関係
		self.plt.setLabel('left', "Y Axis", units='s')
		self.plt.setLabel('bottom', "X Axis", units='s')
		self.plt.setYRange(-4200,4200)    #y軸の上限、下限の設定
		self.plt.setXRange(-4200,4200)
		self.plt.showGrid(x=True,y=True,alpha=1.0)

		self.curve=self.plt.plot()  #プロットデータを入れる場所

		#アップデート時間設定
		self.timer=QtCore.QTimer()
		self.timer.timeout.connect(self.update)
		self.timer.start(1)    #10msごとにupdateを呼び出し

    def update(self):
		self.curve.setData(plot_x,plot_y,pen=None,symbol='o')   #プロットデータを格納

def get_pre_r_seq(seq):
	pre_r_seq = seq
	return pre_r_seq

def Removalfun(seq):
	# 3sigma
	mean = seq.mean() # 平均値
	sigma = seq.std() # 標準偏差

	low = mean - 3 * sigma
	high = mean + 3 * sigma

	sol = np.where((seq < low) & (seq > high), get_pre_r_seq, seq)

	return sol

def Human_Tracking_pyInit(manager):
		profile = OpenRTM_aist.Properties(defaults_str=human_tracking_py_spec)
		manager.registerFactory(profile,
							Human_Tracking_py,
							OpenRTM_aist.Delete)

def MyModuleInit(manager):
	Human_Tracking_pyInit(manager)

	# Create a component
	comp = manager.createComponent("Human_Tracking_py")

def main():

	mgr = OpenRTM_aist.Manager.init(sys.argv)
	mgr.activateManager()
	mgr.setModuleInitProc(MyModuleInit)
	mgr.activateManager()
	mgr.runManager(True)
	#mgr.shutdown()

if __name__ == "__main__":
	main()
	plotwin=PlotWindow()
	if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
		QtGui.QApplication.instance().exec_()

