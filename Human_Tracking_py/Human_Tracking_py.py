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

rand_x=np.random.rand(1000)*20-10
rand_y=np.random.rand(1000)*20-10

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

		self._x=0
		self._y=0
		self.x_seq=[]
		self.y_seq=[]
		

		# canvas properties
		#self.width = width
		#self.height = height
		# zero of canvas
		#self.x0 = width/2
		#self.y0 = height/2

		#self.wd = 150
		#self.canvas = canvas

		#self.after(20, self.on_update)


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
		### matplot########################################
		#fig,self.ax = plt.subplots(1, 1)
		self._x=0
		self._y=0
		# 初期化的に一度plotしなければならない
		# そのときplotしたオブジェクトを受け取る受け取る必要がある．
		# listが返ってくるので，注意
		#self.lines, = self.ax.plot(self._x, self._y)
		######################################################

		return RTC.RTC_OK
	
	def onDeactivated(self, ec_id):
	
		return RTC.RTC_OK

	def onExecute(self, ec_id):
		if self._Range_dataIn.isNew():
			_rangeData = self._Range_dataIn.read()
			self.range_data = _rangeData.ranges

			#print(self.range_data[100])
			beg_angle = -30 # LRFの始まりの角度
			angle_per_step=360.0/1024.0 # 角度分解能
			offset_step = 0
			n=np.arange(681) # ステップ数
			th_seq= np.deg2rad((n + offset_step) * angle_per_step + beg_angle) # ラジアン
			#print("#####theta_seqence######")
			#print(th_seq)
			r_seq=np.array(self.range_data)
			#print("#####r_seqence_before######")
			#print(r_seq)
			
			####外れ値の計算#######################################################
			r_seq=np.where(r_seq>4000,0,r_seq)
			r_seq=np.where(r_seq<10,0,r_seq)
			#print("#####r_seqence_after######")
			#print(r_seq)
			#print(len(r_seq))

			r_mean = r_seq.mean()
			#print("####mean#####")
			#print(r_mean)

			r_sigma = r_seq.std()
			#print("####sigma#####")
			#print(r_sigma)

			r_low = r_mean - 3 * r_sigma
			#print("####low#####")
			#print(r_low)

			r_high = r_mean + 3 * r_sigma
			#print("####high#####")
			#print(r_high)

			r_seq=np.where(r_seq < r_low , 0, r_seq)
			r_seq=np.where(r_seq > r_high , 0, r_seq)

			x_seq_raw=r_seq[45:726]*np.cos(th_seq)	# xに変換
			y_seq_raw=r_seq[45:726]*np.sin(th_seq)	# yに変換
			#print("######x_seq_raw######")
			#print(x_seq_raw)
			#print("######y_seq_raw######")
			#print(y_seq_raw)

			self.x_seq=np.delete(x_seq_raw,np.where(x_seq_raw==0)) # xが0だったら消す
			#print("######self.x_seq######")
			#print(self.x_seq)

			self.y_seq=np.delete(y_seq_raw,np.where(y_seq_raw==0)) # xが0だったら消す
			#print("######self.y_seq######")
			#print(self.y_seq)
			###########################################################################

			"""
			print("#####x_seqence######")
			print(self.x_seq)
			print("#####y_seqence######")
			print(self.y_seq)
			"""
			#a=trans_x_y(self.range_data[i],i)
			#print(a)
			#self._x =round(a[0],3)
			#self._y =round(a[1],3)
			#print(self._x)
			#print(self._y)
			#self.x_seq.append(self._x) # urgのxのデータ
			#self.y_seq.append(self._y) # urgのyのデータ
			global rand_x,rand_y
			rand_x=self.x_seq
			rand_y=self.y_seq
		return RTC.RTC_OK

'''
def trans_x_y(r, n):
	beg_angle = -30 # LRFの始まりの角度
	angle_per_step=360.0/1024.0 # 角度分解能
	offset_step = 0
	th = (n + offset_step) * angle_per_step + beg_angle
	x=r*np.cos(th)
	y=r*np.sin(th)
	xy=[x,y]
	return xy
'''
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
		self.data_x = rand_x
		self.data_y = rand_y
		self.curve.setData(rand_x,rand_y,pen=None,symbol='o')   #プロットデータを格納


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

