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
import cv2
import numpy as np

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

class Human_Tracking(OpenRTM_aist.DataFlowComponentBase):
	

	def __init__(self, manager):
		OpenRTM_aist.DataFlowComponentBase.__init__(self, manager)

		self._d_Range_data = RTC.RangeData(RTC.Time(0,0),"null")
		"""
		"""
		self._Range_dataIn = OpenRTM_aist.InPort("Range_data", self._d_Range_data)
		self._d_human_dataX = RTC.TimedOctet(RTC.Time(0,0),"null")
		"""
		"""
		self._human_dataXOut = OpenRTM_aist.OutPort("human_dataX", self._d_human_dataX)
		self._d_human_dataY = RTC.TimedLongSeq(RTC.Time(0,0),"null")
		"""
		"""
		self._human_dataYOut = OpenRTM_aist.OutPort("human_dataY", self._d_human_dataY)


		


		# initialize of configuration-data.
		# <rtc-template block="init_conf_param">
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
		
		
		return RTC.RTC_OK


	def onActivated(self, ec_id):
		print("on Activate")
		return RTC.RTC_OK
	

	def onDeactivated(self, ec_id):
		print("on Deactivate")
		return RTC.RTC_OK

	def onExecute(self, ec_id):
	
		return RTC.RTC_OK

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
	cntours,_= cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
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
particle_filter_cur_frame = 0
def particle_filter(partiles, func, image, max_frame = 10):
	global particle_filter_cur_frame
	if image[func(image)].size <= 0:
		if particle_filter_cur_frame >= max_frame:
			return None, -1, -1
		particle_filter_cur_frame = min(particle_filter_cur_frame + 1, max_frame)
	else:
		particle_filter_cur_frame = 0
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

