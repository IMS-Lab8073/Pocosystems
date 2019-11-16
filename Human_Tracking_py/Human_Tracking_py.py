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
if sys.version_info[0] == 2:
      from Tix import *
else:
  from tkinter.tix import *
import pyqtgraph as pg


# Import RTM module
import RTC
import OpenRTM_aist


# Import Service implementation class
# <rtc-template block="service_impl">

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
			for i in range(682):
				a=trans_x_y(self.range_data[i],i)
				print(a)
				self._x =round(a[0],3)
				self._y =round(a[1],3)
				print(self._x)
				print(self._y)
				self.x_seq.append(self._x) # urgのxのデータ
				self.y_seq.append(self._y) # urgのyのデータ
			

			#humanItem = Wid.getPlotItem()
			#self.humanItem.plot(self.x_seq,self.y_seq,clear = True)
			#QtGui.QApplication.processEvents()

			#self.pw = pg.PlotWidget()
			
			#self.setCentralWidget(pw)
			#p1 = self.pw.plotItem
			#p1.addItem(pg.ScatterPlotItem(self.x_seq, self.y_seq, clrea=True, size = 7.5,antialias = True))

			#app = QApplication(sys.argv)
			#mainWin = GraphWindow()
			### matplot #################################
			#self.lines.set_data(self.x_seq,self.y_seq)
			#self.ax.set_xlim(-4200,4200)
			#self.ax.set_ylim(-4200,4200)
			#plt.scatter(self.x_seq, self.y_seq, marker='.', s=50)
			#plt.pause(.001)
			#plt.cla()
			############################################
		return RTC.RTC_OK
	def get_range_data(self):
		return self.range_data

	def get_range_data_x(self):
		return self.x_seq

	def get_range_data_y(self):
		return self.y_seq

'''
def init(self):
	self.canvas = Canvas(self, bg="#000000",width = self.width, height = self.height)
	self.canvas.pack(side=LEFT)
	return
'''
class ToggleItem:
	def __init__(self):
		self.active = True
		return

	def __del__(self):
		self.delete()
		return

	def activate(self):
		self.active = True
		self.draw()
		self.point_draw()
		return

	def deactivate(self):
		self.active = False
		self.delete()
		return

	def toggle(self):
		if self.active:
			self.deactivate()
		else:
			self.activate()
		return


class CanvasText(ToggleItem):
	def __init__(self, canvas, text, x, y,_text,_x,_y):
		ToggleItem.__init__(self)
		self.canvas = canvas
		self.id = self.canvas.create_text(x, y, text=text)
		self.text = text
		self.x = x
		self.y = y
		self._text = "・"
		self._x = 0
		self._y = 0
		self.draw_text(x, y, text)
		self.draw_point(_x,_y,_text)
		return

	def draw(self):
		if self.active == False: return
		self.delete()
		self.id = self.canvas.create_text(self.x, self.y, text=self.text)
		return

	def draw_text(self, x, y, text):
		self.x = x
		self.y = y
		self.text = text
		self.draw()
		return

	def set_point(self):
		if self.active == False: return
		self.delete()
		self.point = self.canvas.create_point(self._x, self._y,text=self._text)
		return

	def draw_point(self,_x,_y,_text):
		self._x = 10
		self._y = 8
		self.text = _text
		self.set_point()
		return

	def delete(self):
		self.canvas.delete(self.id)
		return

class CanvasGrid(ToggleItem):
	def __init__(self, canvas, x0, y0, width, height, pitch, color, linewd):
		ToggleItem.__init__(self)
		self.canvas = canvas
		self.x0 = x0
		self.y0 = y0
		self.width = width
		self.height = height
		self.pitch = pitch
		self.color = color
		self.linewd = linewd
		self.idx = []
		self.idy = []
		self.draw()
		return

	def draw(self):
		if self.active == False: return
		self.delete()
		
		x_start = int(self.x0 % self.pitch)
		x_num   = int((self.width - x_start) / self.pitch) + 1
		for x in range(x_num):
			x0 = x_start + self.pitch * x
		id = self.canvas.create_line(x0, 0, x0, self.height,
									fill=self.color, width=self.linewd)
		self.idx.append(id)

		y_start = int(self.y0 % self.pitch)
		y_num   = int((self.height - y_start) / self.pitch) + 1
		for y in range(y_num):
			y0 = y_start + self.pitch * y
		id = self.canvas.create_line(0, y0, self.width, y0,
									fill=self.color, width=self.linewd)
		self.idy.append(id)

		for i in self.idx:
			self.canvas.tag_lower(i)
		for i in self.idy:
			self.canvas.tag_lower(i)

		return

	def delete(self):
		for i in self.idx:
			self.canvas.delete(i)
		for i in self.idy:
			self.canvas.delete(i)
		return

	def set_pitch(self, pitch):
		if pitch != 0:
			self.pitch = pitch

		self.draw()
		return


class CanvasAxis(ToggleItem):
	def __init__(self, canvas, width, height, color="#ffffff", linewd=1):
		ToggleItem.__init__(self)
		self.x0 = width/2
		self.y0 = height/2
		self.width = width
		self.height = height
		self.color = color
		self.linewd = linewd
		self.canvas = canvas
		self.id = [None] * 4
		#self.point=[None]*681
		#self._x_seq=[None]*681
		#self._y_seq=[None]*681
		self.draw()
		#self.d=10
		#self.point_draw()
		#self.d+=10
		#time.sleep(0.5)
		#self.after(20, self.on_update)

		return

	def draw(self):
		if self.active == False: return
		self.id[0] = self.canvas.create_line(0, self.height/2, 
											self.width, self.height/2,
											fill = self.color,
											width = self.linewd)
		self.id[1] = self.canvas.create_text(self.width-10 ,
											self.height/2 + 10,
											text="x",
											fill = self.color,
											font="courier 12")
		self.id[2] = self.canvas.create_line(self.width/2, 0, 
											self.width/2, self.height,
											fill = self.color,
											width = self.linewd)
		self.id[3] = self.canvas.create_text(self.width/2 + 10,
											+ 10, text="y",
											fill = self.color,
											font="courier 12")
		return
	'''
	def point_draw(self):
		if self.active == False: return
		for i in range(681):
			#for j in range(100):			
			#import random
			#a=random.random()*100
			#b=random.random()*100+random.random()*100
			a=i*2+2*self.d
			b=i+2-3*self.d
			print(a)
			print(b)
			self._x_seq[i]=a
			self._y_seq[i]=b
			self.point[i] = self.canvas.create_text(self.width/2+self._x_seq[i] ,
													self.width/2-self._y_seq[i],text="・",
													fill = "#ffff00",
													font="courier 12")
		time.sleep(0.5)
		return
		'''

	def delete(self):
		for i in self.id:
			self.canvas.delete(i)
		return
	
	def point_delete(self):
		if self.point != None:
			self.canvas.delete(self.point)
		return
	
class ScaledObject:
	def __init__(self, simulator):
		self.simulator = simulator
		#self.tick = simulator.get_tick()
		self.canvas = simulator.get_canvas()
		self.trans = simulator.get_translation()
		return

	def translate(self, x, y, dx, dy, dth):
		return self.trans(x, y, dx, dy, dth)

	def get_tick(self):
		return self.simulator.get_tick()


#------------------------------------------------------------
# LRFrange range data drawing
#
#------------------------------------------------------------
class LRFrange(ScaledObject):
    def __init__(self,canvas, width, height, simulator,
                line_color="#ff0000", fill_color="#ff0000", linewd=1):
        ScaledObject.__init__(self, simulator)
        self.fill_color = fill_color
        self.line_color = line_color
        self.default_fill_color = fill_color
        self.default_line_color = line_color
        self.linewd = linewd
        self.rdata = []
        self.pre_data = []
        self._x_seq = []
        self._y_seq = []
		self.canvas = canvas
		self.x0 = width/2
		self.y0 = height/2
		self.width = width
		self.height = height


        self.poly_id = None
        self.source = None

        # URG parameter
        self.beg_angle = -45
        self.end_angle = 225
        self.angle_per_step = 360.0 / 1024.0
        self.valid_beg_angle = 44 * 360.0 / 1024.0
        self.valid_end_angle = self.valid_beg_angle + 725 * self.angle_per_step
        self.offset_step = 0

        self.threshold = 0.0
        self.sfilter = 0.0
        self.tfilter = 0.0

        self.threshold_check = BooleanVar()
        self.threshold_check.set(True)
        self.tfilter_check = BooleanVar()
        self.tfilter_check.set(True)
        self.sfilter_check = BooleanVar()
        self.sfilter_check.set(True)

        self.threshold_var = DoubleVar()
        self.threshold_var.set(self.threshold)
        self.tfilter_var = DoubleVar()
        self.tfilter_var.set(self.tfilter)
        self.sfilter_var = DoubleVar()
        self.sfilter_var.set(self.sfilter)

        self.update()
        return

    def create_ctrl(self, frame):
        self.lrf_fill_check = StringVar()
        self.lrf_fill_check.set("on")
        self.lrf_line_check = StringVar()
        self.lrf_line_check.set("on")

	"""
        text = Label(frame, text="LRF range area", anchor=W, justify=LEFT)
        # "Line" check box
        line = Checkbutton(frame, text="Line",
                        onvalue="on", offvalue="off",
                        justify=LEFT, anchor=W,
                        variable=self.lrf_line_check,
                        command=self.line_toggle)
        # "Fill" check box
        fill = Checkbutton(frame, text="Fill",
                        onvalue="on", offvalue="off",
                        justify=LEFT, anchor=W,
                        variable=self.lrf_fill_check,
                        command=self.fill_toggle)
        # Threshold (check box/scale)
        thresh = Checkbutton(frame, text="Threshold",
                            onvalue=True, offvalue=False,
                            justify=LEFT, anchor=W,
                            variable=self.threshold_check)
        thresh_scale = Scale(frame, from_=0, to=100, resolution=0.1,
                            label="Threshold", command=self.on_threshold,
                            variable=self.threshold_var, orient=HORIZONTAL)
        # Time-Filter (check box/scale)
        tfilter = Checkbutton(frame, text="Filter(Time)",
                            onvalue=True, offvalue=False,
                            justify=LEFT, anchor=W,
                            variable=self.tfilter_check)
        tfilter_scale = Scale(frame, from_=0, to=1, resolution=0.01,
                            label="Filter", command=self.on_tfilter,
                            variable=self.tfilter_var, orient=HORIZONTAL)
        # Spacial-Filter (check box/scale)
        sfilter = Checkbutton(frame, text="Filter(Spacial)",
                            onvalue=True, offvalue=False,
                            justify=LEFT, anchor=W,
                            variable=self.sfilter_check)
        sfilter_scale = Scale(frame, from_=0, to=1, resolution=0.01,
                            label="Filter", command=self.on_sfilter,
                            variable=self.sfilter_var, orient=HORIZONTAL)

        for w in [text, line, fill, thresh, thresh_scale,
                tfilter, tfilter_scale, sfilter, sfilter_scale]:
            w.pack(side=TOP, anchor=W, fill=X) 
        return
	"""

	def on_threshold(self, var):
		self.threshold = self.threshold_var.get()
		return

    def on_sfilter(self, var):
        self.sfilter = self.sfilter_var.get()
        return

    def on_tfilter(self, var):
        self.tfilter = self.tfilter_var.get()
        return

    def line_toggle(self):
        if self.lrf_line_check.get() == "on":
            self.line_color = self.default_line_color
        else:
            self.line_color = ""
        return

    def fill_toggle(self):
        if self.lrf_fill_check.get() == "on":
            self.fill_color = self.default_fill_color
        else:
            self.fill_color = ""
        return

    def set_data_source(self, source):
        self.source = source
        return

    def set_value(self, _xseq,_y_seq):
        self.rdata_x = _x_seq
        self.rdata_y = _y_seq
        return

    def draw(self):
        self.delete()
        rpos = []
        rpos.append(self.translate(0, 0, 0, 0, 0))
        rpos.append(self.range_to_pos(self.rdata))
        self.poly_id = self.canvas.create_polygon(rpos,
                                                width = self.linewd,
                                                outline = self.line_color,
                                                fill = self.fill_color,
                                                smooth = 1,
                                                splinesteps = 5)
        return
	
    def point_draw(self):
        self.delete()
        for i in range(681):
            self.point[i] = self.canvas.create_text(self.width/2+self._x_seq[i] ,
													self.width/2-self._y_seq[i],text="・",
													fill = "#ffff00",
													font="courier 12")
        time.sleep(0.5)
        return
	

    def range_to_pos(self, data):
        pos = []
        pre_d = 0
        
        tfilter = self.tfilter_check.get()
        sfilter = self.sfilter_check.get()
        thresh  = self.threshold_check.get()

        # Time-Filter
        if tfilter and len(data) == len(self.pre_data):
            for (n, d) in enumerate(data):
                k_t = self.tfilter
                data[n] = self.pre_data[n] * k_t + d * (1 - k_t)

        # Spacial Filter
        for (n, d) in enumerate(data):
            # Threshold
            if thresh and d < self.threshold:
                d = 10000 #pre_d

        if sfilter:
            k_s = self.sfilter
            d = pre_d * k_s + d * (1 - k_s)
        pre_d = d
                
        # n: step number
        # d: length data
        #deg = (n + self.offset_step) * self.angle_per_step + self.beg_angle
        #th = deg * math.pi / 180
        th = (n + self.offset_step) * self.angle_per_step + self.beg_angle
        x = d * math.cos(th)
        y = d * math.sin(th)
        pos.append(self.translate(x, y, 0, 0, 0))
        self.pre_data = data
        return pos

    def delete(self):
        if self.poly_id != None:
            self.canvas.delete(self.point)
        return

    def update(self):
		print("Hello update")
		if self.source != None:
			#rdata = self.source.get_range_data()
			_x_seq= self.source.get_range_data_x()
			_y_seq= self.source.get_range_data_y()
		#if len(_x_seq) != 0:
			#self._x_seq = _x_seq
		#if len(_y_seq) != 0:
			#self._y_seq = _y_seq

		#if len(rdata) != 0:
			#self.rdata = rdata
		#res = self.source.get_angular_res()
		#if res:
			#self.angle_per_step = res
		#beg_angle = self.source.get_start_point()
		#if beg_angle:
			#self.beg_angle = beg_angle
		#end_angle = self.source.get_end_point()
		#if end_angle:
			#self.end_angle = end_angle
		else:
			pass
		self.point_draw()


class TkLRFViewer(Frame):
	def __init__(self, master=None, width=480, height=480):
		Frame.__init__(self, master)

		# canvas properties
		self.width = width
		self.height = height
		# zero of canvas
		self.x0 = width/2
		self.y0 = height/2

		self.wd = 150

		self.robots = {}

		self.robot = None
		self.postext = None

		#self.scale = 1.0
		#self.scale_var = DoubleVar()
		#self.scale_var.set(self.scale)

		self.grid_pitch = 80

		self.tick = 0.1
		self.default_tick = 0.1
		#self.tickscale_var = DoubleVar()
		#self.tickscale_var.set(self.tick)

		self.axis_check = StringVar()
		self.axis_check.set("on")
		self.grid_check = StringVar()
		self.grid_check.set("on")
		self.rnames = {}

		self.init()
		self.pack()

		self.after(20, self.on_update)

		self.x_seq=[None]*681
		self.y_seq=[None]*681

		return

	def init(self):
		print("Hello init")
		self.canvas = Canvas(self, bg="#000000",width = self.width, height = self.height)
		self.canvas.pack(side=LEFT)

		self.can_grid = CanvasGrid(self.canvas, self.x0, self.y0,self.width, self.height, self.grid_pitch,"#aaaaaa", 1)
		self.can_axis = CanvasAxis(self.canvas, self.width, self.height,"#ffffff", 1)

		self.frame = Frame(self)
		self.frame.pack(side=LEFT)

		# Screen control
		# self.scrctrl_frame = Frame(self.frame, width=self.wd, height=300,relief=GROOVE, bd=2)
		# self.scrctrl_frame.pack(side=TOP, fill=X)
		# self.create_scale(self.scrctrl_frame)
		# self.create_checkbutton(self.scrctrl_frame)



		# self.lrfctrl_frame = Frame(self.frame, width=self.wd, height=300,relief=GROOVE, bd=2)
		# self.lrfctrl_frame.pack(side=TOP, fill=X)
		self.lrf = LRFrange(self)
		self.lrf.create_ctrl(self.lrfctrl_frame)

		return

	def on_update(self):
		print("Hello on_update")
		self.lrf.update()
		self.after(20, self.on_update)
		return

	def get_canvas(self):
		return self.canvas

	def get_translation(self):
		return self.real_to_canvas
	def real_to_canvas(self, x, y, dx, dy, dt):
        # Simulator coordinate system -> display coordinate system
		# x, y: original position
		# dx, dy, dt: translation and rotation vector
		# translation and rotation
		x_tmp = (math.cos(dt) * x - math.sin(dt) * y + dx)/self.scale
		y_tmp = (math.sin(dt) * x + math.cos(dt) * y + dy)/self.scale
		# align to canvas coordinate system (origin is center and y+ is upward)
		xo =  x_tmp  + self.x0
		yo = -y_tmp + self.y0
		return xo, yo



import threading

class test_data_creator(threading.Thread):
    def __init__(self, lrf, step = 681):
        threading.Thread.__init__(self)
        import time
        self.lrf = lrf
        self.step = step
        self.flag = True
        return

    def stop(self):
        self.flag = False
        return

    def run(self):
        import random
        _x_seq = [0] * 681
        _y_seq = [0] * 681
        pre = 0
        while self.flag:
            for i in range(681):
				a=random.random()*100
				b=random.random()*100+random.random()*100
				a=i*2+2*i
				b=i+2-3*i
				print(a)
				print(b)
				_x_seq[i]=a
				_y_seq[i]=b
        self.lrf.set_value(_x_seq,_y_seq)
        time.sleep(0.1)
        
        return

"""
	def get_tick(self):
		print("Hello get_tick")
		return self.tick

	def get_canvas(self):
		print("Hello get_canvas")
		return self.canvas

	def get_translation(self):
		print("Hello get_translation")
		return self.real_to_canvas
"""

def trans_x_y(r, n):
	beg_angle = -30 # LRFの始まりの角度
	angle_per_step=360.0/1024.0 # 角度分解能
	offset_step = 0
	th = (n + offset_step) * angle_per_step + beg_angle
	x=r*math.cos(th)
	y=r*math.sin(th)
	xy=[x,y]
	return xy

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
	m = TkLRFViewer(Tk())
	m.master.title("Human_Tracking")
	mgr = OpenRTM_aist.Manager.init(sys.argv)
	mgr.activateManager()
	mgr.setModuleInitProc(MyModuleInit)
	mgr.activateManager()
	mgr.runManager(True)
	lrf_rtc = mgr.createComponent("LRFViewer")
	m.lrf.set_data_source(lrf_rtc)
	m.mainloop()
	mgr.shutdown()

if __name__ == "__main__":
	main()
