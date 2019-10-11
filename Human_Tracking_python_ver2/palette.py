#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -*- Python -*-

from Tix import *
import time
import math

"""
pixele2dist = 18
test = [0 for i in range(1100)]

draw_scale = 1.0
x_ashi = [0 for i in range(21)]
y_ashi = [0 for i in range(21)]

"""

#------------------------------------------------
# window描画に必要そうなやつ
#------------------------------------------------
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
    def __init__(self, canvas, text, x, y):
        ToggleItem.__init__(self)
        self.canvas = canvas
        self.id = self.canvas.create_text(x, y, text=text)
        self.text = text
        self.x = x
        self.y = y
        self.draw_text(x, y, text)
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
            id = self.canvas.create_line(x0, 0, x0, self.height, fill=self.color, width=self.linewd)
            self.idx.append(id)

        y_start = int(self.y0 % self.pitch)
        y_num   = int((self.height - y_start) / self.pitch) + 1
        for y in range(y_num):
            y0 = y_start + self.pitch * y
            id = self.canvas.create_line(0, y0, self.width, y0, fill=self.color, width=self.linewd)
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
        self.draw()
        return

    def draw(self):
        if self.active == False: return
        self.delete()
        self.id[0] = self.canvas.create_line(0, self.height/2, self.width, self.height/2, fill = self.color, width = self.linewd)
        self.id[1] = self.canvas.create_text(self.width - 10, self.height/2 + 10, text="x", fill = self.color, font="courier 12")
        self.id[2] = self.canvas.create_line(self.width/2, 0, self.width/2, self.height, fill = self.color, width = self.linewd)
        self.id[3] = self.canvas.create_text(self.width/2 + 10, + 10, text="y", fill = self.color, font="courier 12")
        return

    def delete(self):
        for i in self.id:
            self.canvas.delete(i)
        return


class ScaledObject:
    def __init__(self, simulator):
        self.simulator = simulator
        self.tick = simulator.get_tick()
        self.canvas = simulator.get_canvas()
        self.trans = simulator.get_translation()
        return

    def translate(self, x, y, dx, dy, dth):
        return self.trans(x, y, dx, dy, dth)

    def get_tick(self):
        return self.simulator.get_tick()

#---------------------------------------------------------
# URGのデータを描画するのに必要そうなやつ
#---------------------------------------------------------
class LRFrange(ScaledObject):
    def __init__(self, simulator, line_color="#ff0000", fill_color="#ff0000", linewd=1):
        ScaledObject.__init__(self, simulator)
        self.fill_color = fill_color
        self.line_color = line_color
        self.default_fill_color = fill_color
        self.default_line_color = line_color
        self.linewd = linewd
        self.rdata = []
        self.pre_data = []
        self.poly_id = None
        self.source = None

        # URG parameter
        self.start_angle = -45
        self.end_angle = 225
        self.angle_per_step = 360.0 / 1024.0
        self.valid_start_angle = 44 * 360.0 / 1024.0
        self.valid_end_angle = self.valid_start_angle + 725 * self.angle_per_step
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

    def set_data_source(self, source):
        self.source = source
        return

    def set_value(self, data):
        self.rdata = data
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

    def range_to_pos(self, data):
        pos = []
        pre_d = 0
        
        thresh  = self.threshold_check.get()

        # Spacial Filter
        for (n, d) in enumerate(data):
            # Threshold
            if thresh and d < self.threshold:
                d = 10000 #pre_d
            pre_d = d
                
            # n: step number
            # d: length data
            #deg = (n + self.offset_step) * self.angle_per_step + self.beg_angle
            #th = deg * math.pi / 180
            th = (n + self.offset_step) * self.angle_per_step + self.start_angle
            x = d * math.cos(th)
            y = d * math.sin(th)
            pos.append(self.translate(x, y, 0, 0, 0))
        self.pre_data = data
        return pos

    def delete(self):
        if self.poly_id != None:
            self.canvas.delete(self.poly_id)
        return

    def update(self):
        if self.source != None:
            rdata = self.source.get_range_data()
            if len(self.rdata) != 0:
                self.rdata = rdata

            res = self.source.get_angular_res()
            if res:
                self.angle_per_step = res

            start_angle = self.source.get_start_point()
            if start_angle:
                self.start_angle = start_angle

            end_angle = self.source.get_end_point()
            if end_angle:
                self.end_angle = end_angle

        else:
            pass
        self.draw()

#----------------------------------------------------------------
# TkLRFViewer main windows class
#----------------------------------------------------------------
# 
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

        self.scale = 1.0
        self.scale_var = DoubleVar()
        self.scale_var.set(self.scale)

        self.grid_pitch = 50

        self.tick = 0.1
        self.default_tick = 0.1
        self.tickscale_var = DoubleVar()
        self.tickscale_var.set(self.tick)

        self.axis_check = StringVar()
        self.axis_check.set("on")
        self.grid_check = StringVar()
        self.grid_check.set("on")
        self.rnames = {}


        self.init()
        self.pack()


        self.after(20, self.on_update)
        return


    def init(self):
        self.canvas = Canvas(self, bg="#000000",
                            width = self.width, height = self.height)
        self.canvas.pack(side=LEFT)

        self.can_grid = CanvasGrid(self.canvas, self.x0, self.y0,
                                self.width, self.height, self.grid_pitch,
                                "#aaaaaa", 1)
        self.can_axis = CanvasAxis(self.canvas, self.width, self.height,
                                "#ffffff", 1)

        self.frame = Frame(self)
        self.frame.pack(side=LEFT)