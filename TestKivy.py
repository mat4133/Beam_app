from kivy.app import App
from kivy.lang import Builder
from kivy.uix.screenmanager import Screen, ScreenManager
from kivy.properties import ObjectProperty
from kivy.config import Config

Config.set('kivy', 'keyboard_mode', 'dock')

from kivy.uix.floatlayout import FloatLayout
from kivy.uix.popup import Popup
from kivy.graphics import Line, Triangle, Rectangle, Color
from kivy.core.window import Window
from kivy_garden.graph import Graph, MeshLinePlot
from copy import deepcopy
from math import *
import numpy as np

class Indeterminate_load(FloatLayout):
    pass

class Lengthchange_Error(FloatLayout):
    pass

class Length_Error(FloatLayout):
    pass

class Position_Error(FloatLayout):
    pass

class Before_Length_Error(FloatLayout):
    pass

class Non_Equation_Error(FloatLayout):
    pass

class Non_Float_Error(FloatLayout):
    pass

class Support_Input(FloatLayout):
    def __init__(self, type):
        super().__init__()
        self.type = type
    support_dis = ObjectProperty(None)

    def inputsupport(self):
        global update_drawing_list
        support = [distance_check(self.support_dis.text)]
        support.append(self.type)
        global total_supports
        if None not in support:
            total_supports.append(support)
            update_drawing_list.append(support)
            popupWindow.dismiss()

class Moment_Input(FloatLayout):
    moment_mag = ObjectProperty(None)
    moment_dis = ObjectProperty(None)

    def inputmoment(self):
        global update_drawing_list
        moment = [distance_check(self.moment_dis.text)]
        moment += int_error(self.moment_mag.text)
        moment.append("Moment")
        global total_equation
        if None not in moment:
            total_equation.append(moment)
            update_drawing_list.append(moment)
            popupWindow.dismiss()

class Point_Input(FloatLayout):
    point_mag = ObjectProperty(None)
    point_dis = ObjectProperty(None)
    point_angle = ObjectProperty(None)

    def inputpoint(self):
        global update_drawing_list
        point = [distance_check(self.point_dis.text)]
        point += int_error(self.point_mag.text, self.point_angle.text)
        point.append("Point_load")
        global total_equation
        if None not in point:
            total_equation.append(point)
            update_drawing_list.append(point)
            popupWindow.dismiss()

class Distributed_Input(FloatLayout):
    dis_start = ObjectProperty(None)
    dis_end = ObjectProperty(None)
    dis_startv = ObjectProperty(None)
    dis_endv = ObjectProperty(None)

    def inputdis(self):
        global update_drawing_list
        distributed_load = [distance_check(self.dis_start.text)]
        distributed_load.append(distance_check(self.dis_end.text))
        distributed_load += equation(self.dis_startv.text, self.dis_endv.text, self.dis_start.text, self.dis_end.text)
        global total_equation
        if None not in distributed_load:
            total_equation.append(distributed_load)
            update_drawing_list.append(distributed_load)
            popupWindow.dismiss()

class InputWindow(Screen):
    def __init__(self, **kwargs):
        super(InputWindow, self).__init__(**kwargs)
        self.abs_max = 0

    length = ObjectProperty(None)

    def lengthbtn(self):
        global length
        if length != None:
            Show_Popup(Lengthchange_Error(), "Error")
        else:
            length = length_check(self.length.text)
            if length != None:
                with self.canvas:
                    Color(1,1,1)
                    Rectangle(pos=(Window.size[0]/20, Window.size[1]/2.353), size=(Window.size[0]/2, Window.size[1]/300))

    def supportbtn(self, type):
        if length != None:
            Show_Popup(Support_Input(type), "Support Input")
        else:
            Show_Popup(Before_Length_Error(), "Error")

    def momentbtn(self):
        if length != None:
            Show_Popup(Moment_Input(), "Moment Input")
        else:
            Show_Popup(Before_Length_Error(), "Error")

    def pointbtn(self):
        if length != None:
            Show_Popup(Point_Input(), "Point Load Input")
        else:
            Show_Popup(Before_Length_Error(), "Error")

    def DLbtn(self):
        if length != None:
            Show_Popup(Distributed_Input(), "Distributed Load Input")
        else:
            Show_Popup(Before_Length_Error(), "Error")

    def update_drawing(self):
        global update_drawing_list, length
        with self.canvas:
            Color(0,0,0)
            Rectangle(pos = (0,0), size = (Window.size[0]/1.75, Window.size[1]/1.4))
        with self.canvas:
            Color(1,1,1)
            Rectangle(pos=(Window.size[0] / 20, Window.size[1] / 2.353), size=(Window.size[0] / 2, Window.size[1] / 300))
        beam_length = Window.size[0]/2
        y_pos = Window.size[1] / 2.353 + Window.size[1] / 600
        for items in update_drawing_list:
            if len(items) < 4:
                pass
            elif items[3] == "Point_load":
                self.abs_max = max(abs(self.abs_max), abs(items[1]))
            elif items[3] == "UDL":
                self.abs_max = max(abs(self.abs_max), abs(items[2]))
            elif items[4] == "VDL":
                equation = (items[3]-items[2])/(items[1]-items[0])*x+items[2]
                y_points = []
                x_points = []
                interval = (items[1] - items[0]) / 1000
                max_point = 0
                min_point = 0
                for i in range(1000):
                    y_point = equation.calculate(i * interval)
                    if y_point > max_point:
                        max_point = y_point
                    if y_point < min_point:
                        min_point = y_point
                    y_points.append(y_point)
                    abs_position = Window.size[0] / 20 + (float(items[0] + i * interval) * beam_length) / length
                    x_points.append(abs_position)
                self.abs_max = max(abs(max_point), abs(min_point), self.abs_max)
        for items in update_drawing_list:
            if items[1] == "Fixed":
                with self.canvas:
                    Color(1,0,0)
                    x_size = Window.size[0]/75
                    y_size = Window.size[0]/25
                    x_pos = Window.size[0] / 20 + (float(items[0]) * beam_length) / length - x_size/2
                    y_pos = Window.size[1] / 2.353 - (y_size/2)+ Window.size[1]/300
                    Rectangle(pos = (x_pos,y_pos), size = (x_size, y_size))
            elif items[1] == "Pin":
                with self.canvas:
                    Color(1,0,0)
                    x_pos = Window.size[0] / 20 + (float(items[0]) * beam_length) / length
                    y_pos = Window.size[1] / 2.353
                    x_difference = Window.size[0] / 37.5
                    y_difference = Window.size[1] / 25
                    points = [x_pos, y_pos, x_pos - x_difference, y_pos - y_difference, x_pos + x_difference,
                              y_pos - y_difference]
                    Triangle(points=points)
            elif items[1] == "Roller":
                with self.canvas:
                    Color(1, 0, 0)
                    x_pos = Window.size[0] / 20 + (float(items[0]) * beam_length) / length
                    y_pos = Window.size[1] / 2.353
                    x_difference = Window.size[0] / 37.5
                    y_difference = Window.size[1] / 25
                    points = [x_pos, y_pos, x_pos - x_difference, y_pos - y_difference, x_pos + x_difference,
                              y_pos - y_difference]
                    Triangle(points=points)
                    Color(1, 1, 1)
                    radius = x_difference/2
                    Line(circle = (x_pos, y_pos-y_difference-radius, radius))
            elif items[2] == "Moment":
                x_pos = Window.size[0] / 20 + (float(items[0]) * beam_length) / length
                y_pos = Window.size[1] / 2.353 + Window.size[1] / 600
                radius = Window.size[0] / 50
                x_tip = Window.size[0] / 120
                y_tip = Window.size[0] / 160
                with self.canvas:
                    Color(1, 1, 1)
                    Line(circle=(x_pos, y_pos, radius, 0, 180))
                    Line(points=[x_pos, y_pos+radius, x_pos+x_tip, y_pos+radius-y_tip, x_pos, y_pos+radius,
                                 x_pos+x_tip, y_pos+radius+y_tip])
            elif items[3] == "Point_load":
                angle = radians(float(items[2]))
                x_pos = Window.size[0] / 20 + (float(items[0]) * beam_length) / length
                y_pos = Window.size[1] / 2.353 + Window.size[1]/600
                if items[1] > 0:
                    tip_y_pos = (items[1]*sin(angle)/self.abs_max)*(Window.size[1]/1.5-Window.size[1]/2.353)
                    tip_x_pos = -(items[1]*cos(angle)/self.abs_max)*(Window.size[1]/1.5-Window.size[1] / 2.353)
                elif items[1] < 0:
                    tip_y_pos = (items[1]*sin(angle)/self.abs_max)*(Window.size[1]/1.5-Window.size[1]/2.353)
                    tip_x_pos = (items[1] * cos(angle) / self.abs_max) * (Window.size[1] / 1.5 - Window.size[1]/2.353)
                with self.canvas:
                    Color(1, 1, 1)
                    right_x_wing = -Window.size[0]/80*cos(angle)+Window.size[1]/120*sin(angle)
                    right_y_wing = Window.size[0]/80*sin(angle)+Window.size[1]/120*cos(angle)
                    left_x_wing = -Window.size[0]/80*cos(angle)-Window.size[1]/120*sin(angle)
                    left_y_wing = Window.size[0]/80*sin(angle)-Window.size[1]/120*cos(angle)
                    if items[1] > 0:
                        Line(points=[x_pos, y_pos, x_pos+right_x_wing, y_pos+right_y_wing, x_pos, y_pos, x_pos+left_x_wing,
                                     y_pos+left_y_wing],width=1)
                        Line(points=[x_pos, y_pos,x_pos+tip_x_pos,y_pos+tip_y_pos], width=1)
                    elif items[1] == 0:
                        pass
                    else:
                        Line(points=[x_pos, y_pos, x_pos+right_x_wing, y_pos-right_y_wing, x_pos, y_pos, x_pos+left_x_wing,
                                     y_pos-left_y_wing],
                             width=1)
                        Line(points=[x_pos, y_pos, x_pos+tip_x_pos, y_pos+tip_y_pos], width=1)
            elif items[3] == "UDL":
                self.abs_max = max(abs(self.abs_max), abs(items[2]))
                arrows = int(max(3, (items[1]-items[0])/length*20)-2)
                start_pos = Window.size[0] / 20 + (float(items[0]) * beam_length) / length
                end_pos = Window.size[0] / 20 + (float(items[1]) * beam_length) / length
                base_height = Window.size[1] / 2.353 + Window.size[1]/600
                arrow_x = Window.size[1]/120
                if items[2] > 0:
                    arrow_y = Window.size[0]/80
                elif items[2] < 0:
                    arrow_y = -Window.size[0]/80
                else:
                    arrow_y = 0
                    arrow_x = 0
                height = (items[2]/self.abs_max)*(Window.size[1]/1.5-Window.size[1]/2.353)
                with self.canvas:
                    Color(1, 1, 1)
                    Line(points=[start_pos, base_height+height, end_pos, base_height+height])
                    Line(points=[start_pos+arrow_x, base_height+arrow_y, start_pos, base_height,
                                 start_pos-arrow_x, base_height+arrow_y, start_pos, base_height,
                                 start_pos, base_height+height])
                    Line(points=[end_pos + arrow_x, base_height + arrow_y, end_pos, base_height,
                                 end_pos - arrow_x, base_height + arrow_y, end_pos, base_height,
                                 end_pos, base_height + height])
                    for i in range(arrows):
                        position = (i+1)*(end_pos-start_pos)/(arrows+1)+start_pos
                        Line(points=[position + arrow_x, base_height + arrow_y, position, base_height,
                                     position - arrow_x, base_height + arrow_y, position, base_height,
                                     position, base_height + height])
            elif items[4] == "VDL":
                y_pos = Window.size[1] / 2.353 + Window.size[1]/600
                y_points = []
                x_points = []
                interval = (items[1] - items[0]) / 1000
                max_point = 0
                min_point = 0
                equation = (items[3] - items[2]) / (items[1] - items[0]) * x + items[2]
                for i in range(1000):
                    y_point = equation.calculate(0 + i * interval)
                    if y_point > max_point:
                        max_point = y_point
                    if y_point < min_point:
                        min_point = y_point
                    y_points.append(equation.calculate(0 + i * interval))
                    abs_position = Window.size[0] / 20 + (float(items[0] + i * interval) * beam_length) / length
                    x_points.append(abs_position)
                arrows = int(max(3, (items[1] - items[0]) / length * 20))
                start_pos = Window.size[0] / 20 + (float(items[0]) * beam_length) / length
                end_pos = Window.size[0] / 20 + (float(items[1]) * beam_length) / length
                arrow_x = Window.size[1] / 120
                arrow_interval = floor(1000/arrows)
                points = []
                for i in range(len(y_points)):
                    y_abs = (y_points[i] / self.abs_max) * (Window.size[1] / 1.5 - Window.size[1] / 2.353) + y_pos
                    if ((i) % arrow_interval == 0 and i < 951) or i == 0 or i == 999:
                        if y_points[i] > 0 and y_abs > (Window.size[0] / 80 + y_pos):
                            arrow_y = Window.size[0] / 80 + y_pos
                        elif y_points[i] < 0 and y_abs < (-Window.size[0] / 80 + y_pos) :
                            arrow_y = -Window.size[0] / 80 + y_pos
                        else:
                            arrow_y = y_pos
                        with self.canvas:
                            Color(1,1,1)
                            Line(points=[x_points[i], y_abs, x_points[i], y_pos], width=1)
                            Line(points=[x_points[i]+arrow_x, arrow_y, x_points[i], y_pos, x_points[i]-arrow_x, arrow_y])
                    points.append(x_points[i])
                    points.append(y_abs)
                with self.canvas:
                    Color(1,1,1)
                    Line(points=points, width=1)

    def clear(self):
        global length, total_equation, total_supports, update_drawing_list, sreset, mreset, dreset, areset
        total_equation = []
        total_supports = []
        length = None
        update_drawing_list = []
        self.length.text = ''
        with self.canvas:
            Color(0,0,0)
            Rectangle(pos = (0,0), size = (Window.size[0]/1.7, Window.size[1]/1.4))

    def load_check(self):
        global total_supports
        num_fixed = 0
        num_pin = 0
        for supports in total_supports:
            if supports[1] == "Fixed":
                num_fixed += 1
            elif supports[1] == "Pin":
                num_pin += 1
        if num_fixed > 0 or (num_pin > 0 and len(total_supports) > 1 ):
            self.manager.current = "Shear"
        else:
            Show_Popup(Indeterminate_load(), "Error")




# Popup's for user input errors
def distance_check(value):
    global length
    try:
        value = float(value)
        if value < 0 or value > length:
            Show_Popup(Position_Error(), "Error")
            return None
        else:
            return value
    except ValueError:
        Show_Popup(Non_Float_Error(), "Error")
        return None

def equation(startv, endv, start, end):
    try:
        startv = float(startv)
        endv = float(endv)
        start = float(start)
        end = float(end)
        gradient = (endv-startv)/(end-start)
        if startv == endv:
            return [startv, "UDL"]
        else:
            return [startv, endv, "VDL"]
    except ValueError:
        Show_Popup(Non_Float_Error(), "Error")
        return None

    # add code to turn start and end values into udl and vdl with equation

def length_check(value):
    try:
        value = float(value)
        if value <= 0:
            Show_Popup(Length_Error(), "Error")
            return None
        else:
            return value
    except ValueError:
        Show_Popup(Non_Float_Error(), "Error")
        return None

def int_error(*args):
    output = []
    error = 0
    for values in args:
        try:
            values = float(values)
            output.append(values)
        except ValueError:
            error = 1
            Show_Popup(Non_Float_Error(), "Error")
    if error == 0:
        return output

def Show_Popup(func, title):
    show = func
    global popupWindow
    popupWindow = Popup(title = title, content = show, size_hint = (0.7,0.7), pos_hint = {'x':0.15,'y':0.15})

    popupWindow.open()

class ShearWindow(Screen):

    enter = 0
    min_value = ObjectProperty(None)
    max_value = ObjectProperty(None)

    def eqprint(self):
        global total_equation, total_supports, length, results
        results = support_sort_finder(length, total_supports, total_equation)
        self.add_widget(results[0][0])
        self.max_value.text = "Max stress:" + str(results[0][1][1])
        self.min_value.text = "Min stress:" + str(results[0][1][0])

    def eqremove(self):
        self.remove_widget(results[0][0])

#REMOVE WIDGETS OFF SCREENS WHEN CALC AGAIN

class MomentWindow(Screen):

    enter = 0
    min_value = ObjectProperty(None)
    max_value = ObjectProperty(None)

    def eqprint(self):
        global results
        self.add_widget(results[1][0])
        self.max_value.text = "Max Moment:" + str(results[1][1][1])
        self.min_value.text = "Min Moment:" + str(results[1][1][0])

    def eqremove(self):
        self.remove_widget(results[1][0])

class AxialWindow(Screen):
    enter = 0
    min_value = ObjectProperty(None)
    max_value = ObjectProperty(None)

    def eqprint(self):
        global results
        self.add_widget(results[3][0])
        self.max_value.text = "Max Stress:" + str(results[3][1][1])
        self.min_value.text = "Min Stress:" + str(results[3][1][0])

    def eqremove(self):
        self.remove_widget(results[3][0])

class DeflectionWindow(Screen):
    enter = 0
    min_value = ObjectProperty(None)
    max_value = ObjectProperty(None)

    def eqprint(self):
        global results
        self.add_widget(results[2][0])
        self.max_value.text = "Max Deflection:" + str(results[2][1][1])
        self.min_value.text = "Min Deflection:" + str(results[2][1][0])

    def eqremove(self):
        self.remove_widget(results[2][0])

class WindowManager(ScreenManager):
    pass


total_equation = [] #I think ready
total_supports = [] #I also think ready
length = None #ready
update_drawing_list = []

## Backend code goes here


# Inputs: Loads (with direction), distances between loads, Angle of loads, type of loads
# Inputs: Positions of supports and type of support
# Outputs: Loads on each support, stress diagrams, bending moment diagrams.

# Supports takes format [[position from LHS in m, type of support],[position from LHS in m, type of support]]
# Args for load will take start and end point of UDL in list form e.g. [1,5]

class Symbol():

    def __init__(self, value):
        self.value = str(value)

    def __radd__(self, other):
        other = Symbol(other)
        return(Symbol(self.value+'+'+other.value))

    def __add__(self, other):
        other = Symbol(other)
        return(Symbol(self.value+'+'+other.value))

    def __sub__(self, other):
        other = Symbol(other)
        return(Symbol(self.value+'-'+other.value))

    def __rsub__(self, other):
        other = Symbol(other)
        return(Symbol(other.value+'-'+self.value))

    def __mul__(self, other):
        other = Symbol(other)
        self.value = '('+self.value+')'
        other.value = '('+other.value+')'
        return(Symbol(self.value+'*'+other.value))

    def __rmul__(self, other):
        other = Symbol(other)
        self.value = '(' + self.value + ')'
        other.value = '(' + other.value + ')'
        return (Symbol(other.value+'*'+self.value))

    def __truediv__(self, other):
        other = Symbol(other)
        self.value = '(' + self.value + ')'
        other.value = '(' + other.value + ')'
        return (Symbol(self.value +'/'+ other.value))

    def __rtruediv__(self, other):
        other = Symbol(other)
        self.value = '(' + self.value + ')'
        other.value = '(' + other.value + ')'
        return (Symbol(other.value + '/'+ self.value))

    def __pow__(self, power, modulo=None):
        power = Symbol(power)
        self.value = '(' + self.value + ')'
        power.value = '(' + power.value + ')'
        return (Symbol(self.value + '**' + power.value))

    def __rpow__(self, power, modulo=None):
        power = Symbol(power)
        self.value = '(' + self.value + ')'
        power.value = '(' + power.value + ')'
        return (Symbol(power.value+'**'+self.value))

    def __repr__(self):
        return repr(self.value)

    def replace(self, symbol, number):
        return Symbol(self.value.replace(symbol, str(number)))

    def evaluate(self):
        return eval(self.value)

    def calculate(self, number):
        new_equation = self.replace('x', number)
        return new_equation.evaluate()


x = Symbol('x')

class Support():
    def __init__(self, distance):
        self.distance = distance

class Vertical_Support(Support):
    pass

class Moment_Support(Support):
    pass

class Horizontal_Support(Support):
    pass

class Horizontal_Load():
    def __init__(self, distance, magnitude):
        self.distance = distance
        self.magnitude = magnitude

class Condition:
    def __init__(self, Power_level, distance, condition):
        self.Power_level = Power_level
        self.distance = distance
        self.condition = condition

    def __repr__(self):
        return repr((self.Power_level, self.distance, self.condition))

class Point_Load:
    def __init__(self, distance, magnitude, angle):
        angle = radians(angle)
        self.distance = distance
        self.end = distance
        self.hos_magnitude = round(magnitude*cos(angle), 12)
        self.ver_magnitude = round(magnitude*sin(angle), 12)
        self.angle = angle

    def __repr__(self):
        return repr((self.distance, self.ver_magnitude, self.hos_magnitude,self.__class__.__name__))


class VDL:
    def __init__(self, start, coefficient):
        self.distance = start
        self.coefficient = coefficient

    def __repr__(self):
        return repr((self.distance, self.equation))

class UDL:
    def __init__(self, start, magnitude):
        self.distance = start
        self.magnitude = magnitude

class Moment:
    def __init__(self, distance, magnitude):
        self.distance = distance
        self.magnitude = magnitude

    def __repr__(self):
        return repr((self.distance, self.magnitude, self.__class__.__name__))


class Macauley:
    def __init__(self, Load):
        self.distance = Load.distance
        if Load.__class__.__name__ == "Point_Load":
            self.multiplier = 1
            self.power = 1
            self.equation = self.multiplier*Load.ver_magnitude*(x - self.distance)**self.power
            self.magnitude = Load.ver_magnitude
            self.unknown = 'known'

        elif Load.__class__.__name__ == "Moment":
            self.multiplier = 1
            self.power = 0
            self.equation = self.multiplier*Load.magnitude*(x - self.distance)**self.power
            self.magnitude = Load.magnitude
            self.unknown = 'known'

        elif Load.__class__.__name__ == "UDL":
            self.multiplier = 0.5
            self.power = 2
            self.equation = self.multiplier*Load.magnitude*(x-self.distance)**self.power
            self.magnitude = Load.magnitude
            self.unknown = 'known'

        elif Load.__class__.__name__ == "VDL":
            self.power = 3
            self.multiplier = 1
            self.magnitude = Load.coefficient * (1-((self.power-1)/(self.power)))
            self.equation = self.multiplier*self.magnitude*((x-self.distance)**self.power)
            self.unknown = 'known'

        elif Load.__class__.__name__ == "Horizontal_Support":
            self.multiplier = 1
            self.power = 0
            self.equation = self.multiplier*(x - self.distance)**self.power
            self.unknown = 'unknown'

        elif Load.__class__.__name__ == "Moment_Support":
            self.multiplier = 1
            self.power = 0
            self.equation = self.multiplier*(x - self.distance)**self.power
            self.magnitude = 1
            self.unknown = 'unknown'

        elif Load.__class__.__name__ == "Vertical_Support":
            self.multiplier = 1
            self.magnitude = 1
            self.power = 1
            self.equation = self.multiplier*(x - self.distance)**self.power
            self.unknown = 'unknown'

        elif Load.__class__.__name__ == "Horizontal_Load":
            self.multiplier = 1
            self.power = 0
            self.equation = self.multiplier * Load.magnitude * (x - self.distance) ** self.power
            self.magnitude = Load.magnitude
            self.unknown = 'known'


    def calculate(self, datapoint):
        if self.distance > datapoint:
            return 0
        elif self.equation == 0:
            return 0
        else:
            replaced_equation = self.equation.replace('x',datapoint)
            return replaced_equation.evaluate()

    def integrate(self):
        self.power += 1
        self.multiplier = self.multiplier/(max(self.power, 1))
        self.equation = (self.multiplier*self.magnitude*(x-self.distance)**self.power)


    def differentiate(self):
        if self.power == 0:
            self.power -= 1
            self.equation = 0
        else:
            self.multiplier = self.multiplier*self.power
            self.power -= 1
            self.equation = self.multiplier*self.magnitude * (x - self.distance) ** (self.power)

    def update(self, new_magnitude):
        if self.power < 0:
            self.equation = 0
        else:
            self.magnitude = new_magnitude
            self.equation = (self.multiplier*new_magnitude* (x - self.distance) ** self.power)

    def __repr__(self):
        if self.unknown == "unknown":
            return repr((self.equation, 'unknown'))
        else:
            return repr(self.equation)


def load_taker(load_list):
    Horizontal_loads = []
    Macauley_list = []
    for load_input in load_list:
        if load_input[2] == "Moment":
            Macauley_list.append(Macauley(Moment(load_input[0], load_input[1])))
        elif load_input[3] == "UDL":
            formatted_load_input = UDL(load_input[0], load_input[2])
            extra_load = UDL(load_input[1], load_input[2]*-1)
            Macauley_list.append(Macauley(formatted_load_input))
            Macauley_list.append(Macauley(extra_load))
        elif load_input[3] == "Point_load":
            formatted_load_input = Point_Load(load_input[0], load_input[1], load_input[2])
            if formatted_load_input.hos_magnitude == 0:
                Macauley_list.append(Macauley(formatted_load_input))
            else:
                horizontal_load = Horizontal_Load(load_input[0], formatted_load_input.hos_magnitude)
                vertical_load = Point_Load(load_input[0], formatted_load_input.ver_magnitude, 90)
                Macauley_list.append(Macauley(vertical_load))
                Horizontal_loads.append(Macauley(horizontal_load))
        elif load_input[4] == "VDL":
            loads_to_add = []
            loads_to_add.append(UDL(load_input[0], load_input[2]))
            loads_to_add.append(UDL(load_input[1], load_input[2]*-1))
            loads_to_add.append(UDL(load_input[1], load_input[3]*-1))
            coefficient = (load_input[3]-load_input[2])/(load_input[1]-load_input[0])
            loads_to_add.append(VDL(load_input[0], coefficient))
            loads_to_add.append(VDL(load_input[1], coefficient*-1))
            for loads in loads_to_add:
                Macauley_list.append(Macauley(loads))
    return Horizontal_loads, Macauley_list


def support_taker(ogsupport_list, length):
    unknowns = 2
    hunknowns = 0
    hsupport_list = []
    support_list = []
    condition_list = []
    hinges = []
    for supports in ogsupport_list:
        if supports[1] == "Fixed":
            unknowns += 2
            hunknowns += 1
            support_list.append([supports[0], "Moment"])
            support_list.append([supports[0], "Vertical"])
            condition_list.append(Condition(4, supports[0], 0))
            condition_list.append(Condition(3, supports[0], 0))
            hsupport_list.append([supports[0]])
        elif supports[1] == "Pin":
            unknowns += 1
            hunknowns += 1
            support_list.append([supports[0], "Vertical"])
            condition_list.append(Condition(4, supports[0], 0))
            hsupport_list.append([supports[0]])
        elif supports[1] == "Hinge":
            hinges.append(Condition(2, supports[0], 0))
        else:
            unknowns += 1
            condition_list.append(Condition(4, supports[0], 0))
            support_list.append([supports[0], "Vertical"])
    Macauley_Supports = []
    counter = 0
    for supports in support_list:
        if supports[1] == "Vertical":
            Macauley_Supports.append(Macauley(Vertical_Support(supports[0])))
        else:
            Macauley_Supports.append(Macauley(Moment_Support(supports[0])))
        counter += 1
    Horizontal_Supports = []
    counter = 0
    for supports in hsupport_list:
        Horizontal_Supports.append(Macauley(Horizontal_Support(supports[0])))
        counter += 1
    condition_list.append(Condition(2, length, 0))
    condition_list.append(Condition(1, length, 0))
    condition_list += hinges
    return Macauley_Supports, condition_list, unknowns, Horizontal_Supports

def Macauley_integrate(list, number):
    integrated_list = deepcopy(list)
    for all in integrated_list:
        all.integrate()
    if number == 0:
        integrated_list.append(Macauley(Moment_Support(0)))
    else:
        integrated_list.append(Macauley(Moment_Support(0)))
    return integrated_list

def Macauley_differentiate(list):
    differentiated_list = deepcopy(list)
    for all in differentiated_list:
        all.differentiate()
    return differentiated_list

def support_remover(supports, unknowns):
    while len(supports) > unknowns:
        max = 0
        for support in supports:
            if max < support.Power_level:
                max = support.Power_level
                to_remove = support
        supports.remove(to_remove)
    return supports

def horizontal_solver(support_list, load_list, length):
    total_length = length
    #RA + RB + RC = (total loads)
    #iterate through from LHS until 2nd horizontal support reached (each time adding -load*distance to RHS, -RA distance to LHS, -RB distance to LHS)
    #then go again starting with ended equation until last support reached
    unknowns = len(support_list)
    equation = 0
    for i in load_list:
        equation += i.magnitude
    equation_list = [equation]
    LHS_equation_list = []
    LHS_append = []
    for i in support_list:
        LHS_append.append(1)
    LHS_equation_list.append(LHS_append)
    LHS_append = []
    for i in range(len(support_list)):
        LHS_append.append(0)
    load_list += support_list
    load_list = sorted(load_list, key=lambda Macauley: Macauley.distance)
    LHS_equation = 0
    unknowns = 0
    unknown_position = -1
    magnitude = 0
    for loads in range(len(load_list)):
        if loads < len(load_list)-1:
            length = load_list[loads+1].distance - load_list[loads].distance
        else:
            length = total_length - load_list[loads].distance
        if load_list[loads] in support_list:
            unknowns += 1
            unknown_position += 1
            if unknowns == 2:
                equation_list.append(LHS_equation)
                LHS_equation_list.append(LHS_append)
                LHS_equation = 0
                LHS_append = []
                for i in range(len(support_list)):
                    LHS_append.append(0)
                unknowns = 1
            else:
                for i in range(unknown_position+1):
                    if i == 0:
                        LHS_append[i] += length
                    else:
                        LHS_append[i] -= length
                LHS_equation += magnitude*length
        else:
            magnitude += load_list[loads].magnitude
            if unknowns == 1:
                for i in range(unknown_position + 1):
                    if i == 0:
                        LHS_append[i] += length
                    else:
                        LHS_append[i] -= length
                LHS_equation += magnitude*length
    solved_supports = np.linalg.solve(LHS_equation_list,equation_list)
    counter = 0
    for load in load_list:
        if load.unknown == 'unknown':
            load.unknown = 'known'
            load.update(solved_supports[counter]*-1)
            counter+=1

    return load_list

    #solved_supports = sp.solve(equation_list, unknown_list)
    #for symbol in unknown_list:
        #for loads in load_list:
            #if loads.magnitude == symbol:
                #loads.update(solved_supports.get(symbol))
    #return load_list

def update_lists(list, solved_supports):
    counter = 0
    for loads in list:
        if loads.unknown == "unknown":
            loads.update(solved_supports[counter]*-1)
            loads.unknown = 'known'
            counter+=1

def support_sort_finder(length, support_list, load_list):  # Finds support reactions
    loads = load_taker(load_list)
    Horizontal_loads = loads[0]
    Macauley_list = loads[1]
    supports = support_taker(support_list, length)
    Macauley_list += supports[0]
    unknowns = supports[2]
    horizontal_support_list = supports[3]
    Macauley_list = sorted(Macauley_list, key=lambda Macauley: Macauley.distance)
    Angle_list = Macauley_integrate(Macauley_list, 0)
    Deflection_list = Macauley_integrate(Angle_list, 1)
    Shear_list = Macauley_differentiate(Macauley_list)
    support_list = support_remover(supports[1], unknowns)
    C = []
    B = []
    for conditions in support_list:
        equation = 0
        A = [] # list of all unknown functions (LHS to RHS with b/c last and 0 for below angle)
        # append sum of all non unknown functions
        if conditions.Power_level == 4:
            for deflections in Deflection_list:
                if deflections.unknown == "known":
                    equation += deflections.calculate(conditions.distance)
                else:
                    A.append(deflections.calculate(conditions.distance))
        elif conditions.Power_level == 3:
            for angles in Angle_list:
                if angles.unknown == "known":
                    equation += angles.calculate(conditions.distance)
                else:
                    A.append(angles.calculate(conditions.distance))
        elif conditions.Power_level == 2:
            for moments in Macauley_list:
                if moments.unknown == "known":
                    equation += moments.calculate(conditions.distance)
                else:
                    A.append(moments.calculate(conditions.distance))
        else:
            for shears in Shear_list:
                if shears.unknown == "known":
                    equation += shears.calculate(conditions.distance)
                else:
                        A.append(shears.calculate(conditions.distance))
        B.append(equation)
        while len(A) < unknowns:
            A.append(0)
        C.append(A)
    solved_supports = np.linalg.solve(C,B)
    update_lists(Macauley_list, solved_supports)
    update_lists(Shear_list, solved_supports)
    update_lists(Angle_list, solved_supports)
    update_lists(Deflection_list, solved_supports)
    #Use lists C, B to work out support values then replace
    horizontal_list = horizontal_solver(horizontal_support_list, Horizontal_loads, length)
    hposition = {'x': 0, 'y': 0.05}
    hlabel = 'Axial Stress (kN)'
    sposition = {'x': 0, 'y': 0.05}
    slabel = 'Shear stress (kN)'
    mposition = {'x': 0, 'y': 0.05}
    mlabel = 'Bending moment (kNm)'
    dposition = {'x': 0, 'y': 0.05}
    dlabel = 'Deflection (1/EI)'
    Shear_graph = all_graph(Shear_list, length, sposition, slabel)
    Moment_graph = all_graph(Macauley_list, length, mposition, mlabel)
    Deflection_graph = all_graph(Deflection_list, length, dposition, dlabel)
    Axial_graph = all_graph(horizontal_list, length, hposition, hlabel)
    return Shear_graph, Moment_graph, Deflection_graph, Axial_graph

def all_graph(Shear_list, length, position, label):
    datapoints = np.arange(0, length, 0.01)
    point_data_list = []
    multiplier = 1
    if label == "Bending moment (kNm)" or label == "Deflection (1/EI)" or 'Axial Stress (kN)':
        multiplier = -1
    ymax = 0
    ymin = 0
    min_y = 0
    max_y = 0
    for i in range(len(datapoints)):
        point_data = 0
        for all in Shear_list:
            point_data += all.calculate(datapoints[i])*multiplier
        if point_data + abs(point_data / 5) > ymax:
            ymax = point_data + abs(point_data / 5)
        elif ymin > point_data - abs(point_data / 5):
            ymin = point_data - abs(point_data / 5)
        point_data_list.append((datapoints[i], point_data))
        if i == 0:
            max_y = point_data
            min_y = point_data
        else:
            max_y = max(point_data, max_y)
            min_y = min(point_data, min_y)
    ymax = int(round(ymax, 0))
    ymin = int(round(ymin, 0))
    max_y = round(max_y, 3)
    min_y = round(min_y, 3)
    if ymax == 0:
        ymax = 1
    if ymin == 0:
        ymin = -1
    graph = Graph(pos_hint=position, size_hint=(0.7, 0.9), xlabel='Position', ylabel=label, x_ticks_minor=5,
                  x_ticks_major=length / 10, x_grid_label=True, y_grid_label=True, y_ticks_major=(ymax-ymin)/5,
                  y_ticks_minor=5, padding=5, x_grid=True, y_grid=True, xmax=length, xmin=0, ymin=ymin, ymax=ymax)
    plot = MeshLinePlot()
    plot.points = point_data_list
    graph.add_plot(plot)
    return graph, [min_y, max_y]

## Starting the App

kv = Builder.load_file("Test.kv")

class MyApp(App):

    def build(self):
        return kv

if __name__ == "__main__":
    MyApp().run()
