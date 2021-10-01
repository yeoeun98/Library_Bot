#!/usr/bin/env python3
#
import sys
import tkinter as tk
from PIL import ImageTk, Image
import rospy
from std_msgs.msg import String

rospy.init_node("poseSub", anonymous=True)

bookIdx = []
with open('/home/user/catkin_ws/src/search/csv/data.csv') as csvfile:
    reader = csvfile.readline().replace("\n","")
    bookIdx = reader.split(",")

class Display():
    def mapDisplay(self):
        mapPng = Image.open("/home/user/catkin_ws/src/display/img/map.png")
        mainPng = mapPng.resize((800,330))

        self.img = ImageTk.PhotoImage(mainPng)
        self.image_on_canvas = self.canvas.create_image(0, 0, anchor=tk.NW)
        self.canvas.itemconfig(self.image_on_canvas, image=self.img)

    def poseDisplay(self, data):
        print(data)
        self.labelPos.set("목적지로 이동합니다.")
        strData = str(data)
        bookname = strData[7:-1]
        idx = 0
        for i in range(len(bookIdx)):
            if bookIdx[i] != bookname: 
                idx += 1        
            else: break
        Display.bookPose(self,idx)

    def arriveDisplay(self,data):
        self.labelPos.set("현재 위치: 출발지")
        Display.mapDisplay(self)

    def bookPose(self,num):
        if num <= 9:
            # Android, Compiler, HTML5, Python, Samsung, ROS
            place = [(0, 0),(105, 230),(105,40),(300, 230),(300,40),(533, 230),(533,40)]
            mapPng = Image.open("/home/user/catkin_ws/src/display/img/map.png")
            background = mapPng.resize((800,330))
            pointPng = Image.open("/home/user/catkin_ws/src/display/img/pin.png")
            foreground = pointPng.resize((70,70))
            background.paste(foreground,place[num])

            self.img = ImageTk.PhotoImage(background)
            self.canvas.itemconfig(self.image_on_canvas, image=self.img)

        else: self.labelPos.set("목적지에 도착했습니다.")

    def __init__(self, window, title):
        self.window = window
        self.window.title(title)

        self.labelPos = tk.StringVar()
        self.label = tk.Label(window, fg='black',textvariable=self.labelPos)
        self.labelPos.set("현재 위치: 출발지")
        self.label.grid(row=0, column=0,pady=5)

        self.canvas = tk.Canvas(window, width=800, height=370)
        self.canvas.grid(row=1, column=0)

        Display.mapDisplay(self)

        bookSub = rospy.Subscriber("bookPoint", String, self.poseDisplay)
        arriveSub = rospy.Subscriber("bookArrive", String, self.arriveDisplay)

        self.window.mainloop()

def main(args):
    Display(tk.Tk(),"지도")
 
if __name__ == '__main__':
    main(sys.argv)
