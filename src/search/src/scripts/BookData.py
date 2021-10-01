#!/usr/bin/env python3
#
import roslib
import sys
import time
import tkinter
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import csv

global popWarning
BOOK_INDEX_NUM = 9 #book num
CSV_ADDRESS = '/home/user/catkin_ws/src/search/csv/data.csv' #csv파일의 주소 입력

# bookIdx: ['Apple', 'Python', 'ROS', 'HTML5', 'Compiler', 'Samsung', 'Programming', 'Android', 'C']
# Apple, Python, ROS, HTML5: 시뮬레이션 좌표
# Compiler, Samsung, Programming, Android, C: 실제 구동 좌표
# line[10]: simulation pose / line[11]: map pose

bookIdx = []
bookStatus = []
with open(CSV_ADDRESS) as csvfile:
    reader = csvfile.readline().replace("\n","")
    bookIdx = reader.split(",")

rospy.init_node("posePub", anonymous=True)
posePub = rospy.Publisher("posePoint",PoseStamped,queue_size=1)
bookPub = rospy.Publisher("bookPoint",String,queue_size=1)
arrivePub = rospy.Publisher("bookArrive",String,queue_size=1)

class BookSearch:
    def setResult(bookGoal):
        global posePub
        global bookPub
        goal = PoseStamped() 
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = float(bookGoal[3])
        goal.pose.position.y = float(bookGoal[4]) 
        goal.pose.position.z = float(bookGoal[5])
        goal.pose.orientation.x = float(bookGoal[6])
        goal.pose.orientation.y = float(bookGoal[7])
        goal.pose.orientation.z = float(bookGoal[8])
        goal.pose.orientation.w = float(bookGoal[9])
        rospy.loginfo(goal)
        posePub.publish(goal)
        bookPub.publish(bookGoal[0])

    def printResult(self,result):
        if result == []: 
            self.labelResult.set("\n검색한 도서가 없습니다.")
            self.labelArrive.set("현재 위치: 출발지")
        else:
            self.labelResult.set("\n서가 위치: {}\n책장 위치: {}\n".format(result[1], result[2]))
            self.labelArrive.set("목적지로 이동합니다.")
            UISystem.createPop(self)

    def bookResult(self, bookname):
        bookInfo = []
        bookInfo.clear()
        idx = 0
        for i in range(len(bookIdx)):
            if bookIdx[i] != bookname:  idx += 1
            else: break
        if idx > BOOK_INDEX_NUM: pass
        else:
            with open(CSV_ADDRESS,newline='') as csvfile:
                reader = csv.reader(csvfile)
                for line in reader:
                    bookInfo.append(line[idx])
            BookSearch.setResult(bookInfo)
        BookSearch.printResult(self, bookInfo)
        bookStatus.append(bookInfo)


class UISystem:
    def arriveHome(self, data):
        global arrivePub
        rospy.loginfo("goal x: %f y: %f", data.pose.position.x, data.pose.position.y)
        start_goal=[]
        start_goal.clear()
        with open(CSV_ADDRESS,newline='') as csvfile:
                reader = csv.reader(csvfile)
                for line in reader:
                    start_goal.append(line[11])
        print(abs(float(bookStatus[-1][3]) - float(start_goal[3])))
        #if(abs(data.pose.position.x - float(start_goal[3])>0.1)) & (abs(data.pose.position.y - float(start_goal[4]))>0.1):
        if((abs(float(bookStatus[-1][4]) - float(start_goal[4]))>0.1)):
            print('IN')
            BookSearch.setResult(start_goal)
            bookStatus.clear()
            bookStatus.append(start_goal)
            self.labelArrive.set("목적지에 도착했습니다.")

        else: 
            arrivePub.publish("FINISH")
            self.labelArrive.set("현재 위치: 출발지")
            UISystem.closePop()


    def __init__(self, window, title):
        self.window = window
        self.window.title(title)
        self.window.geometry("300x240")

        self.labelName = tkinter.Label(window,text="도서명을 입력하세요.")
        self.labelName.pack(pady=10)

        self.entryValue = tkinter.StringVar()
        self.entry = tkinter.Entry(window, textvariable=self.entryValue)
        self.entry.bind('<Return>', self.getTextEnter)
        self.entry.pack(pady=5)
        self.entry.focus()

        self.button = tkinter.Button(window,text="검색", command=self.getTextClick)
        self.button.pack(pady=5)

        self.labelValue = tkinter.StringVar()
        self.label = tkinter.Label(window, fg='black',textvariable=self.labelValue)
        self.label.pack()

        self.labelResult = tkinter.StringVar()
        self.label = tkinter.Label(window, fg='black',textvariable=self.labelResult)
        self.label.pack()

        self.labelArrive = tkinter.StringVar()
        self.label = tkinter.Label(window, fg='black',textvariable=self.labelArrive)
        self.labelArrive.set("현재 위치: 출발지")
        self.label.pack()

        goalSub = rospy.Subscriber("goal", PoseStamped, self.arriveHome)

        self.window.mainloop()

    def getTextEnter(self, event):
        UISystem.getTextClick(self)

    def getTextClick(self):
        bookname = self.entryValue.get()
        self.labelValue.set('도서명: ' + bookname)
        self.entryValue.set('')
        BookSearch.bookResult(self, bookname)

    def closePop():
        popWarning.destroy()

    def createPop(self):
        global popWarning 
        popWarning = tkinter.Toplevel()
        popWarning.title("WARNING")
        popWarning.configure(bg='red')
        popWarning.geometry("500x240")

        popWarning.label = tkinter.Label(popWarning,bg='red',text="로봇이 작동 중입니다.")
        popWarning.label.pack(pady=5)

        popWarning.button = tkinter.Button(popWarning,bg='black',fg='white',text="close",command = UISystem.closePop)
        popWarning.button.pack(pady=5)
            

def main(args):
    UISystem(tkinter.Tk(),"도서 검색 시스템")
 
if __name__ == '__main__':
    main(sys.argv)
