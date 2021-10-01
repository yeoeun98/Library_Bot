#!/usr/bin/env python3
#
import roslib
import sys
import time
import tkinter
import rospy
from geometry_msgs.msg import PoseStamped #이동에 필요한 좌표의 형태: PoseStamped
from std_msgs.msg import String
from std_msgs.msg import Int32

posePub = rospy.Publisher("posePoint",PoseStamped,queue_size=1)
# goal = PoseStamped()
# current_pose = PoseStamped()
# current_goal = PoseStamped()
# person_info = Int32()
# tf = False

# def tfTrue():
    # global tf
    # tf = True

# def tfFalse():
    # global tf
    # tf = False

rospy.init_node("posePub", anonymous=True)

#딕셔너리에 넣기에 좌표의 정보가 많으므로 매개 변수로 사용할 정수를 설정
mybooks = [{"도서명":"Apple", "서가 위치": "A열", "책장 위치": "1층", "좌표":1},
           {"도서명":"Python", "서가 위치": "B열", "책장 위치": "1층", "좌표":2},
           {"도서명":"ROS", "서가 위치": "C열", "책장 위치": "1층", "좌표":3},
           {"도서명":"HTML5", "서가 위치": "D열", "책장 위치": "2층", "좌표":4},
           {"도서명":"Compiler", "서가 위치": "E열", "책장 위치": "2층", "좌표":5},
           {"도서명":"Samsung","서가 위치": "F열", "책장 위치": "2층", "좌표":6},
           {"도서명":"Programming", "서가 위치": "G열", "책장 위치": "3층", "좌표":7},
           {"도서명":"Android", "서가 위치": "H열", "책장 위치": "3층", "좌표":8},
           {"도서명":"C", "서가 위치": "I열", "책장 위치": "3층", "좌표":9}]

keyword = {'Apple':'도서명', 'Python':'도서명', 'ROS':'도서명', 'HTML5':'도서명', 'Compiler':'도서명',
           'Samsung':'도서명', 'Programming':'도서명', 'Android':'도서명', 'C':'도서명'}


# def startPose(start):
    # rospy.loginfo("start x : %f, y : %f", start.pose.position.x, start.pose.position.y)

def callback(data):
    rospy.loginfo("goal x:%f y:%f", data.pose.position.x, data.pose.position.y);
    start_goal = PoseStamped()
    start_goal.header.frame_id = "map"
    start_goal.header.stamp = rospy.Time.now()
    
    # simulation
    #start_goal.pose.position.x = -2.0
    #start_goal.pose.position.y = -0.4
    #start_goal.pose.position.z = 0.0
    #start_goal.pose.orientation.x = 0.0
    #start_goal.pose.orientation.y = 0.0
    #start_goal.pose.orientation.z = 0.0
    #start_goal.pose.orientation.w = 0.9
    
    # map
    start_goal.pose.position.x = 1.525000
    start_goal.pose.position.y = 0.270000
    start_goal.pose.position.z = 0.0
    start_goal.pose.orientation.x = 0.0
    start_goal.pose.orientation.y = 0.0
    start_goal.pose.orientation.z = -0.7
    start_goal.pose.orientation.w = 0.6

    if(abs(data.pose.position.x - start_goal.pose.position.x)>0.1):
        if(abs(data.pose.position.y - start_goal.pose.position.y)>0.1):
            goalTake(start_goal)
    start_goal.clear()

# arriveSub = rospy.Subscriber("arrive_info", String, queue_size = 1)
goalSub = rospy.Subscriber("goal", PoseStamped, callback)
# startSub = rospy.Subscriber("start", PoseStamped, startPose)

# def currentPose(data):
    # current_pose.header.frame_id = "map"
    ##current_pose.header.stamp = rospy.Time.now()
    # current_pose.pose = data.pose

# currentSub = rospy.Subscriber("current", PoseStamped, currentPose)


# def personInfo(data):
    # rospy.loginfo("tf : %d", tf)
    # person_info = data.data
    # if(person_info == 0):
        # rospy.loginfo("사람 있을때tf : %d", tf)
        # if(tf == False):
            # current_pose.header.stamp = rospy.Time.now()
            # current_goal.pose = current_pose.pose
            # goalTake(current_pose)
            # tfTrue()
    # else:
        # if(tf == True):
            # goalTake(goal)
            # tfFalse()
    # rospy.loginfo("tf : %d", tf)
# personSub = rospy.Subscriber("person_info", Int32, personInfo)


def findBooks(self, key):
    matchList = []
    for idx, val in enumerate(mybooks):
        if key == val.get(keyword.get(key)):
            matchList.append(idx)
    return matchList

def printResult(self,findResult):
    for target in findResult:
        self.labelResult.set("\n서가 위치: {}\n책장 위치: {}\n".format(mybooks[target].get('서가 위치'),
                                                    mybooks[target].get('책장 위치')))
        poseDect = mybooks[target].get('좌표')
        break
    poseTake(poseDect)


def poseTake(poseDect):
    #publish할 goal을 PoseStamped 타입으로 사용
    goal = PoseStamped() 
    #header 설정
    goal.header.frame_id = "map"
    goal.header.stamp = rospy.Time.now()
   
    #도서마다 좌표 설정
    if poseDect == 1:
        # simulation
        #goal.pose.position.x = 1.5
        #goal.pose.position.y = -1.0
        #goal.pose.position.z = 0.0
        #goal.pose.orientation.x = 0.0
        #goal.pose.orientation.y = 0.0
        #goal.pose.orientation.z = 0.9
        #goal.pose.orientation.w = 0.3

        # map 6
        # goal.pose.position.x = 2.184999
        # goal.pose.position.y = -3.089999
        # goal.pose.position.z = 0.0
        # goal.pose.orientation.x = 0.0
        # goal.pose.orientation.y = 0.0
        # goal.pose.orientation.z = 0.9
        # goal.pose.orientation.w = 0.3
        
        # map6!
        goal.pose.position.x = 1.305
        goal.pose.position.y = -2.6
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = -0.743198
        goal.pose.orientation.w = 0.669072
        goalTake(goal)

    elif poseDect == 2:
        #simulation
        #goal.pose.position.x = 0.5
        #goal.pose.position.y = 1.7
        #goal.pose.position.z = 0.0
        #goal.pose.orientation.x = 0.0
        #goal.pose.orientation.y = 0.0
        #goal.pose.orientation.z = 0.9
        #goal.pose.orientation.w = 0.3

        #map 4
        goal.pose.position.x = 2.504999
        goal.pose.position.y = -1.530000
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.026288
        goal.pose.orientation.w = 0.998697
        goalTake(goal)

    elif poseDect == 3:
        #simulation
        #goal.pose.position.x = -0.5
        #goal.pose.position.y = -1.9
        #goal.pose.position.z = 0.0
        #goal.pose.orientation.x = 0.0
        #goal.pose.orientation.y = 0.0
        #goal.pose.orientation.z = 0.9
        #goal.pose.orientation.w = 0.2

        #map 2
        goal.pose.position.x = 2.674999
        goal.pose.position.y = -0.100000
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = -0.0
        goal.pose.orientation.w = 1.0
        goalTake(goal)

    elif poseDect == 4:
        #goal.pose.position.x = 0.1
        #goal.pose.position.y = 1.5
        #goal.pose.position.z = 0.0
        #goal.pose.orientation.x = 0.0
        #goal.pose.orientation.y = 0.0
        #goal.pose.orientation.z = 0.9
        #goal.pose.orientation.w = 0.3

        #map 1
        goal.pose.position.x = 0.195000
        goal.pose.position.y = 0.230000
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.999941
        goal.pose.orientation.w = -0.010868

#설정된 goal을 publish
def goalTake(goal):
    rospy.loginfo(goal)
    posePub.publish(goal)

def BookNumber(self, bookname):
    findResult = findBooks(self, bookname)
    printResult(self, findResult) if findResult != [] else self.labelResult.set("\n검색한 도서가 없습니다.")


class Searching:
    def __init__(self, window, title):
        self.window = window
        self.window.title(title)
        self.window.geometry("300x200")

        self.labelName = tkinter.Label(window,text="도서명을 입력하세요.")
        self.labelName.pack(pady=10)

        self.entryValue = tkinter.StringVar()
        self.entry = tkinter.Entry(window, textvariable=self.entryValue)
        self.entry.bind('<Return>', self.getTextEnter)
        self.entry.pack(pady=5)
        self.entry.focus() #검색창 위에 커서 위치

        self.button = tkinter.Button(window,text="검색", command=self.getTextClick)
        self.button.pack(pady=5)

        self.labelValue = tkinter.StringVar()
        self.label = tkinter.Label(window, fg='black',textvariable=self.labelValue)
        self.label.pack()

        self.labelResult = tkinter.StringVar()
        self.label = tkinter.Label(window, fg='black',textvariable=self.labelResult)
        self.label.pack()

        self.window.mainloop()
    
    def getTextEnter(self, event):
        bookname = self.entryValue.get()
        for idx, val in enumerate(keyword):
            if val == bookname:
                self.labelValue.set('도서명: ' + bookname)
                break
            else:
                self.labelValue.set('도서명: ' + bookname)
        self.entryValue.set('')
        BookNumber(self, bookname)
        return bookname
    
    def getTextClick(self):
        bookname = self.entryValue.get()
        for idx, val in enumerate(keyword):
            if val == bookname:
                self.labelValue.set('도서명: ' + bookname)
                break
            else:
                self.labelValue.set('도서명: ' + bookname)
        self.entryValue.set('')
        BookNumber(self, bookname)
        return bookname
    

def main(args):
    Searching(tkinter.Tk(),"도서 검색 시스템")

    
 

if __name__ == '__main__':
    main(sys.argv)