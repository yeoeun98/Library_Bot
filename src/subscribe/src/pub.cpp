// ROS 기본 헤더파일
#include "ros/ros.h"
// (패키지이름 / 메세지파일.h)
#include "subscribe/subscribe_msg.h" 

int main(int argc, char **argv)
{
  // 노드명 초기화
  ros::init(argc, argv, "subscribe_pub");
  // ROS시스템과 통신을 위한 노드핸들 선언 
  ros::NodeHandle nh; 

  ros::Publisher pub = nh.advertise<subscribe::subscribe_msg>("subscribe_topic", 100);
  //퍼블리시어 선언, 패키지 ()의 메시지파일()을 이용
  //퍼블리시어()을 작성
  //토픽명은 ()이며 퍼블리시어큐 사이즈를 ()개로 설정
  //publisher이기 때문에 advertise 만듬

  //루프 주기를 2Hz로 설정 (1초에 2번)
  ros::Rate loop_rate(2);

  // sunscribe_msg 메시지 파일 형식으로 msg 라는 메시지를 선언
  // (()::() ())
  subscribe::subscribe_msg msg;

  // 메시지에 사용될 변수 선언
  int count = 0;

  //ros 가 활성화되면
  while(ros::ok())
  {
    //현재 시간을 msg 하위 stamp메세지에 담음
    msg.stamp = ros::Time::now();
    //count 변수 값을 msg 하위 data 메세지에 담음
    msg.data = count;
    // 메시지를 표시
    ROS_INFO("send msg = %d", msg.stamp.sec);
    ROS_INFO("send msg = %d", msg.stamp.nsec);
    ROS_INFO("send msg = %d", msg.data);

    // 메시지를 발행
    pub.publish(msg);
    // 위에서 정한 루프 주기에 따라 슬립
    loop_rate.sleep();
    ++count;
  }
  return 0;
}
