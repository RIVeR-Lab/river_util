#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <ncurses.h>

#include <sstream>

#define FORWARD_CH 'w'
#define FORWARD_LEFT_CH 'q'
#define FORWARD_RIGHT_CH 'e'
#define BACKWARD_CH 's'
#define LEFT_CH 'a'
#define RIGHT_CH 'd'
#define STOP_CH 'z'
#define QUIT_CH 'p'

int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyboard_twist_teleop");

  ros::NodeHandle n;

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  initscr();
  cbreak();
  halfdelay(1);
  noecho();
  printw("Keyboard Twist Teleop\n");
  printw("Forward: %c\n", FORWARD_CH);
  printw("Forward Left: %c\n", FORWARD_LEFT_CH);
  printw("Forward Right: %c\n", FORWARD_RIGHT_CH);
  printw("Backward: %c\n", BACKWARD_CH);
  printw("Left: %c\n", LEFT_CH);
  printw("Right: %c\n", RIGHT_CH);
  printw("Stop: %c\n", STOP_CH);
  printw("Quit: %c\n", QUIT_CH);

  refresh();

  int ch = ERR;
  while (ros::ok())
    {
      int newCh = getch();
      if(newCh!=ERR)
	ch = newCh;

      geometry_msgs::Twist msg;
	
      if(ch==FORWARD_CH){
	msg.linear.x = 1.0;
	msg.angular.z = 0.0;
	vel_pub.publish(msg);
      }
      else if(ch==BACKWARD_CH){
	msg.linear.x = -1.0;
	msg.angular.z = 0.0;
	vel_pub.publish(msg);
      }
      else if(ch==LEFT_CH){
	msg.linear.x = 0.0;
	msg.angular.z = 1.0;
	vel_pub.publish(msg);
      }
      else if(ch==RIGHT_CH){
	msg.linear.x = 0.0;
	msg.angular.z = -1.0;
	vel_pub.publish(msg);
      }
      else if(ch==FORWARD_LEFT_CH){
	msg.linear.x = 1.0;
	msg.angular.z = 0.5;
	vel_pub.publish(msg);
      }
      else if(ch==FORWARD_RIGHT_CH){
	msg.linear.x = 1.0;
	msg.angular.z = -0.5;
	vel_pub.publish(msg);
      }
      else if(ch==STOP_CH){
	msg.linear.x = 0.0;
	msg.angular.z = 0.0;
	vel_pub.publish(msg);
      }
      else if(ch==QUIT_CH){
	msg.linear.x = 0.0;
	msg.angular.z = 0.0;
	vel_pub.publish(msg);
	ros::shutdown();
      }

      ros::spinOnce();
    }

  endwin();

  return 0;
}
