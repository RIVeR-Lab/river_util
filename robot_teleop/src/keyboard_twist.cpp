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

#define INCREASE_POWER_CH '\''
#define DECREASE_POWER_CH '/'

int main(int argc, char **argv)
{
  ros::init(argc, argv, "keyboard_twist_teleop", ros::init_options::AnonymousName);

  ros::NodeHandle n;

  ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  initscr();
  cbreak();
  halfdelay(7);//getch delays 7/10 second
  noecho();
  curs_set(0);//hide the cursor
  printw("Keyboard Twist Teleop\n");
  printw("Forward: %c\n", FORWARD_CH);
  printw("Forward Left: %c\n", FORWARD_LEFT_CH);
  printw("Forward Right: %c\n", FORWARD_RIGHT_CH);
  printw("Backward: %c\n", BACKWARD_CH);
  printw("Left: %c\n", LEFT_CH);
  printw("Right: %c\n", RIGHT_CH);
  printw("Stop: %c\n", STOP_CH);
  printw("Power +: %c\n", INCREASE_POWER_CH);
  printw("Power -: %c\n", DECREASE_POWER_CH);
  printw("Quit: %c\n", QUIT_CH);
  refresh();

  int ch = ERR;
  double power = 1.0;
  while (ros::ok())
    {
      int newCh = getch();
      if(newCh!=ERR)//a key was pressed
	ch = newCh;
      else
	ch = STOP_CH;

      if(ch==INCREASE_POWER_CH){
	power += 0.1;
	ch = STOP_CH;
      }
      else if(ch==DECREASE_POWER_CH){
	power -= 0.1;
	if(power<0)
	  power = 0;
	ch = STOP_CH;
      }

      geometry_msgs::Twist msg;
	
      if(ch==FORWARD_CH){
	msg.linear.x = power;
	msg.angular.z = 0.0;
	vel_pub.publish(msg);
      }
      else if(ch==BACKWARD_CH){
	msg.linear.x = -power;
	msg.angular.z = 0.0;
	vel_pub.publish(msg);
      }
      else if(ch==LEFT_CH){
	msg.linear.x = 0.0;
	msg.angular.z = power;
	vel_pub.publish(msg);
      }
      else if(ch==RIGHT_CH){
	msg.linear.x = 0.0;
	msg.angular.z = -power;
	vel_pub.publish(msg);
      }
      else if(ch==FORWARD_LEFT_CH){
	msg.linear.x = power;
	msg.angular.z = power/2;
	vel_pub.publish(msg);
      }
      else if(ch==FORWARD_RIGHT_CH){
	msg.linear.x = power;
	msg.angular.z = -power/2;
	vel_pub.publish(msg);
      }
      else if(ch==QUIT_CH){
	msg.linear.x = 0.0;
	msg.angular.z = 0.0;
	vel_pub.publish(msg);
	ros::shutdown();
      }
      else{//anything else is stop
	msg.linear.x = 0.0;
	msg.angular.z = 0.0;
	vel_pub.publish(msg);
      }

      move(12, 0);
      clrtoeol();
      printw("power: %.2f", power);

      move(14, 0);
      clrtoeol();
      printw("linear: %.2f, angular: %.2f", msg.linear.x, msg.angular.z);

      ros::spinOnce();
    }

  endwin();

  return 0;
}
