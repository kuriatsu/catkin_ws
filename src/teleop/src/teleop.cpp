#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <sound_play/sound_play.h>
#include <cmath>

class joyteleop{
    private :
        ros::Subscriber sub;
        ros::Publisher pub;
        geometry_msgs::Twist twist;
		sound_play::SoundClient sc;
		int emerjency;
		bool joymode;

	public :
        joyteleop();
        void joyop_loop();
		bool getmode();

    private :
        void joyCallback(const sensor_msgs::Joy &joy_msg);

};

class YpAutoware{
	private :
		ros::Subscriber sub;
		ros::Publisher pub;
		geometry_msgs::Twist twist;
		
	public :
		YpAutoware();
		joyteleop joyop;

	private :
		void YpCallback(const geometry_msgs::TwistStamped &twistamp);
};


YpAutoware::YpAutoware(){
	ros::NodeHandle n;
    sub = n.subscribe("/twist_cmd", 1, &YpAutoware::YpCallback, this);
    pub = n.advertise<geometry_msgs::Twist>("/ypspur_ros/cmd_vel", 1);

}

void YpAutoware::YpCallback(const geometry_msgs::TwistStamped &twistamp){
	twist.linear.x = twistamp.twist.linear.x;
	twist.angular.z = twistamp.twist.angular.z;

	if(!joyop.getmode()){
		pub.publish(twist);
		ROS_INFO("automode\n");
	}
}




joyteleop::joyteleop(){
	ros::NodeHandle n;

	emerjency = 1;
	joymode = true;
    sub = n.subscribe("joy", 1, &joyteleop::joyCallback, this);
    pub = n.advertise<geometry_msgs::Twist>("/ypspur_ros/cmd_vel", 1);

}

void joyteleop::joyop_loop() {
    ros::Rate loop_rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        if (joymode) pub.publish(twist);
        loop_rate.sleep();
    }

return;
}


void joyteleop::joyCallback(const sensor_msgs::Joy &joy_msg) {
	int dash = 1;

	if (joy_msg.buttons[0]){
		if(!joymode){
			joymode = true;
			sc.playWave("/usr/share/sounds/robot_sounds/pipe.wav");
			ROS_INFO("joymode\n");
		}else if(joymode){
			joymode = false;
			sc.playWave("/usr/share/sounds/robot_sounds/powerup.wav");
			ROS_INFO("automode\n");
		}
	}

	if (joy_msg.buttons[2]){
		if (emerjency==1.0){
			(emerjency)+=0.5;
		}else if (emerjency==1.5){
			(emerjency)-=0.5;
		}
	}

	if (joy_msg.buttons[4]){
		dash = 2;
	}

	if (joy_msg.buttons[1]){
		 sc.playWave("/usr/share/sounds/robot_sounds/jump.wav");
	}
	if (joy_msg.buttons[3]){
		 sc.playWave("/usr/share/sounds/robot_sounds/coin.wav");
	}

	twist.linear.x = (emerjency)*dash*0.5*joy_msg.axes[3];
    twist.angular.z = dash*0.5*joy_msg.axes[0]*joy_msg.axes[3];

    return;

}

bool joyteleop::getmode(){
	return joymode;
}


int main(int argc, char **argv) {

    ros::init(argc, argv, "kuri_ypjoyteleop");
	ros::NodeHandle n;
	YpAutoware ypauto;
	if(ypauto.joyop.getmode()){
		ypauto.joyop.joyop_loop();
	}
    return (0);

}
