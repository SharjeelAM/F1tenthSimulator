#include <ros/ros.h>
#include <std_msgs/Float32.h>

class SimpleNode {
    public:
    SimpleNode(){
        nh_ = ros::NodeHandle();

        pub_ = nh_.advertise<std_msgs::Float32>("/f1tenth/velocity", 1000000000);
    }

    void run(){
        ros::Rate loop_rate(10);

        while (ros::ok()){
            std_msgs::Float32 msg;
            msg.data = 70000000;

            pub_.publish(msg);

            loop_rate.sleep();
        }
    }

    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
};

int main(int argc, char** argv) {
    
    ros::init(argc, argv, "simple_node");

    SimpleNode simple_node;

    simple_node.run();

    return 0;
}