#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <assignment2/ArmInfo.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>

/**
 * @brief Class done to make the arm of the robot move to detect all markers
 * 
 */
class MoveJoint {

private:
    ros::NodeHandle nh_;
    // Initialize ROS server and publishers
    ros::Publisher move_cam_pub;
    ros::Publisher move_base_pub;
    ros::ServiceServer motion_srv;
    // Variables initialization
    std_msgs::Float64 joint1_pos;
    std_msgs::Float64 joint2_pos;
    bool move_arm = true;          // Variable to determine if the arm should move or stop
    bool cam_up = true;            // Variable to determine if the camera should look up or down
    double current_base_pos = 0;   // Variable to store the current base joint position
    double current_cam_pos = 0;    // Variable to store the current camera joint position
    double full_base_cycle = 6.2;  // Variable to determine when full cycle for the base joint is obtained
    double cam_lim = 0.2;          // Variable to determine the motion for the camera joint 
    
public:
    /**
     * @brief Construct of the MoveJoint class.
     * After the initialization of publishers and services is done, the arm is moved until all markers have been detected.
     * After that, the node is shut down.
     * 
     */
    MoveJoint() {
    	std::cout << "\nNode Move_camera was correctly launched\n\n\n"; 
        move_cam_pub = nh_.advertise<std_msgs::Float64>("/robot/joint2_position_controller/command", 10);
        move_base_pub = nh_.advertise<std_msgs::Float64>("/robot/joint1_position_controller/command", 10);
        motion_srv = nh_.advertiseService("/arm_info", &MoveJoint::motionCallback, this);
        ROS_INFO("Move the arm until markers have been detected!\n");
        // Loop until all markers have been detected
        while(move_arm){
            ROS_INFO("TILTING THE CAMERA!\n");
            // Rotate the base joint
            rotate_camera_joint(move_cam_pub);
            ROS_INFO("ROTATE THE BASE!\n");
            // Rotate the camera joint
            rotate_base_joint(move_base_pub);
            // Make the camera go up or down
            cam_up = !cam_up;
            // Check if some information arrived from the client
            ros::spinOnce();
        }
        // ROS Shutdown
        ros::shutdown(); // Done once all markers have been detected       
    } 
    
    /**
     * @brief Function done to move the camera joint
     * 
     * @param move_cam_pub publisher for the camera joint command
     * 
     * This method allow the camera joint to tilt of about 20 degrees both upwards or downwards
     * depending on the value of the cam_up variable. If it is true, the camera tilts upwards,
     * else it tilts downwards.
     */
    void rotate_camera_joint(ros::Publisher move_cam_pub) {
        // Set the rate of the loop to run at 10 Hz
        ros::Rate loop_rate(10);
        // If the camera should go up
        if(cam_up){
            // Loop until the current camera position is equal to 30 degrees
	        while (current_cam_pos < cam_lim) {
                // Create a message to store the joint position
		        joint2_pos.data = current_cam_pos;
	            // Publish the message to the robot/joint2_position_controller/command topic
	            move_cam_pub.publish(joint2_pos);
		        // Increment the current position by 0.5 degrees
		        current_cam_pos += 0.03;
	            // Sleep for the time remaining until the next loop iteration
	            loop_rate.sleep();
	        }
        }
        else {
            // Loop until the current camera position is equal to -30 degrees
            while (current_cam_pos > -cam_lim) {
                // Create a message to store the joint position
		        joint2_pos.data = current_cam_pos;
	            // Publish the message to the robot/joint2_position_controller/command topic
	            move_cam_pub.publish(joint2_pos);
		        // Decrement the current position by 0.5 degrees
		        current_cam_pos -= 0.03;
	            // Sleep for the time remaining until the next loop iteration
	            loop_rate.sleep();
	        }
        }
    }  
    
    /**
     * @brief Function done to move the base joint
     * 
     * @param move_cam_pub publisher for the base joint command
     * 
     * This method allow the base joint to rotate about itself of 360 degrees.
     */
    void rotate_base_joint(ros::Publisher move_base_pub) {
        // Set the rate of the loop to run at 10 Hz
        ros::Rate loop_rate(10);
        // Loop until the current base position is equal to home position
        while (current_base_pos < full_base_cycle)
        {
            // Create a message to store the joint position
            joint1_pos.data = current_base_pos;
            // Publish the message to the robot/joint1_position_controller/command topic
            move_base_pub.publish(joint1_pos);
            // Increment the current position by 1 degree
            current_base_pos += 0.1;
            // Sleep for the time remaining until the next loop iteration
            loop_rate.sleep();
        }
        current_base_pos = 0;        
    }  
    
    /**
     * @brief Function done to move the arm to its home position
     * 
     * @param move_base_pub publisher for the base joint command
     * @param move_cam_pub publisher for the camera joint command
     * 
     * Method called once all markers have been detected.
     * This is done to allow the arm return in its home position. It is done in two steps:
     * 1) Make the base joint rotate until it reaches its home position
     * 2) Make the camera joint rotate until it reaches its home position
     */
    void move_to_surv_pos(ros::Publisher move_base_pub, ros::Publisher move_cam_pub) {
        // Set the rate of the loop to run at 20 Hz
        ros::Rate loop_rate(20);
        // Loop until the current position is equal to 360 degrees
        while (current_base_pos > 0)
        {
            joint1_pos.data = current_base_pos;
            move_base_pub.publish(joint1_pos);
            current_base_pos -= 0.1;
            loop_rate.sleep();
        }
        // Base joint to home position
        current_base_pos = 0;
        joint1_pos.data = current_base_pos;
        move_base_pub.publish(joint1_pos);
        // Loop until the current camera position is equal to home position
        while (current_cam_pos > 0) { // If cam was pointing upwards
	        joint2_pos.data = current_cam_pos;
	        move_cam_pub.publish(joint2_pos);
	        current_cam_pos -= 0.03;
	        loop_rate.sleep();
        }
        while (current_cam_pos < 0) { // If cam was pointing downwards
	        joint2_pos.data = current_cam_pos;
	        move_cam_pub.publish(joint2_pos);
	        current_cam_pos += 0.03;
	        loop_rate.sleep();
        }
        // Camera joint to home position
        current_cam_pos = 0;
        joint2_pos.data = current_cam_pos;
        move_cam_pub.publish(joint2_pos);
    }  
    
    /**
     * @brief Function called when the client node makes a request.
     * 
     * @param req int to state if all markes have been detected.
     * @param res int to state that the server executed correctly.
     * @return true 
     * 
     * Method called from the client once all markers have been detected.
     * It makes the arm go back to its home position and then the motion of the arm is ended.
     */
    bool motionCallback(assignment2::ArmInfo::Request &req, assignment2::ArmInfo::Response &res) {
    	// Stop the motion of the camera
        move_arm = false;
        // Get the request of the client
        req.done;
        ROS_INFO("\n\nAll markers was detected\n\nArm is going to home position!\n");
        // Make both camera and base joints go to their home position
        move_to_surv_pos(move_base_pub, move_cam_pub);
        // Send the response to the client
        res.success = 1;
        return true;
    }
};

/**
 * @brief Main function
 * 
 * @param argc 
 * @param argv 
 * @return 0
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "move_joint");
    MoveJoint node;
    ros::Rate loop_rate(50);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
