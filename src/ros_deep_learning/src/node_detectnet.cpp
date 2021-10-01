/*
 * Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "ros_compat.h"
#include "image_converter.h"

#include <jetson-inference/detectNet.h>

#include <unordered_map>

//yeoeun
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseStamped.h>

// vision_msgs::VisionInfo info_msg;

// globals
detectNet* net = NULL;
uint32_t overlay_flags = detectNet::OVERLAY_NONE;

imageConverter* input_cvt   = NULL;
imageConverter* overlay_cvt = NULL;

Publisher<vision_msgs::Detection2DArray> detection_pub = NULL;
Publisher<sensor_msgs::Image> overlay_pub = NULL;
Publisher<vision_msgs::VisionInfo> info_pub = NULL;
//yeoeun cmd_vel 토픽 다르게
Publisher<geometry_msgs::Twist> personInfo_pub = NULL;
//yeoeun 도서관 코드
// Publisher<std_msgs::Int32> person_pub = NULL;
// Publisher<geometry_msgs::PoseStamped> current_goal_pub = NULL;

vision_msgs::VisionInfo info_msg;

//yeoeun
// geometry_msgs::PoseStamped current_pose;
// geometry_msgs::PoseStamped goal_pose;

//yeoeun 토픽 바꿔서
geometry_msgs::Twist cmd_vel;
geometry_msgs::PoseStamped start_goal;
geometry_msgs::PoseStamped current_goal;

// triggered when a new subscriber connected
void info_callback()
{
	ROS_INFO("new subscriber connected to vision_info topic, sending VisionInfo msg");
	info_pub->publish(info_msg);
}


// publish overlay image
bool publish_overlay( detectNet::Detection* detections, int numDetections )
{
	// get the image dimensions
	const uint32_t width  = input_cvt->GetWidth();
	const uint32_t height = input_cvt->GetHeight();

	// assure correct image size
	if( !overlay_cvt->Resize(width, height, imageConverter::ROSOutputFormat) )
		return false;

	// generate the overlay
	if( !net->Overlay(input_cvt->ImageGPU(), overlay_cvt->ImageGPU(), width, height, 
				   imageConverter::InternalFormat, detections, numDetections, overlay_flags) )
	{
		return false;
	}

	// populate the message
	sensor_msgs::Image msg;

	if( !overlay_cvt->Convert(msg, imageConverter::ROSOutputFormat) )
		return false;

	// populate timestamp in header field
	msg.header.stamp = ROS_TIME_NOW();

	// publish the message	
	overlay_pub->publish(msg);
	ROS_DEBUG("publishing %ux%u overlay image", width, height);
}


// input image subscriber callback
void img_callback( const sensor_msgs::ImageConstPtr input )
{
	// 시작점 정의
	// simulation
	start_goal.pose.position.x = -2.0;
	start_goal.pose.position.y = -0.4;	
	start_goal.pose.position.z = 0.0;	
	start_goal.pose.orientation.x = 0.0;	
	start_goal.pose.orientation.y = 0.0;	
	start_goal.pose.orientation.z = 0.0;	
	start_goal.pose.orientation.w = 0.9;	

	// map
	// start_goal.pose.position.x = 1.525;
	// start_goal.pose.position.y = 0.27;
	// start_goal.pose.position.z = 0.0;
	// start_goal.pose.orientation.x = 0.0;
	// start_goal.pose.orientation.y = 0.0;
	// start_goal.pose.orientation.z = -0.7;
	// start_goal.pose.orientation.w = 0.6;

	// convert the image to reside on GPU
	if( !input_cvt || !input_cvt->Convert(input) )
	{
		ROS_INFO("failed to convert %ux%u %s image", input->width, input->height, input->encoding.c_str());
		return;	
	}

	// classify the image
	detectNet::Detection* detections = NULL;

	const int numDetections = net->Detect(input_cvt->ImageGPU(), input_cvt->GetWidth(), input_cvt->GetHeight(), &detections, detectNet::OVERLAY_NONE);

	//yeoeun 도서관 코드
	// std_msgs::Int32 num;
	// num.data = 0;

	//yeoeun 토픽다르게
	int num = 0;

	// verify success	
	if( numDetections < 0 )
	{
		ROS_ERROR("failed to run object detection on %ux%u image", input->width, input->height);
		return;
	}

	// if objects were detected, send out message
	if( numDetections >= 0 )
	{
		ROS_INFO("detected %i objects in %ux%u image", numDetections, input->width, input->height);

		// create a detection for each bounding box
		vision_msgs::Detection2DArray msg;

		for( int n=0; n < numDetections; n++ )
		{
			detectNet::Detection* det = detections + n;
			// class = ClassID;
			ROS_INFO("object %i class #%u (%s)  confidence=%f", n, det->ClassID, net->GetClassDesc(det->ClassID), det->Confidence);
			ROS_INFO("object %i bounding box (%f, %f)  (%f, %f)  w=%f  h=%f", n, det->Left, det->Top, det->Right, det->Bottom, det->Width(), det->Height()); 
			
			//yeoeun
			if((det->ClassID) == 1){
				num = num + 1;
			}

			vision_msgs::Detection2D detMsg;

			detMsg.bbox.size_x = det->Width();
			detMsg.bbox.size_y = det->Height();
			
			float cx, cy;
			det->Center(&cx, &cy);

			detMsg.bbox.center.x = cx;
			detMsg.bbox.center.y = cy;

			detMsg.bbox.center.theta = 0.0f;		// TODO optionally output object image

			// create classification hypothesis
			vision_msgs::ObjectHypothesisWithPose hyp;
			
		#if ROS_DISTRO >= ROS_GALACTIC
			hyp.hypothesis.class_id = det->ClassID;
			hyp.hypothesis.score = det->Confidence;
		#else
			hyp.id = det->ClassID;
			hyp.score = det->Confidence;
		#endif
			detMsg.results.push_back(hyp);
			msg.detections.push_back(detMsg);
		}
		// ROS_INFO("person %d", num.data);
		//yeoeun 도서관 코드
		// person_pub->publish(num);
		if(start_goal.pose.position.x != current_goal.pose.position.x){
			if(num == 0){
				geometry_msgs::Twist cmd_vel0;
				cmd_vel0.linear.x = 0.0;
      			cmd_vel0.linear.y = 0.0;
				cmd_vel0.linear.z = 0.0;
      			cmd_vel0.angular.x = 0.0;
				cmd_vel0.angular.y = 0.0;
				cmd_vel0.angular.z = 0.0;
				personInfo_pub -> publish(cmd_vel0);
			}
			if(num>0){
				personInfo_pub -> publish(cmd_vel);
			}
		}
		else if(start_goal.pose.position.x == current_goal.pose.position.x){
			personInfo_pub -> publish(cmd_vel);
		}

		// populate timestamp in header field
		msg.header.stamp = ROS_TIME_NOW();

		// publish the detection message
		detection_pub->publish(msg);
	}

	// generate the overlay (if there are subscribers)
	if( ROS_NUM_SUBSCRIBERS(overlay_pub) > 0 )
		publish_overlay(detections, numDetections);
}

void cmd_vel_callback(geometry_msgs::Twist cmd_vel1){
	cmd_vel = cmd_vel1;
	ROS_INFO("cmd_vel %f %f", cmd_vel.linear.x, cmd_vel.linear.y);
	// personInfo_pub -> publish(cmd_vel);
}

void goal_callback(geometry_msgs::PoseStamped goal){
	current_goal.pose.position.x = goal.pose.position.x;
	current_goal.pose.position.y = goal.pose.position.y;
}

// node main loop
int main(int argc, char **argv)
{
	/*
	 * create node instance
	 */
	ROS_CREATE_NODE("detectnet");

	/*
	 * retrieve parameters
	 */	
	std::string model_name  = "ssd-mobilenet-v2";
	std::string model_path;
	std::string prototxt_path;
	std::string class_labels_path;
	
	std::string input_blob  = DETECTNET_DEFAULT_INPUT;
	std::string output_cvg  = DETECTNET_DEFAULT_COVERAGE;
	std::string output_bbox = DETECTNET_DEFAULT_BBOX;
	std::string overlay_str = "box,labels,conf";

	float mean_pixel = 0.0f;
	float threshold  = DETECTNET_DEFAULT_THRESHOLD;

	ROS_DECLARE_PARAMETER("model_name", model_name);
	ROS_DECLARE_PARAMETER("model_path", model_path);
	ROS_DECLARE_PARAMETER("prototxt_path", prototxt_path);
	ROS_DECLARE_PARAMETER("class_labels_path", class_labels_path);
	ROS_DECLARE_PARAMETER("input_blob", input_blob);
	ROS_DECLARE_PARAMETER("output_cvg", output_cvg);
	ROS_DECLARE_PARAMETER("output_bbox", output_bbox);
	ROS_DECLARE_PARAMETER("overlay_flags", overlay_str);
	ROS_DECLARE_PARAMETER("mean_pixel_value", mean_pixel);
	ROS_DECLARE_PARAMETER("threshold", threshold);


	/*
	 * retrieve parameters
	 */
	ROS_GET_PARAMETER("model_name", model_name);
	ROS_GET_PARAMETER("model_path", model_path);
	ROS_GET_PARAMETER("prototxt_path", prototxt_path);
	ROS_GET_PARAMETER("class_labels_path", class_labels_path);
	ROS_GET_PARAMETER("input_blob", input_blob);
	ROS_GET_PARAMETER("output_cvg", output_cvg);
	ROS_GET_PARAMETER("output_bbox", output_bbox);
	ROS_GET_PARAMETER("overlay_flags", overlay_str);
	ROS_GET_PARAMETER("mean_pixel_value", mean_pixel);
	ROS_GET_PARAMETER("threshold", threshold);

	overlay_flags = detectNet::OverlayFlagsFromStr(overlay_str.c_str());


	/*
	 * load object detection network
	 */
	if( model_path.size() > 0 )
	{
		// create network using custom model paths
		net = detectNet::Create(prototxt_path.c_str(), model_path.c_str(), 
						    mean_pixel, class_labels_path.c_str(), threshold, 
						    input_blob.c_str(), output_cvg.c_str(), output_bbox.c_str());
	}
	else
	{
		// determine which built-in model was requested
		detectNet::NetworkType model = detectNet::NetworkTypeFromStr(model_name.c_str());

		if( model == detectNet::CUSTOM )
		{
			ROS_ERROR("invalid built-in pretrained model name '%s', defaulting to pednet", model_name.c_str());
			model = detectNet::SSD_MOBILENET_V2;
		}

		// create network using the built-in model
		net = detectNet::Create(model, threshold);
	}

	if( !net )
	{
		ROS_ERROR("failed to load detectNet model");
		return 0;
	}


	/*
	 * create the class labels parameter vector
	 */
	std::hash<std::string> model_hasher;  // hash the model path to avoid collisions on the param server
	std::string model_hash_str = std::string(net->GetModelPath()) + std::string(net->GetClassPath());
	const size_t model_hash = model_hasher(model_hash_str);
	
	ROS_INFO("model hash => %zu", model_hash);
	ROS_INFO("hash string => %s", model_hash_str.c_str());

	// obtain the list of class descriptions
	std::vector<std::string> class_descriptions;
	const uint32_t num_classes = net->GetNumClasses();

	for( uint32_t n=0; n < num_classes; n++ )
		class_descriptions.push_back(net->GetClassDesc(n));

	// create the key on the param server
	std::string class_key = std::string("class_labels_") + std::to_string(model_hash);

	ROS_DECLARE_PARAMETER(class_key, class_descriptions);
	ROS_SET_PARAMETER(class_key, class_descriptions);
		
	// populate the vision info msg
	std::string node_namespace = ROS_GET_NAMESPACE();
	ROS_INFO("node namespace => %s", node_namespace.c_str());

	info_msg.database_location = node_namespace + std::string("/") + class_key;
	info_msg.database_version  = 0;
	info_msg.method 		  = net->GetModelPath();
	
	ROS_INFO("class labels => %s", info_msg.database_location.c_str());


	/*
	 * create image converter objects
	 */
	input_cvt = new imageConverter();
	overlay_cvt = new imageConverter();

	if( !input_cvt || !overlay_cvt )
	{
		ROS_ERROR("failed to create imageConverter objects");
		return 0;
	}


	/*
	 * advertise publisher topics
	 */
	ROS_CREATE_PUBLISHER(vision_msgs::Detection2DArray, "detections", 25, detection_pub);
	ROS_CREATE_PUBLISHER(sensor_msgs::Image, "overlay", 2, overlay_pub);
	
	ROS_CREATE_PUBLISHER_STATUS(vision_msgs::VisionInfo, "vision_info", 1, info_callback, info_pub);

	//yeoeun 도서관 코드
	// ROS_CREATE_PUBLISHER(std_msgs::Int32, "person_info", 1, person_pub);
	//yeoeun 토픽 다르게
	ROS_CREATE_PUBLISHER(geometry_msgs::Twist, "cmd_vel", 1,  personInfo_pub);
	auto vel_sub = ROS_CREATE_SUBSCRIBER(geometry_msgs::Twist, "cmd_vel1", 1,  cmd_vel_callback);
	auto pose_sub = ROS_CREATE_SUBSCRIBER(geometry_msgs::PoseStamped, "posePoint", 1,  goal_callback);
	

	// ROS_CREATE_PUBLISHER(geometry_msgs::PoseStamped, "currentGoal", 1, current_goal_pub);

	/*
	 * subscribe to image topic
	 */

	auto img_sub = ROS_CREATE_SUBSCRIBER(sensor_msgs::Image, "image_in", 5, img_callback);

	//yeoeun
	// auto currnet_sub = ROS_CREATE_SUBSCRIBER(geometry_msgs::PoseStamped, "current", 5, current_callback);

	// auto current_pose_sub_ = ROS_CREATE_SUBSCRIBER(geometry_msgs::PoseStamped, "current", 1, current_callback);

	// auto goal_sub_ = ROS_CREATE_SUBSCRIBER(geometry_msgs::PoseStamped, "posePoint", 1, goal_callback);


	/*
	 * wait for messages
	 */
	ROS_INFO("detectnet node initialized, waiting for messages");
	ROS_SPIN();


	/*
	 * free resources
	 */
	delete net;
	delete input_cvt;
	delete overlay_cvt;

	return 0;
}

