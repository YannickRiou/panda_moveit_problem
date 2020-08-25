
#include <geometric_shapes/shapes.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/shape_operations.h>

#include<string>

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene/planning_scene.h>

void spawnObject(moveit::planning_interface::PlanningSceneInterface& psi) {

	moveit_msgs::CollisionObject o;
	o.id= "object";
	o.header.frame_id= "panda_link0";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.65;
	o.primitive_poses[0].position.y = 0.18;
	o.primitive_poses[0].position.z = 0.86;
	o.primitive_poses[0].orientation.x =0.0;
	o.primitive_poses[0].orientation.y =0.0;
	o.primitive_poses[0].orientation.z =0.0;
	o.primitive_poses[0].orientation.w =1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
	o.primitives[0].dimensions.resize(3);
	o.primitives[0].dimensions[0]= 0.055;
	o.primitives[0].dimensions[1]= 0.055;
	o.primitives[0].dimensions[2]= 0.055*2;
	psi.applyCollisionObject(o);

	/*o.id= "obstacle";
	o.header.frame_id= "base_footprint";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.6;
	o.primitive_poses[0].position.y = -0.05;
	o.primitive_poses[0].position.z = 0.75+(0.30/2);
	o.primitive_poses[0].orientation.x =0.0;
	o.primitive_poses[0].orientation.y =0.0;
	o.primitive_poses[0].orientation.z =0.0;
	o.primitive_poses[0].orientation.w =1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
	o.primitives[0].dimensions.resize(3);
	o.primitives[0].dimensions[0]= 0.05;
	o.primitives[0].dimensions[1]= 0.05;
	o.primitives[0].dimensions[2]= 0.30;
	psi.applyCollisionObject(o);

	o.id= "obstacle2";
	o.header.frame_id= "base_footprint";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.6;
	o.primitive_poses[0].position.y = -0.35;
	o.primitive_poses[0].position.z = 0.75+(0.30/2);
	o.primitive_poses[0].orientation.x =0.0;
	o.primitive_poses[0].orientation.y =0.0;
	o.primitive_poses[0].orientation.z =0.0;
	o.primitive_poses[0].orientation.w =1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
	o.primitives[0].dimensions.resize(3);
	o.primitives[0].dimensions[0]= 0.05;
	o.primitives[0].dimensions[1]= 0.05;
	o.primitives[0].dimensions[2]= 0.30;
	psi.applyCollisionObject(o);
	
	o.id= "obstacle3";
	o.header.frame_id= "base_footprint";
	o.primitive_poses.resize(1);
	o.primitive_poses[0].position.x = 0.6;
	o.primitive_poses[0].position.y = -0.20;
	o.primitive_poses[0].position.z = 0.75+0.3;
	o.primitive_poses[0].orientation.x =0.0;
	o.primitive_poses[0].orientation.y =0.0;
	o.primitive_poses[0].orientation.z =0.0;
	o.primitive_poses[0].orientation.w =1.0;
	o.primitives.resize(1);
	o.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
	o.primitives[0].dimensions.resize(3);
	o.primitives[0].dimensions[0]= 0.05;
	o.primitives[0].dimensions[1]= 0.30;
	o.primitives[0].dimensions[2]= 0.05;
	psi.applyCollisionObject(o);

	*/
	moveit_msgs::CollisionObject box;
	shape_msgs::Mesh mesh;
  	shapes::ShapeMsg mesh_msg;
  	shapes::Mesh* m;

	box.id = "obstacle";
	box.header.frame_id= "panda_link0";
	std::string mesh_uri("package://panda_test/mesh/dt_box_panda.dae");
	m = shapes::createMeshFromResource(mesh_uri);
	shapes::constructMsgFromShape(m, mesh_msg);
	mesh = boost::get<shape_msgs::Mesh>(mesh_msg);
	// Add the mesh to the Collision object message
	box.meshes.push_back(mesh);
	geometry_msgs::Pose pose;
	pose.orientation.x = 0.0;
	pose.orientation.y = 0.0;
	pose.orientation.z = -0.707;
	pose.orientation.w = 0.707;	
	pose.position.x = 0.66;
	pose.position.y = -0.20;
	pose.position.z = 0.91;
	box.mesh_poses.push_back(pose);
	psi.applyCollisionObject(box);
	


	moveit_msgs::CollisionObject table;
	table.id= "tableLaas";
	table.header.frame_id= "panda_link0";
	table.primitive_poses.resize(1);
	table.primitive_poses[0].position.x = 0.95;
	table.primitive_poses[0].position.y = 0.0;
	table.primitive_poses[0].position.z = 0.75/2;
	table.primitive_poses[0].orientation.x =0.0;
	table.primitive_poses[0].orientation.y =0.0;
	table.primitive_poses[0].orientation.z =0.0;
	table.primitive_poses[0].orientation.w =1.0;
	table.primitives.resize(1);
	table.primitives[0].type= shape_msgs::SolidPrimitive::BOX;
	table.primitives[0].dimensions.resize(3);
	table.primitives[0].dimensions[0]= 0.85;
	table.primitives[0].dimensions[1]= 1.35;
	table.primitives[0].dimensions[2]= 0.75;
	psi.applyCollisionObject(table);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "panda_pb");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh("~");

	const std::string ARM_PLANNING_GROUP = "panda_arm";

	const robot_state::JointModelGroup* arm_joint_model_group;

	

    moveit::planning_interface::MoveGroupInterface arm_move_group(ARM_PLANNING_GROUP);

	arm_joint_model_group =
		arm_move_group.getCurrentState()->getJointModelGroup(ARM_PLANNING_GROUP);

	arm_move_group.setNamedTarget("start");
	arm_move_group.move();

	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	spawnObject(planning_scene_interface);

	std::cout << "waiting for any key + <enter>\n";
	char ch;
	std::cin >> ch;

	moveit_msgs::AttachedCollisionObject ao;
	ao.link_name = "panda_link8";
	ao.object.id= "object";

	arm_move_group.setNamedTarget("end");

	planning_scene_interface.applyAttachedCollisionObject(ao);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;


	std::cout << "waiting for any key + <enter>\n";
	std::cin >> ch;

	// Plan doesn't show the collision object moving with the gripper
	bool success = (arm_move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


	std::cout << "waiting for any key + <enter>\n";
	std::cin >> ch;
	
	arm_move_group.execute(my_plan);




	return 0;
}
