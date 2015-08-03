/*!
 * \CarlActionExecutor.cpp
 * \brief Sequential execution of general actions on the CARL robot.
 *
 * CarlActionExecutor creates and maintains a list of general actions, which can be executed one at a time or
 * in its entirety.  The list includes a set of services for adding and removing actions, and the action themselves
 * include general failure cases, reported if execution has to stop.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date July 27, 2015
 */

#include <carl_action_executor/CarlActionExecutor.h>

using namespace std;

CarlActionExecutor::CarlActionExecutor() :
  moveBaseClient("move_base"),
  moveCarlClient("move_carl"),
  armActionClient("carl_moveit_wrapper/common_actions/arm_action"),
  pickupClient("carl_moveit_wrapper/common_actions/pickup"),
  storeClient("carl_moveit_wrapper/common_actions/store"),
  executeServer(node, "carl_action_executor/execute", boost::bind(&CarlActionExecutor::executeAction, this, _1), false)
{
  recognizedObjectsCounter = 0;

  recognizedObjectsSubscriber = node.subscribe("/object_recognition_listener/recognized_objects", 1, &CarlActionExecutor::recognizedObjectsCallback, this);

  segmentClient = node.serviceClient<std_srvs::Empty>("/rail_segmentation/segment");
  lookAtFrameClient = node.serviceClient<carl_dynamixel::LookAtFrame>("/asus_controller/look_at_frame");

  add_action_server = node.advertiseService("carl_action_executor/add_action", &CarlActionExecutor::addAction, this);
  clear_action_list_server = node.advertiseService("carl_action_executor/clear_action_list", &CarlActionExecutor::clearActionList, this);
  get_action_list_server = node.advertiseService("carl_action_executor/get_action_list", &CarlActionExecutor::getActionList, this);
  insert_action_server = node.advertiseService("carl_action_executor/insert_action", &CarlActionExecutor::insertAction, this);
  remove_action_server = node.advertiseService("carl_action_executor/remove_action", &CarlActionExecutor::removeAction, this);

  executeServer.start();
}

void CarlActionExecutor::executeAction(const carl_action_executor::ExecuteGoalConstPtr &goal)
{
  carl_action_executor::ExecuteResult result;
  carl_action_executor::ExecuteFeedback feedback;
  result.success = false;
  result.error = carl_action_executor::GeneralAction::SUCCESS;

  if (goal->execute_all)
  {
    while (!action_list.empty())
    {
      //execute actions until one fails or no actions remain
      feedback.current_action = action_list.at(0).action_type;
      feedback.actions_remaining = action_list.size() - 1;
      executeServer.publishFeedback(feedback);

      result.error = executeSingleAction(action_list.at(0));
      //remove the action if it was successfully executed
      if (result.error == carl_action_executor::GeneralAction::SUCCESS)
      {
        action_list.erase(action_list.begin());
      }
      //set preempted if the action server requests preempt
      else if (result.error == carl_action_executor::GeneralAction::PREEMPTED)
      {
        result.failed_action = action_list.at(0);
        executeServer.setPreempted(result);
        return;
      }
      //otherwise stop execution and report failure
      else
      {
        result.failed_action = action_list.at(0);
        executeServer.setSucceeded(result);
        return;
      }
    }
  }
  else
  {
    //execute only the current action
    if (!action_list.empty())
    {
      feedback.current_action = action_list.at(0).action_type;
      feedback.actions_remaining = action_list.size() - 1;
      executeServer.publishFeedback(feedback);

      result.error = executeSingleAction(action_list.at(0));
      //remove the action if it was successfully executed
      if (result.error == carl_action_executor::GeneralAction::SUCCESS)
        action_list.erase(action_list.begin());
      else if (result.error == carl_action_executor::GeneralAction::PREEMPTED)
      {
        result.failed_action = action_list.at(0);
        executeServer.setPreempted(result);
      }
      else
      {
        result.failed_action = action_list.at(0);
        executeServer.setSucceeded(result);
      }
    }
  }

  if (result.error == carl_action_executor::GeneralAction::SUCCESS)
    result.success = true;

  executeServer.setSucceeded(result);
}

unsigned char CarlActionExecutor::executeSingleAction(const carl_action_executor::GeneralAction &action)
{
  switch (action.action_type)
  {
    case carl_action_executor::GeneralAction::NAVIGATE:
      if (!action.nav_pose.header.frame_id.empty()) //pose navigation
      {
        move_base_msgs::MoveBaseGoal moveBaseGoal;
        moveBaseGoal.target_pose = action.nav_pose;
        moveBaseClient.sendGoal(moveBaseGoal);
        while (!moveBaseClient.getState().isDone())
        {
          if (executeServer.isPreemptRequested())
          {
            ROS_INFO("Canceling nav goal...");
            moveBaseClient.cancelAllGoals();
            return carl_action_executor::GeneralAction::PREEMPTED;
          }
        }
        if (moveBaseClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          return carl_action_executor::GeneralAction::SUCCESS;
        }
        else
        {
          return carl_action_executor::GeneralAction::NAVIGATION_EXECUTION_FAILURE;
        }
      }
      else  //rail location server navigation
      {
        carl_navigation::MoveCarlGoal moveCarlGoal;
        moveCarlGoal.location = action.nav_location;
        moveCarlClient.sendGoal(moveCarlGoal);
        while (!moveCarlClient.getState().isDone())
        {
          if (executeServer.isPreemptRequested())
          {
            ROS_INFO("Canceling nav goal...");
            moveCarlClient.cancelAllGoals();
            return carl_action_executor::GeneralAction::PREEMPTED;
          }
        }
        if (moveCarlClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
          return carl_action_executor::GeneralAction::SUCCESS;
        }
        else
        {
          return carl_action_executor::GeneralAction::NAVIGATION_EXECUTION_FAILURE;
        }
      }


    case carl_action_executor::GeneralAction::READY_ARM:
    {
      carl_moveit::ArmGoal readyArmGoal;
      readyArmGoal.action = carl_moveit::ArmGoal::READY;
      armActionClient.sendGoal(readyArmGoal);
      while (!armActionClient.getState().isDone()) {
        if (executeServer.isPreemptRequested()) {
          ROS_INFO("Canceling arm ready goal...");
          armActionClient.cancelAllGoals();
          return carl_action_executor::GeneralAction::PREEMPTED;
        }
      }

      if (armActionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED
          && armActionClient.getResult()->success) {
        return carl_action_executor::GeneralAction::SUCCESS;
      }
      else {
        return carl_action_executor::GeneralAction::MANIPULATION_PLANNING_FAILURE;
      }
    }


    case carl_action_executor::GeneralAction::RETRACT_ARM:
    {
      carl_moveit::ArmGoal retractArmGoal;
      retractArmGoal.action = carl_moveit::ArmGoal::RETRACT;
      armActionClient.sendGoal(retractArmGoal);
      while (!armActionClient.getState().isDone()) {
        if (executeServer.isPreemptRequested()) {
          ROS_INFO("Canceling arm retract goal...");
          armActionClient.cancelAllGoals();
          return carl_action_executor::GeneralAction::PREEMPTED;
        }
      }
      if (armActionClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED
          && armActionClient.getResult()->success) {
        return carl_action_executor::GeneralAction::SUCCESS;
      }
      else {
        return carl_action_executor::GeneralAction::MANIPULATION_PLANNING_FAILURE;
      }
    }


    case carl_action_executor::GeneralAction::PICKUP:
    {
      //if no object name was specified, grasp at the given manipulation pose instead
      if (action.object_name.empty())
      {
        carl_moveit::PickupGoal pickupGoal;
        pickupGoal.lift = true;
        pickupGoal.verify = false;  //note: for now pickups are unverified always, because CARL can't do this.  If possible, change this to always verify pickups.
        pickupGoal.pose = action.manipulation_pose;
        pickupClient.sendGoal(pickupGoal);

        while (!pickupClient.getState().isDone())
        {
          if (executeServer.isPreemptRequested())
          {
            ROS_INFO("Canceling pickup goal...");
            pickupClient.cancelAllGoals();
            return carl_action_executor::GeneralAction::PREEMPTED;
          }
        }

        if (pickupClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED || !pickupClient.getResult()->success)
        {
          ROS_INFO("Pickup failed.");
          return carl_action_executor::GeneralAction::MANIPULATION_PLANNING_FAILURE;
        }
        else
        {
          ROS_INFO("Pickup succeeded!");
          return carl_action_executor::GeneralAction::SUCCESS;
        }
      }
      //otherwise grab an object with the given name
      else
      {
        string objectName = boost::to_upper_copy(action.object_name);
        for (unsigned int i = 0; i < recognizedObjects.objects.size(); i++)
        {
          if (recognizedObjects.objects[i].name == objectName)
          {
            ROS_INFO("Found object, attempting pickup...");
            carl_moveit::PickupGoal pickupGoal;
            pickupGoal.lift = true;
            pickupGoal.verify = false;  //note: for now pickups are unverified always, because CARL can't do this.  If possible, change this to always verify pickups.

            for (unsigned int j = 0; j < recognizedObjects.objects[i].grasps.size(); j++)
            {
              ROS_INFO("Attempting pickup with grasp %d", j);
              pickupGoal.pose = recognizedObjects.objects[i].grasps[j].grasp_pose;
              pickupClient.sendGoal(pickupGoal);
              while (!pickupClient.getState().isDone())
              {
                if (executeServer.isPreemptRequested())
                {
                  ROS_INFO("Canceling pickup goal...");
                  pickupClient.cancelAllGoals();
                  return carl_action_executor::GeneralAction::PREEMPTED;
                }
              }

              if (pickupClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED || !pickupClient.getResult()->success)
              {
                ROS_INFO("Pickup failed, moving on to a new grasp...");
              }
              else
              {
                ROS_INFO("Pickup succeeded!");
                return carl_action_executor::GeneralAction::SUCCESS;
              }
            }

            return carl_action_executor::GeneralAction::MANIPULATION_PLANNING_FAILURE;
          }
        }
      }



      return carl_action_executor::GeneralAction::OBJECT_NOT_FOUND;
    }

    case carl_action_executor::GeneralAction::STORE:
    {
      carl_moveit::StoreGoal storeGoal;
      storeGoal.store_pose = action.manipulation_pose;

      storeClient.sendGoal(storeGoal);
      while (!storeClient.getState().isDone())
      {
        if (executeServer.isPreemptRequested())
        {
          ROS_INFO("Canceling store goal...");
          storeClient.cancelAllGoals();
          return carl_action_executor::GeneralAction::PREEMPTED;
        }
      }

      if (storeClient.getState() != actionlib::SimpleClientGoalState::SUCCEEDED || !storeClient.getResult()->success)
      {
        ROS_INFO("Store failed.");
        return carl_action_executor::GeneralAction::MANIPULATION_PLANNING_FAILURE;
      }
      else
      {
        ROS_INFO("Store succeeded!");
        return carl_action_executor::GeneralAction::SUCCESS;
      }
    }


    case carl_action_executor::GeneralAction::SEGMENT:
    {
      //look at surface, if a surface is specified
      if (!action.surface_frame.empty())
      {
        carl_dynamixel::LookAtFrame lookSrv;
        lookSrv.request.frame = action.surface_frame;
        if (!lookAtFrameClient.call(lookSrv))
        {
          ROS_INFO("Couldn't call look at frame client.");
          return carl_action_executor::GeneralAction::SENSING_FAILURE;
        }
      }

      std_srvs::Empty segmentSrv;
      if (!segmentClient.call(segmentSrv))
      {
        ROS_INFO("Couldn't call segmentation service.");
        return carl_action_executor::GeneralAction::SENSING_FAILURE;
      }

      return carl_action_executor::GeneralAction::SUCCESS;
    }


    case carl_action_executor::GeneralAction::SEGMENT_AND_RECOGNIZE:
    {
      //look at surface, if a surface is specified
      if (!action.surface_frame.empty())
      {
        carl_dynamixel::LookAtFrame lookSrv;
        lookSrv.request.frame = action.surface_frame;
        if (!lookAtFrameClient.call(lookSrv))
        {
          ROS_INFO("Couldn't call look at frame client.");
          return carl_action_executor::GeneralAction::SENSING_FAILURE;
        }
      }

      std_srvs::Empty segmentSrv;
      recognizedObjectsCounter = 0;
      if (!segmentClient.call(segmentSrv))
      {
        ROS_INFO("Couldn't call segmentation service.");
        return carl_action_executor::GeneralAction::SENSING_FAILURE;
      }

      //spin and wait
      bool finished = false;
      while (!finished)
      {
        if (executeServer.isPreemptRequested())
        {
          ROS_INFO("Canceling action execution during recognition...");
          return carl_action_executor::GeneralAction::PREEMPTED;
        }

        {
          boost::mutex::scoped_lock lock(recognizedObjectsMutex);
          finished = recognizedObjectsCounter >= 2;
        }
      }

      return carl_action_executor::GeneralAction::SUCCESS;
    }
  }

  return carl_action_executor::GeneralAction::SUCCESS;
}

void CarlActionExecutor::recognizedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &objects)
{
  boost::mutex::scoped_lock lock(recognizedObjectsMutex);

  recognizedObjects = objects;
  recognizedObjectsCounter++;
}


/*********************************************************************
 ************************** List Management **************************
 *********************************************************************/

bool CarlActionExecutor::addAction(carl_action_executor::AddAction::Request &req, carl_action_executor::AddAction::Response &res)
{
  action_list.push_back(req.action);
  return true;
}

bool CarlActionExecutor::clearActionList(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
  action_list.clear();
  return true;
}

bool CarlActionExecutor::getActionList(carl_action_executor::GetActionList::Request &req, carl_action_executor::GetActionList::Response &res)
{
  res.action_list = action_list;
  return true;
}

bool CarlActionExecutor::insertAction(carl_action_executor::InsertAction::Request &req, carl_action_executor::InsertAction::Response &res)
{
  if (action_list.size() < req.index)
  {
    action_list.push_back(req.action);
  }
  else
  {
    action_list.insert(action_list.begin() + req.index, req.action);
  }
  return true;
}

bool CarlActionExecutor::removeAction(carl_action_executor::RemoveAction::Request &req, carl_action_executor::RemoveAction::Response &res)
{
  if (req.index < action_list.size())
  {
    action_list.erase(action_list.begin() + req.index);
  }
  return true;
}

int main(int argc, char **argv)
{
  // initialize ROS and the node
  ros::init(argc, argv, "carl_action_executor");

  // initialize the joystick controller
  CarlActionExecutor cae;

  ros::spin();

  return EXIT_SUCCESS;
}
