/*!
 * \CarlActionExecutor.h
 * \brief Sequential execution of general actions on the CARL robot.
 *
 * CarlActionExecutor creates and maintains a list of general actions, which can be executed one at a time or
 * in its entirety.  The list includes a set of services for adding and removing actions, and the action themselves
 * include general failure cases, reported if execution has to stop.
 *
 * \author David Kent, WPI - davidkent@wpi.edu
 * \date July 27, 2015
 */

#ifndef CARL_ACTION_EXECUTOR_H_
#define CARL_ACTION_EXECUTOR_H_

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread/mutex.hpp>
#include <carl_action_executor/AddAction.h>
#include <carl_action_executor/ExecuteAction.h>
#include <carl_action_executor/GeneralAction.h>
#include <carl_action_executor/GetActionList.h>
#include <carl_action_executor/InsertAction.h>
#include <carl_action_executor/RemoveAction.h>
#include <carl_dynamixel/LookAtFrame.h>
#include <carl_navigation/MoveCarlAction.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <rail_manipulation_msgs/ArmAction.h>
#include <rail_manipulation_msgs/PickupAction.h>
#include <rail_manipulation_msgs/SegmentedObjectList.h>
#include <rail_manipulation_msgs/StoreAction.h>
#include <std_srvs/Empty.h>

class CarlActionExecutor
{
public:
  /*!
   * Creates a carl_action_executor object that can be used to define and execute an action list.
   */
  CarlActionExecutor();

private:
  ros::NodeHandle node;

  ros::Subscriber recognizedObjectsSubscriber;

  ros::ServiceClient segmentClient;
  ros::ServiceClient lookAtFrameClient;

  ros::ServiceServer add_action_server;
  ros::ServiceServer clear_action_list_server;
  ros::ServiceServer get_action_list_server;
  ros::ServiceServer insert_action_server;
  ros::ServiceServer remove_action_server;

  actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> moveBaseClient;
  actionlib::SimpleActionClient<carl_navigation::MoveCarlAction> moveCarlClient;
  actionlib::SimpleActionClient<rail_manipulation_msgs::ArmAction> armActionClient;
  actionlib::SimpleActionClient<rail_manipulation_msgs::PickupAction> pickupClient;
  actionlib::SimpleActionClient<rail_manipulation_msgs::StoreAction> storeClient;

  actionlib::SimpleActionServer<carl_action_executor::ExecuteAction> executeServer;

  int recognizedObjectsCounter;

  std::vector<carl_action_executor::GeneralAction> action_list;
  rail_manipulation_msgs::SegmentedObjectList recognizedObjects;

  boost::mutex recognizedObjectsMutex;

  void executeAction(const carl_action_executor::ExecuteGoalConstPtr &goal);

  unsigned char executeSingleAction(const carl_action_executor::GeneralAction &action);

  void recognizedObjectsCallback(const rail_manipulation_msgs::SegmentedObjectList &objects);

  /**
  * \brief Callback for adding an action to the list
  * @param req service request
  * @param res service response
  * @return true on success
  */
  bool addAction(carl_action_executor::AddAction::Request &req, carl_action_executor::AddAction::Response &res);

  /**
  * \brief Callback for clearing the action list
  * @param req service request
  * @param res service response
  * @return true on success
  */
  bool clearActionList(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

  /**
  * \brief Callback for getting the list of actions
  * @param req service request
  * @param res service response
  * @return true on success
  */
  bool getActionList(carl_action_executor::GetActionList::Request &req, carl_action_executor::GetActionList::Response &res);

  /**
  * \brief Callback for inserting an action to the list
  * @param req service request
  * @param res service response
  * @return true on success
  */
  bool insertAction(carl_action_executor::InsertAction::Request &req, carl_action_executor::InsertAction::Response &res);

  /**
  * \brief Callback for removing an action from the list
  * @param req service request
  * @param res service response
  * @return true on success
  */
  bool removeAction(carl_action_executor::RemoveAction::Request &req, carl_action_executor::RemoveAction::Response &res);
};

int main(int argc, char **argv);

#endif
