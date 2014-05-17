/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014 Max-Planck-Institute for Intelligent Systems,
 *                     University of Southern California,
 *                     Karlsruhe Institute of Technology
 *    Jan Issac (jan.issac@gmail.com)
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * @date 04/15/2014
 * @author Jan Issac (jan.issac@gmail.com)
 * Max-Planck-Institute for Intelligent Systems, University of Southern California (USC),
 *   Karlsruhe Institute of Technology (KIT)
 */

#ifndef ONI_VICON_COMMON_ACTION_MACROS_HPP
#define ONI_VICON_COMMON_ACTION_MACROS_HPP

#include <actionlib/client/simple_action_client.h>

/**
 * ROS actionlib helper macros. In case of using many actions, the implementation will quickly
 * become cumbersome and the code becomes cluttered.
 *
 * The following macros implement most of the requirements of an action in a client node.
 *
 * Usage:
 *
 * Assume we have a defined action Record.action which is supposed to trigger a camera capturing.
 * Integrating this action requires the following steps:
 *
 * Declaration my_client.hpp:
 *
 *   class my_client
 *   {
 *     ...
 *
 *     ACTION_IMPLEMENT(Record); // thats it for the declaration part!
 *
 *     ...
 *   };
 *
 * Implementation my_client.cpp
 *
 *   my_client::my_client():  ACTION(Record)("action_server_name"){ }
 *
 *   ACTION_ON_ACTIVE(my_client, record_action_namespace, Record) { ... }
 *   ACTION_ON_FEEDBACK(my_client, record_action_namespace, Record) { ... }
 *   ACTION_ON_DONE(my_client, record_action_namespace, Record) { ... }
 *
 *
 * An action is started by first setting the goal followed by the sending the goal
 *
 *  ACTION_GOAL(Record).some_attribute = ...;
 *  ACTION_SEND_GOAL(my_client, record_action_namespace, Record);
 *
 * Done.
 *
 *
 *
 * What the macros do:
 * The following is equivalent to the code presendted above
 *
 * Declaration my_client.hpp:
 *   class my_client
 *   {
 *     ...
 *   public:
 *      typedef  actionlib::SimpleActionClient<record_action_namespace::RecordAction>  RecordClient;
 *   private:
 *       private:
 *       void RecordActiveCB();
 *       void RecordDoneCB(const actionlib::SimpleClientGoalState state,
 *                         const record_action_namespace::RecordResultConstPtr result);
 *       void RecordFeedbackCB(record_action_namespace::RecordFeedbackConstPtr feedback);
 *       RecordClient Record_ac_;
 *       record_action_namespace::RecordGoal Record_goal_;
 *   }
 *
 * Implementation my_client.cpp
 *
 *   my_client::my_client(): Record_ac_("action_server_name") { ... }
 *
 *   void my_client::RecordActiveCB() { ... }
 *   void my_client::RecordDoneCB(const actionlib::SimpleClientGoalState state,
 *                              const record_action_namespace::RecordResultConstPtr result) { ... }
 *   void my_client::RecordFeedbackCB(record_action_namespace::RecordFeedbackConstPtr feedback) { ... }
 *
 * Starting an action:
 *
 *  Record_goal_.some_attribute = ...;
 *  Record_ac_.sendGoal(Record_goal_,
 *       boost::bind(&my_client::RecordDoneCB, this, _1, _2),
 *       boost::bind(&my_client::RecordActiveCB, this),
 *       boost::bind(&my_client::RecordFeedbackCB, this, _1))
 */

#define ACTION(action_name) action_name##_ac_
#define ACTION_GOAL(action_name) action_name##_goal_
#define ACTION_STATE(action_name) state
#define ACTION_RESULT(action_name) result
#define ACTION_FEEDBACK(action_name) feedback

#define ACTION_IMPLEMENT_CLIENT(action_namespace, action_name) \
    public: \
        typedef  actionlib::SimpleActionClient<action_namespace::action_name##Action> \
            action_name##Client;\
    private:\
        void action_name ## ActiveCB();\
        void action_name ## DoneCB(const actionlib::SimpleClientGoalState state,\
                                   const action_namespace::action_name##ResultConstPtr result);\
        void action_name ## FeedbackCB(action_namespace::action_name##FeedbackConstPtr feedback);\
        action_name##Client action_name##_ac_;\
        action_namespace::action_name##Goal action_name##_goal_;

#define ACTION_INIT(action_namespace, action_name)\
    action_name##_ac_(action_namespace::action_name##Goal::ACTION_NAME, true)

#define ACTION_ON_ACTIVE(current_class_name, action_namespace, action_name)\
    void current_class_name::action_name ## ActiveCB()

#define ACTION_ON_FEEDBACK(current_class_name, action_namespace, action_name)\
    void current_class_name::action_name ## FeedbackCB(\
        action_namespace::action_name##FeedbackConstPtr feedback)

#define ACTION_ON_DONE(current_class_name, action_namespace, action_name)\
    void current_class_name::action_name ## DoneCB(\
        const actionlib::SimpleClientGoalState state,\
        const action_namespace::action_name##ResultConstPtr result)

#define ACTION_SEND_GOAL(current_class_name, action_namespace, action_name) \
    action_name##_ac_.sendGoal(action_name##_goal_, \
        boost::bind(&current_class_name::action_name##DoneCB, this, _1, _2),\
        boost::bind(&current_class_name::action_name##ActiveCB, this),\
        boost::bind(&current_class_name::action_name##FeedbackCB, this, _1))

#define ACTION_SHUTDOWN(action_name, is_active)\
    if (action_name##_ac_.isServerConnected()) \
    { \
        action_name##_ac_.cancelAllGoals(); \
        if (is_active) \
        { \
            action_name##_ac_.waitForResult(ros::Duration(0.5)); \
        } \
    } \
    else if (is_active) \
    { \
        action_name##_ac_.stopTrackingGoal(); \
    }

#endif
