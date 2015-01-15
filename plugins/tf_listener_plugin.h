#ifndef ED_TF_LISTENER_PLUGIN_H_
#define ED_TF_LISTENER_PLUGIN_H_

#include <ed/plugin.h>
#include <ed/uuid.h>

#include <tf2_msgs/TFMessage.h>
#include <ros/callback_queue.h>

struct TFLink
{
    std::string tf_parent_id;
    std::string tf_child_id;
    ed::UUID ed_parent_id;
    ed::UUID ed_child_id;
};

class TFListenerPlugin : public ed::Plugin
{

public:

    TFListenerPlugin();

    virtual ~TFListenerPlugin();

    void configure(tue::Configuration config);

    void process(const ed::WorldModel& world, ed::UpdateRequest& req);

private:

    std::map<std::string, TFLink> links_;

    // TF listening
//    ros::CallbackQueue cb_queue_;

//    void tfCallback(const tf2_msgs::TFMessage::ConstPtr& tf_msg);

};

#endif
