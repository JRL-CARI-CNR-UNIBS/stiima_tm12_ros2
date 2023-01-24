#include "stiima_tm12_hw/tm_ros2_svr.h"
#include<iostream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

TmSvrRos2::TmSvrRos2(TmDriver &iface, bool stick_play)
    : Node("TmSvrRos2")
    , svr_(iface.svr)
    , state_(iface.state)
    , sct_(iface.sct)
    , iface_(iface)
{
    jns_.clear();
    jns_.push_back("joint_1");
    jns_.push_back("joint_2");
    jns_.push_back("joint_3");
    jns_.push_back("joint_4");
    jns_.push_back("joint_5");
    jns_.push_back("joint_6");


    pm_.svr_pub = this->create_publisher<tm_msgs::msg::SvrResponse>("svr_response", 1);
    
    svr_updated_ = false;

    ethernetSlaveConnection = std::make_unique<EthernetSlaveConnection>
    (iface,std::bind(&TmSvrRos2::publish_svr, this),stick_play);

    connect_tm_srv_ = this->create_service<tm_msgs::srv::ConnectTM>(
        "connect_tmsvr", std::bind(&TmSvrRos2::connect_tmsvr, this,
        std::placeholders::_1, std::placeholders::_2));
    write_item_srv_ = this->create_service<tm_msgs::srv::WriteItem>(
        "write_item", std::bind(&TmSvrRos2::write_item, this,
        std::placeholders::_1, std::placeholders::_2));
    ask_item_srv_ = this->create_service<tm_msgs::srv::AskItem>(
        "ask_item", std::bind(&TmSvrRos2::ask_item, this,
        std::placeholders::_1, std::placeholders::_2));

}

TmSvrRos2::~TmSvrRos2()
{
    if (getDataThread.joinable()) {
      getDataThread.join();
    }

    print_info("TM_ROS: (Ethernet slave) halt");		
    svr_updated_ = true;
    svr_cv_.notify_all();

}

void TmSvrRos2::publish_svr()
{
    PubMsg &pm = pm_;
    TmSvrData &data = svr_.data;
    {
        std::lock_guard<std::mutex> lck(svr_mtx_);
        pm.svr_msg.id = data.transaction_id();
        pm.svr_msg.mode = (int)(data.mode());
        pm.svr_msg.content = std::string{ data.content(), data.content_len() };
        pm.svr_msg.error_code = (int)(data.error_code());
        svr_updated_ = true;
    }
    svr_cv_.notify_all();

    if ((int)(pm.svr_msg.error_code) != 0) {
        print_error("TM_ROS: (Ethernet slave): MSG: (%s) (%d) %s", pm.svr_msg.id.c_str(), (int)pm.svr_msg.mode, pm.svr_msg.content.c_str());
        print_error("TM_ROS: (Ethernet slave): ROS Node Data Error %d", (int)(pm.svr_msg.error_code));
    } 
    
    pm.svr_msg.header.stamp = this->rclcpp::Node::now();
    pm.svr_pub->publish(pm.svr_msg);
}

bool TmSvrRos2::connect_tmsvr(
        const std::shared_ptr<tm_msgs::srv::ConnectTM::Request> req,
        std::shared_ptr<tm_msgs::srv::ConnectTM::Response> res)
{
    bool rb = true;
    int t_o = (int)(1000.0 * req->timeout);
    int t_v = (int)(1000.0 * req->timeval);
    if (req->connect) {

        rb = ethernetSlaveConnection->connect(t_o);
    }
    if (req->reconnect) {
        rb = ethernetSlaveConnection->re_connect(t_o,t_v);
    }
    else {
        ethernetSlaveConnection->no_connect();
    }
    res->ok = rb;
    return rb;
}

bool TmSvrRos2::write_item(
    const std::shared_ptr<tm_msgs::srv::WriteItem::Request> req,
    std::shared_ptr<tm_msgs::srv::WriteItem::Response> res)
{
    bool rb;
    std::string content = req->item + "=" + req->value;
    rb = (svr_.send_content_str(req->id, content) == TmCommRC::OK);
    res->ok = rb;
    return rb;
}

bool TmSvrRos2::ask_item(
    const std::shared_ptr<tm_msgs::srv::AskItem::Request> req,
    std::shared_ptr<tm_msgs::srv::AskItem::Response> res)
{
    PubMsg &pm = pm_;
    bool rb = false;

    svr_mtx_.lock();
    svr_updated_ = false;
    svr_mtx_.unlock();

    rb = (svr_.send_content(req->id, TmSvrData::Mode::READ_STRING, req->item) == TmCommRC::OK);

    {
        std::unique_lock<std::mutex> lck(svr_mtx_);
        if (rb && req->wait_time > 0.0) {
            if (!svr_updated_) {
                svr_cv_.wait_for(lck, std::chrono::duration<double>(req->wait_time));
            }
            if (!svr_updated_) {
                rb = false;
            }
            res->id = pm.svr_msg.id;
            res->value = pm.svr_msg.content;
        }
        svr_updated_ = false;
    }
    res->ok = rb;
    return rb;
}
