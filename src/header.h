#pragma once
#include <chrono>
#include <functional>
#include <memory>
#include <regex>
#include <cmath>

#include <string>
#include <vector>
#include <deque>
#include <array>
#include <map>
#include <set>

#include <thread>
#include <atomic>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "vehicle_interfaces/control.h"
#include "vehicle_interfaces/utils.h"

#include "vehicle_interfaces/msg/chassis_info.hpp"
#include "vehicle_interfaces/msg/control_chassis.hpp"
#include "vehicle_interfaces/msg/controller_info.hpp"
#include "vehicle_interfaces/msg/control_server.hpp"
#include "vehicle_interfaces/msg/control_steering_wheel.hpp"

#include "vehicle_interfaces/srv/controller_info_reg.hpp"
#include "vehicle_interfaces/srv/controller_info_req.hpp"
#include "vehicle_interfaces/srv/control_server.hpp"
#include "vehicle_interfaces/srv/control_steering_wheel_reg.hpp"
#include "vehicle_interfaces/srv/control_steering_wheel_req.hpp"

#include "IDClient.h"

#define REMOTE_STEERINGWHEEL_LENGTH 6

using namespace std::chrono_literals;

class Params : public vehicle_interfaces::GenericParams
{
private:
    // Callback
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr _paramsCallbackHandler;
    std::function<void(const rclcpp::Parameter)> cbFunc_;
    std::atomic<bool> cbFuncSetF_;

public:
    int msg_type = 0;
    int controller_mode = 0;
    std::string service_name = "controller";

    double timeout_ms = 30.0;
    double period_ms = 50.0;
    int privilege = 100;
    int pub_type = 0;

    std::string externalHostIP = "61.220.23.240";
    std::string externalPort = "10000";
    std::string externalID = "CAR1";
    double externalTimeout_ms = 2000.0;

    std::string controlService = "controlserver_0";

private:
    void _getParams()
    {
        this->get_parameter("msg_type", this->msg_type);
        this->get_parameter("controller_mode", this->controller_mode);
        this->get_parameter("service_name", this->service_name);
        this->get_parameter("timeout_ms", this->timeout_ms);
        this->get_parameter("period_ms", this->period_ms);
        this->get_parameter("privilege", this->privilege);
        this->get_parameter("pub_type", this->pub_type);
        this->get_parameter("externalHostIP", this->externalHostIP);
        this->get_parameter("externalPort", this->externalPort);
        this->get_parameter("externalID", this->externalID);
        this->get_parameter("externalTimeout_ms", this->externalTimeout_ms);
        this->get_parameter("controlService", this->controlService);
    }

    rcl_interfaces::msg::SetParametersResult _paramsCallback(const std::vector<rclcpp::Parameter>& params)
    {
        rcl_interfaces::msg::SetParametersResult ret;
        ret.successful = true;
        ret.reason = "";

        if (!this->cbFuncSetF_)
            return ret;

        for (const auto& param : params)
        {
            try
            {
                this->cbFunc_(param);
            }
            catch (...)
            {
                ret.successful = false;
                ret.reason = "[Params::_paramsCallback] Caught Unknown Exception!";
            }
        }

        return ret;
    }

public:
    Params(std::string nodeName) : vehicle_interfaces::GenericParams(nodeName), 
        cbFuncSetF_(false)
    {
        this->declare_parameter<int>("msg_type", this->msg_type);
        this->declare_parameter<int>("controller_mode", this->controller_mode);
        this->declare_parameter<std::string>("service_name", this->service_name);
        this->declare_parameter<double>("timeout_ms", this->timeout_ms);
        this->declare_parameter<double>("period_ms", this->period_ms);
        this->declare_parameter<int>("privilege", this->privilege);
        this->declare_parameter<int>("pub_type", this->pub_type);
        this->declare_parameter<std::string>("externalHostIP", this->externalHostIP);
        this->declare_parameter<std::string>("externalPort", this->externalPort);
        this->declare_parameter<std::string>("externalID", this->externalID);
        this->declare_parameter<double>("externalTimeout_ms", this->externalTimeout_ms);
        this->declare_parameter<std::string>("controlService", this->controlService);
        this->_getParams();

        this->_paramsCallbackHandler = this->add_on_set_parameters_callback(std::bind(&Params::_paramsCallback, this, std::placeholders::_1));
    }

    void addCallbackFunc(const std::function<void(const rclcpp::Parameter)>& callback)
    {
        this->cbFunc_ = callback;
        this->cbFuncSetF_ = true;
    }
};


class Controller : public vehicle_interfaces::VehicleServiceNode
{
private:
    const std::shared_ptr<Params> params_;// Controller parameters.
    std::shared_ptr<vehicle_interfaces::SteeringWheelControllerServer> controller_;// Communicate with ControlServerController.
    rclcpp::executors::SingleThreadedExecutor* executor_;// Executor for Controller.
    std::thread* execTh_;// Executor thread.
    vehicle_interfaces::msg::ControlSteeringWheel controlSteeringWheelMsg_;// ControlSteeringWheel message.
    std::chrono::high_resolution_clock::time_point controlSteeringWheelMsgTs_;// Timestamp of controlSteeringWheelMsg_.
    std::mutex controlSteeringWheelMsgLock_;// Lock controlSteeringWheelMsg_.

    // ChassisInfo requested from ControlServer.
    vehicle_interfaces::msg::ChassisInfo chassisInfo_;
    std::atomic<bool> chassisInfoF_;// Check valid chassisInfo_.

    // Ying IDClient.
    IDClient* idclient_;// ID client.
    std::atomic<bool> idclientF_;// Check valid idclient_.
    vehicle_interfaces::Timer* idclientTm_;// Call _idclientCbFunc().
    std::atomic<double> idclientTmPeriod_ms_;// The period of idclient check.
    std::atomic<bool> idclientTmF_;// Enable/Disable idclient check.

    // Remote flag.
    std::atomic<bool> remoteF_;// Remote flag.
    std::string remoteDevice_;// Remote device name.

    // ControlServer output signal subscription.
    rclcpp::Subscription<vehicle_interfaces::msg::Chassis>::SharedPtr chassisSub_;// Chassis subscription.
    vehicle_interfaces::msg::Chassis chassisMsg_;// Chassis message.
    std::mutex chassisMsgLock_;// Lock chassisMsg_.

    // Node control.
    rclcpp::Node::SharedPtr reqClientNode_;// Node for request client.
    std::atomic<bool> exitF_;

private:
    template <typename T>
    void _safeSave(T* ptr, const T value, std::mutex& lock)
    {
        std::lock_guard<std::mutex> _lock(lock);
        *ptr = value;
    }

    template <typename T>
    T _safeCall(const T* ptr, std::mutex& lock)
    {
        std::lock_guard<std::mutex> _lock(lock);
        return *ptr;
    }

    /**
     * Receive message event handler for IDClient.
     */
    void RecvMsgEventHandler(IDClient* idc, std::string fromDevice, std::string recvMsg)
    {
        if (!this->remoteF_ && recvMsg == "RemoteRegister")// Set remote device name
        {
            try// Register remote device
            {
                idc->requestIDTableFromServer();
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                idc->sendMsgToClient(fromDevice, "ControlRegister");// Send apply signal.
                this->remoteF_ = true;
                this->remoteDevice_ = fromDevice;

                // Update external signal timestamp.
                std::lock_guard<std::mutex> lock(this->controlSteeringWheelMsgLock_);
                this->controlSteeringWheelMsgTs_ = std::chrono::high_resolution_clock::now();

                RCLCPP_INFO(this->get_logger(), "[Controller::RecvMsgEventHandler] Remote device registered: %s", fromDevice.c_str());
            }
            catch (...)
            {
                RCLCPP_ERROR(this->get_logger(), "[Controller::RecvMsgEventHandler] Remote device register failed: %s", fromDevice.c_str());
                auto table = idc->getIDTable();
                for (auto& i : table)
                    std::cerr << " [" << i << "] ";
            }
        }
        else if (this->remoteF_ && recvMsg == "RequestComposition")// Response chassis composition
        {
            if (!this->chassisInfoF_)// ChassisInfo not ready.
                return;
            std::string sendStr = "@COMP!";
            sendStr += "$@M!";
            for (int i = 0; i < this->chassisInfo_.vehicle_type; i++)
                sendStr += "@M!M" + std::to_string(i) + "_1:";
            sendStr.pop_back();

            sendStr += "$@S!";
            for (int i = 0; i < this->chassisInfo_.vehicle_type; i++)
                sendStr += "@S!S" + std::to_string(i) + "_1:";
            sendStr.pop_back();

            sendStr += "$@B!";
            for (int i = 0; i < this->chassisInfo_.vehicle_type; i++)
                sendStr += "@B!B" + std::to_string(i) + "_1:";
            sendStr.pop_back();
            
            idc->sendMsgToClient(fromDevice, sendStr);

            // Update external signal timestamp.
            std::lock_guard<std::mutex> lock(this->controlSteeringWheelMsgLock_);
            this->controlSteeringWheelMsgTs_ = std::chrono::high_resolution_clock::now();

            RCLCPP_INFO(this->get_logger(), "[Controller::RecvMsgEventHandler] Chassis composition sent to remote device: %s", fromDevice.c_str());
        }
        else if (this->remoteF_ && recvMsg != "RemoteRegister")// Control signal from remote device.
        {
            try
            {
                std::vector<std::string> splitStr = vehicle_interfaces::split(recvMsg, ":");
                if (splitStr.size() < REMOTE_STEERINGWHEEL_LENGTH)
                    return;

                vehicle_interfaces::msg::ControlSteeringWheel tmp;
                if (splitStr[0] == "Park")
                    tmp.gear = vehicle_interfaces::msg::ControlSteeringWheel::GEAR_PARK;
                else if (splitStr[0] == "Reverse")
                    tmp.gear = vehicle_interfaces::msg::ControlSteeringWheel::GEAR_REVERSE;
                else if (splitStr[0] == "Neutral")
                    tmp.gear = vehicle_interfaces::msg::ControlSteeringWheel::GEAR_NEUTRAL;
                else if (splitStr[0] == "Drive")
                    tmp.gear = vehicle_interfaces::msg::ControlSteeringWheel::GEAR_DRIVE;
                tmp.steering = std::stoi(splitStr[1]);
                tmp.pedal_throttle = std::stoi(splitStr[2]);
                tmp.pedal_brake = std::stoi(splitStr[3]);
                tmp.pedal_clutch = std::stoi(splitStr[4]);
                tmp.func_0 = std::stoi(splitStr[5]);// Default Ackermann steering mode.
                tmp.func_1 = 0;
                tmp.func_2 = 0;
                tmp.func_3 = 0;

                auto subChassisMsg = this->_safeCall(&this->chassisMsg_, this->chassisMsgLock_);

                char buf[1024];
                if (subChassisMsg.drive_motor.size() == this->chassisInfo_.vehicle_type)
                {
                    sprintf(buf, "#%s!%f:%f:%f:%f:%f:%f:%f:%f:%d:%d", 
                            splitStr.back().c_str(), 
                            subChassisMsg.drive_motor[0], subChassisMsg.steering_motor[0], 
                            subChassisMsg.drive_motor[1], subChassisMsg.steering_motor[1], 
                            subChassisMsg.drive_motor[2], subChassisMsg.steering_motor[2], 
                            subChassisMsg.drive_motor[3], subChassisMsg.steering_motor[3], (int)tmp.gear, (int)tmp.func_0);
                }
                else
                {
                    sprintf(buf, "#%s!%f:%f:%f:%f:%f:%f:%f:%f:%d:%d", 
                            splitStr.back().c_str(), 
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, (int)tmp.gear, (int)tmp.func_0);
                }
                idc->sendMsgToClient(fromDevice, buf);
                // printf("Send: %s\n", buf);

                // Filled controlSteeringWheelMsg_ with remote message.
                std::lock_guard<std::mutex> lock(this->controlSteeringWheelMsgLock_);// Lock controlSteeringWheelMsg_.
                this->controller_->setControlSignal(this->controlSteeringWheelMsg_);
                this->controlSteeringWheelMsg_ = tmp;
                this->controlSteeringWheelMsgTs_ = std::chrono::high_resolution_clock::now();
            }
            catch (...)
            {
                this->remoteF_ = false;
                this->idclientF_ = false;
                RCLCPP_ERROR(this->get_logger(), "[Controller::RecvMsgEventHandler] Remote device message error: %s", recvMsg.c_str());
            }
        }
    }

    /**
     * (Sub-thread) Timer callback function for idclient.
     */
    void _idclientCbFunc()
    {
        if (!this->idclientF_)// Need reconnect to id server.
        {
            // Set host and packet parameters
            IDServerProp prop(params_->externalHostIP, params_->externalPort, PACKET_HEADER_SIZE, PACKET_PAYLOAD_SIZE);
            try
            {
                this->remoteF_ = false;
                if (this->idclient_ == nullptr)
                {
                    this->idclient_ = new IDClient(prop);
                    this->idclient_->setRecvMsgEventHandler(std::bind(&Controller::RecvMsgEventHandler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), true);
                    this->idclient_->connToServer();
                    this->idclient_->regToServer(params_->externalID);
                    this->idclientF_ = this->idclient_->isServerConn();
                }
                else
                {
                    delete this->idclient_;
                    this->idclient_ = new IDClient(prop);
                    this->idclient_->setRecvMsgEventHandler(std::bind(&Controller::RecvMsgEventHandler, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3), true);
                    this->idclient_->connToServer();
                    this->idclient_->regToServer(params_->externalID);
                    this->idclientF_ = this->idclient_->isServerConn();
                }
            }
            catch (...)
            {
                if (this->idclient_ != nullptr)
                    delete this->idclient_;
                this->idclient_ = nullptr;
                this->idclientF_ = false;
                RCLCPP_ERROR(this->get_logger(), "[Controller::_idclientCbFunc] Caught unexpected error.");
            }
            if (!this->idclientF_)
                RCLCPP_ERROR(this->get_logger(), "[Controller::_idclientCbFunc] ID client connection failed.");
            return;
        }

        std::lock_guard<std::mutex> lock(this->controlSteeringWheelMsgLock_);// Lock controlSteeringWheelMsg_.
        if (this->remoteF_ && std::chrono::high_resolution_clock::now() - this->controlSteeringWheelMsgTs_ > std::chrono::duration<double, std::milli>(this->params_->externalTimeout_ms))
        {
            this->controlSteeringWheelMsg_ = vehicle_interfaces::msg::ControlSteeringWheel();
            this->controlSteeringWheelMsg_.gear = vehicle_interfaces::msg::ControlSteeringWheel::GEAR_PARK;
            // Set control signal.
            this->controller_->setControlSignal(this->controlSteeringWheelMsg_);
            this->remoteF_ = false;
            RCLCPP_ERROR(this->get_logger(), "[Controller::_idclientCbFunc] Remote device timeout.");
        }
    }

    /**
     * Chassis message callback function.
     */
    void _chassisCbFunc(const vehicle_interfaces::msg::Chassis::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(this->chassisMsgLock_);// Lock chassisMsg_.
        this->chassisMsg_ = *msg;
    }

public:
    Controller(const std::shared_ptr<Params>& params) : 
        vehicle_interfaces::VehicleServiceNode(params), 
        rclcpp::Node(params->nodeName), 
        params_(params), 
        executor_(nullptr), 
        execTh_(nullptr), 
        chassisInfoF_(false), 
        idclient_(nullptr), 
        idclientF_(false), 
        idclientTm_(nullptr), 
        idclientTmPeriod_ms_(0), 
        idclientTmF_(false), 
        remoteF_(false), 
        exitF_(false)
    {
        // Initialize steering wheel control message.
        this->controlSteeringWheelMsg_.gear = vehicle_interfaces::msg::ControlSteeringWheel::GEAR_PARK;
        this->controlSteeringWheelMsg_.steering = 0.0;
        this->controlSteeringWheelMsg_.pedal_throttle = 0.0;
        this->controlSteeringWheelMsg_.pedal_brake = 0.0;
        this->controlSteeringWheelMsg_.pedal_clutch = 0.0;
        this->controlSteeringWheelMsg_.func_0 = 1;// Default Ackermann steering mode.
        this->controlSteeringWheelMsg_.func_1 = 0;
        this->controlSteeringWheelMsg_.func_2 = 0;
        this->controlSteeringWheelMsg_.func_3 = 0;

        // Create idclient timer.
        this->idclientTmPeriod_ms_ = params->period_ms;
        RCLCPP_INFO(this->get_logger(), "[Controller] Initializing idclient timer...");
        this->idclientTm_ = new vehicle_interfaces::Timer(params->period_ms, std::bind(&Controller::_idclientCbFunc, this));
        this->idclientTm_->start();

        vehicle_interfaces::msg::ControllerInfo conInfo;
        conInfo.msg_type = params->msg_type;
        conInfo.controller_mode = params->controller_mode;
        conInfo.service_name = params->service_name;
        conInfo.timeout_ms = params->timeout_ms;
        conInfo.period_ms = params->period_ms;
        conInfo.privilege = params->privilege;
        conInfo.pub_type = params->pub_type;

        if (conInfo.msg_type == vehicle_interfaces::msg::ControllerInfo::MSG_TYPE_STEERING_WHEEL)
        {
            this->controller_ = std::make_shared<vehicle_interfaces::SteeringWheelControllerServer>(conInfo, params->controlService);
            this->controller_->setControlSignal(this->controlSteeringWheelMsg_);// Default control signal.
            this->executor_ = new rclcpp::executors::SingleThreadedExecutor();
            this->executor_->add_node(this->controller_);
            this->execTh_ = new std::thread(vehicle_interfaces::SpinExecutor, this->executor_, "controller", 1000.0);
        }

        // Service client node.
        this->reqClientNode_ = rclcpp::Node::make_shared(params->nodeName + "_client");

        {
            auto client = this->reqClientNode_->create_client<vehicle_interfaces::srv::ControllerInfoReg>(params->controlService + "_Reg");
            bool stopF = false;
            vehicle_interfaces::ConnToService(client, stopF, std::chrono::milliseconds(5000), -1);

            auto request = std::make_shared<vehicle_interfaces::srv::ControllerInfoReg::Request>();
            request->request = conInfo;
CONTROLLER_REG_TAG:
            auto result = client->async_send_request(request);
#if ROS_DISTRO == 0
            if (rclcpp::spin_until_future_complete(this->reqClientNode_, result, 500ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
            if (rclcpp::spin_until_future_complete(this->reqClientNode_, result, 500ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
            {
                RCLCPP_INFO(this->get_logger(), "[Controller] Register to control server success.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "[Controller] Register to control server failed.");
                goto CONTROLLER_REG_TAG;
            }
            request.reset();
            client.reset();
        }

        {
            auto client = this->reqClientNode_->create_client<vehicle_interfaces::srv::ControlServer>(params->controlService);
            bool stopF = false;
            vehicle_interfaces::ConnToService(client, stopF, std::chrono::milliseconds(5000), -1);

            auto request = std::make_shared<vehicle_interfaces::srv::ControlServer::Request>();
CONTROLSERVER_TAG:
            auto result = client->async_send_request(request);
#if ROS_DISTRO == 0
            if (rclcpp::spin_until_future_complete(this->reqClientNode_, result, 500ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
            if (rclcpp::spin_until_future_complete(this->reqClientNode_, result, 500ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
            {
                auto response = result.get();
                this->chassisInfo_ = response->status.chassis_info;
                this->chassisInfoF_ = true;
                RCLCPP_INFO(this->get_logger(), "[Controller] Request chassis info success.");
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "[Controller] Request chassis info failed.");
                goto CONTROLSERVER_TAG;
            }
            request.reset();
            client.reset();
        }

        // Init chassis subscription.
        this->chassisSub_ = this->create_subscription<vehicle_interfaces::msg::Chassis>(
            params->controlService, 10, std::bind(&Controller::_chassisCbFunc, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "[Controller] Constructed.");
    }

    ~Controller()
    {
        this->close();
    }

    void close()
    {
        if (this->exitF_)// Ignore process if called repeatedly.
            return;
        this->exitF_ = true;// All looping process will be braked if exitF_ set to true.

        // Destroy idclient timer.
        if (this->idclientTm_ != nullptr)
        {
            this->idclientTm_->destroy();
            delete this->idclientTm_;
        }
        // Destroy executor.
        if (this->execTh_ != nullptr)
        {
            this->executor_->cancel();
            this->execTh_->join();
            delete this->execTh_;
            delete this->executor_;
        }
    }
};
