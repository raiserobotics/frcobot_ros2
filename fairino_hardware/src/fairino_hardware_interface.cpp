#include "fairino_hardware/fairino_hardware_interface.hpp"

namespace fairino_hardware {

hardware_interface::CallbackReturn FairinoHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &sysinfo) {
  if (hardware_interface::SystemInterface::on_init(sysinfo) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  info_ = sysinfo; // info_是父类中定义的变量

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {

    // 指令部分
    if (joint.command_interfaces.size() != 1) { // 开放servoJ
      RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                   "Joint '%s' has %zu command interfaces found. 1 expected.",
                   joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name !=
        hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as first "
                   "command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // if (joint.command_interfaces[1].name !=
    // hardware_interface::HW_IF_EFFORT){//预留，用于关节扭矩直接控制
    //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
    //            "Joint '%s' have %s command interfaces found as first command
    //            interface. '%s' expected.", joint.name.c_str(),
    //            joint.command_interfaces[1].name.c_str(),
    //            hardware_interface::HW_IF_EFFORT);
    //     return hardware_interface::CallbackReturn::ERROR;
    // }

    // 关节状态部分
    if (joint.state_interfaces.size() < 1) {
      RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                   "Joint '%s' has %zu state interface. At least 1 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
                   "Joint '%s' have %s state interface as first state "
                   "interface. '%s' expected.",
                   joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    // {
    //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
    //                 "Joint '%s' have %s state interface as second state
    //                 interface. '%s' expected.", joint.name.c_str(),
    //                 joint.state_interfaces[1].name.c_str(),
    //                 hardware_interface::HW_IF_VELOCITY);
    //     return hardware_interface::CallbackReturn::ERROR;
    // }

    // if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
    //     RCLCPP_FATAL(rclcpp::get_logger("FairinoHardwareInterface"),
    //                 "Joint '%s' have %s state interface as third state
    //                 interface. '%s' expected.", joint.name.c_str(),
    //                 joint.state_interfaces[2].name.c_str(),
    //                 hardware_interface::HW_IF_EFFORT);
    //     return hardware_interface::CallbackReturn::ERROR;
    // }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
} // end on_init

std::vector<hardware_interface::StateInterface>
FairinoHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // 导出关节相关的状态接口(位置，速度，扭矩)
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &_jnt_position_state[i]));

    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
    //     &_jnt_velocity_state.at(i)));

    // state_interfaces.emplace_back(hardware_interface::StateInterface(
    //     info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
    //     &_jnt_torque_state.at(i)));
  }

  // 导出
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
FairinoHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &_jnt_position_command[i]));

    //     command_interfaces.emplace_back(hardware_interface::CommandInterface(//预留的扭矩控制接口
    //         info_.joints[i].name, hardware_interface::HW_IF_EFFORT,
    //         &_jnt_torque_command.at(i)));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_activate(
    const rclcpp_lifecycle::State &previous_state) {
  using namespace std::chrono_literals;
  RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
              "Starting ...please wait...");
  // 做变量的初始化工作
  _ptr_robot = std::make_unique<FRRobot>(); // 创建机器人实例
  for (int i = 0; i < 6; i++) {             // 初始化变量
    _jnt_position_command[i] = 0;
    _jnt_velocity_command[i] = 0;
    _jnt_torque_command[i] = 0;
    _jnt_position_state[i] = 0;
    _jnt_velocity_state[i] = 0;
    _jnt_torque_state[i] = 0;
  }
  _control_mode = 0; // 默认是位置控制,0-位置控制，1-扭矩控制 2-速度控制
  _enable_freedrive = false;
  _disable_freedrive = false;
  _in_freedrive = false;
  errno_t returncode = _ptr_robot->RPC(info_.hardware_parameters.at("ip_address").c_str()); // 建立xmlrpc连接
  rclcpp::sleep_for(200ms); // 等待一段时间让控制器的rpc连接建立完毕
  if (returncode != 0) {
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                "机械臂SDK连接失败！请检查端口时候被占用");
    return hardware_interface::CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                "机械臂SDK连接成功！");
  }
  // 做第一步的工作，读取当前状态数据
  JointPos jntpos;
  returncode = _ptr_robot->GetActualJointPosDegree(0, &jntpos);
  /*
  获取反馈位置后同步到指令位置以维持当前状态，如果发现读取失败，那么就无法激活插件，
  因为错误的反馈位置会导致初始指令位置下发出现严重偏差导致事故
  */
  if (returncode == 0) {
    for (int j = 0; j < 6; j++) {
      _jnt_position_command[j] = jntpos.jPos[j] / 180.0 * M_PI;
    }
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                "初始指令位置: %f,%f,%f,%f,%f,%f", _jnt_position_command[0],
                _jnt_position_command[1], _jnt_position_command[2],
                _jnt_position_command[3], _jnt_position_command[4],
                _jnt_position_command[5]);
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                "机械臂硬件启动成功!");
  } else {
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                "读取初始关节角度错误，硬件无法启动！请检查通讯内容");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Start freedrive service node
  _freedrive_node = std::make_shared<rclcpp::Node>(info_.name + "_freedrive");
  _start_freedrive_srv = _freedrive_node->create_service<std_srvs::srv::Trigger>(
      "start_freedrive",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr resp) {
        _enable_freedrive.store(true);
        resp->success = true;
        resp->message = "Freedrive enable requested";
      });
  _stop_freedrive_srv = _freedrive_node->create_service<std_srvs::srv::Trigger>(
      "stop_freedrive",
      [this](const std_srvs::srv::Trigger::Request::SharedPtr,
             std_srvs::srv::Trigger::Response::SharedPtr resp) {
        _disable_freedrive.store(true);
        resp->success = true;
        resp->message = "Freedrive disable requested";
      });
  _freedrive_executor =
      std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  _freedrive_executor->add_node(_freedrive_node);
  _freedrive_executor_thread =
      std::thread([this]() { _freedrive_executor->spin(); });

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FairinoHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State &previous_state) {
  RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
              "Stopping ...please wait...");

  if (_freedrive_executor) {
    _freedrive_executor->cancel();
  }
  if (_freedrive_executor_thread.joinable()) {
    _freedrive_executor_thread.join();
  }
  _start_freedrive_srv.reset();
  _stop_freedrive_srv.reset();
  _freedrive_node.reset();
  _freedrive_executor.reset();

  // If we were in freedrive, exit drag mode before stopping
  if (_in_freedrive.load()) {
    _ptr_robot->DragTeachSwitch(0);
    _ptr_robot->Mode(0);
    _in_freedrive = false;
  }

  _ptr_robot->StopMotion(); // 停止机器人
  _ptr_robot->CloseRPC();   // 销毁实例，连接断开
  _ptr_robot.release();
  RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
              "System successfully stopped!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type FairinoHardwareInterface::read(
    const rclcpp::Time &time,
    const rclcpp::Duration
        &period) { // 从RTDE反馈数据中获取所需的位置，速度和扭矩信息

  if (_enable_freedrive.exchange(false)) {
    errno_t mode_err = _ptr_robot->Mode(1);
    errno_t drag_err = _ptr_robot->DragTeachSwitch(1);
    if (mode_err == 0 && drag_err == 0) {
      _in_freedrive = true;
      RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                  "Freedrive enabled");
    } else {
      RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                  "Failed to enable freedrive: Mode=%d DragTeach=%d", mode_err,
                  drag_err);
      // Rollback: if Mode(1) succeeded but DragTeachSwitch failed, return to
      // auto mode so ServoJ commands continue working
      if (mode_err == 0) {
        _ptr_robot->Mode(0);
      }
    }
  } else if (_disable_freedrive.exchange(false)) {
    errno_t drag_err = _ptr_robot->DragTeachSwitch(0);
    errno_t mode_err = _ptr_robot->Mode(0);
    errno_t servo_err = _ptr_robot->ServoMoveStart();
    _in_freedrive = false;
    if (mode_err == 0 && drag_err == 0) {
      RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                  "Freedrive disabled");
    } else {
      RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                  "Failed to disable freedrive: DragTeach=%d Mode=%d", drag_err,
                  mode_err);
    }
    if (servo_err != 0) {
      RCLCPP_WARN(rclcpp::get_logger("FairinoHardwareInterface"),
                  "ServoMoveStart failed after disabling freedrive: %d",
                  servo_err);
    }
  }

  JointPos state_data;
  error_t returncode = _ptr_robot->GetActualJointPosDegree(1, &state_data);
  if (returncode == 0) {
    for (int i = 0; i < 6; i++) {
      _jnt_position_state[i] =
          state_data.jPos[i] / 180.0 * M_PI; // 注意单位转换，moveit统一用弧度
      //_jnt_torque_state[i] = state_data.jt_cur_tor[i];//注意单位转换
    }
  } else {
    return hardware_interface::return_type::ERROR;
  }
  //RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "System successfully read: %f,%f,%f,%f,%f,%f",_jnt_position_state[0],\
    _jnt_position_state[1],_jnt_position_state[2],_jnt_position_state[3],_jnt_position_state[4],_jnt_position_state[5]);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
FairinoHardwareInterface::write(const rclcpp::Time &time,
                                const rclcpp::Duration &period) {
  if (_in_freedrive) {
    return hardware_interface::return_type::OK;
  }
  if (_control_mode == 0) { // 位置控制模式
    if (std::any_of(&_jnt_position_command[0], &_jnt_position_command[5],
                    [](double c) { return not std::isfinite(c); })) {
      return hardware_interface::return_type::ERROR;
    }
    JointPos cmd;
    ExaxisPos extcmd{0, 0, 0, 0};
    for (auto j = 0; j < 6; j++) {
      cmd.jPos[j] = _jnt_position_command[j] / M_PI * 180; // 注意单位转换
    }
    //RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"), "ServoJ下发位置:%f,%f,%f,%f,%f,%f",\
            cmd.jPos[0],cmd.jPos[1],cmd.jPos[2],cmd.jPos[3],cmd.jPos[4],cmd.jPos[5]);
    int returncode = _ptr_robot->ServoJ(&cmd, &extcmd, 0, 0, 0.008, 0, 0);
    if (returncode != 0) {
      RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                  "ServoJ指令下发错误,错误码:%d", returncode);
    }
  } else if (_control_mode == 1) { // 扭矩控制模式
    if (std::any_of(&_jnt_torque_command[0], &_jnt_torque_command[5],
                    [](double c) { return not std::isfinite(c); })) {
      return hardware_interface::return_type::ERROR;
    }
    //_ptr_robot->write(_jnt_torque_command);//注意单位转换
  } else {
    RCLCPP_INFO(rclcpp::get_logger("FairinoHardwareInterface"),
                "指令发送错误:未识别当前所处控制模式");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

} // namespace fairino_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(fairino_hardware::FairinoHardwareInterface,
                       hardware_interface::SystemInterface)
