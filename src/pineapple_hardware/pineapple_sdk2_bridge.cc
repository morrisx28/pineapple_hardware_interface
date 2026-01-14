#include "pineapple_sdk2_bridge.h"

PineappleSdk2Bridge::PineappleSdk2Bridge(const char *dev_sn)
{
    for (int i=0; i < num_motor_; i++)
    {
        dm_data_list.push_back(damiao::DmActData{.motorType = static_cast<damiao::DM_Motor_Type>(motor_type[i]),
        .mode = damiao::POS_VEL_MODE,
        .can_id=static_cast<uint16_t>(can_id_list[i]),
        .mst_id=static_cast<uint16_t>(mst_id_list[i])});
    }
    motor_control = std::make_shared<damiao::Motor_Control>(nom_baud,dat_baud,
      dev_sn,&dm_data_list);

    low_cmd_go_suber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    low_cmd_go_suber_->InitChannel(bind(&PineappleSdk2Bridge::LowCmdGoHandler, this, placeholders::_1), 1);

    low_state_go_puber_.reset(new ChannelPublisher<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    low_state_go_puber_->InitChannel();

    lowStatePuberThreadPtr = CreateRecurrentThreadEx("lowstate", UT_CPU_ID_NONE, 2000, &PineappleSdk2Bridge::PublishLowStateGo, this);

}

PineappleSdk2Bridge::~PineappleSdk2Bridge()
{
    
}

void PineappleSdk2Bridge::SetMotorToZero()
{
    for (int i = 0; i < num_motor_; i++) 
    {
        motor_control->set_zero_position(*motor_control->getMotor(can_id_list[i]));
    }
}

void PineappleSdk2Bridge::LowCmdGoHandler(const void *msg)
{
    const unitree_go::msg::dds_::LowCmd_ *cmd = (const unitree_go::msg::dds_::LowCmd_ *)msg;
    for (int i = 0; i < num_motor_; i++)
    {
        motor_control->control_mit(*motor_control->getMotor(can_id_list[i]), cmd->motor_cmd()[i].kp(), cmd->motor_cmd()[i].kd(), 
                                                            cmd->motor_cmd()[i].q(), cmd->motor_cmd()[i].dq(), cmd->motor_cmd()[i].tau());
        // motor_control->control_vel(*motor_control->getMotor(can_id_list[i]), cmd->motor_cmd()[i].dq());
    }
}

void PineappleSdk2Bridge::PublishLowStateGo()
{

    for (uint16_t i = 0;i < num_motor_; i++)
    {
        low_state_go_.motor_state()[i].q() = motor_control->getMotor(can_id_list[i])->Get_Position();
        low_state_go_.motor_state()[i].dq() = motor_control->getMotor(can_id_list[i])->Get_Velocity();
        low_state_go_.motor_state()[i].tau_est() = motor_control->getMotor(can_id_list[i])->Get_tau();
    }
    

    low_state_go_puber_->Write(low_state_go_);

    //     if (have_imu_)
    //     {
    //         low_state_go_.imu_state().quaternion()[0] = mj_data_->sensordata[dim_motor_sensor_ + 0];
    //         low_state_go_.imu_state().quaternion()[1] = mj_data_->sensordata[dim_motor_sensor_ + 1];
    //         low_state_go_.imu_state().quaternion()[2] = mj_data_->sensordata[dim_motor_sensor_ + 2];
    //         low_state_go_.imu_state().quaternion()[3] = mj_data_->sensordata[dim_motor_sensor_ + 3];

    //         low_state_go_.imu_state().gyroscope()[0] = mj_data_->sensordata[dim_motor_sensor_ + 4];
    //         low_state_go_.imu_state().gyroscope()[1] = mj_data_->sensordata[dim_motor_sensor_ + 5];
    //         low_state_go_.imu_state().gyroscope()[2] = mj_data_->sensordata[dim_motor_sensor_ + 6];

    //         low_state_go_.imu_state().accelerometer()[0] = mj_data_->sensordata[dim_motor_sensor_ + 7];
    //         low_state_go_.imu_state().accelerometer()[1] = mj_data_->sensordata[dim_motor_sensor_ + 8];
    //         low_state_go_.imu_state().accelerometer()[2] = mj_data_->sensordata[dim_motor_sensor_ + 9];
    //     }

    //     low_state_go_puber_->Write(low_state_go_);
    // }
}

