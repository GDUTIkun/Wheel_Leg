#include "Car.h"
#include "uart_protocol_test.h"
#include <stdio.h>

#define CAR_START_STACK                 128
#define CAR_START_PRIORITY              1
TaskHandle_t car_start_handle;
void Car_Start(void* pv);

#define DATA_TASK_STACK                 128
#define DATA_TASK_STACK_PRIORITY        5
TaskHandle_t data_task_handle;
void Data_Task(void* pv);

#define Motor_Control_TASK_STACK              128*2
#define Motor_Control_TASK_STACK_PRIORITY     3
TaskHandle_t motor_control_task_handle;
void Motor_Control_Task(void* pv);

#define Motor_Check_TASK_STACK              128
#define Motor_Check_TASK_STACK_PRIORITY     2
TaskHandle_t motor_check_task_handle;
void Motor_Check_Task(void* pv);

#define CONTROL_TASK_STACK              128
#define CONTROL_TASK_STACK_PRIORITY     4
TaskHandle_t control_task_handle;
void Control_Task(void* pv);

#define DEBUG_TASK_STACK                 128
#define DEBUG_TASK_STACK_PRIORITY        2
TaskHandle_t debug_task_handle;
void Debug_Task(void* pv);

void Can_Callback_Function(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer);

Class_Motor_DJI_C620 motor_3508_L;
Class_Motor_DJI_C620 motor_3508_R;
Class_Motor_DJI_GIM6010 motor_GIM6010_L_hip;
Class_Motor_DJI_GIM6010 motor_GIM6010_L_knee;
Class_Motor_DJI_GIM6010 motor_GIM6010_R_hip;
Class_Motor_DJI_GIM6010 motor_GIM6010_R_knee;

bool init_finished = false;
void Car_Init(void)
{
    CAN_Init(&hfdcan1, Can_Callback_Function);

    motor_3508_L.Init(&hfdcan1, Motor_DJI_ID_0x201);
    motor_3508_R.Init(&hfdcan1, Motor_DJI_ID_0x202);
    motor_GIM6010_L_hip.Init(&hfdcan1, CAN_Motor_ID_0x4E, 0.0f);
    motor_GIM6010_L_knee.Init(&hfdcan1, CAN_Motor_ID_0x6E, 0.0f);
    motor_GIM6010_R_hip.Init(&hfdcan1, CAN_Motor_ID_0x2E, 0.0f);
    motor_GIM6010_R_knee.Init(&hfdcan1, CAN_Motor_ID_0x8E, 0.0f);

    UartProtocolTest_Init();

    init_finished = true;

    xTaskCreate( (TaskFunction_t) Car_Start,
                (char *) "Car_Start", 
                (configSTACK_DEPTH_TYPE) CAR_START_STACK,
                (void *) NULL,
                (UBaseType_t) CAR_START_PRIORITY,
                (TaskHandle_t *) &car_start_handle );
    vTaskStartScheduler();
}

void Car_Start(void* pv)
{
    taskENTER_CRITICAL();

    xTaskCreate( (TaskFunction_t) Data_Task,
                (char *) "Data_Task", 
                (configSTACK_DEPTH_TYPE) DATA_TASK_STACK,
                (void *) NULL,
                (UBaseType_t) DATA_TASK_STACK_PRIORITY,
                (TaskHandle_t *) &data_task_handle );

    xTaskCreate( (TaskFunction_t) Motor_Control_Task,
                (char *) "Motor_Control_Task", 
                (configSTACK_DEPTH_TYPE) Motor_Control_TASK_STACK,
                (void *) NULL,
                (UBaseType_t) Motor_Control_TASK_STACK_PRIORITY,
                (TaskHandle_t *) &motor_control_task_handle );

    xTaskCreate( (TaskFunction_t) Motor_Check_Task,
                (char *) "Motor_Check_Task",
                (configSTACK_DEPTH_TYPE) Motor_Check_TASK_STACK,
                (void *) NULL,
                (UBaseType_t) Motor_Check_TASK_STACK_PRIORITY,
                (TaskHandle_t *) &motor_check_task_handle );

    xTaskCreate( (TaskFunction_t) Control_Task,
                (char *) "Control_Task", 
                (configSTACK_DEPTH_TYPE) CONTROL_TASK_STACK,
                (void *) NULL,
                (UBaseType_t) CONTROL_TASK_STACK_PRIORITY,
                (TaskHandle_t *) &control_task_handle );
                
    xTaskCreate( (TaskFunction_t) Debug_Task,
                (char *) "Debug_Task", 
                (configSTACK_DEPTH_TYPE) DEBUG_TASK_STACK,
                (void *) NULL,
                (UBaseType_t) DEBUG_TASK_STACK_PRIORITY,
                (TaskHandle_t *) &debug_task_handle );

    vTaskDelete(NULL);
                
    taskEXIT_CRITICAL();
}

/*===========================================
执行的任务
==============================================*/

double gyx, gyy, gyz, y;
double accx, accy, accz;

void Data_Task(void* pv)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    
    while(1)
    {
        JY901S_Acc(&accx, &accy,&accz);
        JY901S_Gyro(&gyx, &gyy, &gyz, &y);

        xTaskNotifyGive(control_task_handle);//给控制任务通知
        xTaskDelayUntil(&pxPreviousWakeTime, 5);
    }
}

static float Normalize_Deg_Signed(float angle)
{
    while (angle > 180.0f)
    {
        angle -= 360.0f;
    }
    while (angle <= -180.0f)
    {
        angle += 360.0f;
    }
    return angle;
}

static const float knee_relative_min_deg = -140.0f;
static const float knee_relative_max_deg = -70.0f;
static const float knee_limit_epsilon_deg = 0.001f;

float target = 0.0f;
float l_ref,r_ref;
float l_hip_min, l_hip_max, l_knee_min, l_knee_max;
float r_hip_angle, r_knee_angle, l_hip_angle, l_knee_angle;
float r_knee_relative_angle, l_knee_relative_angle;
float r_hip_omega, r_knee_omega, l_hip_omega, l_knee_omega;
uint8_t knee_limit_flag = 0;
uint8_t debugflag = 0;
void Motor_Control_Task(void* pv)
{    
    while(1)
    {
        motor_3508_L.Set_Target_Torque(target);
        motor_3508_L.TIM_Calculate_PeriodElapsedCallback();
        motor_3508_R.Set_Target_Torque(target);
        motor_3508_R.TIM_Calculate_PeriodElapsedCallback();
        motor_GIM6010_L_hip.Set_Target_Torque(-target);
        motor_GIM6010_L_hip.TIM_Calculate_PeriodElapsedCallback();
        motor_GIM6010_L_knee.Set_Target_Torque(-target);
        motor_GIM6010_L_knee.TIM_Calculate_PeriodElapsedCallback();
        motor_GIM6010_R_hip.Set_Target_Torque(target);
        motor_GIM6010_R_hip.TIM_Calculate_PeriodElapsedCallback();
        motor_GIM6010_R_knee.Set_Target_Torque(target);
        motor_GIM6010_R_knee.TIM_Calculate_PeriodElapsedCallback();
        l_knee_angle =  Basic_Math_Rad_To_Deg(motor_GIM6010_L_knee.Get_Now_Angle());
        r_knee_angle =  Basic_Math_Rad_To_Deg(motor_GIM6010_R_knee.Get_Now_Angle());
        l_hip_angle =  Basic_Math_Rad_To_Deg(motor_GIM6010_L_hip.Get_Now_Angle());
        r_hip_angle =  Basic_Math_Rad_To_Deg(motor_GIM6010_R_hip.Get_Now_Angle());
        l_knee_relative_angle = Normalize_Deg_Signed(l_knee_angle - l_hip_angle);
        r_knee_relative_angle = Normalize_Deg_Signed(r_knee_angle - r_hip_angle);
        knee_limit_flag =
            ((l_knee_relative_angle <= knee_relative_min_deg + knee_limit_epsilon_deg) ||
             (l_knee_relative_angle >= knee_relative_max_deg - knee_limit_epsilon_deg) ||
             (r_knee_relative_angle <= knee_relative_min_deg + knee_limit_epsilon_deg) ||
             (r_knee_relative_angle >= knee_relative_max_deg - knee_limit_epsilon_deg)) ? 1 : 0;
        l_knee_omega =  Basic_Math_Rad_To_Deg(motor_GIM6010_L_knee.Get_Now_Omega());
        r_knee_omega =  Basic_Math_Rad_To_Deg(motor_GIM6010_R_knee.Get_Now_Omega());
        l_hip_omega =  Basic_Math_Rad_To_Deg(motor_GIM6010_L_hip.Get_Now_Omega());
        r_hip_omega =  Basic_Math_Rad_To_Deg(motor_GIM6010_R_hip.Get_Now_Omega());
        TIM_1ms_CAN_PeriodElapsedCallback();
       
        
        vTaskDelay(1);
    }
}

void Motor_Check_Task(void* pv)
{
    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    
    while(1)
    {
        motor_3508_L.TIM_100ms_Alive_PeriodElapsedCallback();
        motor_3508_R.TIM_100ms_Alive_PeriodElapsedCallback();
        motor_GIM6010_L_hip.TIM_100ms_Alive_PeriodElapsedCallback();
        motor_GIM6010_L_knee.TIM_100ms_Alive_PeriodElapsedCallback();
        motor_GIM6010_R_hip.TIM_100ms_Alive_PeriodElapsedCallback();
        motor_GIM6010_R_knee.TIM_100ms_Alive_PeriodElapsedCallback();
        TIM_1ms_CAN_PeriodElapsedCallback();
        
        vTaskDelay(100);
    }
}


void Control_Task(void* pv)
{
    while(1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);//等待通知
        
       
    }
}


void Debug_Task(void* pv)
{
    TickType_t last_report_tick = xTaskGetTickCount();
    UartProtocolTestStats stats = {0};

    while(1)
    {
        UartProtocolTest_Process();

        const TickType_t now = xTaskGetTickCount();
        if ((now - last_report_tick) >= pdMS_TO_TICKS(1000))
        {
            last_report_tick = now;
            UartProtocolTest_GetStats(&stats);
            printf("uart2 rx_ok=%lu rx_crc=%lu rx_gap=%lu tx=%lu tx_err=%lu last_rx=%u last_tx=%u\r\n",
                   (unsigned long)stats.frames_ok,
                   (unsigned long)stats.crc_errors,
                   (unsigned long)stats.rx_seq_gaps,
                   (unsigned long)stats.tx_frames,
                   (unsigned long)stats.tx_errors,
                   (unsigned int)stats.last_seq,
                   (unsigned int)stats.last_tx_seq);
        }

        if(debugflag == 1)
        {
            
            
            
            debugflag = 0;
        }
        vTaskDelay(1);
    }
}


/*===================================================================================*/
//各种回调
void Can_Callback_Function(FDCAN_RxHeaderTypeDef &Header, uint8_t *Buffer)
{
    // DJI电机专属

    // 根据ID分配数据
    switch (Header.Identifier)
    {
    case (0x201):
    {
        motor_3508_L.CAN_RxCpltCallback();
        break;
    }
    case (0x202):
    {
        motor_3508_R.CAN_RxCpltCallback();
        break;
    }
    case 0x29:
    case 0x3C:
    {
        motor_GIM6010_R_hip.CAN_RxCpltCallback(Header.Identifier);
        break;
    }
    case 0x49:
    case 0x5C:
    {
        motor_GIM6010_L_hip.CAN_RxCpltCallback(Header.Identifier);
        break;
    }
    case 0x69:
    case 0x7C:
    {
        motor_GIM6010_L_knee.CAN_RxCpltCallback(Header.Identifier);
        break;
    }
    case 0x89:
    case 0x9C:
    {
        motor_GIM6010_R_knee.CAN_RxCpltCallback(Header.Identifier);
        break;
    }
    default:
        break;
    }
    
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1)
    {
        HAL_IncTick();
    }
    else if(htim->Instance == TIM3)
    {
        HAL_TIM_Base_Stop_IT(&htim3);
        delayflag = 1;
    }
}
