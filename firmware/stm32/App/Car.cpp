#include "Car.h"

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
Class_Motor_DJI_GIM6010 motor_GIM6010_L_1;
Class_Motor_DJI_GIM6010 motor_GIM6010_L_2;
Class_Motor_DJI_GIM6010 motor_GIM6010_R_1;
Class_Motor_DJI_GIM6010 motor_GIM6010_R_2;

bool init_finished = false;
float refangle = 1.5656f;
void Car_Init(void)
{
    CAN_Init(&hfdcan1, Can_Callback_Function);

    motor_3508_L.Init(&hfdcan1, Motor_DJI_ID_0x201);
    motor_3508_R.Init(&hfdcan1, Motor_DJI_ID_0x202);
    motor_GIM6010_L_1.Init(&hfdcan1, CAN_Motor_ID_0x4E, 0.0f);
    motor_GIM6010_L_2.Init(&hfdcan1, CAN_Motor_ID_0x6E, 0.0f);
    motor_GIM6010_R_1.Init(&hfdcan1, CAN_Motor_ID_0x2E, 0.0f);
    motor_GIM6010_R_2.Init(&hfdcan1, CAN_Motor_ID_0x8E, 0.0f);

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

float target = 0.0f;
float l,r;
uint8_t debugflag = 0;
void Motor_Control_Task(void* pv)
{    
    while(1)
    {
        motor_3508_L.Set_Target_Torque(target);
        motor_3508_L.TIM_Calculate_PeriodElapsedCallback();
        motor_3508_R.Set_Target_Torque(target);
        motor_3508_R.TIM_Calculate_PeriodElapsedCallback();
        motor_GIM6010_L_1.Set_Target_Torque(-target);
        motor_GIM6010_L_1.TIM_Calculate_PeriodElapsedCallback();
        motor_GIM6010_L_2.Set_Target_Torque(-target);
        motor_GIM6010_L_2.TIM_Calculate_PeriodElapsedCallback();
        motor_GIM6010_R_1.Set_Target_Torque(target);
        motor_GIM6010_R_1.TIM_Calculate_PeriodElapsedCallback();
        motor_GIM6010_R_2.Set_Target_Torque(target);
        motor_GIM6010_R_2.TIM_Calculate_PeriodElapsedCallback();
        l =  Basic_Math_Rad_To_Deg(motor_GIM6010_L_1.Get_Now_Angle());
        r =  Basic_Math_Rad_To_Deg(motor_GIM6010_R_1.Get_Now_Angle());
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
        motor_GIM6010_L_1.TIM_100ms_Alive_PeriodElapsedCallback();
        motor_GIM6010_L_2.TIM_100ms_Alive_PeriodElapsedCallback();
        motor_GIM6010_R_1.TIM_100ms_Alive_PeriodElapsedCallback();
        motor_GIM6010_R_2.TIM_100ms_Alive_PeriodElapsedCallback();
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


    while(1)
    {
        

        if(debugflag == 1)
        {
            
            
            
            debugflag = 0;
        }
        vTaskDelay(100);
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
        motor_GIM6010_R_1.CAN_RxCpltCallback(Header.Identifier);
        break;
    }
    case 0x49:
    case 0x5C:
    {
        motor_GIM6010_L_1.CAN_RxCpltCallback(Header.Identifier);
        break;
    }
    case 0x69:
    case 0x7C:
    {
        motor_GIM6010_L_2.CAN_RxCpltCallback(Header.Identifier);
        break;
    }
    case 0x89:
    case 0x9C:
    {
        motor_GIM6010_R_2.CAN_RxCpltCallback(Header.Identifier);
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
