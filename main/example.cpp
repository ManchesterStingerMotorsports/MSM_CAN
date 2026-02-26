#include "MSM_CAN.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static volatile bool got_msg1 = false;
static volatile bool got_msg2 = false;
static volatile bool got_msg3 = false;

static uint8_t last_msg1[8];
static uint8_t last_msg2[8];
static uint8_t last_msg3[8];

static void can_callback(uint16_t id,                                                   //example callback function you want to run when you receive a frame you are subscribed to
                         const uint8_t data[8],                                         //for simplicity all ids share the same callback in this implementation but 
                         uint32_t timestamp)                                            //if you wanted to you could have a unique callback for each ID
{
    if (id == 0x200)
    {
        for (int i = 0; i < 8; i++) last_msg1[i] = data[i];
        got_msg1 = true;
    }

    if (id == 0x201)
    {
        for (int i = 0; i < 8; i++) last_msg2[i] = data[i];
        got_msg2 = true;
    }

    if (id == 0x202)
    {
        for (int i = 0; i < 8; i++) last_msg3[i] = data[i];
        got_msg3 = true;
    }
}

extern "C" void app_main(void)                                                          //main function (entry point for program)
{
    MSM_CAN::set_hardware_filters(0x200, 0x202);                                        //Set range of IDs you want to listen to 

    MSM_CAN::init(GPIO_NUM_5, GPIO_NUM_4);                                              //initiate CAN on your GPIO pins

    MSM_CAN::subscribe(0x200, can_callback);                                            //subscribe to the CAN IDs you want to listen to.  
    MSM_CAN::subscribe(0x201, can_callback);
    MSM_CAN::subscribe(0x202, can_callback);

    while (!(got_msg1 && got_msg2 && got_msg3))                                         //in this particular implementation we wait until we receive all messages THEN we formulate a reply
    {                                                                          
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    uint8_t tx_data[8];                                                                 //declare data struct
    MSM_CAN::clear_payload(tx_data);

    MSM_CAN::pack_u16(tx_data, 0, (uint16_t)((last_msg1[0] << 8) | last_msg1[1]));      //using helper packer functions load the tx data 
    MSM_CAN::pack_u16(tx_data, 2, (uint16_t)((last_msg2[0] << 8) | last_msg2[1]));
    MSM_CAN::pack_u16(tx_data, 4, (uint16_t)((last_msg3[0] << 8) | last_msg3[1]));      
    
    //note there is space to pack 1 more u16 in, but in this implementation its left empty (we set them to 0 earlier using MSM_CAN::clear_payload)                                                                            
    
    MSM_CAN::send_msg(0x500, tx_data);                                                  //send out message

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}