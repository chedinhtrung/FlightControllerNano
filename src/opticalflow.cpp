#include "opticalflow.h"

/*
Users can use microlink_decode as their serial port data processing function
The minimum effective distance value is 10 (mm), and 0 indicates that the distance value is not available
optical flow velocity value unit：cm/s@1m
Calculation formula: speed(cm/s) = optical flow velocity * height(m)
*/

OpticalFlow::OpticalFlow(){
    OF_SERIAL.begin(115200);
}

bool OpticalFlow::micolink_decode(uint8_t data)
{
    if(micolink_parse_char(data) == false)
        return 0;
    
    switch(msg.msg_id)
    {
        case MICOLINK_MSG_ID_RANGE_SENSOR:
        {
            memcpy(&payload, msg.payload, msg.len);
            return 1;
        } 

        default:
            return 0;
        }
}

bool micolink_check_sum(MICOLINK_MSG_t* msg)
{
    uint8_t length = msg->len + 6;
    uint8_t temp[MICOLINK_MAX_LEN];
    uint8_t checksum = 0;

    memcpy(temp, msg, length);

    for(uint8_t i=0; i<length; i++)
    {
        checksum += temp[i];
    }

    if(checksum == msg->checksum)
    {return true;}   
    else {
        Serial.println("Bad checksum");
        return false;
    }
        
}

bool OpticalFlow::micolink_parse_char(uint8_t data)
{
    switch(msg.status)
    {
    case 0:    
        if(data == MICOLINK_MSG_HEAD)
        {
            msg.head = data;
            msg.status++;
        }
        break;
        
    case 1:     // device id
        msg.dev_id = data;
        msg.status++;
        break;
    
    case 2:     // system id
        msg.sys_id = data;
        msg.status++;
        break;
    
    case 3:     // message id 
        msg.msg_id = data;
        msg.status++;
        break;
    
    case 4:     // 
        msg.seq = data;
        msg.status++;
        break;
    
    case 5:     // payload length
        msg.len = data;
        if(msg.len == 0)
            msg.status += 2;
        else if(msg.len > MICOLINK_MAX_PAYLOAD_LEN)
            msg.status = 0;
        else
            msg.status++;
        break;
        
    case 6:     // payload receive
        msg.payload[msg.payload_cnt++] = data;
        if(msg.payload_cnt == msg.len)
        {
            msg.payload_cnt = 0;
            msg.status++;
        }
        break;
        
    case 7:     // check sum
        msg.checksum = data;
        msg.status = 0;
        if(micolink_check_sum(&msg))
        {
            return true;
        }
        
    default:
        msg.status = 0;
        msg.payload_cnt = 0;
        break;
    }

    return false;
}
