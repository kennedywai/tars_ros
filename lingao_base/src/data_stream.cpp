/*
 *  Serial Port Data encoder
 */

#include "data_stream.h"
#include "Serial_Async.h"

#include <unistd.h>
#include <chrono>

Data_Stream::Data_Stream(Transmission* _trans)
{
    Receive_State = DATA_RECEIVE_FIND_HEAD1;
    trans = _trans;
    trans->setReadCallback(std::bind(&Data_Stream::msg_ReceiveCallBack, this, std::placeholders::_1));

    getMessageBlockingID = MSG_ID_NULL;
}

void Data_Stream::msg_ReceiveCallBack(size_t len)
{
    vecBuff readBuff = trans->readData();
    if(readBuff.empty() == false)
    {
        std::lock_guard<std::mutex> lock(receive_mutex_);
        rxdata_parsing(readBuff);
    }
}

// 搜索头
bool Data_Stream::search_Head(unsigned char data)
{
    switch (Receive_State) {
    case DATA_RECEIVE_FIND_HEAD1:
        if(data == HEAD_FLAG1)
        {
            Receive_State = DATA_RECEIVE_FIND_HEAD2;
            return true;
        }
        break;

    case DATA_RECEIVE_FIND_HEAD2:
        if(data == HEAD_FLAG2)
        {
            Receive_State = DATA_RECEIVE_LENGHT;
            rx_Head = MessageHead_st();
            return true;
        }
        else
            Receive_State = DATA_RECEIVE_FIND_HEAD1;
        break;

    case DATA_RECEIVE_LENGHT:
        Receive_State = DATA_RECEIVE_MSG_ID;
        rx_Head.data_length = data;
        return true;
        break;

    case DATA_RECEIVE_MSG_ID:
        Receive_State = DATA_RECEIVE_DATA;
        rx_Head.msg_id = data;
        return true;
        break;

    default:
        break;
    }
    return false;
}

// 接收数据解析
void Data_Stream::rxdata_parsing(vecBuff newdata)
{

    read_buffer.insert(read_buffer.end(), newdata.begin(), newdata.end());

    while (true) {
        if(Receive_State < DATA_RECEIVE_DATA)
        {
            if(read_buffer.empty() == false)
            {
                //DEBUG
                // printf("buff:  ");
                // for(auto aa : read_buffer)
                //         printf("%02x ", (unsigned char)aa);
                // std::cout << std::endl;

                unsigned int byte_pos=0;
                if(Receive_State == DATA_RECEIVE_LENGHT) byte_pos =2;
                if(Receive_State == DATA_RECEIVE_MSG_ID)  byte_pos =3;

                for(; byte_pos<read_buffer.size(); byte_pos++)
                {
                    search_Head((unsigned char)read_buffer[byte_pos]);
                    if(Receive_State == DATA_RECEIVE_LENGHT && byte_pos!=1)
                    {
                        read_buffer.erase(read_buffer.begin(), read_buffer.begin()+byte_pos-1);
                        byte_pos = 1;
                    }
                    else if(Receive_State == DATA_RECEIVE_DATA)break;
                }
                //clean data
                if(Receive_State == DATA_RECEIVE_FIND_HEAD1)
                {
                    read_buffer.clear();
                }
                else if(Receive_State == DATA_RECEIVE_FIND_HEAD2)
                {
                    if (read_buffer.size() > 10)
                    {
                        read_buffer.erase(read_buffer.begin(), read_buffer.end()-1);
                    }
                    Receive_State = DATA_RECEIVE_FIND_HEAD1;
                }
            }
        }
        if(Receive_State == DATA_RECEIVE_DATA)
        {
            unsigned int endData = rx_Head.data_length + sizeof(MessageHead_st)-1;
            if(read_buffer.size() > endData)
            {
                unsigned char check = 0;
                for(unsigned int i=0; i< endData; i++)
                {
                    check += read_buffer[i];
                }
                // 解析完成
                if(read_buffer[endData] == check)
                {
                    MessageFormat_st msgData(rx_Head);
                    msgData.head.data_length -= 1;
                    memcpy(&msgData.data, &read_buffer[4], rx_Head.data_length-1);
                    data_undecode(msgData);

                    read_buffer.erase(read_buffer.begin(), read_buffer.begin()+endData+1);
                }
                else 
                    read_buffer.erase(read_buffer.begin(), read_buffer.begin()+2);

                Receive_State = DATA_RECEIVE_FIND_HEAD1;
            }
            else return;
        }
        if(Receive_State == DATA_RECEIVE_FIND_HEAD1 && read_buffer.size() > sizeof (MessageHead_st));
        else return;
    }
}

void Data_Stream::data_undecode(MessageFormat_st msgData)
{
    bool check = false;
    std::lock_guard<std::mutex> lock(getData_mutex_);
    switch ((Message_Id_Enum)msgData.head.msg_id)
    {
    case MSG_ID_SET_VELOCITY:
        check = true;
        break;
    case MSG_ID_GET_VELOCITY:
        if(sizeof(Data_Format_Liner) == msgData.head.data_length)
        {
            rxData_liner.EndianSwapSet(msgData.data);
            check = true;
        }
        break;

    case MSG_ID_GET_VOLTAGE:
        if(sizeof(Data_Format_BAT) == msgData.head.data_length)
        {
            rxData_battery.EndianSwapSet(msgData.data);
            check = true;
        }
        break;

    case MSG_ID_GET_IMU:
        if(sizeof(Data_Format_IMU) == msgData.head.data_length)
        {
            rxDdata_imu.EndianSwapSet(msgData.data);
            // memcpy(&rxDdata_imu, &msgData.data, msgData.head.data_length);
            check = true;
        }
        break;

    case MSG_ID_GET_RC:
        if(sizeof(Data_Format_RC) == msgData.head.data_length)
        {
            rxData_rc.EndianSwapSet(msgData.data);
            check = true;
        }
        break;

    case MSG_ID_GET_VER:
        if (sizeof(Data_Format_VER) == msgData.head.data_length)
        {
            version.EndianSwapSet(msgData.data);
        }
        else  verError = true;

        check = true;
        break;
        
    default:
        std::cout << "[ WARN ] unknow Received data" << std::endl;
        return;
    }
    if(check == false)
    {
        std::cout << "[ WARN ] Received data format error" << std::endl;
        return;
    }
    if(getMessageBlockingID == msgData.head.msg_id)
    {
        getMessageBlockingID = MSG_ID_NULL;
        cv.notify_all();
    }
}

bool Data_Stream::version_detection(void)
{
    for (size_t i = 0; i < 4; i++)
    {
        verError = false;
        if (get_Message(MSG_ID_GET_VER, 300))
        {
            if (verError == false && (version.protoVer >= LA_PROTO_VER_0220 ))
            {
                return true;
            }
            else 
            {
                return false;
            }
        }
    }
    version.equipmentIdentity = 0;
    version.protoVer = 0;
    return false;
    
}


bool Data_Stream::get_Message(Message_Id_Enum msgId, int timeoutMs)
{
    bool isSend = false;
    switch (msgId)
    {
    case MSG_ID_GET_VER:
    case MSG_ID_GET_IMU:
    case MSG_ID_GET_RC:
    case MSG_ID_GET_VELOCITY:
    case MSG_ID_GET_VOLTAGE:
        isSend = msg_Transmit(msgId);
        if (!isSend)return false;
        getMessageBlockingID = msgId;
        break;

    case MSG_ID_SET_VELOCITY:
        getMessageBlockingID = msgId;
        break;
    
    default:
        return false;
        break;
    }
    auto const timeout= std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    std::unique_lock<std::mutex> lck(m);

    while (getMessageBlockingID != MSG_ID_NULL)
    {
        if(cv.wait_until(lck,timeout) == std::cv_status::timeout)
            return false;
    }
    return true;
}

void Data_Stream::update_liner_speed(Data_Format_Liner linertx)
{
    msg_Transmit(MSG_ID_SET_VELOCITY, (unsigned char*)&linertx, sizeof(Data_Format_Liner));
    get_Message(MSG_ID_SET_VELOCITY);
}

bool Data_Stream::msg_Transmit(Message_Id_Enum msgId)
{
    MessageFormat_st msgf(msgId);
    return msg_Transmit(msgf);
}

bool Data_Stream::msg_Transmit(Message_Id_Enum msgId, unsigned char* tdata, int size)
{
    MessageFormat_st msgf(msgId, tdata , size);
    return msg_Transmit(msgf);
}

bool Data_Stream::msg_Transmit(MessageFormat_st msgf)
{
    if(trans == 0)return false;

    vecBuff data((unsigned char*)&msgf, (unsigned char*)&msgf+ 4+ msgf.head.data_length);
    trans->writeData(data);

            //DEBUG
        //     printf("send:  ");
        // for(auto aa : data)
        //         printf("%02x ", (unsigned char)aa);
        // std::cout << std::endl;

    return true;
}

