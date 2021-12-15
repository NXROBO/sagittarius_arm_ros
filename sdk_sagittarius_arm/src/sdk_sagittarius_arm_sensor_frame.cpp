/* 
 * Software License Agreement (BSD License)
 * Copyright (c) 2021, NXROBO.
 * All rights reserved.
 * Author: litian.zhuang  <litian.zhuang@nxrobo.com>   
 */
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_sensor_frame.h>
#include <sdk_sagittarius_arm/sdk_sagittarius_arm_constants.h>

namespace sdk_sagittarius_arm
{
    CSDKeliLsSensFrame::CSDKeliLsSensFrame()
    {
        mSensDataLength = 0;
        m_pSensData  = NULL;
    }

    CSDKeliLsSensFrame::~CSDKeliLsSensFrame()
    {
        if(m_pSensData != NULL)
            delete m_pSensData;
    }

    uint8_t CSDKeliLsSensFrame::GetFrameHeader()
    {
        return m_pSensData->header;
    }

    uint8_t CSDKeliLsSensFrame::GetCommandId()
    {
        return m_pSensData->cmd_id;
    }

    uint16_t CSDKeliLsSensFrame::GetRangeStart()
    {
        return m_pSensData->range_start;
    }

    uint16_t CSDKeliLsSensFrame::GetRangeEnd()
    {
        return m_pSensData->range_end;
    }

    int  CSDKeliLsSensFrame::GetSensDataCount()
    {
        return m_pSensData->range_end - m_pSensData->range_start + 1;
    }

    uint16_t CSDKeliLsSensFrame::GetSensDataOfIndex(int index)
    {
        if(index < 0 || index > (m_pSensData->range_end - m_pSensData->range_start))
        {
            ROS_ERROR("Fail to get of index %d.", index);
            return 0;
        }

        return m_pSensData->sens_data[index];
    }

    bool CSDKeliLsSensFrame::CheckFrame(char *buff, int length, uint8_t value)
    {
        int i = 0;
        uint8_t result = 0;
        bool checkframe;

        /* Get configure form launch script */
        ros::param::get("~checkframe", checkframe);
        if (checkframe == false)
        {
            /* Disable check frame function, default check true*/
            return true;
        }

        if (buff == NULL || length <= 0)
        {
            printf("CheckFrame: parameter failed\n");
            return false;
        }

        for (i = 0; i < length; i++)
        {
            result += (*buff++);
        }

        if (result == value)
        {
            return true;
        }

        printf("CheckFrame: check failed, length = %d, result = 0x%X, value = 0x%X\n",
               length, result, value);

        return false;
    }

    bool CSDKeliLsSensFrame::InitFromSensBuff(char *buff, int length)
    {

        return true;
    }

    void CSDKeliLsSensFrame::DumpFrameHeader()
    {
        if(m_pSensData == NULL || mSensDataLength == 0)
        {
            return;
        }

        ROS_DEBUG("Frame Header: 0x%02X", this->GetFrameHeader());
        ROS_DEBUG("Command   ID: 0x%02X", this->GetCommandId());
        ROS_DEBUG("Angle  START: 0x%04X", this->GetRangeStart());
        ROS_DEBUG("Angle    END: 0x%04X", this->GetRangeEnd());
    }

    void CSDKeliLsSensFrame::DumpFrameData()
    {
        if(m_pSensData == NULL || mSensDataLength == 0)
        {
            return;
        }

        int dataCount = this->GetSensDataCount();
        ROS_DEBUG("Data   Count: %d", dataCount);

        int idx = 1;
        while(idx <= dataCount)
        {
            printf("%u ", static_cast<unsigned int>(this->GetSensDataOfIndex(idx - 1)));

            idx++;
            if(idx % 48 == 0)
                printf("\n");
        }
        printf("\n");
    }

} /*namespace sdk_sagittarius_arm*/
