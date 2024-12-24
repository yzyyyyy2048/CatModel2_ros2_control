/******************************************************************************
Copyright (c) 2021, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

 * Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include <CatModel2_v2_interface/gait/ModeSequenceTemplate.h>

#include <ocs2_msgs/msg/mode_schedule.hpp>
#include <vector>

namespace ocs2::legged_robot {
    /** Convert mode sequence template to ROS message */
    inline ocs2_msgs::msg::ModeSchedule createModeSequenceTemplateMsg(
        const ModeSequenceTemplate &ModeSequenceTemplate) {
        ocs2_msgs::msg::ModeSchedule modeScheduleMsg;
        modeScheduleMsg.event_times.assign(
            ModeSequenceTemplate.switchingTimes.begin(),
            ModeSequenceTemplate.switchingTimes.end());
        modeScheduleMsg.mode_sequence.assign(
            ModeSequenceTemplate.modeSequence.begin(),
            ModeSequenceTemplate.modeSequence.end());
        return modeScheduleMsg;
    }

    /** Convert ROS message to mode sequence template */
    inline ModeSequenceTemplate readModeSequenceTemplateMsg(
        const ocs2_msgs::msg::ModeSchedule &modeScheduleMsg) {
        std::vector switchingTimes(modeScheduleMsg.event_times.begin(),
                                             modeScheduleMsg.event_times.end());
        std::vector<size_t> modeSequence(modeScheduleMsg.mode_sequence.begin(),
                                         modeScheduleMsg.mode_sequence.end());
        return {switchingTimes, modeSequence};
    }
}
