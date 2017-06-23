/*
 Copyright (c) 2017, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <controller_common/can_queue_service.h>
#include <rtt/plugin/ServicePlugin.hpp>
#include <rtt/Service.hpp>
#include <rtt/Logger.hpp>

#include <can_driver/CANDev.h>

namespace sockets_can_queue {

class SocketsCanQueue: public controller_common::CanQueueService {
public:
    explicit SocketsCanQueue(RTT::TaskContext* owner)
            : controller_common::CanQueueService(owner)
    { }

    void initialize(const std::string& dev_name, const std::vector<std::pair<uint32_t, uint32_t > >& filters);
    bool send(uint16_t can_id, uint16_t len, const int8_t *data);
    bool readReply(uint16_t can_id, uint16_t &dlc, int8_t *data);

private:

    std::shared_ptr<CANDev > can_dev_;
};

void SocketsCanQueue::initialize(const std::string& dev_name, const std::vector<std::pair<uint32_t, uint32_t > >& filters)
{
    std::vector<CANDev::FilterElement > filter_vec;
    for (int i = 0; i < filters.size(); ++i) {
        filter_vec.push_back( CANDev::FilterElement(filters[i].first, filters[i].second) );
    }

    can_dev_.reset( new CANDev(dev_name, dev_name, filter_vec) );
}

bool SocketsCanQueue::send(uint16_t can_id, uint16_t len, const int8_t *data) {
    return can_dev_->send(can_id, len, reinterpret_cast<const uint8_t* >(data));
}

bool SocketsCanQueue::readReply(uint16_t can_id, uint16_t &dlc, int8_t *data_out) {
    int data_read;
    data_read = can_dev_->waitForReply(can_id, reinterpret_cast<uint8_t* >(data_out));
    if (data_read == 0) {
        return false;
    }

    dlc = data_read;
    return true;
}

}   // namespace sockets_can_queue

ORO_SERVICE_NAMED_PLUGIN(sockets_can_queue::SocketsCanQueue, "can_queue");

