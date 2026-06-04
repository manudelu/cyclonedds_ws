#include <iostream>
#include <thread>
#include <chrono>
#include <dds/dds.hpp>
#include "dds/dds.h"
#include "Trigger.hpp"

int DOMAIN_ID {42};

int main(int argc, char** argv) {
    dds::domain::DomainParticipant dp(DOMAIN_ID);

    dds::topic::Topic<std_srvs::srv::dds_::Trigger_Request_>
        req_topic(dp, "rq/advrf/spot/startRequest");
    dds::topic::Topic<std_srvs::srv::dds_::Trigger_Response_>
        rep_topic(dp, "rr/advrf/spot/startReply");

    dds::sub::DataReader<std_srvs::srv::dds_::Trigger_Request_> reader(
        dds::sub::Subscriber(dp), req_topic,
        dds::sub::qos::DataReaderQos()
            << dds::core::policy::Reliability::Reliable()
            << dds::core::policy::History::KeepLast(10)
    );
    dds::pub::DataWriter<std_srvs::srv::dds_::Trigger_Response_> writer(
        dds::pub::Publisher(dp), rep_topic,
        dds::pub::qos::DataWriterQos()
            << dds::core::policy::Reliability::Reliable()
            << dds::core::policy::History::KeepLast(10)
    );

    dds_entity_t c_reader = reader.delegate().get()->get_ddsc_entity();
    dds_entity_t c_writer = writer.delegate().get()->get_ddsc_entity();

    std::cout << "[Service] Ready.\n";

    std_srvs::srv::dds_::Trigger_Request_ request{};
    void* raw[1] = { &request };
    dds_sample_info_t info[1];

    while (true) {
        int n = dds_take(c_reader, raw, info, 1, 1);
        for (int i = 0; i < n; ++i) {
            if (!info[i].valid_data) continue;

            std::cout << "[Service] Request received.\n";

            std_srvs::srv::dds_::Trigger_Response_ reply{};
            reply.success(true);
            reply.message("OK");

            dds_write(c_writer, &reply);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}