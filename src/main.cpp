#include "power_cap.hpp"

int main()
{
    PowerCapDataHolder* powercapDataHolderObj =
        powercapDataHolderObj->getInstance();

    int ret = 0;
    std::string intfName;

    phosphor::logging::log<phosphor::logging::level::INFO>(
        "Start power cap service...");

    sd_event* event = nullptr;
    ret = sd_event_default(&event);
    if (ret < 0)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(
            "Error creating a default sd_event handler");
        return ret;
    }
    EventPtr eventP{event};
    event = nullptr;

    sdbusplus::bus::bus bus = sdbusplus::bus::new_default();
    sdbusplus::server::manager_t m{bus, DBUS_OBJECT_NAME};

    intfName = DBUS_INTF_NAME;
    bus.request_name(intfName.c_str());

    PowerCap powerCap{bus, DBUS_OBJECT_NAME, eventP};

    try
    {
        bus.attach_event(eventP.get(), SD_EVENT_PRIORITY_NORMAL);
        ret = sd_event_loop(eventP.get());
        if (ret < 0)
        {
            phosphor::logging::log<phosphor::logging::level::ERR>(
                "Error occurred during the sd_event_loop",
                phosphor::logging::entry("RET=%d", ret));
        }
    }
    catch (std::exception& e)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>(e.what());
        return -1;
    }
    return 0;
}
