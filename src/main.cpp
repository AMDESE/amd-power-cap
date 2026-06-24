#include "power_cap.hpp"

struct EventDeleter
{
    void operator()(sd_event* event) const
    {
        event = sd_event_unref(event);
    }
};

using EventPtr = std::unique_ptr<sd_event, EventDeleter>;

int main(int argc, char** argv)
{
    PowerCapDataHolder* powercapDataHolderObj =
        powercapDataHolderObj->getInstance();

    int ret = 0;
    std::string intfName;

    // Host instance to service (systemd template %i). Defaults to host0,
    // which preserves the legacy single-instance 2P behavior.
    int hostInstance = 0;
    if (argc > 1)
    {
        try
        {
            hostInstance = std::stoi(argv[1]);
        }
        catch (const std::exception& e)
        {
            hostInstance = 0;
        }
    }

    phosphor::logging::log<phosphor::logging::level::INFO>(
        "Start power cap service...",
        phosphor::logging::entry("HOST=%d", hostInstance));

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

    PowerCap powerCap{bus, hostInstance};

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
