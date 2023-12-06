#include "power_cap.hpp"

#define CMD_BUFF_LEN    (256)

void apml_unbind()
{
    int rc;
    // Unbind sbtsi and sbrmi drivers
    char cmd[CMD_BUFF_LEN];

    sprintf(cmd, "/usr/bin/set-apml.sh unbind");
    rc = system(cmd);
    if (rc < 0)
        sd_journal_print(LOG_ERR, "Failed to run system cmd: %s \n", cmd);
}

int apml_bind()
{
    int rc;
    // bind sbtsi and sbrmi drivers
    char cmd[CMD_BUFF_LEN];

    sprintf(cmd, "/usr/bin/set-apml.sh bind");
    rc = system(cmd);
    if (rc < 0)
        sd_journal_print(LOG_ERR, "Failed to run system cmd: %s \n", cmd);
    return rc;
}

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

    // Unbind sbtsi and sbrmi drivers
    apml_unbind();
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
