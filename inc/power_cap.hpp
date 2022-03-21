#include <fcntl.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <phosphor-logging/elog-errors.hpp>
#include <xyz/openbmc_project/Collection/DeleteAll/server.hpp>
#include <xyz/openbmc_project/Common/error.hpp>
#include <xyz/openbmc_project/Control/Power/Cap/server.hpp>
#include <xyz/openbmc_project/State/Host/server.hpp>

const static constexpr char *PowerCapName =
    "PowerCap";
const static constexpr char *PowerCapEnableName =
    "PowerCapEnable";

class PowerCapDataHolder
{
    static PowerCapDataHolder *instance;

    PowerCapDataHolder()
    {
    }

  public:
    static PowerCapDataHolder *getInstance()
    {
        if (!instance)
            instance = new PowerCapDataHolder;
        return instance;
    }

    const static constexpr char *PropertiesIntf =
        "org.freedesktop.DBus.Properties";
    const static constexpr char *HostStatePathPrefix =
        "/xyz/openbmc_project/state/host0";
};

struct EventDeleter
{
    void operator()(sd_event *event) const
    {
        event = sd_event_unref(event);
    }
};

using EventPtr = std::unique_ptr<sd_event, EventDeleter>;
namespace StateServer = sdbusplus::xyz::openbmc_project::State::server;

struct PowerCap
{
    PowerCapDataHolder *powercapDataHolderObj =
        powercapDataHolderObj->getInstance();

    PowerCap(sdbusplus::bus::bus &bus, const char *path, EventPtr &event) :
        bus(bus),
        propertiesChangedPowerCapValue(
            bus,
            sdbusplus::bus::match::rules::type::signal() +
                sdbusplus::bus::match::rules::member("PropertiesChanged") +
                sdbusplus::bus::match::rules::path(
                   "/xyz/openbmc_project/control/host0/power_cap") +
                sdbusplus::bus::match::rules::argN(0, "xyz.openbmc_project.Control.Power.Cap") +
                sdbusplus::bus::match::rules::interface(
                    powercapDataHolderObj->PropertiesIntf),
            [this](sdbusplus::message::message &msg) {
                std::string objectName;
                std::map<std::string, std::variant<uint32_t,bool>> msgData;
                msg.read(objectName, msgData);
                // Check if it was the PowerCap property that changed.
                auto valPropMap = msgData.find("PowerCap");
                if (valPropMap != msgData.end())
                {
                    phosphor::logging::log<phosphor::logging::level::INFO>(
                    "PowerCap property changed");

                    userPCapLimit = std::get<uint32_t>(valPropMap->second);
                    //do_power_capping();
                }
            }),
        propertiesChangedSignalCurrentHostState(
            bus,
            sdbusplus::bus::match::rules::type::signal() +
                sdbusplus::bus::match::rules::member("PropertiesChanged") +
                sdbusplus::bus::match::rules::path(
                    powercapDataHolderObj->HostStatePathPrefix)  +
                sdbusplus::bus::match::rules::interface(
                    powercapDataHolderObj->PropertiesIntf),
            [this](sdbusplus::message::message &msg) {
                std::string objectName;
                std::map<std::string, std::variant<std::string>> msgData;
                msg.read(objectName, msgData);
                // Check if CPU was powered-on
                auto valPropMap = msgData.find("CurrentHostState");
                {
                    if (valPropMap != msgData.end())
                    {
                        StateServer::Host::HostState currentHostState =
                            StateServer::Host::convertHostStateFromString(
                                std::get<std::string>(valPropMap->second));
                        if (currentHostState != StateServer::Host::HostState::Off)
                        {

                            enableAPMLMuxChannel();
                            //onHostPwrChange();
                        }
                    }
                }
        })
    {
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "PowerCap is created");
        //init_power_capping();     // init from BMC stored settings
    }
    ~PowerCap()
    {
    }

  private:

    sdbusplus::bus::bus &bus;
    sdbusplus::bus::match_t propertiesChangedPowerCapValue;
    sdbusplus::bus::match_t propertiesChangedSignalCurrentHostState;
    std::string BoardName;
    unsigned int userPCapLimit;     // user requested limit
    int AppliedPowerCapData;        // actual limit accepted by CPU
    bool PowerCapEnableData;        // is feature enabled

    //power cap functions
    bool get_power_cap_enabled_setting();
    void get_power_cap_limit();
    void set_power_cap_limit(uint32_t pwr_limit);
    void init_power_capping();
    bool do_power_capping();
    void onHostPwrChange();
    int  getGPIOValue(const std::string& name);
    void enableAPMLMuxChannel();

    // oob-lib functions
    bool  getPlatformID();
    uint32_t set_oob_pwr_limit(struct bus_info bus, uint32_t req_pwr_limit);

    // d-bus functions
    template <typename T>
    T getProperty(sdbusplus::bus::bus& bus, const char* service, const char* path,
              const char* interface, const char* propertyName);
    std::string getService(sdbusplus::bus::bus& bus, const char* path,
              const char* interface);
};
