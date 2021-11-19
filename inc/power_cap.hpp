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

	uint32_t power = 0;

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

                    PowerCapData = std::get<uint32_t>(valPropMap->second);
                    	do_power_capping();
                }
				else 
                {
                    phosphor::logging::log<phosphor::logging::level::INFO>(
                    "PowerCap property not changed");
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
                // Check if it was the Value property that changed.
                auto valPropMap = msgData.find("CurrentHostState");
                {
					uint32_t power_cap_data = 0;
                    if (valPropMap != msgData.end())
                    {
                        StateServer::Host::HostState currentHostState =
                            StateServer::Host::convertHostStateFromString(
                                std::get<std::string>(valPropMap->second));
                        if (currentHostState != StateServer::Host::HostState::Off)
                        {
							apply_power_capping();
                    	}
                	}
            	}
		})
    {
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "PowerCap is created");
    }
    ~PowerCap()
    {
    }

  private:

    sdbusplus::bus::bus &bus;
    sdbusplus::bus::match_t propertiesChangedPowerCapValue;
    sdbusplus::bus::match_t propertiesChangedSignalCurrentHostState;
	bool do_power_capping();
	void apply_power_capping();
    bool  getPlatformID();
	uint32_t PowerCapData;
	bool PowerCapEnableData;
	void get_power_cap_enable_data();
	void get_power_cap_data();
	std::string BoardName;
	template <typename T>
	T getProperty(sdbusplus::bus::bus& bus, const char* service, const char* path,
              const char* interface, const char* propertyName);
	std::string getService(sdbusplus::bus::bus& bus, const char* path,
                       const char* interface);
};
