#include <fcntl.h>
#include <unistd.h>

#include <phosphor-logging/elog-errors.hpp>

#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>
// #include <xyz/openbmc_project/Collection/DeleteAll/server.hpp>
#include <xyz/openbmc_project/Common/error.hpp>
#include <xyz/openbmc_project/Control/Power/Cap/server.hpp>
#include <xyz/openbmc_project/State/Host/server.hpp>

const static constexpr char* PowerCapName = "PowerCap";
const static constexpr char* PowerCapEnableName = "PowerCapEnable";

// HPAR (hardware partitioning) mode, read from
// xyz.openbmc_project.Settings :: /xyz/openbmc_project/control/HostMode
//   CurrentMode == 0 -> 2P    (single OS spanning both sockets, host0)
//   CurrentMode == 1 -> 2x1P  (two independent CPUs, host1 + host2)
constexpr int HPAR_MODE_2P = 0;
constexpr int HPAR_MODE_2X1P = 1;

class PowerCapDataHolder
{
    static PowerCapDataHolder* instance;

    PowerCapDataHolder() {}

  public:
    static PowerCapDataHolder* getInstance()
    {
        if (!instance)
            instance = new PowerCapDataHolder;
        return instance;
    }

    const static constexpr char* PropertiesIntf =
        "org.freedesktop.DBus.Properties";

    // Build the power_cap settings object path for a given host instance.
    static std::string powerCapObjPath(int hostInstance)
    {
        return "/xyz/openbmc_project/control/host" +
               std::to_string(hostInstance) + "/power_cap";
    }

    // Build the host state object path for a given host instance.
    static std::string hostStateObjPath(int hostInstance)
    {
        return "/xyz/openbmc_project/state/host" +
               std::to_string(hostInstance);
    }
};

namespace StateServer = sdbusplus::xyz::openbmc_project::State::server;

struct PowerCap
{
    PowerCapDataHolder* powercapDataHolderObj =
        powercapDataHolderObj->getInstance();

    // hostInstance selects which partition this daemon instance services:
    //   0 -> host0  (2P mode, drives all present sockets)
    //   1 -> host1  (2x1P mode, drives socket 0 / P0)
    //   2 -> host2  (2x1P mode, drives socket 1 / P1)
    PowerCap(sdbusplus::bus::bus& bus, int hostInstance) :
        bus(bus), hostInstance(hostInstance),
        powerCapPath(PowerCapDataHolder::powerCapObjPath(hostInstance)),
        hostStatePath(PowerCapDataHolder::hostStateObjPath(hostInstance)),
        propertiesChangedPowerCapValue(
            bus,
            sdbusplus::bus::match::rules::type::signal() +
                sdbusplus::bus::match::rules::member("PropertiesChanged") +
                sdbusplus::bus::match::rules::path(powerCapPath) +
                sdbusplus::bus::match::rules::argN(
                    0, "xyz.openbmc_project.Control.Power.Cap") +
                sdbusplus::bus::match::rules::interface(
                    powercapDataHolderObj->PropertiesIntf),
            [this](sdbusplus::message::message& msg) {
                std::string objectName;
                std::map<std::string, std::variant<uint32_t, bool>> msgData;
                msg.read(objectName, msgData);
                // Check if it was the PowerCap property that changed.
                auto valPropMap = msgData.find("PowerCap");
                if (valPropMap != msgData.end())
                {
                    sd_journal_print(LOG_DEBUG, "PowerCap property changed \n");
                    userPCapLimit = std::get<uint32_t>(valPropMap->second);
                    do_power_capping();
                }
            }),
        propertiesChangedSignalCurrentHostState(
            bus,
            sdbusplus::bus::match::rules::type::signal() +
                sdbusplus::bus::match::rules::member("PropertiesChanged") +
                sdbusplus::bus::match::rules::path(hostStatePath) +
                sdbusplus::bus::match::rules::interface(
                    powercapDataHolderObj->PropertiesIntf),
            [this](sdbusplus::message::message& msg) {
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
                        if (currentHostState !=
                            StateServer::Host::HostState::Off)
                        {
                            init_power_capping();
                            onHostPwrChange();
                        }
                    }
                }
            })
    {
        sd_journal_print(LOG_DEBUG, "PowerCap is created for host%d \n",
                         hostInstance);
        get_num_of_proc();
        build_socket_list();
    }
    ~PowerCap() {}

  private:
    sdbusplus::bus::bus& bus;
    int hostInstance;           // 0 (2P/host0), 1 (host1/P0), 2 (host2/P1)
    std::string powerCapPath;   // settings object this instance owns
    std::string hostStatePath;  // host state object this instance watches
    int num_of_proc = 1;
    std::vector<uint8_t> socketList; // APML soc_die_num(s) this instance drives
    sdbusplus::bus::match_t propertiesChangedPowerCapValue;
    sdbusplus::bus::match_t propertiesChangedSignalCurrentHostState;
    unsigned int board_id = 0;
    unsigned int userPCapLimit; // user requested limit
    int AppliedPowerCapData;    // actual limit accepted by CPU
    bool PowerCapEnableData;    // is feature enabled

    // power cap functions
    bool get_power_cap_enabled_setting();
    void get_power_cap_limit();
    void set_power_cap_limit(uint32_t pwr_limit);
    void init_power_capping();
    bool do_power_capping();
    void onHostPwrChange();
    int getGPIOValue(const std::string& name);

    // HPAR / topology helpers
    void build_socket_list();
    int get_hpar_mode();
    bool is_instance_applicable();

    // oob-lib functions
    void get_num_of_proc();
    uint32_t set_oob_pwr_limit(uint8_t bus, uint32_t req_pwr_limit);
    // d-bus functions
    template <typename T>
    T getProperty(sdbusplus::bus::bus& bus, const char* service,
                  const char* path, const char* interface,
                  const char* propertyName);
    std::string getService(sdbusplus::bus::bus& bus, const char* path,
                           const char* interface);
};
