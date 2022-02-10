#include "power_cap.hpp"

#include "iomanip"
#include <boost/asio.hpp>
#include <boost/asio/spawn.hpp>
#include <boost/asio/error.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/property.hpp>

extern "C" {
#include "linux/i2c-dev.h"
#include "i2c/smbus.h"
#include "esmi_common.h"
#include "esmi_mailbox.h"
#include "esmi_rmi.h"
}

#define COMMAND_BOARD_ID    ("/sbin/fw_printenv -n board_id")
#define COMMAND_LEN         3
#define SMU_INIT_WAIT       180
#define MAX_RETRY           10
#define APML_I2C_BUS_P0     (2)
#define P0_SLV_ADDR         (60)
#define APML_I2C_BUS_P1     (3)
#define P1_SLV_ADDR         (56)
#define CPU_MAX_PWR_LIMIT   (1000) //1000 watts, max perf

constexpr auto POWER_SERVICE = "xyz.openbmc_project.Settings";
std::string POWER_PATH ="/xyz/openbmc_project/control/host0/power_cap";
constexpr auto POWER_INTERFACE = "xyz.openbmc_project.Control.Power.Cap";
constexpr auto POWER_CAP_STR = "PowerCap";
constexpr auto POWER_CAP_ENABLE_STR = "PowerCapEnable";
constexpr auto MAPPER_BUSNAME = "xyz.openbmc_project.ObjectMapper";
constexpr auto MAPPER_PATH = "/xyz/openbmc_project/object_mapper";
constexpr auto MAPPER_INTERFACE = "xyz.openbmc_project.ObjectMapper";

PowerCapDataHolder* PowerCapDataHolder::instance = 0;

// Set power limit to CPU using OOB library
uint32_t PowerCap::set_oob_pwr_limit (uint32_t bus, uint32_t addr, uint32_t req_pwr_limit)
{
    oob_status_t ret;
    uint32_t current_pwr_limit;

    ret = read_socket_power_limit(bus, addr, &current_pwr_limit);
    if((ret == OOB_SUCCESS) && (current_pwr_limit != 0))
    {
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "Initial Power Cap Value ",phosphor::logging::entry(
            "Initial Power Cap Value %d",current_pwr_limit));
    }
    else
    {
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "unable to read power limit");
        return -1;
    }

    /* CPU is already running at requested limit
     * OOB deals in milliwatts only */
    if ((current_pwr_limit/1000) == req_pwr_limit)
    {
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "CPU already operating at requested power limit");
        return req_pwr_limit;
    }

    // Set user supplied limit to CPU (in milliwatts)
    ret = write_socket_power_limit(bus, addr, (req_pwr_limit * 1000));
    if (ret != OOB_SUCCESS)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>("Setting power cap value failed");
        return -1;
    }

    // Readback and confirm the max limit accepted by CPU
    // if CPU doesnt support user limit, it returns its default power limit
    ret = read_socket_power_limit(bus, addr, &current_pwr_limit);
    if((ret == OOB_SUCCESS) && (current_pwr_limit != 0))
    {
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "Updated Power Cap Value ",phosphor::logging::entry(
            "Updated Power Cap Value %d",current_pwr_limit));
        return (current_pwr_limit/1000);
    }
    else
    {
        phosphor::logging::log<phosphor::logging::level::ERR>("Readback power cap value failed");
        return -1;
    }

    return -1;
}

// read stored settings, user requested limit and apply power cap
bool PowerCap::do_power_capping() {

    int ret = -1;

    if((PowerCap::getPlatformID() == false) || 
       ((userPCapLimit == 0) && (AppliedPowerCapData == 0))) /* factory defaults */
    {
        userPCapLimit = CPU_MAX_PWR_LIMIT; // Don't limit, max performance
    }

    /* Do nothing, if new limit is same as old */
    if (AppliedPowerCapData == userPCapLimit)
        return true;

    //P0 Power Cap Value Update
    ret = PowerCap::set_oob_pwr_limit(APML_I2C_BUS_P0, P0_SLV_ADDR, userPCapLimit);

    if ((ret != -1) &&         /* socket P0 set was successful */
        ((PowerCap::BoardName.compare("Quartz") == 0) ||
         (PowerCap::BoardName.compare("Titanite") == 0))
       )
    {
        //P1 Power Cap Value Update
        ret = PowerCap::set_oob_pwr_limit(APML_I2C_BUS_P1, P1_SLV_ADDR, userPCapLimit);
    }

    // update d-bus property if CPU applied a different limit
    // Assume we have a 240W CPU part, but user requests 320W
    // CPU will report 240W since it is the max.
    //
    // We assume both sockets have same OPN
    // TBD: check if 2P config supports different OPNs
    if ((ret > 0) && (ret != userPCapLimit))
        PowerCap::set_power_cap_limit(ret);

    if (ret > 0)
        return true;
    else
        return false;
}

bool PowerCap::getPlatformID()
{
    FILE *pf;
    char data[COMMAND_LEN];

    // Setup pipe for reading and execute to get u-boot environment
    // variable board_id.
    pf = popen(COMMAND_BOARD_ID,"r");

    // Error handling
    if(pf < 0)
    {
        std::cerr << "Unable to get Board ID, errno: " << errno << "message: " << strerror(errno) << "\n";
        return false;
    }

    // Get the data from the process execution
    if (fgets(data, COMMAND_LEN, pf) == NULL)
    {
        std::cerr << "Board ID data is null, errno: " << errno << "message: " << strerror(errno) << "\n";
        return false;
    }

    // the data is now in 'data'
    if (pclose(pf) != 0)
    {
        std::cerr << " Error: Failed to close command stream\n";
        return false;
    }
    std::string board_id(data);
    if((board_id.compare("3D") == 0) || (board_id.compare("40") == 0) || (board_id.compare("41") == 0)
        || (board_id.compare("42") == 0) || (board_id.compare("52") == 0))
    {
        PowerCap::BoardName = "Onyx";
        return true;
    }
    if((board_id.compare("3E") == 0 ) || (board_id.compare("43") == 0) || (board_id.compare("44") ==0)
        || (board_id.compare("45") == 0) || (board_id.compare("51") == 0))
    {
        PowerCap::BoardName = "Quartz";
        return true;
    }
    if((board_id.compare("46")== 0) || (board_id.compare("47") == 0) || (board_id.compare("48") == 0))
    {
        PowerCap::BoardName = "Ruby";
        return true;
    }
    if((board_id.compare("49") == 0 ) || (board_id.compare("4A") == 0) || (board_id.compare("4B") == 0)
        || (board_id.compare("4C") == 0) || (board_id.compare("4D") == 0) || (board_id.compare("4E") == 0))
    {
        PowerCap::BoardName = "Titanite";
        return true;
    }

    return false;
}

// CPU loses the power limit applied after reboot
// re-apply previous value from BMC NV
void PowerCap::onHostPwrChange()
{
    uint32_t retry = 0;
    bool status = false;

    status = get_power_cap_enabled_setting();

    if(status && PowerCapEnableData == true)
    {
        get_power_cap_limit();

        // loop until SMU firmware initalizes
        while((do_power_capping() == false) && (retry < MAX_RETRY))
        {
            sleep(30);
            phosphor::logging::log<phosphor::logging::level::ERR>
                 ("SMU not initialized, retrying...");
            retry++;
        }
    }
    else
    {
        phosphor::logging::log<phosphor::logging::level::ERR>("Power cap not enabled");
    }

}
void PowerCap::init_power_capping()
{
    uint32_t retry = 0;
    bool status = false;

    status = get_power_cap_enabled_setting();

    while((status == false) || (retry < MAX_RETRY))
    {
        sleep(10); //retry in 10s interval till phosphor-settings service loads
        status = get_power_cap_enabled_setting();
        retry++;
    }

    if(status)
    {
        PowerCap::get_power_cap_limit();

        // if host is off when BMC booted, this will do nothing
        // power cap settings will be applied when host power state changes
        PowerCap::do_power_capping();
    }
    else if(retry >= MAX_RETRY)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>("Power cap settings not found");
        exit -1;
    }
}

void PowerCap::get_power_cap_limit()
{
    std::string settingManager = getService(bus, POWER_PATH.c_str() , POWER_INTERFACE);

    AppliedPowerCapData = getProperty<uint32_t>(bus, settingManager.c_str(),
                                           POWER_PATH.c_str(),
                                           POWER_INTERFACE, POWER_CAP_STR) ;
}

bool PowerCap::get_power_cap_enabled_setting()
{
    try
    {
        std::string settingManager = getService(bus, POWER_PATH.c_str() , POWER_INTERFACE);
    if (settingManager.empty())
            return false;

        PowerCapEnableData = getProperty<bool>(bus, settingManager.c_str(), POWER_PATH.c_str(),
                                           POWER_INTERFACE, POWER_CAP_ENABLE_STR) ;
    }
    catch (const sdbusplus::exception::SdBusError& ex)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>("sdbus error");
    }
    return true;
}

template <typename T>
T PowerCap::getProperty(sdbusplus::bus::bus& bus, const char* service, const char* path,
              const char* interface, const char* propertyName)
{
    auto method = bus.new_method_call(service, path,
                                      "org.freedesktop.DBus.Properties", "Get");
    method.append(interface, propertyName);
    std::variant<T> value{};
    try
    {
        auto reply = bus.call(method);
        reply.read(value);
        return std::get<T>(value);
    }
    catch (const sdbusplus::exception::SdBusError& ex)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>("GetProperty call failed");
    }
}

std::string PowerCap::getService(sdbusplus::bus::bus& bus, const char* path,
                       const char* interface)
{
    auto mapper = bus.new_method_call(MAPPER_BUSNAME, MAPPER_PATH,
                                      MAPPER_INTERFACE, "GetObject");

    mapper.append(path, std::vector<std::string>({interface}));
    try
    {
        auto mapperResponseMsg = bus.call(mapper);

        std::vector<std::pair<std::string, std::vector<std::string>>>
            mapperResponse;
        mapperResponseMsg.read(mapperResponse);
        if (mapperResponse.empty())
        {
            phosphor::logging::log<phosphor::logging::level::ERR>("Error reading mapper response");
        }
        if (mapperResponse.size() < 1)
        {
            return "";
        }
        return mapperResponse[0].first;
    }
    catch (const sdbusplus::exception::SdBusError& ex)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>("Mapper call failed");
    }
    return "";
}

void PowerCap::set_power_cap_limit(uint32_t value)
{
    sdbusplus::bus::bus bus = sdbusplus::bus::new_default();
    boost::system::error_code ec;
    boost::asio::io_context io;
    auto conn = std::make_shared<sdbusplus::asio::connection>(io);

    conn->async_method_call(
        [this](boost::system::error_code ec) {
            if (ec)
            {
                phosphor::logging::log<phosphor::logging::level::ERR>
                    ("Failed to set power cap value in dbus interface");
            }
        },
        "xyz.openbmc_project.Settings",
        "/xyz/openbmc_project/control/host0/power_cap",
        "org.freedesktop.DBus.Properties", "Set",
        "xyz.openbmc_project.Control.Power.Cap", "PowerCap",
        std::variant<uint32_t>(value));
}
