#include "power_cap.hpp"

#include "iomanip"
#include <boost/asio.hpp>
#include <boost/asio/spawn.hpp>
#include <boost/asio/error.hpp>
#include <sdbusplus/asio/connection.hpp>
#include <sdbusplus/asio/property.hpp>
#include <gpiod.hpp>
#include <filesystem>

extern "C" {
#include <unistd.h>
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
#define CPU_MAX_PWR_LIMIT   (1000) //1000 watts, max perf

/* Definition for APML Mux setting */
#define MAX_APML_BUS    0x02
#define IMX3112_MUX     0x70
#define IMX3112_MR46    0x46
#define IMX3112_MR40    0x40  // MUX port sel
#define IMX3112_MR41    0x41  // MUX port rw enable
#define FNAME_LEN       64
#define CMD_BUFF_LEN    256
#define I3C_BUS_NUM     4

constexpr auto ONYX_SLT     = 61;   //0x3D
constexpr auto ONYX_1       = 64;   //0x40
constexpr auto ONYX_2       = 65;   //0x41
constexpr auto ONYX_3       = 66;   //0x42
constexpr auto ONYX_FR4     = 82;   //0x52
constexpr auto QUARTZ_DAP   = 62;   //0x3E
constexpr auto QUARTZ_1     = 67;   //0x43
constexpr auto QUARTZ_2     = 68;   //0x44
constexpr auto QUARTZ_3     = 69;   //0x45
constexpr auto QUARTZ_FR4   = 81;   //0x51
constexpr auto RUBY_1       = 70;   //0x46
constexpr auto RUBY_2       = 71;   //0x47
constexpr auto RUBY_3       = 72;   //0x48
constexpr auto TITANITE_1   = 73;   //0x49
constexpr auto TITANITE_2   = 74;   //0x4A
constexpr auto TITANITE_3   = 75;   //0x4B
constexpr auto TITANITE_4   = 76;   //0x4C
constexpr auto TITANITE_5   = 77;   //0x4D
constexpr auto TITANITE_6   = 78;   //0x4E

const std::string PwrOkName = "MON_POST_COMPLETE";
constexpr auto POWER_SERVICE = "xyz.openbmc_project.Settings";
std::string POWER_PATH ="/xyz/openbmc_project/control/host0/power_cap";
constexpr auto POWER_INTERFACE = "xyz.openbmc_project.Control.Power.Cap";
constexpr auto POWER_CAP_STR = "PowerCap";
constexpr auto POWER_CAP_ENABLE_STR = "PowerCapEnable";
constexpr auto MAPPER_BUSNAME = "xyz.openbmc_project.ObjectMapper";
constexpr auto MAPPER_PATH = "/xyz/openbmc_project/object_mapper";
constexpr auto MAPPER_INTERFACE = "xyz.openbmc_project.ObjectMapper";

PowerCapDataHolder* PowerCapDataHolder::instance = 0;

struct i2c_info p0_info = {4, 0x22400000002, 0};
struct i2c_info p1_info = {5, 0x22400000002, 0};

// Set power limit to CPU using OOB library
uint32_t PowerCap::set_oob_pwr_limit (struct i2c_info bus, uint32_t req_pwr_limit)
{
    oob_status_t ret;
    uint32_t current_pwr_limit;

    ret = read_socket_power_limit(bus, &current_pwr_limit);
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
    ret = write_socket_power_limit(bus, (req_pwr_limit * 1000));
    if (ret != OOB_SUCCESS)
    {
        phosphor::logging::log<phosphor::logging::level::ERR>("Setting power cap value failed");
        return -1;
    }

    // Readback and confirm the max limit accepted by CPU
    // if CPU doesnt support user limit, it returns its default power limit
    ret = read_socket_power_limit(bus, &current_pwr_limit);
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

    if((userPCapLimit == 0) && (AppliedPowerCapData == 0))/* factory defaults */
    {
        userPCapLimit = CPU_MAX_PWR_LIMIT; // Set very high value
                                           // CPU settles at max based on OPN
    }

    /* Do nothing, if new limit is same as old */
    if (AppliedPowerCapData == userPCapLimit)
        return true;

    //P0 Power Cap Value Update
    ret = PowerCap::set_oob_pwr_limit(p0_info, userPCapLimit);

    if ((ret != -1) &&         /* socket P0 set was successful */
        (num_of_proc == 2) )
    {
        //P1 Power Cap Value Update
        ret = PowerCap::set_oob_pwr_limit(p1_info, userPCapLimit);
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
    std::stringstream ss;

    // Setup pipe for reading and execute to get u-boot environment
    // variable board_id.
    pf = popen(COMMAND_BOARD_ID,"r");
    if(pf > 0)
    {
        if (fgets(data, COMMAND_LEN , pf) != NULL)
        {
            ss << std::hex << (std::string)data;
            ss >> board_id;
        }
        pclose(pf);
        if ( board_id > 0 || board_id < 0xFF )
        {
            switch (board_id)
            {
                case ONYX_SLT:
                case ONYX_1 ... ONYX_3:
                case ONYX_FR4:
                case RUBY_1 ... RUBY_3:
                    num_of_proc = 1;
                    break;
                case QUARTZ_DAP:
                case QUARTZ_1 ... QUARTZ_3:
                case QUARTZ_FR4:
                case TITANITE_1 ... TITANITE_6:
                    num_of_proc = 2;
                    break;
                default:
                    num_of_proc = 1;
                    break;
            }//switch
            return true;
        }
     }
    else
    {
        std::cerr << "Failed to open command stream" << std::endl;
    }

    return false;
}

int  PowerCap::getGPIOValue(const std::string& name)
{
    int value;
    gpiod::line gpioLine;

    // Find the GPIO line
    gpioLine = gpiod::find_line(name);
    if (!gpioLine)
    {
        std::cerr << "Can't find line: %s" << name.c_str() << std::endl;
        return -1;
    }
    try
    {
        gpioLine.request({__FUNCTION__, gpiod::line_request::DIRECTION_INPUT});
    }
    catch (std::system_error& exc)
    {
        std::cerr << "Error setting gpio as Input:: %s" << name.c_str() << std::endl;
        return -1;
    }

    try
    {
        value = gpioLine.get_value();
    }
    catch (std::system_error& exc)
    {
        std::cerr << "Error getting gpio value for: %s" << name.c_str() << std::endl;
        return -1;
    }

    return value;
}

void unbindDrivers(int i, int bus)
{
    char cmd[CMD_BUFF_LEN];

    /* Unbind sbtsi driver */
    sprintf(cmd, "echo %d-22400000001 > /sys/bus/i3c/drivers/sbtsi_i3c/unbind", bus);
    system(cmd);

    /* Unbind platform driver */
    sprintf(cmd, "echo 1e7a%d000.i3c%d > /sys/bus/platform/drivers/dw-i3c-master/unbind", (6+i), bus);
    system(cmd);
}

void bindDrivers(int i, int bus)
{
    char cmd[CMD_BUFF_LEN];

    /* Bind platform driver */
    sprintf(cmd, "echo 1e7a%d000.i3c%d > /sys/bus/platform/drivers/dw-i3c-master/bind", (6+i), bus);
    system(cmd);
    sleep(1);
}
void setAPMLMux(int i, int bus)
{
    char cmd[CMD_BUFF_LEN];

    /* Set the Mux */
    sprintf(cmd,"/usr/bin/i3ctransfer -d /dev/i3c-%d-4cc00000000 -w 0x%x,0x00,0x01", bus, IMX3112_MR46);
    system(cmd);
    sprintf(cmd,"/usr/bin/i3ctransfer -d /dev/i3c-%d-4cc00000000 -w 0x%x,0x00,0x40", bus, IMX3112_MR40);
    system(cmd);
    sprintf(cmd,"/usr/bin/i3ctransfer -d /dev/i3c-%d-4cc00000000 -w 0x%x,0x00,0x40", bus, IMX3112_MR41);
    system(cmd);

}

void PowerCap::enableAPMLMuxChannel()
{
    char fname[FNAME_LEN];
//    int num_of_apml_bus = MAX_APML_BUS;
    int retry = 0;
    bool enableAPMLMux = false;

    while (retry < MAX_RETRY)
    {
        sleep(10);
        if (getGPIOValue(PwrOkName) > 0)
        {
            std::cout <<"POST Complete reached - Enable APML Mux" << std::endl;
            enableAPMLMux = true;
            break;
        }
        retry++;
    }

    if (enableAPMLMux == true)
    {
        int bus = I3C_BUS_NUM;
        for (int i=0; i < num_of_proc; i++ )
        {
            sprintf(fname, "/dev/i3c-%d-4cc00000000", bus);
            if ( std::filesystem::exists(fname))
            {
                std::cout << "I3C-"<< bus <<" Mux dev exists" << std::endl;
                /* Unbind drivers */
                unbindDrivers(i, bus);
            }
            /* Bind drivers */
            bindDrivers(i, bus);
            /* Set Mux device */
            setAPMLMux(i, bus);
            /* Unbind drivers */
            unbindDrivers(i, bus);
            /* Bind drivers */
            bindDrivers(i, bus);
            bus++;
        } /* for loop */
    }
    std::cout <<"APML MUX setting sucessful ..." << std::endl;

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
