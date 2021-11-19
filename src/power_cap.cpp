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
#define COMMAND_LEN 		3
#define SMU_INIT_WAIT       180


constexpr auto POWER_SERVICE = "xyz.openbmc_project.Settings";
std::string POWER_PATH ="/xyz/openbmc_project/control/host0/power_cap";
constexpr auto POWER_INTERFACE = "xyz.openbmc_project.Control.Power.Cap";
constexpr auto POWER_CAP_STR = "PowerCap";
constexpr auto POWER_CAP_ENABLE_STR = "PowerCapEnable";
constexpr auto MAPPER_BUSNAME = "xyz.openbmc_project.ObjectMapper";
constexpr auto MAPPER_PATH = "/xyz/openbmc_project/object_mapper";
constexpr auto MAPPER_INTERFACE = "xyz.openbmc_project.ObjectMapper";

PowerCapDataHolder* PowerCapDataHolder::instance = 0;

bool PowerCap::do_power_capping() {

    oob_status_t ret;
    uint32_t p0_read_power = 0;
	uint32_t p1_read_power = 0;
    uint32_t p0_i2c_bus = 2;
    uint32_t p0_i2c_addr = 60;
    uint32_t p1_i2c_bus = 3;
    uint32_t p1_i2c_addr = 56;
    uint32_t board_id;	
	uint32_t pow;	
	phosphor::logging::log<phosphor::logging::level::INFO>(
                    "Applying power cap value ");

	if(PowerCap::getPlatformID() == false)
	{
		phosphor::logging::log<phosphor::logging::level::INFO>(
			"Couldnt find the board id of the platform");
		return false;
	}
	pow = PowerCapData * 1000;

	//PO Power Cap Value Update
    ret = read_socket_power_limit(p0_i2c_bus, p0_i2c_addr, &p0_read_power);
	if(ret == OOB_SUCCESS)
	 {
		phosphor::logging::log<phosphor::logging::level::INFO>(
			"Initial Power Cap Value P0 ",phosphor::logging::entry(
			"Initial Power Cap Value P0 %d",p0_read_power));
	}

    ret = write_socket_power_limit(p0_i2c_bus, p0_i2c_addr, pow);
    if (ret != OOB_SUCCESS) 
	{
        phosphor::logging::log<phosphor::logging::level::ERR>("Setting power cap value to P0 failed");
		return false;
    }

    ret = read_socket_power_limit(p0_i2c_bus, p0_i2c_addr, &p0_read_power);
    if(ret == OOB_SUCCESS) 
	{
        phosphor::logging::log<phosphor::logging::level::INFO>(
            "Updated Power Cap Value P0 ",phosphor::logging::entry(
            "Updated Power Cap Value P0 %d",p0_read_power));
    }

	if(PowerCap::BoardName.compare("Quartz") == 0 )
	{
		//P1 Power Cap Value Update
    	ret = read_socket_power_limit(p1_i2c_bus, p1_i2c_addr, &p1_read_power);
    	if(ret == OOB_SUCCESS)
	 	{
        	phosphor::logging::log<phosphor::logging::level::INFO>(
            	"Initial Power Cap Value P1 ",phosphor::logging::entry(
            	"Initial Power Cap Value P1 %d",p1_read_power));
    	}
    	ret = write_socket_power_limit(p1_i2c_bus, p1_i2c_addr, pow);
    	if (ret != OOB_SUCCESS) 
		{
        	phosphor::logging::log<phosphor::logging::level::ERR>("Setting power cap value to P1 failed");
			return false;
    	}

    	ret = read_socket_power_limit(p1_i2c_bus, p1_i2c_addr, &p1_read_power);
    	if(ret == OOB_SUCCESS) 
		{
        	phosphor::logging::log<phosphor::logging::level::INFO>(
            	"Updated Power Cap Value P1 ",phosphor::logging::entry(
            	"Updated Power Cap Value P1 %d",p1_read_power));
		}
	}
	return true;
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

void PowerCap::apply_power_capping()
{
	std::cout <<"Applying power cap value after AC cycle " << std::endl;
	get_power_cap_enable_data();

	if(PowerCapEnableData == true)
	{
		get_power_cap_data();
		sleep(SMU_INIT_WAIT);
		do_power_capping();		
	}

}

void PowerCap::get_power_cap_data() 
{
	std::string settingManager = getService(bus, POWER_PATH.c_str() , POWER_INTERFACE);

    PowerCapData = getProperty<uint32_t>(bus, settingManager.c_str(), POWER_PATH.c_str(),
                                           POWER_INTERFACE, POWER_CAP_STR) ;
}

void PowerCap::get_power_cap_enable_data()
{
	std::string settingManager = getService(bus, POWER_PATH.c_str() , POWER_INTERFACE);
    PowerCapEnableData = getProperty<bool>(bus, settingManager.c_str(), POWER_PATH.c_str(),
                                           POWER_INTERFACE, POWER_CAP_ENABLE_STR) ;
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
}

