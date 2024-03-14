#include <ros/ros.h>
#include <dynamixel_sdk.h>

#define PORT_NAME "/dev/ttyUSB0"
#define PROTOCOL_VERSION 2.0
#define ADDR_LED 65
#define BAUDRATE 57600
#define DXL_ID 1

dynamixel::PortHandler *myPortHandler;
dynamixel::PacketHandler *myPacketHandler;

void mostrarValor(uint8_t *dato, size_t size)
{
    std::string valor = "";
    for (size_t i = 0; i < size; ++i)
    {
        valor += std::to_string(dato[i]);
    }

    std::cout<<valor<<std::endl;
    
}

bool switchLED()
{
    int dxl_comm_result = 0;
    uint8_t dxl_error = 0;
    uint8_t *dato = new uint8_t[1];
    

    myPacketHandler->read1ByteTxRx(myPortHandler, DXL_ID, ADDR_LED, dato, &dxl_error);
    if(dato[0] == 0)
    {
        dxl_comm_result = myPacketHandler->write1ByteTxRx(myPortHandler,DXL_ID, ADDR_LED, 1, &dxl_error);
    } else if(dato[0] == 1)
    {
        dxl_comm_result = myPacketHandler->write1ByteTxRx(myPortHandler,DXL_ID, ADDR_LED, 0, &dxl_error);
    }

    if (dxl_comm_result != COMM_SUCCESS)    
    {
        ROS_ERROR("Failed to turn on the LED for Dynamixel ID %d", DXL_ID);
        ROS_ERROR("Error code: %d", dxl_error);
        return false;

    } else
    {
        // Leer el valor del registro del LED
        dxl_comm_result = myPacketHandler->read1ByteTxRx(myPortHandler, DXL_ID, ADDR_LED, dato, &dxl_error);
        
        if (dxl_comm_result != COMM_SUCCESS)
        {
            ROS_ERROR("Failed to read from the LED for Dynamixel ID %d", DXL_ID);
            ROS_ERROR("Error code: %d", dxl_error);
            delete[] dato;
            return false;
        }

        mostrarValor(dato, sizeof(dato));  // Pasar el puntero al byte leído

        delete[] dato; // Liberar memoria después de usarla
        return true;
    }
}

bool openUSB()
{
    if (!myPortHandler->openPort()) 
    {
        ROS_ERROR("Failed to open the port!");
        return false;
    }

    if (!myPortHandler->setBaudRate(BAUDRATE))  
    {
        ROS_ERROR("Failed to set the baudrate!");
        return false;
    }

    return true;
}

int main()
{
    myPortHandler = dynamixel::PortHandler::getPortHandler(PORT_NAME);
    myPacketHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);



    if(!openUSB())
    {
        std::cout<<"ERROR OPENING THE PORT"<<std::endl;
    } else 
    {
        std::cout<<"SUCCESS OPENING THE PORT"<<std::endl;
    }

    if(switchLED())
    {
        std::cout<<"LED STATE SWITCHED"<<std::endl;
    } else 
    {
        std::cout<<"FAIL SWITCHING THE LED STATE"<<std::endl;
    }


}
