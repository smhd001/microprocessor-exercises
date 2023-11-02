#include "packet.h"
#include "program.h"

#define CRC_DIVISOR 0x19A

static packet p;
extern UART_HandleTypeDef huart2;
extern Config conf;

int myVar;

// reentrant ~ thread safe
unsigned char crc_calculator(unsigned  char *data);

static RxState rxState = RX_IDLE;

void stateMachine(uint8_t gotByte){
    static int a = 1;
    static int d = 0;
    a++;
    switch (rxState) {
        case RX_IDLE:
            if(gotByte == PREAMBLE){
                rxState = READ_SRC;
            } else
                return;
            break;
        case READ_SRC:
            p.src = gotByte;
            rxState = READ_DST;
            break;
        case READ_DST:
        	p.dst = gotByte;
        	rxState = DATA_LEN;
            break;
        case DATA_LEN:
        	p.len = gotByte;
        	rxState = DATA;
            break;
        case DATA:
			p.data[d] = gotByte;
        	if (d >= p.len)
        	{
        		d = 0;
        		rxState = CRC_B;
        	}else
        	{
        		d++;
        	}
        	break;
        case CRC_B:
        	p.crc = gotByte;
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
        	if(1)//check crc
        	{
        		if (p.data[0] == 0x11 )
        		{
        			conf.buzzer_type = p.data[1];
        			conf.buzzer_min = p.data[2];
        			conf.buzzer_max = p.data[3];
        		}
        		if (p.data[0] == 0x41)
        		{
        			conf.temperature_max = p.data[1];
        		}
        		if (p.data[0] == 0x42)
        		{
        			conf.light_max = p.data[1];
        		}
        	}
        	rxState = RX_IDLE;
        default:
            break;
    }
}
void send_data(int t,int l)
{
	 unsigned  char data[100];

	data[0] = 0xAA;
	HAL_UART_Transmit(&huart2, data, 1, 100);//preamble
	data[1] = 0;
	HAL_UART_Transmit(&huart2, data + 1, 1, 100);//dest
	data[2] = 0;
	HAL_UART_Transmit(&huart2, data + 2, 1, 100);//src
	data[3] = 2;
	HAL_UART_Transmit(&huart2, data + 3, 1, 100);//data len
	data[4] = 0x21;
	HAL_UART_Transmit(&huart2, data + 4, 1, 100);//control
	data[5] = (char) l;
	HAL_UART_Transmit(&huart2, data + 5, 1, 100);//light
	data[6] = 0;
	HAL_UART_Transmit(&huart2, data + 6, 1, 100);//crc

	data[0] = 0xAA;
	HAL_UART_Transmit(&huart2, data, 1, 100);//preamble
	data[1] = 0;
	HAL_UART_Transmit(&huart2, data + 1, 1, 100);//dest
	data[2] = 0;
	HAL_UART_Transmit(&huart2, data + 2, 1, 100);//src
	data[3] = 2;
	HAL_UART_Transmit(&huart2, data + 3, 1, 100);//data len
	data[4] = 0x22;
	HAL_UART_Transmit(&huart2, data + 4, 1, 100);//control
	data[5] = (unsigned  char) t;
	HAL_UART_Transmit(&huart2, data + 5, 1, 100);//temperature
	data[6] = crc_calculator(data);
	HAL_UART_Transmit(&huart2, data + 6, 1, 100);//crc

}


// Function to calculate CRC for a buffer of data
unsigned char crc_calculator(unsigned  char *data)
{
    unsigned char crc = 0x00;
    unsigned char extract;
    unsigned char sum;
    unsigned char len = data[3] + 4;

    // Calculate CRC for all bytes in buffer
    for (unsigned char i = 1; i < len; i++)
    {
        extract = data[i];
        for (unsigned char tempI = 8; tempI; tempI--)
        {
            sum = (crc ^ extract) & 0x01;
            crc >>= 1;
            if (sum)
            {
                crc ^= CRC_DIVISOR;
            }
            extract >>= 1;
        }
    }

    // Return calculated CRC
    return crc;
}

// Function to check if a buffer of data has a valid CRC
int crc_check(unsigned  char *data, unsigned  char expected_crc)
{
    unsigned  char crc = 0x00;
    unsigned  char extract;
    unsigned  char sum;
    unsigned char len = data[3] + 4;
    // Calculate CRC for all bytes in buffer except last byte (which is the CRC itself)
    for (unsigned char i = 1; i < (len - 1); i++)
    {
        extract = data[i];
        for (unsigned char tempI = 8; tempI; tempI--)
        {
            sum = (crc ^ extract) & 0x01;
            crc >>= 1;
            if (sum)
            {
                crc ^= CRC_DIVISOR;
            }
            extract >>= 1;
        }
    }

    // Compare calculated CRC with expected CRC
    return (crc == expected_crc);
}
/*

	data[0] = 0xAA;
	HAL_UART_Transmit(&huart2, data, 1, 100);//preamble
	data[1] = 0;
	HAL_UART_Transmit(&huart2, data, 1, 100);//dest
	data[2] = 0;
	HAL_UART_Transmit(&huart2, data, 1, 100);//src
	data[3] = 2;
	HAL_UART_Transmit(&huart2, data, 1, 100);//data len
	data[4] = 0x21;
	HAL_UART_Transmit(&huart2, data, 1, 100);//control
	data[5] = (char) l;
	HAL_UART_Transmit(&huart2, data, 1, 100);//light
	data[6] = 0;
	HAL_UART_Transmit(&huart2, data, 1, 100);//crc

	data[0] = 0xAA;
	HAL_UART_Transmit(&huart2, data, 1, 100);//preamble
	data[0] = 0;
	HAL_UART_Transmit(&huart2, data, 1, 100);//dest
	data[0] = 0;
	HAL_UART_Transmit(&huart2, data, 1, 100);//src
	data[0] = 2;
	HAL_UART_Transmit(&huart2, data, 1, 100);//data len
	data[0] = 0x22;
	HAL_UART_Transmit(&huart2, data, 1, 100);//control
	data[0] = (unsigned  char) t;
	HAL_UART_Transmit(&huart2, data, 1, 100);//temperature
	data[0] = crc_calculator(data);
	HAL_UART_Transmit(&huart2, data, 1, 100);//crc

 */
