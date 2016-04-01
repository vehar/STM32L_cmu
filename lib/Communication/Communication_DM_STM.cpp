
#include "Communication_DM_STM.h"
#include "spi.h"

 unsigned char Communication::Packed_START	= 0;
 unsigned char Communication::Packed_CMD	= 0;
 unsigned char Communication::Packed_SIZE	= 0;
 unsigned char Communication::Packed_data	= 0;

 unsigned char Communication::Packed_XOR	= 0;
 unsigned char Communication::Real_XOR		= 0;
 unsigned char Communication::Packed_stop	= 0;

 unsigned char Communication::Rx_buf[SPI_BUFF_SIZE];
 unsigned char Communication::Tx_buf[SPI_BUFF_SIZE];


Communication::Communication()
{
	RxBufClear();
	TxBufClear();
	Spi_hw_init();
}

Communication::~Communication(){}

unsigned char Communication::XorCalc (unsigned char *Arr, int start, int stop)
{
unsigned char result = 0;
	for(int i = start; i<=stop; i++)
	{
		result ^=Arr[i];
	}
return result;
}

void Communication::CmdPack(int cmd)// fill tx buf with cmd and other data
{
TxBufClear();
	Tx_buf[START_POS] = START_BYTE;
	Tx_buf[COMAND_POS] = cmd;//KBD_DATA
	Tx_buf[PACKED_SIZE_POS] = 0x01;
	Tx_buf[DATA_1_POS] = 0x00;
	Tx_buf[DATA_2_POS] = 0x00;
	Tx_buf[5] = XorCalc(Tx_buf,COMAND_POS,4);
	Tx_buf[6] = STOP_1_BYTE;
	Tx_buf[7] = STOP_2_BYTE;
}

void Communication::CmdPack(int cmd, int data_cnt, char* data)// fill tx buf with cmd & cpy-ed data buf
{
TxBufClear();
	Tx_buf[START_POS] = START_BYTE;
	Tx_buf[COMAND_POS] = cmd;//KBD_DATA
	Tx_buf[PACKED_SIZE_POS] = data_cnt;

	memcpy(Tx_buf+DATA_1_POS, data, data_cnt);//cpy data buff into Tx_packed buff

	Tx_buf[PACKED_SIZE_POS+data_cnt+1] = XorCalc(Tx_buf,COMAND_POS,data_cnt+2);//Код комманды ^ Кол-во байт ^ Данные = XOR
	Tx_buf[PACKED_SIZE_POS+data_cnt+2] = STOP_1_BYTE;
	Tx_buf[PACKED_SIZE_POS+data_cnt+3] = STOP_2_BYTE;
}

int Communication::PackedCorrect(unsigned char* Packed)
{
	int result = 0;

	Packed_START	= Packed[START_POS];
	Packed_CMD		= Packed[COMAND_POS];
	Packed_SIZE		= Packed[PACKED_SIZE_POS];
	Packed_XOR		= Packed[DATA_1_POS + Packed_SIZE];

	Real_XOR		= XorCalc(Packed,COMAND_POS,Packed_SIZE+2);

	if(Real_XOR != Packed_XOR)
	return 0;

	if(Packed_START == START_BYTE &&							
	   Packed[DATA_1_POS + Packed_SIZE +1] == STOP_1_BYTE && 
	   Packed[DATA_1_POS + Packed_SIZE +2] == STOP_2_BYTE) //if packed valid - parse
	{
		result = 1;
	}
	else
	{
		//printf("SPI_invalid start %d\n", SPI_Rx_buf[0] ); 
		 RxBufClear();
		 result = 0;
	}
return result;
}

void Communication::DataExchange()//TODO: implement
{
	SPI_exchange(Rx_buf, Tx_buf, SPI_BUFF_SIZE);
}


void Communication::DataTx(uint16_t number)
{
	StartSPI_Rx_Only(number);
}

void Communication::DataRx()
{
	StartSPI_Rx_Only(10);
}

void Communication::RxBufClear(void)
{
	memset(Rx_buf,0,SPI_BUFF_SIZE);
}
void Communication::TxBufClear(void)
{
	memset(Tx_buf,0,SPI_BUFF_SIZE);
}
