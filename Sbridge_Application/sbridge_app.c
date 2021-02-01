#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <linux/ioctl.h>
#include <pthread.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <errno.h>
#include "wilc3000_ble_burst_firmware.h"

struct usart_frame {
	uint8_t header[4];
	uint32_t header_size;
	uint32_t min_size;
	int (*handler)(uint8_t *buffer, uint8_t size);
};

typedef struct {
	uint32_t cmd;
	uint32_t addr;
	uint32_t val;

}uart_cmd_hdr;

typedef struct {
	uint32_t cmd;
	uint32_t addr;
	uint32_t val;
	uint8_t b_buffer[4*1024];
}block_uart_cmd_hdr;


typedef struct {
	uint8_t b_buffer[4*1024];
	uint32_t payload_length;

}t_block_data;

#define min(a,b)	(((a) < (b)) ? (a) : (b))


#define CMD_READ_REG 		_IOR('q', 1, uart_cmd_hdr *)
#define CMD_WRITE_REG 		_IOW('q', 2, uart_cmd_hdr *)
#define CMD_READ_BLOCK_REG 	_IOR('q', 3, uart_cmd_hdr *)
#define CMD_WRITE_BLOCK_REG 	_IOW('q', 4, uart_cmd_hdr *)

char *file_name = "/dev/query";

int set_interface_attribs (int fd, int speed, unsigned char raw);//, int parity);
void set_blocking (int fd, int should_block);
int sendbyte(unsigned int value);
void ble_uart_getchar(int status);

pthread_t thread_id;
FILE *fpr, *fpw;
FILE *fpr_ble, *fpw_ble;
static uint32_t gain_offset;
static uint32_t xo_offset;
static uint32_t find_bank;
int interface_type = 0;
char cmd_uart [15]="/dev/";
char ble_uart [15] = "/dev/";
int ble_res = 0;
int ble_tx = 0;
int mac_address_app=0;
int mac_address_write=0;
int xo_offset_write=0;
uint32_t efuse_offset_1;
uint32_t efuse_offset_2;
// Address to load firmware
#define IRAM_START 0x80000000

// Larger blocks (e.g. 8192) cause hang
#define FIRMWARE_CHUNK_SIZE 4096

 static int     download_count;
 static uint8_t event_buffer[15];
 static uint8_t command_buffer[12];
 static uint8_t * fw_data;
 static uint32_t        fw_size;
 static uint32_t        fw_offset;

enum ioctl_cmds{

	SPI_READ_REG = 0,
	SPI_WRITE_REG,
	SPI_WRITE_BLOCK,
	SPI_READ_BLOCK,
};

static uint8_t checksum_check(uint8_t *buffer, uint8_t size)
{
	uint8_t checksum = 0;
	int i;

	for (i = 0; i < size; i++) {
		checksum ^= buffer[i];
	}

	return checksum;
}

static void buffer_to_cmd_hdr(uint8_t *buffer, uart_cmd_hdr *cmd_hdr)
{
	union {
		uint32_t i;
		char c[4];
	}
	
	bint = {0x01020304};

	if (bint.c[0] == 1) {
		/* Big endian. */
		cmd_hdr->cmd = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) | ((uint32_t)buffer[3] << 0);
		cmd_hdr->addr = ((uint32_t)buffer[4] << 24) | ((uint32_t)buffer[5] << 16) | ((uint32_t)buffer[6] << 8) | ((uint32_t)buffer[7] << 0);
		cmd_hdr->val = ((uint32_t)buffer[8] << 24) | ((uint32_t)buffer[9] << 16) | ((uint32_t)buffer[10] << 8) | ((uint32_t)buffer[11] << 0);
	} else {
		/* Little endian. */
		memcpy(cmd_hdr, buffer, sizeof(uart_cmd_hdr));
	}
}

static void buffer_to_b_cmd_hdr(uint8_t *buffer, block_uart_cmd_hdr *b_cmd_hdr)
{
	union {
		uint32_t i;
		char c[4];
	}
	
	bint = {0x01020304};

	if (bint.c[0] == 1) {
		/* Big endian. */
		b_cmd_hdr->cmd = ((uint32_t)buffer[0] << 24) | ((uint32_t)buffer[1] << 16) | ((uint32_t)buffer[2] << 8) | ((uint32_t)buffer[3] << 0);
		b_cmd_hdr->addr = ((uint32_t)buffer[4] << 24) | ((uint32_t)buffer[5] << 16) | ((uint32_t)buffer[6] << 8) | ((uint32_t)buffer[7] << 0);
		b_cmd_hdr->val = ((uint32_t)buffer[8] << 24) | ((uint32_t)buffer[9] << 16) | ((uint32_t)buffer[10] << 8) | ((uint32_t)buffer[11] << 0);
	} else {
		/* Little endian. */
		memcpy(b_cmd_hdr, buffer, sizeof(uart_cmd_hdr));
	}
}

unsigned char recieved_frame[13];

int sendbyte(unsigned int value)
{
	unsigned char sbuf[4];
	int iter;

	for(iter=3; iter>=0; iter--){
		
		sbuf[iter] = (value >> (8*iter)) & 0xff;
		write(fileno(fpw), sbuf+iter, 1);
	}
}

int send_ble_response(unsigned char value)
{
	unsigned char sbuf[4];
	int iter;

	for(iter=3; iter>=0; iter--){
		
		sbuf[iter] = (value >> (8*iter)) & 0xff;
		write(fileno(fpw), sbuf+iter, 1);
	}
}


static int usart_write_reg_handler(uint8_t *buffer, uint8_t size)
{
	uart_cmd_hdr cmd_hdr;
	int fd, init_buff[0];

	fd = open(file_name, O_RDWR);
	if (fd == -1)
	{
        	perror("query_apps open");
	        return 2;
	}

	buffer_to_cmd_hdr(buffer + 1, &cmd_hdr);

	if (ioctl(fd, CMD_WRITE_REG, &cmd_hdr) == -1)
   	{
        	perror("query_apps ioctl set");
	}
	
	init_buff[0]=0xAC;
	write(fileno(fpw), init_buff, 1);
	close (fd);

	return 0;
}

static int usart_read_reg_with_ret_handler(uint8_t *buffer, uint8_t size)
{
	uart_cmd_hdr cmd_hdr;
	uint32_t *valp, fd , init_buff[0];
	unsigned char snum[33];

	buffer_to_cmd_hdr(buffer + 1, &cmd_hdr);

	fd = open(file_name, O_RDWR);
	if (fd == -1)
	{
        	perror("query_apps open");
	        return 2;
	}

	if (ioctl(fd, CMD_READ_REG, &cmd_hdr) == -1)
   	{
        	perror("query_apps ioctl set");
	}

	init_buff[0]=0xAC;
	write(fileno(fpw), init_buff, 1);

	sendbyte(cmd_hdr.val);
   	close (fd);

	return 0;
}




void write_wilc_reg(uint32_t address, uint32_t value)
{
	uart_cmd_hdr cmd_hdr;
	cmd_hdr.addr=0;
	cmd_hdr.cmd=0;
	cmd_hdr.val=0;
	uint32_t fd;
	fd = open(file_name, O_RDWR);
	if (fd == -1)
	{
	 	perror("query_apps open");	       
	}
	cmd_hdr.addr=address;
	cmd_hdr.val=value;
	 if (ioctl(fd, CMD_WRITE_REG, &cmd_hdr) == -1)
	{
		perror("query_apps ioctl set");
	}
	close (fd);
}




uint32_t read_wilc_reg(uint32_t address)
{
	uart_cmd_hdr cmd_hdr;
	cmd_hdr.addr=0;
	cmd_hdr.cmd=0;
	cmd_hdr.val=0;
	uint32_t fd;
	uint32_t val32=0;
	fd = open(file_name, O_RDWR);
	if (fd == -1)
	{
		perror("query_apps open");
	}
	cmd_hdr.addr=address;
	if (ioctl(fd, CMD_READ_REG, &cmd_hdr) == -1)
	{
		perror("query_apps ioctl set");
	}
	val32 = cmd_hdr.val;
	return val32;
	close (fd);
}

void read_wilc_efuse_with_ret_handler(uint32_t* gain_offset, uint32_t* find_bank)
{
	uint32_t efuse_used=0, valid_efuse_bank=0;
	uint32_t bank=0;
	uint32_t data[24];
	uint32_t efuse_offset0,efuse_offset1,efuse_offset2,efuse_offset3; 

	for (int i = 0; i < 24; i++)
	{
		data[i] = 0 ;
	}

	int bankIdx =0,i;
	printf("\r\nDisplay entire efuse memory content:\r\n");

	for (bankIdx = 0,i=0; bankIdx<6,i<6; bankIdx++,i++) 
	{
		uint32_t bankAddr;
		uint32_t val32;

		bankAddr = (bankIdx<2) ? 0x102c + bankIdx * 32 : 0x1380 + (bankIdx - 2) * 16;

		data[i*4] =read_wilc_reg(bankAddr);						
		printf("%x	",data[i*4]);		
		data[i*4+1] = read_wilc_reg(bankAddr+4);	
		printf("%x	", data[i*4+1]);		
		data[i*4+2]  = read_wilc_reg(bankAddr+8);
		printf("%x	",data[i*4+2]);
		if(bankIdx ==0)
		{
			*gain_offset=data[i*4+2];				
		}
		data[i*4+3]  = read_wilc_reg(bankAddr+12);
		printf("%x 	",data[i*4+3]);

		printf("\r\n");

	}


	for (bankIdx = 0,i=0; bankIdx<6,i<6; bankIdx++,i++) 
	{

		efuse_offset0=data[i*4];
		efuse_offset1=data[i*4+1];
		efuse_offset2=data[i*4+2];
		efuse_offset3=data[i*4+3];

		//checking whether the eFuse bit is set or not
		if(((efuse_offset0 & 0x80000000) >> (7*4))==8)
		{

			++bank;    
			efuse_used=1;
			//check eFuse Invalid Bit
			if(((efuse_offset0 & 0x40000000) >> (7*4)) !=4)
			{ 
				valid_efuse_bank=1;
			}
		}


	}
	if((data[(bank-1) *4]) & 0x01000000)	
	{
		efuse_offset_1 = data[(bank-1) *4];
		efuse_offset_2 = data[((bank-1) *4)+1];	
		printf("Previous oofset1: %x offset2: %x\n",efuse_offset_1,efuse_offset_2);
	}        

	*find_bank=bank;
}



void fn_wReg32_confirm(uint32_t address, uint32_t value)
{ 
	int val32_check = 0;
	for(int i = 0; i <100; i++)
	{

		write_wilc_reg(address,value);
		val32_check = read_wilc_reg(address); 
		if(value == val32_check)
		{
			i=100;
		}
	}
}

int write_efuse_by_bit( int bankIdx, uint32_t* dat, int loop)
{

	uint32_t bankAddr =0;
	uint32_t val32_check = 0;

	bankAddr = (bankIdx < 2 ) ? 0x101c + bankIdx * 32 : 0x1340 + (bankIdx -2) * 16;

	fn_wReg32_confirm(bankAddr, dat[0]);
	printf("BankIdx:%x  Value:%x\n",bankAddr,dat[0]);
	fn_wReg32_confirm(bankAddr + 4, dat[1]);
	printf("BankIdx:%x  Value:%x\n",bankAddr+4,dat[1]);
	fn_wReg32_confirm(bankAddr + 8, dat[2]);
	printf("BankIdx:%x  Value:%x\n",bankAddr+8,dat[2]);
	fn_wReg32_confirm(bankAddr + 12, dat[3]);
	printf("BankIdx:%x  Value:%x\n",bankAddr+12,dat[3]);

	
	//connect internal efuse_vddq to paldo 
	val32_check = read_wilc_reg(0x1428);
	fn_wReg32_confirm(0x1428,(val32_check | (1<<18)));

	//Force Tx mode to Enable paldo
	val32_check = read_wilc_reg(0x1480);
	val32_check &= ~0xffff0f00;
	val32_check |= 0x13330300;
	fn_wReg32_confirm(0x1480,val32_check); 

	val32_check = read_wilc_reg(0x1484);
	val32_check &= ~0xff;
	val32_check |= 0x11;
	fn_wReg32_confirm(0x1484,val32_check); 

	int retry =0;

	bankAddr = (bankIdx < 2 ) ? 0x1014 + bankIdx * 4 : 0x1320 + (bankIdx -2) * 4;
	fn_wReg32_confirm(bankAddr,0x007c081e); 

	retry = 100;
	do
	{
		val32_check = read_wilc_reg(bankAddr);
	}while( (retry-- >0) && ((val32_check & (1ul << 31)) !=(1ul <<31 )) );

	//clear the FUSE bit
	fn_wReg32_confirm(bankAddr,0x007c081c);
       
	if(retry<=0)
	{
		printf("error in loop %i\n", loop);
		// connect internal efuse_vddq to GND
		val32_check = read_wilc_reg(0x1428);
		fn_wReg32_confirm(0x1428, (val32_check & (~(1<<18)))); 

		val32_check = read_wilc_reg(0x1480);
		val32_check &= ~0xffff0f00;
		fn_wReg32_confirm(0x1480,val32_check); 

		val32_check = read_wilc_reg(0x1484);
		val32_check &= ~0xff;
		fn_wReg32_confirm(0x1484,val32_check); 
		return 1;
	}


	//connect internal efuse_vddq to GND

	val32_check = read_wilc_reg(0x1428);
	fn_wReg32_confirm(0x1428,(val32_check & (~(1<<18))));

	val32_check = read_wilc_reg(0x1480);
	val32_check &= ~0xffff0f00;
	fn_wReg32_confirm(0x1480,val32_check);  

	val32_check = read_wilc_reg(0x1484);
	val32_check &= ~0xff;
	fn_wReg32_confirm(0x1484,val32_check); 
	printf("written to efuse bank %d\r\n", bankIdx);
 
  	return 0;
}


int char_to_integer(char* c)
{
	int len=strlen(c),i,j;
	unsigned int mac=0;
	int ascii[50];
       
	for(i=0;i<len;i++)
	{
		if(c[i]==48 ||c[i]==49 || c[i]==50 || c[i]==51 || c[i]==52 || c[i]==53 ||c[i]==54 ||c[i]==55 ||c[i]==56 ||c[i]==57  )
		{  
			ascii[i]=c[i]-48;
		}
		else if (c[i]==65 ||c[i]==66 || c[i]==67 || c[i]==68 || c[i]==69 || c[i]==70)
		{
			ascii[i]=c[i]-55;
		}
		else if(97<=c[i]<=102)
		{  

			ascii[i]=c[i]-87;
		}
	}
	for(i=len-1,j=0;i>=0,j<len;i--,j++)
	{
		mac=mac | (ascii[j]<<(4*i));
	}
	return mac;	
}

void clear_efuse_bank(uint32_t* find_bank)
{
	int bankIdx=*find_bank;
	int loop=0;
	uint32_t bank_content[5];
	printf("Bank Indx: %d \n",bankIdx);

	bank_content[0]= (uint32_t) (0xc1000000); 
	bank_content[1]= (uint32_t) (0x00000000);
	bank_content[2]= (uint32_t) (0x00000000);
	bank_content[3]= (uint32_t) (0x00000000);

	printf("\n\n Invalidating the previous banks\n");
	for(int clear_efuse_bank =0; clear_efuse_bank < bankIdx; clear_efuse_bank++)
	{
		for(loop =0;loop<100;loop++)
		{

			if(write_efuse_by_bit(clear_efuse_bank, bank_content, loop) ==0)
			{
				break;
			}
		}
		if(loop==100)
		{
			printf("\n Bank %d invalidating Failure!!!\n",bankIdx);
		}
	}
}

int write_wilc_efuse_handler(char* offset_1, char* offset_2, uint32_t* offset_3, char* offset_4, uint32_t* find_bank)
{
	int bankIdx=*find_bank;
	uint32_t freq_offset=*offset_3;
	int option;
	uint32_t val32;
	uint32_t* dat2;
	uint32_t* mac1;
	uint32_t bank_content[5];


          //checking whether the freq offset bit is set or not
	if((xo_offset_write == 0) && (mac_address_write == 1)) 
	{
               if(((freq_offset & 0x80000000) >> (7*4))==8)
		{
			printf(" \nfreq offset value %x",((freq_offset & 0x7fff0000)>>(4*4)));
			printf("\n Do you want to proceed with this offset value from previous bank?\n Press 1 to proceed \n Press 0 to discard\n");
			scanf ("%d", &option);
			printf("\n selected value %d\n", option);
			if(option)
			{
				bank_content[0]= char_to_integer(offset_1); 
				bank_content[1]= char_to_integer(offset_2);
				bank_content[2]= *offset_3;
				bank_content[3]= char_to_integer(offset_4);
			} 
			else
			{
				bank_content[0]= char_to_integer(offset_1); 
				bank_content[1]= char_to_integer(offset_2);
				bank_content[2]= 0x00000000;
				bank_content[3]= char_to_integer(offset_4);
				printf("\n omitting freq offset and writing only mac address\n");
				printf("current freq offset value:0x0000\n");
			}
		}
		else
		{
			printf("\n Warning!!! freq offset used bit is not set\n");
			bank_content[0]= char_to_integer(offset_1); 
			bank_content[1]= char_to_integer(offset_2);
			bank_content[2]= 0x00000000;
			bank_content[3]= char_to_integer(offset_4);
			printf("current freq offset value:0x0000\n");
		}
	}
	
	if((xo_offset_write == 1) && (mac_address_write == 0))
	{			 
		printf("\n Wrtting new frequency offset value with previous mac address values\n");
		bank_content[0]= efuse_offset_1; 
		bank_content[1]= efuse_offset_2;
		bank_content[2]= ((xo_offset << 16) | 0x80000000);
		bank_content[3]= char_to_integer(offset_4);		
	}

	if((xo_offset_write == 1) && (mac_address_write == 1))
	{			 
		printf("\n Wrtting new frequency offset value with mac address \n");
		bank_content[0]= char_to_integer(offset_1); 
		bank_content[1]= char_to_integer(offset_2);
		bank_content[2]= ((xo_offset << 16) | 0x80000000);
		bank_content[3]= char_to_integer(offset_4);	
	}
	printf("Bank1:%x \t Bank2:%x \t Bank3:%x \t Bank4:%x \n",bank_content[0],bank_content[1],bank_content[2],bank_content[3]);
	
	/*Writting in to efuse register */
#if 1	
	for(int loop2=0; loop2< 100; loop2++)
	{
		for(int loop =0; loop<100;loop++)
		{
			if(write_efuse_by_bit(bankIdx, bank_content, loop) ==0)
			{
				loop =100;
				return 0;
			}
		}
		loop2=999;
	}
#endif 
	return 1;
}


static int usart_read_block_handler(uint8_t *buffer, uint8_t size)
{
	block_uart_cmd_hdr b_cmd_hdr;
	uart_cmd_hdr cmd_hdr;
	uint32_t val, fd, count, payload_length;
	unsigned char snum[10], ack[1];

	buffer_to_b_cmd_hdr(buffer + 1, &b_cmd_hdr);

	fd = open(file_name, O_RDWR);
	if (fd == -1)
	{
        	perror("query_apps open");
		return 2;
	}

	if (ioctl(fd, CMD_READ_BLOCK_REG, &b_cmd_hdr) == -1)
   	{
        	perror("query_apps ioctl set");
	}
	
	ack[0]=0xAC;
	write(fileno(fpw), ack, 1);

	for (count=0;count<= ((b_cmd_hdr.cmd >> 16)&0xFFFF); count++)
		sendbyte(b_cmd_hdr.b_buffer[count]);

   	close (fd);
	return 0;
}


static int usart_write_block_handler(uint8_t *buffer, uint8_t size)
{
	block_uart_cmd_hdr b_cmd_hdr;
	uart_cmd_hdr cmd_hdr;
	uint32_t val, fd, count, payload_length;
	unsigned char snum[10], ack[1], test;

	buffer_to_b_cmd_hdr(buffer + 1, &b_cmd_hdr);
	payload_length = (b_cmd_hdr.cmd >> 16) & 0xFFFF;

	ack[0]=0xAC;
	write(fileno(fpw), ack, 1);

	for (count=0 ; count< payload_length; count++)
	{
		b_cmd_hdr.b_buffer[count] = fgetc(fpr);
	}

	fd = open(file_name, O_RDWR);
	if (fd == -1)
	{
        	perror("query_apps open");
		return 2;
	}

	if (ioctl(fd, CMD_WRITE_BLOCK_REG, &b_cmd_hdr) == -1)
   	{
        	perror("query_apps ioctl set");
	}
   	close (fd);
	return 0;
}

static int usart_reconfigure_handler(uint8_t *buffer, uint8_t size)
{
	uart_cmd_hdr cmd_hdr;
	uint32_t val, fd, count, payload_length;
	unsigned char snum[10], ack[1];
	
	buffer_to_cmd_hdr(buffer + 1, &cmd_hdr);

	ack[0]=0xAC;
	write(fileno(fpw), ack, 1);
	return 0;
}


static int uart_sync_handler(uint8_t *buffer, uint8_t size){

	unsigned char bufff[1];

	bufff[0]=0x5B;
	write(fileno(fpw), bufff, 1);
}

int ble_uart_buffer_write(uint8_t *buf, uint16_t size)
{
	write(fileno(fpw_ble), buf, size);
}

int ble_hci_handler(uint8_t *buf, uint16_t size)
{
	ble_uart_buffer_write(buf,size);

}

 // HCI commands used for firmware upload
 static uint8_t hci_reset_command[] = { 0x01, 0x03, 0x0c, 0x00 };
 static uint8_t hci_read_local_version_information_command[] = { 0x01, 0x01, 0x10, 0x00 };
 static uint8_t hci_vendor_specific_reset_command[] = { 0x01, 0x55, 0xfc, 0x00 };

 void little_endian_store_32(uint8_t *buffer, uint16_t pos, uint32_t value){
	 buffer[pos++] = (uint8_t)(value);
	 buffer[pos++] = (uint8_t)(value >> 8);
	 buffer[pos++] = (uint8_t)(value >> 16);
	 buffer[pos++] = (uint8_t)(value >> 24);
 }

 uint8_t bt_uart_firmware_download(void)
 {
	 uint8_t s8Ret = 0;
	 uint16_t bytes_to_write = 0;

	fw_size = sizeof(ble_firmware);
	fw_data =(uint8_t *) ble_firmware;
	
	do 
	{
		bytes_to_write = min((fw_size - fw_offset), FIRMWARE_CHUNK_SIZE);
		// setup write command
		command_buffer[0] = 1;
		command_buffer[1] = 0x52;
		command_buffer[2] = 0xfc;
		command_buffer[3] = 8; // NOTE: this is in violation of the Bluetooth Specification, but documented in the Atmel-NNNNN-ATWIL_Linux_Porting_Guide
		little_endian_store_32(command_buffer, 4, IRAM_START + fw_offset);
		little_endian_store_32(command_buffer, 8, bytes_to_write);
		ble_uart_buffer_write(&command_buffer[0], 12);		
		ble_uart_buffer_write(&fw_data[fw_offset], bytes_to_write);	
		
		fw_offset += bytes_to_write;   
		if (fw_offset >= fw_size){
			return -1;
		}
		printf("fw_offset: %d fw_size: %d \n",fw_offset,fw_size);
	} while (1);

	 return s8Ret;
 }

 void ble_serial_bridge_init(void)
 {
	ble_uart_buffer_write(&hci_reset_command[0],sizeof(hci_reset_command));	
	ble_uart_buffer_write(&hci_read_local_version_information_command[0], sizeof(hci_read_local_version_information_command));	
	bt_uart_firmware_download();
	ble_uart_buffer_write(&hci_vendor_specific_reset_command[0], sizeof(hci_vendor_specific_reset_command));
 }


static int dummy_reg(uint8_t *buffer, uint8_t size){

	unsigned char bufff[1];

	bufff[0]=0x5B;
	write(fileno(fpw), bufff, 1);

}


struct usart_frame usart_handler[] = {
	{{0x12, 0}, 1, 1, uart_sync_handler}, /* nm_uart_sync_cmd */
	{{0xa5, 0x00, 0}, 2, sizeof(uart_cmd_hdr) + 1, usart_read_reg_with_ret_handler}, /* nm_uart_read_reg_with_ret */
	{{0xa5, 0x01, 0}, 2, sizeof(uart_cmd_hdr) + 1, usart_write_reg_handler}, /* nm_uart_write_reg */
	{{0xa5, 0x02, 0}, 2, sizeof(uart_cmd_hdr) + 1, usart_read_block_handler}, /* nm_uart_read_block */
	{{0xa5, 0x03, 0}, 2, sizeof(uart_cmd_hdr) + 1, usart_write_block_handler}, /* nm_uart_write_block */
	{{0xa5, 0x04, 0}, 2, 2, NULL}, /* Reset */
	{{0xa5, 0x05, 0}, 2, sizeof(uart_cmd_hdr) + 1,NULL},//usart_reconfigure_handler}, /* nm_uart_reconfigure */
	{{0xa5, 0x0A, 0}, 2, 2, NULL}, /* Read SPI read GPIO */
};

#define HANDLER_SIZE sizeof(usart_handler) / sizeof(struct usart_frame)

void parse_string(unsigned char *str, unsigned char *buffer){

	unsigned char icmd[3], i=0, x=0, a=0;
	int icmdl;
    
	for (i=0, a=0; i<26; i=i+2, a++)
	{
		for (x=0;x<2;x++)
		{
			icmd[x]=str[i+x];				
		}
		
		icmd[2]='\0';
 		icmdl = (int)strtol(icmd, NULL, 16);  
 		buffer[a] = icmdl;		
	}
}

/**Due to WILC reset error issue added this hardcode values*/
char reset_buffer_actual[13] = {0xA5,0x01,0x15,0x00,0x00,0x00,0x14,0x00,0x00,0xFF,0xFB,0xFF,0xFF};
char reset_buffer_error[13] = {0xA5,0x01,0x10,0x00,0x00,0x00,0x14,0x00,0x00,0xFF,0xFF,0xFF,0xFF};

char reset_wilc1000_sdio_buffer[13] = {0xA5,0x01,0x10,0x00,0x00,0x00,0x14,0x00,0x00,0xFF,0xFB,0xFF,0xFF};
char reset_wilc3000_sdio_buffer[13] = {0xA5,0x01,0x10,0x00,0x00,0x00,0x14,0x00,0x00,0xFD,0xFB,0xFF,0xFF};
char reset_spi_buffer[13] = {0xA5,0x01,0x10,0x00,0x00,0x00,0x14,0x00,0x00,0x05,0x00,0x00,0x00};

static void usart_frame_parse(uint8_t *buffer, uint8_t size)
{
	if (size == 0) {
		return;
	}

	for (uint32_t i = 0; i < HANDLER_SIZE; i++) {
		if (size >= usart_handler[i].min_size &&
			!memcmp(usart_handler[i].header, buffer, usart_handler[i].header_size)) {
				if (usart_handler[i].handler) 
				{
//					if(interface_type == 1){						
						if(!(memcmp(reset_spi_buffer, buffer, 13)))
						{
							printf("SDIO Reset\n");
							memcpy(buffer,reset_wilc1000_sdio_buffer,13);
						}
//					}
					if ((usart_handler[i].handler(buffer, size)) == 0) {
						return;
					} else {
						break;
					}
				}
			}
		}
}

int set_interface_attribs (int fd, int speed, unsigned char raw)
{
	struct termios tty;

	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		fprintf (stderr, " Error set_interface_attribs, tcgetattr: %d\n",fd);
		return -1;
	}


	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_lflag &= ~( ECHO | ECHOE | ECHOK | ICRNL | ICANON);
	tty.c_iflag &= ~(PARENB | BRKINT | ICRNL | INPCK | ISTRIP | IXON);
	tty.c_oflag &= ~(OPOST);
	tty.c_oflag &= ~(PARENB);	
	tty.c_cflag |= (CS8 | CLOCAL | CREAD);
	tty.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		fprintf(stderr, "Error set_interface_attribs, tcsetattr \n");
		return -1;
	}

	return 0;
}

int ble_set_interface_attribs (int fd, int speed, unsigned char raw)
{
	struct termios tty;
	struct sigaction saio;

	memset (&tty, 0, sizeof tty);
	printf("FD:%d \n",fd);
	if (tcgetattr (fd, &tty) != 0)
	{
		fprintf (stderr, " Error ble_set_interface_attribs, tcgetattr:%d \n",fd);
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_lflag &= ~( ECHO | ECHOE | ECHOK | ICRNL | ICANON);
	tty.c_iflag &= ~(PARENB | BRKINT | ICRNL | INPCK | ISTRIP | IXON);
	tty.c_lflag &= ~( ECHO | ECHOE | ECHOK | ICRNL | ICANON);
	tty.c_oflag &= ~(OPOST);
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag |= (CS8);
	tty.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);	

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		fprintf(stderr, "Error ble_set_interface_attribs, tcsetattr \n");
		return -1;
	}

	return 0;
}

void set_blocking (int fd, int should_block)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		fprintf(stderr, "\nset_blocking, tcgetattr");
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout
	printf("veof: %p", tty.c_cc[VEOF]);            // 0.5 seconds read timeout
	tty.c_cc[VEOF] = 0;            // 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
		fprintf (stderr, "\nset_blocking, tcsetattr");
}

int parse_baud(int baudrate)
{
	printf("Selected baudrate rate: %d !!! \n",baudrate);
	switch(baudrate)
	{
		case 50: return B50;
		case 75: return B75;
		case 110: return B110;
		case 134: return B134;
		case 150: return B150;
		case 200: return B200;
		case 300: return B300;
		case 600: return B600;
		case 1200: return B1200;
		case 2400: return B2400;
		case 4800: return B4800;
		case 9600: return B9600;
		case 19200: return B19200;
		case 38400: return B38400;
		case 57600: return B57600;
		case 115200: return B115200;
		case 230400: return B230400;
#if 0 /** Not supported by tty serial driver*/		
		case 460800: return B460800;
		case 500000: return B500000;
		case 576000: return B576000;
		case 921600: return B921600;
		case 1152000: return B1152000;
		case 1500000: return B1500000;
		case 2000000: return B2000000;
		case 2500000: return B2500000;
#endif
		default: return 0;
	}
}

void init_serial(int baudrate,char *dev){

	char str[100];
	
	fpw = fopen(dev, "w");
	if (fpw == NULL)
    	{
       	puts("Issue in opening the input file for reading");
    	}

	fpr = fopen(dev, "r");
	if (fpr == NULL)
    	{
       	puts("Issue in opening the input file for writing");
	}

	set_interface_attribs (fileno(fpr), baudrate, 1);  //, 0);
	set_interface_attribs (fileno(fpw), baudrate, 1);  //, 0);
}

void ble_init_serial(int baudrate,char *dev){

	char str[100];
	
	fpw_ble = fopen(dev, "w");
	if (fpw_ble == NULL)
    	{
       	puts("Issue in opening the input file for reading");
    	}

	fpr_ble= fopen(dev, "r");
	if (fpr_ble == NULL)
    	{
       	puts("Issue in opening the input file for writing");
	}

	ble_set_interface_attribs (fileno(fpr_ble), baudrate, 1);  //, 0);
	ble_set_interface_attribs (fileno(fpw_ble), baudrate, 1);  //, 0);
}

void int_to_ascii(unsigned char value){

	unsigned char a1, a2;

	a1 = value/10;
	a2 = value%10;
}

void ble_uart_getchar(int status)
{
	printf("BLE Char Received \n");
	ble_res = 1;
}

int getchar_with_timeout(int ms, FILE * fptr)
{
	int val = 0;
	do {
		sleep(1);
	}while((val = fgetc(fptr)) || ms--);	 
}

void ble_cmd_uart_getchar(void)
{
	unsigned char a, rlen= 1,buffer[20],buf[20],ret =0;

	rlen = 0;
	buffer[rlen++] = fgetc(fpr);
	if(buffer[0]==0x01)
	{
		buffer[rlen++] = fgetc(fpr);
		if(buffer[1] == 0x55){
			do{
				buffer[rlen++] = fgetc(fpr);	
		       }while(rlen<4);					
			buffer[4] = '\0';
			ble_uart_buffer_write(buffer,4);
			ret = read(fileno(fpr_ble),buf,7);
			if (ret == 7)
				write(fileno(fpw), buf, 7);			
		}
		
		if(buffer[1] == 0x03){
			do{
				buffer[rlen++] = fgetc(fpr);	
		       }while(rlen<4);					
			buffer[4] = '\0';
			ble_uart_buffer_write(buffer,4);
			ret = read(fileno(fpr_ble),buf,7);
			if (ret == 7)
				write(fileno(fpw), buf, 7);					
		}
		if(buffer[1] == 0x1E){
			do{
				buffer[rlen++] = fgetc(fpr);	
		       }while(rlen<7);					
			buffer[7] = '\0';
			ble_uart_buffer_write(buffer,7);
			ret = read(fileno(fpr_ble),buf,7);
			if (ret == 7)
				write(fileno(fpw), buf, 7);					
		}
		if(buffer[1] == 0x1D){
			do{
				buffer[rlen++] = fgetc(fpr);	
		       }while(rlen<5);					
			buffer[5] = '\0';
			ble_uart_buffer_write(buffer,5);
			ret = read(fileno(fpr_ble),buf,7);
			if (ret == 7)
				write(fileno(fpw), buf, 7);					
		}
		if(buffer[1] == 0x1F){
			do{
				buffer[rlen++] = fgetc(fpr);	
		       }while(rlen<4);					
			buffer[4] = '\0';
			ble_uart_buffer_write(buffer,4);
			ret = read(fileno(fpr_ble),buf,9);
			if (ret == 9)
				write(fileno(fpw), buf, 9);			
		}
	}		
  	return;
}

void cmd_uart_getchar(void)
{
	unsigned char a, rlen= 0,buffer[20],ret = 0;
	static int count = 0;

	for(a=0; a<=13;a++)
		buffer[13] = '\0';

	buffer[0] = fgetc(fpr);
	
	if(buffer[0]==0xa5)
	{
		for(a=1; a<13;a++) 
			buffer[a]=fgetc(fpr);			
		buffer[13] = '\0';
		rlen=13;
	}
	else if(buffer[0]==0x12)
		rlen=1;

	if (buffer[0] == 0xFF) {
		memset(buffer,0,20);
	}
	else if (buffer[0] != 0x12 && buffer[0] != 0xA5 && buffer[0] != 0x01) {
		/* Undefined message, send error. */
		buffer[0] = 0xEA; 
		write(fileno(fpw), buffer, 1);
		memset(buffer,0,20);
	}
	usart_frame_parse(buffer, rlen);
}

void set_offset_2(char* mac_address,char *offset_2)
{
	int i,j;
	for(i=6,j=0;i<12,j<7;j++,i++)
	{
		offset_2[j]=mac_address[i];
	}
}


int validate_mac(char* mac_address)
{
	int find_mac_length=0,mac_limit=12;

	find_mac_length=strlen(mac_address);

	if(find_mac_length<mac_limit || find_mac_length>mac_limit)
	{
		return 1;
	}
	else 
	return 0;
}

int roundoff(float num) 
{ 
	return num < 0 ? num - 0.5 : num + 0.5; 
} 

int main(int argc, char *argv[])
{
         char offset_1[15]="81";
         char offset_2[9];
         char mac_address[15];
         char gain[]="00";
         char offset_4[]="0";
         char bank_content[25];
         uint32_t bank_address[4]; 
	  float freq_offset;	 
          
	int baudrate = 0,i,rlen = 0,ret = 0;
	unsigned char buffer[20];
	unsigned char  buf[13];
	printf("WILC Serial Bridge Application!!!\n");
	//Check if a baudrate was given
	if(argc >= 1) {
		if(!(strcmp(argv[1], "spi"))) {
			interface_type = 0;
			printf("SPI Interface Selected !!!\n");
		}	
		else if(!(strcmp(argv[1], "sdio"))){
			interface_type = 1;
			printf("SDIO Interface Selected !!!\n");
		}
		else if(!(strcmp(argv[1], "uart"))){
			interface_type = 2;
			printf("WILC3000 BLE UART interface Selected!!!\n");
		}
	}

	if(argc >= 2){
		if (sscanf (argv[2], "%i", &baudrate)!=1) {}		
	}
	if(argc >= 3){
		if(!(strncmp(argv[3], "tty",3))) 
		{
			strcat(cmd_uart,argv[3]);
			printf("Selected Console: %s !!!\n",cmd_uart);
		}		
	}

       if((argc >= 5) && (interface_type == 2)){
		if(!(strncmp(argv[4], "tty",3))) 
		{
			strcat(ble_uart,argv[4]);
		}
		printf("Selected WILC3000 BLE UART interface: %s !!!\n",ble_uart);
	}

	if((argc >= 5) && (interface_type != 2)){
		if(!(strncmp(argv[4], "mac",3))) 
		{
			mac_address_app = 1;
			if((argc >= 6) && (mac_address_app == 1)){
                   		if(sscanf (argv[5],"%s", mac_address)!=1) {}
                       	if(validate_mac(mac_address)==0)
                        	{
                            	 mac_address_write = 1;
					strncat(offset_1, mac_address,6);
					set_offset_2(mac_address,offset_2);
					strncat(offset_2,gain,2);								 
                             	printf("MAC address is valid!!!\n");								
                        	}
				else {
					printf("MAC address is invalid!!!\n");
					return -1;	
				}
        		}
		}		
	}  

       if((argc >= 7) && (interface_type != 2) && (mac_address_app == 1)){
		if(!(strncmp(argv[6], "xooff",5))) 
		{
			xo_offset_write = 1;
			if (sscanf (argv[7], "%f", &freq_offset)!=1) {}			
			xo_offset = roundoff(freq_offset * 64);
		}
		
	}
	   
       if((argc >= 6) && (interface_type != 2) && (mac_address_app == 0)){
	   	if(!(strncmp(argv[4], "xooff",5))) 
		{
			xo_offset_write = 1;
			if (sscanf (argv[5], "%f", &freq_offset)!=1) {}			
			xo_offset = roundoff(freq_offset * 64);
		}
		
	}	   

	baudrate = parse_baud(baudrate);

	if(baudrate == 0)
	{
		baudrate = parse_baud(115200);
		printf("Invalid baud rate, defaulting to 115200.");
	}
     
       init_serial(baudrate,cmd_uart);

	if(interface_type ==2){
		ble_init_serial(baudrate,ble_uart);	
		printf("WILC3000 BLE Firmware Downloading...\n");
		ble_serial_bridge_init();
		printf("WILC3000 BLE Firmware Download Completed\n");
	}

	while(1)
   	{
		if((interface_type == 0) || (interface_type == 1))
		{
			if((mac_address_app == 0) && (xo_offset_write == 0))
			{
				cmd_uart_getchar();	
			}
			else if((mac_address_write == 1) || (xo_offset_write == 1))
	     		{
	     	
				read_wilc_efuse_with_ret_handler(&gain_offset, &find_bank);
				if(find_bank >5)
				{
					printf("\n Bank is full!!!\n");
					return 0;
				}
				if(write_wilc_efuse_handler(offset_1, offset_2, &gain_offset, offset_4,&find_bank)==0)
				{
					clear_efuse_bank(&find_bank);
					mac_address_app = 0;
					mac_address_write  = 0;	
					return 0;
				}
				else
				{
					printf("\n eFuse write failure!!!");
				}  
			}
		}
		else if((interface_type == 2) && (!mac_address_app))
		{
			ble_cmd_uart_getchar();
		}
	}	
	return 0;
}

