#include<stdio.h>
#include<stdlib.h>
#include<fcntl.h>
#include<unistd.h>
#include<assert.h>
#include<termios.h>
#include<string.h>
#include<sys/time.h>
#include<time.h>
#include<sys/types.h>
#include<errno.h>

static int ret;
static int fd;

#define BAUD 115200 //115200 for JY61 ,9600 for others

int uart_open(int fd,const char *pathname)
{
    fd = open(pathname, O_RDWR|O_NOCTTY); 
    if (-1 == fd)
    { 
        perror("Can't Open Serial Port"); 
		return(-1); 
	} 
    else
		printf("open %s success!\n",pathname);
    if(isatty(STDIN_FILENO)==0) 
		printf("standard input is not a terminal device\n"); 
    else 
		printf("isatty success!\n"); 
    return fd; 
}

int uart_set(int fd,int nSpeed, int nBits, char nEvent, int nStop)
{
     struct termios newtio,oldtio; 
     if  ( tcgetattr( fd,&oldtio)  !=  0) {  
      perror("SetupSerial 1");
	  printf("tcgetattr( fd,&oldtio) -> %d\n",tcgetattr( fd,&oldtio)); 
      return -1; 
     } 
     bzero( &newtio, sizeof( newtio ) ); 
     newtio.c_cflag  |=  CLOCAL | CREAD;  
     newtio.c_cflag &= ~CSIZE;  
     switch( nBits ) 
     { 
     case 7: 
      newtio.c_cflag |= CS7; 
      break; 
     case 8: 
      newtio.c_cflag |= CS8; 
      break; 
     } 
     switch( nEvent ) 
     { 
     case 'o':
     case 'O': 
      newtio.c_cflag |= PARENB; 
      newtio.c_cflag |= PARODD; 
      newtio.c_iflag |= (INPCK | ISTRIP); 
      break; 
     case 'e':
     case 'E': 
      newtio.c_iflag |= (INPCK | ISTRIP); 
      newtio.c_cflag |= PARENB; 
      newtio.c_cflag &= ~PARODD; 
      break;
     case 'n':
     case 'N': 
      newtio.c_cflag &= ~PARENB; 
      break;
     default:
      break;
     } 

     /*设置波特率*/ 

switch( nSpeed ) 
     { 
     case 2400: 
      cfsetispeed(&newtio, B2400); 
      cfsetospeed(&newtio, B2400); 
      break; 
     case 4800: 
      cfsetispeed(&newtio, B4800); 
      cfsetospeed(&newtio, B4800); 
      break; 
     case 9600: 
      cfsetispeed(&newtio, B9600); 
      cfsetospeed(&newtio, B9600); 
      break; 
     case 115200: 
      cfsetispeed(&newtio, B115200); 
      cfsetospeed(&newtio, B115200); 
      break; 
    //  case 460800: 
    //   cfsetispeed(&newtio, B460800); 
    //   cfsetospeed(&newtio, B460800); 
    //   break; 
     default: 
      cfsetispeed(&newtio, B9600); 
      cfsetospeed(&newtio, B9600); 
     break; 
     } 
     if( nStop == 1 ) 
      newtio.c_cflag &=  ~CSTOPB; 
     else if ( nStop == 2 ) 
      newtio.c_cflag |=  CSTOPB; 
     newtio.c_cc[VTIME]  = 0; 
     newtio.c_cc[VMIN] = 0; 
     tcflush(fd,TCIFLUSH); 

if((tcsetattr(fd,TCSANOW,&newtio))!=0) 
     { 
      perror("com set error"); 
      return -1; 
     } 
     printf("set done!\n"); 
     return 0; 
}

int uart_close(int fd)
{
    assert(fd);
    close(fd);

    return 0;
}
int send_data(int  fd, char *send_buffer,int length)
{
	length=write(fd,send_buffer,length*sizeof(unsigned char));
	return length;
}
int recv_data(int fd, char* recv_buffer,int length)
{
	length=read(fd,recv_buffer,length);
	return length;
}


void unbloking_configuration(int  fd)
{
    char send_buffer[5];
    send_buffer[0] = 0xFF;
    send_buffer[1] = 0xAA;
    send_buffer[2] = 0x69;
    send_buffer[3] = 0x88;
    send_buffer[4] = 0xB5;
    send_data(fd, send_buffer, 5);
}

void reset_configuration(int  fd)
{
    char send_buffer[5];
    send_buffer[0] = 0xFF;
    send_buffer[1] = 0xAA;
    send_buffer[2] = 0x00;
    send_buffer[3] = 0x01;
    send_buffer[4] = 0x00;
    send_data(fd, send_buffer, 5);
}

void save_configuration(int fd)
{
    char send_buffer[5];
    send_buffer[0] = 0xFF;
    send_buffer[1] = 0xAA;
    send_buffer[2] = 0x00;
    send_buffer[3] = 0x00;
    send_buffer[4] = 0x00;
    send_data(fd, send_buffer, 5);
}

void set_return_rate(int fd)
{
    char send_buffer[5];
    send_buffer[0] = 0xFF;
    send_buffer[1] = 0xAA;
    send_buffer[2] = 0x03;
    send_buffer[3] = 0x0b;
    send_buffer[4] = 0x00;
    unbloking_configuration(fd);
    sleep(1);
    send_data(fd, send_buffer, 5);
    sleep(1);
    save_configuration(fd);
    sleep(1);
}

void set_baund_rate(int fd)
{
    char send_buffer[5];
    send_buffer[0] = 0xFF;
    send_buffer[1] = 0xAA;
    send_buffer[2] = 0x04;
    send_buffer[3] = 0x06;
    send_buffer[4] = 0x00;
    unbloking_configuration(fd);
    sleep(1);
    send_data(fd, send_buffer, 5);
    sleep(1);
    save_configuration(fd);
    sleep(1);
}

void set_output_data(int fd)
{
    const char time_flgBit = 0x01;
    const char acc_flgBit = 0x02;
    const char omega_flgBit = 0x04;
    const char angle_flgBit = 0x08;
    const char mag_flgBit = 0x10;
    char send_buffer[5];
    send_buffer[0] = 0xFF;
    send_buffer[1] = 0xAA;
    send_buffer[2] = 0x02;
    send_buffer[3] = acc_flgBit | omega_flgBit | angle_flgBit;
    send_buffer[4] = 0x00;
    unbloking_configuration(fd);
    sleep(1);
    send_data(fd, send_buffer, 5);
    sleep(1);
    save_configuration(fd);
    sleep(1);
}

void read_return_rate(int fd)
{
    char send_buffer[5];
    send_buffer[0] = 0xFF;
    send_buffer[1] = 0xAA;
    send_buffer[2] = 0x27;
    send_buffer[3] = 0x03; // 0x03 : return rate, 0x22 : sleep
    send_buffer[4] = 0x00;
    send_data(fd, send_buffer, 5);
    // while(true)
    // {
    //     char r_buf[1024];
    //     int ret;
    //     ret = recv_data(fd,r_buf,44);
    //     if(ret == -1)
    //     {
    //         printf("uart read failed!\n");
    //         exit(EXIT_FAILURE);
    //     }

    //     if(r_buf[0] == 0x55)
    //     {
    //         // if(r_buf[1] == 0x54)
    //         {
    //             exit(1);
    //         }
    //     }
    // }
}


float a[3],w[3],Angle[3],h[3];
void ParseData(char chr)
{
		static char chrBuf[100];
		static unsigned char chrCnt=0;
		signed short sData[4];
		unsigned char i;
		char cTemp=0;
		time_t now;
        static time_t befo_time;
        static size_t data_count = 0;

		chrBuf[chrCnt++]=chr;
		if (chrCnt<11) return;
		// for (i=0;i<10;i++) cTemp+=chrBuf[i];

        //debug
        for (i=0;i<10;i++)
        {
            cTemp+=chrBuf[i];

            // if(chrBuf[i]==0x5F)
            // {
            //     for(size_t i = 0; i < chrCnt; i++)
            //     {
            //         printf("%x, ", (unsigned char)chrBuf[i]);
            //     }
            //     printf("\n");
            //     exit(1);
            // }
        }

		if ((chrBuf[0]!=0x55)||((chrBuf[1]&0x50)!=0x50)||(cTemp!=chrBuf[10])) {printf("Error:%x %x\r\n",chrBuf[0],chrBuf[1]);memcpy(&chrBuf[0],&chrBuf[1],10);chrCnt--;return;}
		
		memcpy(&sData[0],&chrBuf[2],8);
		switch(chrBuf[1])
		{
				case 0x51:
					for (i=0;i<3;i++) a[i] = (float)sData[i]/32768.0*16.0;
					time(&now);
                    if(now != befo_time)
                    {
                        data_count = 0;
                        befo_time = now;
                    }
                    data_count++;
                    printf("\r\n(%zu)\tT:%s:a:%6.3f %6.3f %6.3f ", data_count, asctime(localtime(&now)), a[0],a[1],a[2]);					
					break;
				case 0x52:
					for (i=0;i<3;i++) w[i] = (float)sData[i]/32768.0*2000.0;
					printf("w:%7.3f %7.3f %7.3f ",w[0],w[1],w[2]);					
					break;
				case 0x53:
					for (i=0;i<3;i++) Angle[i] = (float)sData[i]/32768.0*180.0;
					printf("A:%7.3f %7.3f %7.3f ",Angle[0],Angle[1],Angle[2]);
					break;
				case 0x54:
					for (i=0;i<3;i++) h[i] = (float)sData[i];
					printf("h:%4.0f %4.0f %4.0f ",h[0],h[1],h[2]);
					
					break;
		}		
		chrCnt=0;		
}

int main(void)
{
    char r_buf[1024];
    bzero(r_buf,1024);

    fd = uart_open(fd, "/dev/ttyUSB0");/*串口号/dev/ttySn,USB口号/dev/ttyUSBn */ 
    if(fd == -1)
    {
        printf("uart_open error\n");
        exit(EXIT_FAILURE);
    }

    if(uart_set(fd,BAUD,8,'N',1) == -1)
    {
        printf("uart set failed!\n");
        exit(EXIT_FAILURE);
    }

    // set_baund_rate(fd);

    // set_return_rate(fd);
    // read_return_rate(fd);
    // set_output_data(fd);

    while(1)
    {
        ret = recv_data(fd,r_buf,44);
        if(ret == -1)
        {
            printf("uart read failed!\n");
            exit(EXIT_FAILURE);
        }
		for (int i=0;i<ret;i++) {/*printf("%2X ",r_buf[i]);*/ParseData(r_buf[i]);}
        usleep(1000);
    }

    ret = uart_close(fd);
    if(ret == -1)
    {
        printf("uart_close error\n");
        exit(EXIT_FAILURE);
    }

    exit(EXIT_SUCCESS);
}
