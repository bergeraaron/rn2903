#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>
#include <time.h>


/*
port=portnum,
baudrate=460800,
bytesize=serial.EIGHTBITS,
parity=serial.PARITY_NONE,
stopbits=serial.STOPBITS_ONE,
timeout=None,  # seconds
writeTimeout=None,
rtscts=True
*/

/* baudrate settings are defined in <asm/termbits.h>, which is
included by <termios.h> */
#define BAUDRATE B57600
/* change this definition for the correct port */
#define MODEMDEVICE "/dev/ttyUSB0"
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define CRTSCTS  020000000000 /*should be defined but isn't with the C99*/
#define FALSE 0
#define TRUE 1

#define bzero(b,len) (memset((b), '\0', (len)), (void) 0)

volatile int STOP=FALSE; 

main()
{
  int fd,c, res;
  struct termios oldtio,newtio;
  unsigned char buf[255];memset(buf,0x00,255);
  unsigned char pkt[255];memset(pkt,0x00,255);
  unsigned char zbpkt[255];memset(zbpkt,0x00,255);
  int pkt_ctr = 0;
  int zbpkt_ctr = 0;
  int pld_ctr = 0;
  unsigned char tmp[2];

/* 
  Open modem device for reading and writing and not as controlling tty
  because we don't want to get killed if linenoise sends CTRL-C.
*/
 fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY ); 
 if (fd <0) {perror(MODEMDEVICE); exit(-1); }

 tcgetattr(fd,&oldtio); /* save current serial port settings */
 bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

/* 
  BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
  CRTSCTS : output hardware flow control (only used if the cable has
			all necessary lines. See sect. 7 of Serial-HOWTO)
  CS8     : 8n1 (8bit,no parity,1 stopbit)
  CLOCAL  : local connection, no modem contol
  CREAD   : enable receiving characters
*/
 newtio.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
 
/*
  IGNPAR  : ignore bytes with parity errors
  ICRNL   : map CR to NL (otherwise a CR input on the other computer
			will not terminate input)
  otherwise make device raw (no other input processing)
*/
 newtio.c_iflag = IGNPAR;// | ICRNL;
 
/*
 Raw output.
*/
 newtio.c_oflag = 0;
 
/*
  ICANON  : enable canonical input
  disable all echo functionality, and don't send signals to calling program
*/
 //newtio.c_lflag = ICANON;
 
/* 
  initialize all control characters 
  default values can be found in /usr/include/termios.h, and are given
  in the comments, but we don't need them here
*/
/*
 newtio.c_cc[VINTR]    = 0;     // Ctrl-c
 newtio.c_cc[VQUIT]    = 0;     // Ctrl-\//
 newtio.c_cc[VERASE]   = 0;     // del
 newtio.c_cc[VKILL]    = 0;     // @ 
 newtio.c_cc[VEOF]     = 4;     // Ctrl-d //
 newtio.c_cc[VTIME]    = 0;     // inter-character timer unused //
 newtio.c_cc[VMIN]     = 1;     // blocking read until 1 character arrives //
 newtio.c_cc[VSWTC]    = 0;     // '\0' //
 newtio.c_cc[VSTART]   = 0;     // Ctrl-q // 
 newtio.c_cc[VSTOP]    = 0;     // Ctrl-s //
 newtio.c_cc[VSUSP]    = 0;     // Ctrl-z //
 newtio.c_cc[VEOL]     = 0;     // '\0' //
 newtio.c_cc[VREPRINT] = 0;     // Ctrl-r //
 newtio.c_cc[VDISCARD] = 0;     // Ctrl-u //
 newtio.c_cc[VWERASE]  = 0;     // Ctrl-w //
 newtio.c_cc[VLNEXT]   = 0;     // Ctrl-v //
 newtio.c_cc[VEOL2]    = 0;     // '\0' //
*/
/* 
  now clean the modem line and activate the settings for the port
*/
 tcflush(fd, TCIFLUSH);
 tcsetattr(fd,TCSANOW,&newtio);

 char cmd[10][256];memset(cmd,0x00,256);

//common against both zigbee and btle
//seq 1
 sprintf(cmd[0],"sys get ver\r\n");
 //seq 2
 sprintf(cmd[1],"mac pause\r\n");
 //seq 3
 sprintf(cmd[2],"radio set pwr 10\r\n");
 //seq 4
 sprintf(cmd[3],"radio rx 0\r\n");

 int ctr = 0;
 int cmdctr = 0;
 cmdctr=0;

printf("%s\n",cmd[cmdctr]);
write(fd,cmd[cmdctr],strlen(cmd[cmdctr]));
usleep(5000);
cmdctr++;
while(1)
 {
        //printf("ctr:%d\n",ctr);
        res = read(fd,buf,255);
        if(res > 0)
        {
                printf("res:%d ctr:%d buf:%s\n",res,ctr,buf);
                tcflush(fd, TCIFLUSH);
                if(cmdctr < 4)
                {
                  //send next command
                  printf("send next cmd:%d cmd:%s\n",cmdctr,cmd[cmdctr]);
                  write(fd,cmd[cmdctr],strlen(cmd[cmdctr]));
            		  usleep(5000);
                  cmdctr++;
                }
                ctr = 0;
          memset(buf,0x00,255);
        }
        usleep(10);
        ctr++;
        if(ctr > 100000000)
                break;
 }

 /* restore the old port settings */
 tcsetattr(fd,TCSANOW,&oldtio);
 
}
