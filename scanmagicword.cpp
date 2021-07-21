#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>        
#include <stdlib.h> 
#include <unistd.h>

#include <iostream>

// from https://e2e.ti.com/support/sensors-group/sensors/f/sensors-forum/947021/iwr6843aop-confusion-regarding-people-counting-and-6843-aop-evm

#define BAUDRATE B921600
#define MODEMDEVICE "/dev/ttyUSB1"
#define FALSE 0
#define TRUE 1
void openport(void);
void sendport(void);
void readport(void);
int fd=0, n;
static int cnt, size, s_cnt;
unsigned char *var;
struct termios oldtp, newtp;
char sendcmd1[10]="\0";
FILE *file;
int magic[]={2,1,4,3,6,5,8,7};

void  readport(void){
  unsigned char buff;
  int counter=0;
  while(1){
    float v;
    unsigned int ival;
    n=read(fd, &buff, 1); // read from device and store in buff, -1 if error, >0 if success
    fcntl(fd,F_SETFL,0);
    if(n==-1){
        goto quit;
        printf("n==-1, quit\n");    
    }

    if(n>0){
        if(buff==magic[counter]){ //check if sequence of magic word in buff, increment counter if present
            printf("\nbuff==magic[counter], counter++\n");
            printf("Magic: ");
            for (int i = 0; i < sizeof(magic)/sizeof(magic[0]); i++) 
                std::cout << magic[i];
            std::cout << std::endl;
            printf("Found: %d \n", magic[counter]);
            printf("counter: %d \n", counter);
            counter++;
        }else{
            //printf("else, counter=0\n");
            counter=0; // reset counter if sequence not in magic word
        }
        printf("Buff: %d \n", buff);
      if(counter==8){
        printf("packet\n");
        }
      }
    }
  quit:
  return;
  }
void openport(void){
  fd = open(MODEMDEVICE, O_RDWR | O_NOCTTY |O_NDELAY );
  printf("fd : %d\n",fd);
  if (fd <0){
    perror(MODEMDEVICE);
    }
                                                                                
  fcntl(fd,F_SETFL,0);
  tcgetattr(fd,&oldtp); 
  bzero(&newtp, sizeof(newtp));
  newtp.c_cflag = BAUDRATE | CRTSCTS | CS8 | CLOCAL | CREAD;
  newtp.c_iflag = IGNPAR | ICRNL;
  }

int  main(){
  openport();
  readport();
  return 0;
  }
