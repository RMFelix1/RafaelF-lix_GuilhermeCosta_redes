/*Non-Canonical Input Processing*/

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <string.h>


//ROLE
#define NOT_DEFINED -1
#define TRANSMITTER 0
#define RECEIVER 1

//size
#define MAX_PAYLOAD_SIZE 1000

//connection
#define BAUDRATE B9600
#define _POSIX_SOURCE 1 /* POSIX compliant source */
#define MAX_RETRANSMISSIONS_DEFAULT 3
#define TIMEOUT_DEFAULT 4

#define FALSE 0
#define TRUE 1

//state machine
volatile int STOP=FALSE;
int state=0;
int stateRead=0;
#define START 0
#define FLAGRCV 1
#define ARCV 2
#define CRCV 3
#define BCCOK 4
#define STOP_STATE_MACHINE 5


typedef struct linkLayer{
char serialPort[50];
int role; //defines the role of the program: 0==Transmitter, 1=Receiver
int baudRate;
int numTries;
int timeOut;
} linkLayer;

void statemachine(unsigned char byte);
int llopen(linkLayer connectionParamaters);
int llwrite(unsigned char *buffer, int length);
int llread(unsigned char *buffer);
void statemachineRead(unsigned char c);

struct termios oldtio,newtio;
int fd;
linkLayer connection;

//control switch
unsigned char c = 0x00;

//statemachineRead
int size=0;
unsigned char response[255];
unsigned char bcc2check = 0x00;

//default control characters
unsigned char defW[5];
defW[0]=0x5c;
defW[1]=0x01;
defW[2]=0x07;
defW[3]=defW[1]^defW[2];
defW[4]=0x5c;
//
unsigned char defR[5];
defR[0]=0x5c;
defR[1]=0x03;
defR[2]=0x06;
defR[3]=defR[1]^defR[2];
defR[4]=0x5c;
//

int main(int argc, char** argv)
{
    char buf[255];
    
    if ( (argc < 2) ||
         ((strcmp("/dev/ttyS0", argv[1])!=0) &&
          (strcmp("/dev/ttyS1", argv[1])!=0) )) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
        exit(1);
    }

    //linkLayer connection;
    strcpy(connection.serialPort,argv[1]);
    connection.role = 1;
    connection.baudRate = BAUDRATE;
    connection.numTries=0;
    connection.timeOut = TIMEOUT_DEFAULT;

    int success = llopen(connection);
    if(success)
    {
        //TESTING
        strcpy(buf, "SIUU");
        int res = write(fd,buf,5);

        tcsetattr(fd,TCSANOW,&oldtio);
        close(fd);
    }
    else printf("UNSUCCESSFUL\n");
    return 0;
}

void statemachine(unsigned char byte)
{
    switch (state)
    {
        case START:
            if(byte==0x5c) state = FLAGRCV;
            break;
        case FLAGRCV:
            if(byte==0x01) state = ARCV;
            else if(byte==0x5c) state = FLAGRCV;
            else state = START;   
            break;
        case ARCV:
            if(byte==0x07) state = CRCV;
            else if(byte==0x5c) state = FLAGRCV;
            else state = START;
            break;
        case CRCV:
            if(byte==(0x01^0x07)) state = BCCOK;
            else if(byte==0x5c) state = FLAGRCV;
            else state = START; 
            break;  
        case BCCOK:  
            if(byte==0x5c) state = STOP_STATE_MACHINE;
            else state = START;  
            break;
            
    }
}

int llopen(linkLayer connectionParameters)
{
    int c, res;
    char buf[255];
    /*
    Open serial port device for reading and writing and not as controlling tty
    because we don't want to get killed if linenoise sends CTRL-C.
    */
    fd = open(connectionParameters.serialPort, O_RDWR | O_NOCTTY );
    if (fd < 0) { perror(connectionParameters.serialPort); exit(-1); }

    if (tcgetattr(fd,&oldtio) == -1) { /* save current port settings */
        perror("tcgetattr");
        exit(-1);
    }

    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
    newtio.c_iflag = IGNPAR;
    newtio.c_oflag = 0;

    /* set input mode (non-canonical, no echo,...) */
    newtio.c_lflag = 0;

    newtio.c_cc[VTIME]    = 0;   /* inter-character timer unused */
    newtio.c_cc[VMIN]     = 5;   /* blocking read until 5 chars received */

    /*
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
    leitura do(s) próximo(s) caracter(es)
    */


    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        exit(-1);
    }

    printf("New termios structure set\n");

    while (STOP==FALSE) {       /* loop for input */
        res = read(fd,buf,5); 

        for(int i=0;i<res;i++) statemachine(buf[i]);
        printf("STATE IS: %d\n",state);
        if(state==5)
        {
            buf[1] = 0x03;
            buf[2] = 0x06;
            buf[3]=buf[1]^buf[2];
            res = write(fd,buf,5);
            STOP=TRUE;
        }
        state=START;
    }

    tcsetattr(fd,TCSANOW,&oldtio);
    return (1);
}

int llwrite(unsigned char *buffer,int length)
{
    unsigned char send[255];

    send[0]=defW[0];

    if (connection.role==1) send[1]=defW[1];
    else send[1]=0x03;

    send[2] = c;
    if(c==0x00) c=0x01;
    else c=0x00;
    
    send[3]=send[1]^send[2];

    unsigned char bcc2 = 0x00;

    for(int i=0;i<length;i++)
    {
        send[4+i]=buffer[i];
        bcc2 = bcc2^buffer[i];
    }

    send[4+length]=bcc2;
    send[4+length+1]=defW[0];

    return write(fd,send,4+length+1);
    //talvez em vez de return, fazer read esperando pelo ACK
}

int llread(unsigned char *buffer)
{
    size = read(fd,response,255);

    for(int i=0;i<size;i+) statemachineRead(response[i]);

    if(state==STOP_STATE_MACHINE)
    {
        strcpy(buffer,response);
        return size;
        //talvez em vez de return, fazer write do ACK
    }
    return -1;
}

void statemachineRead(unsigned char c)
{
    switch (state)
    {
        case START:
            if(byte==defW[0]) state = FLAGRCV;
            break;
        case FLAGRCV:
            if(byte==defW[1]) state = ARCV;
            else if(byte==defW[0]) state = FLAGRCV;
            else state = START;   
            break;
        case ARCV:
            if(byte==defW[2]) state = CRCV;
            else if(byte==defW[0]) state = FLAGRCV;
            else state = START;
            break;
        case CRCV:
            if(byte==(defW[3])) state = BCCOK;
            else if(byte==defW[0]) state = FLAGRCV;
            else state = START; 
            break;  
        case BCCOK:  
            if(byte==defW[0])
            {
                for(int i=0;i<size;i++) bcc2check=bcc2check^response[i];
                if (response[size-1]==bcc2check) state = STOP_STATE_MACHINE;
                break;
            }
            response[size]=c;
            size++;
            break;
    }
}
