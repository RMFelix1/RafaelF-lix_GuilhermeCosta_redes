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
int discState = 0;
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
void ByteDestuffing(unsigned char byte);
int llclose(linkLayer connectionParameters);
void stateMachineClose(unsigned char byte);

struct termios oldtio,newtio;
int fd;
linkLayer connection;

//control switch
unsigned char c = 0x00;

//statemachineRead
int size=0;
unsigned char response[255];
unsigned char bcc2check = 0x00;


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
        llread(buf);
        printf("Got out of first llread\n");
        llread(buf);
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
    newtio.c_cc[VMIN]     = 1;   /* blocking read until 5 chars received */

    /*
    VTIME e VMIN devem ser alterados de forma a proteger com um temporizador a
    leitura do(s) próximo(s) caracter(es)
    */


    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd,TCSANOW,&newtio) == -1) {
        perror("tcsetattr");
        return -1;
    }

    printf("New termios structure set\nWaiting for SET\n");

    while (STOP==FALSE) 
    {   /* loop for input */
        res = read(fd,buf,1); 
        statemachine(buf[0]);
        printf("STATE IS: %d\n",state);
        if(state==5)
        {
            buf[0] = 0x5c;
            buf[1] = 0x03;
            buf[2] = 0x06;
            buf[3]=buf[1]^buf[2];
            buf[4]=0x5c;
            res = write(fd,buf,5);
            STOP=TRUE;
        }
        
    }
    state=START;
    tcsetattr(fd,TCSANOW,&oldtio);
    STOP=FALSE;
    return (1);
}

int llread(unsigned char *buffer)
{
    printf("Entered read\n");
    if(STOP==TRUE) printf("Stop is true\n");
    else printf("Stop is false\n");
    unsigned char dado;
    while(STOP==FALSE)
    {
        read(fd,&dado,1);
        //printf("Read (currently in state %d): %x\n",state,dado);
        //if(state==BCCOK && dado==0x5b) ByteDestuffing(dado);
        //else 

        stateMachineClose(dado);
        if(discState==STOP_STATE_MACHINE) llclose(connection);

        statemachineRead(dado);

        if(state==STOP_STATE_MACHINE)
        {
            for(int j=0;j<size;j++) buffer[j] = response[j];
            printf("End of frame\nSending ACK...\n");
            unsigned char ack[5];
            ack[0] = 0x5c;
            ack[1] = 0x03;
            if(c==0x01) ack[2] = 0x11;
            else ack[2] = 0x01;
            ack[3] = ack[1]^ack[2];
            ack[4] = 0x05c;
            write(fd,ack,5);
            state=START;
            discState=START;
            return size;
        } 
    }
    return -1;
}

void statemachineRead(unsigned char byte)
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
            if(byte==c) state = CRCV;
            else if(byte==0x5c) state = FLAGRCV;
            else state = START;
            break;
        case CRCV:
            if(byte==(0x01^c)) state = BCCOK;
            else if(byte==0x5c) state = FLAGRCV;
            else state = START; 
            break;  
        case BCCOK:  
            if(byte==0x5c)
            {
                for(int i=0;i<size-1;i++) bcc2check=bcc2check^response[i];

                if (response[size-1]==bcc2check) state = STOP_STATE_MACHINE;
                if(c==0x01) c=0x00;
                else c=0x01;

                break;
            }
            response[size]=byte;
            size++;
            break;
    }
}

void ByteDestuffing(unsigned char byte)
{
    unsigned char check;
    read(fd,&check,1);
    if(check==0x7c) 
    {
        response[size]=0x5c;
        size++;
    }
    else 
    {
        statemachineRead(byte);
        statemachineRead(check);
    }
    
}

int llclose(linkLayer connectionParameters)
{
    printf("Entered llclose\n");
    discState=START;
    unsigned char disc[5];
    disc[0] = 0x5c;
    disc[0] = 0x03;
    disc[0] = 0x0a;
    disc[0] = 0x0a^0x03;
    disc[0] = 0x5c;
    write(fd,disc,5);
    while(discState!=STOP_STATE_MACHINE)
    {
        read(fd,disc,1);
        
        stateMachineClose(disc[0]);
    }
    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    printf("LLCLOSED\n");
    return 1;
}

void stateMachineClose (unsigned char byte)
{
    printf("In state %d checking : %x\n",discState,byte);
    switch (discState)
    {
        case START:
            if(byte==0x5c) discState = FLAGRCV;
            break;
        case FLAGRCV:
            if(byte==0x01) discState = ARCV;
            else if(byte==0x5c) discState = FLAGRCV;
            else discState = START; 
            break;
        case ARCV:
            if(byte==0x0a || byte==0x06) discState = CRCV;
            else if(byte==0x5c) discState = FLAGRCV;
            else discState = START;
            break;
        case CRCV:
            if(byte==(0x01^0x0a)||byte==(0x01^0x06)) discState = BCCOK;
            else if(byte==0x5c) discState = FLAGRCV;
            else discState = START; 
            break;  
        case BCCOK: 
            if(byte==0x5c) discState = STOP_STATE_MACHINE; 
            break;
    }
}
