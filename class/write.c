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

//LOGIC
#define FALSE 0
#define TRUE 1

//state machine
volatile int STOP=FALSE;
int state=0;
#define START 0
#define FLAGRCV 1
#define ARCV 2
#define CRCV 3
#define BCCOK 4
#define STOP_STATE_MACHINE 5
int discState=0;

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
void statemachineAck(unsigned char byte);
void ByteStuffing (unsigned char *vec, int pos);
int llclose(linkLayer connectionParameters);
int stateMachineClose(unsigned char byte);



int fd;
struct termios oldtio,newtio;

linkLayer connection;
//control switch
unsigned char c = 0x00;

int main(int argc, char** argv)
{
    char buf[255];
    
    if ( (argc < 2) ||
         ((strcmp("/dev/ttyS0", argv[1])!=0) &&
          (strcmp("/dev/ttyS1", argv[1])!=0) )) {
        printf("Usage:\tnserial SerialPort\n\tex: nserial /dev/ttyS1\n");
        exit(1);
    }

    strcpy(connection.serialPort,argv[1]);
    connection.role = 0;
    connection.baudRate = BAUDRATE;
    connection.numTries=3;
    connection.timeOut = TIMEOUT_DEFAULT;

    int success = llopen(connection);

    if(success)
    {
        //TESTING
        strcpy(buf, "SIUU (write)");
        int res = llwrite(buf,13);

        printf("%d\n",res);
        printf("LLCLOSING...\n");
        llclose(connection);
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
            if(byte==0x03) state = ARCV;
            else if(byte==0x5c) state = FLAGRCV;
            else state = START;   
            break;
        case ARCV:
            if(byte==0x06) state = CRCV;
            else if(byte==0x5c) state = FLAGRCV;
            else state = START;
            break;
        case CRCV:
            if(byte==(0x03^0x06)) state = BCCOK;
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
    char setter[255];

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
        exit(-1);
    }

    printf("New termios structure set\n");

    setter[0]=0x5c;
    setter[1]=0x01;
    setter[2]=0x07;
    setter[3]=setter[1]^setter[2];
    setter[4]=setter[0];

    write(fd,setter,5);
    
    while (STOP==FALSE) {       /* loop for input */
        res = read(fd,buf,1); 

        statemachine(buf[0]); 
        printf("STATE IS: %d\n",state);
        if(state==5) STOP=TRUE;
        
    }
    state=START;
    tcsetattr(fd,TCSANOW,&oldtio);
    return (1);
}
int llwrite(unsigned char *buffer,int length)
{
    unsigned char send[255];

    send[0]=0x5c;
    
    if (connection.role==0) send[1]=0x01;
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
        printf("Passing %x to sender\n",buffer[i]);
    }

    send[4+length]=bcc2;
    send[4+length+1]=0x5c;


    //ByteStuffing
    //for(int i = 0;i<length;i++) if(send[4+i]==0x5c) ByteStuffing(send,i);

    for(int j = 0;j<4+length+2;j++) printf("Wrote: %x\n",send[j]);
    
    int tamanho = write(fd,send,4+length+2);

    unsigned char ack;
    for(int i=0; i<5;i++) 
    {
        read(fd,&ack,1);
        printf("Read (in state: %d): %x\n",state,ack);
        statemachineAck(ack);
    }
    if(state==STOP_STATE_MACHINE)
    {
        state=START;
        printf("DEU ACK\n");
        return tamanho;
    }
    return -1;
    //talvez em vez de return, fazer read esperando pelo ACK
}
void statemachineAck(unsigned char byte)
{
    switch (state)
    {
        case START:
            if(byte==0x5c) state = FLAGRCV;
            break;
        case FLAGRCV:
            if(byte==0x03) state = ARCV;
            else if(byte==0x5c) state = FLAGRCV;
            else state = START;   
            break;
        case ARCV:
            if(c==0x00)
                if(byte==0x01) state = CRCV;
                else if(byte==0x5c) state = FLAGRCV;
                else state = START;
            if(c==0x01)
                if(byte==0x11) state = CRCV;
                else if(byte==0x5c) state = FLAGRCV;
                else state = START;
            break;
        case CRCV:
            printf("CONTROL CHECKER IS: %x\n",c);
            if(c==0x00)
                if(byte==0x02) state = BCCOK;
                else if(byte==0x5c) state = FLAGRCV;
                else state = START;
            if(c==0x01)
                if(byte==0x12) state = BCCOK;
                else if(byte==0x5c) state = FLAGRCV;
                else state = START; 
            break;  
        case BCCOK:  
            if(byte==0x5c) state = STOP_STATE_MACHINE;
            else state = START;  
            break;
            
    }
}

void ByteStuffing (unsigned char *vec, int pos)
{
    for (int i=254;i>pos+1;i--) vec[i]=vec[i-1];
    vec[pos]=0x5b;
    vec[pos+1]=0x7c;
}

int stateMachineClose(unsigned char byte)
{
    switch (discState)
    {
        case START:
            if(byte==0x5c) state = FLAGRCV;
            break;
        case FLAGRCV:
            if(byte==0x03) state = ARCV;
            else if(byte==0x5c) state = FLAGRCV;
            else state = START;   
            break;
        case ARCV:
            if(byte==0x0a) state = CRCV;
            else if(byte==0x5c) state = FLAGRCV;
            else state = START;
            break;
        case CRCV:
            if(byte==0x03^0x0a) state = BCCOK;
            else if(byte==0x5c) state = FLAGRCV;
            else state = START; 
            break;  
        case BCCOK:  
            if(byte==0x5c) state = STOP_STATE_MACHINE;
            else state = START;  
            break;
            
    }
}
int llclose(linkLayer connectionParameters)
{
    //transmitter
    printf("Entered llclose\n");
    unsigned char disc[5];
    disc[0] = 0x5c;
    disc[1] = 0x01;
    disc[2] = 0x0a;
    disc[3] = 0x01^0x0a;
    disc[4] = 0x5c;
    for(int i=0;i<5;i++) printf("Wrote: %x\n",disc[i]);
    write(fd,disc,5);
    while(discState!=STOP_STATE_MACHINE)
    {
        read(fd,disc,1);
        stateMachineClose(disc[0]);
    }
    disc[0]= 0x5c;
    disc[1] = 0x01;
    disc[2] = 0x06;
    disc[3] = 0x01^0x06;
    disc[4] = 0x5c;
    tcsetattr(fd,TCSANOW,&oldtio);
    close(fd);
    return 1;
}
