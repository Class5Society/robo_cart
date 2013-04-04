
#include "iostream"
#include "fcntl.h"
#include <termios.h>
using namespace std;

class serial
{
    private:
    int numvar;
    int hndlserial;

    public:
    int Openport();
    void Configureport(int);
    void ReadPort();
    int WritePort();
};
int serial::Openport()
{
    hndlserial = open("/dev/ttyO4",O_RDWR|O_NOCTTY|O_NDELAY);
    if(hndlserial == -1)
    {
        return 0;
    }
    else
    {
        return hndlserial;
    }
}
void serial::Configureport(int hndlport)
{
    struct termios options;

    fcntl(hndlserial, F_SETFL, FNDELAY);    /* Configure port reading */

    tcgetattr(hndlserial, &options);        /* Get the current options for the port */
    cfsetispeed(&options, B57600);          /* Set the baud rates to 9600 */
    cfsetospeed(&options, B57600);
    options.c_cflag |= (CLOCAL | CREAD);    /* Enable the receiver and set local mode */
    options.c_cflag &= ~PARENB;             /* Mask the character size to 8 bits, no parity */
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |=  CS8;                /* Select 8 data bits */
    options.c_cflag &= ~CRTSCTS;            /* Disable hardware flow control */
    options.c_lflag &= ~(ICANON | ECHO | ISIG);/* Enable data to be processed as raw input */
    tcsetattr(hndlserial, TCSANOW, &options); /* Set the new options for the port */

}

void serial::ReadPort()
{
    cout << "Reading Port..." << endl;
    int numBuff = 255;
    char szBuff[255];

    int numtemp;

    while (1)
    {
        numtemp = read(hndlserial, &szBuff, sizeof(szBuff));
        //cout << numtemp << endl;
        if (szBuff != 0)
        {
            cout << szBuff;
            szBuff[0]=0;
        }
    }

}

int main()
{
    serial O_serial;
    int hndltemp;

    hndltemp = O_serial.Openport();
    if(hndltemp == 0)
    {
        cout << "Serial Port Open - Status Fail" << endl;
    }
    else
    {
        cout << "Serial Port Open - Status Pass with handle value " << hndltemp << endl;
        O_serial.Configureport(hndltemp);
    }

    O_serial.ReadPort();
    return 0;
}

