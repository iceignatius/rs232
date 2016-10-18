#include <assert.h>

#ifdef __linux__
#include <unistd.h>
#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#endif

#ifdef _WIN32
#include <windows.h>
#endif

#include "rs232.h"

#if   defined(__linux__)
    static const int   invalid_port = -1;
#elif defined(_WIN32)
    static void* const invalid_port = INVALID_HANDLE_VALUE;
#else
    #error No implementation on this platform!
#endif

//------------------------------------------------------------------------------
#ifdef __linux__
static
bool linux_initialize_port_termios(int port)
{
    bool succeed = false;
    do
    {
        struct termios term;
        if( tcgetattr(port, &term) ) break;

        term.c_iflag      = IGNPAR;
        term.c_oflag      = 0;
        term.c_lflag      = 0;
        term.c_cflag      = CREAD                 |
                            CLOCAL                |
                            B9600                 |
                            CS8                   |
                            (0/*stopbits for 1*/) |
                            (0/*parity for none*/);
        term.c_cc[VEOF]   = _POSIX_VDISABLE;
        term.c_cc[VEOL]   = _POSIX_VDISABLE;
        term.c_cc[VERASE] = _POSIX_VDISABLE;
        term.c_cc[VINTR]  = _POSIX_VDISABLE;
        term.c_cc[VKILL]  = _POSIX_VDISABLE;
        term.c_cc[VMIN]   = _POSIX_VDISABLE;
        term.c_cc[VQUIT]  = _POSIX_VDISABLE;
        term.c_cc[VSTART] = _POSIX_VDISABLE;
        term.c_cc[VSTOP]  = _POSIX_VDISABLE;
        term.c_cc[VSUSP]  = _POSIX_VDISABLE;
        term.c_cc[VTIME]  = _POSIX_VDISABLE;

        if( tcsetattr(port, TCSANOW, &term) ) break;

        succeed = true;
    } while(false);

    return succeed;
}
//------------------------------------------------------------------------------
static
bool linux_initialize_port_ioctl(int port)
{
    bool succeed = false;
    do
    {
        int flag;
        if( ioctl(port, TIOCMGET, &flag) ) break;

        flag &= ~TIOCM_DTR;
        flag &= ~TIOCM_RTS;

        if( ioctl(port, TIOCMSET, &flag) ) break;

        succeed = true;
    } while(false);

    return succeed;
}
//------------------------------------------------------------------------------
static
bool initialize_port_config(int port)
{
    bool succeed = false;
    do
    {
        if( !linux_initialize_port_termios(port) ) break;
        if( !linux_initialize_port_ioctl  (port) ) break;

        succeed = true;
    } while(false);

    return succeed;
}
#endif
//------------------------------------------------------------------------------
#ifdef _WIN32
static
bool windows_initialize_port_timeouts(HANDLE port)
{
    bool succeed = false;
    do
    {
        COMMTIMEOUTS timeouts;
        if( !GetCommTimeouts(port, &timeouts) ) break;

        timeouts.ReadIntervalTimeout         = MAXDWORD;
        timeouts.ReadTotalTimeoutMultiplier  = 0;
        timeouts.ReadTotalTimeoutConstant    = 0;
        timeouts.WriteTotalTimeoutMultiplier = 0;
        timeouts.WriteTotalTimeoutConstant   = 0;

        if( !SetCommTimeouts(port, &timeouts) ) break;

        succeed = true;
    } while(false);

    return succeed;
}
//------------------------------------------------------------------------------
static
bool windows_initialize_port_dcb(HANDLE port)
{
    bool succeed = false;
    do
    {
        DCB dcb;
        dcb.DCBlength = sizeof(dcb);
        if( !GetCommState(port, &dcb) ) break;

        dcb.BaudRate     = CBR_9600;
        dcb.fOutxCtsFlow = FALSE;
        dcb.fOutxDsrFlow = FALSE;
        dcb.fDtrControl  = DTR_CONTROL_DISABLE;
        dcb.fOutX        = FALSE;
        dcb.fInX         = FALSE;
        dcb.fRtsControl  = RTS_CONTROL_DISABLE;
        dcb.ByteSize     = 8;
        dcb.Parity       = NOPARITY;
        dcb.StopBits     = ONESTOPBIT;

        if( !SetCommState(port, &dcb) ) break;

        succeed = true;
    } while(false);

    return succeed;
}
//------------------------------------------------------------------------------
static
bool initialize_port_config(HANDLE port)
{
    bool succeed = false;
    do
    {
        if( !windows_initialize_port_timeouts(port) ) break;
        if( !windows_initialize_port_dcb     (port) ) break;

        succeed = true;
    } while(false);

    return succeed;
}
#endif
//------------------------------------------------------------------------------
void rs232_init(rs232_t *obj)
{
    /**
     * Initialize a RS-232 object
     * @obj : The RS-232 object to be operated.
     */
    assert( obj );

    obj->port = invalid_port;
}
//------------------------------------------------------------------------------
void rs232_deinit(rs232_t *obj)
{
    /**
     * Finish a RS-232 object
     * @obj : The RS-232 object to be operated.
     */
    assert( obj );

    rs232_close(obj);
}
//------------------------------------------------------------------------------
bool rs232_open(rs232_t *obj, const char *device)
{
    /**
     * Open a RS-232 serial port
     * @obj    : The RS-232 object to be operated.
     * @device : The name of the serial port device, including the file path if it does have.
     *           The device name depend on different systems,
     *           such like : "/dev/ttyS0", /dev/ttyACM0", "/dev/ttyUSB0" on Linux;
     *           or "COM1", "COM2" on Windows.
     * @return : TRUE if succeed; and FALSE if failed.
     */
    assert( obj );

    if( !device ) return false;

    bool succeed = false;
    do
    {
        rs232_close(obj);

#if   defined(__linux__)
        obj->port = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
        if( obj->port == invalid_port ) break;
#elif defined(_WIN32)
        // - There is no need to translate device name and call Unicode function,
        //   because all devices name are in ASCII range.
        obj->port = CreateFileA(device,
                                GENERIC_READ | GENERIC_WRITE,
                                FILE_SHARE_READ | FILE_SHARE_WRITE,
                                NULL,
                                OPEN_EXISTING,
                                0,
                                NULL);
        if( obj->port == invalid_port ) break;
#else
    #error No implementation on this platform!
#endif

        if( !initialize_port_config(obj->port) ) break;
        if( !rs232_flush(obj) ) break;

        succeed = true;
    } while(false);

    if( !succeed ) rs232_close(obj);

    return succeed;
}
//------------------------------------------------------------------------------
void rs232_close(rs232_t *obj)
{
    /**
     * Close a RS-232 serial port
     * @obj : The RS-232 object to be operated.
     */
    assert( obj );

    if( obj->port == invalid_port ) return;

#if   defined(__linux__)
    close(obj->port);
#elif defined(_WIN32)
    CloseHandle(obj->port);
#else
    #error No implementation on this platform!
#endif

    obj->port = invalid_port;
}
//------------------------------------------------------------------------------
bool rs232_is_opened(const rs232_t *obj)
{
    /**
     * Check is the port opened
     * @obj    : The RS-232 object to be operated.
     * @return : TRUE if the port is opened; and FALSE if not.
     */
    assert( obj );

    return !( obj->port == invalid_port );
}
//------------------------------------------------------------------------------
#ifdef __linux__
static
int linux_baud_flag_to_value(speed_t flag)
{
    switch( flag )
    {
    case B0       :  return 0;
    case B50      :  return 50;
    case B75      :  return 75;
    case B110     :  return 110;
    case B134     :  return 134;
    case B150     :  return 150;
    case B200     :  return 200;
    case B300     :  return 300;
    case B600     :  return 600;
    case B1200    :  return 1200;
    case B1800    :  return 1800;
    case B2400    :  return 2400;
    case B4800    :  return 4800;
    case B9600    :  return 9600;
    case B19200   :  return 19200;
    case B38400   :  return 38400;
    case B57600   :  return 57600;
    case B115200  :  return 115200;
    case B230400  :  return 230400;
    case B460800  :  return 460800;
    case B500000  :  return 500000;
    case B576000  :  return 576000;
    case B921600  :  return 921600;
    case B1000000 :  return 1000000;
    case B1152000 :  return 1152000;
    case B1500000 :  return 1500000;
    case B2000000 :  return 2000000;
    case B2500000 :  return 2500000;
    case B3000000 :  return 3000000;
    case B3500000 :  return 3500000;
    case B4000000 :  return 4000000;
    default       :  return -1;
    }
}
#endif
//------------------------------------------------------------------------------
int rs232_get_baud(const rs232_t *obj)
{
    /**
     * Get baud rate
     * @obj    : The RS-232 object to be operated.
     * @return : Baud rate in integer value if succeed;
     *           and -1 if failed.
     * Remarks : This function will be failed if the port was not opened.
     */
    assert( obj );

#if   defined(__linux__)
    struct termios term;
    if( tcgetattr(obj->port, &term) ) return -1;

    speed_t ispeed = cfgetispeed(&term);
    speed_t ospeed = cfgetospeed(&term);
    if( ispeed != ospeed ) return -1;

    return linux_baud_flag_to_value(ispeed);
#elif defined(_WIN32)
    DCB dcb;
    if( !GetCommState(obj->port, &dcb) ) return -1;

    return dcb.BaudRate;
#else
    #error No implementation on this platform!
#endif
}
//------------------------------------------------------------------------------
#ifdef __linux__
static
int linux_databits_flag_to_value(tcflag_t flag)
{
    switch( flag )
    {
    case CS5 :  return  5;
    case CS6 :  return  6;
    case CS7 :  return  7;
    case CS8 :  return  8;
    default  :  return -1;
    }
}
#endif
//------------------------------------------------------------------------------
int rs232_get_databits(const rs232_t *obj)
{
    /**
     * Get data bits
     * @obj    : The RS-232 object to be operated.
     * @return : Data bits in integer value if succeed;
     *           and -1 if failed.
     * Remarks : This function will be failed if the port was not opened.
     */
    assert( obj );

#if   defined(__linux__)
    struct termios term;
    if( tcgetattr(obj->port, &term) ) return -1;

    return linux_databits_flag_to_value( term.c_cflag & CSIZE );
#elif defined(_WIN32)
    DCB dcb;
    if( !GetCommState(obj->port, &dcb) ) return -1;

    return dcb.ByteSize;
#else
    #error No implementation on this platform!
#endif
}
//------------------------------------------------------------------------------
#ifdef _WIN32
static
int windows_stopbits_flag_to_value(BYTE flag)
{
    switch( flag )
    {
    case ONESTOPBIT  :  return  1;
    case TWOSTOPBITS :  return  2;
    default          :  return -1;
    }
}
#endif
//------------------------------------------------------------------------------
int rs232_get_stopbits(const rs232_t *obj)
{
    /**
     * Get stop bits
     * @obj    : The RS-232 object to be operated.
     * @return : Stop bits in integer value if succeed;
     *           and -1 if failed.
     * Remarks : This function will be failed if the port was not opened.
     */
    assert( obj );

#if   defined(__linux__)
    struct termios term;
    if( tcgetattr(obj->port, &term) ) return -1;

    return ( term.c_cflag & CSTOPB )?( 2 ):( 1 );
#elif defined(_WIN32)
    DCB dcb;
    if( !GetCommState(obj->port, &dcb) ) return -1;

    return windows_stopbits_flag_to_value(dcb.StopBits);
#else
    #error No implementation on this platform!
#endif
}
//------------------------------------------------------------------------------
int rs232_get_parity(const rs232_t *obj)
{
    /**
     * Get parity
     * @obj    : The RS-232 object to be operated.
     * @return : One of the values defined in rs232_parity_t if succeed;
     *           and -1 if failed.
     * Remarks : This function will be failed if the port was not opened.
     */
    assert( obj );

#if   defined(__linux__)
    struct termios term;
    if( tcgetattr(obj->port, &term) ) return false;

    if( !( term.c_cflag & PARENB ) ) return RS232_PARITY_NONE;
    return ( term.c_cflag & PARODD )?( RS232_PARITY_ODD ):( RS232_PARITY_EVEN );
#elif defined(_WIN32)
    DCB dcb;
    if( !GetCommState(obj->port, &dcb) ) return -1;

    switch( dcb.Parity )
    {
    case NOPARITY   :  return RS232_PARITY_NONE;
    case ODDPARITY  :  return RS232_PARITY_ODD;
    case EVENPARITY :  return RS232_PARITY_EVEN;
    default         :  return -1;
    }
#else
    #error No implementation on this platform!
#endif
}
//------------------------------------------------------------------------------
bool rs232_get_dtr(const rs232_t *obj)
{
    /**
     * Check if DTR enabled
     * @obj    : The RS-232 object to be operated.
     * @return : TRUE if DTR enabled; and FALSE if DTR disabled or function failed.
     * Remarks : This function will be failed if the port was not opened.
     */
    assert( obj );

#if   defined(__linux__)
    int flag;
    if( ioctl(obj->port, TIOCMGET, &flag) ) return false;

    return flag & TIOCM_DTR;
#elif defined(_WIN32)
    DCB dcb;
    if( !GetCommState(obj->port, &dcb) ) return false;

    return !( dcb.fDtrControl == DTR_CONTROL_DISABLE );
#else
    #error No implementation on this platform!
#endif
}
//------------------------------------------------------------------------------
bool rs232_get_rts(const rs232_t *obj)
{
    /**
     * Check if RTS enabled
     * @obj    : The RS-232 object to be operated.
     * @return : TRUE if RTS enabled; and FALSE if RTS disabled or function failed.
     * Remarks : This function will be failed if the port was not opened.
     */
    assert( obj );

#if   defined(__linux__)
    int flag;
    if( ioctl(obj->port, TIOCMGET, &flag) ) return false;

    return flag & TIOCM_RTS;
#elif defined(_WIN32)
    DCB dcb;
    if( !GetCommState(obj->port, &dcb) ) return false;

    return !( dcb.fRtsControl == RTS_CONTROL_DISABLE );
#else
    #error No implementation on this platform!
#endif
}
//------------------------------------------------------------------------------
#ifdef __linux__
static
speed_t linux_baud_value_to_flag(int value)
{
    switch( value )
    {
    case 0       :  return B0;
    case 50      :  return B50;
    case 75      :  return B75;
    case 110     :  return B110;
    case 134     :  return B134;
    case 150     :  return B150;
    case 200     :  return B200;
    case 300     :  return B300;
    case 600     :  return B600;
    case 1200    :  return B1200;
    case 1800    :  return B1800;
    case 2400    :  return B2400;
    case 4800    :  return B4800;
    case 9600    :  return B9600;
    case 19200   :  return B19200;
    case 38400   :  return B38400;
    case 57600   :  return B57600;
    case 115200  :  return B115200;
    case 230400  :  return B230400;
    case 460800  :  return B460800;
    case 500000  :  return B500000;
    case 576000  :  return B576000;
    case 921600  :  return B921600;
    case 1000000 :  return B1000000;
    case 1152000 :  return B1152000;
    case 1500000 :  return B1500000;
    case 2000000 :  return B2000000;
    case 2500000 :  return B2500000;
    case 3000000 :  return B3000000;
    case 3500000 :  return B3500000;
    case 4000000 :  return B4000000;
    default       :  return -1;
    }
}
#endif
//------------------------------------------------------------------------------
#ifdef _WIN32
static
bool windows_baud_check_available(int baud)
{
    switch( baud )
    {
    case CBR_110:
    case CBR_300:
    case CBR_600:
    case CBR_1200:
    case CBR_2400:
    case CBR_4800:
    case CBR_9600:
    case CBR_14400:
    case CBR_19200:
    case CBR_38400:
    case CBR_56000:
    case CBR_57600:
    case CBR_115200:
    case CBR_128000:
    case CBR_256000:
        return true;

    default:
        return false;

    }
}
#endif
//------------------------------------------------------------------------------
bool rs232_set_baud(rs232_t *obj, int baud)
{
    /**
     * Set baud rate
     * @obj    : The RS-232 object to be operated.
     * @baud   : The baud rate in integer value.
     * @return : TRUE if succeed; and FALSE if failed.
     * Remarks : This function will be failed if the port was not opened.
     */
    assert( obj );

#if   defined(__linux__)
    struct termios term;
    if( tcgetattr(obj->port, &term) ) return false;

    speed_t flag = linux_baud_value_to_flag(baud);
    if( flag & !CBAUD ) return false;

    if( cfsetispeed(&term, flag) ) return false;
    if( cfsetospeed(&term, flag) ) return false;

    return !tcsetattr(obj->port, TCSANOW, &term);
#elif defined(_WIN32)
    DCB dcb;
    if( !GetCommState(obj->port, &dcb) ) return false;

    if( !windows_baud_check_available(baud) ) return false;
    dcb.BaudRate = baud;

    return SetCommState(obj->port, &dcb);
#else
    #error No implementation on this platform!
#endif
}
//------------------------------------------------------------------------------
#ifdef __linux__
static
tcflag_t linux_databits_value_to_flag(int value)
{
    switch( value )
    {
    case 5  : return CS5;
    case 6  : return CS6;
    case 7  : return CS7;
    case 8  : return CS8;
    default : return -1;
    }
}
#endif
//------------------------------------------------------------------------------
#ifdef _WIN32
static
bool windows_databits_check_available(int databits)
{
    return ( 5 <= databits )&&( databits <= 8 );
}
#endif
//------------------------------------------------------------------------------
bool rs232_set_databits(rs232_t *obj, int databits)
{
    /**
     * Set data bits
     * @obj      : The RS-232 object to be operated.
     * @databits : Data bits in integer value, and values abailable from 5 to 8 only.
     * @return   : TRUE if succeed; and FALSE if failed.
     * Remarks   : This function will be failed if the port was not opened.
     */
    assert( obj );

#if   defined(__linux__)
    struct termios term;
    if( tcgetattr(obj->port, &term) ) return false;

    tcflag_t flag = linux_databits_value_to_flag(databits);
    if( flag & ~CSIZE ) return false;

    term.c_cflag &= ~CSIZE;
    term.c_cflag |= flag;

    return !tcsetattr(obj->port, TCSANOW, &term);
#elif defined(_WIN32)
    DCB dcb;
    if( !GetCommState(obj->port, &dcb) ) return false;

    if( !windows_databits_check_available(databits) ) return false;
    dcb.ByteSize = databits;

    return SetCommState(obj->port, &dcb);
#else
    #error No implementation on this platform!
#endif
}
//------------------------------------------------------------------------------
#ifdef _WIN32
static
bool windows_stopbits_check_available(int stopbits)
{
    return ( stopbits == 1 )||( stopbits == 2 );
}
//------------------------------------------------------------------------------
static
BYTE windows_stopbits_value_to_flag(int stopbits)
{
    switch( stopbits )
    {
    case 1  :  return ONESTOPBIT;
    case 2  :  return TWOSTOPBITS;
    default :  return -1;
    }
}
#endif
//------------------------------------------------------------------------------
bool rs232_set_stopbits(rs232_t *obj, int stopbits)
{
    /**
     * Set stop bits
     * @obj      : The RS-232 object to be operated.
     * @stopbits : Stop bits in integer value, and 1 and 2 are available only.
     * @return   : TRUE if succeed; and FALSE if failed.
     * Remarks   : This function will be failed if the port was not opened.
     */
    assert( obj );

#if   defined(__linux__)
    struct termios term;
    if( tcgetattr(obj->port, &term) ) return false;

    switch( stopbits )
    {
    case 1:
        term.c_cflag &= ~CSTOPB;
        break;

    case 2:
        term.c_cflag |= CSTOPB;
        break;

    default:
        return false;

    }

    return !tcsetattr(obj->port, TCSANOW, &term);
#elif defined(_WIN32)
    DCB dcb;
    if( !GetCommState(obj->port, &dcb) ) return false;

    if( !windows_stopbits_check_available(stopbits) ) return false;
    dcb.StopBits = windows_stopbits_value_to_flag(stopbits);

    return SetCommState(obj->port, &dcb);
#else
    #error No implementation on this platform!
#endif
}
//------------------------------------------------------------------------------
bool rs232_set_parity(rs232_t *obj, int parity)
{
    /**
     * Set parity
     * @obj    : The RS-232 object to be operated.
     * @parity : One if the values defined in rs232_parity_t.
     * @return : TRUE if succeed; and FALSE if failed.
     * Remarks : This function will be failed if the port was not opened.
     */
    assert( obj );

#if   defined(__linux__)
    struct termios term;
    if( tcgetattr(obj->port, &term) ) return false;

    switch( parity )
    {
    case RS232_PARITY_NONE:
        term.c_cflag &= ~PARENB;
        term.c_cflag &= ~PARODD;
        break;

    case RS232_PARITY_ODD:
        term.c_cflag |= PARENB;
        term.c_cflag |= PARODD;
        break;

    case RS232_PARITY_EVEN:
        term.c_cflag |= PARENB;
        term.c_cflag &= ~PARODD;
        break;

    default:
        return false;

    }

    return !tcsetattr(obj->port, TCSANOW, &term);
#elif defined(_WIN32)
    DCB dcb;
    if( !GetCommState(obj->port, &dcb) ) return false;

    switch( parity )
    {
    case RS232_PARITY_NONE:
        dcb.Parity = NOPARITY;
        break;

    case RS232_PARITY_ODD:
        dcb.Parity = ODDPARITY;
        break;

    case RS232_PARITY_EVEN:
        dcb.Parity = EVENPARITY;
        break;

    default:
        return false;

    }

    return SetCommState(obj->port, &dcb);
#else
    #error No implementation on this platform!
#endif
}
//------------------------------------------------------------------------------
bool rs232_set_dtr(rs232_t *obj, bool enable_dtr)
{
    /**
     * Set DTR
     * @obj        : The RS-232 object to be operated.
     * @enable_dtr : TRUE to enable DTR mode, and FALSE to disable it.
     * @return     : TRUE if succeed; and FALSE if failed.
     * Remarks     : This function will be failed if the port was not opened.
     */
    assert( obj );

#if   defined(__linux__)
    int flag;
    if( ioctl(obj->port, TIOCMGET, &flag) ) return false;

    if( enable_dtr ) flag |=  TIOCM_DTR;
    else             flag &= ~TIOCM_DTR;

    return !ioctl(obj->port, TIOCMSET, &flag);
#elif defined(_WIN32)
    DCB dcb;
    if( !GetCommState(obj->port, &dcb) ) return false;

    dcb.fDtrControl = enable_dtr ? DTR_CONTROL_ENABLE : DTR_CONTROL_DISABLE;

    return SetCommState(obj->port, &dcb);
#else
    #error No implementation on this platform!
#endif
}
//------------------------------------------------------------------------------
bool rs232_set_rts(rs232_t *obj, bool enable_rts)
{
    /**
     * Set RTS
     * @obj        : The RS-232 object to be operated.
     * @enable_rts : TRUE to enable RTS mode, and FALSE to disable it.
     * @return     : TRUE if succeed; and FALSE if failed.
     * Remarks     : This function will be failed if the port was not opened.
     */
    assert( obj );

#if   defined(__linux__)
    int flag;
    if( ioctl(obj->port, TIOCMGET, &flag) ) return false;

    if( enable_rts ) flag |=  TIOCM_RTS;
    else             flag &= ~TIOCM_RTS;

    return !ioctl(obj->port, TIOCMSET, &flag);
#elif defined(_WIN32)
    DCB dcb;
    if( !GetCommState(obj->port, &dcb) ) return false;

    dcb.fRtsControl = enable_rts ? RTS_CONTROL_ENABLE : RTS_CONTROL_DISABLE;

    return SetCommState(obj->port, &dcb);
#else
    #error No implementation on this platform!
#endif
}
//------------------------------------------------------------------------------
bool rs232_flush(rs232_t *obj)
{
    /**
     * Flush and clean IO buffer
     * @obj    : The RS-232 object to be operated.
     * @return : TRUE if succeed; and FALSE if failed.
     */
    assert( obj );

    bool res = false;
    do
    {
        if( !rs232_is_opened(obj) ) break;

#if   defined(__linux__)
        if( tcflush(obj->port, TCIOFLUSH) ) break;
#elif defined(_WIN32)
        if( !FlushFileBuffers(obj->port) ) break;
        if( !PurgeComm(obj->port, PURGE_RXABORT | PURGE_RXCLEAR | PURGE_TXABORT | PURGE_TXCLEAR) ) break;
#else
    #error No implementation on this platform!
#endif

        res = true;
    } while(false);

    // Clear receive buffer for insurance.
    char buf[64];
    while( 0 < rs232_receive(obj, buf, sizeof(buf)) )
    {}

    return res;
}
//------------------------------------------------------------------------------
int rs232_send(rs232_t *obj, const void *data, size_t size)
{
    /**
     * Send data
     * @obj    : The RS-232 object to be operated.
     * @data   : Data to send.
     * @size   : Size of data to send.
     * @return : Size of data sent, or -1 if failed.
     */
    assert( obj );

    if( !rs232_is_opened(obj) ) return -1;
    if( !data || !size ) return 0;

#if   defined(__linux__)
    int sentsz = write(obj->port, data, size);
    if( sentsz < 0 ) sentsz = ( errno == EAGAIN )?( 0 ):( -1 );
    return sentsz;
#elif defined(_WIN32)
    DWORD sentsz;
    return ( WriteFile(obj->port, data, size, &sentsz, NULL) )?( sentsz ):( -1 );
#else
    #error No implementation on this platform!
#endif
}
//------------------------------------------------------------------------------
int rs232_receive(rs232_t *obj, void *buf, size_t size)
{
    /**
     * Receive data
     * @obj    : The RS-232 object to be operated.
     * @buf    : A buffer to receive data.
     * @size   : Size of data to receive.
     * @return : Size of data received, or -1 if failed.
     * Remarks : This function will return immediately without waiting whether the IO buffer have data or not.
     */
    assert( obj );

    if( !rs232_is_opened(obj) ) return -1;
    if( !buf || !size ) return 0;

#if   defined(__linux__)
    int recsz = read(obj->port, buf, size);
    if( recsz < 0 ) recsz = ( errno == EAGAIN )?( 0 ):( -1 );
    return recsz;
#elif defined(_WIN32)
    DWORD recsz;
    return ( ReadFile(obj->port, buf, size, &recsz, NULL) )?( recsz ):( -1 );
#else
    #error No implementation on this platform!
#endif
}
//------------------------------------------------------------------------------
