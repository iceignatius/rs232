/******************************************************************************
 * Name      : RS232
 * Purpose   : Data exchange through RS-232 serial port
 * Author    : 王文佑
 * Created   : 2014.10.21
 * Licence   : ZLib Licence
 * Reference : http://www.openfoundry.org/of/projects/2419
 ******************************************************************************/
#ifndef _RS232_H_
#define _RS232_H_

#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
#include <string>
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef enum rs232_parity_t
{
    RS232_PARITY_NONE = 0,
    RS232_PARITY_ODD,
    RS232_PARITY_EVEN,
} rs232_parity_t;

typedef struct rs232_t
{
#if   defined(__linux__)
    int   port;
#elif defined(_WIN32)
    void *port;
#else
    #error No implementation on this platform!
#endif
} rs232_t;

/*
 * Notification :
 *
 * Users should set their configuration to the object after it opened if they have their special configuration,
 * or the configuration will be as below :
 *
 *   Baud rate : 9600
 *   Data bits : 8
 *   Stop bits : 1
 *   Parity    : None
 *   DTR       : Disabled
 *   RTS       : Disabled
 */

void rs232_init        (      rs232_t *obj);
void rs232_deinit      (      rs232_t *obj);

bool rs232_open        (      rs232_t *obj, const char *device);
void rs232_close       (      rs232_t *obj);
bool rs232_is_opened   (const rs232_t *obj);

int  rs232_get_baud    (const rs232_t *obj);
int  rs232_get_databits(const rs232_t *obj);
int  rs232_get_stopbits(const rs232_t *obj);
int  rs232_get_parity  (const rs232_t *obj);
bool rs232_get_dtr     (const rs232_t *obj);
bool rs232_get_rts     (const rs232_t *obj);

bool rs232_set_baud    (      rs232_t *obj, int  baud);
bool rs232_set_databits(      rs232_t *obj, int  databits);
bool rs232_set_stopbits(      rs232_t *obj, int  stopbits);
bool rs232_set_parity  (      rs232_t *obj, int  parity);
bool rs232_set_dtr     (      rs232_t *obj, bool enable_dtr);
bool rs232_set_rts     (      rs232_t *obj, bool enable_rts);

bool rs232_flush       (      rs232_t *obj);
int  rs232_send        (      rs232_t *obj, const void *data, size_t size);
int  rs232_receive     (      rs232_t *obj, void *buf, size_t size);

#ifdef __cplusplus
}  // extern "C"
#endif

// C++ Wrapper
#ifdef __cplusplus
class TRS232 : protected rs232_t
{
public:
    static const int ParityNone = RS232_PARITY_NONE;
    static const int ParityOdd  = RS232_PARITY_ODD;
    static const int ParityEven = RS232_PARITY_EVEN;

public:
    TRS232()  { rs232_init  (this); }
    ~TRS232() { rs232_deinit(this); }
private:
    TRS232(const TRS232 &Src);             // Not allowed to use!
    TRS232& operator=(const TRS232 &Src);  // Not allowed to use!

public:

    bool Open(const std::string &Device)     { return rs232_open        (this, Device.c_str()); }
    void Close()                             {        rs232_close       (this); }
    bool IsOpened()                    const { return rs232_is_opened   (this); }

    int  GetBaud()                     const { return rs232_get_baud    (this); }
    int  GetDataBits()                 const { return rs232_get_databits(this); }
    int  GetStopBits()                 const { return rs232_get_stopbits(this); }
    int  GetParity()                   const { return rs232_get_parity  (this); }
    bool GetDTR()                      const { return rs232_get_dtr     (this); }
    bool GetRTS()                      const { return rs232_get_rts     (this); }

    bool SetBaud(int Baud)                   { return rs232_set_baud    (this, Baud); }
    bool SetDataBits(int DataBits)           { return rs232_set_databits(this, DataBits); }
    bool SetStopBits(int StopBits)           { return rs232_set_stopbits(this, StopBits); }
    bool SetParity(int Parity)               { return rs232_set_parity  (this, Parity); }
    bool SetDTR(bool Enable)                 { return rs232_set_dtr     (this, Enable); }
    bool SetRTS(bool Enable)                 { return rs232_set_rts     (this, Enable); }

    bool Flush()                             { return rs232_flush       (this); }
    int  Send(const void *Data, size_t Size) { return rs232_send        (this, Data, Size); }
    int  Receive(void *Buffer, size_t Size)  { return rs232_receive     (this, Buffer, Size); }

};
#endif

#endif
