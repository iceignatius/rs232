#include <assert.h>
#include <string.h>
#include <stdio.h>

#ifdef __linux__
#include <unistd.h>
#endif

#ifdef _WIN32
#include <windows.h>
#endif

#ifdef NDEBUG
    #error This test program must work with macro "ASSERT" enabled!
#endif

#include "rs232.h"

void sleep_awhile(void)
{
#if   defined(__linux__)
    usleep(1*1000*1000);
#elif defined(_WIN32)
    Sleep(1*1000);
#else
    #error No implementation on this platform!
#endif
}

int main(void)
{
#if   defined(__linux__)
    static const char* const device = "/dev/ttyS0";
#elif defined(_WIN32)
    static const char* const device = "COM1";
#endif

    printf("== Before test process : ==\n");
    printf("1. Please check this device \"%s\" is available.\n", device);
    printf("2. Be sure that you had connect the RS-232 port to a loop back connector.\n");
    printf("\n");

    rs232_t comm;
    rs232_init(&comm);

    bool open_result = rs232_open(&comm, device);
    printf("Open device \"%s\" : %s\n", device, ( open_result )?( "Succeed" ):( "Failed" ));
    if( !open_result ) return 1;

    // Test default configuration

    assert(  rs232_get_baud    (&comm) == 9600              );
    assert(  rs232_get_databits(&comm) == 8                 );
    assert(  rs232_get_stopbits(&comm) == 1                 );
    assert(  rs232_get_parity  (&comm) == RS232_PARITY_NONE );
    assert( !rs232_get_dtr     (&comm)                      );
    assert( !rs232_get_rts     (&comm)                      );

    // Test change configuration

    assert( rs232_set_baud    (&comm, 38400)            );
    assert( rs232_set_databits(&comm, 6)                );
    assert( rs232_set_stopbits(&comm, 2)                );
    assert( rs232_set_parity  (&comm, RS232_PARITY_ODD) );
    assert( rs232_set_dtr     (&comm, true)             );
    assert( rs232_set_rts     (&comm, true)             );

    assert( rs232_get_baud    (&comm) == 38400            );
    assert( rs232_get_databits(&comm) == 6                );
    assert( rs232_get_stopbits(&comm) == 2                );
    assert( rs232_get_parity  (&comm) == RS232_PARITY_ODD );
    assert( rs232_get_dtr     (&comm)                     );
    assert( rs232_get_rts     (&comm)                     );

    // Restore configuration

    assert( rs232_set_baud    (&comm, 9600 )             );
    assert( rs232_set_databits(&comm, 8)                 );
    assert( rs232_set_stopbits(&comm, 1)                 );
    assert( rs232_set_parity  (&comm, RS232_PARITY_NONE) );
    assert( rs232_set_dtr     (&comm, false)             );
    assert( rs232_set_rts     (&comm, false)             );

    assert(  rs232_get_baud    (&comm) == 9600              );
    assert(  rs232_get_databits(&comm) == 8                 );
    assert(  rs232_get_stopbits(&comm) == 1                 );
    assert(  rs232_get_parity  (&comm) == RS232_PARITY_NONE );
    assert( !rs232_get_dtr     (&comm)                      );
    assert( !rs232_get_rts     (&comm)                      );

    printf("Properties configured:\n");
    printf("  Baud rate : %d\n", rs232_get_baud    (&comm));
    printf("  Data bits : %d\n", rs232_get_databits(&comm));
    printf("  Stop bits : %d\n", rs232_get_stopbits(&comm));
    printf("  Parity    : %d\n", rs232_get_parity  (&comm));
    printf("  DTR       : %d\n", rs232_get_dtr     (&comm));
    printf("  RTS       : %d\n", rs232_get_rts     (&comm));

    // Send-receive test

    // - Send something
    static const char garbage[] = "garbage message";
    assert( sizeof(garbage) == rs232_send(&comm, garbage, sizeof(garbage)) );
    printf("Sent.\n");

    // - Flush
    sleep_awhile();
    assert( rs232_flush(&comm) );
    printf("Flushed.\n");

    // - Send message
    static const char message[] = "RS-232 test message";
    assert( sizeof(message) == rs232_send(&comm, message, sizeof(message)) );
    printf("Sent.\n");

    // - Receive message
    sleep_awhile();
    char buffer[1024] = {0};
    int  recsz = rs232_receive(&comm, buffer, sizeof(buffer));
    printf("Received size : %d\n", recsz);
    printf("Response message : [%s]\n", buffer);

    // - Check response
    bool echo_result = ( recsz == sizeof(message) )&&( !memcmp(buffer, message, recsz) );
    printf("Response check : %s\n", (echo_result?"Succeed.":"FAILED!"));

    rs232_deinit(&comm);

    return 0;
}
