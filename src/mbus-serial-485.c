//------------------------------------------------------------------------------
// Copyright (C) 2011, Robert Johansson, Raditex AB
// All rights reserved.
//
// rSCADA
// http://www.rSCADA.se
// info@rscada.se
//
//------------------------------------------------------------------------------

#include <unistd.h>
#include <limits.h>
#include <fcntl.h>

#include <sys/types.h>

#include <stdio.h>
#include <strings.h>

#include <errno.h>
#include <string.h>

#include <esp_log.h>

#include "mbus-serial-485.h"
#include "mbus-protocol-aux.h"
#include "mbus-protocol.h"

#define TAG "MBUS-485"

#define PACKET_BUFF_SIZE 2048
#define TX_PACKET_BUFF_SIZE 0
#define RX_PACKET_BUFF_SIZE 2048

//------------------------------------------------------------------------------
/// Set up a serial connection handle.
//------------------------------------------------------------------------------
uint32_t
mbus_serial_connect(mbus_handle *handle)
{
    mbus_serial_data *serial_data;
    
    if (handle == NULL)
        return -1;

    serial_data = (mbus_serial_data *) handle->auxdata;
    if (serial_data == NULL)
        return -1;

    //
    // create the SERIAL connection
    //

    ESP_LOGI(TAG, "Init");
    // Set config
    ESP_ERROR_CHECK(uart_param_config(serial_data->port, &(serial_data->config)));
    // Set pins
    ESP_ERROR_CHECK(uart_set_pin(serial_data->port, serial_data->tx, serial_data->rx, serial_data->rts, serial_data->cts));
    //ESP_ERROR_CHECK(uart_set_line_inverse(serial_data->port, UART_SIGNAL_RTS_INV));
    // Install driver
    ESP_ERROR_CHECK(uart_driver_install(serial_data->port, RX_PACKET_BUFF_SIZE, TX_PACKET_BUFF_SIZE, 10, &(serial_data->queue), 0));
    ESP_ERROR_CHECK(uart_set_mode(serial_data->port, UART_MODE_RS485_HALF_DUPLEX));

    
    // TODO: TO HANDLE THE COMMENT BELOW
    // The specification mentions link layer response timeout this way:
    // The time structure of various link layer communication types is described in EN60870-5-1. The answer time
    // between the end of a master send telegram and the beginning of the response telegram of the slave shall be
    // between 11 bit times and (330 bit times + 50ms).
    //
    // Nowadays the usage of USB to serial adapter is very common, which could
    // result in additional delay of 100 ms in worst case.
    //
    // For 2400Bd this means (330 + 11) / 2400 + 0.15 = 292 ms (added 11 bit periods to receive first byte).
    // I.e. timeout of 0.3s seems appropriate for 2400Bd.

    serial_data->idle_time = pdMS_TO_TICKS((uint32_t) ((double) (33000.0/((double)serial_data->config.baud_rate))));
    ESP_LOGI(TAG, "Idle time %d", serial_data->idle_time);
    serial_data->max_wait = pdMS_TO_TICKS((uint32_t) (((double) (330000.0/((double)serial_data->config.baud_rate))) + 50.0));
    ESP_LOGI(TAG, "Max wait %d", serial_data->max_wait);

#ifdef MBUS_SERIAL_DEBUG
    printf("%s: t.c_cflag = %x\n", __PRETTY_FUNCTION__, term->c_cflag);
    printf("%s: t.c_oflag = %x\n", __PRETTY_FUNCTION__, term->c_oflag);
    printf("%s: t.c_iflag = %x\n", __PRETTY_FUNCTION__, term->c_iflag);
    printf("%s: t.c_lflag = %x\n", __PRETTY_FUNCTION__, term->c_lflag);
#endif

    return 0;
}

//------------------------------------------------------------------------------
// Set baud rate for serial connection
//------------------------------------------------------------------------------
uint32_t
mbus_serial_set_baudrate(mbus_handle *handle, uint32_t baudrate)
{
    mbus_serial_data *serial_data;

    if (handle == NULL)
        return -1;

    serial_data = (mbus_serial_data *) handle->auxdata;

    if (serial_data == NULL)
        return -1;

    if (uart_set_baudrate(serial_data->port, baudrate) != ESP_OK)
    {
        return -1;
    }

    serial_data->config.baud_rate = baudrate;
    return 0;
}


//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
uint32_t
mbus_serial_disconnect(mbus_handle *handle)
{
    mbus_serial_data *serial_data;

    if (handle == NULL)
        return -1;

    serial_data = (mbus_serial_data *) handle->auxdata;

    if (serial_data == NULL)
        return -1;

    if (!uart_is_driver_installed(serial_data->port))
    {
       return -1;
    }

    uart_driver_delete(serial_data->port);

    return 0;
}

void
mbus_serial_data_free(mbus_handle *handle)
{
    mbus_serial_data *serial_data;

    if (handle)
    {
        serial_data = (mbus_serial_data *) handle->auxdata;

        if (serial_data == NULL)
        {
            return;
        }
        free(serial_data);
        handle->auxdata = NULL;
    }
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
uint32_t
mbus_serial_send_frame(mbus_handle *handle, mbus_frame *frame)
{
    uint8_t buff[PACKET_BUFF_SIZE];
    uint32_t len, ret;
    mbus_serial_data *serial_data;
    ESP_LOGI(TAG, "Send frame");
    if (handle == NULL || frame == NULL)
        return -1;

    serial_data = (mbus_serial_data *) handle->auxdata;

    if (serial_data == NULL)
        return -1;

    // Make sure serial connection is open
    if (!uart_is_driver_installed(serial_data->port))
    {
       return -1;
    }

    ESP_LOGI(TAG, "Packing it...");
    if ((len = mbus_frame_pack(frame, buff, sizeof(buff))) == -1)
    {
        ESP_LOGE(TAG, "%s: mbus_frame_pack failed\n", __PRETTY_FUNCTION__);
        return -1;
    }

#ifdef MBUS_SERIAL_DEBUG
    // if debug, dump in HEX form to stdout what we write to the serial port
    printf("%s: Dumping M-Bus frame [%d bytes]: ", __PRETTY_FUNCTION__, len);
    uint32_t i;
    for (i = 0; i < len; i++)
    {
       printf("%.2X ", buff[i]);
    }
    printf("\n");
#endif
    ESP_LOGI(TAG, "Sending it...");
    if ((ret = uart_write_bytes(serial_data->port, buff, len)) == len)
    {
        ESP_LOGI(TAG, "Sent %d", ret);
        //
        // call the send event function, if the callback function is registered
        //
        if (handle->send_event)
                handle->send_event(MBUS_HANDLE_TYPE_SERIAL, (const char*) buff, len);
    }
    else
    {
        ESP_LOGE(TAG, "%s: Failed to write frame to socket (ret = %d: %s)\n", __PRETTY_FUNCTION__, ret, strerror(errno));
        return -1;
    }

    return 0;
}

//------------------------------------------------------------------------------
//
//------------------------------------------------------------------------------
uint32_t
mbus_serial_recv_frame(mbus_handle *handle, mbus_frame *frame)
{
    char buff[PACKET_BUFF_SIZE];
    int32_t remaining, timeouts;
    ssize_t len, nread;
    mbus_serial_data *serial_data;

    if (handle == NULL || frame == NULL)
    {
        fprintf(stderr, "%s: Invalid parameter.\n", __PRETTY_FUNCTION__);
        return MBUS_RECV_RESULT_ERROR;
    }
    serial_data = (mbus_serial_data *) handle->auxdata;

    if (serial_data == NULL)
        return -1;

    // Make sure serial connection is open
    if (!uart_is_driver_installed(serial_data->port))
    {
        fprintf(stderr, "%s: Serial connection is not available.\n", __PRETTY_FUNCTION__);
        return -1;
    }

    memset((void *)buff, 0, sizeof(buff));

    //
    // read data until a packet is received
    //
    remaining = 1; // start by reading 1 byte
    len = 0;
    timeouts = 0;
    do {
        ESP_LOGI(TAG, "Reading remaining = %d", remaining);

        if (len + remaining > PACKET_BUFF_SIZE)
        {
            // avoid out of bounds access
            ESP_LOGE(TAG, "avoid out of bounds access");
            return MBUS_RECV_RESULT_ERROR;
        }

        if ((nread = uart_read_bytes(serial_data->port, &buff[len], remaining, pdMS_TO_TICKS(serial_data->max_wait))) == -1)
        {
            ESP_LOGE(TAG, "Error reading from uart");
       //     fprintf(stderr, "%s: aborting recv frame (remaining = %d, len = %d, nread = %d)\n",
         //          __PRETTY_FUNCTION__, remaining, len, nread);
            return MBUS_RECV_RESULT_ERROR;
        }
        len += nread;
        ESP_LOGI(TAG, "Read %d bytes", nread);
        ESP_LOG_BUFFER_HEX(TAG, buff, len);

//   printf("%s: Got %d byte [remaining %d, len %d]\n", __PRETTY_FUNCTION__, nread, remaining, len);

        if (nread == 0)
        {
            timeouts++;
            ESP_LOGW(TAG, "Timeout %d", timeouts);
            if (timeouts >= 3)
            {
                // abort to avoid endless loop
                //ESP_LOGW(stderr, "%s: Timeout\n", __PRETTY_FUNCTION__);
                // Insert idle time
                vTaskDelay(pdMS_TO_TICKS(serial_data->idle_time));
            }
        }
        else
        {
            if (len > PACKET_BUFF_SIZE)
            {
                len -= nread;
                // avoid overflow
                ESP_LOGE(TAG, "avoid overflow");
                return MBUS_RECV_RESULT_ERROR;
            }
            remaining = mbus_parse(frame, (unsigned char *) buff, len);
        }
    } while (remaining > 0 && timeouts < 3);

    ESP_LOGI(TAG, "MBUS PARSE EXIT = %d", remaining);

    if (len == 0)
    {
        // No data received
        return MBUS_RECV_RESULT_TIMEOUT;
    }

    //
    // call the receive event function, if the callback function is registered
    //
    if (handle->recv_event)
        handle->recv_event(MBUS_HANDLE_TYPE_SERIAL, buff, len);

    if (remaining != 0)
    {
        // Would be OK when e.g. scanning the bus, otherwise it is a failure.
        // printf("%s: M-Bus layer failed to receive complete data.\n", __PRETTY_FUNCTION__);
        ESP_LOGE(TAG, "Errror:\n%s", mbus_error_str());
        return MBUS_RECV_RESULT_INVALID;
    }

    if (len == -1)
    {
        fprintf(stderr, "%s: M-Bus layer failed to parse data.\n", __PRETTY_FUNCTION__);
        return MBUS_RECV_RESULT_ERROR;
    }

    return MBUS_RECV_RESULT_OK;
}
