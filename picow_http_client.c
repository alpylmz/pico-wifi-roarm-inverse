/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
*/

#include <stdio.h>
#include "pico/stdio.h"
#include "pico/cyw43_arch.h"
#include "pico/async_context.h"
#include "lwip/altcp_tls.h"
#include "example_http_client_util.h"

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"

#include "hardware/sync.h"

#include "inverse_kinematics/inverse_kinematics.hpp"


#define HOST "192.168.4.1"
//#define URL_REQUEST "http://192.168.4.1/js?json={\"T\":102}"
#define URL_PATH "/js?json={\"T\":102}" 

#define RESPONSE_BUFFER_SIZE 2048
#define URL_BUFFER_SIZE 512
#define URL_BASE_PATH "/js?json="

// Global or appropriately scoped response buffer
char response_buffer[RESPONSE_BUFFER_SIZE];
int response_buffer_len = 0;

char command[] = "{\"T\":102,\"base\":-0.106,\"shoulder\":-1.23,\"elbow\":-0.614,\"wrist\":-1.14,\"roll\":0.065,\"hand\":0.0004,\"spd\":0,\"acc\":10}";

// Define the structure to hold the parsed data
typedef struct {
    int T;
    float x;
    float y;
    float z;
    float tit;
    float b;
    float s;
    float e;
    float t; // Note: 't' conflicts with the type name 'time_t' sometimes, rename if issues arise
    float r;
    float g;
    int tB;
    int tS;
    int tE;
    int tT;
    int tR;
} ResponseData;

// Global or appropriately scoped variable to store the parsed data
ResponseData parsed_data;

int parse_json_response_sscanf(const char *buffer, ResponseData *data) {
    // Ensure data pointer is valid
    if (!data || !buffer) {
        return -1;
    }

    // IMPORTANT: This format string MUST exactly match the JSON structure,
    // including braces, quotes, colons, and commas.
    // %d for integers, %f for floats.
    // It assumes the order of keys is always the same.
    const char *format = "{\"T\":%d,\"x\":%f,\"y\":%f,\"z\":%f,\"tit\":%f,\"b\":%f,\"s\":%f,\"e\":%f,\"t\":%f,\"r\":%f,\"g\":%f,\"tB\":%d,\"tS\":%d,\"tE\":%d,\"tT\":%d,\"tR\":%d}";

    int items_scanned = sscanf(buffer, format,
                               &data->T, &data->x, &data->y, &data->z,
                               &data->tit, &data->b, &data->s, &data->e,
                               &data->t, &data->r, &data->g,
                               &data->tB, &data->tS, &data->tE,
                               &data->tT, &data->tR);

    // Check if the expected number of items were successfully scanned (16 in this case)
    if (items_scanned == 16) {
        return 0; // Success
    } else {
        printf("sscanf parsing failed. Expected 16 items, got %d\n", items_scanned);
        // Optional: Print the buffer that failed to parse for debugging
        // printf("Failed buffer: %s\n", buffer);
        return -1; // Failure
    }
}

err_t my_http_client_receive_store_fn(void *arg, struct altcp_pcb *pcb, struct pbuf *p, err_t err) {
    u16_t offset = 0;
    response_buffer_len = 0;

    while(offset < p->tot_len) {
        char c = (char)pbuf_get_at(p, offset++);
        if (offset < RESPONSE_BUFFER_SIZE - 1) { // Leave space for null terminator
            response_buffer[offset - 1] = c;
            response_buffer_len++;
        }
    }
    response_buffer[offset - 1] = '\0'; // Null terminate the string

    return ERR_OK;
}

// --- Send Command and Parse Response Function ---
/**
 * @brief Sends an HTTP GET request with the command embedded in the URL,
 * receives the response, parses it, and prints the parsed data.
 *
 * @param json_command The JSON command string to send (will be appended to URL_BASE_PATH).
 * @return 0 on success (request sent and response parsed), non-zero on failure.
 */
int sendCommandAndParseResponse(const char* json_command) {
    if (!json_command) {
        printf("Error: json_command cannot be NULL.\n");
        return -1;
    }

    // Construct the full URL path
    char url_buffer[URL_BUFFER_SIZE];
    int required_len = snprintf(url_buffer, URL_BUFFER_SIZE, "%s%s", URL_BASE_PATH, json_command);

    if (required_len < 0 || required_len >= URL_BUFFER_SIZE) {
        printf("Error: Failed to construct URL or URL buffer too small (need %d, have %d).\n",
               required_len, URL_BUFFER_SIZE);
        return -2;
    }

    printf("Sending command: %s\n", json_command);
    printf("Constructed URL Path: %s\n", url_buffer);

    // Reset response buffer before request
    response_buffer[0] = '\0';
    response_buffer_len = 0;

    // Setup HTTP request structure
    EXAMPLE_HTTP_REQUEST_T req = {0};
    req.hostname = HOST;
    req.url = url_buffer; // Use the dynamically constructed URL path
    req.headers_fn = http_client_header_print_fn; // Optional: Prints headers
    req.recv_fn = my_http_client_receive_store_fn; // Use our storing function

    // Get async context (assuming cyw43 is initialized)
    async_context_t *context = cyw43_arch_async_context();
    if (!context) {
        printf("Error: Failed to get async context.\n");
        return -3;
    }

    // Perform the synchronous request
    printf("Performing HTTP GET request to %s%s\n", req.hostname, req.url);
    int result = http_client_request_sync(context, &req);

    printf("HTTP request finished. Result code: %d\n", result);

    if (result != 0) {
        printf("HTTP request failed.\n");
        return result; // Return the HTTP error code
    }

    // Check if we actually received anything
    if (response_buffer_len == 0) {
         printf("Warning: HTTP request successful, but received no data.\n");
         // Depending on requirements, this might be an error or expected
         return -4; // Return a custom error code for empty response
    }

    printf("Raw Response Received (%u bytes):\n---\n%s\n---\n", (unsigned int)response_buffer_len, response_buffer);

    // Try parsing the response (using the global parsed_data struct)
    if (parse_json_response_sscanf(response_buffer, &parsed_data) == 0) {
        printf("Parsing successful!\n");
        // Print the parsed data:
        printf("  T   = %d\n", parsed_data.T);
        printf("  x   = %f\n", parsed_data.x);
        printf("  y   = %f\n", parsed_data.y);
        printf("  z   = %f\n", parsed_data.z);
        printf("  tit = %f\n", parsed_data.tit);
        printf("  b   = %f\n", parsed_data.b);
        printf("  s   = %f\n", parsed_data.s);
        printf("  e   = %f\n", parsed_data.e);
        printf("  t   = %f\n", parsed_data.t);
        printf("  r   = %f\n", parsed_data.r);
        printf("  g   = %f\n", parsed_data.g);
        printf("  tB  = %d\n", parsed_data.tB);
        printf("  tS  = %d\n", parsed_data.tS);
        printf("  tE  = %d\n", parsed_data.tE);
        printf("  tT  = %d\n", parsed_data.tT);
        printf("  tR  = %d\n", parsed_data.tR);
        return 0; // Success!
    } else {
        printf("Failed to parse the JSON response.\n");
        return -5; // Return a custom error code for parsing failure
    }
}


// --- Main Function ---
int main() {
    stdio_init_all();

    sleep_ms(10000);
    // first, test inverse kinematics:

    double goal_position[3] = {-0.253370, 0.307280, 0.384357};
    double goal_rotation[3][3] = {
        {-0.807539, -0.043014, -0.588243},
        {-0.587219, 0.152085, 0.795012},
        {0.055267, 0.987431, -0.148073}
    };

    // q_start could be a random position or the current position of the robot.
    double q_start[NU] = {1.506867, -1.0104, -0.214423, -1.1439, 0.0055117};
    //double q_start[NU] = {2.07809, -0.645178, 1.72935, 1.82307, 2.51461};
    double q_out[NU] = {0};

    inverse_kinematics_roarm(
        goal_position,
        goal_rotation,
        q_start,
        q_out
    );

    printf("Output joint angles: ");
    for (int i = 0; i < 5; i++) {
        printf("%f ", q_out[i]);
    }
    printf("\n");

    double goal_position_sec[3] = {-0.063227, 0.093742, 0.384357};
    double goal_rotation_sec[3][3] = {
        {-0.832132, -0.552419, 0.048882},
        {-0.041797, 0.150363, 0.987747},
        {-0.553000, 0.819893, -0.148211}
    };
    
    double q_start_sec[NU] = {1.506867, -1.0104, -0.214423, -1.1439, 0.0055117};
    double q_out_sec[NU] = {0};

    inverse_kinematics_roarm(
        goal_position_sec,
        goal_rotation_sec,
        q_start_sec,
        q_out_sec
    );

    printf("Output joint angles: ");
    for (int i = 0; i < 5; i++) {
        printf("%f ", q_out_sec[i]);
    }
    printf("\n");



    printf("Pico HTTP Client - Command Sender\n");

    if (cyw43_arch_init()) {
        printf("Failed to initialise cyw43\n");
        return 1;
    }
    cyw43_arch_enable_sta_mode();

    printf("Connecting to Wi-Fi '%s'...\n", WIFI_SSID);
    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("Failed to connect.\n");
        cyw43_arch_deinit();
        return 1;
    }
    printf("Connected.\n");

    // --- Example Usage ---
    // Define your command string
    const char *command1 = "{\"T\":102}"; // Simple command from original code
    const char *command2 = "{\"T\":102,\"base\":-0.106,\"shoulder\":-1.23,\"elbow\":-0.614,\"wrist\":-1.14,\"roll\":0.065,\"hand\":0.0004,\"spd\":0,\"acc\":10}"; // More complex one

    printf("\n--- Sending Command 1 ---\n");
    int status1 = sendCommandAndParseResponse(command1);
    if (status1 == 0) {
        printf("Command 1 processed successfully.\n");
        // You can now use the values in the global 'parsed_data' struct if needed
        // float current_x = parsed_data.x;
    } else {
        printf("Command 1 failed with status code: %d\n", status1);
    }

    sleep_ms(3000); // Small delay between requests (optional)

    printf("\n--- Sending Command 2 ---\n");
    // Note: The response format is assumed to be the same regardless of the command sent.
    // If the response structure changes based on the command, the parser needs adjustment.
    int status2 = sendCommandAndParseResponse(command2);
     if (status2 == 0) {
        printf("Command 2 processed successfully.\n");
        // parsed_data struct is now updated with the results from this command
    } else {
        printf("Command 2 failed with status code: %d\n", status2);
    }
    // --- End Example Usage ---


    cyw43_arch_deinit();
    printf("\nTest finished.\n");
    return (status1 == 0 && status2 == 0) ? 0 : 1; // Indicate overall success/failure
}






/*
 int main() {
     stdio_init_all();
     printf("Hello world!\n");
     if (cyw43_arch_init()) {
         printf("failed to initialise\n");
         return 1;
     }
     cyw43_arch_enable_sta_mode();
     if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 10000)) {
         printf("failed to connect\n");
         return 1;
     }

 
     EXAMPLE_HTTP_REQUEST_T req1 = {0};
     req1.hostname = HOST;
     req1.url = URL_PATH;
     printf("URL REQUEST\n");
     printf(URL_PATH);
     printf("\n");
     req1.headers_fn = http_client_header_print_fn;
     //req1.recv_fn = http_client_receive_print_fn;
     req1.recv_fn = my_http_client_receive_store_fn;
     int result = http_client_request_sync(cyw43_arch_async_context(), &req1);

     printf("result: ");
     printf("%s\n", response_buffer);

     if (parse_json_response_sscanf(response_buffer, &parsed_data) == 0) {
        printf("Parsing successful!\n");
        // Access the data:
        printf("T   = %d\n", parsed_data.T);
        printf("x   = %f\n", parsed_data.x);
        printf("y   = %f\n", parsed_data.y);
        printf("z   = %f\n", parsed_data.z);
        printf("tit = %f\n", parsed_data.tit);
        printf("b   = %f\n", parsed_data.b);
        printf("s   = %f\n", parsed_data.s);
        printf("e   = %f\n", parsed_data.e);
        printf("t   = %f\n", parsed_data.t);
        printf("r   = %f\n", parsed_data.r);
        printf("g   = %f\n", parsed_data.g);
        printf("tB  = %d\n", parsed_data.tB);
        printf("tS  = %d\n", parsed_data.tS);
        printf("tE  = %d\n", parsed_data.tE);
        printf("tT  = %d\n", parsed_data.tT);
        printf("tR  = %d\n", parsed_data.tR);

        // Now you can use parsed_data.x, parsed_data.y etc. elsewhere
    } else {
        printf("Failed to parse the JSON response.\n");
    }
     
     if (result != 0) {
         panic("test failed");
     }
     cyw43_arch_deinit();
     printf("Test passed\n");
     sleep_ms(100);
     return 0;
 }
*/
