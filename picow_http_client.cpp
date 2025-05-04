#include <stdio.h>
#include <string.h> // Required for snprintf and memcpy
#include "pico/stdio.h"
#include "pico/cyw43_arch.h"
#include "pico/async_context.h"
#include "lwip/altcp_tls.h"
#include "example_http_client_util.hpp"

#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "hardware/pll.h"

#include "hardware/sync.h"

#include "inverse_kinematics/inverse_kinematics.hpp" // Make sure NU is defined here (e.g., #define NU 5)

// --- Constants and Globals (Keep existing ones like HOST, buffers, etc.) ---
#define HOST "192.168.4.1"
#define RESPONSE_BUFFER_SIZE 2048
#define URL_BUFFER_SIZE 512
#define JSON_COMMAND_BUFFER_SIZE 256
#define URL_BASE_PATH "/js?json="
#define JOINT_TARGET_EPSILON 0.1
#define JOINT_POLL_TIMEOUT_MS 3000
#define JOINT_POLLING_INTERVAL_MS 300

double curr_goal[NU+1];
double curr_hand_pos = 2.5; // Current hand position

// Global or appropriately scoped response buffer
char response_buffer[RESPONSE_BUFFER_SIZE];
int response_buffer_len = 0;

// Define the structure to hold the parsed data from the response
typedef struct {
    int T; float x; float y; float z; float tit; float b; float s; float e;
    float t; float r; float g; int tB; int tS; int tE; int tT; int tR;
} ResponseData;

// Global or appropriately scoped variable to store the parsed data
ResponseData parsed_data;

// --- Helper Functions (Keep existing: parse_json_response_sscanf, my_http_client_receive_store_fn, sendCommandAndParseResponse) ---

// ...(paste your existing parse_json_response_sscanf function here)...
int parse_json_response_sscanf(const char *buffer, ResponseData *data) {
    // Ensure data pointer is valid
    if (!data || !buffer) {
        return -1;
    }
    const char *format = "{\"T\":%d,\"x\":%f,\"y\":%f,\"z\":%f,\"tit\":%f,\"b\":%f,\"s\":%f,\"e\":%f,\"t\":%f,\"r\":%f,\"g\":%f,\"tB\":%d,\"tS\":%d,\"tE\":%d,\"tT\":%d,\"tR\":%d}";
    int items_scanned = sscanf(buffer, format,
                               &data->T, &data->x, &data->y, &data->z,
                               &data->tit, &data->b, &data->s, &data->e,
                               &data->t, &data->r, &data->g,
                               &data->tB, &data->tS, &data->tE,
                               &data->tT, &data->tR);
    if (items_scanned == 16) {
        return 0; // Success
    } else {
        printf("sscanf parsing failed. Expected 16 items, got %d\n", items_scanned);
        return -1; // Failure
    }
}

// ...(paste your existing my_http_client_receive_store_fn function here)...
err_t my_http_client_receive_store_fn(void *arg, struct altcp_pcb *pcb, struct pbuf *p, err_t err) {
    if (p == NULL) {
        // printf("Connection closed or error occurred (pbuf is NULL, err = %d).\n", err);
        return ERR_OK;
    }
    if (response_buffer_len + p->tot_len >= RESPONSE_BUFFER_SIZE) {
        printf("Error: Response buffer overflow prevented. Current len: %d, incoming: %u\n",
               response_buffer_len, p->tot_len);
        u16_t space_left = RESPONSE_BUFFER_SIZE - response_buffer_len - 1;
        if (space_left > 0) {
            pbuf_copy_partial(p, response_buffer + response_buffer_len, space_left, 0);
            response_buffer_len += space_left;
            response_buffer[response_buffer_len] = '\0';
        }
        altcp_recved(pcb, p->tot_len);
        pbuf_free(p);
        return ERR_BUF;
    }
    u16_t copied_len = pbuf_copy_partial(p, response_buffer + response_buffer_len, p->tot_len, 0);
    response_buffer_len += copied_len;
    response_buffer[response_buffer_len] = '\0';
    altcp_recved(pcb, p->tot_len);
    pbuf_free(p);
    return ERR_OK;
}


// ...(paste your existing sendCommandAndParseResponse function here)...
int sendCommandAndParseResponse(const char* json_command) {
    if (!json_command) {
        printf("Error: json_command cannot be NULL.\n");
        return -1;
    }
    char url_buffer[URL_BUFFER_SIZE];
    int required_len = snprintf(url_buffer, URL_BUFFER_SIZE, "%s%s", URL_BASE_PATH, json_command);
    if (required_len < 0 || required_len >= URL_BUFFER_SIZE) {
        printf("Error: Failed to construct URL or URL buffer too small (need %d, have %d).\n",
               required_len, URL_BUFFER_SIZE);
        return -2;
    }
    //printf("Sending command JSON: %s\n", json_command);
    response_buffer[0] = '\0';
    response_buffer_len = 0;
    EXAMPLE_HTTP_REQUEST_T req = {0};
    req.hostname = HOST;
    req.url = url_buffer;
    req.recv_fn = my_http_client_receive_store_fn;
    async_context_t *context = cyw43_arch_async_context();
    if (!context) {
        printf("Error: Failed to get async context.\n");
        return -3;
    }
    //printf("Performing HTTP GET request to %s%s\n", req.hostname, req.url);
    int result = http_client_request_sync(context, &req);
    //printf("HTTP request finished. Result code: %d\n", result);
    if (result != 0) {
        printf("HTTP request failed with lwip error code: %d.\n", result);
        return result;
    }
    if (response_buffer_len == 0) {
         printf("Warning: HTTP request successful, but received no data.\n");
         return -4;
    }
    printf("Raw Response Received (%d bytes):\n---\n%s\n---\n", response_buffer_len, response_buffer);
    if (parse_json_response_sscanf(response_buffer, &parsed_data) == 0) {
        //printf("Parsing successful!\n");
        // Print parsed data (optional, can be verbose in a loop)
        // printf("  T=%d, x=%.3f, y=%.3f, z=%.3f, b=%.3f, s=%.3f, e=%.3f, t=%.3f, r=%.3f, g=%.3f\n",
        //        parsed_data.T, parsed_data.x, parsed_data.y, parsed_data.z,
        //        parsed_data.b, parsed_data.s, parsed_data.e, parsed_data.t,
        //        parsed_data.r, parsed_data.g);
        return 0; // Success!
    } else {
        printf("Failed to parse the JSON response.\n");
        return -5; // Parsing failure
    }
}

// --- Structure to hold a target pose ---
typedef struct {
    double position[3];
    double rotation[3][3];
} TargetPose;

#define OPEN_HAND TargetPose{ {900.0, 900.0, 900.0}, {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0}} }
#define CLOSE_HAND TargetPose{ {1000.0, 1000.0, 1000.0}, {{1.0, 0.0, 0.0}, {0.0, 1.0, 0.0}, {0.0, 0.0, -1.0}} }

bool is_open_hand(const TargetPose& pose) {
    return pose.position[0] == OPEN_HAND.position[0] &&
           pose.position[1] == OPEN_HAND.position[1] &&
           pose.position[2] == OPEN_HAND.position[2];
}

bool is_close_hand(const TargetPose& pose) {
    return pose.position[0] == CLOSE_HAND.position[0] &&
           pose.position[1] == CLOSE_HAND.position[1] &&
           pose.position[2] == CLOSE_HAND.position[2];
}

TargetPose create_pose_from_xyz_yaw(double x, double y, double z, double yaw_rad) {
    TargetPose pose;

    // 1. Set the position vector directly
    pose.position[0] = x;
    pose.position[1] = y;
    pose.position[2] = z;

    // 2. Calculate the rotation matrix for Z-axis rotation (Yaw)
    double cos_yaw = cos(yaw_rad);
    double sin_yaw = sin(yaw_rad);

    // Rotation matrix for rotation around Z by yaw_rad:
    // [ cos(yaw) -sin(yaw)  0 ]
    // [ sin(yaw)  cos(yaw)  0 ]
    // [    0         0      1 ]
    pose.rotation[0][0] = cos_yaw;
    pose.rotation[0][1] = -sin_yaw;
    pose.rotation[0][2] = 0.0;

    pose.rotation[1][0] = sin_yaw;
    pose.rotation[1][1] = cos_yaw;
    pose.rotation[1][2] = 0.0;

    pose.rotation[2][0] = 0.0;
    pose.rotation[2][1] = 0.0;
    pose.rotation[2][2] = -1.0;

    return pose;
}

// --- Define the sequence of target poses ---
// Add as many poses as you need here
TargetPose target_poses[] = {
    // Pose 1 (Example from your original code)
    //{
    //    {0.099508, 0.258437, 0.253702},
    //    {
    //        {0.965013, -0.262201, -0.000446}, 
    //        {0.261309, -0.961865, -0.080833}, 
    //        {0.021623, 0.077889,-0.996728} 
    //    }
    //},
    //{
    //    {0.099508, 0.258437, 0.203702},
    //    {
    //        {0.965013, -0.262201, -0.000446}, 
    //        {0.261309, -0.961865, -0.080833}, 
    //        {0.021623, 0.077889,-0.996728} 
    //    }
    //},
    OPEN_HAND,
    create_pose_from_xyz_yaw(0.099508, 0.208437, 0.253702, 0.0),
    create_pose_from_xyz_yaw(0.159508, 0.208437, 0.153702, 0.0),
    CLOSE_HAND,
    //// -y axis is towards the wall
    //{
    //    {0.099508, 0.208437, 0.253702},
    //    {
    //        {0.965013, -0.262201, -0.000446}, 
    //        {0.261309, -0.961865, -0.080833}, 
    //        {0.021623, 0.077889,-0.996728} 
    //    }
    //},
    //// +x is towards the window + slightly opposite side of the wall
    //{
    //    {0.159508, 0.208437, 0.253702},
    //    {
    //        {0.965013, -0.262201, -0.000446}, 
    //        {0.261309, -0.961865, -0.080833}, 
    //        {0.021623, 0.077889,-0.996728} 
    //    }
    //},
    //// -z axis is towards the floor + slightly to the wall
    //{
    //    {0.159508, 0.208437, 0.153702},
    //    {
    //        {0.965013, -0.262201, -0.000446}, 
    //        {0.261309, -0.961865, -0.080833}, 
    //        {0.021623, 0.077889,-0.996728} 
    //    }
    //},
    //{
    //    {0.309508, 0.208437, 0.153702},
    //    {
    //        {0.965013, -0.262201, -0.000446}, 
    //        {0.261309, -0.961865, -0.080833}, 
    //        {0.021623, 0.077889,-0.996728} 
    //    }
    //}
    // Add more poses here...
};

/**
 * @brief Sends a command with target joint angles and waits for the robot to report being close.
 *
 * Sends the command repeatedly and polls the robot's status via HTTP GET response.
 * Compares the reported joint angles (b, s, e, t, r from parsed_data)
 * to the target joint angles provided in target_q.
 * Continues polling until all reported angles are within JOINT_TARGET_EPSILON
 * of their respective targets, or until a timeout occurs.
 *
 * !!! --- ASSUMPTIONS --- !!!
 * - Relies on global 'parsed_data' being updated by sendCommandAndParseResponse.
 * - Relies on sendCommandAndParseResponse returning 0 on success (incl. parsing).
 * - Assumes NU = 5 and the order in target_q matches base, shoulder, elbow, wrist, roll.
 * - Assumes parsed_data fields b, s, e, t, r correspond to these joints IN THAT ORDER.
 * - Assumes the 'hand' value in the command can be a fixed default.
 *
 * @param target_q Array of NU target joint angles (radians).
 * @return 0 on success (target reached within epsilon).
 * -1 on communication or parsing error during polling.
 * -2 on timeout.
 * -3 on command formatting error.
 */
int waitForJointTargetReached(const double target_q[NU]) {
    // Local buffer for the command to avoid interfering with other potential uses
    // of a global buffer outside this function's polling loop.
    char command_buffer[JSON_COMMAND_BUFFER_SIZE];
    int command_len;
    int status;

    // 1. Format the target joint angles into the JSON command string
    //    Uses the provided target_q for joint values.
    //    Uses fixed values for Hand, Speed, Acceleration based on previous examples.
    command_len = snprintf(command_buffer, JSON_COMMAND_BUFFER_SIZE,
                           "{\"T\":102,\"base\":%.6f,\"shoulder\":%.6f,\"elbow\":%.6f,\"wrist\":%.6f,\"roll\":%.6f,\"hand\":%.4f,\"spd\":%d,\"acc\":%d}",
                           target_q[0], target_q[1], target_q[2], target_q[3], target_q[4],
                           curr_hand_pos, // Default hand value from previous code - adjust if needed
                           0,    // Default speed
                           2);  // Default acceleration

    if (command_len < 0 || command_len >= JSON_COMMAND_BUFFER_SIZE) {
        printf("ERROR [waitForJointTargetReached]: Failed to format joint command.\n");
        return -3; // Formatting error
    }
    
    // 2. Start Polling Loop with Timeout
    absolute_time_t timeout_time = make_timeout_time_ms(JOINT_POLL_TIMEOUT_MS);
    int poll_count = 0;
    bool first_poll = true; // Flag to ensure we check status at least once

    while (first_poll || !time_reached(timeout_time)) {
        first_poll = false; // Ensure loop terminates if timeout is 0 and condition met first time
        poll_count++;

        // 3. Send the command to request status / continue motion toward target_q
        // printf("DEBUG [waitForJointTargetReached]: Polling attempt %d\n", poll_count);
        status = sendCommandAndParseResponse(command_buffer);

        if (status != 0) {
            // If the command send or response parsing fails during polling
            printf("ERROR [waitForJointTargetReached]: Communication/Parsing failed (status %d) during poll %d. Aborting wait.\n", status, poll_count);
            return -1; // Communication or parsing error
        }

        // 4. Check if all joints reported in parsed_data are within epsilon
        bool all_reached = true;
        double reported_q[NU]; // Array to hold reported values for checking

        // --- Map parsed_data fields to reported_q array ---
        // !!! CRITICAL: Ensure this order matches your robot's response and target_q order !!!
        reported_q[0] = (double)parsed_data.b; // Base
        reported_q[1] = (double)parsed_data.s; // Shoulder
        reported_q[2] = (double)parsed_data.e; // Elbow
        reported_q[3] = (double)parsed_data.t; // Wrist ('t')
        reported_q[4] = (double)parsed_data.r; // Roll ('r')
        // --- End Mapping ---

        // Optional: Print current reported angles for debugging
        // printf("DEBUG [waitForJointTargetReached]: Poll %d Reported=[%.3f, %.3f, %.3f, %.3f, %.3f]\n", poll_count,
        //        reported_q[0], reported_q[1], reported_q[2], reported_q[3], reported_q[4]);

        for (int j = 0; j < NU; ++j) {
            double diff = fabs(reported_q[j] - target_q[j]);
            if (diff > JOINT_TARGET_EPSILON) {
                printf("DEBUG [waitForJointTargetReached]: Joint %d delta %.4f > Eps %.4f\n", j, diff, JOINT_TARGET_EPSILON); // Verbose
                all_reached = false;
                break; // No need to check further joints for this poll cycle
            }
        }

        // 5. Exit loop if target reached
        if (all_reached) {
            printf("INFO [waitForJointTargetReached]: Joint target reached within epsilon after %d polls.\n", poll_count);
            return 0; // Success
        }

        // 6. Check for timeout *after* checking condition, before sleeping
        if (time_reached(timeout_time)) {
             printf("ERROR [waitForJointTargetReached]: Timeout after %d polls.\n", poll_count);
             return -2; // Timeout error
        }

        // 7. Wait before next poll cycle
        sleep_ms(JOINT_POLLING_INTERVAL_MS);

    } // End while loop

    // This part should only be reached if the timeout happened exactly on the last check
    printf("ERROR [waitForJointTargetReached]: Timeout likely occurred exactly at loop end.\n");
    return -2; // Timeout error
}

// --- Main Function ---
int main() {
    stdio_init_all();
    printf("Pico HTTP Client + Inverse Kinematics Sequence\n");
    sleep_ms(5000); // Wait for serial connection

    // Calculate the number of poses defined
    int num_poses = sizeof(target_poses) / sizeof(target_poses[0]);
    printf("Defined %d target poses.\n", num_poses);

    // --- IK and Command Variables ---
    // Initial guess for the very first IK calculation (e.g., home position)
    // Ensure NU is defined correctly (likely 5 for base, shoulder, elbow, wrist, roll)
    double initial_q_start[NU] = {1.506867, -1.0104, -0.214423, -1.1439, 0.0055117};
    double guess_q[NU] = {1.506867, -1.0104, -0.214423, -1.1439, 0.0055117};
    // double initial_q_start[NU] = {0.0, -M_PI_2, M_PI_2, -M_PI_2, 0.0}; // Example alternative start

    double current_q[NU] = {0}; // Holds the current/last known joint angles
    double q_out[NU] = {0};     // Holds the result of the latest IK calculation
    char ik_command_buffer[JSON_COMMAND_BUFFER_SIZE]; // Buffer for the command string
    int command_len;
    int status = 0;
    bool overall_success = true;

    // Initialize current_q with the starting configuration
    memcpy(current_q, initial_q_start, sizeof(initial_q_start));

    // --- WiFi Connection ---
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
    printf("Connected to WiFi.\n");

    // --- Loop through the poses ---
    for (int i = 0; i < num_poses; ++i) {
        printf("\n--- Processing Pose %d of %d ---\n", i + 1, num_poses);

        if(is_open_hand(target_poses[i])) {
            printf("Pose %d is OPEN_HAND.\n", i + 1);
            // send {"T": 106, "cmd": 2.6, "spd": 2, "acc": 2}
            command_len = snprintf(ik_command_buffer, JSON_COMMAND_BUFFER_SIZE,
                                "{\"T\":106,\"cmd\":%.4f,\"spd\":%d,\"acc\":%d}",
                                2.9,  // Default hand value
                                2,    // Default speed
                                2);  // Default acceleration
        } else if(is_close_hand(target_poses[i])) {
            //// send {"T": 107, "tor": 100}
            //command_len = snprintf(ik_command_buffer, JSON_COMMAND_BUFFER_SIZE,
            //                    "{\"T\":107,\"tor\":%d}",
            //                    100); // Default torque value
            //command_len = snprintf(ik_command_buffer, JSON_COMMAND_BUFFER_SIZE,
            //                    "{\"T\":106,\"cmd\":%.4f,\"spd\":%d,\"acc\":%d}",
            //                    0.98,  // Default hand value
            //                    2,    // Default speed
            //                    2);  // Default acceleration
            // above commands is not working for some reason
            // let's try {"T": 121, "joint": 5, "angle": 0.2, "spd": 2, "acc": 2}
            command_len = snprintf(ik_command_buffer, JSON_COMMAND_BUFFER_SIZE,
                                "{\"T\":121,\"joint\":%d,\"angle\":%.4f,\"spd\":%d,\"acc\":%d}",
                                6,    // Joint number for hand
                                0.9,  // Hand position
                                2,    // Default speed
                                2);  // Default acceleration
            printf("Pose %d is CLOSE_HAND.\n", i + 1);
        } else {       
            // Calculate Inverse Kinematics for the current target pose
            // Use 'current_q' (result from previous step or initial start) as the starting guess
            inverse_kinematics_roarm(
                target_poses[i].position,
                target_poses[i].rotation,
                guess_q,
                q_out      // Store the result here
            );

            printf("IK Output (q_out): ");
            for (int j = 0; j < NU; j++) {
                printf("%.6f ", q_out[j]);
            }
            printf("\n");

            // Format the IK result into a JSON command string
            // Using default values: hand = 0.0, spd = 0, acc = 10, T = 102
            command_len = snprintf(ik_command_buffer, JSON_COMMAND_BUFFER_SIZE,
                                "{\"T\":102,\"base\":%.6f,\"shoulder\":%.6f,\"elbow\":%.6f,\"wrist\":%.6f,\"roll\":%.6f,\"spd\":%d,\"acc\":%d}",
                                q_out[0], q_out[1], q_out[2], q_out[3], q_out[4],
                                //curr_hand_pos, // Default hand/gripper value
                                0,      // Default speed
                                2);    // Default acceleration

            if (command_len < 0 || command_len >= JSON_COMMAND_BUFFER_SIZE) {
                printf("Error: Failed to format IK command for pose %d or buffer too small.\n", i + 1);
                overall_success = false;
                // Decide whether to continue or break
                // continue; // Skip sending this command
                break;    // Stop the sequence on formatting error
            }
        }

        // Send the command
        status = sendCommandAndParseResponse(ik_command_buffer);
        if (status != 0) {
            printf("Command for pose %d failed with status code: %d\n", i + 1, status);
            overall_success = false;
            // Decide whether to continue or break
            // continue; // Try next pose even if this one failed
             break;    // Stop the sequence on communication error
        } else {
             printf("Command for pose %d processed successfully.\n", i + 1);
             // Update current_q with the newly calculated angles for the *next* iteration's guess
             memcpy(current_q, q_out, sizeof(q_out));
        }

        // Optional delay between sending commands for each pose
        //printf("Waiting 3 seconds before next pose...\n");
        //sleep_ms(3000);
        if(is_open_hand(target_poses[i])) {
            printf("Waiting for hand to open...\n");
            sleep_ms(1000);
        } else if(is_close_hand(target_poses[i])) {
            printf("Waiting for hand to close...\n");
            sleep_ms(4000);
        } else {
            printf("Waiting for joint target to be reached to ...\n");
            // print q_out
            for (int j = 0; j < NU; j++) {
                printf("%.6f ", q_out[j]);
            }
            printf("\n");
            waitForJointTargetReached(q_out);
        }
    } // End of pose loop

    // --- Cleanup ---
    cyw43_arch_deinit();
    printf("\nSequence finished.\n");
    printf("Overall status: %s\n", overall_success ? "Success" : "Failure (One or more steps failed)");

    return overall_success ? 0 : 1;
}

// --- Ensure you have these definitions somewhere, e.g., in a secrets.h or directly ---
// const char WIFI_SSID[] = "YOUR_SSID";
// const char WIFI_PASSWORD[] = "YOUR_PASSWORD";
// --- ---