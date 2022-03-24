/******************************************************************************
 * File Name: mqtt_operation.c
 *
 * Description: This file contains threads, functions, and other resources related
 * to MQTT operations.
 *
 *******************************************************************************
 * Copyright 2019-2021, Cypress Semiconductor Corporation (an Infineon company) or
 * an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
 *
 * This software, including source code, documentation and related
 * materials ("Software") is owned by Cypress Semiconductor Corporation
 * or one of its affiliates ("Cypress") and is protected by and subject to
 * worldwide patent protection (United States and foreign),
 * United States copyright laws and international treaty provisions.
 * Therefore, you may use this Software only as provided in the license
 * agreement accompanying the software package from which you
 * obtained this Software ("EULA").
 * If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 * non-transferable license to copy, modify, and compile the Software
 * source code solely for use in connection with Cypress's
 * integrated circuit products.  Any reproduction, modification, translation,
 * compilation, or representation of this Software except as specified
 * above is prohibited without the express written permission of Cypress.
 *
 * Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 * reserves the right to make changes to the Software without notice. Cypress
 * does not assume any liability arising out of the application or use of the
 * Software or any product or circuit described in the Software. Cypress does
 * not authorize its products for use in any products where a malfunction or
 * failure of the Cypress product may reasonably be expected to result in
 * significant property damage, injury or death ("High Risk Product"). By
 * including Cypress's product in a High Risk Product, the manufacturer
 * of such system or application assumes all risk of such use and in doing
 * so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

#include "cyhal.h"
#include "cybsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "aws_demo.h"
#include "cJSON/cJSON.h"
#include "mqtt_operation.h"
#include "aws_clientcredential.h"
#include "aws_clientcredential_keys.h"
#include "iot_default_root_certificates.h"

/* Include AWS IoT metrics macros header. */
#include "aws_iot_metrics.h"

/*******************************************************************************
 * Macros
 ******************************************************************************/
/* Thread delays */
#define MQTT_THREAD_LOOP_DELAY_MS                (1000)

/* MQTT Publish Topic */
#define MQTT_TOPIC                               clientcredentialIOT_THING_NAME\
                                                    "/example/topic"
/* MQTT topics used */
#define MQTT_TOPIC_COUNT                         (1)

/* Network buffer size for MQTT transfer */
#define NETWORK_BUFFER_SIZE                      (1024)

/* Maximum packets to be published */
#define MAX_PUBLISH_MESSAGE_COUNT                (20)

/* Time to wait for receive the packet */
#define PROCESS_LOOP_TIMEOUT_MS                  (700U)
#define MQTT_PROCESS_LOOP_PACKET_WAIT_COUNT_MAX  (20U)

/*******************************************************************************
 * Global variable
 ******************************************************************************/
/* Handle to the queue to receive the gesture data */
extern QueueHandle_t gesture_data_q;

/* Handle to the various tasks */
extern TaskHandle_t mqtt_task_handle, gesture_task_handle;

/* Network Buffer */
uint8_t uc_shared_buffer[NETWORK_BUFFER_SIZE];

/* Variable to calculate task run time */
static uint32_t global_entry_time_ms;

/* Packet type received from the broker */
static uint16_t packet_type_received = 0U;

/* Packet identifiers */
static uint16_t subscribe_packet_identifier;
static uint16_t publish_packet_identifier;
static uint16_t unsubscribe_packet_identifier;

/*******************************************************************************
 * Structures
 ******************************************************************************/
/* MQTT buffer for packet transfer */
MQTTFixedBuffer_t mqtt_buffer =
{
    uc_shared_buffer,
    NETWORK_BUFFER_SIZE
};

/* Strucure for topic filters */
typedef struct topic_filter_context
{
    const char * pcTopicFilter;
    MQTTSubAckStatus_t xSubAckStatus;
} topic_filter_context_t;

/* Run-time state of topic filters */
static topic_filter_context_t topic_filter_context[MQTT_TOPIC_COUNT] =
{
    {MQTT_TOPIC, MQTTSubAckFailure}
};

/******************************************************************************
 * Function Prototypes
 *******************************************************************************/
static int connect_to_server(NetworkContext_t * p_network_context);
static int backoff_for_retry(BackoffAlgorithmContext_t * p_retry_params);
static int create_mqtt_connection_with_broker(MQTTContext_t * p_mqtt_context,
        NetworkContext_t * p_network_context);
static uint32_t get_time_ms(void);
static void mqtt_event_callback(MQTTContext_t * p_mqtt_context,
        MQTTPacketInfo_t * p_packet_info,
        MQTTDeserializedInfo_t * p_deserialized_info);
static void mqtt_process_incoming_publish(MQTTPublishInfo_t * p_publish_info);
static MQTTStatus_t wait_for_packet(MQTTContext_t * p_mqtt_context,
        uint16_t packet_type);
static int mqtt_subscribe(MQTTContext_t * p_mqtt_context);
static void mqtt_process_response(MQTTPacketInfo_t * incoming_packet,
        uint16_t packet_id);
static void update_sub_ack_status(MQTTPacketInfo_t * p_packet_info);
static int mqtt_unsubscribe_from_topic(MQTTContext_t * p_mqtt_context);

/*******************************************************************************
 * Function Name: task_mqtt
 ********************************************************************************
 * Summary:
 * This task establishes a secure TLS and MQTT connection with the broker and
 * on receipt of command from control task over the queue, it publishes the
 * detected gesture to an AWS thing shadow. The number of packets published is
 * controlled by MAX_PUBLISH_MESSAGE_COUNT after which the demo will exit.
 *
 * Parameters:
 *  void *param : Task parameter defined during task creation (unused)
 *
 *******************************************************************************/
void mqtt_task(void* param)
{
    /* Suppress warning for unused parameter */
    (void)param;

    /* MQTT data containing the detected gesture */
    mqtt_data_t mqtt_data;

    /* JSON message to send */
    char json[MAX_JSON_MESSAGE_LENGTH];

    /* Buffer for topic name */
    char topic[MAX_TOPIC_LENGTH];

    /* Length of topic */
    uint16_t topic_length = 0;

    /* Length of JSON message */
    uint16_t message_length = 0;

    /* Variables to hold operation status */
    MQTTStatus_t mqtt_status;
    BaseType_t demo_status = pdFAIL;

    /* Handle of the MQTT connection used in this demo. */
    NetworkContext_t network_context = { 0 };
    MQTTContext_t mqtt_context = { 0 };

    /* Connection indication flags */
    BaseType_t is_connection_established = pdFALSE;
    BaseType_t is_mqtt_connection_established = pdFALSE;

    /* Initialize the parameters for transport layer */
    SecureSocketsTransportParams_t secure_sockets_transport_params = { 0 };

    /* Counter for published messages */
    static int published_messages_counter = 0;

    /* Set the entry time of the application */
    global_entry_time_ms = get_time_ms();

    /* Initialize the network parameters */
    network_context.pParams = &secure_sockets_transport_params;

    /* Establish secure TLS connection with the server */
    demo_status = connect_to_server(&network_context);
    print_status(demo_status, "Server Connection Failed!", "TLS Connection Established");

    /* Check if TLS connection is successful */
    if(demo_status == pdPASS){

        /* Set a flag indicating a TLS connection exists */
        is_connection_established = pdTRUE;

        /* Create MQTT connection with the broker */
        demo_status = create_mqtt_connection_with_broker(&mqtt_context, &network_context);
        print_status(demo_status, "MQTT Connection Failed!", "MQTT Connection Established");
    }

    /* Check if MQTT connection is successful */
    if(demo_status == pdPASS)
    {
        /* Set a flag indicating a MQTT connection exists */
        is_mqtt_connection_established = pdTRUE;

        /* Subscribe to MQTT topic */
        demo_status = mqtt_subscribe(&mqtt_context);
        print_status(demo_status, "MQTT Subscribe Failed!", "MQTT Subscribe Successful");
    }

    /* Send command to get detected gesture from the device shadow */
    mqtt_data.detected_gesture = GET_GESTURE;

    /* Resume the other tasks once the MQTT connection is complete */
    vTaskResume(gesture_task_handle);

    /* Send initial command over queue to publish start message */
    xQueueOverwrite(gesture_data_q, &mqtt_data);

    while(demo_status == pdPASS)
    {
        /* Block until a command has been received over queue */
        xQueueReceive(gesture_data_q, &mqtt_data, portMAX_DELAY);

        /* Get the length of the topic */
        topic_length = snprintf(topic, sizeof(topic), MQTT_TOPIC);

        /* Check the detected gesture sent on the queue */
        switch(mqtt_data.detected_gesture)
        {
        /* No gesture detected */
        case GESTURE_NONE:
        {
            message_length = snprintf(json, sizeof(json), "{\"gesture\" : \"none\"}");
            break;
        }
        /* Circular gesture detected */
        case GESTURE_CIRCLE:
        {
            message_length = snprintf(json, sizeof(json), "{\"gesture\" : \"circle\"}");
            break;
        }
        /* Side-to-side gesture detected */
        case GESTURE_SIDE_TO_SIDE:
        {
            message_length = snprintf(json, sizeof(json), "{\"gesture\" : \"side-to-side\"}");
            break;
        }
        /* Square gesture detected */
        case GESTURE_SQUARE:
        {
            message_length = snprintf(json, sizeof(json), "{\"gesture\" : \"square\"}");
            break;
        }
        /* Send empty message and set topic to get shadow status */
        case GET_GESTURE:
        {
            message_length = snprintf(json, sizeof(json), "{\"message\" : \"starting demo\"}");
            topic_length = snprintf(topic, sizeof(topic), MQTT_TOPIC);
            break;
        }
        /* Invalid command */
        default:
        {
            /* Handle invalid command here */
            break;
        }
        }

        /* Publish the detected gesture only if connection has been established */
        if((is_connection_established == pdTRUE) && (is_mqtt_connection_established == pdTRUE)){

            /* Publish the detected gesture */
            demo_status = publish_message(&mqtt_context, topic, topic_length, json, message_length);
            print_status(demo_status, "Publish Failed!", "Publish Successful");

            /* Wait to receive the publish message from the broker */
            LogInfo(("Attempt to receive publish message from broker."));
            mqtt_status = wait_for_packet(&mqtt_context, MQTT_PACKET_TYPE_PUBLISH);
            check_mqtt_status(mqtt_status, "Wait for Packet");

            /* Increment successfully published messages counter */
            published_messages_counter++;
        }

        /* End the demo once the max publish count is reached */
        if(published_messages_counter == MAX_PUBLISH_MESSAGE_COUNT){

            /* Publish demo finished */
            message_length = snprintf(json, sizeof(json), "{\"message\" : \"ending demo\"}");
            topic_length = snprintf(topic, sizeof(topic), MQTT_TOPIC);

            publish_message(&mqtt_context, topic, strlen(topic), json, strlen(json));
            print_status(demo_status, "Publish Failed!", "Publish Successful");

            /* Wait to receive the publish message from the broker */
            LogInfo(("Attempt to receive publish message from broker."));
            mqtt_status = wait_for_packet(&mqtt_context, MQTT_PACKET_TYPE_PUBLISH);
            check_mqtt_status(mqtt_status, "Wait for Packet");

            goto exit_cleanup;
        }

        /* Minimum delay between publishes */
        vTaskDelay(pdMS_TO_TICKS(MQTT_THREAD_LOOP_DELAY_MS));
    }

    /* Disconnect all connections and delete tasks to save resources */
    exit_cleanup:

    LogInfo(("\nTerminating sensor and control tasks...\n"));

    if (gesture_task_handle != NULL)
    {
        vTaskDelete(gesture_task_handle);
    }

    /* Disconnect all connections */
    demo_status = disconnect_connection(&mqtt_context, &network_context, is_connection_established,
            is_mqtt_connection_established);
    print_status(demo_status, "Closing connections failed!", "Closed all connections successfully");

    LogInfo(("-----------------Demo Complete------------------"));

    /* Delete MQTT task */
    vTaskDelete(mqtt_task_handle);
}

/*******************************************************************************
 * Function Name: print_status
 ********************************************************************************
 * Summary:
 * This function prints the error/success message based on operation status
 *
 * Parameters:
 *  BaseType_t demo_status: Status of the operation
 *  char *error_message: Error message to be printed
 *  char *success_message: Success message to be printed
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void print_status(BaseType_t demo_status, char *error_message, char *success_message)
{
    if(demo_status != pdPASS)
    {
        LogError(("Demo failed with error: %s\r\n", error_message));
    }
    else
    {
        LogInfo(("%s\r\n", success_message));
    }
}

/*******************************************************************************
 * Function Name: disconnect_connection
 ********************************************************************************
 * Summary:
 * This function disconnects TLS and MQTT connection if they exist
 *
 * Parameters:
 *  MQTTContext_t *p_mqtt_context: MQTT operation context
 *  NetworkContext_t * p_network_context: Network operation context
 *  BaseType_t is_connection_established: TLS connection status flag
 *  BaseType_t is_mqtt_connection_established: MQTT connection status flag
 *
 * Return:
 *  int status : `pdTRUE` if operation is successful 
 *               `pdFAIL` otherwise.
 *
 *******************************************************************************/
int disconnect_connection(MQTTContext_t *p_mqtt_context,
        NetworkContext_t * p_network_context,
        BaseType_t is_connection_established,
        BaseType_t is_mqtt_connection_established)
{
    BaseType_t demo_status = pdFAIL;
    TransportSocketStatus_t network_status;
    MQTTStatus_t mqtt_status;

    /* Check if MQTT connection exists */
    if(is_mqtt_connection_established == pdTRUE)
    {
        /* Unsubscribe from the topic */
        LogInfo(("Unsubscribe from the MQTT topic %s.", MQTT_TOPIC));
        demo_status = mqtt_unsubscribe_from_topic(p_mqtt_context);
        print_status(demo_status, "Unsubscribe Failed!", "Unsubscribe Successful");

        /* Process incoming UNSUBACK packet from the broker. */
        mqtt_status = wait_for_packet(p_mqtt_context, MQTT_PACKET_TYPE_UNSUBACK);
        check_mqtt_status(mqtt_status, "Unsubscribe");

        /* Disconnect MQTT connection */
        LogInfo(("Disconnecting the MQTT connection with %s.", clientcredentialMQTT_BROKER_ENDPOINT));
        mqtt_status = MQTT_Disconnect(p_mqtt_context);
        check_mqtt_status(mqtt_status, "Disconnect");
    }

    /* Check if TLS connection exists */
    if(is_connection_established == pdTRUE)
    {
        /* Close the network connection.  */
        network_status = SecureSocketsTransport_Disconnect(p_network_context);

        /* Check status of network disconnection */
        if(network_status != TRANSPORT_SOCKET_STATUS_SUCCESS)
        {
            demo_status = pdFAIL;
            LogError(("SecureSocketsTransport_Disconnect() failed to close the network connection. "
                    "StatusCode=%d.", (int) network_status));
        }
    }

    return demo_status;
}

/*******************************************************************************
 * Function Name: check_mqtt_status
 ********************************************************************************
 * Summary:
 * This function prints the error/success message based on operation status
 *
 * Parameters:
 *  MQTTStatus_t mqtt_status: Status of the MQTT operation
 *  char* operation: Name of the operation to be checked
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void check_mqtt_status(MQTTStatus_t mqtt_status, char* operation)
{
    if(mqtt_status != MQTTSuccess)
    {
        LogError(("MQTT Status for %s failed with"
                "StatusCode=%d.", operation, (int) mqtt_status));
    }
}

/*******************************************************************************
 * Function Name: publish_message
 ********************************************************************************
 * Summary:
 * This function publishes the message to the AWS Thing
 *
 * Parameters:
 *  MQTTContext_t *p_mqtt_context: MQTT operation context
 *  char* topic: Name of the topic
 *  uint16_t topic_length: Length of the topic name
 *  char* mqtt_message: Message to be published
 *  uint16_t message_length: Length of the message to be published
 *
 * Return:
 *  int status : `pdTRUE` if operation is successful 
 *               `pdFAIL` otherwise.
 *
 *******************************************************************************/
int publish_message(MQTTContext_t *p_mqtt_context,
        char* topic,
        uint16_t topic_length,
        char* mqtt_message,
        uint16_t message_length)
{
    MQTTStatus_t mqtt_status;
    MQTTPublishInfo_t publish_info;
    BaseType_t demo_status = pdPASS;

    /* Some fields are not used by this demo so start with everything at 0. */
    (void) memset((void *) &publish_info, 0x00, sizeof(publish_info));

    /* This demo uses QoS1. */
    publish_info.qos = MQTTQoS1;
    publish_info.retain = false;
    publish_info.pTopicName = topic;
    publish_info.topicNameLength = (uint16_t) strlen(topic);
    publish_info.pPayload = mqtt_message;
    publish_info.payloadLength = strlen(mqtt_message);

    /* Get a unique packet id. */
    publish_packet_identifier = MQTT_GetPacketId(p_mqtt_context);

    LogInfo(("Publish to the MQTT topic %s.", topic));

    /* Send PUBLISH packet. Packet ID is not used for a QoS1 publish. */
    mqtt_status = MQTT_Publish(p_mqtt_context, &publish_info, publish_packet_identifier);
    check_mqtt_status(mqtt_status, "Publish");

    return demo_status;
}

/*******************************************************************************
 * Function Name: create_mqtt_connection_with_broker
 ********************************************************************************
 * Summary:
 * This function establishes a MQTT connection with the broker
 *
 * Parameters:
 *  MQTTContext_t *p_mqtt_context: MQTT operation context
 *  NetworkContext_t * p_network_context: Network operation context
 *
 * Return:
 *  int status : `pdTRUE` if operation is successful 
 *               `pdFAIL` otherwise.
 *
 *******************************************************************************/
static int create_mqtt_connection_with_broker(MQTTContext_t * p_mqtt_context,
        NetworkContext_t * p_network_context)
{
    MQTTStatus_t result;
    MQTTConnectInfo_t connect_info;
    bool session_present;
    TransportInterface_t transport;
    BaseType_t status = pdFAIL;

    /* Fill in Transport Interface send and receive function pointers. */
    transport.pNetworkContext = p_network_context;
    transport.send = SecureSocketsTransport_Send;
    transport.recv = SecureSocketsTransport_Recv;

    /* Initialize MQTT library. */
    result = MQTT_Init(p_mqtt_context, &transport, get_time_ms, mqtt_event_callback, &mqtt_buffer);
    configASSERT(result == MQTTSuccess);

    /* Some fields are not used in this demo so start with everything at 0. */
    (void) memset((void *) &connect_info, 0x00, sizeof(connect_info));

    /* Start with a clean session */
    connect_info.cleanSession = true;

    /* Set the unique client identifier */
    connect_info.pClientIdentifier = clientcredentialIOT_THING_NAME;
    connect_info.clientIdentifierLength = (uint16_t) strlen(clientcredentialIOT_THING_NAME);

    /* Use the metrics string as username to report the OS and MQTT client version
     * metrics to AWS IoT. */
    connect_info.pUserName = AWS_IOT_METRICS_STRING;
    connect_info.userNameLength = AWS_IOT_METRICS_STRING_LENGTH;

    /* Set MQTT keep-alive period */
    connect_info.keepAliveSeconds = KEEP_ALIVE_TIMEOUT_SECONDS;

    /* Connect to MQTT broker */
    result = MQTT_Connect(p_mqtt_context,
            &connect_info,
            NULL,
            CONNACK_RECV_TIMEOUT_MS,
            &session_present);

    /* Check status */
    if(result != MQTTSuccess)
    {
        LogError(("Failed to establish MQTT connection: Server=%s, MQTTStatus=%s",
                clientcredentialMQTT_BROKER_ENDPOINT, MQTT_Status_strerror(result)));
    }
    else
    {
        /* Successfully established and MQTT connection with the broker. */
        LogInfo(("An MQTT connection is established with %s.", clientcredentialMQTT_BROKER_ENDPOINT));
        status = pdPASS;
    }

    return status;
}

/*******************************************************************************
 * Function Name: connect_to_server
 ********************************************************************************
 * Summary:
 * This function establishes a secure TLS connection with the server
 *
 * Parameters:
 *  NetworkContext_t * p_network_context: Network operation context
 *
 * Return:
 *  int status : `pdTRUE` if operation is successful 
 *               `pdFAIL` otherwise.
 *
 *******************************************************************************/
static int connect_to_server(NetworkContext_t * p_network_context)
{
    ServerInfo_t server_info = { 0 };
    SocketsConfig_t sockets_config = { 0 };
    TransportSocketStatus_t network_status = TRANSPORT_SOCKET_STATUS_SUCCESS;
    BackoffAlgorithmContext_t reconnect_params;
    BaseType_t backoff_status = pdFALSE;

    /* Set the credentials for establishing a TLS connection. */
    /* Initializer server information. */
    server_info.pHostName = clientcredentialMQTT_BROKER_ENDPOINT;
    server_info.hostNameLength = strlen(clientcredentialMQTT_BROKER_ENDPOINT);
    server_info.port = clientcredentialMQTT_BROKER_PORT;

    /* Configure credentials for TLS mutual authenticated session. */
    sockets_config.enableTls = true;
    sockets_config.pAlpnProtos = NULL;
    sockets_config.maxFragmentLength = 0;
    sockets_config.disableSni = false;
    sockets_config.pRootCa = tlsATS1_ROOT_CERTIFICATE_PEM;
    sockets_config.rootCaSize = sizeof(tlsATS1_ROOT_CERTIFICATE_PEM);
    sockets_config.sendTimeoutMs = TRANSPORT_SEND_RECV_TIMEOUT_MS;
    sockets_config.recvTimeoutMs = TRANSPORT_SEND_RECV_TIMEOUT_MS;

    /* Initialize reconnect attempts and interval. */
    BackoffAlgorithm_InitializeParams(&reconnect_params,
            RETRY_BACKOFF_BASE_MS,
            RETRY_MAX_BACKOFF_DELAY_MS,
            RETRY_MAX_ATTEMPTS);

    /* Attempt to connect to MQTT broker. If connection fails, retry after
     * a timeout. Timeout value will exponentially increase till maximum
     * attempts are reached.
     */
    do
    {
        /* Attempt to create a mutually authenticated TLS connection. */
        network_status = SecureSocketsTransport_Connect(p_network_context,
                &server_info,
                &sockets_config);

        if(network_status != TRANSPORT_SOCKET_STATUS_SUCCESS)
        {
            LogWarn(("Connection to the broker failed. Attempting connection retry after backoff delay."));

            /* As the connection attempt failed, we will retry the connection after an
             * exponential backoff with jitter delay. */

            /* Calculate the backoff period for the next retry attempt and perform the wait operation. */
            backoff_status = backoff_for_retry(&reconnect_params);
        }
    } while((network_status != TRANSPORT_SOCKET_STATUS_SUCCESS) && (backoff_status == pdPASS));

    return (network_status == TRANSPORT_SOCKET_STATUS_SUCCESS) ? pdPASS : pdFAIL;
}

/*******************************************************************************
 * Function Name: backoff_for_retry
 ********************************************************************************
 * Summary:
 * This function calculates the backoff period for next retry attempt
 *
 * Parameters:
 *  BackoffAlgorithmContext_t * p_retry_params: pointer to retry parameter
 *
 * Return:
 *  int status : `pdPASS` if operation is successful 
 *               `pdFAIL` otherwise.
 *
 *******************************************************************************/
static int backoff_for_retry(BackoffAlgorithmContext_t * p_retry_params)
{
    BaseType_t xReturnStatus = pdFAIL;
    uint16_t us_next_retry_backoff = 0U;
    BackoffAlgorithmStatus_t backoff_algo_status = BackoffAlgorithmSuccess;

    /**
     * To calculate the backoff period for the next retry attempt, we will
     * generate a random number to provide to the backoffAlgorithm library.
     *
     * Note: The PKCS11 module is used to generate the random number as it allows access
     * to a True Random Number Generator (TRNG) if the vendor platform supports it.
     * It is recommended to use a random number generator seeded with a device-specific
     * entropy source so that probability of collisions from devices in connection retries
     * is mitigated.
     */
    uint32_t ul_random_num = 0;

    if(xPkcs11GenerateRandomNumber((uint8_t *) &ul_random_num,
            sizeof(ul_random_num)) == pdPASS)
    {
        /* Get back-off value (in milliseconds) for the next retry attempt. */
        backoff_algo_status = BackoffAlgorithm_GetNextBackoff(p_retry_params, ul_random_num, &us_next_retry_backoff);

        if(backoff_algo_status == BackoffAlgorithmRetriesExhausted)
        {
            LogError(("All retry attempts have exhausted. Operation will not be retried"));
        }
        else if(backoff_algo_status == BackoffAlgorithmSuccess)
        {
            /* Perform the backoff delay. */
            vTaskDelay(pdMS_TO_TICKS(us_next_retry_backoff));

            xReturnStatus = pdPASS;

            LogInfo(("Retry attempt %lu out of maximum retry attempts %lu.",
                    (p_retry_params->attemptsDone + 1),
                    p_retry_params->maxRetryAttempts));
        }
    }
    else
    {
        LogError(("Unable to retry operation with broker: Random number generation failed"));
    }

    return xReturnStatus;
}

/*******************************************************************************
 * Function Name: get_time_ms
 ********************************************************************************
 * Summary:
 * This function calculates the elapsed time in milliseconds
 *
 * Parameters:
 *  void
 *
 * Return:
 *  int time: Time elapsed in milliseconds
 *
 *******************************************************************************/
static uint32_t get_time_ms(void)
{
    TickType_t xTickCount = 0;
    uint32_t ul_time_ms = 0UL;

    /* Get the current tick count. */
    xTickCount = xTaskGetTickCount();

    /* Convert the ticks to milliseconds. */
    ul_time_ms = (uint32_t) xTickCount * MILLISECONDS_PER_TICK;

    /* Reduce global_entry_time_ms from obtained time so as to always return the
     * elapsed time in the application. */
    ul_time_ms = (uint32_t) (ul_time_ms - global_entry_time_ms);

    return ul_time_ms;
}

/*******************************************************************************
 * Function Name: mqtt_event_callback
 ********************************************************************************
 * Summary:
 * This function calculates the elapsed time in milliseconds
 *
 * Parameters:
 *  MQTTContext_t *p_mqtt_context: MQTT operation context
 *  MQTTPacketInfo_t * p_packet_info: MQTT packet information
 *  MQTTDeserializedInfo_t * p_deserialized_info: MQTT deserialized information
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void mqtt_event_callback(MQTTContext_t * p_mqtt_context,
        MQTTPacketInfo_t * p_packet_info,
        MQTTDeserializedInfo_t * p_deserialized_info)
{
    /* The MQTT context is not used for this demo. */
    (void) p_mqtt_context;

    /* Check if the packet type is PUBLISH */
    if((p_packet_info->type & 0xF0U) == MQTT_PACKET_TYPE_PUBLISH)
    {
        mqtt_process_incoming_publish(p_deserialized_info->pPublishInfo);
    }
    else
    {
        mqtt_process_response(p_packet_info, p_deserialized_info->packetIdentifier);
    }
}

/*******************************************************************************
 * Function Name: mqtt_process_incoming_publish
 ********************************************************************************
 * Summary:
 * This function processes the incoming publish packet
 *
 * Parameters:
 *  MQTTPublishInfo_t * p_publish_info: Pointer to publish information
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void mqtt_process_incoming_publish(MQTTPublishInfo_t * p_publish_info)
{
    configASSERT(p_publish_info != NULL);

    /* Set the global for indicating that an incoming publish is received. */
    packet_type_received = MQTT_PACKET_TYPE_PUBLISH;

    /* Process incoming Publish. */
    LogInfo(("Incoming QoS : %d\n", p_publish_info->qos));

    /* Verify the received publish is for the we have subscribed to. */
    if((p_publish_info->topicNameLength == strlen(MQTT_TOPIC)) &&
            (0 == strncmp(MQTT_TOPIC, p_publish_info->pTopicName, p_publish_info->topicNameLength)))
    {
        LogInfo(("Incoming Publish Topic Name: %.*s matches subscribed topic."
                "Incoming Publish Message : %.*s",
                p_publish_info->topicNameLength,
                p_publish_info->pTopicName,
                p_publish_info->payloadLength,
                p_publish_info->pPayload));
    }
    else
    {
        LogInfo(("Incoming Publish Topic Name: %.*s does not match subscribed topic.",
                p_publish_info->topicNameLength,
                p_publish_info->pTopicName));
    }
}

/*******************************************************************************
 * Function Name: wait_for_packet
 ********************************************************************************
 * Summary:
 * This function waits for server to send the acknowledgement packet back
 *
 * Parameters:
 *  MQTTContext_t * p_mqtt_context: Pointer to MQTT operation context
 *  uint16_t packet_type: Packet type
 *
 * Return:
 *  MQTTStatus_t status: Status of MQTT operation
 *
 *******************************************************************************/
static MQTTStatus_t wait_for_packet(MQTTContext_t * p_mqtt_context,
        uint16_t packet_type)
{
    uint8_t count = 0U;
    MQTTStatus_t mqtt_status = MQTTSuccess;

    /* Reset the packet type received. */
    packet_type_received = 0U;

    /* Wait till packet is received or timeout */
    while((packet_type_received != packet_type) &&
            (count++ < MQTT_PROCESS_LOOP_PACKET_WAIT_COUNT_MAX) &&
            (mqtt_status == MQTTSuccess))
    {
        /* Event callback */
        mqtt_status = MQTT_ProcessLoop(p_mqtt_context, PROCESS_LOOP_TIMEOUT_MS);
    }

    /* Check status */
    if((mqtt_status != MQTTSuccess) || (packet_type_received != packet_type))
    {
        LogError(("MQTT_ProcessLoop failed to receive packet: Packet type=%02X, LoopDuration=%u, Status=%s",
                packet_type,
                (PROCESS_LOOP_TIMEOUT_MS * count),
                MQTT_Status_strerror(mqtt_status)));
    }

    return mqtt_status;
}

/*******************************************************************************
 * Function Name: mqtt_subscribe
 ********************************************************************************
 * Summary:
 * This function subscribes to the specified MQTT topic
 *
 * Parameters:
 *  MQTTContext_t * p_mqtt_context: Pointer to MQTT operation context
 *
 * Return:
 *  int status : `pdPASS` if operation is successful 
 *               `pdFAIL` otherwise.
 *
 *******************************************************************************/
static int mqtt_subscribe(MQTTContext_t * p_mqtt_context)
{
    MQTTStatus_t mqtt_status = MQTTSuccess;
    BackoffAlgorithmContext_t retry_params;
    BaseType_t backoff_status = pdFAIL;
    MQTTSubscribeInfo_t mqtt_subscription[MQTT_TOPIC_COUNT];
    BaseType_t failed_subscribe_to_topic = pdFALSE;
    uint32_t topic_count = 0U;
    BaseType_t demo_status = pdFAIL;

    /* Some fields not used by this demo so start with everything at 0. */
    (void) memset((void *) &mqtt_subscription, 0x00, sizeof(mqtt_subscription));

    /* Get a unique packet id. */
    subscribe_packet_identifier = MQTT_GetPacketId(p_mqtt_context);

    /* Subscribe to the mqttexampleTOPIC topic filter. This example subscribes to
     * only one topic and uses QoS1. */
    mqtt_subscription[0].qos = MQTTQoS1;
    mqtt_subscription[0].pTopicFilter = MQTT_TOPIC;
    mqtt_subscription[0].topicFilterLength = (uint16_t) strlen(MQTT_TOPIC);

    /* Initialize retry attempts and interval. */
    BackoffAlgorithm_InitializeParams(&retry_params,
            RETRY_BACKOFF_BASE_MS,
            RETRY_MAX_BACKOFF_DELAY_MS,
            RETRY_MAX_ATTEMPTS);

    do
    {
        /* The client is now connected to the broker. Subscribe to the topic */
        LogInfo(("Attempt to subscribe to the MQTT topic %s.", MQTT_TOPIC));
        mqtt_status = MQTT_Subscribe(p_mqtt_context,
                mqtt_subscription,
                sizeof(mqtt_subscription) / sizeof(MQTTSubscribeInfo_t),
                subscribe_packet_identifier);
        check_mqtt_status(mqtt_status, "Subscribe");

        if(mqtt_status != MQTTSuccess)
        {
            LogError(("Failed to SUBSCRIBE to MQTT topic %s. Error=%s",
                    MQTT_TOPIC, MQTT_Status_strerror(mqtt_status)));
        }
        else
        {
            demo_status = pdPASS;
            LogInfo(("SUBSCRIBE sent for topic %s to broker.", MQTT_TOPIC));

            /* Process incoming packet from the broker */
            mqtt_status = wait_for_packet(p_mqtt_context, MQTT_PACKET_TYPE_SUBACK);
            check_mqtt_status(mqtt_status, "Wait for Packet");
        }

        if(demo_status == pdPASS)
        {
            /* Reset flag before checking suback responses. */
            failed_subscribe_to_topic = pdFALSE;

            /* Check if recent subscription request has been rejected */
            for(topic_count = 0; topic_count < MQTT_TOPIC_COUNT; topic_count++)
            {
                if(topic_filter_context[topic_count].xSubAckStatus == MQTTSubAckFailure)
                {
                    failed_subscribe_to_topic = pdTRUE;

                    /* As the subscribe attempt failed, we will retry the connection after an
                     * exponential backoff with jitter delay. */

                    /* Retry subscribe after exponential back-off. */
                    LogWarn(("Server rejected subscription request. Attempting to re-subscribe to topic %s.",
                            topic_filter_context[topic_count].pcTopicFilter));

                    backoff_status = backoff_for_retry(&retry_params);

                    break;
                }
            }
        }
    } while((failed_subscribe_to_topic == pdTRUE) && (backoff_status == pdPASS));

    return demo_status;
}

/*******************************************************************************
 * Function Name: mqtt_process_response
 ********************************************************************************
 * Summary:
 * This function processes the incoming response packet
 *
 * Parameters:
 *  MQTTPacketInfo_t * incoming_packet: Pointer to incoming packet
 *  uint16_t packet_id: ID of the packet
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void mqtt_process_response(MQTTPacketInfo_t * incoming_packet,
        uint16_t packet_id)
{
    uint32_t topic_count = 0U;

    switch(incoming_packet->type)
    {
    case MQTT_PACKET_TYPE_PUBACK:

        LogInfo(("PUBACK received for packet Id %u.", packet_id));

        /* Make sure ACK packet identifier matches with Request packet identifier. */
        configASSERT(publish_packet_identifier == packet_id);
        break;

    case MQTT_PACKET_TYPE_SUBACK:

        /* Update the packet type received to SUBACK. */
        packet_type_received = MQTT_PACKET_TYPE_SUBACK;

        /* Check the server response to our subscription request */
        update_sub_ack_status(incoming_packet);

        for(topic_count = 0; topic_count < MQTT_TOPIC_COUNT; topic_count++)
        {
            if(topic_filter_context[ topic_count ].xSubAckStatus != MQTTSubAckFailure)
            {
                LogInfo(("Subscribed to the topic %s with maximum QoS %u.",
                        topic_filter_context[ topic_count ].pcTopicFilter,
                        topic_filter_context[ topic_count ].xSubAckStatus));
            }
        }

        /* Make sure ACK packet identifier matches with Request packet identifier. */
        configASSERT(subscribe_packet_identifier == packet_id);

        break;

    case MQTT_PACKET_TYPE_UNSUBACK:

        LogInfo(("Unsubscribed from the topic %s.", MQTT_TOPIC));

        /* Update the packet type received to UNSUBACK. */
        packet_type_received = MQTT_PACKET_TYPE_UNSUBACK;

        /* Make sure ACK packet identifier matches with Request packet identifier. */
        configASSERT(unsubscribe_packet_identifier == packet_id);

        break;

    case MQTT_PACKET_TYPE_PINGRESP:

        LogInfo(("Ping Response successfully received."));

        break;

        /* Any other packet type is invalid. */
    default:
        LogWarn(("mqtt_process_response() called with unknown packet type:(%02X).",
                incoming_packet->type));
    }
}

/*******************************************************************************
 * Function Name: update_sub_ack_status
 ********************************************************************************
 * Summary:
 * This function updates the status of published packets
 *
 * Parameters:
 *  MQTTPacketInfo_t * incoming_packet: Pointer to incoming packet
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void update_sub_ack_status(MQTTPacketInfo_t * p_packet_info)
{
    MQTTStatus_t mqtt_status = MQTTSuccess;
    uint8_t * payload = NULL;
    size_t size = 0;
    uint32_t topic_count = 0U;

    /* Get the status codes */
    mqtt_status = MQTT_GetSubAckStatusCodes(p_packet_info, &payload, &size);

    /* MQTT_GetSubAckStatusCodes always returns success if called with packet info
     * from the event callback and non-NULL parameters. */
    configASSERT(mqtt_status == MQTTSuccess);

    for(topic_count = 0; topic_count < size; topic_count++)
    {
        topic_filter_context[topic_count].xSubAckStatus = payload[topic_count];
    }
}

/*******************************************************************************
 * Function Name: mqtt_unsubscribe_from_topic
 ********************************************************************************
 * Summary:
 * This function unsubscribes from subscribed topics
 *
 * Parameters:
 *  MQTTContext_t * p_mqtt_context: Pointer to MQTT operation context
 *
 * Return:
 *  int status : `pdPASS` if operation is successful 
 *               `pdFAIL` otherwise.
 *
 *******************************************************************************/
static int mqtt_unsubscribe_from_topic(MQTTContext_t * p_mqtt_context)
{
    MQTTStatus_t mqtt_status;
    MQTTSubscribeInfo_t mqtt_subscription[MQTT_TOPIC_COUNT];
    BaseType_t demo_status = pdPASS;

    /* Some fields not used by this demo so start with everything at 0. */
    (void) memset((void *) &mqtt_subscription, 0x00, sizeof(mqtt_subscription));

    /* Get a unique packet id. */
    subscribe_packet_identifier = MQTT_GetPacketId(p_mqtt_context);

    /* Subscribe to the mqttexampleTOPIC topic filter. This example subscribes to
     * only one topic and uses QoS1. */
    mqtt_subscription[0].qos = MQTTQoS1;
    mqtt_subscription[0].pTopicFilter = MQTT_TOPIC;
    mqtt_subscription[0].topicFilterLength = (uint16_t) strlen(MQTT_TOPIC);

    /* Get next unique packet identifier. */
    unsubscribe_packet_identifier = MQTT_GetPacketId(p_mqtt_context);

    /* Send UNSUBSCRIBE packet. */
    mqtt_status = MQTT_Unsubscribe(p_mqtt_context,
            mqtt_subscription,
            sizeof(mqtt_subscription) / sizeof(MQTTSubscribeInfo_t),
            unsubscribe_packet_identifier);

    /* Check status */
    if(mqtt_status != MQTTSuccess)
    {
        demo_status = pdFAIL;
        LogError(("Failed to send UNSUBSCRIBE request to broker: TopicFilter=%s, Error=%s",
                MQTT_TOPIC,
                MQTT_Status_strerror(mqtt_status)));
    }

    return demo_status;
}

/* [] END OF FILE */
