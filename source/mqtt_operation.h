/******************************************************************************
* File Name: mqtt_operation.h
*
* Description: This file contains function declarations related to MQTT
* operations.
*
*******************************************************************************
* Copyright 2018-2021, Cypress Semiconductor Corporation (an Infineon company) or
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
#ifndef MQTT_OPERATION_H_
#define MQTT_OPERATION_H_

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdint.h>
#include "core_mqtt.h"
#include "backoff_algorithm.h"
#include "pkcs11_helpers.h"
#include "transport_secure_sockets.h"
#include "gesture.h"

/*******************************************************************************
 * Defines
 ******************************************************************************/
/* MQTT Broker info */
#define MQTT_TIMEOUT_MS                    (5000)
#define TOPIC_FILTER_COUNT                 (2)
#define PUBLISH_RETRY_LIMIT                (10)
#define PUBLISH_RETRY_MS                   (1000)
#define MAX_JSON_MESSAGE_LENGTH            (100)
#define MAX_TOPIC_LENGTH                   (50)
#define RETRY_MAX_ATTEMPTS                 (5U)
#define RETRY_MAX_BACKOFF_DELAY_MS         (5000U)
#define RETRY_BACKOFF_BASE_MS              (500U)
#define TRANSPORT_SEND_RECV_TIMEOUT_MS     (500U)
#define KEEP_ALIVE_TIMEOUT_SECONDS         (120U)
#define CONNACK_RECV_TIMEOUT_MS            (1000U)
#define MILLISECONDS_PER_SECOND            (1000U)
#define MILLISECONDS_PER_TICK              (MILLISECONDS_PER_SECOND / configTICK_RATE_HZ)

/* Queue length of message queues used in this project */
#define SINGLE_ELEMENT_QUEUE               (1u)

/*******************************************************************************
 * Data structure and enumeration
 ******************************************************************************/
/* Structure used for storing MQTT data */
typedef struct
{
    gesture_t detected_gesture;
} mqtt_data_t;

struct NetworkContext
{
    SecureSocketsTransportParams_t * pParams;
};

/*******************************************************************************
 * MQTT functions
 ******************************************************************************/

/* Thread function */
void mqtt_task(void* param);

int establish_mqtt_connection(MQTTContext_t *p_mqtt_context,
        NetworkContext_t * p_network_context,
        char* topic,
        char* mqtt_message);

int publish_message( MQTTContext_t *p_mqtt_context,
                     char* topic,
                     uint16_t topic_length,
                     char* mqtt_message,
                     uint16_t message_length);

int disconnect_connection(MQTTContext_t *p_mqtt_context,
        NetworkContext_t * p_network_context,
        BaseType_t is_connection_established,
        BaseType_t is_mqtt_connection_established);

void check_mqtt_status(MQTTStatus_t mqtt_status, char* operation);
void print_status(BaseType_t demo_status, char *error_message, char *success_message);

#endif /* MQTT_OPERATION_H_ */
