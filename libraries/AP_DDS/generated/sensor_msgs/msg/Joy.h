// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*!
 * @file Joy.h
 * This header file contains the declaration of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#ifndef _Joy_H_
#define _Joy_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <stdbool.h>
#include "std_msgs/msg/Header.h"

typedef struct sensor_msgs_msg_Joy
{
    std_msgs_msg_Header header;
    uint32_t axes_size;
    float axes[100];

    uint32_t buttons_size;
    int32_t buttons[100];

} sensor_msgs_msg_Joy;

struct ucdrBuffer;

bool sensor_msgs_msg_Joy_serialize_topic(struct ucdrBuffer* writer, const sensor_msgs_msg_Joy* topic);
bool sensor_msgs_msg_Joy_deserialize_topic(struct ucdrBuffer* reader, sensor_msgs_msg_Joy* topic);
uint32_t sensor_msgs_msg_Joy_size_of_topic(const sensor_msgs_msg_Joy* topic, uint32_t size);


#ifdef __cplusplus
}
#endif

#endif // _Joy_H_