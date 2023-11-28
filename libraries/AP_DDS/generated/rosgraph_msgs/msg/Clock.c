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
 * @file Clock.c
 * This source file contains the definition of the described types in the IDL file.
 *
 * This file was generated by the tool gen.
 */

#include "Clock.h"

#include <ucdr/microcdr.h>
#include <string.h>

bool rosgraph_msgs_msg_Clock_serialize_topic(ucdrBuffer* writer, const rosgraph_msgs_msg_Clock* topic)
{
    bool success = true;

        success &= builtin_interfaces_msg_Time_serialize_topic(writer, &topic->clock);
    return success && !writer->error;
}

bool rosgraph_msgs_msg_Clock_deserialize_topic(ucdrBuffer* reader, rosgraph_msgs_msg_Clock* topic)
{
    bool success = true;

        success &= builtin_interfaces_msg_Time_deserialize_topic(reader, &topic->clock);
    return success && !reader->error;
}

uint32_t rosgraph_msgs_msg_Clock_size_of_topic(const rosgraph_msgs_msg_Clock* topic, uint32_t size)
{
    uint32_t previousSize = size;
        size += builtin_interfaces_msg_Time_size_of_topic(&topic->clock, size);
    return size - previousSize;
}
