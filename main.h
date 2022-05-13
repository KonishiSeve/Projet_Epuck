#ifndef MAIN_H
#define MAIN_H

//DELETE ?
#ifdef __cplusplus
extern "C" {
#endif

#include "camera/dcmi_camera.h"
#include "msgbus/messagebus.h"
#include "parameter/parameter.h"
#include "sensors/proximity.h"

#define IMAGE_BUFFER_SIZE 640

/** Robot wide IPC bus. */
extern messagebus_t bus;
extern parameter_namespace_t parameter_root;

//DELETE
void SendUint8ToComputer(uint8_t* data, uint16_t size);

//DELETE ?
#ifdef __cplusplus
}
#endif

#endif
