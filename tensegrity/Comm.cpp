#include "Comm.h"


Message<byte[4]> makeMessage(byte *bytes) {
    Message<byte[4]> m;
    m.seq        = bytes[1];
    m.type       = bytes[2];
    m.index      = bytes[3];
    m.payload[0] = bytes[4];
    m.payload[1] = bytes[5];
    m.payload[2] = bytes[6];
    m.payload[3] = bytes[7];
    return m;
}


void Comm::listen(uint8_t type,
            void (*callback)(const Message<byte[4]>& message))  {
    callbacks.insert(std::make_pair(type, callback));
}


bool Comm::receive(String &str) {
    char *err = NULL;
    byte bytes[8];
    for (int i = 0; i < 8; i++) {
        char hex[3];
        hex[0] = str.charAt(2 * i);
        hex[1] = str.charAt(2 * i + 1);
        hex[2] = '\0';
        bytes[i] = strtol(hex, &err, 16);
    }
    if (*err != '\0')   return false;
    if (bytes[0] != id) return false;

    Message<byte[4]> message = makeMessage(bytes);

    auto end = callbacks.end();
    for (auto itr = callbacks.find(message.type); itr != end; itr++)
        (*itr->second)(message);
    for (auto itr = callbacks.find(TYPE_ALL); itr != end; itr++)
        (*itr->second)(message);

    return true;
}

void Comm::send(uint8_t type, uint8_t index, const byte* payload) {
    byte data[8] = {id, seq, type, index};
    for (int i = 0; i < 4; i++) data[4+i] = payload[i];

    char str[17];

    for (int i = 0; i < 8; i++) {
        char hex[3];
        sprintf(hex, "%X", data[i]);
        if (data[i] < 16) {
            str[i*2]   = '0';
            str[i*2+1] = hex[0];
        }
        else {
            str[i*2]   = hex[0];
            str[i*2+1] = hex[1];
        }
    }
    str[16] = '\0';
    writeLine(str);
}

void Comm::send(uint8_t type, uint8_t index) {
    byte empty[4];
    send(type, index, empty);
}


void SerialComm::update() {
    while (Serial.available() > 0) {
        String str = Serial.readStringUntil('\n');
        receive(str);
    }
}

void SerialComm::writeLine(char* str) {
    Serial.println(str);
}

#define MESH_PREFIX   "tensegrity"
#define MESH_PASSWORD "tensegrity"
#define MESH_PORT     5555

MeshComm *meshComm = nullptr;

void receivedCallback(uint32_t from, String &msg);
void newConnectionCallback(uint32_t nodeId);
void changedConnectionCallback();
void nodeTimeAdjustedCallback(int32_t offset);


bool MeshComm::init(Scheduler *scheduler) {
    if (meshComm != nullptr) {
        log_e("MeshComm must be unique");
        return false;
    }
    meshComm = this;

    // mesh.setDebugMsgTypes(ERROR | MESH_STATUS | CONNECTION | SYNC | COMMUNICATION | GENERAL | MSG_TYPES | REMOTE);
    mesh.setDebugMsgTypes(ERROR | STARTUP | MESH_STATUS | CONNECTION | SYNC | GENERAL | MSG_TYPES | REMOTE);
    mesh.init(MESH_PREFIX, MESH_PASSWORD, scheduler, MESH_PORT);
    mesh.onReceive(&receivedCallback);
    mesh.onNewConnection(&newConnectionCallback);
    mesh.onChangedConnections(&changedConnectionCallback);
    mesh.onNodeTimeAdjusted(&nodeTimeAdjustedCallback);

    return true;
}

void MeshComm::update() {
    mesh.update();
}

void MeshComm::writeLine(char* str) {
    mesh.sendBroadcast(str);
}



void receivedCallback( uint32_t from, String &msg) {
    meshComm->receive(msg);
    Serial.printf("startHere: Received from %u msg=%s\n", from, msg.c_str());
}

void newConnectionCallback(uint32_t nodeId) {
    Serial.printf("--> startHere: New Connection, nodeId = %u\n", nodeId);
}

void changedConnectionCallback() {
    Serial.print("Changed connections\n");
}

void nodeTimeAdjustedCallback(int32_t offset) {
    Serial.printf("Adjusted time %u. Offset = %d\n", meshComm->mesh.getNodeTime(),offset);
}

