#pragma once

#include <Arduino.h>
#include <map>
#include <painlessMesh.h>

#define TYPE_ALL 0

template <typename T> class Message {
 public:
    uint8_t seq;
    uint8_t type;
    uint8_t index;
    T       payload;
};


typedef void (*Callback)(const Message<byte[4]>& payload);

class Comm {
private:
    uint8_t id;

    uint8_t seq = 0;

    std::multimap<uint8_t, Callback> callbacks;

public:

    Comm(uint8_t id): id(id) {};

    void nextSequence() {seq++;};

    // Receiving
    template <typename T>
        void listen(uint8_t type,
                    void (*callback)(const Message<T>& message)) {
        listen(type, reinterpret_cast<Callback>(callback));
    }

    // Sending
    void send(uint8_t type, uint8_t index);
    template <typename T>
        void send(uint8_t type, uint8_t index, const T& payload) {
        return send(type, index, reinterpret_cast<const byte*>(&payload));
    }


    /* template <typename T> */
    /*     bool expect(uint8_t type, uint8_t* index, T& payload) { */
    /*     return expect(type, index, reinterpret_cast<byte*>(&payload)); */
    /* } */


    bool receive(String &str);

private:
    void listen(uint8_t type,
                void (*callback)(const Message<byte[4]>& message));
    void send(uint8_t type, uint8_t index, const byte* payload);
    /* bool expect(uint8_t type, uint8_t* index, byte* payload); */
    /* void expect(uint8_t type, */
    /*             void (*callback)(uint8_t index, const byte* payload)); */

protected:
    virtual void writeLine(char* str) = 0;
};

class SerialComm: public Comm {
public:
    SerialComm(uint8_t id): Comm::Comm(id) {};
    void update();

private:
    void writeLine(char* str);
};

class MeshComm: public Comm {
public:
    painlessMesh mesh;

MeshComm(uint8_t id): Comm::Comm(id) {};

    bool init(Scheduler *scheduler);
    void update();

private:
    void writeLine(char* str);
};
