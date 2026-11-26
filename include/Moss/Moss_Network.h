
//                        MIT License
//
//                  Copyright (c) 2026 Toby
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

/* =================================================================
Modified ENet6 v6.1.2 by SirLynix https://github.com/SirLynix/enet6/
================================================================= */

/*!
 * @file Moss_Network.h
 * @brief High-performance networking layer powered by ENet 6 for the Moss Framework.
 * https://github.com/SirLynix/enet6/
 *
 * The Moss networking module provides a lightweight, low-latency, and reliable communication layer
 * built on **ENet 6**, an advanced version of the classic ENet library designed for real-time
 * multiplayer games, simulation, and XR synchronization.
 *
 * ---
 *
 * ### Core Features:
 * - **Connection-Oriented UDP** — Uses ENet 6’s hybrid protocol for reliability without TCP overhead.
 * - **Automatic Packet Fragmentation / Reassembly** — Transparent large-packet handling with zero-copy transfer.
 * - **Reliability Channels** — Distinct logical channels for reliable and unreliable data streams.
 * - **Low-Latency Message Delivery** — Fine-grained control over packet priority and delivery guarantees.
 * - **Peer Management** — Full lifecycle handling for connection, disconnection, and peer state tracking.
 * - **Compression & Security** — Optional zlib / LZ4 compression and encryption hooks for secure, efficient communication.
 *
 * ---
 *
 * ### Common Use Cases:
 * - Multiplayer gameplay replication (e.g., player movement, events, physics states).
 * - Host-authoritative or peer-to-peer game logic.
 * - Real-time XR world synchronization.
 * - Dedicated game servers, lobbies, or matchmaking systems.
 *
 * ---
 *
 * ### Advanced Features:
 * - **Multichannel System:**  
 *   Supports isolated communication lanes such as:
 *   - Player Input / State Replication
 *   - Chunk or Level Streaming
 *   - Entity Synchronization
 *   - Chat and UI Events
 *
 * - **Packet Abstraction Layer:**  
 *   Moss wraps ENet packets into type-safe structures (e.g., `Packet_EntityUpdate`, `Packet_PlayerState`),
 *   enabling deterministic serialization for cross-platform play.
 *
 * - **Threaded Networking:**  
 *   Optional worker thread mode for asynchronous send/receive, minimizing latency on the main loop.
 *
 * - **Server Authority Layer:**  
 *   Integrated logic for validating client packets and preventing unauthorized world manipulation.
 *
 * ---
 *
 * ### Design Goals:
 * - Lightweight, efficient, and dependency-free beyond ENet.
 * - Deterministic packet ordering for gameplay-critical updates.
 * - Extendable for cloud-based hosting, replay capture, and mod networking.
 * - Unified API for both **client** and **server** roles.
 *
 * ---
 * 
 * Status:
 *  - UDP / ENet-style protocol ........ IMPLEMENTED
 *  - TCP transport & framing .......... IMPLEMENTED
 *  - WebRTC (data + signaling hooks) .. IMPLEMENTED
 *  - HTTP/REST (async, TLS, server) ... IMPLEMENTED
 *
 * Partially Implemented / Scaffolded:
 *  - WebSocket (native support; WASM bridge pending)
 *  - NAT traversal (UDP punch + STUN/TURN hooks)
 *
 * Planned / Future Extensions:
 *  - QUIC / WebTransport backend
 *  - Relay server support
 *  - Scripting layer integration
 */

#ifndef MOSS_NETWORK_H
#define MOSS_NETWORK_H

#include <Moss/Moss_stdinc.h>

#include <stdlib.h>
#include <cstdint>
#include <cstddef>

#if defined(MOSS_PLATFORM_WINDOWS)

    #ifdef MOSS_COMPILER_MSVC
        #ifdef ENET_BUILDING_LIB
            #pragma warning(disable: 4267)
            #pragma warning(disable: 4244)
            #pragma warning(disable: 4018)
            #pragma warning(disable: 4146)
        #endif

        #ifndef _CRT_SECURE_NO_DEPRECATE
            #define _CRT_SECURE_NO_DEPRECATE
        #endif

        #ifndef _CRT_SECURE_NO_WARNINGS
            #define _CRT_SECURE_NO_WARNINGS
        #endif
    #endif

    // Must come before any Windows header: avoids the old winsock.h clash and the min/max macros.
    #ifndef WIN32_LEAN_AND_MEAN
        #define WIN32_LEAN_AND_MEAN
    #endif
    #ifndef NOMINMAX
        #define NOMINMAX
    #endif
    #include <winsock2.h>
    #include <ws2tcpip.h>

    #define ENET_SOCKETSET_EMPTY(sockset)          FD_ZERO(&(sockset))
    #define ENET_SOCKETSET_ADD(sockset, socket)    FD_SET((socket), &(sockset))
    #define ENET_SOCKETSET_REMOVE(sockset, socket) FD_CLR((socket), &(sockset))
    #define ENET_SOCKETSET_CHECK(sockset, socket)  FD_ISSET((socket), &(sockset))

    #define ENET_SOCKET_NULL INVALID_SOCKET
    #define CLOSE_SOCKET(s) closesocket((s))
    #define SOCK_ERRNO WSAGetLastError()
    #define SOCK_EWOULDBLOCK WSAEWOULDBLOCK

    using ENetSocket = SOCKET;
    using ENetSocketSet = fd_set;

#elif defined(MOSS_PLATFORM_LINUX) || \
      defined(MOSS_PLATFORM_MACOS) || \
      defined(MOSS_PLATFORM_ANDROID) || \
      defined(MOSS_PLATFORM_IOS) || \
      defined(MOSS_PLATFORM_BSD) || \
      defined(MOSS_PLATFORM_UNIX)

    #include <errno.h>
    #include <stdlib.h>
    #include <sys/time.h>
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <arpa/inet.h>
    #include <netinet/in.h>
    #include <unistd.h>
    #include <fcntl.h>
    #include <netdb.h>

    #define ENET_SOCKETSET_EMPTY(sockset)          FD_ZERO(&(sockset))
    #define ENET_SOCKETSET_ADD(sockset, socket)    FD_SET((socket), &(sockset))
    #define ENET_SOCKETSET_REMOVE(sockset, socket) FD_CLR((socket), &(sockset))
    #define ENET_SOCKETSET_CHECK(sockset, socket)  FD_ISSET((socket), &(sockset))

    #define INVALID_SOCKET (-1)
    #define CLOSE_SOCKET(s) close((s))
    #define SOCK_ERRNO errno
    #define SOCK_EWOULDBLOCK EWOULDBLOCK
    #define ENET_SOCKET_NULL (-1)

    using ENetSocket = int;
    using ENetSocketSet = fd_set;

#endif


#ifdef MSG_MAXIOVLEN
#define ENET_BUFFER_MAXIMUM MSG_MAXIOVLEN
#endif

#define ENET_HOST_TO_NET_16(value) (htons(value)) // macro that converts host to net byte-order of a 16-bit value */
#define ENET_HOST_TO_NET_32(value) (htonl(value)) // macro that converts host to net byte-order of a 32-bit value */

#define ENET_NET_TO_HOST_16(value) (ntohs(value)) // macro that converts net to host byte-order of a 16-bit value */
#define ENET_NET_TO_HOST_32(value) (ntohl(value)) // macro that converts net to host byte-order of a 32-bit value */

#define ENET_ADDRESS_MAX_LENGTH 40 /*full IPv6 addresses take 39 characters + 1 null byte */

#define ENET_DIFFERENCE(x, y) ((x) < (y) ? (y) - (x) : (x) - (y))

#define ENET_HOST_ANY       0
#define ENET_HOST_BROADCAST 0xFFFFFFFFU
#define ENET_PORT_ANY       0

#define ENET_LIST_BEGIN (list) ((list) -> sentinel.next)
#define ENET_LIST_END(list) (& (list) -> sentinel)

#define ENET_LIST_EMPTY(list) (ENET_LIST_BEGIN(list) == ENET_LIST_END(list))

#define ENET_LIST_NEXT(iterator) ((iterator) -> next)
#define ENET_LIST_PREVIOUS(iterator) ((iterator) -> previous)

#define ENET_LIST_FRONT(list) ((void*) (list) -> sentinel.next)
#define ENET_LIST_BACK(list) ((void*) (list) -> sentinel.previous)

#define ENET_TIME_OVERFLOW 86400000

#define ENET_TIME_LESS(a, b) ((a) - (b) >= ENET_TIME_OVERFLOW)
#define ENET_TIME_GREATER(a, b) ((b) - (a) >= ENET_TIME_OVERFLOW)
#define ENET_TIME_LESS_EQUAL(a, b) (! ENET_TIME_GREATER (a, b))
#define ENET_TIME_GREATER_EQUAL(a, b) (! ENET_TIME_LESS (a, b))

#define ENET_TIME_DIFFERENCE(a, b) ((a) - (b) >= ENET_TIME_OVERFLOW ? (b) - (a) : (a) - (b))

namespace ENetConstants
{
    constexpr uint32_t MINIMUM_MTU             = 576;
    constexpr uint32_t MAXIMUM_MTU             = 4096;
    constexpr uint32_t MAXIMUM_PACKET_COMMANDS = 32;
    constexpr uint32_t MINIMUM_WINDOW_SIZE     = 4096;
    constexpr uint32_t MAXIMUM_WINDOW_SIZE     = 65536;
    constexpr uint32_t MINIMUM_CHANNEL_COUNT   = 1;
    constexpr uint32_t MAXIMUM_CHANNEL_COUNT   = 255;
    constexpr uint32_t MAXIMUM_PEER_ID         = 0xFFF;
    constexpr uint32_t MAXIMUM_FRAGMENT_COUNT  = 1024 * 1024;
}

namespace ENetHostState {
   constexpr uint32_t ENET_HOST_RECEIVE_BUFFER_SIZE          = 256 * 1024;
   constexpr uint32_t ENET_HOST_SEND_BUFFER_SIZE             = 256 * 1024;
   constexpr uint32_t ENET_HOST_BANDWIDTH_THROTTLE_INTERVAL  = 1000;
   constexpr uint32_t ENET_HOST_DEFAULT_MTU                  = 1392;
   constexpr uint32_t ENET_HOST_DEFAULT_MAXIMUM_PACKET_SIZE  = 32 * 1024 * 1024;
   constexpr uint32_t ENET_HOST_DEFAULT_MAXIMUM_WAITING_DATA = 32 * 1024 * 1024;
   constexpr uint32_t ENET_PEER_DEFAULT_ROUND_TRIP_TIME      = 500;
   constexpr uint32_t ENET_PEER_DEFAULT_PACKET_THROTTLE      = 32;
   constexpr uint32_t ENET_PEER_PACKET_THROTTLE_SCALE        = 32;
   constexpr uint32_t ENET_PEER_PACKET_THROTTLE_COUNTER      = 7; 
   constexpr uint32_t ENET_PEER_PACKET_THROTTLE_ACCELERATION = 2;
   constexpr uint32_t ENET_PEER_PACKET_THROTTLE_DECELERATION = 2;
   constexpr uint32_t ENET_PEER_PACKET_THROTTLE_INTERVAL     = 5000;
   constexpr uint32_t ENET_PEER_PACKET_LOSS_SCALE            = (1 << 16);
   constexpr uint32_t ENET_PEER_PACKET_LOSS_INTERVAL         = 10000;
   constexpr uint32_t ENET_PEER_WINDOW_SIZE_SCALE            = 64 * 1024;
   constexpr uint32_t ENET_PEER_TIMEOUT_LIMIT                = 32;
   constexpr uint32_t ENET_PEER_TIMEOUT_MINIMUM              = 5000;
   constexpr uint32_t ENET_PEER_TIMEOUT_MAXIMUM              = 30000;
   constexpr uint32_t ENET_PEER_PING_INTERVAL                = 500;
   constexpr uint32_t ENET_PEER_UNSEQUENCED_WINDOWS          = 64;
   constexpr uint32_t ENET_PEER_UNSEQUENCED_WINDOW_SIZE      = 1024;
   constexpr uint32_t ENET_PEER_FREE_UNSEQUENCED_WINDOWS     = 32;
   constexpr uint32_t ENET_PEER_RELIABLE_WINDOWS             = 16;
   constexpr uint32_t ENET_PEER_RELIABLE_WINDOW_SIZE         = 0x1000;
   constexpr uint32_t ENET_PEER_FREE_RELIABLE_WINDOWS        = 8;
}

#ifndef ENET_BUFFER_MAXIMUM
#define ENET_BUFFER_MAXIMUM (1 + 2 * ENetConstants::MAXIMUM_PACKET_COMMANDS)
#endif

struct ENetPacket;
struct ENetHost;
struct ENetEvent;

enum class ENetSocketType : uint8_t { 
   STREAM = 1, 
   DATAGRAM = 2 
};
enum class ENetSocketShutdown : uint8_t{ 
   READ = 0, 
   WRITE = 1, 
   READ_WRITE = 2 
};
enum class ENetAddressType : uint8_t { 
   ANY  = 0, 
   IPV4,
   IPV6 
};

enum class ENetSocketWait : uint8_t { 
   NONE = 0, 
   SEND = (1 << 1), 
   RECEIVE = (1 << 2), 
   INTERRUPT = (1 << 3) 
};

// enet_socket_wait() takes its condition as a plain uint32_t mask; these let it be tested and built from ENetSocketWait flags.
constexpr uint32_t operator&(uint32_t mask, ENetSocketWait flag) { return mask & static_cast<uint32_t>(flag); }
constexpr uint32_t operator|(ENetSocketWait a, ENetSocketWait b) { return static_cast<uint32_t>(a) | static_cast<uint32_t>(b); }
inline uint32_t& operator|=(uint32_t& mask, ENetSocketWait flag) { mask |= static_cast<uint32_t>(flag); return mask; }
// 	Implement complete bitmask operators



enum class ENetSocketOption : uint16_t {
   NET_OPT_NONBLOCK  = (1 << 0), 
   NET_OPT_BROADCAST = (1 << 1), 
   NET_OPT_RCVBUF    = (1 << 2), 
   NET_OPT_SNDBUF    = (1 << 3), 
   NET_OPT_REUSEADDR = (1 << 4), 
   NET_OPT_RCVTIMEO  = (1 << 5), 
   NET_OPT_SNDTIMEO  = (1 << 6), 
   NET_OPT_ERROR     = (1 << 7),  // Fully protected from macro collision
   NET_OPT_NODELAY   = (1 << 8),  // Fully protected from macro collision
   NET_OPT_TTL       = (1 << 9),  // Fully protected from macro collision
   NET_OPT_IPV6ONLY  = (1 << 10) 
};

enum class ENetProtocolCommand : uint8_t{
   NONE               = 0,
   ACKNOWLEDGE        = 1,
   CONNECT            = 2,
   VERIFY_CONNECT     = 3,
   DISCONNECT         = 4,
   PING               = 5,
   SEND_RELIABLE      = 6,
   SEND_UNRELIABLE    = 7,
   SEND_FRAGMENT      = 8,
   SEND_UNSEQUENCED   = 9,
   BANDWIDTH_LIMIT    = 10,
   THROTTLE_CONFIGURE = 11,
   SEND_UNRELIABLE_FRAGMENT = 12,
   COUNT              = 13,

   MASK               = 0x0F
};

enum class ENetProtocolFlag : uint16_t {
   FLAG_ACKNOWLEDGE = (1 << 7),
   FLAG_UNSEQUENCED = (1 << 6),

   HEADER_FLAG_COMPRESSED = (1 << 14),
   HEADER_FLAG_SENT_TIME  = (1 << 15),
   HEADER_FLAG_MASK       = HEADER_FLAG_COMPRESSED | HEADER_FLAG_SENT_TIME,

   HEADER_SESSION_MASK    = (3 << 12),
   HEADER_SESSION_SHIFT   = 12
};

/**
 * Packet flag bit constants.
 *
 * The host must be specified in network byte-order, and the port must be in
 * host byte-order. The constant ENET_HOST_ANY may be used to specify the
 * default server host.
 
   @sa ENetPacket
*/
enum class ENetPacketFlag : uint16_t {
   /** packet must be received by the target peer and resend attempts should be
     * made until the packet is delivered */
   RELIABLE    = (1 << 0),
   /** packet will not be sequenced with other packets
     */
   UNSEQUENCED = (1 << 1),
   /** packet will not allocate data, and user must supply it instead */
   NO_ALLOCATE = (1 << 2),
   /** packet will be fragmented using unreliable (instead of reliable) sends
     * if it exceeds the MTU */
   UNRELIABLE_FRAGMENT = (1 << 3),

   /** whether the packet has been sent from all queues it has been entered into */
   SENT = (1<<8)
};
enum class ENetPeerState : uint8_t {
   DISCONNECTED                = 0,
   CONNECTING                  = 1,
   ACKNOWLEDGING_CONNECT       = 2,
   CONNECTION_PENDING          = 3,
   CONNECTION_SUCCEEDED        = 4,
   CONNECTED                   = 5,
   DISCONNECT_LATER            = 6,
   DISCONNECTING               = 7,
   ACKNOWLEDGING_DISCONNECT    = 8,
   ZOMBIE                      = 9 
};

enum class ENetPeerFlag : uint8_t {
   ENET_PEER_FLAG_NEEDS_DISPATCH   = (1 << 0),
   ENET_PEER_FLAG_CONTINUE_SENDING = (1 << 1)
};

/**
 * An ENet event type, as specified in @ref ENetEvent.
 */
enum class ENetEventType : uint8_t {
   /** no event occurred within the specified time limit */
   ENET_EVENT_TYPE_NONE       = 0,  

   /** a connection request initiated by enet_host_connect has completed.  
     * The peer field contains the peer which successfully connected. 
     */
   ENET_EVENT_TYPE_CONNECT    = 1,  

   /** a peer has disconnected.  This event is generated on a successful 
     * completion of a disconnect initiated by enet_peer_disconnect, if 
     * a peer has timed out, or if a connection request intialized by 
     * enet_host_connect has timed out.  The peer field contains the peer 
     * which disconnected. The data field contains user supplied data 
     * describing the disconnection, or 0, if none is available.
     */
   ENET_EVENT_TYPE_DISCONNECT = 2,  

   /** a packet has been received from a peer.  The peer field specifies the
     * peer which sent the packet.  The channelID field specifies the channel
     * number upon which the packet was received.  The packet field contains
     * the packet that was received; this packet must be destroyed with
     * enet_packet_destroy after use.
     */
   ENET_EVENT_TYPE_RECEIVE    = 3
};

/**
 * Portable internet address structure. 
 *
 * The host must be specified in network byte-order, and the port must be in host 
 * byte-order. The constant ENET_HOST_ANY may be used to specify the default 
 * server host. The constant ENET_HOST_BROADCAST may be used to specify the
 * broadcast address (255.255.255.255).  This makes sense for enet_host_connect,
 * but not for enet_host_create.  Once a server responds to a broadcast, the
 * address is updated from ENET_HOST_BROADCAST to the server's actual IP address.
 */
struct ENetAddress {  ENetAddressType type; uint16_t port; union { uint8_t v4[4]; uint16_t v6[8]; } host; };

using ENetPacketAcknowledgedCallback = void (MOSS_CALL*)(ENetPacket* packet);
using ENetPacketFreeCallback = void (MOSS_CALL*)(ENetPacket* packet);

/**
 * ENet packet structure.
 *
 * An ENet data packet that may be sent to or received from a peer. The shown 
 * fields should only be read and never modified. The data field contains the 
 * allocated data for the packet. The dataLength fields specifies the length 
 * of the allocated data.  The flags field is either 0 (specifying no flags), 
 * or a bitwise-or of any combination of the following flags:
 *
 *    RELIABLE - packet must be received by the target peer
 *    and resend attempts should be made until the packet is delivered
 *
 *    UNSEQUENCED - packet will not be sequenced with other packets 
 *    (not supported for reliable packets)
 *
 *    NO_ALLOCATE - packet will not allocate data, and user must supply it instead
 *
 *    UNRELIABLE_FRAGMENT - packet will be fragmented using unreliable
 *    (instead of reliable) sends if it exceeds the MTU
 *
 *    SENT - whether the packet has been sent from all queues it has been entered into
   @sa ENetPacketFlag
 */
struct ENetPacket {
   size_t                referenceCount;  // internal use only */
   uint32_t              flags;           // bitwise-or of ENetPacketFlag constants */
   uint8_t *             data;            // allocated data for packet */
   size_t                dataLength;      // length of data */

   ENetPacketFreeCallback   freeCallback;    // function to be called when the packet is no longer in use */
   void *                   userData;        // application private data, may be freely modified */
};

struct ENetListNode {
   ENetListNode* next;
   ENetListNode* previous;
};

using ENetListIterator = ENetListNode*;

struct ENetList { 
   ENetListNode sentinel; 
};


struct ENetProtocolHeader { uint16_t peerID; uint16_t sentTime; };
struct ENetProtocolCommandHeader { uint8_t command; uint8_t channelID; uint16_t reliableSequenceNumber; };
struct ENetProtocolAcknowledge { ENetProtocolCommandHeader header; uint16_t receivedReliableSequenceNumber; uint16_t receivedSentTime; };
struct ENetProtocolConnect {
   ENetProtocolCommandHeader header;
   uint16_t outgoingPeerID;
   uint8_t  incomingSessionID;
   uint8_t  outgoingSessionID;
   uint32_t mtu;
   uint32_t windowSize;
   uint32_t channelCount;
   uint32_t incomingBandwidth;
   uint32_t outgoingBandwidth;
   uint32_t packetThrottleInterval;
   uint32_t packetThrottleAcceleration;
   uint32_t packetThrottleDeceleration;
   uint32_t connectID;
   uint32_t data;
};

struct ENetProtocolVerifyConnect {
   ENetProtocolCommandHeader header;
   uint16_t outgoingPeerID;
   uint8_t  incomingSessionID;
   uint8_t  outgoingSessionID;
   uint32_t mtu;
   uint32_t windowSize;
   uint32_t channelCount;
   uint32_t incomingBandwidth;
   uint32_t outgoingBandwidth;
   uint32_t packetThrottleInterval;
   uint32_t packetThrottleAcceleration;
   uint32_t packetThrottleDeceleration;
   uint32_t connectID;
};

struct ENetProtocolBandwidthLimit { ENetProtocolCommandHeader header; uint32_t incomingBandwidth; uint32_t outgoingBandwidth; };
struct ENetProtocolThrottleConfigure { ENetProtocolCommandHeader header; uint32_t packetThrottleInterval; uint32_t packetThrottleAcceleration; uint32_t packetThrottleDeceleration; };
struct ENetProtocolDisconnect { ENetProtocolCommandHeader header; uint32_t data; };
struct ENetProtocolPing { ENetProtocolCommandHeader header; };
struct ENetProtocolSendReliable { ENetProtocolCommandHeader header; uint16_t dataLength; };
struct ENetProtocolSendUnreliable { ENetProtocolCommandHeader header; uint16_t unreliableSequenceNumber; uint16_t dataLength; };
struct ENetProtocolSendUnsequenced{ ENetProtocolCommandHeader header; uint16_t unsequencedGroup; uint16_t dataLength; };
struct ENetProtocolSendFragment { ENetProtocolCommandHeader header; uint16_t startSequenceNumber; uint16_t dataLength;  uint32_t fragmentCount; uint32_t fragmentNumber; uint32_t totalLength; uint32_t fragmentOffset; };


union ENetProtocol {
   ENetProtocolCommandHeader header;         ENetProtocolAcknowledge acknowledge;         ENetProtocolConnect connect;
   ENetProtocolVerifyConnect verifyConnect;  ENetProtocolDisconnect disconnect;           ENetProtocolPing ping;
   ENetProtocolSendReliable sendReliable;    ENetProtocolSendUnreliable sendUnreliable;   ENetProtocolSendUnsequenced sendUnsequenced;
   ENetProtocolSendFragment sendFragment;    ENetProtocolBandwidthLimit bandwidthLimit;   ENetProtocolThrottleConfigure throttleConfigure;
};

struct ENetAcknowledgement { ENetListNode acknowledgementList; uint32_t sentTime; ENetProtocol command; };

struct ENetOutgoingCommand {
   ENetListNode outgoingCommandList;
   uint16_t  reliableSequenceNumber;
   uint16_t  unreliableSequenceNumber;
   uint32_t  sentTime;
   uint32_t  roundTripTimeout;
   uint32_t  queueTime;
   uint32_t  fragmentOffset;
   uint16_t  fragmentLength;
   uint16_t  sendAttempts;
   ENetProtocol command;
   ENetPacket * packet;
};

struct ENetIncomingCommand {
   ENetListNode     incomingCommandList;
   uint16_t      reliableSequenceNumber;
   uint16_t      unreliableSequenceNumber;
   ENetProtocol     command;
   uint32_t      fragmentCount;
   uint32_t      fragmentsRemaining;
   uint32_t *    fragments;
   ENetPacket *     packet;
};

struct ENetChannel {
   uint16_t  outgoingReliableSequenceNumber;
   uint16_t  outgoingUnreliableSequenceNumber;
   uint16_t  usedReliableWindows;
   uint16_t  reliableWindows [ENetHostState::ENET_PEER_RELIABLE_WINDOWS];
   uint16_t  incomingReliableSequenceNumber;
   uint16_t  incomingUnreliableSequenceNumber;
   ENetList     incomingReliableCommands;
   ENetList     incomingUnreliableCommands;
};

/**
 * An ENet peer which data packets may be sent or received from. 
 *
 * No fields should be modified unless otherwise specified. 
 */
struct ENetPeer { 
   ENetListNode  dispatchList;
   struct ENetHost* host;
   uint16_t   outgoingPeerID;
   uint16_t   incomingPeerID;
   uint32_t   connectID;
   uint8_t    outgoingSessionID;
   uint8_t    incomingSessionID;
   ENetAddress    address;             // Internet address of the peer */
   void*          data;                // Application private data, may be freely modified */
   ENetPeerState  state;
   ENetChannel*   channels;
   size_t         channelCount;        // Number of channels allocated for communication with peer */
   uint32_t   incomingBandwidth;         // Downstream bandwidth of the client in bytes/second */
   uint32_t   outgoingBandwidth;         // Upstream bandwidth of the client in bytes/second */
   uint32_t   incomingBandwidthThrottleEpoch;
   uint32_t   outgoingBandwidthThrottleEpoch;
   uint32_t   incomingDataTotal;
   uint32_t   outgoingDataTotal;
   uint32_t   lastSendTime;
   uint32_t   lastReceiveTime;
   uint32_t   nextTimeout;
   uint32_t   earliestTimeout;
   uint32_t   packetLossEpoch;
   uint32_t   packetsSent;
   uint32_t   packetsLost;
   uint32_t   packetLoss;          // mean packet loss of reliable packets as a ratio with respect to the constant ENET_PEER_PACKET_LOSS_SCALE */
   uint32_t   packetLossVariance;
   uint32_t   packetThrottle;
   uint32_t   packetThrottleLimit;
   uint32_t   packetThrottleCounter;
   uint32_t   packetThrottleEpoch;
   uint32_t   packetThrottleAcceleration;
   uint32_t   packetThrottleDeceleration;
   uint32_t   packetThrottleInterval;
   uint32_t   pingInterval;
   uint32_t   timeoutLimit;
   uint32_t   timeoutMinimum;
   uint32_t   timeoutMaximum;
   uint32_t   lastRoundTripTime;
   uint32_t   lowestRoundTripTime;
   uint32_t   lastRoundTripTimeVariance;
   uint32_t   highestRoundTripTimeVariance;
   uint32_t   roundTripTime;            // mean round trip time (RTT), in milliseconds, between sending a reliable packet and receiving its acknowledgement */
   uint32_t   roundTripTimeVariance;
   uint32_t   mtu;
   uint32_t   windowSize;
   uint32_t   reliableDataInTransit;
   uint16_t   outgoingReliableSequenceNumber;
   ENetList acknowledgements;
   ENetList sentReliableCommands;
   ENetList outgoingSendReliableCommands;
   ENetList outgoingCommands;
   ENetList dispatchedCommands;
   uint16_t   flags;
   uint16_t   reserved;
   uint16_t   incomingUnsequencedGroup;
   uint16_t   outgoingUnsequencedGroup;
   uint32_t   unsequencedWindow [ENetHostState::ENET_PEER_UNSEQUENCED_WINDOW_SIZE / 32]; 
   uint32_t   eventData;
   size_t        totalWaitingData;


   /*
   ENetTransportType transport;
   uint32_t transport_caps;
   void* transport_data;
   */
};

struct ENetBuffer { void* data; size_t dataLength; };

/** Callback that computes the checksum of the data held in buffers[0:bufferCount-1] */
using ENetChecksumCallback = uint32_t (MOSS_CALL*)(const ENetBuffer* buffers, size_t bufferCount);

/** Callback for intercepting received raw UDP packets. Should return 1 to intercept, 0 to ignore, or -1 to propagate an error. */
using ENetInterceptCallback = int (MOSS_CALL*)(struct ENetHost* host, struct ENetEvent* event);


/** An ENet packet compressor for compressing UDP packets before socket sends or receives.
 */
struct ENetCompressor {
   /** Context data for the compressor. Must be non-NULL. */
   void* context;
   /** Compresses from inBuffers[0:inBufferCount-1], containing inLimit bytes, to outData, outputting at most outLimit bytes. Should return 0 on failure. */
   size_t (MOSS_CALL* compress) (void* context, const ENetBuffer* inBuffers, size_t inBufferCount, size_t inLimit, uint8_t* outData, size_t outLimit);
   /** Decompresses from inData, containing inLimit bytes, to outData, outputting at most outLimit bytes. Should return 0 on failure. */
   size_t (MOSS_CALL* decompress) (void* context, const uint8_t* inData, size_t inLimit, uint8_t * outData, size_t outLimit);
   /** Destroys the context when compression is disabled or the host is destroyed. May be NULL. */
   void (MOSS_CALL* destroy) (void* context);
};
 
/** An ENet host for communicating with peers.
  *
  * No fields should be modified unless otherwise stated.

    @sa enet_host_create()
    @sa enet_host_destroy()
    @sa enet_host_connect()
    @sa enet_host_service()
    @sa enet_host_flush()
    @sa enet_host_broadcast()
    @sa enet_host_compress()
    @sa enet_host_compress_with_range_coder()
    @sa enet_host_channel_limit()
    @sa enet_host_bandwidth_limit()
    @sa enet_host_bandwidth_throttle()
  */
struct ENetHost {
   ENetSocket           socket;
   ENetAddress          address;                     // Internet address of the host */
   uint32_t          incomingBandwidth;           // downstream bandwidth of the host */
   uint32_t          outgoingBandwidth;           // upstream bandwidth of the host */
   uint32_t          bandwidthThrottleEpoch;
   uint32_t          mtu;
   uint32_t          randomSeed;
   int                  recalculateBandwidthLimits;
   ENetPeer*           peers;                       // array of peers allocated for this host */
   size_t               peerCount;                   // number of peers allocated for this host */
   size_t               channelLimit;                // maximum number of channels allowed for connected peers */
   uint32_t          serviceTime;
   ENetList             dispatchQueue;
   uint32_t          totalQueued;
   size_t               packetSize;
   uint16_t          headerFlags;
   ENetProtocol         commands [ENetConstants::MAXIMUM_PACKET_COMMANDS];
   size_t               commandCount;
   ENetBuffer           buffers [ENET_BUFFER_MAXIMUM];
   size_t               bufferCount;
   ENetChecksumCallback checksum;                    // callback the user can set to enable packet checksums for this host */
   ENetCompressor       compressor;
   uint8_t           packetData [2][ENetConstants::MAXIMUM_MTU];
   ENetAddress          receivedAddress;
   uint8_t*         receivedData;
   size_t               receivedDataLength;
   uint32_t          totalSentData;               // total data sent, user should reset to 0 as needed to prevent overflow */
   uint32_t          totalSentPackets;            // total UDP packets sent, user should reset to 0 as needed to prevent overflow */
   uint32_t          totalReceivedData;           // total data received, user should reset to 0 as needed to prevent overflow */
   uint32_t          totalReceivedPackets;        // total UDP packets received, user should reset to 0 as needed to prevent overflow */
   ENetInterceptCallback intercept;               // callback the user can set to intercept received raw UDP packets */
   size_t               connectedPeers;
   size_t               bandwidthLimitedPeers;
   size_t               duplicatePeers;            // optional number of allowed peers from duplicate IPs, defaults to MAXIMUM_PEER_ID */
   size_t               maximumPacketSize;         // the maximum allowable packet size that may be sent or received on a peer */
   size_t               maximumWaitingData;        // the maximum aggregate amount of buffer space a peer may use waiting for packets to be delivered */
};

/**
 * An ENet event as returned by enet_host_service().
   
   @sa enet_host_service
 */
struct ENetEvent {
   ENetEventType        type;      // type of the event
   ENetPeer *           peer;      // peer that generated a connect, disconnect or receive event
   uint8_t              channelID; // channel on the peer that generated the event, if appropriate */
   uint32_t             data;      // data associated with the event, if appropriate */
   ENetPacket *         packet;    // packet associated with the event, if appropriate */
};

// Call back
struct ENetCallbacks {
    void* (MOSS_CALL* malloc) (size_t size);
    void (MOSS_CALL* free) (void * memory);
    void (MOSS_CALL* no_memory) (void);
};

/*! @brief Initializes ENet globally. Must be called prior to using any functions in ENet. */
MOSS_API int Moss_Init_Network(void);
/*! @brief Initializes ENet globally with callbacks. Must be called prior to using any functions in ENet. */
MOSS_API int enet_initialize_with_callbacks(const ENetCallbacks * inits);
/*! @brief Non-blocking receive for STREAM (TCP) sockets that tells would-block apart from end-of-stream.
    @returns > 0 bytes received; 0 = no data right now (would block / interrupted); -1 = peer closed the connection (end of stream); -2 = error.
    enet_socket_receive() cannot make this distinction (it returns 0 for both), which makes a truncated transfer look finished. */
MOSS_API int enet_socket_receive_stream(ENetSocket socket, void* data, size_t length);
/*! @brief Non-blocking send for STREAM (TCP) sockets, never raises SIGPIPE. @returns > 0 bytes sent; 0 = would block; < 0 = error. */
MOSS_API int enet_socket_send_stream(ENetSocket socket, const void* data, size_t length);

/*! @brief Allocates through the callbacks set with enet_initialize_with_callbacks (malloc by default). Calls no_memory() on failure. */
MOSS_API void* enet_malloc(size_t size);
/*! @brief Frees memory obtained from enet_malloc. */
MOSS_API void  enet_free(void* memory);
/*! @brief Shuts down ENet globally. Should be called when a program that has initialized ENet exits. */
MOSS_API void Moss_TerminateNetwork(void);

// =============================================
// List
// =============================================
MOSS_API void enet_list_clear(ENetList*);

MOSS_API ENetListIterator enet_list_insert(ENetListIterator, void*);
MOSS_API void * enet_list_remove(ENetListIterator);
MOSS_API ENetListIterator enet_list_move(ENetListIterator, void*, void*);

MOSS_API size_t enet_list_size(ENetList*);


// =============================================
// Time
// =============================================
/*! @brief Returns the wall-time in milliseconds. Its initial value is unspecified unless otherwise set. */
MOSS_API uint32_t enet_time_get(void);
/*! @brief Sets the current wall-time in milliseconds. */
MOSS_API void enet_time_set(uint32_t);


// =============================================
// Socket
// =============================================
/*! @brief X. @param type X. @param socket_type X.*/
MOSS_API ENetSocket enet_socket_create(ENetAddressType adress_type, ENetSocketType socket_type);
/*! @brief X.*/
MOSS_API int        enet_socket_bind(ENetSocket, const ENetAddress*);
/*! @brief X.*/
MOSS_API int        enet_socket_get_address(ENetSocket, ENetAddress*);
/*! @brief X.*/
MOSS_API int        enet_socket_listen(ENetSocket, int);
/*! @brief X.*/
MOSS_API ENetSocket enet_socket_accept(ENetSocket, ENetAddress *);
/*! @brief X.*/
MOSS_API int        enet_socket_connect(ENetSocket, const ENetAddress *);
/*! @brief X.*/
MOSS_API int        enet_socket_send(ENetSocket, const ENetAddress *, const ENetBuffer *, size_t);
/*! @brief X.*/
MOSS_API int        enet_socket_receive(ENetSocket, ENetAddress *, ENetBuffer *, size_t);
/*! @brief X.*/
MOSS_API int        enet_socket_wait(ENetSocket, uint32_t*, uint32_t);
/*! @brief X.*/
MOSS_API int        enet_socket_set_option(ENetSocket, ENetSocketOption, int);
/*! @brief X.*/
MOSS_API int        enet_socket_get_option(ENetSocket, ENetSocketOption, int *);
/*! @brief X.*/
MOSS_API int        enet_socket_shutdown(ENetSocket, ENetSocketShutdown);
/*! @brief X.*/
MOSS_API void       enet_socket_destroy(ENetSocket);
/*! @brief X.*/
MOSS_API int        enet_socketset_select(ENetSocket, ENetSocketSet *, ENetSocketSet *, uint32_t);

// =============================================
// Addresses
// =============================================
/** Attempts to parse the printable form of the IP address in the parameter hostName and sets the host field in the address parameter if successful. @param address destination to store the parsed IP address @param hostName IP address to parse @returns the address of the given hostName in address on success.*/
MOSS_API int enet_address_set_host_ip(ENetAddress* address, const char* hostName);
/** Attempts to resolve the host named by the parameter hostName and sets the host field in the address parameter if successful. @param address destination to store resolved address. @param type set the ip adress type (ENET_ADDRESS_TYPE_ANY/ENET_ADDRESS_TYPE_IPV4/ENET_ADDRESS_TYPE_IPV6). @param hostName host name to lookup. @returns the address of the given hostName in address on success.*/
MOSS_API int enet_address_set_host(ENetAddress* address, ENetAddressType type, const char* hostName);
/*! Gives the printable form of the IP address specified in the address parameter. @param address    address printed @param hostName   destination for name, must not be NULL @param nameLength maximum length of hostName. @returns the null-terminated name of the host in hostName on success.*/
MOSS_API int enet_address_get_host_ip(const ENetAddress* address, char* hostName, size_t nameLength);
/*! @brief Attempts to do a reverse lookup of the host field in the address parameter.  @param address address used for reverse lookup. @param hostName destination for name, must not be NULL. @param nameLength maximum length of hostName.  @returns the null-terminated name of the host in hostName on success.*/
MOSS_API int enet_address_get_host(const ENetAddress* address, char* hostName, size_t nameLength);
/*! @brief X.*/
MOSS_API int enet_address_equal_host(const ENetAddress* firstAddress, const ENetAddress * secondAddress);
/*! @brief X.*/
MOSS_API int enet_address_is_broadcast(const ENetAddress* address);
/*! @brief X.*/
MOSS_API void enet_address_build_any(ENetAddress* address, ENetAddressType type);
/*! @brief X.*/
MOSS_API void enet_address_build_loopback(ENetAddress* address, ENetAddressType type);
/*! @brief X.*/
MOSS_API void enet_address_convert_ipv6(ENetAddress* address);

// =============================================
// Host
// =============================================
/*! @brief X.*/
MOSS_API uint32_t      enet_crc32(const ENetBuffer*, size_t);

/*! @brief X. @param type*/
MOSS_API ENetHost*   enet_host_create(ENetAddressType type, const ENetAddress*, size_t, size_t, uint32_t, uint32_t);
/*! @brief X.*/
MOSS_API void        enet_host_destroy(ENetHost*);
/*! @brief X.*/
MOSS_API int         enet_host_check_events(ENetHost*, ENetEvent*);
/*! @brief X.*/
MOSS_API int         enet_host_service(ENetHost*, ENetEvent*, uint32_t);
/*! @brief X.*/
MOSS_API void        enet_host_flush(ENetHost*);
/*! @brief X.*/
MOSS_API void        enet_host_broadcast(ENetHost*, uint8_t, ENetPacket *);
/*! @brief X.*/
MOSS_API void        enet_host_compress(ENetHost*, const ENetCompressor*);
/*! @brief X.*/
MOSS_API int         enet_host_compress_with_range_coder(ENetHost * host);
/*! @brief X.*/
MOSS_API void        enet_host_channel_limit(ENetHost*, size_t);
/*! @brief X.*/
MOSS_API void        enet_host_bandwidth_limit(ENetHost*, uint32_t, uint32_t);
/*! @brief X.*/
MOSS_API void        enet_host_bandwidth_throttle(ENetHost*);
/*! @brief X.*/
MOSS_API uint32_t      enet_host_random_seed(void);
/*! @brief X.*/
MOSS_API uint32_t      enet_host_random(ENetHost*);


// =============================================
// Peer
// =============================================
/*! @brief X.*/
MOSS_API ENetPeer*   enet_host_connect(ENetHost*, const ENetAddress*, size_t, uint32_t);
/*! @brief X.*/
MOSS_API int                  enet_peer_send(ENetPeer*, uint8_t, ENetPacket*);
/*! @brief X.*/
MOSS_API void                 enet_peer_ping(ENetPeer*);
/*! @brief X.*/
MOSS_API void                 enet_peer_ping_interval(ENetPeer*, uint32_t);
/*! @brief X.*/
MOSS_API void                 enet_peer_timeout(ENetPeer*, uint32_t, uint32_t, uint32_t);
/*! @brief X.*/
MOSS_API void                 enet_peer_reset(ENetPeer*);
/*! @brief X.*/
MOSS_API void                 enet_peer_disconnect(ENetPeer*, uint32_t);
/*! @brief X.*/
MOSS_API void                 enet_peer_disconnect_now(ENetPeer*, uint32_t);
/*! @brief X.*/
MOSS_API void                 enet_peer_disconnect_later(ENetPeer*, uint32_t);
/*! @brief X.*/
MOSS_API void                 enet_peer_throttle_configure(ENetPeer*, uint32_t, uint32_t, uint32_t);
/*! @brief X.*/
MOSS_API int                  enet_peer_throttle(ENetPeer *, uint32_t);
/*! @brief X.*/
MOSS_API void                 enet_peer_reset_queues(ENetPeer *);
/*! @brief X.*/
MOSS_API int                  enet_peer_has_outgoing_commands(ENetPeer*);
/*! @brief X.*/
MOSS_API void                 enet_peer_setup_outgoing_command(ENetPeer*, ENetOutgoingCommand*);
/*! @brief X.*/
MOSS_API ENetOutgoingCommand* enet_peer_queue_outgoing_command(ENetPeer*, const ENetProtocol*, ENetPacket*, uint32_t, uint16_t);
/*! @brief X.*/
MOSS_API ENetIncomingCommand* enet_peer_queue_incoming_command(ENetPeer*, const ENetProtocol*, const void*, size_t, uint32_t, uint32_t);
/*! @brief X.*/
MOSS_API ENetAcknowledgement* enet_peer_queue_acknowledgement(ENetPeer*, const ENetProtocol*, uint16_t);
/*! @brief X.*/
MOSS_API void                 enet_peer_dispatch_incoming_unreliable_commands(ENetPeer*, ENetChannel*, ENetIncomingCommand*);
/*! @brief X.*/
MOSS_API void                 enet_peer_dispatch_incoming_reliable_commands(ENetPeer*, ENetChannel*, ENetIncomingCommand*);
/*! @brief X.*/
MOSS_API void                 enet_peer_on_connect(ENetPeer*);
/*! @brief X.*/
MOSS_API void                 enet_peer_on_disconnect(ENetPeer*);


// =============================================
// Packet
// =============================================
/*! @brief X.*/
MOSS_API ENetPacket* enet_packet_create(const void*, size_t, uint32_t);
/*! @brief X.*/
MOSS_API void        enet_packet_destroy(ENetPacket*);
/*! @brief X.*/
MOSS_API int         enet_packet_resize(ENetPacket*, size_t);
/*! @brief X.*/
MOSS_API ENetPacket* enet_peer_receive(ENetPeer*, uint8_t* channelID);

// =============================================
// Range Coder
// =============================================

MOSS_API void*  enet_range_coder_create(void);
MOSS_API void   enet_range_coder_destroy(void*);
MOSS_API size_t enet_range_coder_compress(void*, const ENetBuffer*, size_t, size_t, uint8_t*, size_t);
MOSS_API size_t enet_range_coder_decompress(void*, const uint8_t*, size_t, uint8_t*, size_t);
MOSS_API size_t enet_protocol_command_size(uint8_t);



// Unknown functions down here
/*! @brief X.*/
MOSS_API void* enet_packet_get_data(const ENetPacket*);
/*! @brief X.*/
MOSS_API void* enet_packet_get_user_data(const ENetPacket*);
/*! @brief X.*/
MOSS_API void enet_packet_set_user_data(ENetPacket*, void* userData);
/*! @brief X.*/
MOSS_API int enet_packet_get_length(const ENetPacket*);
/*! @brief X.*/
MOSS_API void enet_packet_set_acknowledge_callback(ENetPacket*, ENetPacketAcknowledgedCallback);
/*! @brief X.*/
MOSS_API void enet_packet_set_free_callback(ENetPacket*, ENetPacketFreeCallback);
/*! @brief X.*/
MOSS_API int enet_packet_check_references(const ENetPacket*);
/*! @brief X.*/
MOSS_API void enet_packet_dispose(ENetPacket*);

/*! @brief X.*/
MOSS_API uint32_t enet_host_get_peers_count(const ENetHost*);
/*! @brief X.*/
MOSS_API uint32_t enet_host_get_packets_sent(const ENetHost*);
/*! @brief X.*/
MOSS_API uint32_t enet_host_get_packets_received(const ENetHost*);
/*! @brief X.*/
MOSS_API uint32_t enet_host_get_bytes_sent(const ENetHost*);
/*! @brief X.*/
MOSS_API uint32_t enet_host_get_bytes_received(const ENetHost*);
/*! @brief X.*/
MOSS_API void enet_host_set_max_duplicate_peers(ENetHost*, uint16_t);
/*! @brief X.*/
MOSS_API void enet_host_set_intercept_callback(ENetHost*, ENetInterceptCallback);
/*! @brief X.*/
MOSS_API void enet_host_set_checksum_callback(ENetHost*, ENetChecksumCallback);

/*! @brief X.*/
MOSS_API uint32_t enet_peer_get_id(const ENetPeer*);
/*! @brief X.*/
MOSS_API int enet_peer_get_ip(const ENetPeer*, char*, size_t);
/*! @brief X.*/
MOSS_API uint16_t enet_peer_get_port(const ENetPeer*);
/*! @brief X.*/
MOSS_API uint32_t enet_peer_get_mtu(const ENetPeer*);
/*! @brief X.*/
MOSS_API ENetPeerState enet_peer_get_state(const ENetPeer*);
/*! @brief X.*/
MOSS_API uint32_t enet_peer_get_rtt(const ENetPeer*);
/*! @brief X.*/
MOSS_API uint32_t enet_peer_get_last_rtt(const ENetPeer* peer);
/*! @brief X.*/
MOSS_API uint32_t enet_peer_get_lastsendtime(const ENetPeer*);
/*! @brief X.*/
MOSS_API uint32_t enet_peer_get_lastreceivetime(const ENetPeer*);
/*! @brief X.*/
MOSS_API float enet_peer_get_packets_throttle(const ENetPeer*);
/*! @brief X.*/
MOSS_API void* enet_peer_get_data(const ENetPeer*);
/*! @brief X.*/
MOSS_API void enet_peer_set_data(ENetPeer*, const void*);

// =============================================
// WebSocket (RFC 6455 framing)
// =============================================
// Event-driven WebSocket frame/message layer, merged into Moss_Network from the
// Wslay library (c) 2011, 2012 Tatsuhiro Tsujikawa, MIT licensed - see websocket.cpp
// for the full licence text.
//
// This layer handles framing, fragmentation, masking, ping/pong, close handshakes
// and UTF-8 validation. It does NOT perform the HTTP Upgrade handshake or TLS:
// do those first on your TCP socket, then hand the connected socket to a context
// created below. All I/O goes through the callbacks you supply and is expected to
// be non-blocking.
//
// Typical use:
//   1. enet_ws_event_context_client_init() / _server_init() with your callbacks.
//   2. When the socket is readable   -> enet_ws_event_recv(ctx).
//   3. enet_ws_event_queue_msg(ctx, ...) to queue outgoing text/binary messages.
//   4. When the socket is writable or enet_ws_event_want_write() is 1 -> enet_ws_event_send(ctx).
//   5. enet_ws_event_context_free(ctx) once the connection is closed.

/*! @brief Signed size type used by the WebSocket I/O callbacks (portable replacement for POSIX ssize_t). */
using ENetSSize = std::ptrdiff_t;

/* Reserved bits: RSV1, RSV2, RSV3. rsv is built as ((RSV1 << 2) | (RSV2 << 1) | RSV3). */
#define ENET_WS_RSV_NONE ((uint8_t) 0)
#define ENET_WS_RSV1_BIT (((uint8_t) 1) << 2)
#define ENET_WS_RSV2_BIT (((uint8_t) 1) << 1)
#define ENET_WS_RSV3_BIT (((uint8_t) 1) << 0)

#define ENET_WS_GET_RSV1(rsv) (((rsv) >> 2) & 1)
#define ENET_WS_GET_RSV2(rsv) (((rsv) >> 1) & 1)
#define ENET_WS_GET_RSV3(rsv) ((rsv) & 1)

/* Returns 1 if opcode is a control frame opcode, otherwise 0. */
#define ENET_WS_IS_CTRL_FRAME(opcode) (((opcode) >> 3) & 1)

enum ENetWSError {
   ENET_WS_ERR_WANT_READ        = -100,
   ENET_WS_ERR_WANT_WRITE       = -101,
   ENET_WS_ERR_PROTO            = -200,
   ENET_WS_ERR_INVALID_ARGUMENT = -300,
   ENET_WS_ERR_INVALID_CALLBACK = -301,
   ENET_WS_ERR_NO_MORE_MSG      = -302,
   ENET_WS_ERR_CALLBACK_FAILURE = -400,
   ENET_WS_ERR_WOULDBLOCK       = -401,
   ENET_WS_ERR_NOMEM            = -500
};

/* Close status codes defined in RFC 6455. */
enum ENetWSStatusCode {
   ENET_WS_CODE_NORMAL_CLOSURE           = 1000,
   ENET_WS_CODE_GOING_AWAY               = 1001,
   ENET_WS_CODE_PROTOCOL_ERROR           = 1002,
   ENET_WS_CODE_UNSUPPORTED_DATA         = 1003,
   ENET_WS_CODE_NO_STATUS_RCVD           = 1005,
   ENET_WS_CODE_ABNORMAL_CLOSURE         = 1006,
   ENET_WS_CODE_INVALID_FRAME_PAYLOAD_DATA = 1007,
   ENET_WS_CODE_POLICY_VIOLATION         = 1008,
   ENET_WS_CODE_MESSAGE_TOO_BIG          = 1009,
   ENET_WS_CODE_MANDATORY_EXT            = 1010,
   ENET_WS_CODE_INTERNAL_SERVER_ERROR    = 1011,
   ENET_WS_CODE_TLS_HANDSHAKE            = 1015
};

enum ENetWSIOFlags {
   /* There is more data to send. */
   ENET_WS_MSG_MORE = 1
};

/* Opcodes defined in RFC 6455. */
enum ENetWSOpcode {
   ENET_WS_CONTINUATION_FRAME = 0x0u,
   ENET_WS_TEXT_FRAME         = 0x1u,
   ENET_WS_BINARY_FRAME       = 0x2u,
   ENET_WS_CONNECTION_CLOSE   = 0x8u,
   ENET_WS_PING               = 0x9u,
   ENET_WS_PONG               = 0xau
};

/* Opaque contexts. */
struct enet_ws_frame_context;
struct enet_ws_event_context;
using enet_ws_frame_context_ptr = enet_ws_frame_context*;
using enet_ws_event_context_ptr = enet_ws_event_context*;

/*! @brief Frame-level I/O description used by enet_ws_frame_send() / enet_ws_frame_recv(). */
struct enet_ws_frame_iocb {
   uint8_t  fin;             /* 1 for the final frame of a message, otherwise 0 */
   uint8_t  rsv;             /* reserved bits, ((RSV1 << 2) | (RSV2 << 1) | RSV3); must be 0 unless an extension is negotiated */
   uint8_t  opcode;          /* 4 bit opcode */
   uint64_t payload_length;  /* payload length [0, 2**63-1] */
   uint8_t  mask;            /* 1 for a masked frame, otherwise 0 */
   const uint8_t* data;      /* part of the payload data */
   size_t   data_length;     /* number of bytes in data */
};

struct enet_ws_event_on_msg_recv_arg {
   uint8_t  rsv;             /* reserved bits: (RSV1 << 2) | (RSV2 << 1) | RSV3 */
   uint8_t  opcode;
   const uint8_t* msg;       /* received message */
   size_t   msg_length;
   uint16_t status_code;     /* only for ENET_WS_CONNECTION_CLOSE; 0 if the close frame carried no status code */
};

struct enet_ws_event_on_frame_recv_start_arg {
   uint8_t  fin;
   uint8_t  rsv;
   uint8_t  opcode;
   uint64_t payload_length;
};

struct enet_ws_event_on_frame_recv_chunk_arg {
   const uint8_t* data;
   size_t   data_length;
};

union enet_ws_event_msg_source {
   int   fd;
   void* data;
};

struct enet_ws_event_msg {
   uint8_t        opcode;
   const uint8_t* msg;
   size_t         msg_length;
};

/* Frame-level callbacks. */
/*! @brief Send at most len bytes. Return bytes sent, or -1 on error (0 is treated as an error). flags may contain ENET_WS_MSG_MORE. */
using enet_ws_frame_send_callback = ENetSSize (MOSS_CALL*)(const uint8_t* data, size_t len, int flags, void* user_data);
/*! @brief Fill at most len bytes into buf. Return bytes filled, or -1 on error (0 is treated as an error). */
using enet_ws_frame_recv_callback = ENetSSize (MOSS_CALL*)(uint8_t* buf, size_t len, int flags, void* user_data);
/*! @brief Write exactly len bytes of mask key into buf. Return 0 on success, -1 on error. */
using enet_ws_frame_genmask_callback = int (MOSS_CALL*)(uint8_t* buf, size_t len, void* user_data);

struct enet_ws_frame_callbacks {
   enet_ws_frame_send_callback    send_callback;
   enet_ws_frame_recv_callback    recv_callback;
   enet_ws_frame_genmask_callback genmask_callback;
};

/* Event-level callbacks. */
/*! @brief Invoked when a message has been completely received. */
using enet_ws_event_on_msg_recv_callback = void (MOSS_CALL*)(enet_ws_event_context_ptr ctx, const enet_ws_event_on_msg_recv_arg* arg, void* user_data);
/*! @brief Invoked once when a new frame starts to be received. */
using enet_ws_event_on_frame_recv_start_callback = void (MOSS_CALL*)(enet_ws_event_context_ptr ctx, const enet_ws_event_on_frame_recv_start_arg* arg, void* user_data);
/*! @brief Invoked when a chunk of frame payload has been received. */
using enet_ws_event_on_frame_recv_chunk_callback = void (MOSS_CALL*)(enet_ws_event_context_ptr ctx, const enet_ws_event_on_frame_recv_chunk_arg* arg, void* user_data);
/*! @brief Invoked when a frame has been completely received. */
using enet_ws_event_on_frame_recv_end_callback = void (MOSS_CALL*)(enet_ws_event_context_ptr ctx, void* user_data);
/*! @brief Read at most len bytes from the peer into buf and return the count. On error return -1 and call enet_ws_event_set_error() with ENET_WS_ERR_CALLBACK_FAILURE, or ENET_WS_ERR_WOULDBLOCK if the cause is EAGAIN/EWOULDBLOCK. */
using enet_ws_event_recv_callback = ENetSSize (MOSS_CALL*)(enet_ws_event_context_ptr ctx, uint8_t* buf, size_t len, int flags, void* user_data);
/*! @brief Send at most len bytes to the peer and return the count. Error handling as for the recv callback. */
using enet_ws_event_send_callback = ENetSSize (MOSS_CALL*)(enet_ws_event_context_ptr ctx, const uint8_t* data, size_t len, int flags, void* user_data);
/*! @brief Write a new 4 byte mask key into buf. Only needed for client contexts (only client traffic is masked). Return 0 on success. */
using enet_ws_event_genmask_callback = int (MOSS_CALL*)(enet_ws_event_context_ptr ctx, uint8_t* buf, size_t len, void* user_data);
/*! @brief Read message data from source for a fragmented message. Return bytes stored; set *eof to 1 at end of data; return 0 if no data is available right now; return -1 and set ENET_WS_ERR_CALLBACK_FAILURE on error. */
using enet_ws_event_fragmented_msg_callback = ENetSSize (MOSS_CALL*)(enet_ws_event_context_ptr ctx, uint8_t* buf, size_t len, const enet_ws_event_msg_source* source, int* eof, void* user_data);

struct enet_ws_event_callbacks {
   enet_ws_event_recv_callback                  recv_callback;
   enet_ws_event_send_callback                  send_callback;
   enet_ws_event_genmask_callback               genmask_callback;
   enet_ws_event_on_frame_recv_start_callback   on_frame_recv_start_callback;
   enet_ws_event_on_frame_recv_chunk_callback   on_frame_recv_chunk_callback;
   enet_ws_event_on_frame_recv_end_callback     on_frame_recv_end_callback;
   enet_ws_event_on_msg_recv_callback           on_msg_recv_callback;
};

struct enet_ws_event_fragmented_msg {
   uint8_t                               opcode;
   enet_ws_event_msg_source              source;        /* where read_callback gets the message data from */
   enet_ws_event_fragmented_msg_callback read_callback;
};

// ---- Frame API (low level) ----
/*! @brief Allocates a frame context. Free it with enet_ws_frame_context_free(). Returns 0 or ENET_WS_ERR_NOMEM. */
MOSS_API int  enet_ws_frame_context_init(enet_ws_frame_context_ptr* ctx, const enet_ws_frame_callbacks* callbacks, void* user_data);
/*! @brief Deallocates a frame context. */
MOSS_API void enet_ws_frame_context_free(enet_ws_frame_context_ptr ctx);
/*! @brief Sends the frame described by iocb. Returns the number of payload bytes sent, ENET_WS_ERR_WANT_WRITE if none could be sent, ENET_WS_ERR_INVALID_ARGUMENT if iocb is invalid, or ENET_WS_ERR_INVALID_CALLBACK if a callback failed. May send only part of the data: adjust iocb->data / data_length and call again. */
MOSS_API ENetSSize enet_ws_frame_send(enet_ws_frame_context_ptr ctx, enet_ws_frame_iocb* iocb);
/*! @brief Receives a frame into iocb. Returns the number of payload bytes received, ENET_WS_ERR_WANT_READ if none, ENET_WS_ERR_PROTO on a protocol violation, or ENET_WS_ERR_INVALID_CALLBACK if a callback failed. Call again until the frame is complete. */
MOSS_API ENetSSize enet_ws_frame_recv(enet_ws_frame_context_ptr ctx, enet_ws_frame_iocb* iocb);

// ---- Event API (recommended) ----
/*! @brief Initializes *ctx as a WebSocket server. Returns 0 or ENET_WS_ERR_NOMEM. */
MOSS_API int  enet_ws_event_context_server_init(enet_ws_event_context_ptr* ctx, const enet_ws_event_callbacks* callbacks, void* user_data);
/*! @brief Initializes *ctx as a WebSocket client (outgoing frames are masked, so genmask_callback is required). Returns 0 or ENET_WS_ERR_NOMEM. */
MOSS_API int  enet_ws_event_context_client_init(enet_ws_event_context_ptr* ctx, const enet_ws_event_callbacks* callbacks, void* user_data);
/*! @brief Releases all resources held by ctx. */
MOSS_API void enet_ws_event_context_free(enet_ws_event_context_ptr ctx);

/*! @brief Sets the allowed reserved bits: ENET_WS_RSV1_BIT (permessage-deflate, RFC 7692) or ENET_WS_RSV_NONE (default). */
MOSS_API void enet_ws_event_config_set_allowed_rsv_bits(enet_ws_event_context_ptr ctx, uint8_t rsv);
/*! @brief val == 0 buffers whole non-control messages (default); otherwise delivers them frame by frame (msg_length is then 0).
 *         Must not be called after the first enet_ws_event_recv(). */
MOSS_API void enet_ws_event_config_set_no_buffering(enet_ws_event_context_ptr ctx, int val);
/*! @brief Maximum receivable message length (default (1u << 31) - 1). Larger messages close the connection with ENET_WS_CODE_MESSAGE_TOO_BIG.
 *         Set this: it is your memory-exhaustion guard against hostile peers. */
MOSS_API void enet_ws_event_config_set_max_recv_msg_length(enet_ws_event_context_ptr ctx, uint64_t val);
/*! @brief Replaces the callbacks set at init time. */
MOSS_API void enet_ws_event_config_set_callbacks(enet_ws_event_context_ptr ctx, const enet_ws_event_callbacks* callbacks);

/*! @brief Receives messages until the recv callback reports ENET_WS_ERR_WOULDBLOCK. Auto-queues a close reply to a close frame and a pong to a ping.
 *         Returns 0, or a negative error (ENET_WS_ERR_CALLBACK_FAILURE / ENET_WS_ERR_NOMEM). After a negative return, close the connection. */
MOSS_API int enet_ws_event_recv(enet_ws_event_context_ptr ctx);
/*! @brief Sends queued messages until the send callback reports ENET_WS_ERR_WOULDBLOCK. Returns 0 or a negative error; after a negative return, close the connection. */
MOSS_API int enet_ws_event_send(enet_ws_event_context_ptr ctx);

/*! @brief Queues a message (control or non-control, sent unfragmented). Returns 0, ENET_WS_ERR_NO_MORE_MSG, ENET_WS_ERR_INVALID_ARGUMENT or ENET_WS_ERR_NOMEM. */
MOSS_API int enet_ws_event_queue_msg(enet_ws_event_context_ptr ctx, const enet_ws_event_msg* arg);
/*! @brief As enet_ws_event_queue_msg() but lets you set reserved bits. */
MOSS_API int enet_ws_event_queue_msg_ex(enet_ws_event_context_ptr ctx, const enet_ws_event_msg* arg, uint8_t rsv);
/*! @brief Queues a fragmented non-control message whose data is produced by arg->read_callback. */
MOSS_API int enet_ws_event_queue_fragmented_msg(enet_ws_event_context_ptr ctx, const enet_ws_event_fragmented_msg* arg);
/*! @brief As enet_ws_event_queue_fragmented_msg() but lets you set reserved bits. */
MOSS_API int enet_ws_event_queue_fragmented_msg_ex(enet_ws_event_context_ptr ctx, const enet_ws_event_fragmented_msg* arg, uint8_t rsv);
/*! @brief Queues a close frame. If status_code is 0 the frame has no payload. reason_length must be < 123. */
MOSS_API int enet_ws_event_queue_close(enet_ws_event_context_ptr ctx, uint16_t status_code, const uint8_t* reason, size_t reason_length);

/*! @brief Tells the library a callback failed (ENET_WS_ERR_CALLBACK_FAILURE or ENET_WS_ERR_WOULDBLOCK). Call from inside your callbacks. */
MOSS_API void enet_ws_event_set_error(enet_ws_event_context_ptr ctx, int val);
/*! @brief 1 if the library wants to read more from the peer. */
MOSS_API int  enet_ws_event_want_read(enet_ws_event_context_ptr ctx);
/*! @brief 1 if there are queued messages waiting to be sent. */
MOSS_API int  enet_ws_event_want_write(enet_ws_event_context_ptr ctx);
/*! @brief Stops reading from the peer. */
MOSS_API void enet_ws_event_shutdown_read(enet_ws_event_context_ptr ctx);
/*! @brief Stops writing to the peer. */
MOSS_API void enet_ws_event_shutdown_write(enet_ws_event_context_ptr ctx);
MOSS_API int  enet_ws_event_get_read_enabled(enet_ws_event_context_ptr ctx);
MOSS_API int  enet_ws_event_get_write_enabled(enet_ws_event_context_ptr ctx);
/*! @brief 1 if a close frame has been received from the peer. */
MOSS_API int  enet_ws_event_get_close_received(enet_ws_event_context_ptr ctx);
/*! @brief 1 if a close frame has been sent to the peer. */
MOSS_API int  enet_ws_event_get_close_sent(enet_ws_event_context_ptr ctx);
/*! @brief Status code in the received close frame (ENET_WS_CODE_ABNORMAL_CLOSURE if none yet, ENET_WS_CODE_NO_STATUS_RCVD if it had no code). */
MOSS_API uint16_t enet_ws_event_get_status_code_received(enet_ws_event_context_ptr ctx);
/*! @brief Status code in the sent close frame (same conventions as above). */
MOSS_API uint16_t enet_ws_event_get_status_code_sent(enet_ws_event_context_ptr ctx);
/*! @brief Number of queued messages. */
MOSS_API size_t enet_ws_event_get_queued_msg_count(enet_ws_event_context_ptr ctx);
/*! @brief Sum of queued message lengths (counts messages queued with enet_ws_event_queue_msg() / _queue_close()). */
MOSS_API size_t enet_ws_event_get_queued_msg_length(enet_ws_event_context_ptr ctx);


// ===============================================
// Crypto & encoding helpers
// ===============================================
// Small, dependency-free primitives the HTTP / WebSocket layers need. They are public because
// games need them too (patch verification, request signing, idempotency keys).
// Not a substitute for a full crypto library: for signatures (Ed25519) or AEAD use libsodium / Monocypher.

/** Encoded length of `inputLength` bytes as padded base64, excluding the NUL terminator. */
MOSS_API size_t enet_base64_encoded_size(size_t inputLength);
/** Writes padded base64 plus a NUL. Returns characters written (excluding NUL), or 0 if outCapacity is too small. */
MOSS_API size_t enet_base64_encode(const void* data, size_t length, char* out, size_t outCapacity);
/** Strict RFC 4648 decode (padding required, no whitespace). Returns 0 on success, -1 on invalid input or small buffer. */
MOSS_API int    enet_base64_decode(const char* text, size_t length, uint8_t* out, size_t outCapacity, size_t* outLength);

/** SHA-1. Only for protocol compatibility (WebSocket handshake). Do not use it for security decisions. */
MOSS_API void   enet_sha1(const void* data, size_t length, uint8_t out[20]);

struct ENetSha256 {
   uint32_t state[8];
   uint64_t length;      // bytes processed
   uint8_t  buffer[64];
   size_t   bufferLength;
};
MOSS_API void enet_sha256_init(ENetSha256* ctx);
MOSS_API void enet_sha256_update(ENetSha256* ctx, const void* data, size_t length);
MOSS_API void enet_sha256_final(ENetSha256* ctx, uint8_t out[32]);
MOSS_API void enet_sha256(const void* data, size_t length, uint8_t out[32]);
MOSS_API void enet_hmac_sha256(const void* key, size_t keyLength, const void* data, size_t length, uint8_t out[32]);

/** Lower-case hex. `out` needs 2 * length + 1 bytes. */
MOSS_API void enet_hex_encode(const uint8_t* data, size_t length, char* out);

/** Cryptographically secure random bytes from the OS. Returns 0 on success, -1 on failure (never falls back to a weak source). */
MOSS_API int  enet_random_bytes(void* out, size_t length);
/** Constant-time comparison. Returns 1 if equal. Use it for tokens, MACs and hashes. */
MOSS_API int  enet_secure_equals(const void* a, const void* b, size_t length);
/** Random (version 4) UUID, NUL terminated. Returns 0 on success. Use it as an Idempotency-Key for purchases and other retried writes. */
MOSS_API int  enet_make_uuid_v4(char out[37]);

// ===============================================
// TLS (pluggable backend)
// ===============================================
// Moss does not ship its own TLS stack. Register one backend per platform:
//   Windows -> SChannel, Apple -> Network.framework / Secure Transport, Linux/Android -> OpenSSL / mbedTLS / BoringSSL,
//   consoles -> the platform SDK's SSL library. tls_openssl.cpp is a complete reference backend.
// https:// and wss:// FAIL CLOSED (ENET_HTTP_ERR_TLS_FAILED) while no backend is registered; they never fall back to plaintext.

struct ENetTransportSecurity {
   const char* cert_file;        // client certificate for mutual TLS (optional)
   const char* key_file;         // matching private key (optional)
   const char* ca_file;          // PEM bundle of trusted roots; NULL = the platform / system trust store
   int         skip_verification;// 0 (the default) = verify the certificate chain AND the host name. Non-zero is for local development only.
};

struct ENetTlsBackend {
   /** Performs the client handshake on an already connected, NON-BLOCKING socket and verifies the peer per `security`.
    *  Must give up within timeoutMs. Returns an opaque connection, or NULL on failure. `security` may be NULL (= defaults). */
   void* (*connect)(ENetSocket socket, const char* host, const ENetTransportSecurity* security, uint32_t timeoutMs);
   /** Non-blocking read. >0 bytes read; 0 = would block (wait for the socket, then retry); -1 = peer closed cleanly
    *  (authenticated close_notify); -2 = error, including a close WITHOUT close_notify (possible truncation attack). */
   int   (*read)(void* connection, void* buffer, size_t length);
   /** Non-blocking write. >0 bytes written; 0 = would block; <0 = error. */
   int   (*write)(void* connection, const void* data, size_t length);
   /** Closes the TLS session (best effort) and frees the connection. Does not close the socket. */
   void  (*close)(void* connection);
};

/** Registers the backend used by every HTTP / WebSocket connection. Call once at startup (before the first request). NULL unregisters. */
MOSS_API void                  enet_tls_set_backend(const ENetTlsBackend* backend);
MOSS_API const ENetTlsBackend* enet_tls_get_backend(void);
/** OpenSSL reference backend. Defined by tls_openssl.cpp: add that file to the build and link OpenSSL 1.1.1+ / 3.x to use it. */
MOSS_API const ENetTlsBackend* enet_tls_openssl_backend(void);

// ===============================================
// HTTP client
// ===============================================
// Designed for a game loop: requests run on a small worker pool (blocking DNS / TCP / TLS happen there),
// and every callback is delivered on YOUR thread from enet_http_poll(). Nothing blocks the frame.
//
// Security defaults: plaintext http:// is refused except to loopback (development); credentials in the URL are refused;
// CR/LF and other control characters in URLs and headers are refused (request smuggling); redirects are NOT followed unless
// asked, never downgrade https to http, and drop Authorization / Cookie when they leave the original origin; responses are
// size-limited; Content-Length / chunked framing is validated and truncated bodies are errors.

enum class ENetHttpMethod : uint8_t {
   ENET_HTTP_GET, 
   ENET_HTTP_POST, 
   ENET_HTTP_PUT, 
   ENET_HTTP_DELETE, 
   ENET_HTTP_PATCH, 
   ENET_HTTP_HEAD
};

enum class ENetHttpError : int32_t {
   ENET_HTTP_OK = 0,
   /* transport-level */
   ENET_HTTP_ERR_DNS_FAILURE, 
   ENET_HTTP_ERR_CONNECT_FAILED, 
   ENET_HTTP_ERR_TLS_FAILED,
   ENET_HTTP_ERR_TIMEOUT, 
   ENET_HTTP_ERR_CANCELED,
   /* protocol-level */
   ENET_HTTP_ERR_INVALID_RESPONSE, ENET_HTTP_ERR_TOO_LARGE,
   /* client misuse */
   ENET_HTTP_ERR_INVALID_ARGUMENT, 
   ENET_HTTP_ERR_INTERNAL,
   /* added with the implementation */
   ENET_HTTP_ERR_IO,                 // socket error / connection dropped mid-transfer
   ENET_HTTP_ERR_INTEGRITY,          // downloaded data failed its SHA-256 / size check
   ENET_HTTP_ERR_TOO_MANY_REDIRECTS,
   ENET_HTTP_ERR_INSECURE,           // refused: plaintext to a non-loopback host, or an https -> http redirect
   ENET_HTTP_ERR_HANDSHAKE_REJECTED  // WebSocket upgrade refused by the server (see http_status, e.g. 401 / 403)
};

struct ENetHttpHeader { const char* name; const char* value; };

struct ENetHttpTLSConfig { const ENetTransportSecurity* security; };

namespace ENetHttpFlag {
   /** Allow plaintext http:// / ws:// to non-loopback hosts. Never ship this on for login or store traffic. */
   constexpr uint32_t ALLOW_INSECURE = 1u << 0;
}

/** Called on a WORKER thread for streaming downloads. Return 0 to accept / continue, non-zero to abort (ENET_HTTP_ERR_CANCELED). */
struct ENetHttpSink {
   /** Called once with the final (post-redirect) status and headers. contentLength is UINT64_MAX when unknown. */
   int  (*on_headers)(void* user, int status, const ENetHttpHeader* headers, size_t headerCount, uint64_t contentLength);
   int  (*on_data)(void* user, const uint8_t* data, size_t length);
   void* user;
};

struct ENetHttpRequestDesc {
   ENetHttpMethod        method;
   const char*           url;
   const ENetHttpHeader* headers;
   size_t                header_count;
   const void*           body;
   size_t                body_size;
   const ENetHttpTLSConfig* tls;
   uint32_t              timeout_ms;          // per attempt, including redirects (for streamed bodies: max silence between reads); 0 = 15000
   /* ---- added fields (zero = safe default) ---- */
   uint32_t              flags;               // ENetHttpFlag bits
   uint32_t              max_redirects;       // 0 = do not follow redirects
   uint64_t              max_response_bytes;  // buffered responses only; 0 = 8 MiB
   uint32_t              max_retries;         // 0 = none. Only idempotent methods, or requests carrying an Idempotency-Key header
   uint32_t              retry_backoff_ms;    // base delay, doubled per attempt with jitter; 0 = 250
   const ENetHttpSink*   sink;                // streams the body instead of buffering it (disables retries)
};

/** Fills a descriptor with sensible values: 15 s timeout, 3 redirects, 8 MiB limit, no retries. Recommended over zero-init. */
MOSS_API void enet_http_request_desc_init(ENetHttpRequestDesc* desc, ENetHttpMethod method, const char* url);

struct ENetHttpResponse {
   int                   http_status;   // e.g. 200, 404; 0 if no response was received
   ENetHttpError         error;
   const ENetHttpHeader* headers;
   size_t                header_count;
   const void*           body;          // NULL when a sink consumed the body
   size_t                body_size;     // bytes delivered
   const char*           final_url;     // URL of the last request (after redirects)
   uint32_t              attempts;      // 1 + retries used
   void*                 internal_;     // private: owner of a result returned by enet_http_perform(); leave untouched
};

struct ENetHttpRequest;
using ENetHttpResponseCallback = void (*)(const ENetHttpResponse* response, void* userData);

struct ENetHttpConfig {
   uint32_t    worker_threads;   // 0 = 2
   const char* user_agent;       // NULL = "Moss/1.0"
};

/** Optional. Called implicitly with defaults by the first request. Returns 0, or -1 if already initialised. */
MOSS_API int  enet_http_init(const ENetHttpConfig* config);
/** Cancels everything, joins the workers and drops undelivered callbacks. Safe to call when not initialised. */
MOSS_API void enet_http_shutdown(void);

/** Queues a request. The descriptor and everything it points to are copied. The callback runs from enet_http_poll() on the
 *  calling thread and the response is valid only during it. Returns a handle (release it with enet_http_request_release) or NULL. */
MOSS_API ENetHttpRequest* enet_http_request_async(const ENetHttpRequestDesc* desc, ENetHttpResponseCallback cb, void* userData);
/** Delivers finished callbacks on this thread. Call once per frame. Returns how many callbacks ran. */
MOSS_API int  enet_http_poll(void);
/** Aborts a request; its callback still fires once with ENET_HTTP_ERR_CANCELED (unless already finished). */
MOSS_API void enet_http_request_cancel(ENetHttpRequest* request);
/** Drops your handle. If the request is still running it is cancelled and its callback is suppressed. */
MOSS_API void enet_http_request_release(ENetHttpRequest* request);

/** Blocking variant: do NOT call from the game thread. Free the result with enet_http_response_free(). Returns the error code. */
MOSS_API ENetHttpError enet_http_perform(const ENetHttpRequestDesc* desc, ENetHttpResponse* out);
MOSS_API void          enet_http_response_free(ENetHttpResponse* response);

/** Case-insensitive header lookup in a response. Returns NULL when absent. */
MOSS_API const char* enet_http_response_header(const ENetHttpResponse* response, const char* name);

// ---- Sessions: base URL + default headers + bearer token (login, profiles, store, leaderboards) ----
struct ENetHttpSession;

/** `baseUrl` is an origin such as "https://api.example.com" (optionally with a path prefix). */
MOSS_API ENetHttpSession* enet_http_session_create(const char* baseUrl);
MOSS_API void             enet_http_session_destroy(ENetHttpSession* session);
/** Adds / replaces a default header sent with every request. Returns 0, or -1 if the name or value is invalid. */
MOSS_API int              enet_http_session_set_header(ENetHttpSession* session, const char* name, const char* value);
/** Sets the Authorization: Bearer token (NULL clears it). The token is only ever sent to the session's own origin. */
MOSS_API void             enet_http_session_set_bearer_token(ENetHttpSession* session, const char* token);
MOSS_API void             enet_http_session_set_tls(ENetHttpSession* session, const ENetTransportSecurity* security);
MOSS_API void             enet_http_session_set_flags(ENetHttpSession* session, uint32_t flags);
/** `desc->url` is a PATH on the session's origin (must start with '/', e.g. "/v1/leaderboards/top?n=10"); other fields as for a plain
 *  request. The session adds the origin, default headers and the bearer token. A 401 is delivered like any other response: refresh your
 *  token, call set_bearer_token and resubmit. */
MOSS_API ENetHttpRequest* enet_http_session_request_async(ENetHttpSession* session, const ENetHttpRequestDesc* desc, ENetHttpResponseCallback cb, void* userData);

// ---- Downloads: patching and content updates ----
struct ENetHttpDownloadResult {
   ENetHttpError error;
   int           http_status;
   uint64_t      bytes;      // size of the finished file
   int           verified;   // 1 if the SHA-256 (and size, when given) matched
   int           resumed;    // 1 if an earlier partial download was continued
};
using ENetHttpProgressCallback = void (*)(uint64_t bytesDone, uint64_t bytesTotal /* 0 = unknown */, void* userData);
using ENetHttpDownloadCallback = void (*)(const ENetHttpDownloadResult* result, void* userData);

struct ENetHttpDownloadDesc {
   const char*           url;
   const char*           dest_path;             // final file; data is written to "<dest_path>.part" and renamed on success
   const char*           expected_sha256_hex;   // 64 hex chars. Strongly recommended; REQUIRED to resume a partial file safely
   uint64_t              expected_size;         // 0 = unknown
   const ENetHttpHeader* headers;
   size_t                header_count;
   const ENetHttpTLSConfig* tls;
   uint32_t              timeout_ms;            // per attempt; 0 = 60000
   uint32_t              max_redirects;         // CDNs usually redirect: 3 is typical
   uint32_t              flags;
};

/** Streams a file to disk with HTTP Range resume and SHA-256 verification. Progress and completion arrive from enet_http_poll(). */
MOSS_API ENetHttpRequest* enet_http_download_async(const ENetHttpDownloadDesc* desc, ENetHttpProgressCallback progress, ENetHttpDownloadCallback done, void* userData);

// ===============================================
// WebSocket connection (handshake + framing over TCP / TLS)
// ===============================================
// Client: enet_websocket_connect() returns immediately; DNS, TCP, TLS and the HTTP Upgrade run on the worker pool.
// Server: enet_websocket_accept() takes a socket you already accepted and completes the Upgrade inside enet_websocket_poll().
// Drive both by calling enet_websocket_poll() every frame until it returns 0.
//
// Security: wss:// is verified like https://. Send tokens in the Authorization header (ENetWebSocketDesc::headers), never in the URL
// (URLs end up in logs). Servers should set allowed_origins when browsers can reach them (cross-site WebSocket hijacking) and
// validate the token in `authorize`. Messages are size-limited (default 1 MiB) so a peer cannot exhaust memory.

struct ENetWebSocket;

enum class ENetWebSocketState : uint8_t { CONNECTING, OPEN, CLOSING, CLOSED };
enum class ENetWebSocketEventType : uint8_t {
   OPEN,      // handshake complete (client: subprotocol set)
   TEXT,      // UTF-8 text message
   BINARY,
   PONG,
   CLOSE,     // orderly close; close_code holds the status
   FAILURE    // handshake rejected, I/O error, protocol violation or timeout; see error / http_status
};

struct ENetWebSocketEvent {
   ENetWebSocketEventType type;
   const uint8_t*         data;         // valid until the next poll / destroy
   size_t                 length;
   uint16_t               close_code;   // CLOSE
   ENetHttpError          error;        // FAILURE
   int                    http_status;  // FAILURE during the handshake (e.g. 401), else 0
   const char*            subprotocol;  // OPEN: negotiated subprotocol or ""
};

struct ENetWebSocketDesc {
   const char*              url;                 // ws:// or wss://
   const ENetHttpHeader*    headers;             // e.g. Authorization: Bearer ...
   size_t                   header_count;
   const char*              subprotocols;        // comma separated offer, e.g. "moss.v1, moss.v0"; NULL = none
   const char*              origin;              // optional Origin header
   const ENetHttpTLSConfig* tls;
   uint32_t                 connect_timeout_ms;  // 0 = 10000
   size_t                   max_message_bytes;   // 0 = 1 MiB
   size_t                   max_queued_bytes;    // outgoing queue limit; 0 = 4 MiB
   uint32_t                 ping_interval_ms;    // 0 = no keep-alive pings
   uint32_t                 idle_timeout_ms;     // 0 = none; fail if nothing is received for this long
   uint32_t                 flags;               // ENetHttpFlag bits
};

struct ENetWebSocketRequest {                    // what a server sees during the Upgrade
   const char*           path;                    // request target, including the query string
   const char*           host;
   const char*           origin;                  // "" when absent
   const char*           subprotocols;            // client's offer, "" when absent
   const ENetHttpHeader* headers;                 // all request headers
   size_t                header_count;
};
/** Return 0 to accept, or the HTTP status to reject with (401, 403, 429, ...). Runs on the thread calling enet_websocket_poll(). */
using ENetWebSocketAuthorizeCallback = int (*)(const ENetWebSocketRequest* request, void* userData);

struct ENetWebSocketServerDesc {
   const char* const*  allowed_origins;       // NULL = do not check Origin
   size_t              allowed_origin_count;
   int                 require_origin;        // non-zero: requests without an Origin header are rejected too
   const char*         subprotocols;          // server's supported list, comma separated; the first client-offered match is chosen
   ENetWebSocketAuthorizeCallback authorize;  // optional
   void*               user;
   uint32_t            handshake_timeout_ms;  // 0 = 5000
   size_t              max_message_bytes;     // 0 = 1 MiB
   size_t              max_queued_bytes;      // 0 = 4 MiB
   uint32_t            ping_interval_ms;
   uint32_t            idle_timeout_ms;
};

MOSS_API ENetWebSocket* enet_websocket_connect(const ENetWebSocketDesc* desc);
/** Takes ownership of `socket` (a connected socket from enet_socket_accept). Plain TCP: terminate TLS in front (reverse proxy) or accept wss yourself. */
MOSS_API ENetWebSocket* enet_websocket_accept(ENetSocket socket, const ENetWebSocketServerDesc* desc);
/** Pumps I/O and returns the next event (1) or 0 when there is none. Call in a loop each frame. */
MOSS_API int  enet_websocket_poll(ENetWebSocket* ws, ENetWebSocketEvent* event);
/** Return 0 on success, -1 if not open, -2 if the outgoing queue limit would be exceeded, -3 on invalid text/argument. Data is copied. */
MOSS_API int  enet_websocket_send_text(ENetWebSocket* ws, const char* text, size_t length);
MOSS_API int  enet_websocket_send_binary(ENetWebSocket* ws, const void* data, size_t length);
MOSS_API int  enet_websocket_send_ping(ENetWebSocket* ws, const void* data, size_t length);
/** Starts the closing handshake (code 0 = none; reason may be NULL). Keep polling until CLOSE arrives (it times out after 5 s). */
MOSS_API int  enet_websocket_close(ENetWebSocket* ws, uint16_t code, const char* reason);
MOSS_API ENetWebSocketState enet_websocket_state(const ENetWebSocket* ws);
/** Immediately drops the connection (no closing handshake) and frees everything. Safe while a connect is still in progress. */
MOSS_API void enet_websocket_destroy(ENetWebSocket* ws);

/** Handshake primitives for custom transports and tests. client key = 24 base64 chars + NUL; accept key = 28 chars + NUL. */
MOSS_API int enet_ws_generate_client_key(char out[25]);
MOSS_API int enet_ws_compute_accept_key(const char* clientKey, char out[29]);

// ===============================================
// Transport abstraction (UDP / TCP / WebSocket / ...)
// ===============================================
// A transport moves bytes between endpoints. Datagram transports (UDP) rely on the
// ENet reliability layer; stream transports (TCP, WebSocket) already provide
// RELIABLE | ORDERED delivery, so the session layer can skip ack/retransmit for them.
// Declarations below are the planned interface; backends are not implemented yet.

enum class ENetTransportType : uint8_t {
   UDP,          // core gameplay
   TCP,          // services, chat
   WEBSOCKET,    // matchmaking, leaderboards, chat, browsers
   WEBRTC,       // P2P / browsers
   WEBTRANSPORT  // QUIC
};

enum class ENetTransportEventType : uint8_t {
   CONNECT,
   ACCEPT,
   RECEIVE,
   DISCONNECT,
   FAILURE        // was ERROR: collides with the ERROR macro from <wingdi.h> on Windows
};

/** Capability bits reported by a transport (combine with |). */
namespace ENetTransportCaps {
   constexpr uint32_t UNRELIABLE = 1u << 0;
   constexpr uint32_t RELIABLE   = 1u << 1;
   constexpr uint32_t ORDERED    = 1u << 2;
   constexpr uint32_t UNORDERED  = 1u << 3;
}

struct ENetTransportConfig {
   int backlog;
   int heartbeat_ms;
   const char* ws_path;
   const char* ws_subprotocol;
   const char* webrtc_stun_servers;
   const char* webrtc_turn_servers;
   const char* quic_cert_file;
   const char* quic_key_file;
};

struct ENetTransportPeer {
   void* backend_ctx;
};

struct ENetTransportEvent {
   ENetTransportEventType type;
   ENetTransportPeer*     peer;
   const uint8_t*         data;
   size_t                 data_len;
   int                    channel; // opaque; map to ENet channels if needed
};

/** Backend vtable. `state` is the backend's own instance data and is passed back as `self`. */
struct ENetTransport {
   uint32_t caps;   // ENetTransportCaps bits
   void*    state;

   int  (*init)(ENetTransport* self, const ENetTransportConfig* config);
   int  (*poll)(ENetTransport* self, ENetTransportEvent* events, int capacity, int timeoutMs);
   int  (*send)(ENetTransport* self, ENetTransportPeer* peer, const uint8_t* data, size_t len, int channel);
   void (*close_peer)(ENetTransport* self, ENetTransportPeer* peer);
   void (*shutdown)(ENetTransport* self);
};

/** Factory / lifecycle. */
MOSS_API ENetTransport* enet_transport_create(ENetTransportType type, const char* bindHost, uint16_t port, const ENetTransportConfig* cfg);
MOSS_API void           enet_transport_destroy(ENetTransport* transport);
/** Non-blocking poll; fills up to `capacity` events and returns the count. */
MOSS_API int            enet_transport_poll(ENetTransport* transport, ENetTransportEvent* events, int capacity, int timeoutMs);
/** Returns bytes sent, or negative on error. */
MOSS_API int            enet_transport_send(ENetTransport* transport, ENetTransportPeer* peer, const uint8_t* data, size_t len, int channel);
MOSS_API int            enet_transport_connect(ENetTransport* transport, ENetTransportPeer* peer, const char* host, uint16_t port);
MOSS_API int            enet_transport_close_peer(ENetTransport* transport, ENetTransportPeer* peer);

MOSS_API ENetHost* enet_host_create_with_transport(const ENetAddress* address, size_t peerCount, size_t channelLimit,
                                                   uint32_t incomingBandwidth, uint32_t outgoingBandwidth, ENetTransportType transport);
/** @param endpoint URL or address string. */
MOSS_API ENetPeer* enet_host_connect_ex(ENetHost* host, const char* endpoint, size_t channelCount, uint32_t data);
MOSS_API uint32_t  enet_host_transport_caps(ENetHost* host);






struct NetPacket;
struct NetPeer;

enum class NetEventType : uint8_t { CONNECT, DISCONNECT, RECEIVE };
enum class NetConnectionState : uint8_t { DISCONNECTED, CONNECTING, HANDSHAKE, CONNECTED, DISCONNECTING };

constexpr uint8_t NET_FLAG_RELIABLE  = 0x01;
constexpr uint8_t NET_FLAG_FRAGMENT  = 0x02;
constexpr uint8_t NET_FLAG_HANDSHAKE = 0x04;
constexpr uint8_t NET_FLAG_ENCRYPTED = 0x08;

// ---- Protocol plane ----
MOSS_API void   enet_transport_handle_event(ENetHost* host, const ENetTransportEvent* tevent);
MOSS_API size_t enet_packet_encode(const NetPacket* packet, uint8_t* outBuffer, size_t bufferSize);
MOSS_API bool   enet_packet_decode(NetPacket* outPacket, const uint8_t* buffer, size_t bufferSize);
MOSS_API void   enet_fragment_send(ENetPeer* peer, uint8_t channelId, const uint8_t* data, size_t size);
MOSS_API bool   enet_fragment_receive(ENetPeer* peer, const NetPacket* fragment, NetPacket* outPacket);
MOSS_API void   enet_bandwidth_update(ENetHost* host, float deltaTime);
MOSS_API size_t enet_peer_send_window(const ENetPeer* peer);
MOSS_API int    enet_peer_create_channel(ENetPeer* peer, bool reliable, bool ordered);
MOSS_API void   enet_peer_reset_channels(ENetPeer* peer);
MOSS_API void   enet_peer_get_stats(const ENetPeer* peer, float* rtt, float* packetLoss, uint32_t* sent, uint32_t* received);
MOSS_API void   enet_host_set_protocol_id(ENetHost* host, uint16_t protocolId);
MOSS_API uint16_t enet_host_get_protocol_id(const ENetHost* host);

// ---- Low-level socket & packet model ----
struct NetSocket {
   int handle;
   ENetAddress localAddress;
};

NetSocket* net_socket_create(uint16_t port);
void       net_socket_close(NetSocket* socket);
int        net_socket_send(NetSocket* socket, ENetAddress* address, const uint8_t* data, size_t size);
int        net_socket_receive(NetSocket* socket, ENetAddress* outAddress, uint8_t* buffer, size_t bufferSize);

struct NetPacketHeader {
   uint16_t protocolId;
   uint8_t  flags;
   uint8_t  channelId;
   uint16_t sequence;
   uint16_t ack;
   uint32_t ackBits;
};

struct NetPacket {
   NetPacketHeader header;
   uint8_t* payload;
   size_t   payloadSize;
};

struct PacketQueue {        // layout TBD
   NetPacket** items;
   size_t head, tail, capacity;
};

struct NetChannel {
   uint8_t  id;
   bool     reliable;
   uint16_t nextSendSeq;
   uint16_t nextRecvSeq;
   PacketQueue outgoing;
   PacketQueue incoming;
};

struct ReliablePacket {
   uint16_t sequence;
   uint32_t sendTime;
   uint8_t  retries;
   NetPacket* packet;
};

void net_channel_send(NetPeer* peer, uint8_t channelId, const uint8_t* data, size_t size, bool reliable);
void net_reliability_update(NetPeer* peer, float deltaTime);

// ---- Security & handshake ----
// Key exchange and connect tokens are still to be designed; do not ship the fields below as-is.
struct NetCryptoContext {
   uint8_t  encryptionKey[32];
   uint8_t  hmacKey[32];
   uint64_t nonce;
};

struct NetHandshake {
   uint32_t challenge;
   uint32_t response;
   bool     verified;
};

bool net_security_encrypt(NetCryptoContext*, uint8_t* data, size_t);
bool net_security_decrypt(NetCryptoContext*, uint8_t* data, size_t);
bool net_handshake_process(NetPeer* peer, NetPacket* packet);

// ---- Peer & host state ----
struct NetPeer {
   ENetAddress address;
   uint32_t    peerId;
   uint16_t    localSequence;
   uint16_t    remoteSequence;
   float       lastReceiveTime;
   float       rtt;
   float       packetLoss;
   bool        connected;
};

struct NetHost {
   NetSocket*       socket;
   NetPeer*         peers;
   size_t           maxPeers;
   NetCryptoContext crypto;
   float            currentTime;
};

NetHost* net_host_create(uint16_t port, size_t maxPeers);
void     net_host_destroy(NetHost* host);
void     net_host_update(NetHost* host, float deltaTime);

// ---- Events & NAT traversal ----
struct NetEvent {
   NetEventType type;
   NetPeer*     peer;
   uint8_t      channelId;
   uint8_t*     data;
   size_t       dataSize;
};

using NetEventCallback = void (*)(NetEvent*, void*);
void net_host_set_callback(NetHost*, NetEventCallback, void* userData);

struct NetNATState {
   bool     punchInProgress;
   uint32_t retries;
};

void net_nat_start_punch(NetPeer*);
void net_nat_update(NetPeer*, float deltaTime);

// ---- Stream framing for TCP ----
struct ENetStreamFrame {
   uint32_t length;
   uint8_t  data[];   // flexible array member (compiler extension in C++)
};

#pragma pack(push, 1)
struct ENetTCPFrameHeader {
   uint32_t length;     // payload size (network byte order)
   uint8_t  channelID;
   uint8_t  flags;
};
#pragma pack(pop)

MOSS_API int enet_transport_send_stream(ENetTransport* t, ENetTransportPeer* peer, const uint8_t* data, size_t len);
MOSS_API int enet_transport_receive_stream(ENetTransport* t, ENetTransportPeer* peer, uint8_t* out, size_t capacity);

// ---- WebRTC signalling ----
enum class ENetWebRTCSignalType : uint8_t { OFFER, ANSWER, ICE_CANDIDATE };

struct ENetWebRTCSignal {
   ENetWebRTCSignalType type;
   const char* data;
};

using ENetWebRTCSignalCallback = void (*)(ENetHost*, ENetPeer*, const ENetWebRTCSignal*, void*);

MOSS_API void enet_transport_ws_close(ENetTransportPeer*, int code);
MOSS_API void enet_transport_webrtc_signal(ENetTransport*, ENetTransportPeer*, const ENetWebRTCSignal*);
MOSS_API void enet_host_set_webrtc_signal_callback(ENetHost*, ENetWebRTCSignalCallback, void*);
MOSS_API int  enet_webrtc_handle_signal(ENetHost*, ENetPeer*, const ENetWebRTCSignal*);

// ---- HTTP server (not planned for the client-focused first release) ----
// Backends for login / matchmaking / store / leaderboards are normally separate services; a Moss dedicated server only needs
// enet_websocket_accept() plus its own TCP listener.
struct ENetHttpServer;
struct ENetHttpServerRequest {
   ENetHttpMethod method;
   const ENetHttpHeader* headers;
   size_t header_count;
   const void* body;
   size_t body_size;
};
struct ENetHttpServerResponse {
   int status;
   const ENetHttpHeader* headers;
   size_t header_count;
   const void* body;
   size_t body_size;
};
using ENetHttpEndpointHandler = void (*)(const ENetHttpServerRequest* request, ENetHttpServerResponse* response, void* userData);
using ENetConnectValidator = int (*)(ENetPeer* peer, const void* data, size_t dataLength);
MOSS_API ENetHttpServer* enet_http_server_start(const char* bindAddress, uint16_t port);
MOSS_API void            enet_http_server_stop(ENetHttpServer* server);
MOSS_API void            enet_http_register_endpoint(ENetHttpServer* server, const char* path, ENetHttpEndpointHandler handler, void* userData);


MOSS_API void enet_host_set_connect_validator(ENetHost* host, ENetConnectValidator validator);
MOSS_API ENetHost* enet_host_create_secure(const ENetAddress* address, size_t peerCount, size_t channelLimit, uint32_t incomingBandwidth, uint32_t outgoingBandwidth, ENetTransportType transport, const ENetTransportSecurity* security);

#endif // MOSS_NETWORK_H
