/** 
 @file  unix.c
 @brief ENet Unix system specific functions
*/
#ifndef _WIN32

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/time.h>
#include <netinet/tcp.h>
#include <netdb.h>
#include <unistd.h>
#include <string.h>
#include <errno.h>
#include <time.h>
#include <stdint.h>

#define ENET_BUILDING_LIB 1
#include <Moss/Moss_Network.h>

/* Every platform Moss targets (Linux, Android, macOS, iOS, BSD and the console POSIX layers)
   has these. They are defaulted here, not left to the build system, because without
   HAS_GETADDRINFO the resolver falls back to gethostbyname(): IPv4 only, which breaks
   IPv6-only / NAT64 mobile networks. */
#ifndef HAS_FCNTL
#define HAS_FCNTL 1
#endif
#ifndef HAS_SOCKLEN_T
#define HAS_SOCKLEN_T 1
#endif
#ifndef HAS_MSGHDR_FLAGS
#define HAS_MSGHDR_FLAGS 1
#endif
#ifndef HAS_GETADDRINFO
#define HAS_GETADDRINFO 1
#endif
#ifndef HAS_GETNAMEINFO
#define HAS_GETNAMEINFO 1
#endif
#ifndef HAS_INET_PTON
#define HAS_INET_PTON 1
#endif
#ifndef HAS_INET_NTOP
#define HAS_INET_NTOP 1
#endif
/* poll() is avoided on Apple platforms (unreliable for some descriptor types); select() is used there. */
#if !defined(__APPLE__) && !defined(HAS_POLL)
#define HAS_POLL 1
#endif
#if defined(__APPLE__) && defined(HAS_POLL)
#undef HAS_POLL
#endif

#ifdef HAS_FCNTL
#include <fcntl.h>
#endif

#ifdef HAS_POLL
#include <poll.h>
#endif

#if !defined(HAS_SOCKLEN_T) && !defined(__socklen_t_defined)
typedef int socklen_t;
#endif

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif

static uint32_t timeBase = 0;

/* Static helpers, defined at the bottom of the file but used throughout. */
static int enet_address_from_sock_addr4(ENetAddress* address, const struct sockaddr_in* sockAddr);
static int enet_address_from_sock_addr6(ENetAddress* address, const struct sockaddr_in6* sockAddr);
static int enet_address_from_addr_info(ENetAddress* address, const struct addrinfo* info);
static int enet_address_from_sock_addr(ENetAddress* address, const struct sockaddr* sockAddr);
static int enet_address_to_sock_addr(const ENetAddress* address, void* sockAddr);
[[maybe_unused]] static int addressFamily[] = {
    AF_UNSPEC, /* ENetAddressType::ANY */
    AF_INET,   /* ENetAddressType::IPV4 */
    AF_INET6   /* ENetAddressType::IPV6 */
};

int Moss_Init_Network (void)
{
    return 0;
}

void Moss_TerminateNetwork(void)
{
}

uint32_t enet_host_random_seed (void)
{
    return (uint32_t) time (NULL);
}

uint32_t enet_time_get (void)
{
    struct timeval timeVal;

    gettimeofday (& timeVal, NULL);

    return timeVal.tv_sec * 1000 + timeVal.tv_usec / 1000 - timeBase;
}

void
enet_time_set (uint32_t newTimeBase)
{
    struct timeval timeVal;

    gettimeofday (& timeVal, NULL);
    
    timeBase = timeVal.tv_sec * 1000 + timeVal.tv_usec / 1000 - newTimeBase;
}

int enet_address_set_host (ENetAddress * address, ENetAddressType type, const char * name)
{
#ifdef HAS_GETADDRINFO
    struct addrinfo hints;
    struct addrinfo* result;
    struct addrinfo* resultList = NULL;
    uint16_t port;
    ENetAddress tempAddress;
    int bestScore = -1;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_UNSPEC;

    if (getaddrinfo(name, NULL, &hints, &resultList) != 0)
        return -1;

    port = address->port; /* preserve port */

    for (result = resultList; result != NULL; result = result->ai_next)
    {
        if (result->ai_addr != NULL)
        {
            if (enet_address_from_addr_info(&tempAddress, result) == 0)
            {
                tempAddress.port = port; /* preserve port */

                int addressScore = 0;
                if (tempAddress.type == type || (tempAddress.type == ENetAddressType::IPV6 && type == ENetAddressType::ANY))
                    addressScore += 10;
                else if (tempAddress.type == ENetAddressType::IPV4)
                {
                    if (type == ENetAddressType::ANY)
                        addressScore += 5; /* lower score than IPv6 addresses */
                    else if (type == ENetAddressType::IPV6)
                    {
                        /* Convert that IPv4 to an IPv6 */
                        enet_address_convert_ipv6(&tempAddress);
                        addressScore += 3; /* lower score than a real IPv6 */
                    }
                }

                if (addressScore > bestScore)
                {
                    memcpy(address, &tempAddress, sizeof(ENetAddress));
                    bestScore = addressScore;
                }
            }
        }
    }

    if (resultList != NULL)
        freeaddrinfo(resultList);

    if (bestScore >= 0)
        return 0;
#else
    struct hostent * hostEntry = NULL;
#ifdef HAS_GETHOSTBYNAME_R
    struct hostent hostData;
    char buffer [2048];
    int errnum;

#if defined(linux) || defined(__linux) || defined(__linux__) || defined(__FreeBSD__) || defined(__FreeBSD_kernel__) || defined(__DragonFly__) || defined(__GNU__)
    gethostbyname_r (name, & hostData, buffer, sizeof (buffer), & hostEntry, & errnum);
#else
    hostEntry = gethostbyname_r (name, & hostData, buffer, sizeof (buffer), & errnum);
#endif
#else
    hostEntry = gethostbyname (name);
#endif

    if (hostEntry != NULL && hostEntry->h_addr_list != NULL && hostEntry->h_addr_list[0] != NULL)
    {
        if (hostEntry->h_addrtype == AF_INET && hostEntry->h_length >= 4)
        {
            address->type = ENetAddressType::IPV4;
            memcpy(address->host.v4, hostEntry->h_addr_list[0], 4);
            address->port = port;
            return 0;
        }
        if (hostEntry->h_addrtype == AF_INET6 && hostEntry->h_length >= 16)
        {
            address->type = ENetAddressType::IPV6;
            memcpy(address->host.v6, hostEntry->h_addr_list[0], 16);
            address->port = port;
            return 0;
        }
    }
#endif

    if (enet_address_set_host_ip(address, name) == 0) { if (type == ENetAddressType::ANY) { enet_address_convert_ipv6(address); } return 0; }
    else { return -1; }
}

int enet_address_get_host(const ENetAddress * address, char * name, size_t nameLength) {
    unsigned char sockAddrBuf[sizeof(struct sockaddr_in6)];
    int socketAddressLen = enet_address_to_sock_addr(address, sockAddrBuf);
#ifdef HAS_GETNAMEINFO
    int err;

    err = getnameinfo ((struct sockaddr *) sockAddrBuf, socketAddressLen, name, nameLength, NULL, 0, NI_NAMEREQD);
    if (! err)
    {
        if (name != NULL && nameLength > 0 && ! memchr (name, '\0', nameLength))
          return -1;

        return 0;
    }
    if (err != EAI_NONAME)
      return -1;
#else
    struct in_addr in;
    struct hostent * hostEntry = NULL;
#ifdef HAS_GETHOSTBYADDR_R
    struct hostent hostData;
    char buffer [2048];
    int errnum;

#if defined(linux) || defined(__linux) || defined(__linux__) || defined(__FreeBSD__) || defined(__FreeBSD_kernel__) || defined(__DragonFly__) || defined(__GNU__)
    gethostbyaddr_r ((char *) sockAddrBuf, socketAddressLen, addressFamily[(int) address->type], & hostData, buffer, sizeof (buffer), & hostEntry, & errnum);
#else
    hostEntry = gethostbyaddr_r ((char *) sockAddrBuf, socketAddressLen, addressFamily[(int) address->type], & hostData, buffer, sizeof (buffer), & errnum);
#endif
#else
    hostEntry = gethostbyaddr ((char *) sockAddrBuf, socketAddressLen, addressFamily[(int) address->type]);
#endif

    if (hostEntry != NULL)
    {
       size_t hostLen = strlen (hostEntry -> h_name);
       if (hostLen >= nameLength)
         return -1;
       memcpy (name, hostEntry -> h_name, hostLen + 1);
       return 0;
    }
#endif

    return enet_address_get_host_ip (address, name, nameLength);
}

int enet_socket_bind (ENetSocket socket, const ENetAddress * address) {
    unsigned char sockAddrBuf[sizeof(struct sockaddr_in6)];
    socklen_t socketAddressLen;

    if (address != NULL)
    {
        int len = enet_address_to_sock_addr(address, sockAddrBuf);
        if (len == 0)
            return -1;
        socketAddressLen = (socklen_t) len;
    }
    else
    {
        /* No address: bind to the wildcard address, port 0, in the socket's own family. */
        struct sockaddr_storage local;
        socklen_t localLen = sizeof(local);
        memset(&local, 0, sizeof(local));
        if (getsockname(socket, (struct sockaddr*) &local, &localLen) == -1)
            return -1;

        memset(sockAddrBuf, 0, sizeof(sockAddrBuf));
        if (local.ss_family == AF_INET6)
        {
            struct sockaddr_in6* sin6 = (struct sockaddr_in6*) sockAddrBuf;
            sin6->sin6_family = AF_INET6;
            sin6->sin6_addr = in6addr_any;
            socketAddressLen = sizeof(struct sockaddr_in6);
        }
        else
        {
            struct sockaddr_in* sin = (struct sockaddr_in*) sockAddrBuf;
            sin->sin_family = AF_INET;
            sin->sin_addr.s_addr = htonl(INADDR_ANY);
            socketAddressLen = sizeof(struct sockaddr_in);
        }
    }

    return bind(socket, (struct sockaddr*) sockAddrBuf, socketAddressLen);
}

int enet_socket_get_address (ENetSocket socket, ENetAddress * address) {
    unsigned char sockAddrBuf[sizeof(struct sockaddr_in6)] = { 0 };
    socklen_t bufferLength = sizeof(sockAddrBuf);

    if (getsockname(socket, (struct sockaddr *) sockAddrBuf, &bufferLength) == -1) return -1;

    return enet_address_from_sock_addr(address, (struct sockaddr *) sockAddrBuf);
}

int enet_socket_listen (ENetSocket socket, int backlog) {
    return listen (socket, backlog < 0 ? SOMAXCONN : backlog);
}

ENetSocket enet_socket_create(ENetAddressType address_type, ENetSocketType socket_type) {
    /* ANY and IPV6 use an IPv6 socket (ANY is made dual-stack by the caller with NET_OPT_IPV6ONLY = 0). */
    int family = (address_type == ENetAddressType::IPV4) ? PF_INET : PF_INET6;
    ENetSocket sock = socket(family, socket_type == ENetSocketType::DATAGRAM ? SOCK_DGRAM : SOCK_STREAM, 0);

#ifdef SO_NOSIGPIPE
    /* Apple has no MSG_NOSIGNAL: writing to a closed TCP/WebSocket connection would raise SIGPIPE and kill the process. */
    if (sock != -1)
    {
        int on = 1;
        setsockopt(sock, SOL_SOCKET, SO_NOSIGPIPE, (char*) &on, sizeof(on));
    }
#endif

    return sock;
}

int enet_socket_set_option (ENetSocket socket, ENetSocketOption option, int value) {
    int result = -1;
    switch (option)
    {
        case ENetSocketOption::NET_OPT_NONBLOCK:
#ifdef HAS_FCNTL
            result = fcntl (socket, F_SETFL, (value ? O_NONBLOCK : 0) | (fcntl (socket, F_GETFL) & ~O_NONBLOCK));
#else
            result = ioctl (socket, FIONBIO, & value);
#endif
            break;

        case ENetSocketOption::NET_OPT_BROADCAST:
            result = setsockopt (socket, SOL_SOCKET, SO_BROADCAST, (char *) & value, sizeof (int));
            break;

        case ENetSocketOption::NET_OPT_REUSEADDR:
            result = setsockopt (socket, SOL_SOCKET, SO_REUSEADDR, (char *) & value, sizeof (int));
            break;

        case ENetSocketOption::NET_OPT_RCVBUF:
            result = setsockopt (socket, SOL_SOCKET, SO_RCVBUF, (char *) & value, sizeof (int));
            break;

        case ENetSocketOption::NET_OPT_SNDBUF:
            result = setsockopt (socket, SOL_SOCKET, SO_SNDBUF, (char *) & value, sizeof (int));
            break;

        case ENetSocketOption::NET_OPT_RCVTIMEO:
        {
            struct timeval timeVal;
            timeVal.tv_sec = value / 1000;
            timeVal.tv_usec = (value % 1000) * 1000;
            result = setsockopt (socket, SOL_SOCKET, SO_RCVTIMEO, (char *) & timeVal, sizeof (struct timeval));
            break;
        }

        case ENetSocketOption::NET_OPT_SNDTIMEO:
        {
            struct timeval timeVal;
            timeVal.tv_sec = value / 1000;
            timeVal.tv_usec = (value % 1000) * 1000;
            result = setsockopt (socket, SOL_SOCKET, SO_SNDTIMEO, (char *) & timeVal, sizeof (struct timeval));
            break;
        }

        case ENetSocketOption::NET_OPT_NODELAY:
            result = setsockopt (socket, IPPROTO_TCP, TCP_NODELAY, (char *) & value, sizeof (int));
            break;

        case ENetSocketOption::NET_OPT_TTL:
            result = setsockopt (socket, IPPROTO_IP, IP_TTL, (char *) & value, sizeof (int));
            break;

        case ENetSocketOption::NET_OPT_IPV6ONLY:
            result = setsockopt (socket, IPPROTO_IPV6, IPV6_V6ONLY, (char *) & value, sizeof (int));
            break;

        default:
            break;
    }
    return result == -1 ? -1 : 0;
}

int enet_socket_get_option (ENetSocket socket, ENetSocketOption option, int * value) {
    int result = -1;
    socklen_t len;
    switch (option)
    {
        case ENetSocketOption::NET_OPT_ERROR:
            len = sizeof (int);
            result = getsockopt (socket, SOL_SOCKET, SO_ERROR, value, & len);
            break;

        case ENetSocketOption::NET_OPT_TTL:
            len = sizeof (int);
            result = getsockopt (socket, IPPROTO_IP, IP_TTL, (char *) value, & len);
            break;

        default:
            break;
    }
    return result == -1 ? -1 : 0;
}

int enet_socket_connect (ENetSocket socket, const ENetAddress * address) {
    unsigned char sockAddrBuf[sizeof(struct sockaddr_in6)];
    int socketAddressLen = enet_address_to_sock_addr(address, sockAddrBuf);
    int result;

    result = connect(socket, (struct sockaddr*) sockAddrBuf, socketAddressLen);
    if (result == -1 && errno == EINPROGRESS) return 0;

    return result;
}

ENetSocket enet_socket_accept (ENetSocket socket, ENetAddress * address) {
    int result;
    unsigned char sockAddrBuf[sizeof(struct sockaddr_in6)] = { 0 };
    socklen_t socketAddressLen = sizeof(sockAddrBuf);

    result = accept(socket,  address != NULL ? (struct sockaddr*) sockAddrBuf : NULL, address != NULL ? & socketAddressLen : NULL);
    
    if (result == -1) return ENET_SOCKET_NULL;

    if (address != NULL) { if (enet_address_from_sock_addr(address, (struct sockaddr*) sockAddrBuf) != 0) return ENET_SOCKET_NULL; }

    return result;
} 
    
int enet_socket_shutdown (ENetSocket socket, ENetSocketShutdown how) { return shutdown(socket, (int) how); }

void enet_socket_destroy (ENetSocket socket) { if (socket != -1) { close (socket);} }

int enet_socket_send (ENetSocket socket, const ENetAddress * address, const ENetBuffer * buffers, size_t bufferCount) {
    unsigned char sockAddrBuf[sizeof(struct sockaddr_in6)];
    struct msghdr msgHdr;
    int sentLength;

    memset (& msgHdr, 0, sizeof (struct msghdr));

    if (address != NULL)
    {
        msgHdr.msg_namelen = enet_address_to_sock_addr(address, sockAddrBuf);
        if (msgHdr.msg_namelen == 0)
            return -1;

        msgHdr.msg_name = (struct sockaddr *) sockAddrBuf;
    }

    msgHdr.msg_iov = (struct iovec *) buffers;
    msgHdr.msg_iovlen = bufferCount;

    sentLength = sendmsg (socket, & msgHdr, MSG_NOSIGNAL);
    
    if (sentLength == -1)
    {
       if (errno == EWOULDBLOCK || errno == EAGAIN)
         return 0;

       return -1;
    }

    return sentLength;
}

int enet_socket_receive (ENetSocket socket, ENetAddress * address, ENetBuffer * buffers, size_t bufferCount) {
    unsigned char sockAddrBuf[sizeof(struct sockaddr_in6)] = { 0 };
    socklen_t socketAddressLen = sizeof(sockAddrBuf);
    struct msghdr msgHdr;
    int recvLength;

    memset (& msgHdr, 0, sizeof (struct msghdr));

    if (address != NULL)
    {
        msgHdr.msg_name = (struct sockaddr*) &sockAddrBuf;
        msgHdr.msg_namelen = socketAddressLen;
    }

    msgHdr.msg_iov = (struct iovec *) buffers;
    msgHdr.msg_iovlen = bufferCount;

    recvLength = recvmsg (socket, & msgHdr, MSG_NOSIGNAL);

    if (recvLength == -1)
    {
        switch (errno)
        {
            case EWOULDBLOCK:
#if EAGAIN != EWOULDBLOCK
            case EAGAIN:
#endif
                return 0;
            case EINTR:
            case EMSGSIZE:
                return -2;
            default:
                return -1;
        }
    }

#ifdef HAS_MSGHDR_FLAGS
    if (msgHdr.msg_flags & MSG_TRUNC)
      return -2;
#endif

    if (address != NULL)
    {
        if (enet_address_from_sock_addr(address, (struct sockaddr*) sockAddrBuf) != 0)
            return -1;
    }

    return recvLength;
}

int enet_socket_receive_stream(ENetSocket socket, void* data, size_t length) {
    for (;;)
    {
        ssize_t r = recv(socket, data, length, 0);
        if (r > 0) return (int) r;
        if (r == 0) return -1;                      /* orderly shutdown by the peer */
        if (errno == EINTR) continue;
        if (errno == EWOULDBLOCK || errno == EAGAIN) return 0;
        return -2;
    }
}

int enet_socket_send_stream(ENetSocket socket, const void* data, size_t length) {
    for (;;)
    {
        ssize_t r = send(socket, data, length, MSG_NOSIGNAL);
        if (r >= 0) return (int) r;
        if (errno == EINTR) continue;
        if (errno == EWOULDBLOCK || errno == EAGAIN) return 0;
        return -1;
    }
}

int
enet_socketset_select (ENetSocket maxSocket, ENetSocketSet * readSet, ENetSocketSet * writeSet, uint32_t timeout)
{
    struct timeval timeVal;

    timeVal.tv_sec = timeout / 1000;
    timeVal.tv_usec = (timeout % 1000) * 1000;

    return select (maxSocket + 1, readSet, writeSet, NULL, & timeVal);
}

int
enet_socket_wait (ENetSocket socket, uint32_t * condition, uint32_t timeout)
{
#ifdef HAS_POLL
    struct pollfd pollSocket;
    int pollCount;
    
    pollSocket.fd = socket;
    pollSocket.events = 0;

    if (* condition & ENetSocketWait::SEND)
      pollSocket.events |= POLLOUT;

    if (* condition & ENetSocketWait::RECEIVE)
      pollSocket.events |= POLLIN;

    pollCount = poll (& pollSocket, 1, timeout);

    if (pollCount < 0)
    {
        if (errno == EINTR && * condition & ENetSocketWait::INTERRUPT)
        {
            * condition = static_cast<uint32_t>(ENetSocketWait::INTERRUPT);

            return 0;
        }

        return -1;
    }

    * condition = static_cast<uint32_t>(ENetSocketWait::NONE);

    if (pollCount == 0)
      return 0;

    if (pollSocket.revents & POLLOUT)
      * condition |= ENetSocketWait::SEND;
    
    if (pollSocket.revents & POLLIN)
      * condition |= ENetSocketWait::RECEIVE;

    return 0;
#else
    fd_set readSet, writeSet;
    struct timeval timeVal;
    int selectCount;

    timeVal.tv_sec = timeout / 1000;
    timeVal.tv_usec = (timeout % 1000) * 1000;

    FD_ZERO (& readSet);
    FD_ZERO (& writeSet);

    if (* condition & ENetSocketWait::SEND)
      FD_SET (socket, & writeSet);

    if (* condition & ENetSocketWait::RECEIVE)
      FD_SET (socket, & readSet);

    selectCount = select (socket + 1, & readSet, & writeSet, NULL, & timeVal);

    if (selectCount < 0)
    {
        if (errno == EINTR && * condition & ENetSocketWait::INTERRUPT)
        {
            * condition = static_cast<uint32_t>(ENetSocketWait::INTERRUPT);

            return 0;
        }
      
        return -1;
    }

    * condition = static_cast<uint32_t>(ENetSocketWait::NONE);

    if (selectCount == 0)
      return 0;

    if (FD_ISSET (socket, & writeSet))
      * condition |= ENetSocketWait::SEND;

    if (FD_ISSET (socket, & readSet))
      * condition |= ENetSocketWait::RECEIVE;

    return 0;
#endif
}



static int enet_address_from_sock_addr4(ENetAddress * address, const struct sockaddr_in* sockAddr) {
    address->type = ENetAddressType::IPV4;
    address->port = ENET_NET_TO_HOST_16(sockAddr->sin_port);

    memcpy(&address->host.v4[0], &sockAddr->sin_addr.s_addr, 4 * sizeof(uint8_t));

    return 0;
}

static int enet_address_from_sock_addr6(ENetAddress * address, const struct sockaddr_in6* sockAddr) {
    int i;

    address->type = ENetAddressType::IPV6;
    address->port = ENET_NET_TO_HOST_16(sockAddr->sin6_port);

    for (i = 0; i < 8; ++i)
        address->host.v6[i] = ((uint16_t) sockAddr->sin6_addr.s6_addr[i * 2]) << 8 | sockAddr->sin6_addr.s6_addr[i * 2 + 1];

    return 0;
}

static int enet_address_from_addr_info(ENetAddress * address, const struct addrinfo * info) {
    switch (info->ai_family) {
        case AF_INET:
            return enet_address_from_sock_addr4(address, (struct sockaddr_in*) info->ai_addr);
        case AF_INET6:
            return enet_address_from_sock_addr6(address, (struct sockaddr_in6*) info->ai_addr);
        default:
            return -1;
    }
}

static int enet_address_from_sock_addr(ENetAddress * address, const struct sockaddr * sockAddr)
{
    switch (sockAddr->sa_family) {
        case AF_INET:
            return enet_address_from_sock_addr4(address, (struct sockaddr_in*) sockAddr);
        case AF_INET6:
            return enet_address_from_sock_addr6(address, (struct sockaddr_in6*) sockAddr);
        default:
            return -1;
    }
}

static int enet_address_to_sock_addr(const ENetAddress* address, void* sockAddr)
{
    switch (address->type) {
        case ENetAddressType::IPV4: {
            struct sockaddr_in* socketAddress = (struct sockaddr_in*) sockAddr;
            uint32_t addr;

            memset(socketAddress, 0, sizeof(struct sockaddr_in));
            socketAddress->sin_family = AF_INET;
            socketAddress->sin_port = ENET_HOST_TO_NET_16(address->port);

            addr = ((uint32_t) address->host.v4[0]) << 24 | ((uint32_t) address->host.v4[1]) << 16 | ((uint32_t) address->host.v4[2]) <<  8 | ((uint32_t) address->host.v4[3]) <<  0;

            socketAddress->sin_addr.s_addr = htonl(addr);

            return sizeof(struct sockaddr_in);
        }

        case ENetAddressType::IPV6: {
            struct sockaddr_in6* socketAddress = (struct sockaddr_in6*) sockAddr;

            memset(socketAddress, 0, sizeof(struct sockaddr_in6));
            socketAddress->sin6_family = AF_INET6;
            socketAddress->sin6_port = ENET_HOST_TO_NET_16(address->port);

            for (int i = 0; i < 8; ++i)
            {
                /* v6[i] is a host-order 16-bit group; s6_addr is network order (most significant byte first). */
                socketAddress->sin6_addr.s6_addr[i * 2 + 0] = (uint8_t) (address->host.v6[i] >> 8);
                socketAddress->sin6_addr.s6_addr[i * 2 + 1] = (uint8_t) (address->host.v6[i] & 0xFF);
            }

            return sizeof(struct sockaddr_in6);
        }

        default:
            return 0;
    }
}

#endif
