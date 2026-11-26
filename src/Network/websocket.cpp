/*
 * Moss WebSocket layer - RFC 6455 framing, merged into Moss_Network.
 *
 * Derived from Wslay - The WebSocket Library
 *
 * Copyright (c) 2011, 2012 Tatsuhiro Tsujikawa
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Moss changes: renamed to the enet_ws_* / ENET_WS_* namespace, memory goes through
 * enet_malloc / enet_free (so the callbacks in callbacks.cpp apply), ssize_t replaced
 * by ENetSSize, internal containers and context layouts are private to this file.
 */
#define ENET_BUILDING_LIB 1
#include <string.h>
#include <assert.h>
#include <Moss/Moss_Network.h>

// ------------------------------------------------------------------
// Private definitions (not part of the public API)
// ------------------------------------------------------------------

/* Wire-order conversion for the 64-bit extended payload length. */
#if defined(__BYTE_ORDER__) && defined(__ORDER_BIG_ENDIAN__) && (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
#  define ENET_WS_NTOH64(x) (x)
#  define ENET_WS_HTON64(x) (x)
#else
static inline uint64_t enet_ws_byteswap64(uint64_t x) {
  uint64_t lo = ENET_NET_TO_HOST_32((uint32_t)(x & 0xffffffffu));
  uint64_t hi = ENET_NET_TO_HOST_32((uint32_t)(x >> 32));
  return (lo << 32) | hi;
}
#  define ENET_WS_NTOH64(x) enet_ws_byteswap64(x)
#  define ENET_WS_HTON64(x) enet_ws_byteswap64(x)
#endif

enum enet_ws_frame_state {
  ENET_WS_FRAME_PREP_HEADER,
  ENET_WS_FRAME_SEND_HEADER,
  ENET_WS_FRAME_SEND_PAYLOAD,
  ENET_WS_FRAME_RECV_HEADER1,
  ENET_WS_FRAME_RECV_PAYLOADLEN,
  ENET_WS_FRAME_RECV_EXT_PAYLOADLEN,
  ENET_WS_FRAME_RECV_MASKKEY,
  ENET_WS_FRAME_RECV_PAYLOAD
};

enum enet_ws_event_msg_type {
  ENET_WS_NON_FRAGMENTED,
  ENET_WS_FRAGMENTED
};

enum enet_ws_event_close_status {
  ENET_WS_CLOSE_RECEIVED = 1 << 0,
  ENET_WS_CLOSE_QUEUED   = 1 << 1,
  ENET_WS_CLOSE_SENT     = 1 << 2
};

enum enet_ws_event_config {
  ENET_WS_CONFIG_NO_BUFFERING = 1 << 0
};

struct enet_ws_queue_cell { void* data; struct enet_ws_queue_cell* next; };
struct enet_ws_queue      { struct enet_ws_queue_cell* top; struct enet_ws_queue_cell* tail; };

static struct enet_ws_queue* enet_ws_queue_new(void);
static void  enet_ws_queue_free(struct enet_ws_queue* queue);
static int   enet_ws_queue_push(struct enet_ws_queue* queue, void* data);
static int   enet_ws_queue_push_front(struct enet_ws_queue* queue, void* data);
static void  enet_ws_queue_pop(struct enet_ws_queue* queue);
static void* enet_ws_queue_top(struct enet_ws_queue* queue);
static void* enet_ws_queue_tail(struct enet_ws_queue* queue);
static int   enet_ws_queue_empty(struct enet_ws_queue* queue);


struct enet_ws_frame_opcode_memo { uint8_t fin; uint8_t opcode; uint8_t rsv; };

struct enet_ws_frame_context {
  uint8_t  ibuf[4096];
  uint8_t* ibufmark;
  uint8_t* ibuflimit;
  struct enet_ws_frame_opcode_memo iom;
  uint64_t ipayloadlen;
  uint64_t ipayloadoff;
  uint8_t  imask;
  uint8_t  imaskkey[4];
  enum enet_ws_frame_state istate;
  size_t   ireqread;

  uint8_t  oheader[14];
  uint8_t* oheadermark;
  uint8_t* oheaderlimit;
  uint64_t opayloadlen;
  uint64_t opayloadoff;
  uint8_t  omask;
  uint8_t  omaskkey[4];
  enum enet_ws_frame_state ostate;

  struct enet_ws_frame_callbacks callbacks;
  void* user_data;
};

struct enet_ws_event_byte_chunk { uint8_t* data; size_t data_length; };

struct enet_ws_event_imsg {
  uint8_t  fin;
  uint8_t  rsv;
  uint8_t  opcode;
  uint32_t utf8state;
  struct enet_ws_queue* chunks;
  size_t   msg_length;
};

struct enet_ws_event_omsg {
  uint8_t  fin;
  uint8_t  opcode;
  uint8_t  rsv;
  enum enet_ws_event_msg_type type;

  uint8_t* data;
  size_t   data_length;

  union enet_ws_event_msg_source source;
  enet_ws_event_fragmented_msg_callback read_callback;
};

struct enet_ws_event_frame_user_data {
  enet_ws_event_context_ptr ctx;
  void* user_data;
};

struct enet_ws_event_context {
  uint32_t config;                    /* bitwise OR of enum enet_ws_event_config */
  uint64_t max_recv_msg_length;
  uint8_t  server;                    /* 1 if initialized for server use */
  uint8_t  close_status;              /* bitwise OR of enum enet_ws_event_close_status */
  uint16_t status_code_recv;
  uint16_t status_code_sent;
  enet_ws_frame_context_ptr frame_ctx;
  uint8_t  read_enabled;
  uint8_t  write_enabled;
  struct enet_ws_event_imsg imsgs[2]; /* two buffers so control frames can interleave fragmented messages */
  struct enet_ws_event_imsg* imsg;
  uint64_t ipayloadlen;
  uint64_t ipayloadoff;
  int      error;                     /* set by user callbacks */
  struct enet_ws_event_omsg* omsg;    /* message currently being sent, or NULL */
  struct enet_ws_queue* send_queue;       /* non-control frames */
  struct enet_ws_queue* send_ctrl_queue;  /* control frames */
  size_t   queued_msg_count;
  size_t   queued_msg_length;
  uint8_t  obuf[4096];                /* buffer used for fragmented messages */
  uint8_t* obuflimit;
  uint8_t* obufmark;
  uint64_t opayloadlen;
  uint64_t opayloadoff;
  struct enet_ws_event_callbacks callbacks;
  struct enet_ws_event_frame_user_data frame_user_data;
  void*    user_data;
  uint8_t  allowed_rsv_bits;
};

#define UTF8_ACCEPT 0
#define UTF8_REJECT 12


#define enet_ws_min(A, B) (((A) < (B)) ? (A) : (B))
#define ENET_WS_AVAIL_IBUF(ctx) ((size_t)(ctx->ibuflimit - ctx->ibufmark))


static const uint8_t utf8d[] = {
  // The first part of the table maps bytes to character classes that to reduce the size of the transition table and create bitmasks.
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
   0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
   1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,  9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,9,
   7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,  7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,7,
   8,8,2,2,2,2,2,2,2,2,2,2,2,2,2,2,  2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,2,
  10,3,3,3,3,3,3,3,3,3,3,3,3,4,3,3, 11,6,6,6,5,8,8,8,8,8,8,8,8,8,8,8,

  // The second part is a transition table that maps a combination of a state of the automaton and a character class to a state.
   0,12,24,36,60,96,84,12,12,12,48,72, 12,12,12,12,12,12,12,12,12,12,12,12,
  12, 0,12,12,12,12,12, 0,12, 0,12,12, 12,24,12,12,12,12,12,24,12,24,12,12,
  12,12,12,12,12,12,12,24,12,12,12,12, 12,24,12,12,12,12,12,12,12,24,12,12,
  12,12,12,12,12,12,12,36,12,36,12,12, 12,36,12,12,12,12,12,36,12,36,12,12,
  12,36,12,12,12,12,12,12,12,12,12,12,
};


static uint32_t decode(uint32_t* state, uint32_t* codep, uint32_t byte) {
  uint32_t type = utf8d[byte];

  *codep = (*state != UTF8_ACCEPT) ?
    (byte & 0x3fu) | (*codep << 6) :
    (0xff >> type) & (byte);

  *state = utf8d[256 + *state + type];
  return *state;
}

/* End of utf8 dfa */

static ENetSSize enet_ws_event_frame_recv_callback(uint8_t *buf, size_t len, int flags, void *user_data)
{
  struct enet_ws_event_frame_user_data *e =
    (struct enet_ws_event_frame_user_data*)user_data;
  return e->ctx->callbacks.recv_callback(e->ctx, buf, len, flags, e->user_data);
}

static ENetSSize enet_ws_event_frame_send_callback(const uint8_t *data, size_t len, int flags, void *user_data)
{
  struct enet_ws_event_frame_user_data *e =
    (struct enet_ws_event_frame_user_data*)user_data;
  return e->ctx->callbacks.send_callback(e->ctx, data, len, flags,
                                         e->user_data);
}

static int enet_ws_event_frame_genmask_callback(uint8_t *buf, size_t len, void *user_data)
{
  struct enet_ws_event_frame_user_data *e =
    (struct enet_ws_event_frame_user_data*)user_data;
  return e->ctx->callbacks.genmask_callback(e->ctx, buf, len, e->user_data);
}

static int enet_ws_event_byte_chunk_init(struct enet_ws_event_byte_chunk **chunk, size_t len)
{
  *chunk = (struct enet_ws_event_byte_chunk*)enet_malloc
    (sizeof(struct enet_ws_event_byte_chunk));
  if(*chunk == NULL) {
    return ENET_WS_ERR_NOMEM;
  }
  memset(*chunk, 0, sizeof(struct enet_ws_event_byte_chunk));
  if(len) {
    (*chunk)->data = (uint8_t*)enet_malloc(len);
    if((*chunk)->data == NULL) {
      enet_free(*chunk);
      return ENET_WS_ERR_NOMEM;
    }
    (*chunk)->data_length = len;
  }
  return 0;
}

static void enet_ws_event_byte_chunk_free(struct enet_ws_event_byte_chunk *c)
{
  if(!c) {
    return;
  }
  enet_free(c->data);
  enet_free(c);
}

static void enet_ws_event_byte_chunk_copy(struct enet_ws_event_byte_chunk *c,
                                        size_t off,
                                        const uint8_t *data, size_t data_length)
{
  memcpy(c->data+off, data, data_length);
}

static void enet_ws_event_imsg_set(struct enet_ws_event_imsg *m,
                                 uint8_t fin, uint8_t rsv, uint8_t opcode)
{
  m->fin = fin;
  m->rsv = rsv;
  m->opcode = opcode;
  m->msg_length = 0;
}

static void enet_ws_event_imsg_chunks_free(struct enet_ws_event_imsg *m)
{
  if(!m->chunks) {
    return;
  }
  while(!enet_ws_queue_empty(m->chunks)) {
    enet_ws_event_byte_chunk_free((struct enet_ws_event_byte_chunk*)enet_ws_queue_top(m->chunks));
    enet_ws_queue_pop(m->chunks);
  }
}

static void enet_ws_event_imsg_reset(struct enet_ws_event_imsg *m)
{
  m->opcode = 0xffu;
  m->utf8state = UTF8_ACCEPT;
  enet_ws_event_imsg_chunks_free(m);
}

static int enet_ws_event_imsg_append_chunk(struct enet_ws_event_imsg *m, size_t len)
{
  if(len == 0) {
    return 0;
  } else {
    int r;
    struct enet_ws_event_byte_chunk *chunk;
    if((r = enet_ws_event_byte_chunk_init(&chunk, len)) != 0) {
      return r;
    }
    if((r = enet_ws_queue_push(m->chunks, chunk)) != 0) {
      return r;
    }
    m->msg_length += len;
    return 0;
  }
}

static int enet_ws_event_omsg_non_fragmented_init
(struct enet_ws_event_omsg **m, uint8_t opcode, uint8_t rsv,
 const uint8_t *msg, size_t msg_length)
{
  *m = (struct enet_ws_event_omsg*)enet_malloc(sizeof(struct enet_ws_event_omsg));
  if(!*m) {
    return ENET_WS_ERR_NOMEM;
  }
  memset(*m, 0, sizeof(struct enet_ws_event_omsg));
  (*m)->fin = 1;
  (*m)->opcode = opcode;
  (*m)->rsv = rsv;
  (*m)->type = ENET_WS_NON_FRAGMENTED;
  if(msg_length) {
    (*m)->data = (uint8_t*)enet_malloc(msg_length);
    if(!(*m)->data) {
      enet_free(*m);
      return ENET_WS_ERR_NOMEM;
    }
    memcpy((*m)->data, msg, msg_length);
    (*m)->data_length = msg_length;
  }
  return 0;
}

static int enet_ws_event_omsg_fragmented_init
(struct enet_ws_event_omsg **m, uint8_t opcode, uint8_t rsv,
 const union enet_ws_event_msg_source source,
 enet_ws_event_fragmented_msg_callback read_callback)
{
  *m = (struct enet_ws_event_omsg*)enet_malloc(sizeof(struct enet_ws_event_omsg));
  if(!*m) {
    return ENET_WS_ERR_NOMEM;
  }
  memset(*m, 0, sizeof(struct enet_ws_event_omsg));
  (*m)->opcode = opcode;
  (*m)->rsv = rsv;
  (*m)->type = ENET_WS_FRAGMENTED;
  (*m)->source = source;
  (*m)->read_callback = read_callback;
  return 0;
}

static void enet_ws_event_omsg_free(struct enet_ws_event_omsg *m)
{
  if(!m) {
    return;
  }
  enet_free(m->data);
  enet_free(m);
}

static uint8_t* enet_ws_event_flatten_queue(struct enet_ws_queue *queue, size_t len)
{
  if(len == 0) {
    return NULL;
  } else {
    size_t off = 0;
    uint8_t *buf = (uint8_t*)enet_malloc(len);
    if(!buf) {
      return NULL;
    }
    while(!enet_ws_queue_empty(queue)) {
      struct enet_ws_event_byte_chunk *chunk = (struct enet_ws_event_byte_chunk*)enet_ws_queue_top(queue);
      memcpy(buf+off, chunk->data, chunk->data_length);
      off += chunk->data_length;
      enet_ws_event_byte_chunk_free(chunk);
      enet_ws_queue_pop(queue);
      assert(off <= len);
    }
    assert(len == off);
    return buf;
  }
}

static int enet_ws_event_is_msg_queueable(enet_ws_event_context_ptr ctx)
{
  return ctx->write_enabled && (ctx->close_status & ENET_WS_CLOSE_QUEUED) == 0;
}

int enet_ws_event_queue_close(enet_ws_event_context_ptr ctx, uint16_t status_code,
                            const uint8_t *reason, size_t reason_length)
{
  if(!enet_ws_event_is_msg_queueable(ctx)) {
    return ENET_WS_ERR_NO_MORE_MSG;
  } else if(reason_length > 123) {
    return ENET_WS_ERR_INVALID_ARGUMENT;
  } else {
    uint8_t msg[128];
    size_t msg_length;
    struct enet_ws_event_msg arg;
    uint16_t ncode;
    int r;
    if(status_code == 0) {
      msg_length = 0;
    } else {
      ncode = htons(status_code);
      memcpy(msg, &ncode, 2);
      if(reason_length) {
        memcpy(msg+2, reason, reason_length);
      }
      msg_length = reason_length+2;
    }
    arg.opcode = ENET_WS_CONNECTION_CLOSE;
    arg.msg = msg;
    arg.msg_length = msg_length;
    r = enet_ws_event_queue_msg(ctx, &arg);
    if(r == 0) {
      ctx->close_status |= ENET_WS_CLOSE_QUEUED;
    }
    return r;
  }
}

static int enet_ws_event_queue_close_wrapper
(enet_ws_event_context_ptr ctx, uint16_t status_code,
 const uint8_t *reason, size_t reason_length)
{
  int r;
  ctx->read_enabled = 0;
  if((r = enet_ws_event_queue_close(ctx, status_code, reason, reason_length)) &&
     r != ENET_WS_ERR_NO_MORE_MSG) {
    return r;
  }
  return 0;
}

static int enet_ws_event_verify_rsv_bits(enet_ws_event_context_ptr ctx, uint8_t rsv)
{
  return ((rsv & ~ctx->allowed_rsv_bits) == 0);
}

int enet_ws_event_queue_msg(enet_ws_event_context_ptr ctx,
                          const struct enet_ws_event_msg *arg)
{
  return enet_ws_event_queue_msg_ex(ctx, arg, ENET_WS_RSV_NONE);
}

int enet_ws_event_queue_msg_ex(enet_ws_event_context_ptr ctx,
                              const struct enet_ws_event_msg *arg, uint8_t rsv)
{
  int r;
  struct enet_ws_event_omsg *omsg;
  if(!enet_ws_event_is_msg_queueable(ctx)) {
    return ENET_WS_ERR_NO_MORE_MSG;
  }
  /* RSV1 is not allowed for control frames */
  if((ENET_WS_IS_CTRL_FRAME(arg->opcode) &&
      (arg->msg_length > 125 || ENET_WS_GET_RSV1(rsv)))
        || !enet_ws_event_verify_rsv_bits(ctx, rsv)) {
    return ENET_WS_ERR_INVALID_ARGUMENT;
  }
  if((r = enet_ws_event_omsg_non_fragmented_init
      (&omsg, arg->opcode, rsv, arg->msg, arg->msg_length)) != 0) {
    return r;
  }
  if(ENET_WS_IS_CTRL_FRAME(arg->opcode)) {
    if((r = enet_ws_queue_push(ctx->send_ctrl_queue, omsg)) != 0) {
      return r;
    }
  } else {
    if((r = enet_ws_queue_push(ctx->send_queue, omsg)) != 0) {
      return r;
    }
  }
  ++ctx->queued_msg_count;
  ctx->queued_msg_length += arg->msg_length;
  return 0;
}

int enet_ws_event_queue_fragmented_msg
(enet_ws_event_context_ptr ctx, const struct enet_ws_event_fragmented_msg *arg)
{
  return enet_ws_event_queue_fragmented_msg_ex(ctx, arg, ENET_WS_RSV_NONE);
}

int enet_ws_event_queue_fragmented_msg_ex(enet_ws_event_context_ptr ctx,
    const struct enet_ws_event_fragmented_msg *arg, uint8_t rsv)
{
  int r;
  struct enet_ws_event_omsg *omsg;
  if(!enet_ws_event_is_msg_queueable(ctx)) {
    return ENET_WS_ERR_NO_MORE_MSG;
  }
  if(ENET_WS_IS_CTRL_FRAME(arg->opcode) ||
     !enet_ws_event_verify_rsv_bits(ctx, rsv)) {
    return ENET_WS_ERR_INVALID_ARGUMENT;
  }
  if((r = enet_ws_event_omsg_fragmented_init
      (&omsg, arg->opcode, rsv, arg->source, arg->read_callback)) != 0) {
    return r;
  }
  if((r = enet_ws_queue_push(ctx->send_queue, omsg)) != 0) {
    return r;
  }
  ++ctx->queued_msg_count;
  return 0;
}

void enet_ws_event_config_set_callbacks
(enet_ws_event_context_ptr ctx, const struct enet_ws_event_callbacks *callbacks)
{
  ctx->callbacks = *callbacks;
}

static int enet_ws_event_context_init
(enet_ws_event_context_ptr *ctx,
 const struct enet_ws_event_callbacks *callbacks,
 void *user_data)
{
  int i, r;
  struct enet_ws_frame_callbacks frame_callbacks = {
    enet_ws_event_frame_send_callback,
    enet_ws_event_frame_recv_callback,
    enet_ws_event_frame_genmask_callback
  };
  *ctx = (enet_ws_event_context_ptr)enet_malloc(sizeof(struct enet_ws_event_context));
  if(!*ctx) {
    return ENET_WS_ERR_NOMEM;
  }
  memset(*ctx, 0, sizeof(struct enet_ws_event_context));
  enet_ws_event_config_set_callbacks(*ctx, callbacks);
  (*ctx)->user_data = user_data;
  (*ctx)->frame_user_data.ctx = *ctx;
  (*ctx)->frame_user_data.user_data = user_data;
  if((r = enet_ws_frame_context_init(&(*ctx)->frame_ctx, &frame_callbacks,
                                   &(*ctx)->frame_user_data)) != 0) {
    enet_ws_event_context_free(*ctx);
    return r;
  }
  (*ctx)->read_enabled = (*ctx)->write_enabled = 1;
  (*ctx)->send_queue = enet_ws_queue_new();
  if(!(*ctx)->send_queue) {
    enet_ws_event_context_free(*ctx);
    return ENET_WS_ERR_NOMEM;
  }
  (*ctx)->send_ctrl_queue = enet_ws_queue_new();
  if(!(*ctx)->send_ctrl_queue) {
    enet_ws_event_context_free(*ctx);
    return ENET_WS_ERR_NOMEM;
  }
  (*ctx)->queued_msg_count = 0;
  (*ctx)->queued_msg_length = 0;
  for(i = 0; i < 2; ++i) {
    enet_ws_event_imsg_reset(&(*ctx)->imsgs[i]);
    (*ctx)->imsgs[i].chunks = enet_ws_queue_new();
    if(!(*ctx)->imsgs[i].chunks) {
      enet_ws_event_context_free(*ctx);
      return ENET_WS_ERR_NOMEM;
    }
  }
  (*ctx)->imsg = &(*ctx)->imsgs[0];
  (*ctx)->obufmark = (*ctx)->obuflimit = (*ctx)->obuf;
  (*ctx)->status_code_sent = ENET_WS_CODE_ABNORMAL_CLOSURE;
  (*ctx)->status_code_recv = ENET_WS_CODE_ABNORMAL_CLOSURE;
  (*ctx)->max_recv_msg_length = (1u << 31)-1;
  return 0;
}

int enet_ws_event_context_server_init
(enet_ws_event_context_ptr *ctx,
 const struct enet_ws_event_callbacks *callbacks,
 void *user_data)
{
  int r;
  if((r = enet_ws_event_context_init(ctx, callbacks, user_data)) != 0) {
    return r;
  }
  (*ctx)->server = 1;
  return 0;
}

int enet_ws_event_context_client_init
(enet_ws_event_context_ptr *ctx,
 const struct enet_ws_event_callbacks *callbacks,
 void *user_data)
{
  int r;
  if((r = enet_ws_event_context_init(ctx, callbacks, user_data)) != 0) {
    return r;
  }
  (*ctx)->server = 0;
  return 0;
}

void enet_ws_event_context_free(enet_ws_event_context_ptr ctx)
{
  int i;
  if(!ctx) {
    return;
  }
  for(i = 0; i < 2; ++i) {
    enet_ws_event_imsg_chunks_free(&ctx->imsgs[i]);
    enet_ws_queue_free(ctx->imsgs[i].chunks);
  }
  if(ctx->send_queue) {
    while(!enet_ws_queue_empty(ctx->send_queue)) {
      enet_ws_event_omsg_free((struct enet_ws_event_omsg*)enet_ws_queue_top(ctx->send_queue));
      enet_ws_queue_pop(ctx->send_queue);
    }
    enet_ws_queue_free(ctx->send_queue);
  }
  if(ctx->send_ctrl_queue) {
    while(!enet_ws_queue_empty(ctx->send_ctrl_queue)) {
      enet_ws_event_omsg_free((struct enet_ws_event_omsg*)enet_ws_queue_top(ctx->send_ctrl_queue));
      enet_ws_queue_pop(ctx->send_ctrl_queue);
    }
    enet_ws_queue_free(ctx->send_ctrl_queue);
  }
  enet_ws_frame_context_free(ctx->frame_ctx);
  enet_ws_event_omsg_free(ctx->omsg);
  enet_free(ctx);
}

static void enet_ws_event_call_on_frame_recv_start_callback
(enet_ws_event_context_ptr ctx, const struct enet_ws_frame_iocb *iocb)
{
  if(ctx->callbacks.on_frame_recv_start_callback) {
    struct enet_ws_event_on_frame_recv_start_arg arg;
    arg.fin = iocb->fin;
    arg.rsv = iocb->rsv;
    arg.opcode = iocb->opcode;
    arg.payload_length = iocb->payload_length;
    ctx->callbacks.on_frame_recv_start_callback(ctx, &arg, ctx->user_data);
  }
}

static void enet_ws_event_call_on_frame_recv_chunk_callback
(enet_ws_event_context_ptr ctx, const struct enet_ws_frame_iocb *iocb)
{
  if(ctx->callbacks.on_frame_recv_chunk_callback) {
    struct enet_ws_event_on_frame_recv_chunk_arg arg;
    arg.data = iocb->data;
    arg.data_length = iocb->data_length;
    ctx->callbacks.on_frame_recv_chunk_callback(ctx, &arg, ctx->user_data);
  }
}

static void enet_ws_event_call_on_frame_recv_end_callback
(enet_ws_event_context_ptr ctx)
{
  if(ctx->callbacks.on_frame_recv_end_callback) {
    ctx->callbacks.on_frame_recv_end_callback(ctx, ctx->user_data);
  }
}

static int enet_ws_event_is_valid_status_code(uint16_t status_code)
{
  return (1000 <= status_code && status_code <= 1011 &&
          status_code != 1004 && status_code != 1005 && status_code != 1006) ||
    (3000 <= status_code && status_code <= 4999);
}

static int enet_ws_event_config_get_no_buffering(enet_ws_event_context_ptr ctx)
{
  return (ctx->config & ENET_WS_CONFIG_NO_BUFFERING) > 0;
}

int enet_ws_event_recv(enet_ws_event_context_ptr ctx)
{
  struct enet_ws_frame_iocb iocb;
  ENetSSize r;
  while(ctx->read_enabled) {
    memset(&iocb, 0, sizeof(iocb));
    r = enet_ws_frame_recv(ctx->frame_ctx, &iocb);
    if(r >= 0) {
      int new_frame = 0;
      /* RSV1 is not allowed on control and continuation frames */
      if((!enet_ws_event_verify_rsv_bits(ctx, iocb.rsv)) ||
          (ENET_WS_GET_RSV1(iocb.rsv) && (ENET_WS_IS_CTRL_FRAME(iocb.opcode) ||
             iocb.opcode == ENET_WS_CONTINUATION_FRAME)) ||
               (ctx->server && !iocb.mask) || (!ctx->server && iocb.mask)) {
        if((r = enet_ws_event_queue_close_wrapper
            (ctx, ENET_WS_CODE_PROTOCOL_ERROR, NULL, 0)) != 0) {
          return r;
        }
        break;
      }
      if(ctx->imsg->opcode == 0xffu) {
        if(iocb.opcode == ENET_WS_TEXT_FRAME ||
           iocb.opcode == ENET_WS_BINARY_FRAME ||
           iocb.opcode == ENET_WS_CONNECTION_CLOSE ||
           iocb.opcode == ENET_WS_PING ||
           iocb.opcode == ENET_WS_PONG) {
          enet_ws_event_imsg_set(ctx->imsg, iocb.fin, iocb.rsv, iocb.opcode);
          new_frame = 1;
        } else {
          if((r = enet_ws_event_queue_close_wrapper
              (ctx, ENET_WS_CODE_PROTOCOL_ERROR, NULL, 0)) != 0) {
            return r;
          }
          break;
        }
      } else if(ctx->ipayloadlen == 0 && ctx->ipayloadoff == 0) {
        if(iocb.opcode == ENET_WS_CONTINUATION_FRAME) {
          ctx->imsg->fin = iocb.fin;
        } else if(iocb.opcode == ENET_WS_CONNECTION_CLOSE ||
                  iocb.opcode == ENET_WS_PING ||
                  iocb.opcode == ENET_WS_PONG) {
          ctx->imsg = &ctx->imsgs[1];
          enet_ws_event_imsg_set(ctx->imsg, iocb.fin, iocb.rsv, iocb.opcode);
        } else {
          if((r = enet_ws_event_queue_close_wrapper
              (ctx, ENET_WS_CODE_PROTOCOL_ERROR, NULL, 0)) != 0) {
            return r;
          }
          break;
        }
        new_frame = 1;
      }
      if(new_frame) {
        if(ctx->imsg->msg_length+iocb.payload_length >
           ctx->max_recv_msg_length) {
          if((r = enet_ws_event_queue_close_wrapper
              (ctx, ENET_WS_CODE_MESSAGE_TOO_BIG, NULL, 0)) != 0) {
            return r;
          }
          break;
        }
        ctx->ipayloadlen = iocb.payload_length;
        enet_ws_event_call_on_frame_recv_start_callback(ctx, &iocb);
        if(!enet_ws_event_config_get_no_buffering(ctx) ||
           ENET_WS_IS_CTRL_FRAME(iocb.opcode)) {
          if((r = enet_ws_event_imsg_append_chunk(ctx->imsg,
                                                iocb.payload_length)) != 0) {
            ctx->read_enabled = 0;
            return r;
          }
        }
      }
      /* If RSV1 bit is set then it is too early for utf-8 validation */
      if((!ENET_WS_GET_RSV1(ctx->imsg->rsv) &&
          ctx->imsg->opcode == ENET_WS_TEXT_FRAME) ||
            ctx->imsg->opcode == ENET_WS_CONNECTION_CLOSE) {
        size_t i;
        if(ctx->imsg->opcode == ENET_WS_CONNECTION_CLOSE) {
          i = 2;
        } else {
          i = 0;
        }
        for(; i < iocb.data_length; ++i) {
          uint32_t codep;
          if(decode(&ctx->imsg->utf8state, &codep,
                    iocb.data[i]) == UTF8_REJECT) {
            if((r = enet_ws_event_queue_close_wrapper
                (ctx, ENET_WS_CODE_INVALID_FRAME_PAYLOAD_DATA, NULL, 0)) != 0) {
              return r;
            }
            break;
          }
        }
      }
      if(ctx->imsg->utf8state == UTF8_REJECT) {
        break;
      }
      enet_ws_event_call_on_frame_recv_chunk_callback(ctx, &iocb);
      if(iocb.data_length > 0) {
        if(!enet_ws_event_config_get_no_buffering(ctx) ||
           ENET_WS_IS_CTRL_FRAME(iocb.opcode)) {
          struct enet_ws_event_byte_chunk *chunk;
          chunk = (struct enet_ws_event_byte_chunk*)enet_ws_queue_tail(ctx->imsg->chunks);
          enet_ws_event_byte_chunk_copy(chunk, ctx->ipayloadoff,
                                      iocb.data, iocb.data_length);
        }
        ctx->ipayloadoff += iocb.data_length;
      }
      if(ctx->ipayloadoff == ctx->ipayloadlen) {
        if(ctx->imsg->fin &&
           (ctx->imsg->opcode == ENET_WS_TEXT_FRAME ||
            ctx->imsg->opcode == ENET_WS_CONNECTION_CLOSE) &&
           ctx->imsg->utf8state != UTF8_ACCEPT) {
          if((r = enet_ws_event_queue_close_wrapper
              (ctx, ENET_WS_CODE_INVALID_FRAME_PAYLOAD_DATA, NULL, 0)) != 0) {
            return r;
          }
          break;
        }
        enet_ws_event_call_on_frame_recv_end_callback(ctx);
        if(ctx->imsg->fin) {
          if(ctx->callbacks.on_msg_recv_callback ||
             ctx->imsg->opcode == ENET_WS_CONNECTION_CLOSE ||
             ctx->imsg->opcode == ENET_WS_PING) {
            struct enet_ws_event_on_msg_recv_arg arg;
            uint16_t status_code = 0;
            uint8_t *msg = NULL;
            size_t msg_length = 0;
            if(!enet_ws_event_config_get_no_buffering(ctx) ||
               ENET_WS_IS_CTRL_FRAME(iocb.opcode)) {
              msg = enet_ws_event_flatten_queue(ctx->imsg->chunks,
                                              ctx->imsg->msg_length);
              if(ctx->imsg->msg_length && !msg) {
                ctx->read_enabled = 0;
                return ENET_WS_ERR_NOMEM;
              }
              msg_length = ctx->imsg->msg_length;
            }
            if(ctx->imsg->opcode == ENET_WS_CONNECTION_CLOSE) {
              const uint8_t *reason;
              size_t reason_length;
              if(ctx->imsg->msg_length >= 2) {
                memcpy(&status_code, msg, 2);
                status_code = ntohs(status_code);
                if(!enet_ws_event_is_valid_status_code(status_code)) {
                  enet_free(msg);
                  if((r = enet_ws_event_queue_close_wrapper
                      (ctx, ENET_WS_CODE_PROTOCOL_ERROR, NULL, 0)) != 0) {
                    return r;
                  }
                  break;
                }
                reason = msg+2;
                reason_length = ctx->imsg->msg_length-2;
              } else {
                reason = NULL;
                reason_length = 0;
              }
              ctx->close_status |= ENET_WS_CLOSE_RECEIVED;
              ctx->status_code_recv =
                status_code == 0 ? (uint16_t)ENET_WS_CODE_NO_STATUS_RCVD : status_code;
              if((r = enet_ws_event_queue_close_wrapper
                  (ctx, status_code, reason, reason_length)) != 0) {
                enet_free(msg);
                return r;
              }
            } else if(ctx->imsg->opcode == ENET_WS_PING) {
              struct enet_ws_event_msg pong_arg;
              pong_arg.opcode = ENET_WS_PONG;
              pong_arg.msg = msg;
              pong_arg.msg_length = ctx->imsg->msg_length;
              if((r = enet_ws_event_queue_msg(ctx, &pong_arg)) &&
                 r != ENET_WS_ERR_NO_MORE_MSG) {
                ctx->read_enabled = 0;
                enet_free(msg);
                return r;
              }
            }
            if(ctx->callbacks.on_msg_recv_callback) {
              arg.rsv = ctx->imsg->rsv;
              arg.opcode = ctx->imsg->opcode;
              arg.msg = msg;
              arg.msg_length = msg_length;
              arg.status_code = status_code;
              ctx->error = 0;
              ctx->callbacks.on_msg_recv_callback(ctx, &arg, ctx->user_data);
            }
            enet_free(msg);
          }
          enet_ws_event_imsg_reset(ctx->imsg);
          if(ctx->imsg == &ctx->imsgs[1]) {
            ctx->imsg = &ctx->imsgs[0];
          }
        }
        ctx->ipayloadlen = ctx->ipayloadoff = 0;
      }
    } else {
      if(r == ENET_WS_ERR_PROTO) {
        /* Frame-level violation (e.g. control frame over 125 bytes, fragmented control frame): answer with 1002,
           exactly like the message-level violations above, instead of a close frame with no status. */
        if((r = enet_ws_event_queue_close_wrapper
            (ctx, ENET_WS_CODE_PROTOCOL_ERROR, NULL, 0)) != 0) {
          return r;
        }
        break;
      }
      if(r != ENET_WS_ERR_WANT_READ ||
         (ctx->error != ENET_WS_ERR_WOULDBLOCK && ctx->error != 0)) {
        if((r = enet_ws_event_queue_close_wrapper(ctx, 0, NULL, 0)) != 0) {
          return r;
        }
        return ENET_WS_ERR_CALLBACK_FAILURE;
      }
      break;
    }
  }
  return 0;
}

static void enet_ws_event_on_non_fragmented_msg_popped
(enet_ws_event_context_ptr ctx)
{
  ctx->omsg->fin = 1;
  ctx->opayloadlen = ctx->omsg->data_length;
  ctx->opayloadoff = 0;
}

static struct enet_ws_event_omsg* enet_ws_event_send_ctrl_queue_pop
(enet_ws_event_context_ptr ctx)
{
  /*
   * If Close control frame is queued, we don't send any control frame
   * other than Close.
   */
  if(ctx->close_status & ENET_WS_CLOSE_QUEUED) {
    while(!enet_ws_queue_empty(ctx->send_ctrl_queue)) {
      struct enet_ws_event_omsg *msg = (struct enet_ws_event_omsg*)enet_ws_queue_top(ctx->send_ctrl_queue);
      enet_ws_queue_pop(ctx->send_ctrl_queue);
      if(msg->opcode == ENET_WS_CONNECTION_CLOSE) {
        return msg;
      } else {
        enet_ws_event_omsg_free(msg);
      }
    }
    return NULL;
  } else {
    struct enet_ws_event_omsg *msg = (struct enet_ws_event_omsg*)enet_ws_queue_top(ctx->send_ctrl_queue);
    enet_ws_queue_pop(ctx->send_ctrl_queue);
    return msg;
  }
}

int enet_ws_event_send(enet_ws_event_context_ptr ctx)
{
  struct enet_ws_frame_iocb iocb;
  ENetSSize r;
  while(ctx->write_enabled &&
        (!enet_ws_queue_empty(ctx->send_queue) ||
         !enet_ws_queue_empty(ctx->send_ctrl_queue) || ctx->omsg)) {
    if(!ctx->omsg) {
      if(enet_ws_queue_empty(ctx->send_ctrl_queue)) {
        ctx->omsg = (struct enet_ws_event_omsg*)enet_ws_queue_top(ctx->send_queue);
        enet_ws_queue_pop(ctx->send_queue);
      } else {
        ctx->omsg = enet_ws_event_send_ctrl_queue_pop(ctx);
        if(ctx->omsg == NULL) {
          break;
        }
      }
      if(ctx->omsg->type == ENET_WS_NON_FRAGMENTED) {
        enet_ws_event_on_non_fragmented_msg_popped(ctx);
      }
    } else if(!ENET_WS_IS_CTRL_FRAME(ctx->omsg->opcode) &&
              ctx->frame_ctx->ostate == ENET_WS_FRAME_PREP_HEADER &&
              !enet_ws_queue_empty(ctx->send_ctrl_queue)) {
      if((r = enet_ws_queue_push_front(ctx->send_queue, ctx->omsg)) != 0) {
        ctx->write_enabled = 0;
        return r;
      }
      ctx->omsg = enet_ws_event_send_ctrl_queue_pop(ctx);
      if(ctx->omsg == NULL) {
        break;
      }
      /* ctrl message has ENET_WS_NON_FRAGMENTED */
      enet_ws_event_on_non_fragmented_msg_popped(ctx);
    }
    if(ctx->omsg->type == ENET_WS_NON_FRAGMENTED) {
      memset(&iocb, 0, sizeof(iocb));
      iocb.fin = 1;
      iocb.opcode = ctx->omsg->opcode;
      iocb.rsv = ctx->omsg->rsv;
      iocb.mask = ctx->server^1;
      iocb.data = ctx->omsg->data+ctx->opayloadoff;
      iocb.data_length = ctx->opayloadlen-ctx->opayloadoff;
      iocb.payload_length = ctx->opayloadlen;
      r = enet_ws_frame_send(ctx->frame_ctx, &iocb);
      if(r >= 0) {
        ctx->opayloadoff += r;
        if(ctx->opayloadoff == ctx->opayloadlen) {
          --ctx->queued_msg_count;
          ctx->queued_msg_length -= ctx->omsg->data_length;
          if(ctx->omsg->opcode == ENET_WS_CONNECTION_CLOSE) {
            uint16_t status_code = 0;
            ctx->write_enabled = 0;
            ctx->close_status |= ENET_WS_CLOSE_SENT;
            if(ctx->omsg->data_length >= 2) {
              memcpy(&status_code, ctx->omsg->data, 2);
              status_code = ntohs(status_code);
            }
            ctx->status_code_sent =
              status_code == 0 ? (uint16_t)ENET_WS_CODE_NO_STATUS_RCVD : status_code;
          }
          enet_ws_event_omsg_free(ctx->omsg);
          ctx->omsg = NULL;
        } else {
          break;
        }
      } else {
        if(r != ENET_WS_ERR_WANT_WRITE ||
           (ctx->error != ENET_WS_ERR_WOULDBLOCK && ctx->error != 0)) {
          ctx->write_enabled = 0;
          return ENET_WS_ERR_CALLBACK_FAILURE;
        }
        break;
      }
    } else {
      if(ctx->omsg->fin == 0 && ctx->obuflimit == ctx->obufmark) {
        int eof = 0;
        r = ctx->omsg->read_callback(ctx, ctx->obuf, sizeof(ctx->obuf),
                                     &ctx->omsg->source,
                                     &eof, ctx->user_data);
        if(r == 0 && eof == 0) {
          break;
        } else if(r < 0) {
          ctx->write_enabled = 0;
          return ENET_WS_ERR_CALLBACK_FAILURE;
        }
        ctx->obuflimit = ctx->obuf+r;
        if(eof) {
          ctx->omsg->fin = 1;
        }
        ctx->opayloadlen = r;
        ctx->opayloadoff = 0;
      }
      memset(&iocb, 0, sizeof(iocb));
      iocb.fin = ctx->omsg->fin;
      iocb.opcode = ctx->omsg->opcode;
      iocb.rsv = ctx->omsg->rsv;
      iocb.mask = ctx->server ? 0 : 1;
      iocb.data = ctx->obufmark;
      iocb.data_length = ctx->obuflimit-ctx->obufmark;
      iocb.payload_length = ctx->opayloadlen;
      r = enet_ws_frame_send(ctx->frame_ctx, &iocb);
      if(r >= 0) {
        ctx->obufmark += r;
        if(ctx->obufmark == ctx->obuflimit) {
          ctx->obufmark = ctx->obuflimit = ctx->obuf;
          if(ctx->omsg->fin) {
            --ctx->queued_msg_count;
            enet_ws_event_omsg_free(ctx->omsg);
            ctx->omsg = NULL;
          } else {
            ctx->omsg->opcode = ENET_WS_CONTINUATION_FRAME;
            /* RSV1 is not set on continuation frames */
            ctx->omsg->rsv = ctx->omsg->rsv & ~ENET_WS_RSV1_BIT;
          }
        } else {
          break;
        }
      } else {
        if(r != ENET_WS_ERR_WANT_WRITE ||
           (ctx->error != ENET_WS_ERR_WOULDBLOCK &&
            ctx->error != 0)) {
          ctx->write_enabled = 0;
          return ENET_WS_ERR_CALLBACK_FAILURE;
        }
        break;
      }
    }
  }
  return 0;
}

void enet_ws_event_set_error(enet_ws_event_context_ptr ctx, int val) { ctx->error = val; }
int enet_ws_event_want_read(enet_ws_event_context_ptr ctx) { return ctx->read_enabled; }
int enet_ws_event_want_write(enet_ws_event_context_ptr ctx) { return ctx->write_enabled && (!enet_ws_queue_empty(ctx->send_queue) || !enet_ws_queue_empty(ctx->send_ctrl_queue) || ctx->omsg); }
void enet_ws_event_shutdown_read(enet_ws_event_context_ptr ctx) { ctx->read_enabled = 0; }
void enet_ws_event_shutdown_write(enet_ws_event_context_ptr ctx) { ctx->write_enabled = 0; }
int enet_ws_event_get_read_enabled(enet_ws_event_context_ptr ctx) { return ctx->read_enabled; }
int enet_ws_event_get_write_enabled(enet_ws_event_context_ptr ctx) { return ctx->write_enabled; }
int enet_ws_event_get_close_received(enet_ws_event_context_ptr ctx) { return (ctx->close_status & ENET_WS_CLOSE_RECEIVED) > 0; }
int enet_ws_event_get_close_sent(enet_ws_event_context_ptr ctx) { return (ctx->close_status & ENET_WS_CLOSE_SENT) > 0; }
void enet_ws_event_config_set_allowed_rsv_bits(enet_ws_event_context_ptr ctx, uint8_t rsv) { ctx->allowed_rsv_bits = rsv & ENET_WS_RSV1_BIT; } /* We currently only allow ENET_WS_RSV1_BIT or ENET_WS_RSV_NONE */
void enet_ws_event_config_set_no_buffering(enet_ws_event_context_ptr ctx, int val) {
  if(val) { ctx->config |= ENET_WS_CONFIG_NO_BUFFERING; } 
  else { ctx->config &= ~ENET_WS_CONFIG_NO_BUFFERING; }
}
void enet_ws_event_config_set_max_recv_msg_length(enet_ws_event_context_ptr ctx, uint64_t val) { ctx->max_recv_msg_length = val; }
uint16_t enet_ws_event_get_status_code_received(enet_ws_event_context_ptr ctx) { return ctx->status_code_recv; }
uint16_t enet_ws_event_get_status_code_sent(enet_ws_event_context_ptr ctx) { return ctx->status_code_sent; }
size_t enet_ws_event_get_queued_msg_count(enet_ws_event_context_ptr ctx) { return ctx->queued_msg_count; }
size_t enet_ws_event_get_queued_msg_length(enet_ws_event_context_ptr ctx) { return ctx->queued_msg_length; }

struct enet_ws_queue* enet_ws_queue_new(void) {
  struct enet_ws_queue *queue = (struct enet_ws_queue*)enet_malloc(sizeof(struct enet_ws_queue));
  if(!queue) {
    return NULL;
  }
  queue->top = queue->tail = NULL;
  return queue;
}

void enet_ws_queue_free(struct enet_ws_queue *queue) {
  if(!queue) { return; } 
  else {
    struct enet_ws_queue_cell *p = queue->top;
    while(p) {
      struct enet_ws_queue_cell *next = p->next;
      enet_free(p);
      p = next;
    }
    enet_free(queue);
  }
}

int enet_ws_queue_push(struct enet_ws_queue *queue, void *data) {
  struct enet_ws_queue_cell *new_cell = (struct enet_ws_queue_cell*)enet_malloc(sizeof(struct enet_ws_queue_cell));
  if(!new_cell) {
    return ENET_WS_ERR_NOMEM;
  }
  new_cell->data = data;
  new_cell->next = NULL;
  if(queue->tail) {
    queue->tail->next = new_cell;
    queue->tail = new_cell;
  } 
  else { queue->top = queue->tail = new_cell; }
  return 0;
}

int enet_ws_queue_push_front(struct enet_ws_queue *queue, void *data) {
  struct enet_ws_queue_cell *new_cell = (struct enet_ws_queue_cell*)enet_malloc(sizeof(struct enet_ws_queue_cell));
  if(!new_cell) { return ENET_WS_ERR_NOMEM; }
  new_cell->data = data;
  new_cell->next = queue->top;
  queue->top = new_cell;
  if(!queue->tail) { queue->tail = queue->top; }
  return 0;
}

void enet_ws_queue_pop(struct enet_ws_queue *queue) {
  struct enet_ws_queue_cell *top = queue->top;
  assert(top);
  queue->top = top->next;
  if(top == queue->tail) { queue->tail = NULL; }
  enet_free(top);
}

void* enet_ws_queue_top(struct enet_ws_queue *queue) { assert(queue->top); return queue->top->data; }

void* enet_ws_queue_tail(struct enet_ws_queue *queue) { assert(queue->tail); return queue->tail->data; }

int enet_ws_queue_empty(struct enet_ws_queue *queue) { return queue->top == NULL; }


int enet_ws_frame_context_init(enet_ws_frame_context_ptr *ctx, const struct enet_ws_frame_callbacks *callbacks, void *user_data) {
  *ctx = (enet_ws_frame_context_ptr)enet_malloc(sizeof(struct enet_ws_frame_context));
  if(*ctx == NULL) { return -1; }
  memset(*ctx, 0, sizeof(struct enet_ws_frame_context));
  (*ctx)->istate = ENET_WS_FRAME_RECV_HEADER1;
  (*ctx)->ireqread = 2;
  (*ctx)->ostate = ENET_WS_FRAME_PREP_HEADER;
  (*ctx)->user_data = user_data;
  (*ctx)->ibufmark = (*ctx)->ibuflimit = (*ctx)->ibuf;
  (*ctx)->callbacks = *callbacks;
  return 0;
}

void enet_ws_frame_context_free(enet_ws_frame_context_ptr ctx) { enet_free(ctx); }

ENetSSize enet_ws_frame_send(enet_ws_frame_context_ptr ctx, struct enet_ws_frame_iocb *iocb) {
  if(iocb->data_length > iocb->payload_length) {
    return ENET_WS_ERR_INVALID_ARGUMENT;
  }
  if(ctx->ostate == ENET_WS_FRAME_PREP_HEADER) {
    uint8_t *hdptr = ctx->oheader;
    memset(ctx->oheader, 0, sizeof(ctx->oheader));
    *hdptr |= (iocb->fin << 7) & 0x80u;
    *hdptr |= (iocb->rsv << 4) & 0x70u;
    *hdptr |= iocb->opcode & 0xfu;
    ++hdptr;
    *hdptr |= (iocb->mask << 7) & 0x80u;
    if(ENET_WS_IS_CTRL_FRAME(iocb->opcode) && iocb->payload_length > 125) { return ENET_WS_ERR_INVALID_ARGUMENT; }
    if(iocb->payload_length < 126) {
      *hdptr |= iocb->payload_length;
      ++hdptr;
    } else if(iocb->payload_length < (1 << 16)) {
      uint16_t len = htons(iocb->payload_length);
      *hdptr |= 126;
      ++hdptr;
      memcpy(hdptr, &len, 2);
      hdptr += 2;
    } else if(iocb->payload_length < (1ull << 63)) {
      uint64_t len = ENET_WS_HTON64(iocb->payload_length);
      *hdptr |= 127;
      ++hdptr;
      memcpy(hdptr, &len, 8);
      hdptr += 8;
    } else { return ENET_WS_ERR_INVALID_ARGUMENT; } /* Too large payload length */
    if(iocb->mask) {
      if(ctx->callbacks.genmask_callback(ctx->omaskkey, 4, ctx->user_data) != 0) {
        return ENET_WS_ERR_INVALID_CALLBACK;
      } else {
        ctx->omask = 1;
        memcpy(hdptr, ctx->omaskkey, 4);
        hdptr += 4;
      }
    }
    ctx->ostate = ENET_WS_FRAME_SEND_HEADER;
    ctx->oheadermark = ctx->oheader;
    ctx->oheaderlimit = hdptr;
    ctx->opayloadlen = iocb->payload_length;
    ctx->opayloadoff = 0;
  }
  if(ctx->ostate == ENET_WS_FRAME_SEND_HEADER) {
    ptrdiff_t len = ctx->oheaderlimit-ctx->oheadermark;
    ENetSSize r;
    int flags = 0;
    if(iocb->data_length > 0) {
      flags |= ENET_WS_MSG_MORE;
    };
    r = ctx->callbacks.send_callback(ctx->oheadermark, len, flags,
                                     ctx->user_data);
    if(r > 0) {
      if(r > len) {
        return ENET_WS_ERR_INVALID_CALLBACK;
      } else {
        ctx->oheadermark += r;
        if(ctx->oheadermark == ctx->oheaderlimit) {
          ctx->ostate = ENET_WS_FRAME_SEND_PAYLOAD;
        } 
        else { return ENET_WS_ERR_WANT_WRITE; }
      }
    } 
    else { return ENET_WS_ERR_WANT_WRITE; }
  }
  if(ctx->ostate == ENET_WS_FRAME_SEND_PAYLOAD) {
    size_t totallen = 0;
    if(iocb->data_length > 0) {
      if(ctx->omask) {
        uint8_t temp[4096];
        const uint8_t *datamark = iocb->data,
          *datalimit = iocb->data+iocb->data_length;
        while(datamark < datalimit) {
          size_t datalen = datalimit - datamark;
          const uint8_t *writelimit = datamark+
            enet_ws_min(sizeof(temp), datalen);
          size_t writelen = writelimit-datamark;
          ENetSSize r;
          size_t i;
          for(i = 0; i < writelen; ++i) {
            temp[i] = datamark[i]^ctx->omaskkey[(ctx->opayloadoff+i)%4];
          }
          r = ctx->callbacks.send_callback(temp, writelen, 0, ctx->user_data);
          if(r > 0) {
            if((size_t)r > writelen) {
              return ENET_WS_ERR_INVALID_CALLBACK;
            } else {
              datamark += r;
              ctx->opayloadoff += r;
              totallen += r;
            }
          } else {
            if(totallen > 0) {
              break;
            } else {
              return ENET_WS_ERR_WANT_WRITE;
            }
          }
        }
      } else {
        ENetSSize r;
        r = ctx->callbacks.send_callback(iocb->data, iocb->data_length, 0,
                                         ctx->user_data);
        if(r > 0) {
          if((size_t)r > iocb->data_length) {
            return ENET_WS_ERR_INVALID_CALLBACK;
          } else {
            ctx->opayloadoff += r;
            totallen = r;
          }
        } else {
          return ENET_WS_ERR_WANT_WRITE;
        }
      }
    }
    if(ctx->opayloadoff == ctx->opayloadlen) {
      ctx->ostate = ENET_WS_FRAME_PREP_HEADER;
    }
    return totallen;
  }
  return ENET_WS_ERR_INVALID_ARGUMENT;
}

static void enet_ws_shift_ibuf(enet_ws_frame_context_ptr ctx) {
  ptrdiff_t len = ctx->ibuflimit-ctx->ibufmark;
  memmove(ctx->ibuf, ctx->ibufmark, len);
  ctx->ibuflimit = ctx->ibuf+len;
  ctx->ibufmark = ctx->ibuf;
}

static ENetSSize enet_ws_recv(enet_ws_frame_context_ptr ctx)
{
  ENetSSize r;
  if(ctx->ibufmark != ctx->ibuf) {
    enet_ws_shift_ibuf(ctx);
  }
  r = ctx->callbacks.recv_callback
    (ctx->ibuflimit, ctx->ibuf+sizeof(ctx->ibuf)-ctx->ibuflimit,
     0, ctx->user_data);
  if(r > 0) {
    ctx->ibuflimit += r;
  } else {
    r = ENET_WS_ERR_WANT_READ;
  }
  return r;
}

ENetSSize enet_ws_frame_recv(enet_ws_frame_context_ptr ctx, struct enet_ws_frame_iocb *iocb)
{
  ENetSSize r;
  if(ctx->istate == ENET_WS_FRAME_RECV_HEADER1) {
    uint8_t fin, opcode, rsv, payloadlen;
    if(ENET_WS_AVAIL_IBUF(ctx) < ctx->ireqread) {
      if((r = enet_ws_recv(ctx)) <= 0) {
        return r;
      }
    }
    if(ENET_WS_AVAIL_IBUF(ctx) < ctx->ireqread) {
      return ENET_WS_ERR_WANT_READ;
    }
    fin = (ctx->ibufmark[0] >> 7) & 1;
    rsv = (ctx->ibufmark[0] >> 4) & 7;
    opcode = ctx->ibufmark[0] & 0xfu;
    ctx->iom.opcode = opcode;
    ctx->iom.fin = fin;
    ctx->iom.rsv = rsv;
    ++ctx->ibufmark;
    ctx->imask = (ctx->ibufmark[0] >> 7) & 1;
    payloadlen = ctx->ibufmark[0] & 0x7fu;
    ++ctx->ibufmark;
    if(ENET_WS_IS_CTRL_FRAME(opcode) && (payloadlen > 125 || !fin)) {
      return ENET_WS_ERR_PROTO;
    }
    if(payloadlen == 126) {
      ctx->istate = ENET_WS_FRAME_RECV_EXT_PAYLOADLEN;
      ctx->ireqread = 2;
    } else if(payloadlen == 127) {
      ctx->istate = ENET_WS_FRAME_RECV_EXT_PAYLOADLEN;
      ctx->ireqread = 8;
    } else {
      ctx->ipayloadlen = payloadlen;
      ctx->ipayloadoff = 0;
      if(ctx->imask) {
        ctx->istate = ENET_WS_FRAME_RECV_MASKKEY;
        ctx->ireqread = 4;
      } else {
        ctx->istate = ENET_WS_FRAME_RECV_PAYLOAD;
      }
    }
  }
  if(ctx->istate == ENET_WS_FRAME_RECV_EXT_PAYLOADLEN) {
    if(ENET_WS_AVAIL_IBUF(ctx) < ctx->ireqread) {
      if((r = enet_ws_recv(ctx)) <= 0) {
        return r;
      }
      if(ENET_WS_AVAIL_IBUF(ctx) < ctx->ireqread) {
        return ENET_WS_ERR_WANT_READ;
      }
    }
    ctx->ipayloadlen = 0;
    ctx->ipayloadoff = 0;
    memcpy((uint8_t*)&ctx->ipayloadlen+(8-ctx->ireqread),
           ctx->ibufmark, ctx->ireqread);
    ctx->ipayloadlen = ENET_WS_NTOH64(ctx->ipayloadlen);
    ctx->ibufmark += ctx->ireqread;
    if(ctx->ireqread == 8) {
      if(ctx->ipayloadlen < (1 << 16) ||
         ctx->ipayloadlen & (1ull << 63)) {
        return ENET_WS_ERR_PROTO;
      }
    } else if(ctx->ipayloadlen < 126) {
      return ENET_WS_ERR_PROTO;
    }
    if(ctx->imask) {
      ctx->istate = ENET_WS_FRAME_RECV_MASKKEY;
      ctx->ireqread = 4;
    } else {
      ctx->istate = ENET_WS_FRAME_RECV_PAYLOAD;
    }
  }
  if(ctx->istate == ENET_WS_FRAME_RECV_MASKKEY) {
    if(ENET_WS_AVAIL_IBUF(ctx) < ctx->ireqread) {
      if((r = enet_ws_recv(ctx)) <= 0) {
        return r;
      }
      if(ENET_WS_AVAIL_IBUF(ctx) < ctx->ireqread) {
        return ENET_WS_ERR_WANT_READ;
      }
    }
    memcpy(ctx->imaskkey, ctx->ibufmark, 4);
    ctx->ibufmark += 4;
    ctx->istate = ENET_WS_FRAME_RECV_PAYLOAD;
  }
  if(ctx->istate == ENET_WS_FRAME_RECV_PAYLOAD) {
    uint8_t *readlimit, *readmark;
    uint64_t rempayloadlen = ctx->ipayloadlen-ctx->ipayloadoff;
    if(ENET_WS_AVAIL_IBUF(ctx) == 0 && rempayloadlen > 0) {
      if((r = enet_ws_recv(ctx)) <= 0) {
        return r;
      }
    }
    readmark = ctx->ibufmark;
    readlimit = ENET_WS_AVAIL_IBUF(ctx) < rempayloadlen ?
      ctx->ibuflimit : ctx->ibufmark+rempayloadlen;
    if(ctx->imask) {
      for(; ctx->ibufmark != readlimit;
          ++ctx->ibufmark, ++ctx->ipayloadoff) {
        ctx->ibufmark[0] ^= ctx->imaskkey[ctx->ipayloadoff % 4];
      }
    } else {
      ctx->ibufmark = readlimit;
      ctx->ipayloadoff += readlimit-readmark;
    }
    iocb->fin = ctx->iom.fin;
    iocb->rsv = ctx->iom.rsv;
    iocb->opcode = ctx->iom.opcode;
    iocb->payload_length = ctx->ipayloadlen;
    iocb->mask = ctx->imask;
    iocb->data = readmark;
    iocb->data_length = ctx->ibufmark-readmark;
    if(ctx->ipayloadlen == ctx->ipayloadoff) {
      ctx->istate = ENET_WS_FRAME_RECV_HEADER1;
      ctx->ireqread = 2;
    }
    return iocb->data_length;
  }
  return ENET_WS_ERR_INVALID_ARGUMENT;
}
