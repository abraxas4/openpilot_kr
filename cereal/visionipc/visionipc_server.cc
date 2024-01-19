#include <iostream>
#include <chrono>
#include <cassert>
#include <random>
#include <limits>

#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include "cereal/messaging/messaging.h"
#include "cereal/visionipc/ipc.h"
#include "cereal/visionipc/visionipc_server.h"
#include "cereal/logger/logger.h"

std::string get_endpoint_name(std::string name, VisionStreamType type){
  if (messaging_use_zmq()){
    assert(name == "camerad" || name == "navd");
    return std::to_string(9000 + static_cast<int>(type));
  } else {
    return "visionipc_" + name + "_" + std::to_string(type);
  }
}

VisionIpcServer::VisionIpcServer(std::string name, cl_device_id device_id, cl_context ctx) : name(name), device_id(device_id), ctx(ctx) {
  msg_ctx = Context::create();

  std::random_device rd("/dev/urandom");
  std::uniform_int_distribution<uint64_t> distribution(0, std::numeric_limits<uint64_t>::max());
  server_id = distribution(rd);
}

void VisionIpcServer::create_buffers(VisionStreamType type, size_t num_buffers, bool rgb, size_t width, size_t height){
  // TODO: assert that this type is not created yet
  assert(num_buffers < VISIONIPC_MAX_FDS);
  int aligned_w = 0, aligned_h = 0;

  size_t size = 0;
  size_t stride = 0;
  size_t uv_offset = 0;

  if (rgb) {
    visionbuf_compute_aligned_width_and_height(width, height, &aligned_w, &aligned_h);
    size = (size_t)aligned_w * (size_t)aligned_h * 3;
    stride = aligned_w * 3;
  } else {
    size = width * height * 3 / 2;
    stride = width;
    uv_offset = width * height;
  }

  create_buffers_with_sizes(type, num_buffers, rgb, width, height, size, stride, uv_offset);
}

void VisionIpcServer::create_buffers_with_sizes(VisionStreamType type, size_t num_buffers, bool rgb, size_t width, size_t height, size_t size, size_t stride, size_t uv_offset) {
  // Create map + alloc requested buffers
  for (size_t i = 0; i < num_buffers; i++){
    VisionBuf* buf = new VisionBuf();
    buf->allocate(size);
    buf->idx = i;
    buf->type = type;

    if (device_id) buf->init_cl(device_id, ctx);

    rgb ? buf->init_rgb(width, height, stride) : buf->init_yuv(width, height, stride, uv_offset);

    buffers[type].push_back(buf);
  }

  cur_idx[type] = 0;

  // Create msgq publisher for each of the `name` + type combos
  // TODO: compute port number directly if using zmq
  sockets[type] = PubSocket::create(msg_ctx, get_endpoint_name(name, type), false);
}


void VisionIpcServer::start_listener(){
  listener_thread = std::thread(&VisionIpcServer::listener, this);
}

# MJ Comment added :
# Korea 차량 지원을 위해 openpilot_kr에서 업데이트된 VisionIpcServer::listener() 메서드는 
# 지정된 이름을 사용하여 미리 정의된 위치(/tmp/)에 '[name]'을 추가하여 새로운 Unix 도메인 소켓 파일을 생성합니다. 
# 그런 후, 이 새롭게 생성된 파일에 바인딩된 listening 소켓을 설정합니다. 
# 하지만 실패 시 체크나 오류 처리가 없습니다.
# 업데이트된 코드는 OPENPILOT_PREFIX 환경 변수를 기반으로 사용자 지정 경로를 생성하고, 
# 이를 통해 한국 차량과의 호환성을 향상시킵니다. 
# 이 환경 변수가 정의된 경우, 메서드는 접두사와 원래 이름을 결합하여 새로운 경로를 생성합니다. 
# 소켓을 생성하고 새 경로에 바인딩한 후, 
# ipc_bind()가 반환하는 소켓 디스크립터가 음수(-)인 경우 오류 처리를 위해 assert() 문을 사용합니다. 
# 이것은 설정에 실패한 IPC 서버 소켓을 나타내므로, 예외가 throw됩니다.
# 요약하면, 업데이트된 코드는 환경 변수 OPENPILOT_PREFIX를 기반으로 한국 차량과의 호환성을 확보하고, 
# 프로세스 중단 및 디버깅을 위한 오류 처리를 구현합니다. 또한, IPC 서버 소켓의 생성과 바인딩을 보장합니다.

void VisionIpcServer::listener(){
  std::cout << "Starting listener for: " << name << std::endl;

  char* prefix = std::getenv("OPENPILOT_PREFIX");
  std::string path = "/tmp/";
  if (prefix) {
    path = path + std::string(prefix) + "_";
  }
  path = path + "visionipc_" + name;

  int sock = ipc_bind(path.c_str());
  assert(sock >= 0);

  while (!should_exit){
    // Wait for incoming connection
    struct pollfd polls[1] = {{0}};
    polls[0].fd = sock;
    polls[0].events = POLLIN;

    int ret = poll(polls, 1, 100);
    if (ret < 0) {
      if (errno == EINTR || errno == EAGAIN) continue;
      std::cout << "poll failed, stopping listener" << std::endl;
      break;
    }

    if (should_exit) break;
    if (!polls[0].revents) {
      continue;
    }

    // Handle incoming request
    int fd = accept(sock, NULL, NULL);
    assert(fd >= 0);

    VisionStreamType type = VisionStreamType::VISION_STREAM_MAX;
    int r = ipc_sendrecv_with_fds(false, fd, &type, sizeof(type), nullptr, 0, nullptr);
    assert(r == sizeof(type));

    // send available stream types
    if (type == VisionStreamType::VISION_STREAM_MAX) {
      std::vector<VisionStreamType> available_stream_types;
      for (auto& [stream_type, _] : buffers) {
        available_stream_types.push_back(stream_type);
      }
      r = ipc_sendrecv_with_fds(true, fd, available_stream_types.data(), available_stream_types.size() * sizeof(VisionStreamType), nullptr, 0, nullptr);
      assert(r == available_stream_types.size() * sizeof(VisionStreamType));
      close(fd);
      continue;
    }

    if (buffers.count(type) <= 0) {
      std::cout << "got request for invalid buffer type: " << type << std::endl;
      close(fd);
      continue;
    }

    int fds[VISIONIPC_MAX_FDS];
    int num_fds = buffers[type].size();
    VisionBuf bufs[VISIONIPC_MAX_FDS];

    for (int i = 0; i < num_fds; i++){
      fds[i] = buffers[type][i]->fd;
      bufs[i] = *buffers[type][i];

      // Remove some private openCL/ion metadata
      bufs[i].buf_cl = 0;
      bufs[i].copy_q = 0;
      bufs[i].handle = 0;

      bufs[i].server_id = server_id;
    }

    r = ipc_sendrecv_with_fds(true, fd, &bufs, sizeof(VisionBuf) * num_fds, fds, num_fds, nullptr);

    close(fd);
  }

  std::cout << "Stopping listener for: " << name << std::endl;
  close(sock);
  unlink(path.c_str());
}



VisionBuf * VisionIpcServer::get_buffer(VisionStreamType type){
  // Do we want to keep track if the buffer has been sent out yet and warn user?
  assert(buffers.count(type));
  auto b = buffers[type];
  return b[cur_idx[type]++ % b.size()];
}

void VisionIpcServer::send(VisionBuf * buf, VisionIpcBufExtra * extra, bool sync){
  if (sync) {
    if (buf->sync(VISIONBUF_SYNC_FROM_DEVICE) != 0) {
      LOGE("Failed to sync buffer");
    }
  }
  assert(buffers.count(buf->type));
  assert(buf->idx < buffers[buf->type].size());

  // Send over correct msgq socket
  VisionIpcPacket packet = {0};
  packet.server_id = server_id;
  packet.idx = buf->idx;
  packet.extra = *extra;

  sockets[buf->type]->send((char*)&packet, sizeof(packet));
}

VisionIpcServer::~VisionIpcServer(){
  should_exit = true;
  listener_thread.join();

  // VisionBuf cleanup
  for (auto const& [type, buf] : buffers) {
    for (VisionBuf* b : buf){
      if (b->free() != 0) {
        LOGE("Failed to free buffer");
      }
      delete b;
    }
  }

  // Messaging cleanup
  for (auto const& [type, sock] : sockets) {
    delete sock;
  }
  delete msg_ctx;
}
