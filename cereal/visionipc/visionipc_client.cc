#include <chrono>
#include <cassert>
#include <iostream>
#include <thread>

#include "cereal/visionipc/ipc.h"
#include "cereal/visionipc/visionipc_client.h"
#include "cereal/visionipc/visionipc_server.h"
#include "cereal/logger/logger.h"

# MJ Comment added : 
# Korea 차량 지원을 위해 openpilot_kr에서 업데이트된 함수 connect_to_vipc_server()는 
# 고유한 이름을 사용하여 Vehicle Interface Process Control (VIPC) 서버에 연결합니다. 
# 이전 구현과 달리, 이 새로운 버전은 OPENPILOT_PREFIX라는 환경 변수를 사용하여 사용자 지정된 경로를 생성할 수 있습니다. 
# 이 환경 변수는 관련 구성 요소가 포함된 디렉토리를 나타내며, 이를 통해 생성된 경로는 /tmp/와 같은 기존의 형식과 약간 다릅니다.
# 예를 들어, 환경 변수 OPENPILOT_PREFIX가 "/my/custom/directory/"라면, 
# 생성된 경로는 "/tmp/my_custom_directory_visionipc_[name]"입니다. 
# 그러면 이 경로는 한국 차량과 관련된 독립적인 Unix 도메인 소켓을 참조합니다. 
# 따라서, 이 방법을 사용하면 각 차량 플랫폼에 따라 다른 접근 방식을 선택할 수 있습니다.
# 이 외에는, 초기 시도가 실패했을 때 클라이언트가 
# 서버를 시작할 수 있는 시간을 제공하기 위해 블로킹 모드에서 재시도하는 기능은 
# 이전 구현과 동일하게 유지됩니다.
# 요약하면, 업데이트된 코드는 
# 환경 변수 OPENPILOT_PREFIX를 기반으로 한국 차량과의 호환성을 확보하는 데 필요한, 사용자 지정된 경로를 생성합니다.

static int connect_to_vipc_server(const std::string &name, bool blocking) {
  char* prefix = std::getenv("OPENPILOT_PREFIX");
  std::string path = "/tmp/";
  if (prefix) {
    path = path + std::string(prefix) + "_";
  }
  path = path + "visionipc_" + name;

  int socket_fd = ipc_connect(path.c_str());
  while (socket_fd < 0 && blocking) {
    std::cout << "VisionIpcClient connecting" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    socket_fd = ipc_connect(path.c_str());
  }
  return socket_fd;
}

VisionIpcClient::VisionIpcClient(std::string name, VisionStreamType type, bool conflate, cl_device_id device_id, cl_context ctx) : name(name), type(type), device_id(device_id), ctx(ctx) {
  msg_ctx = Context::create();
  sock = SubSocket::create(msg_ctx, get_endpoint_name(name, type), "127.0.0.1", conflate, false);

  poller = Poller::create();
  poller->registerSocket(sock);
}

// Connect is not thread safe. Do not use the buffers while calling connect
bool VisionIpcClient::connect(bool blocking){
  connected = false;

  // Cleanup old buffers on reconnect
  for (size_t i = 0; i < num_buffers; i++){
    if (buffers[i].free() != 0) {
      LOGE("Failed to free buffer %zu", i);
    }
  }

  num_buffers = 0;

  int socket_fd = connect_to_vipc_server(name, blocking);
  if (socket_fd < 0) {
    return false;
  }
  // Send stream type to server to request FDs
  int r = ipc_sendrecv_with_fds(true, socket_fd, &type, sizeof(type), nullptr, 0, nullptr);
  assert(r == sizeof(type));

  // Get FDs
  int fds[VISIONIPC_MAX_FDS];
  VisionBuf bufs[VISIONIPC_MAX_FDS];
  r = ipc_sendrecv_with_fds(false, socket_fd, &bufs, sizeof(bufs), fds, VISIONIPC_MAX_FDS, &num_buffers);

  assert(num_buffers >= 0);
  assert(r == sizeof(VisionBuf) * num_buffers);

  // Import buffers
  for (size_t i = 0; i < num_buffers; i++){
    buffers[i] = bufs[i];
    buffers[i].fd = fds[i];
    buffers[i].import();
    if (buffers[i].rgb) {
      buffers[i].init_rgb(buffers[i].width, buffers[i].height, buffers[i].stride);
    } else {
      buffers[i].init_yuv(buffers[i].width, buffers[i].height, buffers[i].stride, buffers[i].uv_offset);
    }

    if (device_id) buffers[i].init_cl(device_id, ctx);
  }

  close(socket_fd);
  connected = true;
  return true;
}

VisionBuf * VisionIpcClient::recv(VisionIpcBufExtra * extra, const int timeout_ms){
  auto p = poller->poll(timeout_ms);

  if (!p.size()){
    return nullptr;
  }

  Message * r = sock->receive(true);
  if (r == nullptr){
    return nullptr;
  }

  // Get buffer
  assert(r->getSize() == sizeof(VisionIpcPacket));
  VisionIpcPacket *packet = (VisionIpcPacket*)r->getData();

  assert(packet->idx < num_buffers);
  VisionBuf * buf = &buffers[packet->idx];

  if (buf->server_id != packet->server_id){
    connected = false;
    delete r;
    return nullptr;
  }

  if (extra) {
    *extra = packet->extra;
  }

  if (buf->sync(VISIONBUF_SYNC_TO_DEVICE) != 0) {
    LOGE("Failed to sync buffer");
  }

  delete r;
  return buf;
}

std::set<VisionStreamType> VisionIpcClient::getAvailableStreams(const std::string &name, bool blocking) {
  int socket_fd = connect_to_vipc_server(name, blocking);
  if (socket_fd < 0) {
    return {};
  }
  // Send VISION_STREAM_MAX to server to request available streams
  int request = VISION_STREAM_MAX;
  int r = ipc_sendrecv_with_fds(true, socket_fd, &request, sizeof(request), nullptr, 0, nullptr);
  assert(r == sizeof(request));

  VisionStreamType available_streams[VISION_STREAM_MAX] = {};
  r = ipc_sendrecv_with_fds(false, socket_fd, &available_streams, sizeof(available_streams), nullptr, 0, nullptr);
  assert((r >= 0) && (r % sizeof(VisionStreamType) == 0));
  close(socket_fd);
  return std::set<VisionStreamType>(available_streams, available_streams + r / sizeof(VisionStreamType));
}

VisionIpcClient::~VisionIpcClient(){
  for (size_t i = 0; i < num_buffers; i++){
    if (buffers[i].free() != 0) {
      LOGE("Failed to free buffer %zu", i);
    }
  }

  delete sock;
  delete poller;
  delete msg_ctx;
}
