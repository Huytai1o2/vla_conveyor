# VLA Conveyor Control

Hệ thống Vision-Language-Action (VLA) dùng Gemini để nhận diện vật thể theo yêu cầu của người vận hành, sinh action điều khiển và đưa vật thể vào vùng giữa của băng chuyền đảo chiều.

> Trạng thái hiện tại: phần mềm đã qua kiểm tra cú pháp, mock state machine, action validation và giao thức serial. Hệ thống vẫn cần hoàn thành checklist bench-test với camera, YoloUNO và motor thật trước khi được xem là sẵn sàng chạy production hoặc chạy không giám sát.

## Kiến trúc và flow hoạt động

```text
Kết nối YoloUNO + camera
          |
          v
Calibration: chọn 4 góc băng chuyền
          |
          v
OpenCV tạo perspective transform 1000 x 300
          |
          v
Mở dashboard
  - ảnh calibrated realtime bên trái
  - chat, trạng thái và log bên phải
          |
          v
Người dùng nhập yêu cầu vật thể
          |
          v
Ảnh calibrated + instruction + thông số băng chuyền
          |
          v
Gemini sinh VLA action
  direction + duration_ms + task_status
          |
          v
Controller validate action và hiển thị action token
          |
          v
Chuyển action sang lệnh serial của firmware
          |
          v
YoloUNO ACK -> chạy motor -> DONE
          |
          v
Chờ vật ổn định, chụp frame mới và lặp lại
```

### Phân chia trách nhiệm

OpenCV chỉ được dùng để:

- Đọc camera.
- Cho người dùng chọn bốn góc băng chuyền.
- Tính và áp dụng perspective transform.
- Tạo ảnh calibrated `1000 x 300` và vẽ vùng giữa.
- Hiển thị/encode ảnh.

Gemini chịu trách nhiệm:

- Tìm đúng vật thể khớp với instruction trong sidebar.
- Trả tâm vật thể chuẩn hóa `[y, x]`.
- Quyết định `LEFT`, `RIGHT` hoặc `STOP`.
- Sinh trực tiếp `duration_ms` và `task_status`.

Python controller không dùng tọa độ để tính lại action. Controller chỉ kiểm tra kết quả Gemini có an toàn và hợp lệ trước khi gửi xuống firmware.

## VLA action và firmware protocol

Kết quả Gemini hợp lệ được biểu diễn trong GUI bằng action token:

```text
[ACT_RIGHT] [DURATION_0780_MS] [STATUS_MOVE]
```

Action token chỉ dùng để hiển thị và audit. Firmware hiện tại không parse chuỗi có dấu ngoặc. Controller chuyển action sang protocol mà `conveyor_firmware/src/main.cpp` hỗ trợ:

| Controller gửi | Firmware trả | Ý nghĩa |
|---|---|---|
| `PING\n` | `PONG` | Kiểm tra kết nối |
| `STOP\n` | `STOPPED` | Dừng motor |
| `MOVE,RIGHT,780\n` | `ACK,RIGHT,780`, sau đó `DONE` | Chạy sang phải trong 780 ms |
| `MOVE,LEFT,300\n` | `ACK,LEFT,300`, sau đó `DONE` | Chạy sang trái trong 300 ms |

Controller chỉ cho phép duration từ `80` đến `1500 ms`, dù firmware có thể nhận từ `50` đến `3000 ms`.

`IMAGE_RIGHT_IS_FORWARD` trong `conveyor_firmware/src/main.cpp` ánh xạ hướng trên ảnh sang chiều quay vật lý của motor:

- Với `false`: `MOVE,RIGHT` dùng `MOTOR_BACKWARD`, `MOVE,LEFT` dùng `MOTOR_FORWARD`.
- Nếu lệnh `MOVE,RIGHT` làm vật đi sang trái trên ảnh calibrated, đổi giá trị này thành `true` và nạp lại firmware.

## Vòng đời một instruction

1. Sau calibration, dashboard chuyển sang `WAITING_FOR_PROMPT`.
2. Người dùng nhập instruction trong sidebar bên phải và bấm **Send**.
3. Composer bị khóa trong khi instruction đang chạy.
4. Gemini phân tích frame mới nhất và sinh action.
5. Controller validate action, in action token rồi mới gửi serial command.
6. Với `MOVE`, controller đợi đúng `ACK,<direction>,<duration>` rồi mới chấp nhận `DONE`.
7. Sau mỗi pulse, controller chờ vật ổn định rồi phân tích frame mới.
8. Với `CENTERED`, motor nhận `STOP`, sidebar in `SUCCESS` và composer được mở cho instruction tiếp theo.

Nếu vật thể bị lấy khỏi băng chuyền hoặc không còn khớp instruction:

- Pulse đang chạy được phép hoàn thành.
- Lần inference tiếp theo gửi/giữ `STOP`.
- Sidebar in `WARNING: target not recognized - retry n/20`.
- Nếu vật quay lại, counter được reset và vòng điều khiển tiếp tục.
- Sau 20 kết quả no-match liên tiếp, instruction bị hủy và composer được mở lại.

Lỗi Gemini connection, timeout, rate limit, JSON/schema hoặc action không hợp lệ dùng counter riêng `n/5`:

- Motor luôn được giữ ở `STOP`.
- Lỗi network hiển thị `RECONNECTING`.
- Dữ liệu/schema không hợp lệ hiển thị `ERROR`.
- Một kết quả Gemini hợp lệ sẽ reset technical counter.
- Technical failure thứ năm kích hoạt safe shutdown.

## Cài đặt và chạy

Tạo virtual environment và cài dependencies:

```bash
python -m venv .venv
source .venv/bin/activate  # Windows: .venv\Scripts\activate
pip install -r requirements.txt
```

Tạo `.env` từ file mẫu và thêm Gemini API key:

```bash
cp .env.example .env
```

```dotenv
GEMINI_API_KEY=your_gemini_api_key
```

Kiểm tra các constant trong `controller.py` trước khi chạy:

```python
CAMERA_INDEX = 0
SERIAL_PORT = "COM18"
SERIAL_BAUD = 115200
BELT_LENGTH_CM = 77.0
SPEED_TEST_DISTANCE_CM = 77.0
SPEED_TEST_TIME_S = 5.5925
```

Đóng PlatformIO Serial Monitor để tránh chiếm COM port, sau đó chạy:

```bash
python controller.py
```

### Calibration

1. Nhấn `S` để chốt frame calibration hoặc `Q` để thoát.
2. Click bốn góc của vùng băng chuyền theo thứ tự bất kỳ.
3. Chuột phải để undo, `R` để chọn lại.
4. Nhấn `Enter`, `Space` hoặc Return khi polygon hợp lệ.
5. Không di chuyển camera sau calibration. Nếu camera bị đổi vị trí, phải calibrate lại.

Sau calibration, dashboard xuất hiện với ảnh calibrated realtime bên trái và sidebar điều khiển bên phải.

## Bench-test với phần cứng thật

Bench-test là kiểm tra tích hợp trên bàn với camera, board và motor thật. Thực hiện theo thứ tự dưới đây; không bỏ qua các bước an toàn đầu tiên.

### 0. Chuẩn bị an toàn

- Dọn sạch vùng nguy hiểm quanh băng chuyền.
- Dùng một vật nhẹ, không dễ kẹt để thử nghiệm.
- Bảo đảm có thể ngắt nguồn motor ngay lập tức.
- Không để tay gần cơ cấu truyền động khi gửi lệnh MOVE.
- Đóng PlatformIO Serial Monitor trước khi mở controller.

### 1. Firmware handshake

- Nạp firmware trong thư mục `conveyor_firmware` vào YoloUNO.
- Xác nhận baud rate `115200` và COM port đúng.
- Chạy `serial_test.py` khi băng chuyền an toàn.
- Pass khi board trả `PONG`, `ACK,RIGHT,300` và `DONE` đúng thứ tự.
- Fail nếu thiếu ACK/DONE, ACK sai direction/duration hoặc xuất hiện `ERR,*`.

> `serial_test.py` có gửi một pulse `MOVE,RIGHT,300`; chỉ chạy khi motor có thể chuyển động an toàn.

### 2. Kiểm tra hướng motor

- Đặt vật ở vùng dễ quan sát.
- Gửi `MOVE,RIGHT,300`.
- Pass nếu vật di chuyển sang phải trên ảnh calibrated.
- Nếu vật đi sang trái, đổi `IMAGE_RIGHT_IS_FORWARD` trong firmware và nạp lại board.
- Lặp lại với `MOVE,LEFT,300` và xác nhận vật đi sang trái.

### 3. Kiểm tra STOP và DONE vật lý

- Gửi một MOVE ngắn và quan sát motor.
- Pass nếu motor dừng vật lý ngay khi firmware phát `DONE`.
- Bấm **Stop / Exit** trong khi motor đang chạy.
- Pass nếu controller gửi STOP, motor dừng và không có MOVE muộn xuất hiện sau thao tác Stop.
- Ngắt nguồn ngay nếu motor vẫn quay sau `DONE` hoặc `STOPPED`.

### 4. Camera và calibration

- Calibration ở ít nhất ba góc/độ cao camera hợp lý.
- Xác nhận toàn bộ chiều dài băng chuyền nằm trong ảnh warped `1000 x 300`.
- Xác nhận hai đường xanh luôn đánh dấu đúng vùng giữa.
- Dịch chuyển camera sau calibration và xác nhận hệ thống yêu cầu/được chạy lại calibration trước khi tiếp tục.

### 5. Dashboard realtime

- Xác nhận ảnh calibrated bên trái tiếp tục cập nhật khi idle, Gemini đang phân tích, motor đang chạy và controller đang retry.
- Xác nhận sidebar nằm bên phải, có transcript auto-scroll, state hiện tại, composer, Send và Stop/Exit.
- Xác nhận `INFO`, `SUCCESS`, `WARNING`, `RECONNECTING` và `ERROR` có thể phân biệt rõ.
- Xác nhận GUI không bị treo trong thời gian Gemini gọi API hoặc chờ firmware.

### 6. Closed-loop centering

- Đặt một vật lệch tâm và nhập instruction mô tả đúng vật đó.
- Xác nhận action token xuất hiện trước firmware command.
- Xác nhận action direction/duration trong token giống chính xác command `MOVE`.
- Pass khi vật được đưa vào giữa, firmware nhận STOP, sidebar in SUCCESS và composer được mở lại.

### 7. Vật bị lấy khỏi băng chuyền

- Trong lúc đang center, lấy vật ra sau khi một pulse đã bắt đầu.
- Pass nếu pulse hiện tại hoàn thành, inference tiếp theo giữ motor STOP và warning tăng `1/20`, `2/20`, ...
- Đặt đúng vật trở lại và xác nhận controller tự tiếp tục, đồng thời reset no-match counter.
- Để vật vắng mặt đủ 20 lần và xác nhận instruction bị hủy nhưng ứng dụng vẫn chạy.

### 8. Gemini retry

- Tạm thời tạo điều kiện mất mạng hoặc dùng mock để gây timeout/rate limit.
- Xác nhận sidebar hiển thị `RECONNECTING n/5` và motor luôn STOP.
- Dùng mock response sai schema để xác nhận sidebar hiển thị `ERROR`, không gửi MOVE.
- Pass nếu technical failure thứ năm đóng ứng dụng an toàn.

### 9. Camera disconnect

- Rút camera khi dashboard đang idle và khi đang chạy một instruction.
- Pass nếu stale-frame detection chuyển sang ERROR, motor STOP và ứng dụng shutdown an toàn.
- Fail nếu hệ thống tiếp tục dùng frame cũ để gửi MOVE.

### 10. Manual button

- Thử nút tiến/lùi vật lý khi host đang idle và khi một pulse đang chạy.
- Xác nhận hành vi thực tế không làm motor tiếp tục chạy ngoài duration host đã yêu cầu.
- Đây là test bắt buộc vì task nút bấm có thể thay đổi `motor_state` độc lập với host command.

## Điều kiện chấp nhận trước khi chạy production

Chỉ coi hệ thống đạt khi tất cả điều kiện sau đều pass:

- `PING/PONG`, exact `ACK` và `DONE` đúng protocol.
- `RIGHT/LEFT` đúng với hướng vật trên ảnh calibrated.
- Motor dừng vật lý sau `DONE`, STOP và Stop/Exit.
- Không có MOVE được gửi sau khi Stop/Exit bắt đầu.
- Camera disconnect không dẫn đến action dựa trên frame cũ.
- Calibration đúng ở vị trí camera triển khai thực tế.
- Closed-loop đưa đúng vật thể vào giữa và mở lại composer.
- No-match 20 lần và technical retry 5 lần hoạt động đúng.
- Manual button không phá vỡ giới hạn an toàn của host control.

## Giới hạn và rủi ro cần theo dõi

- Firmware phát `DONE` ngay sau khi cập nhật shared motor state; motor task có thể cần thêm một khoảng ngắn để thực hiện I2C STOP vật lý.
- Manual button task có thể thay đổi `motor_state` trong lúc host pulse vẫn đang được firmware theo dõi.
- `IMAGE_RIGHT_IS_FORWARD` phụ thuộc cách đấu dây motor và phải được xác minh trên hệ thống thật.
- Camera phải giữ nguyên vị trí sau calibration.

## Bảo mật

Không commit `.env` hoặc API key. `.env` đã được loại khỏi Git; chỉ chia sẻ `.env.example`.

## Firmware

Thư mục `conveyor_firmware` là PlatformIO project cho YoloUNO/ESP32. Controller hiện giữ nguyên protocol của firmware và không gửi bracketed action token trực tiếp xuống board.
