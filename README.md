# ROBOCON 2026

Kho mã nguồn và thiết kế phần cứng dùng chung của đội Robocon 2026.

> Trạng thái: đang phát triển. Nhánh `main` chỉ dành cho phiên bản đã được kiểm tra.

## Phạm vi dự án

- Firmware điều khiển robot và các module ngoại vi
- Schematic, PCB và thư viện linh kiện
- Tài liệu kỹ thuật, sơ đồ kết nối và quy trình kiểm thử

## Cấu trúc hiện tại

```text
robocon-2026/
├── firmwave/                  # Firmware hiện có
│   ├── Auto_v1.cpp
│   └── line.h
├── hardwave/                  # Dự án Altium, PCB và schematic
│   ├── ROBOCON_AUTO/
│   ├── ROBOCON_T9_LOGIC/
│   └── ROBOCON_T9_V1/
├── docs/                      # Tài liệu làm việc nhóm
├── CONTRIBUTING.md            # Quy tắc đóng góp
└── README.md
```

Tên `firmwave` và `hardwave` được giữ nguyên để không làm hỏng đường dẫn dự án hiện tại. Có thể đổi thành `firmware` và `hardware` trong một Pull Request riêng sau khi cả đội thống nhất.

## Bắt đầu

1. Cài GitHub Desktop và đăng nhập GitHub.
2. Clone repository về máy.
3. Chọn **Fetch origin** và **Pull origin** trước khi bắt đầu.
4. Tạo branch mới từ `main`.
5. Chỉ sửa phần được phân công và kiểm tra trước khi commit.
6. Push branch và tạo Pull Request để thành viên khác duyệt.

Xem hướng dẫn chi tiết tại [CONTRIBUTING.md](CONTRIBUTING.md).

## Quy ước branch

| Loại công việc | Ví dụ |
|---|---|
| Tính năng firmware | `feature/motor-control` |
| Thiết kế phần cứng | `hardware/logic-board-v2` |
| Sửa lỗi | `fix/encoder-reading` |
| Tài liệu | `docs/wiring-diagram` |

## Quy ước commit

Viết ngắn gọn, mô tả đúng thay đổi:

```text
feat: thêm điều khiển động cơ
fix: sửa cách đọc encoder
hardware: cập nhật schematic bo logic
docs: bổ sung sơ đồ kết nối
```

## Nguyên tắc an toàn

- Không commit mật khẩu Wi-Fi, token, khóa SSH hoặc thông tin bí mật.
- Không đưa code chưa kiểm tra trực tiếp lên `main`.
- Không sửa đồng thời cùng một file PCB/SchDoc trên nhiều branch.
- Ghi rõ điện áp, cực tính và phiên bản bo mạch trước khi thử trên robot.
- Luôn có phương án dừng khẩn cấp khi kiểm thử cơ cấu chấp hành.

## Công cụ

- Firmware: C/C++ (bổ sung board, framework và phiên bản khi chốt cấu hình)
- PCB/Schematic: Altium Designer
- Quản lý mã nguồn: Git, GitHub Desktop và GitHub Pull Request

## Thành viên và phân công

Cập nhật bảng này khi đội chốt vai trò:

| Thành viên | Phụ trách | Module |
|---|---|---|
| Chưa cập nhật | Trưởng nhóm | Điều phối và tích hợp |
| Chưa cập nhật | Firmware | Điều khiển robot |
| Chưa cập nhật | Hardware | PCB và hệ thống điện |
| Chưa cập nhật | Mechanical | Cơ khí |
