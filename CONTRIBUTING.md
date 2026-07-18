# Hướng dẫn đóng góp

## Quy trình làm việc

1. Đồng bộ `main` bằng **Fetch origin** và **Pull origin**.
2. Tạo branch riêng cho một nhiệm vụ.
3. Thực hiện thay đổi nhỏ, có phạm vi rõ ràng.
4. Kiểm tra code hoặc thiết kế trước khi commit.
5. Commit với nội dung dễ hiểu.
6. Push branch lên GitHub.
7. Tạo Pull Request và nhờ ít nhất một thành viên duyệt.
8. Chỉ merge khi không còn xung đột và đã kiểm tra tích hợp.

## Checklist firmware

- [ ] Biên dịch thành công
- [ ] Không chứa mật khẩu hoặc token
- [ ] Chân GPIO và giao thức được ghi chú
- [ ] Có giới hạn an toàn cho motor/cơ cấu chấp hành
- [ ] Đã thử trên bàn trước khi lắp vào robot

## Checklist phần cứng

- [ ] Đúng phiên bản schematic và PCB
- [ ] Chạy ERC/DRC
- [ ] Kiểm tra nguồn, GND, cực tính và connector
- [ ] Kiểm tra kích thước footprint
- [ ] Không commit thư mục `History` hoặc `Project Logs`
- [ ] Có ảnh hoặc ghi chú mô tả thay đổi quan trọng

## Tránh xung đột Altium

Các file nhị phân như `.PcbDoc` và `.SchDoc` không thể gộp tự động tốt. Trước khi sửa:

1. Báo với đội file mình sẽ chỉnh.
2. Chỉ để một người sửa file đó tại một thời điểm.
3. Pull bản mới nhất trước khi mở Altium.
4. Commit và push ngay khi hoàn thành một mốc ổn định.

## Review Pull Request

Người review cần kiểm tra:

- Thay đổi có đúng mục tiêu không?
- Có ảnh hưởng tới module khác không?
- Tài liệu chân kết nối và cấu hình đã cập nhật chưa?
- Có file sinh tự động hoặc file tạm bị đưa lên không?
- Có thể hoàn tác thay đổi an toàn không?

