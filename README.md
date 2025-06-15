# Hướng dẫn mô phỏng Raspberry Pi với Proteus

## Tổng quan
Mô phỏng Raspberry Pi đọc data cảm biến khói MQ2 qua MCP3008 trên Proteus

## Yêu cầu
- Proteus (khuyến nghị phiên bản mới nhất)
- Tải GAS SENSOR ở cuối bài viết của đường link sau:
[Link tải thư viện Gas sensor](https://electronicstree.com/mq2-gas-sensor-in-proteus/)

## Các bước thực hiện cài đặt GAS SENSOR vào Proteus
- Giải nén File đã tải ở trên, pass: electronicstree.com
- Copy file trong folder LIB và dán vào `C:\Program Files (x86)\Labcenter Electronics\Proteus 8 Professional\DATA\LIBRARY` (Đây chỉ là folder nếu cài đặt theo mặc định, còn nếu đã đổi folder cài đặt thì tìm đến `<DATA\LIBRARY>`)
- Copy file trong folder MODEL và dán vào `C:\Program Files (x86)\Labcenter Electronics\Proteus 8 Professional\DATA\MODELS`

## Nội dung
- Hiện tại đang là gửi command cho MCP3008 qua chân DIN để có thể nhận data lại từ DOUT, đoạn data nhận bao gồm 10 - 12 bit
- Sau đó chuyển đổi từ bit sang giá trị điện áp và hiển thị trên LCD 

## Giấy phép
Tài liệu này chỉ dùng cho mục đích học tập.
