from __future__ import annotations

import os
import re
import subprocess
from datetime import date
from pathlib import Path
from textwrap import wrap

from PIL import Image, ImageDraw, ImageFont
from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_CELL_VERTICAL_ALIGNMENT, WD_ROW_HEIGHT_RULE, WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Inches, Pt, RGBColor


ROOT = Path(r"D:\ABOUT_ROBOCON_2026\robocon-2026")
OUT_DIR = ROOT / "docs"
TMP_DIR = OUT_DIR / ".report_tmp"
OUT = TMP_DIR / "Bao_cao_ky_thuat_he_thong_ROBOCON_2026_raw.docx"

TEENSY = ROOT / "firmwave" / "ROBOCON_KOSEN_F0"
ESP32 = ROOT / "firmwave" / "Map_do"
SIM = ROOT / "firmwave" / "ESP32_MECHANISM_SIMULATOR"
UI = ROOT / "pid-tuner"

NAVY = "123B5D"
CYAN = "DFF4F7"
PALE = "F3F7FA"
LIGHT = "D9D9D9"
MID = "5C6B73"
BLACK = "000000"
RED = "B42318"
GREEN = "16794F"


def read(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="replace")


def git(args: list[str], cwd: Path) -> str:
    try:
        return subprocess.check_output(["git", *args], cwd=cwd, text=True, encoding="utf-8", errors="replace").strip()
    except Exception:
        return "không xác định"


def set_cell_shading(cell, fill: str) -> None:
    tc_pr = cell._tc.get_or_add_tcPr()
    shd = tc_pr.find(qn("w:shd"))
    if shd is None:
        shd = OxmlElement("w:shd")
        tc_pr.append(shd)
    shd.set(qn("w:fill"), fill)


def set_cell_border(cell, color: str = LIGHT, size: str = "4") -> None:
    tc_pr = cell._tc.get_or_add_tcPr()
    borders = tc_pr.first_child_found_in("w:tcBorders")
    if borders is None:
        borders = OxmlElement("w:tcBorders")
        tc_pr.append(borders)
    for edge in ("top", "left", "bottom", "right", "insideH", "insideV"):
        tag = f"w:{edge}"
        element = borders.find(qn(tag))
        if element is None:
            element = OxmlElement(tag)
            borders.append(element)
        element.set(qn("w:val"), "single")
        element.set(qn("w:sz"), size)
        element.set(qn("w:color"), color)


def set_cell_margins(cell, top=100, start=120, bottom=100, end=120) -> None:
    tc = cell._tc
    tc_pr = tc.get_or_add_tcPr()
    tc_mar = tc_pr.first_child_found_in("w:tcMar")
    if tc_mar is None:
        tc_mar = OxmlElement("w:tcMar")
        tc_pr.append(tc_mar)
    for margin, value in (("top", top), ("start", start), ("bottom", bottom), ("end", end)):
        node = tc_mar.find(qn(f"w:{margin}"))
        if node is None:
            node = OxmlElement(f"w:{margin}")
            tc_mar.append(node)
        node.set(qn("w:w"), str(value))
        node.set(qn("w:type"), "dxa")


def keep_with_next(paragraph) -> None:
    p_pr = paragraph._p.get_or_add_pPr()
    keep = p_pr.find(qn("w:keepNext"))
    if keep is None:
        keep = OxmlElement("w:keepNext")
        p_pr.append(keep)


def add_field(paragraph, instruction: str) -> None:
    run = paragraph.add_run()
    begin = OxmlElement("w:fldChar")
    begin.set(qn("w:fldCharType"), "begin")
    instr = OxmlElement("w:instrText")
    instr.set(qn("xml:space"), "preserve")
    instr.text = instruction
    separate = OxmlElement("w:fldChar")
    separate.set(qn("w:fldCharType"), "separate")
    end = OxmlElement("w:fldChar")
    end.set(qn("w:fldCharType"), "end")
    run._r.extend([begin, instr, separate, end])


def repeat_header(row) -> None:
    tr_pr = row._tr.get_or_add_trPr()
    tbl_header = OxmlElement("w:tblHeader")
    tbl_header.set(qn("w:val"), "true")
    tr_pr.append(tbl_header)


def cant_split(row) -> None:
    tr_pr = row._tr.get_or_add_trPr()
    node = OxmlElement("w:cantSplit")
    tr_pr.append(node)


def set_repeat_table_layout(table, widths: list[float] | None = None) -> None:
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = False
    tbl_pr = table._tbl.tblPr
    layout = tbl_pr.find(qn("w:tblLayout"))
    if layout is None:
        layout = OxmlElement("w:tblLayout")
        tbl_pr.append(layout)
    layout.set(qn("w:type"), "fixed")
    repeat_header(table.rows[0])
    if widths:
        for row in table.rows:
            for i, width in enumerate(widths):
                if i < len(row.cells):
                    row.cells[i].width = Inches(width)


def style_table(table, widths: list[float] | None = None, font_size=8.5) -> None:
    set_repeat_table_layout(table, widths)
    for r_idx, row in enumerate(table.rows):
        cant_split(row)
        for c_idx, cell in enumerate(row.cells):
            set_cell_border(cell)
            set_cell_margins(cell)
            cell.vertical_alignment = WD_CELL_VERTICAL_ALIGNMENT.CENTER
            if r_idx == 0:
                set_cell_shading(cell, NAVY)
            elif r_idx % 2 == 0:
                set_cell_shading(cell, PALE)
            for paragraph in cell.paragraphs:
                paragraph.paragraph_format.space_after = Pt(0)
                paragraph.paragraph_format.line_spacing = 1.05
                if r_idx == 0 or (c_idx == 0 and len(cell.text) < 25):
                    paragraph.alignment = WD_ALIGN_PARAGRAPH.CENTER
                for run in paragraph.runs:
                    run.font.name = "Arial"
                    run._element.get_or_add_rPr().rFonts.set(qn("w:eastAsia"), "Arial")
                    run.font.size = Pt(font_size)
                    if r_idx == 0:
                        run.font.bold = True
                        run.font.color.rgb = RGBColor(255, 255, 255)


def add_table(doc: Document, headers: list[str], rows: list[list[object]], widths=None, font_size=8.5):
    table = doc.add_table(rows=1, cols=len(headers))
    table.style = "Table Grid"
    for i, header in enumerate(headers):
        table.rows[0].cells[i].text = str(header)
    for values in rows:
        cells = table.add_row().cells
        for i, value in enumerate(values):
            cells[i].text = str(value)
    style_table(table, widths, font_size)
    doc.add_paragraph().paragraph_format.space_after = Pt(0)
    return table


def add_bullets(doc: Document, items: list[str], level=0) -> None:
    for item in items:
        p = doc.add_paragraph(style="List Bullet" if level == 0 else "List Bullet 2")
        p.add_run(item)


def add_numbered(doc: Document, items: list[str]) -> None:
    for item in items:
        doc.add_paragraph(item, style="List Number")


def add_code(doc: Document, text: str) -> None:
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Inches(0.25)
    p.paragraph_format.right_indent = Inches(0.2)
    p.paragraph_format.space_before = Pt(3)
    p.paragraph_format.space_after = Pt(6)
    p_pr = p._p.get_or_add_pPr()
    shd = OxmlElement("w:shd")
    shd.set(qn("w:fill"), "F2F2F2")
    p_pr.append(shd)
    run = p.add_run(text)
    run.font.name = "Consolas"
    run._element.get_or_add_rPr().rFonts.set(qn("w:eastAsia"), "Consolas")
    run.font.size = Pt(8.5)


def set_alt_text(inline_shape, title: str, description: str) -> None:
    doc_pr = inline_shape._inline.docPr
    doc_pr.set("title", title)
    doc_pr.set("descr", description)


def add_figure(doc: Document, path: Path, caption: str, width=6.8, alt="") -> None:
    p = doc.add_paragraph()
    p.alignment = WD_ALIGN_PARAGRAPH.CENTER
    shape = p.add_run().add_picture(str(path), width=Inches(width))
    set_alt_text(shape, caption, alt or caption)
    cp = doc.add_paragraph(caption, style="Caption")
    cp.alignment = WD_ALIGN_PARAGRAPH.CENTER
    cp.paragraph_format.space_after = Pt(8)


def font(size: int, bold=False):
    path = Path(r"C:\Windows\Fonts\arialbd.ttf" if bold else r"C:\Windows\Fonts\arial.ttf")
    return ImageFont.truetype(str(path), size=size)


def draw_arrow(draw: ImageDraw.ImageDraw, start, end, color="#2D596D", width=5) -> None:
    draw.line([start, end], fill=color, width=width)
    x1, y1 = start
    x2, y2 = end
    import math
    angle = math.atan2(y2 - y1, x2 - x1)
    length = 18
    for delta in (2.55, -2.55):
        a = angle + delta
        draw.line([end, (x2 + length * math.cos(a), y2 + length * math.sin(a))], fill=color, width=width)


def draw_box(draw, xy, title, lines, fill="#F3F7FA", outline="#123B5D"):
    x1, y1, x2, y2 = xy
    draw.rounded_rectangle(xy, radius=18, fill=fill, outline=outline, width=4)
    draw.text((x1 + 22, y1 + 18), title, font=font(31, True), fill="#000000")
    y = y1 + 65
    for line in lines:
        for part in wrap(line, max(20, int((x2 - x1) / 18))):
            draw.text((x1 + 22, y), part, font=font(22), fill="#22313A")
            y += 29


def make_diagrams() -> dict[str, Path]:
    TMP_DIR.mkdir(parents=True, exist_ok=True)
    diagrams = {}

    img = Image.new("RGB", (1800, 1080), "white")
    d = ImageDraw.Draw(img)
    d.text((60, 35), "Kiến trúc điều khiển và đường dữ liệu", font=font(44, True), fill="#000000")
    draw_box(d, (60, 160, 400, 520), "Máy tính", ["Chrome hoặc Edge", "Giao diện Web Serial", "COM5 57600 baud", "Lệnh và telemetry"])
    draw_box(d, (535, 115, 1135, 615), "Teensy 4.1", ["State machine AUTO", "Mecanum và odometry", "PID bánh xe 100 Hz", "BNO085, VL53L3CX", "2 MCP3008 và nút vật lý", "E STOP phần mềm, watchdog"])
    draw_box(d, (1280, 160, 1740, 520), "ESP32 S3", ["UART2 57600 baud", "2 động cơ bước", "5 van điện từ", "Profile và manual jog", "NVS có CRC"])
    draw_box(d, (535, 745, 1135, 1010), "Cơ cấu chuyển động", ["4 động cơ Mecanum", "4 encoder AB", "PWM 20 kHz", "Bánh FL FR BL BR"])
    draw_arrow(d, (400, 330), (535, 330)); draw_arrow(d, (535, 395), (400, 395))
    draw_arrow(d, (1135, 330), (1280, 330)); draw_arrow(d, (1280, 395), (1135, 395))
    draw_arrow(d, (835, 615), (835, 745)); draw_arrow(d, (900, 745), (900, 615))
    d.text((410, 280), "Serial5", font=font(22, True), fill="#2D596D")
    d.text((1145, 280), "Serial6", font=font(22, True), fill="#2D596D")
    p = TMP_DIR / "kien_truc_he_thong.png"; img.save(p); diagrams["arch"] = p

    img = Image.new("RGB", (1800, 1150), "white")
    d = ImageDraw.Draw(img)
    d.text((60, 35), "Chu kỳ điều khiển Teensy", font=font(44, True), fill="#000000")
    boxes = [
        ((70, 155, 430, 390), "1 Đọc cảm biến", ["Encoder", "BNO085", "Line", "ToF"]),
        ((535, 155, 895, 390), "2 Ước lượng trạng thái", ["Tốc độ bánh", "Pose X Y Yaw", "Line position", "Độ nghiêng cầu"]),
        ((1000, 155, 1360, 390), "3 Điều khiển ngoài", ["Position PID", "Heading PID", "Line PID", "State AUTO"]),
        ((1430, 155, 1760, 390), "4 Động học", ["Vx Vy Wz", "Chuẩn hóa chung", "Ramp mục tiêu"]),
        ((1030, 640, 1430, 925), "5 PID từng bánh", ["Target mm/s", "KFF và deadzone", "PID có dt", "Anti windup"]),
        ((420, 640, 820, 925), "6 Driver và phản hồi", ["PWM 20 kHz", "4 driver motor", "Encoder trả về", "Fault mất encoder"]),
    ]
    for box in boxes: draw_box(d, *box)
    draw_arrow(d, (430, 270), (535, 270)); draw_arrow(d, (895, 270), (1000, 270)); draw_arrow(d, (1360, 270), (1430, 270))
    draw_arrow(d, (1595, 390), (1350, 640)); draw_arrow(d, (1030, 785), (820, 785)); draw_arrow(d, (420, 720), (250, 390))
    d.text((70, 1040), "Các khối 1 đến 6 được cập nhật theo chu kỳ 10 ms; UART, ToF và buzzer chạy không chặn ngoài nhánh điều khiển.", font=font(25), fill="#22313A")
    p = TMP_DIR / "chu_ky_dieu_khien.png"; img.save(p); diagrams["control"] = p

    img = Image.new("RGB", (1800, 2480), "white")
    d = ImageDraw.Draw(img)
    d.text((70, 35), "Luồng AUTO đang được gọi từ START", font=font(44, True), fill="#000000")
    states = [
        ("WAIT START", "DISARM và chờ lệnh"),
        ("CAN START", "Tiến 170 mm bằng encoder, tốc độ 280 mm/s, bỏ qua ToF"),
        ("DI SANG TRAI A", "Đi ngang theo map, đếm line A và giới hạn tìm quanh 1075 mm"),
        ("BU TAM A", "Cặp mắt giữa và trong của cụm 3 mắt cùng H1 H2 phải nằm trên line"),
        ("CAN YAW A", "Chốt điều kiện line; ToF chỉ được ghi nhận"),
        ("CAN A", "Giữ heading tại chỗ đến khi ổn định"),
        ("DOI GAP A", "Kêu 2 tiếng; REAL chờ ESP32 DONE, SIM chờ 4 giây"),
        ("DI SANG TRAI B", "Reset pose tại A rồi đi ngang 200 mm bằng encoder và H1 H2"),
        ("BU TAM B", "Cụm đối diện phải có mắt 0 và 1000 trên line"),
        ("CAN YAW B", "Chốt B; ToF chỉ được ghi nhận"),
        ("CAN B", "Giữ heading tại chỗ đến khi ổn định"),
        ("DOI GAP B", "Kêu 2 tiếng; chờ cơ cấu hoặc timer"),
        ("DI SANG C", "Khóa hai cụm 3 mắt, đi trái 1300 mm bằng encoder"),
        ("CHAY LEN XANH", "Tìm và căn tâm line bằng 8 mắt giữa"),
        ("DI LEN DEM LINE", "Bám line liên tục; nhận dốc, đỉnh cầu và vạch kết thúc"),
        ("FINISH", "Dừng motor, DISARM và phát AUTO COMPLETE"),
    ]
    x1, x2 = 190, 1610
    top = 125
    h = 118
    gap = 28
    for i, (name, desc) in enumerate(states):
        y = top + i * (h + gap)
        fill = "#DFF4F7" if i not in (0, len(states)-1) else "#E8ECEF"
        d.rounded_rectangle((x1, y, x2, y+h), radius=16, fill=fill, outline="#123B5D", width=4)
        d.text((x1+24, y+20), name, font=font(28, True), fill="#000000")
        d.text((x1+390, y+22), desc, font=font(23), fill="#22313A")
        if i < len(states)-1:
            draw_arrow(d, ((x1+x2)//2, y+h), ((x1+x2)//2, y+h+gap), width=4)
    p = TMP_DIR / "luong_auto_hien_tai.png"; img.save(p); diagrams["auto"] = p

    img = Image.new("RGB", (1800, 1030), "white")
    d = ImageDraw.Draw(img)
    d.text((60, 35), "Bắt tay cơ cấu tại điểm A và B", font=font(44, True), fill="#000000")
    draw_box(d, (80, 180, 520, 700), "Teensy", ["Vào DOI GAP", "Gửi POINT A hoặc POINT B", "Lưu thời điểm", "REAL chờ DONE", "SIM tự đếm 4000 ms", "Timeout 15000 ms"])
    draw_box(d, (680, 180, 1120, 700), "UART6", ["TX24 sang RX16", "RX25 nhận TX15", "57600 baud", "Dòng ASCII CR LF", "Parser không chặn"])
    draw_box(d, (1280, 180, 1720, 700), "ESP32", ["Đổi lệnh thành profile", "Chạy AccelStepper", "Điều khiển van", "Phát DONE profile", "STATUS 5 Hz", "STOP ưu tiên"])
    draw_arrow(d, (520, 340), (680, 340)); draw_arrow(d, (1120, 340), (1280, 340))
    draw_arrow(d, (1280, 530), (1120, 530)); draw_arrow(d, (680, 530), (520, 530))
    d.text((540, 295), "Lệnh", font=font(24, True), fill="#2D596D")
    d.text((540, 555), "DONE STATUS ERR", font=font(22, True), fill="#2D596D")
    d.text((150, 835), "Trong SIM 4S, Teensy gửi SIM lệnh để ESP32 không tác động cơ cấu. Teensy tự chuyển bước khi đủ 4 giây.", font=font(25), fill="#22313A")
    p = TMP_DIR / "uart_co_cau.png"; img.save(p); diagrams["uart"] = p

    img = Image.new("RGB", (1800, 760), "white")
    d = ImageDraw.Draw(img)
    d.text((60, 35), "Động học Mecanum cấu hình X", font=font(44, True), fill="#000000")
    formulas = [
        "FL = Vx - Vy - k Wz",
        "FR = Vx + Vy + k Wz",
        "BL = Vx + Vy - k Wz",
        "BR = Vx - Vy + k Wz",
    ]
    for i, line in enumerate(formulas):
        y = 150 + i * 115
        d.rounded_rectangle((180, y, 1000, y+80), radius=12, fill="#F3F7FA", outline="#123B5D", width=3)
        d.text((220, y+18), line, font=font(32, True), fill="#000000")
    draw_box(d, (1130, 150, 1690, 610), "Kích thước và giới hạn", ["k = 0.5 nhân chiều dài cộng chiều rộng", "k hiện tại bằng 305 mm", "Nếu bánh vượt giới hạn, cả 4 target cùng được chia một tỷ lệ", "Tỷ lệ hướng chuyển động được giữ nguyên"])
    p = TMP_DIR / "dong_hoc_mecanum.png"; img.save(p); diagrams["kin"] = p
    return diagrams


def extract_cpp_functions(path: Path) -> list[list[object]]:
    text = read(path)
    rows = []
    pattern = re.compile(r"(?m)^(?:[A-Za-z_][\w:<>,*& ]*\s+)?([A-Za-z_~][\w:]*)\s*\(([^;{}]*)\)\s*(?:const\s*)?\{")
    ignored = {"if", "for", "while", "switch", "catch"}
    for match in pattern.finditer(text):
        name = match.group(1)
        if name in ignored or name.startswith("operator"):
            continue
        line = text.count("\n", 0, match.start()) + 1
        args = " ".join(match.group(2).split())
        rows.append([line, name, args[:120]])
    return rows


def extract_ts_functions(path: Path) -> list[list[object]]:
    text = read(path)
    rows = []
    patterns = [
        re.compile(r"(?m)^(?:export default )?(?:async )?function\s+([A-Za-z_][\w]*)\s*\((.*?)\)\s*\{"),
        re.compile(r"(?m)^\s*(?:private\s+)?(?:async\s+)?([A-Za-z_][\w]*)\s*\((.*?)\)\s*\{")
    ]
    seen = set()
    for pattern in patterns:
        for match in pattern.finditer(text):
            name = match.group(1)
            line = text.count("\n", 0, match.start()) + 1
            key = (line, name)
            if key in seen:
                continue
            seen.add(key)
            args = " ".join(match.group(2).split())
            rows.append([line, name, args[:120]])
    return sorted(rows)


def extract_constants(path: Path) -> list[list[object]]:
    text = read(path)
    rows = []
    pattern = re.compile(r"constexpr\s+([^;]+);", re.S)
    for match in pattern.finditer(text):
        statement = re.sub(r"//.*", "", match.group(1))
        statement = " ".join(statement.split())
        if "=" not in statement:
            continue
        left, value = statement.split("=", 1)
        name_match = re.search(r"([A-Za-z_][\w]*(?:\[[^\]]+\])?)\s*$", left.strip())
        if not name_match:
            continue
        name = name_match.group(1)
        line = text.count("\n", 0, match.start()) + 1
        rows.append([line, name, value.strip()[:180]])
    return rows


def add_heading(doc: Document, text: str, level: int = 1) -> None:
    p = doc.add_heading(text, level=level)
    keep_with_next(p)


def configure_document(doc: Document) -> None:
    sec = doc.sections[0]
    sec.page_width = Inches(8.5)
    sec.page_height = Inches(11)
    sec.top_margin = Inches(0.65)
    sec.bottom_margin = Inches(0.65)
    sec.left_margin = Inches(0.72)
    sec.right_margin = Inches(0.72)

    normal = doc.styles["Normal"]
    normal.font.name = "Arial"
    normal._element.rPr.rFonts.set(qn("w:eastAsia"), "Arial")
    normal.font.size = Pt(10.5)
    normal.font.color.rgb = RGBColor(0, 0, 0)
    normal.paragraph_format.space_after = Pt(6)
    normal.paragraph_format.line_spacing = 1.12

    for style_name, size, before, after in (("Title", 28, 0, 12), ("Subtitle", 13, 0, 8),
                                             ("Heading 1", 18, 12, 7), ("Heading 2", 14, 10, 5),
                                             ("Heading 3", 11.5, 8, 4)):
        style = doc.styles[style_name]
        style.font.name = "Arial"
        style._element.rPr.rFonts.set(qn("w:eastAsia"), "Arial")
        style.font.size = Pt(size)
        style.font.bold = style_name != "Subtitle"
        style.font.color.rgb = RGBColor(0, 0, 0)
        style.paragraph_format.space_before = Pt(before)
        style.paragraph_format.space_after = Pt(after)

    doc.styles["Caption"].font.name = "Arial"
    doc.styles["Caption"]._element.rPr.rFonts.set(qn("w:eastAsia"), "Arial")
    doc.styles["Caption"].font.size = Pt(9)
    doc.styles["Caption"].font.italic = True
    doc.styles["Caption"].font.color.rgb = RGBColor(0, 0, 0)

    header = sec.header
    hp = header.paragraphs[0]
    hp.text = "ROBOCON 2026  |  Tài liệu kiểm soát phần mềm"
    hp.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    for run in hp.runs:
        run.font.name = "Arial"
        run.font.size = Pt(8)
        run.font.color.rgb = RGBColor(92, 107, 115)

    footer = sec.footer
    fp = footer.paragraphs[0]
    fp.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r = fp.add_run("Trang ")
    r.font.name = "Arial"; r.font.size = Pt(8)
    add_field(fp, "PAGE")
    r = fp.add_run("  |  Snapshot ngày 08 tháng 09 năm 2026")
    r.font.name = "Arial"; r.font.size = Pt(8)


def build_report() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    diagrams = make_diagrams()
    doc = Document()
    configure_document(doc)

    title = doc.add_paragraph(style="Title")
    title.alignment = WD_ALIGN_PARAGRAPH.CENTER
    title.add_run("Báo cáo kỹ thuật hệ thống điều khiển robot ROBOCON 2026")
    subtitle = doc.add_paragraph(style="Subtitle")
    subtitle.alignment = WD_ALIGN_PARAGRAPH.CENTER
    subtitle.add_run("Teensy 4.1  ESP32 S3  Giao diện Web Serial")
    doc.add_paragraph()
    add_figure(doc, diagrams["arch"], "Hình 1  Kiến trúc tổng thể của hệ thống", 6.9,
               "Sơ đồ máy tính kết nối Teensy qua Serial5, Teensy kết nối ESP32 qua Serial6 và điều khiển bốn bánh Mecanum.")
    meta = [
        ["Mục đích", "Giải thích toàn bộ source hiện tại để vận hành, hiệu chỉnh, bảo trì và kiểm soát thay đổi"],
        ["Phạm vi", "Firmware Teensy, firmware ESP32 cơ cấu thật, firmware ESP32 giả lập, giao diện telemetry và các công cụ chẩn đoán"],
        ["Ngày chụp source", "08 tháng 09 năm 2026"],
        ["Workspace", str(ROOT)],
        ["Commit repository chính", git(["rev-parse", "--short", "HEAD"], ROOT)],
        ["Commit giao diện", git(["rev-parse", "--short", "HEAD"], UI)],
        ["Trạng thái", "Tài liệu phản ánh file đang có trên đĩa, bao gồm các thay đổi chưa commit trong repository chính"],
    ]
    add_table(doc, ["Thông tin", "Giá trị"], meta, [1.7, 5.2], 9)
    p = doc.add_paragraph()
    p.add_run("Kết luận chính  ").bold = True
    p.add_run("Hệ thống đã tách rõ điều khiển chuyển động trên Teensy và điều khiển cơ cấu trên ESP32. Cả hai firmware cùng giao diện đều build thành công tại thời điểm lập báo cáo. Tuy nhiên source chính đang có nhiều thay đổi chưa commit, một số state và hằng số kế thừa không còn nằm trong tuyến AUTO đang chạy, và tài liệu giao diện còn hai nhãn lưu EEPROM không đúng với hành vi RAM only của Teensy. Các điểm này cần được quản lý trước khi chốt bản thi đấu.")
    doc.add_page_break()

    add_heading(doc, "Mục lục", 1)
    doc.add_paragraph("[[TOC]]")
    doc.add_page_break()

    add_heading(doc, "1 Phạm vi và trạng thái xác minh", 1)
    doc.add_paragraph("Báo cáo này mô tả source đang hoạt động trong workspace, không chỉ mô tả commit Git gần nhất. Cách đọc này cần thiết vì firmware cơ cấu ESP32 và nhiều mô đun mới của Teensy đang ở trạng thái chưa được commit trong repository chính. Các số liệu build bên dưới được kiểm tra trực tiếp từ source hiện tại.")
    add_table(doc, ["Khối", "Kết quả build", "Thông tin chính"], [
        ["Teensy 4.1", "Thành công", "Platform Teensy 5.1.0; code 156176 byte; data 44540 byte; RAM1 còn 300864 byte; RAM2 còn 511872 byte"],
        ["ESP32 S3 cơ cấu", "Thành công", "Espressif32 6.11.0; RAM 20596 trên 327680 byte; flash 328325 trên 3342336 byte"],
        ["Giao diện", "Thành công", "vinext build hoàn tất cho route gốc"],
    ], [1.35, 1.35, 4.2], 9)
    add_heading(doc, "1 1 Các giới hạn của kết quả xác minh", 2)
    add_bullets(doc, [
        "Build thành công xác nhận cú pháp, liên kết thư viện và kích thước chương trình; build không thay thế thử nghiệm phần cứng dưới tải.",
        "Không có phép đo mới về chiều motor, cực tính cảm biến, khoảng cách thực tế hoặc đáp ứng PID trong quá trình lập tài liệu.",
        "Firmware có cơ chế bảo vệ bằng fault, watchdog và giới hạn phần mềm, nhưng E STOP vật lý độc lập chưa xuất hiện trong source Teensy hiện tại; biến dừng hiện tại là chốt E STOP do lệnh serial.",
        "Map_do là project ESP32 cơ cấu thật hiện tại. ESP32_MECHANISM_SIMULATOR là project thay thế để giả lập DONE sau 4 giây.",
    ])

    add_heading(doc, "2 Kiến trúc hệ thống", 1)
    doc.add_paragraph("Máy tính mở giao diện trong Chrome hoặc Edge và trao đổi với Teensy qua Web Serial. Teensy chịu trách nhiệm thời gian thực cho chuyển động, cảm biến, odometry và tuyến AUTO. ESP32 nhận lệnh cơ cấu từ Teensy qua UART riêng, phát xung cho hai động cơ bước, điều khiển năm van và trả trạng thái hoặc DONE.")
    add_table(doc, ["Liên kết", "Đầu thứ nhất", "Đầu thứ hai", "Tốc độ", "Nội dung"], [
        ["Telemetry", "Máy tính COM5", "Teensy Serial5 RX21 TX20", "57600 8N1", "RBT 1 ASCII, lệnh điều khiển, cấu hình, telemetry"],
        ["Cơ cấu", "Teensy Serial6 TX24 RX25", "ESP32 UART2 RX16 TX15", "57600 8N1", "Lệnh profile, jog, van, config, STATUS, DONE, ERR"],
        ["Bảo trì Teensy", "Máy tính COM14", "USB native Teensy", "115200", "Đường nhận lệnh dự phòng; phản hồi chính vẫn được in qua Serial5 do macro Serial"],
        ["Bảo trì ESP32", "Máy tính USB", "USB Serial ESP32", "115200", "Dùng cùng parser với UART2 và nhận bản sao phản hồi"],
    ], [1.05, 1.65, 1.7, 0.95, 2.1], 8.4)
    add_figure(doc, diagrams["control"], "Hình 2  Các tầng điều khiển trong chu kỳ 10 ms", 6.9,
               "Chu kỳ đọc cảm biến, odometry, điều khiển ngoài, động học Mecanum, PID bánh và phản hồi encoder.")

    add_heading(doc, "3 Bố cục source code", 1)
    inventory = [
        ["firmwave ROBOCON KOSEN F0 include RobotConfig h", "Cấu hình tập trung Teensy gồm chân, dấu chiều, PID, line, ToF, AUTO, UART và an toàn"],
        ["firmwave ROBOCON KOSEN F0 src main cpp", "Firmware thi đấu Teensy gồm toàn bộ vòng điều khiển và state machine"],
        ["firmwave ROBOCON KOSEN F0 src mechanism", "Client UART6 và bộ giải mã trạng thái cơ cấu ESP32"],
        ["firmwave ROBOCON KOSEN F0 src communication", "Khung nhị phân RBT 2 có CRC; hiện không được nối vào main RBT 1 ASCII"],
        ["firmwave ROBOCON KOSEN F0 src READ và TUNE", "Các firmware độc lập để đọc line, ToF, optical flow, encoder và tune bốn bánh"],
        ["firmwave Map_do", "Firmware ESP32 S3 cơ cấu thật gồm controller, profile, van, UART và NVS"],
        ["firmwave ESP32_MECHANISM_SIMULATOR", "Firmware giả lập nhận lệnh và trả DONE sau 4 giây"],
        ["pid-tuner app page tsx", "Giao diện chính, Web Serial, parser, log chuyển động, xuất Excel và các trang chức năng"],
        ["pid-tuner app MechanismJog tsx", "Hai cần nhấn giữ cho động cơ bước A và B"],
        ["pid-tuner app mechanism-jog-controller ts", "Bộ phát latest state only 120 ms và hủy lệnh cũ để giảm lag"],
        ["pid-tuner worker và vite config", "Lớp deploy Cloudflare và tích hợp Sites"],
    ]
    add_table(doc, ["Đường dẫn logic", "Vai trò"], inventory, [3.0, 3.9], 8.7)

    add_heading(doc, "4 Phần cứng và sơ đồ chân", 1)
    add_heading(doc, "4 1 Chân Teensy 4 1", 2)
    teensy_pins = [
        ["Motor FL", "RPWM 36  LPWM 37", "PWM 20 kHz"], ["Motor FR", "RPWM 22  LPWM 23", "MOTOR SIGN bằng âm"],
        ["Motor BL RL", "RPWM 18  LPWM 19", "Kênh sau đã được đổi nhận dạng vật lý"], ["Motor BR RR", "RPWM 14  LPWM 15", "Kênh sau đã được đổi nhận dạng vật lý"],
        ["Encoder FL", "A0  B1", "ENCODER SIGN âm"], ["Encoder FR", "A6  B7", "ENCODER SIGN dương"],
        ["Encoder BL RL", "A2  B3", "ENCODER SIGN dương"], ["Encoder BR RR", "A4  B5", "ENCODER SIGN âm"],
        ["MCP3008 center", "CS26  SPI0 11 12 13", "8 mắt giữa CH0 đến CH7"], ["MCP3008 stop", "CS8  SPI0 11 12 13", "6 mắt dừng cùng H1 H2 CH6 CH7"],
        ["I2C Wire1", "SDA17  SCL16", "BNO085 0x4A và VL53L3CX 0x29 dùng chung bus 100 kHz"],
        ["Telemetry", "RX21  TX20", "Serial5 57600"], ["ESP32", "TX24  RX25", "Serial6 57600"],
        ["Nút START", "34", "Active low với INPUT PULLUP"], ["Nút đổi map", "35", "Một nhấn RED  hai nhấn BLUE"],
        ["Buzzer", "40", "Active high qua AO3400A"],
    ]
    add_table(doc, ["Khối", "Chân", "Ghi chú"], teensy_pins, [1.6, 2.1, 3.2], 8.5)
    add_heading(doc, "4 2 Chân ESP32 S3", 2)
    add_table(doc, ["Khối", "GPIO", "Ghi chú"], [
        ["UART2", "RX16  TX15", "Nối chéo với Teensy TX24  RX25 và chung GND"],
        ["Stepper A", "STEP1  DIR2", "AccelStepper DRIVER"], ["Stepper B", "STEP45  DIR48", "GPIO45 là strap và GPIO48 dùng chung LED trên một số board"],
        ["Van 1 đến 5", "4 5 6 7 8", "Active high, safe mask 0; source yêu cầu xác nhận dây trước khi cấp khí"],
        ["Home và cảm biến cơ cấu", "Chưa gán", "HOME chỉ về zero phần mềm; E18 và line input đang bằng âm một"],
    ], [1.7, 1.5, 3.7], 8.8)

    add_heading(doc, "5 Firmware Teensy 4 1", 1)
    add_heading(doc, "5 1 Khởi tạo", 2)
    add_numbered(doc, [
        "Mở USB Serial 115200, cấu hình Serial5 telemetry và Serial6 cơ cấu.",
        "Khởi tạo buzzer và hai nút active low.",
        "Khởi tạo SPI, hai chân CS và bốn cặp PWM ở 20 kHz.",
        "Nạp PID, KFF và deadzone đã biên dịch từ RobotConfig h vào RAM.",
        "Sao chép min max hiệu chuẩn line vào mảng runtime.",
        "Khởi tạo Wire1 100 kHz, thử BNO085 tối đa ba lần và đặt yaw tương đối về zero.",
        "Khởi tạo VL53L3CX ở long distance, timing budget 100 ms và bắt đầu đo liên tục.",
        "Reset pose, phát thông tin boot và giữ robot không chạy cho đến khi có lệnh hợp lệ.",
    ])
    add_heading(doc, "5 2 Vòng lặp hợp tác", 2)
    doc.add_paragraph("Hàm loop không dùng delay trong đường chạy bình thường. Mỗi vòng lặp cập nhật parser USB và telemetry, UART ESP32, optical flow, BNO085, ToF, nút và buzzer. Cứ đủ 10000 micro giây, firmware đọc line, encoder, odometry, kiểm tra fault, cập nhật test hoặc AUTO rồi chạy PID bánh. Telemetry được xử lý sau khối điều khiển.")
    add_code(doc, "loop -> UART -> ESP32 -> Flow -> BNO085 -> ToF -> Nút và buzzer -> chu kỳ 10 ms -> Telemetry")

    add_heading(doc, "5 3 Động học Mecanum và giới hạn", 2)
    add_figure(doc, diagrams["kin"], "Hình 3  Công thức target bốn bánh", 6.8,
               "Bốn công thức vận tốc bánh cho cấu hình Mecanum X và cơ chế chuẩn hóa chung.")
    doc.add_paragraph("Trong công thức, Vx dương là tiến, Vy dương là sang phải theo quy ước nội bộ và Wz dương là quay theo chiều kim đồng hồ. k bằng nửa tổng khoảng cách tâm bánh trước sau và trái phải, hiện bằng 305 mm. Nếu một target vượt giới hạn bánh, setRobotVelocity chia đồng thời cả bốn target cho cùng một hệ số. Sau đó target được ramp đồng bộ với gia tốc 1400 mm trên giây bình phương và giảm tốc 2400 mm trên giây bình phương, nhờ đó tỷ lệ hướng Mecanum không bị phá bởi việc kẹp từng bánh độc lập.")

    add_heading(doc, "5 4 Vòng PID tốc độ bốn bánh", 2)
    add_table(doc, ["Bánh", "Kp", "Ki", "Kd", "KFF", "Deadzone PWM", "Motor sign", "Encoder sign"], [
        ["FL", "0.15", "0.08", "0", "0.035", "36", "+1", "-1"],
        ["FR", "0.14", "0.08", "0", "0.045", "34", "-1", "+1"],
        ["BL RL", "0.14", "0.12", "0", "0.034", "34", "+1", "+1"],
        ["BR RR", "0.15", "0.10", "0", "0.034", "34", "-1", "-1"],
    ], [0.65, 0.55, 0.55, 0.45, 0.65, 0.95, 0.85, 0.95], 8.1)
    doc.add_paragraph("Encoder được đổi từ count sang milimet bằng chu vi bánh 100 mm chia 300 count mỗi vòng. Tốc độ đo được lọc IIR với alpha 0.35. PWM bằng KFF nhân target cộng deadzone theo dấu, sau đó cộng hiệu chỉnh PID. PID dùng dt, giới hạn tích phân 250 và anti windup theo bão hòa đầu ra. Nếu target lớn hơn 100 mm trên giây nhưng encoder không đổi trong 600 ms, firmware chốt FAULT ENCODER và dừng.")

    add_heading(doc, "5 5 Các vòng điều khiển ngoài", 2)
    loops = [
        ["Heading thường", "Kp 0.101  Ki 0  Kd 0.004", "Giữ yaw; deadband 0.50 độ; Wz tối thiểu 0.18; max 1.2 rad trên giây"],
        ["Heading fine", "Kp 0.120  Kd 0.003", "Căn góc chính xác; deadband 0.25 độ; max 0.35"],
        ["Heading ngang", "Fine PID cộng slew", "Deadband 0.45 độ; min 0.06; max 0.20; slew 0.80 rad trên giây bình phương"],
        ["Heading tại trạm", "Fine PID cộng slew", "Deadband 0.45 độ; min 0.05; max 0.16; slew 0.55"],
        ["Position X và Y", "Mỗi trục 1.40  0.05  0.01", "MOVE tương đối và GOTO tuyệt đối; ngưỡng hoàn thành 18 mm"],
        ["Line center", "0.16  0.02  0.003", "Sai số weighted của 8 mắt thành Vy; giảm Vx khi line lệch về mép"],
        ["ToF", "0.80  0  0", "Căn khoảng cách target 160 mm; hiện không chặn A và B"],
        ["Stop forward", "0.20  0.01  0.002", "Căn theo trung bình vị trí hai cụm trong hàm stop alignment kế thừa"],
        ["Stop yaw", "0.0018  0.0001  0.00002", "Căn chênh lệch hai cụm line kế thừa"],
        ["Cụm 3 mắt trái phải", "Kp 0.08", "Kéo cặp mắt mục tiêu về vị trí 500 tại B"],
    ]
    add_table(doc, ["Vòng", "Hệ số hiện tại", "Vai trò"], loops, [1.45, 1.8, 3.65], 8.5)

    add_heading(doc, "5 6 BNO085 và nhận diện dốc cầu", 2)
    doc.add_paragraph("BNO085 phát Game Rotation Vector mỗi 10 ms. Firmware đổi quaternion thành yaw, roll và pitch. Yaw được đặt tương đối theo yawReferenceDeg; tốc độ quay được lọc. Roll và pitch dùng alpha 0.35. Trên cầu, robot chụp roll và pitch nền lúc bắt đầu bám line, chọn trục nghiêng lớn hơn khi độ nghiêng vượt 5 độ trong 150 ms, tăng tốc lên 1300 mm trên giây khi đang leo, nhận đỉnh khi trục nghiêng trở lại dưới 4 độ trong 120 ms sau tối thiểu 300 ms leo, rồi giảm còn 400 mm trên giây để xuống.")

    add_heading(doc, "5 7 Cảm biến line", 2)
    add_table(doc, ["Cụm", "Kênh vật lý", "Thứ tự logic", "Công dụng"], [
        ["8 mắt giữa", "MCP CS26 CH0 đến CH7", "Trọng số âm 3500 đến dương 3500", "Bám line lên cầu và nhận vạch ngang khi ít nhất 6 mắt active"],
        ["3 mắt trái", "MCP CS8 CH3 CH4 CH5", "Ngoài âm 1000  giữa 0  trong dương 1000", "Căn điểm theo map"],
        ["3 mắt phải", "MCP CS8 CH0 CH1 CH2", "Ngoài âm 1000  giữa 0  trong dương 1000", "RED dùng tại A; cụm đối diện dùng tại B"],
        ["H1 H2", "MCP CS8 CH6 CH7", "H1 ở đuôi  H2 ở đầu", "Giữ trôi dọc khi chạy ngang và tìm line dọc"],
    ], [1.2, 1.65, 2.0, 2.05], 8.4)
    doc.add_paragraph("Line đen cho ADC cao hơn nền trắng. Dữ liệu được chuẩn hóa 0 đến 1000 và cảm biến được xem là HIGH trên line khi normalized đạt ngưỡng. Ngưỡng chung là 550; riêng phát hiện nhanh pick line dùng 400. Center có min max đã hiệu chuẩn; sáu mắt stop cũng có min max đã hiệu chuẩn. H1 H2 vẫn dùng min 0 max 1023 mặc định và nên được hiệu chuẩn riêng để tăng độ tin cậy.")

    add_heading(doc, "5 8 VL53L3CX", 2)
    doc.add_paragraph("VL53L3CX dùng địa chỉ 0x29 trên Wire1 chung với BNO085. Firmware chọn vật thể hợp lệ gần nhất trong vùng 35 đến 800 mm, loại bước nhảy lớn hơn 250 mm và lọc median 5 mẫu. Timeout mẫu là 1500 ms. Tại A và B, ToF chỉ được phát bản tin thông tin và không quyết định chuyển state. Target vẫn là 160 mm, dung sai 30 mm. Luồng xuất phát hiện tại tắt nhận ToF trong đoạn tiến 170 mm vì cảm biến ở quá gần tường.")

    add_heading(doc, "5 9 Odometry và MOVE", 2)
    doc.add_paragraph("Odometry Mecanum tính dx và dy trong thân robot từ bốn delta encoder, dùng BNO085 làm yaw khi hợp lệ và xoay delta sang hệ tọa độ thế giới. Optical flow đang tắt, do đó trọng số odometry hiện là encoder 1.0 và flow 0.0. MOVE nhận dx dy tương đối trong hệ tọa độ tại thời điểm bắt đầu. GOTO nhận X Y yaw tuyệt đối trong hệ pose hiện tại. Cả hai đều yêu cầu BNO085 và heartbeat; mất heartbeat sau 600 ms sẽ dừng.")

    add_heading(doc, "5 10 Nút vật lý và buzzer", 2)
    add_table(doc, ["Tác động", "Xử lý"], [
        ["Nhấn START chân 34", "Kêu 1 tiếng, kiểm tra idle, E STOP, fault, BNO và cấu hình; tự chọn AUTO rồi vào CAN START dù không có telemetry"],
        ["Nhấn map chân 35 một lần", "Sau cửa sổ 450 ms chọn sân RED và kêu 2 tiếng"],
        ["Nhấn map hai lần", "Chọn sân BLUE và kêu 3 tiếng"],
        ["Đến A hoặc B và bắt đầu chờ cơ cấu", "Kêu 2 tiếng"],
        ["Fault trong nhóm state căn A hoặc B", "Buzzer chớp tắt 120 ms trong 2 giây"],
    ], [2.5, 4.4], 8.8)
    doc.add_paragraph("Hai nút dùng INPUT PULLUP nên thả là HIGH và nhấn nối GND là LOW. Debounce bằng thời gian, buzzer dùng hàng đợi và không chặn vòng điều khiển.")

    add_heading(doc, "5 11 Fault và watchdog", 2)
    add_table(doc, ["Bit", "Tên", "Nguyên nhân điển hình", "Tác động"], [
        ["0", "IMU TIMEOUT", "BNO không có mẫu quá 250 ms", "AUTO dừng; manual DRIVE có thể bỏ qua riêng bit này"],
        ["1", "FLOW TIMEOUT", "Optical flow mất dữ liệu", "Hiện flow tắt nên không dùng trong tuyến chính"],
        ["2", "LINE LOST", "Mất center line quá 650 ms hoặc H1 H2 mất quá thời gian", "Dừng và FAULT STOP"],
        ["3", "TOF TIMEOUT", "ToF bắt buộc ở hàm căn ToF nhưng mất quá 3 s", "Dừng"],
        ["4", "ENCODER", "Có target trên 100 nhưng encoder đứng 600 ms", "Dừng bốn bánh"],
        ["5", "ESP TIMEOUT", "Cơ cấu REAL không trả lời trong state chờ", "Dừng AUTO"],
        ["6", "STATE TIMEOUT", "State vượt thời gian cho phép", "Dừng AUTO"],
        ["7", "CONFIG", "Cấu hình không đủ hoặc tune bị khóa", "Không cho motor chạy"],
    ], [0.5, 1.25, 3.0, 2.15], 8.2)

    add_heading(doc, "6 Luồng AUTO hiện tại", 1)
    add_figure(doc, diagrams["auto"], "Hình 4  Tuyến AUTO thực sự được gọi từ START", 6.7,
               "Chuỗi state từ WAIT START qua A, B, tìm line cầu, bám line và FINISH.")
    doc.add_paragraph("Sân RED đặt dấu chạy ngang là dương và dùng cụm 3 mắt phải tại A. Sân BLUE đảo dấu chạy ngang và dùng cụm trái tại A. B dùng cụm đối diện. Heading cạnh tranh được chụp đúng lúc START và giữ xuyên suốt A và B.")
    active_states = [
        ["WAIT START", "Robot DISARM", "START UART hoặc nút 34", "CAN START"],
        ["CAN START", "Tiến 170 mm encoder ở 280", "MOVE hoàn thành", "DI SANG TRAI A"],
        ["DI SANG TRAI A", "Tìm pick line A quanh 1075 cộng trừ 200 mm", "Đếm đủ 2 line, encoder trong cửa sổ, mắt giữa và trong đã đi qua", "BU TAM A"],
        ["BU TAM A", "Kéo chậm về line", "Mắt giữa và trong của cụm A cùng H1 H2 ổn định 180 ms", "CAN YAW A"],
        ["CAN YAW A", "Xác nhận line và ghi ToF", "Line còn đúng", "CAN A"],
        ["CAN A", "Căn heading tại chỗ", "X Y đã chốt và heading ổn định", "DOI GAP A"],
        ["DOI GAP A", "Gửi POINT A hoặc SIM POINT A", "DONE hoặc đủ 4 s", "DI SANG TRAI B"],
        ["DI SANG TRAI B", "Reset pose rồi chạy ngang 200 mm", "Relative move hoàn thành", "BU TAM B"],
        ["BU TAM B", "Căn cụm đối diện", "Mắt 0 và dương 1000 cùng nằm trên line 180 ms", "CAN YAW B"],
        ["CAN YAW B", "Chốt B và ghi ToF", "Ngay sau ghi", "CAN B"],
        ["CAN B", "Căn heading tại chỗ", "Heading ổn định", "DOI GAP B"],
        ["DOI GAP B", "Gửi POINT B hoặc SIM POINT B", "DONE hoặc đủ 4 s", "DI SANG C"],
        ["DI SANG C", "Khóa cụm 3 mắt và đi trái 1300 mm ở 800", "MOVE hoàn thành", "CHAY LEN XANH"],
        ["CHAY LEN XANH", "Quét trái phải trong bán kính 180 mm và căn center line", "Line hợp lệ và gần tâm 180 ms", "DI LEN DEM LINE"],
        ["DI LEN DEM LINE", "Bám line, tăng giảm ga theo dốc", "Sau đỉnh cầu gặp vạch ngang ít nhất 80 ms", "FINISH"],
        ["FINISH", "PWM về zero, DISARM", "Lệnh RESET hoặc START mới", "WAIT START qua lệnh"],
    ]
    add_table(doc, ["State", "Hành động", "Điều kiện hoàn tất", "State sau"], active_states, [1.15, 2.35, 2.7, 1.15], 7.8)
    add_heading(doc, "6 1 State kế thừa không nằm trong tuyến START hiện tại", 2)
    dormant = ["CAN START FAST", "CAN START FINE", "BU TAM C", "CAN YAW C", "CAN GIUA LINE C", "HOME CENTER", "HOME DROP", "HOME TO B", "TIEN 34CM B", "DOI THA 2B", "CHO SAU B", "LUI 33CM A", "DOI THA 2A", "CHO SAU THA A", "LUI 3M", "SANG TRAI 1M5"]
    doc.add_paragraph("Các state sau vẫn còn trong enum và switch nhưng không được tuyến START hiện tại chuyển tới. Chúng là phần kế thừa hoặc đường thử. Không nên chỉnh tốc độ trong các state này với kỳ vọng tuyến thi đấu sẽ thay đổi:")
    add_bullets(doc, dormant)
    add_heading(doc, "6 2 Hằng số được định nghĩa nhưng không ảnh hưởng tuyến chính", 2)
    add_table(doc, ["Hằng số", "Giá trị", "Nhận xét"], [
        ["START LATERAL DISTANCE MM", "1000", "Không có tham chiếu trong main hiện tại"],
        ["START LATERAL SPEED MM S", "400", "Không có tham chiếu trong main hiện tại"],
        ["TOF START ESCAPE DISTANCE MM", "250", "Không có tham chiếu trong main hiện tại"],
        ["TOF START ESCAPE SPEED MM S", "300", "Không có tham chiếu trong main hiện tại"],
        ["PICK B LINE TARGET", "1", "Tuyến B hiện dùng encoder 200 mm rồi căn cặp mắt, không đếm line"],
        ["BRIDGE LINE TARGET", "3", "Không được dùng; tuyến hiện dừng theo center cross sau đỉnh"],
        ["BRIDGE SPEED FAST SLOW", "1150 và 1000", "Chỉ còn tham chiếu trong file recovery, không được main dùng"],
        ["BRIDGE ACCEL DECEL", "1800 và 700", "Chỉ còn tham chiếu trong file recovery"],
    ], [2.4, 1.15, 3.35], 8.5)

    add_heading(doc, "7 Giao tiếp giữa Teensy và ESP32", 1)
    add_figure(doc, diagrams["uart"], "Hình 5  Cơ chế chờ cơ cấu tại A và B", 6.8,
               "Teensy gửi lệnh qua UART6, ESP32 chạy profile và trả DONE hoặc Teensy chờ timer ở chế độ SIM.")
    doc.add_paragraph("Chế độ REAL là mặc định sau mỗi lần reset Teensy. Ở chế độ này, robot chỉ rời state chờ khi nhận chuỗi chứa DONE. Nếu sau 5.5 giây chưa DONE, Teensy gửi lại POINT A hoặc POINT B với khoảng cách giữa hai lần gửi ít nhất 5 giây. Timeout state là 15 giây. Chế độ SIM 4S gửi khung SIM để ESP32 xác nhận nhưng không chạy cơ cấu; Teensy tự coi hoàn thành khi đủ 4000 ms.")
    add_table(doc, ["Teensy gửi", "ESP32 cơ cấu thật", "Phản hồi quyết định"], [
        ["POINT A", "Chạy profile PICK A", "DONE PICK A"],
        ["POINT B", "Chạy profile PICK B", "DONE PICK B"],
        ["THA 2A hoặc THA 2B", "Chạy profile RETRACT", "DONE RETRACT"],
        ["SIM lệnh", "Chỉ ACK NO MOTION", "Teensy tự hết 4 giây"],
        ["STOP", "Dừng stepper và đưa van về safe mask", "ACK STOPPED SAFE"],
    ], [1.65, 2.7, 2.55], 8.7)

    add_heading(doc, "8 Firmware ESP32 S3 cơ cấu", 1)
    add_heading(doc, "8 1 Cấu trúc chạy", 2)
    doc.add_paragraph("main cpp chỉ khởi tạo USB Serial, UART2, đọc cấu hình NVS hoặc nạp mặc định, khởi tạo MechanismController và gọi protocol update liên tục. UartProtocol đọc đồng thời UART2 và USB bằng hai buffer cố định 256 byte. Mỗi lần update, controller được gọi trước và sau khi đọc lệnh để AccelStepper run có cơ hội phát xung đều.")
    add_heading(doc, "8 2 Cấu hình profile", 2)
    add_table(doc, ["Profile", "A mm", "B mm", "Dir A", "Dir B", "Speed step s", "Accel step s2", "Chờ ms", "Mode"], [
        ["ROBOT START", "10", "0", "-1", "+1", "5000 5000", "3500 3500", "300", "Tương đối"],
        ["PICK A", "500", "500", "-1", "-1", "5000 5000", "3500 3500", "3000", "Tương đối"],
        ["PICK B", "615", "500", "-1", "+1", "5000 5000", "3500 3500", "3000", "Tương đối"],
        ["RETRACT", "2200", "2200", "+1", "-1", "5000 5000", "3500 3500", "3000", "Tương đối"],
        ["HOME", "0", "0", "+1", "+1", "5000 5000", "3500 3500", "0", "Tuyệt đối"],
    ], [1.05, 0.55, 0.55, 0.55, 0.55, 1.1, 1.1, 0.65, 0.8], 7.7)
    doc.add_paragraph("Mỗi profile mặc định có ba bước: phát lệnh move, chờ hai stepper về target, rồi chờ post wait. Khung dữ liệu cho phép tối đa tám bước và enum đã hỗ trợ bật van, chờ thời gian, chờ stepper, chờ cảm biến và kết thúc. Tuy nhiên profile mặc định hiện không có bước ValveSet hoặc WaitSensor.")
    add_heading(doc, "8 3 Tính vị trí và đồng bộ hai trục", 2)
    doc.add_paragraph("Số step trên milimet bằng step trên vòng nhân hệ số hiệu chỉnh rồi chia bước vít. Mặc định A dùng 200 step trên vòng và vít 12 mm trên vòng; B dùng 200 và 10. Khi hai trục có quãng đường khác nhau, firmware chỉ giảm tốc trục ngắn hơn để thời gian chạy gần bằng trục dài hơn, không tăng quá giới hạn đã cấu hình. Chuyển profile vẫn giữ tọa độ milimet bằng cách đổi currentPosition theo scale mới.")
    add_heading(doc, "8 4 Manual jog", 2)
    doc.add_paragraph("JOG nhận tốc độ có dấu riêng cho A và B trong vùng 10 đến 30000 step trên giây. Giao diện gửi lại mỗi 120 ms. ESP32 yêu cầu có lệnh mới trong 500 ms; quá thời gian sẽ dừng và chốt JogLinkTimeout. Đổi chiều làm tốc độ dư được xóa trước khi ramp lại. Lệnh JOG STOP dừng hai trục nhưng không phát DONE và không thay đổi van.")
    add_heading(doc, "8 5 Van và interlock", 2)
    doc.add_paragraph("Năm van active high khởi động ở safe mask 0. writeOne ghi mức safe trước khi đổi chân sang OUTPUT để giảm xung kích lúc boot. Interlock hiện đều bằng 0, nghĩa là source chưa cấm cặp van nào bật cùng lúc. Nếu mạch dùng hai coil đối nghịch, cần điền VALVE INTERLOCK MASK trước khi chạy khí thật.")
    add_heading(doc, "8 6 NVS và CRC", 2)
    doc.add_paragraph("MechanismConfig có magic MECH, version, size, giới hạn vị trí, năm profile và CRC32. SAVE kiểm tra phạm vi, tính CRC, ghi namespace mechanism key config, đọc lại toàn bộ, so sánh byte và kiểm CRC lần nữa. Nếu load thất bại, ESP32 nạp mặc định vào RAM nhưng không tự ghi NVS. RESTORE DEFAULT cũng chỉ nạp RAM cho đến khi có SAVE.")
    add_heading(doc, "8 7 Fault cơ cấu", 2)
    add_table(doc, ["Mã enum", "Tên", "Ý nghĩa"], [
        ["0", "None", "Không lỗi"], ["1", "Busy", "Dùng cho dừng bất thường có reason khác STOP hoặc BOOT SAFE"],
        ["2", "InvalidProfile", "Profile không hợp lệ"], ["3", "PositionLimit", "Target vượt giới hạn phần mềm"],
        ["4", "MotionTimeout", "Profile hoặc direct move quá deadline"], ["5", "SensorUnavailable", "Profile yêu cầu sensor chưa gán chân"],
        ["6", "SensorTimeout", "Sensor không đạt trạng thái đúng hạn"], ["7", "ValveInterlock", "Hai van xung đột"],
        ["8", "NotZeroed", "HOME khi chưa SET ZERO và không có công tắc home"], ["9", "JogLinkTimeout", "Mất refresh JOG trên 500 ms"],
    ], [0.8, 1.7, 4.4], 8.7)

    add_heading(doc, "9 Firmware ESP32 giả lập", 1)
    doc.add_paragraph("Project ESP32 MECHANISM SIMULATOR dùng cùng UART2 RX16 TX15 ở 57600. Nó nhận POINT A, POINT B, THA 2A, THA 2B hoặc SIM lệnh. Với lệnh hợp lệ, timer không chặn bắt đầu, LED bật và sau 4 giây gửi đúng một dòng DONE. Lệnh lặp khi bận bị bỏ qua nên không làm timer chạy lại. STOP hoặc ESTOP hủy pending ngay. Firmware này không điều khiển stepper hoặc van.")

    add_heading(doc, "10 Giao diện Web Serial", 1)
    add_heading(doc, "10 1 Công nghệ và kết nối", 2)
    doc.add_paragraph("Giao diện dùng React 19, Next 16, TypeScript, vinext và Vite. Bản deploy chạy qua Cloudflare Worker và Sites. Web Serial mở cổng ở baud do người dùng chọn, mặc định 57600 và buffer 8192 byte. Khi kết nối, giao diện gửi PING, GET CONFIG, MECH GET STATUS và MECH GET CONFIG. Nếu chỉ nhận mà không có chiều gửi, giao diện cảnh báo kiểm tra TXD bộ chuyển sang RX5 chân 21.")
    add_heading(doc, "10 2 Các trang chức năng", 2)
    add_table(doc, ["Trang", "Chức năng chính"], [
        ["Tổng quan", "Hiển thị bốn bánh, heading, line và ToF"],
        ["Điều khiển Mecanum", "DRIVE nhấn giữ, heading manual, MOVE tương đối, pose, log và tải Excel"],
        ["AUTO Setup sân", "Chọn map, REAL hoặc SIM 4S, telemetry mode, GOTO, snapshot, START, DISARM, E STOP"],
        ["CƠ CẤU ESP32", "Profile, trạng thái, hai cần jog, năm van, SET ZERO, config RAM và SAVE NVS"],
        ["PID 4 bánh", "Chọn một bánh, PID, target RPM, chạy thuận nghịch và heartbeat an toàn"],
        ["Heading PID", "PID heading, giới hạn Wz, deadband, Wz tối thiểu và giới hạn tốc độ bánh"],
        ["Line PID", "Đọc raw normalized, hiệu chuẩn 8 mắt, 6 mắt và xem H1 H2"],
        ["Cấu hình", "GET CONFIG, SAVE CONFIG, PROFILE LOAD và FACTORY RESET"],
        ["Chẩn đoán", "Trạng thái liên kết, RX TX, state, fault và ARM"],
    ], [1.8, 5.1], 8.8)
    add_heading(doc, "10 3 Cơ chế chống lag", 2)
    doc.add_paragraph("Parser đọc mọi dòng để ghi log, nhưng UI chỉ render theo batch 40 ms. Các record tần suất cao như WHEEL được ghi đè bằng bản mới nhất theo key; record sự kiện như STATE, ACK và ERR vẫn giữ thứ tự. Các dòng không cần cho giao diện như MOTION, DBG, TILT, STOP6, HOLD2, PICK SCAN và BRIDGE SCAN bị bỏ khỏi hàng render. Hàng lệnh serial được tuần tự hóa. DRIVE và JOG dùng latest state only để không tích tụ lệnh cũ phía sau thao tác ghi chậm.")
    add_heading(doc, "10 4 Ghi log và xuất Excel", 2)
    doc.add_paragraph("Giao diện tự ghi mẫu khi MOTION báo active. Mỗi mẫu giữ thời gian PC và robot, mode, map, state, fault, ARM, Vx Vy Wz, sai số vị trí, pose, heading, ToF và target actual error PWM encoder của từng bánh. File tải về là workbook XML Excel có sheet Tổng quan và Dữ liệu 4 bánh. Tốc độ thực tế robot được suy ra từ RPM bốn bánh và đường kính 100 mm; đây là tốc độ suy ra từ encoder, không phải tốc độ đo độc lập ngoài robot.")

    add_heading(doc, "11 Danh mục lệnh điều khiển", 1)
    teensy_commands = [
        ["PING", "Bắt tay và nhận HELLO"], ["GET CONFIG", "Đọc PID, giới hạn, mode, map và cấu hình AUTO"],
        ["HEARTBEAT", "Giữ tune, MOVE, GOTO hoặc test giám sát"], ["MODE MANUAL hoặc AUTO", "Đổi mode khi DISARM"],
        ["FIELD RED hoặc BLUE", "Chọn map khi DISARM"], ["START hoặc RUN", "Chạy tuyến AUTO hiện tại"],
        ["START TEST", "AUTO có heartbeat giám sát"], ["START BRIDGE TEST", "Bắt đầu từ đoạn sau B để thử cầu"],
        ["ARM", "Chỉ test khóa heading AUTO, không chạy tuyến"], ["STOP hoặc DISARM", "Dừng và về WAIT START"],
        ["ESTOP", "Chốt E STOP phần mềm và gửi STOP sang ESP32"], ["RESET", "Xóa E STOP phần mềm và fault"],
        ["DRIVE vx vy wz", "Điều khiển vận tốc manual; phải refresh trong 500 ms"], ["MOVE dx dy maxSpeed", "Vị trí tương đối"],
        ["GOTO x y yaw maxSpeed", "Vị trí tuyệt đối ở mode AUTO"], ["POSE RESET", "Đặt pose về zero khi DISARM"],
        ["MANUAL HEADING ON OFF", "Bật tắt giữ heading manual"], ["MANUAL YAW degrees", "Đặt yaw manual"],
        ["PID WHEEL name kp ki kd", "Đổi PID bánh trong RAM"], ["SET PID index kp ki kd kff deadzone", "Đổi đầy đủ điều khiển một bánh trong RAM"],
        ["PID HEADING", "Đổi PID heading trong RAM"], ["PID LINE", "Đổi PID line trong RAM"],
        ["LIMITS HEADING", "Đổi max Wz, deadband và min Wz khi DISARM"], ["LIMITS MOTOR", "Đổi giới hạn bánh 100 đến 2000 mm trên giây"],
        ["TUNE WHEEL", "Quay một bánh theo target RPM"], ["TUNE STOP", "Dừng tune"],
        ["TEST LINE CENTER STOP", "Bật dữ liệu raw line tương ứng"], ["CAL CENTER hoặc CAL STOP", "Bắt đầu và kết thúc hiệu chuẩn"],
        ["AUTO MECH REAL hoặc SIM 4S", "Chọn cơ cấu thật hoặc giả lập"], ["AUTO TELEM ON REQUEST hoặc SILENT", "Chọn chính sách telemetry AUTO"],
        ["AUTO TELEM SNAPSHOT", "Lấy một snapshot phân mảnh"], ["MECH command", "Chuyển command nguyên văn xuống ESP32"],
    ]
    add_table(doc, ["Lệnh Teensy", "Chức năng"], teensy_commands, [2.65, 4.25], 8.3)
    esp_commands = [
        ["PING", "HELLO ESP32 MECHANISM"], ["GET STATUS", "Trạng thái, vị trí, van, profile, step, zero và jog"],
        ["GET CONFIG", "Header CRC và năm profile"], ["CMD profile", "Chạy ROBOT START, PICK A, PICK B, RETRACT hoặc HOME"],
        ["POINT A POINT B", "Alias trực tiếp cho PICK A và PICK B"], ["THA 2A THA 2B", "Alias trực tiếp cho RETRACT"],
        ["JOG speedA speedB", "Điều khiển giữ tốc độ hai trục"], ["JOG STOP", "Dừng manual jog"],
        ["VALVE n state", "Điều khiển van 1 đến 5 khi idle"], ["STOP ESTOP CMD STOP", "Dừng ưu tiên và safe off van"],
        ["SET ZERO", "Đặt tọa độ phần mềm bằng zero"], ["HOME", "Về zero phần mềm"],
        ["SET PROFILE", "Cập nhật một profile trong RAM"], ["SAVE", "Ghi NVS và kiểm CRC"],
        ["RESTORE DEFAULT", "Nạp mặc định vào RAM"], ["CLEAR FAULT", "Xóa fault khi không bận"],
        ["NANG HA SPEED POS", "Tương thích lệnh USB cũ"], ["SIM action", "Xác nhận không chạy cơ cấu"],
    ]
    add_table(doc, ["Lệnh ESP32", "Chức năng"], esp_commands, [2.3, 4.6], 8.4)

    add_heading(doc, "12 Trình tự vận hành và kiểm tra", 1)
    add_heading(doc, "12 1 Trước khi cấp nguồn", 2)
    add_numbered(doc, [
        "Kiểm tra GND chung, TX RX nối chéo, dây BNO, ToF, hai MCP3008 và encoder.",
        "Xác nhận vùng chạy trống, nguồn motor và nguồn van có nút ngắt độc lập, E STOP sẵn sàng.",
        "Đặt cơ cấu trong vùng hành trình an toàn vì chưa có công tắc home hoặc limit trong source.",
        "Đặt robot ở đúng vạch xuất phát và kiểm tra H1 H2 cùng các cụm 3 mắt theo map.",
    ])
    add_heading(doc, "12 2 Khởi động bằng giao diện", 2)
    add_numbered(doc, [
        "Mở giao diện bằng Chrome hoặc Edge, chọn KẾT NỐI TEENSY và COM5 ở 57600 baud.",
        "Chờ HELLO, GET CONFIG và MECH STATUS. Nếu cơ cấu offline, kiểm tra UART6.",
        "Chọn AUTO, map RED hoặc BLUE, chọn REAL hoặc SIM 4S và chính sách telemetry.",
        "Nếu dùng cơ cấu thật, đọc config ESP32, xác nhận zero và profile trước khi chạy.",
        "Giữ robot DISARM và nhấn START AUTO ĐẾN CẦU. Không nhấn ARM trước vì ARM chỉ test khóa đầu.",
        "Theo dõi sự kiện STATE, FAULT và DONE. Dùng snapshot khi cần nhưng không stream AUTO.",
    ])
    add_heading(doc, "12 3 Khởi động không telemetry", 2)
    add_numbered(doc, [
        "Chờ BNO085 khởi tạo và đảm bảo robot không có fault chốt từ lần chạy trước.",
        "Nhấn nút map chân 35 một lần cho RED hoặc hai lần nhanh cho BLUE; nghe 2 hoặc 3 tiếng xác nhận.",
        "Nhấn START chân 34 một lần; nghe một tiếng. Firmware tự chuyển AUTO và chạy nếu đủ điều kiện.",
        "Nếu còi cảnh báo chớp trong 2 giây, dừng cấp lực và kết nối telemetry để đọc fault.",
    ])
    add_heading(doc, "12 4 Quy trình tune", 2)
    add_numbered(doc, [
        "Tune bốn bánh khi kê khỏi sàn trước, sau đó kiểm tra lại có tải ở tốc độ thấp, trung bình và cao.",
        "Xác nhận chiều motor và encoder trước khi thay Kp Ki Kd KFF hoặc deadzone.",
        "Tune KFF và deadzone để đạt gần target, sau đó tăng Kp, thêm Ki vừa đủ và chỉ dùng Kd khi dữ liệu tốc độ đủ sạch.",
        "Tune heading sau khi bốn bánh đồng đều. Tune line sau khi heading ổn và min max cảm biến đúng.",
        "Mọi PID Teensy chỉnh từ giao diện chỉ nằm trong RAM. Chép giá trị đã chốt vào RobotConfig h rồi build lại.",
    ])

    add_heading(doc, "13 Điểm cần kiểm soát trước khi chốt bản thi đấu", 1)
    risks = [
        ["Cao", "Repository chính đang dirty và Map_do chưa được theo dõi", "Có thể mất hoặc nhầm phiên bản source", "Commit theo mốc có tag và ghi checksum firmware hex"],
        ["Cao", "Không có E STOP vật lý độc lập trong source Teensy", "Mất serial hoặc treo MCU có thể không nhận lệnh ESTOP", "Bổ sung ngõ dừng phần cứng hoặc mạch cắt enable nguồn motor"],
        ["Cao", "ESP32 chưa có limit home switch", "HOME dựa trên zero phần mềm và mất mốc sau sự cố", "Gắn limit, homing từng trục và kiểm tra polarity"],
        ["Cao", "GPIO van và interlock chưa xác minh", "Có thể kích sai van hoặc hai coil đối nghịch", "Đo từng ngõ ra khi chưa cấp khí rồi điền interlock"],
        ["Trung bình", "H1 H2 dùng min max mặc định 0 1023", "Ngưỡng thực tế có thể ít biên chống nhiễu", "Hiệu chuẩn riêng H1 H2 và lưu hằng số"],
        ["Trung bình", "Nhiều state và hằng số kế thừa không được dùng", "Người chỉnh nhầm tham số nhưng robot không đổi", "Xóa hoặc gắn nhãn legacy rõ trong code"],
        ["Trung bình", "RBT 2 có source nhưng main dùng RBT 1 ASCII", "Hai giao thức dễ gây hiểu nhầm", "Quyết định giữ RBT 1 hoặc hoàn tất tích hợp RBT 2"],
        ["Trung bình", "Nhãn EEPROM trên UI và protocol sai", "Người vận hành tưởng PID Teensy đã lưu bền", "Đổi nhãn thành RAM only và sửa tài liệu protocol"],
        ["Trung bình", "USB COM14 nhận lệnh nhưng phản hồi dùng macro Serial5", "Khó tune chỉ bằng USB", "Tách hàm emit theo cổng nguồn lệnh hoặc ghi rõ giới hạn"],
        ["Thấp", "README Teensy còn chi tiết VL53L1X và tuyến cũ", "Tài liệu cục bộ không khớp source", "Cập nhật README từ báo cáo này"],
    ]
    add_table(doc, ["Mức", "Vấn đề", "Tác động", "Hướng xử lý"], risks, [0.7, 2.2, 2.05, 2.0], 8.0)

    add_heading(doc, "14 Ma trận tham số chính", 1)
    params = [
        ["Kích thước", "WHEEL DIAMETER MM", "100", "Quy đổi encoder và RPM"],
        ["Kích thước", "ROBOT LENGTH MM", "260", "k Mecanum"], ["Kích thước", "ROBOT WIDTH MM", "350", "k Mecanum"],
        ["Encoder", "COUNTS PER WHEEL REV", "300", "mm per count"], ["Motor", "MAX WHEEL SPEED", "1300 mm/s", "Trần target bánh"],
        ["Motor", "TARGET ACCEL DECEL", "1400  2400", "Khởi động và dừng mềm"], ["Chu kỳ", "CONTROL PERIOD", "10000 us", "100 Hz"],
        ["Heading", "KP KI KD", "0.101  0  0.004", "Giữ yaw"], ["Heading", "LATERAL MAX WZ", "0.20 rad/s", "Đi ngang A B"],
        ["Line", "ACTIVE NORMALIZED", "550", "Logic HIGH trên line đen"], ["Line", "PICK DETECT", "400", "Phát hiện line hẹp tốc độ cao"],
        ["Line", "CENTER CONTROL SIGN", "-1", "Chiều hiệu chỉnh 8 mắt"], ["H1 H2", "CONTROL SIGN", "+1", "Chiều kéo đuôi và đầu về line"],
        ["ToF", "TARGET", "160 mm", "Thông tin tại A B; dùng trong state căn ToF nếu gọi"], ["ToF", "VALID RANGE", "35 đến 800 mm", "Lọc target"],
        ["Xuất phát", "START ENCODER", "170 mm ở 280 mm/s", "Bỏ qua ToF"], ["A", "EXPECTED LATERAL", "1075 cộng trừ 200 mm", "Cửa sổ tìm line"],
        ["A đến B", "DISTANCE", "200 mm ở 300 mm/s", "Relative move"], ["Sau B", "POST B LEFT", "1300 mm ở 800 mm/s", "Tìm line cầu"],
        ["Cầu", "APPROACH ASCENT DESCENT", "1000  1300  400 mm/s", "Theo phase độ nghiêng"],
        ["UART", "TELEMETRY PERIOD", "200 ms", "5 Hz ngoài AUTO ARM"], ["UART", "MANUAL WATCHDOG", "500 ms", "Dừng DRIVE"],
        ["UART", "POSITION WATCHDOG", "600 ms", "Dừng MOVE GOTO"], ["Cơ cấu", "SIM DELAY", "4000 ms", "AUTO mô phỏng"],
        ["Cơ cấu", "REPLY TIMEOUT", "15000 ms", "FAULT ESP"],
    ]
    add_table(doc, ["Nhóm", "Tham số", "Giá trị", "Ảnh hưởng"], params, [1.0, 2.3, 1.5, 2.1], 8.1)

    add_heading(doc, "15 Phụ lục danh mục hàm", 1)
    doc.add_paragraph("Các bảng sau được trích tự động từ source hiện tại. Số dòng giúp tra cứu trong VS Code; số dòng có thể thay đổi sau khi sửa code.")
    function_files = [
        (TEENSY / "src" / "main.cpp", "Teensy main cpp"),
        (TEENSY / "src" / "mechanism" / "Esp32MechanismClient.cpp", "Teensy Esp32MechanismClient cpp"),
        (TEENSY / "src" / "mechanism" / "MechanismTelemetryView.cpp", "Teensy MechanismTelemetryView cpp"),
        (TEENSY / "src" / "communication" / "Protocol.cpp", "Teensy RBT2 Protocol cpp"),
        (ESP32 / "src" / "main.cpp", "ESP32 main cpp"),
        (ESP32 / "src" / "MechanismController.cpp", "ESP32 MechanismController cpp"),
        (ESP32 / "src" / "PersistentConfig.cpp", "ESP32 PersistentConfig cpp"),
        (ESP32 / "src" / "StepperProfiles.cpp", "ESP32 StepperProfiles cpp"),
        (ESP32 / "src" / "UartProtocol.cpp", "ESP32 UartProtocol cpp"),
        (ESP32 / "src" / "ValveController.cpp", "ESP32 ValveController cpp"),
        (SIM / "src" / "main.cpp", "ESP32 simulator main cpp"),
    ]
    for path, label in function_files:
        add_heading(doc, label, 2)
        rows = extract_cpp_functions(path)
        add_table(doc, ["Dòng", "Hàm", "Tham số"], rows, [0.65, 2.55, 3.7], 7.8)
    add_heading(doc, "Giao diện page tsx", 2)
    add_table(doc, ["Dòng", "Hàm hoặc component", "Tham số"], extract_ts_functions(UI / "app" / "page.tsx"), [0.65, 2.4, 3.85], 7.8)
    add_heading(doc, "Giao diện manual jog", 2)
    rows = extract_ts_functions(UI / "app" / "MechanismJog.tsx") + extract_ts_functions(UI / "app" / "mechanism-jog-controller.ts")
    add_table(doc, ["Dòng", "Hàm hoặc phương thức", "Tham số"], rows, [0.65, 2.4, 3.85], 7.8)

    add_heading(doc, "16 Phụ lục toàn bộ hằng số RobotConfig", 1)
    doc.add_paragraph("Bảng này liệt kê mọi constexpr tìm thấy trong RobotConfig h để kiểm soát thay đổi. Giá trị mảng được giữ theo thứ tự source. Hãy ưu tiên bảng Ma trận tham số chính khi vận hành và dùng phụ lục này khi review code.")
    constants = extract_constants(TEENSY / "include" / "RobotConfig.h")
    add_table(doc, ["Dòng", "Tên", "Giá trị trong source"], constants, [0.6, 2.65, 3.65], 7.6)

    add_heading(doc, "17 Thư viện và môi trường build", 1)
    add_table(doc, ["Khối", "Thư viện hoặc framework", "Phiên bản hoặc vai trò"], [
        ["Teensy", "Arduino Teensy", "Framework 1.60 trên Teensy platform 5.1.0"],
        ["Teensy", "Encoder", "1.4.4; đọc encoder quadrature"],
        ["Teensy", "Adafruit BNO08x", "1.2.7; SH2 Game Rotation Vector"],
        ["Teensy", "PWFusion VL53L3C", "1.0.0; multi target ToF"],
        ["Teensy", "SPI và Wire", "Thư viện lõi Arduino"],
        ["ESP32", "Arduino ESP32", "Platform Espressif32 6.11.0"],
        ["ESP32", "AccelStepper", "1.64; phát xung không chặn"],
        ["ESP32", "Preferences", "NVS thuộc Arduino ESP32"],
        ["UI", "React", "19.2.6"], ["UI", "Next", "16.2.6"], ["UI", "TypeScript", "5.9.3"],
        ["UI", "vinext và Vite", "vinext 0.0.50; Vite 8.0.13"], ["UI", "Cloudflare plugin", "Deploy Worker và Sites"],
    ], [1.1, 2.4, 3.4], 8.7)

    add_heading(doc, "18 Quy tắc kiểm soát thay đổi", 1)
    add_numbered(doc, [
        "Mỗi lần chốt hoạt động ổn định, commit đồng thời RobotConfig h, main cpp, ESP32 source, giao diện và tài liệu protocol.",
        "Gắn tag theo ngày và nội dung thử, lưu file hex hoặc bin kèm checksum và log chạy thật.",
        "Mỗi thay đổi route phải cập nhật một bảng state gồm điều kiện vào, điều kiện ra, timeout, cảm biến chính và hành động lỗi.",
        "Không tái sử dụng chân mà chưa cập nhật bảng chân và kiểm tra xung đột UART PWM encoder I2C SPI.",
        "Mọi tham số tune từ giao diện phải được chép về RobotConfig h hoặc profile ESP32 rồi build lại. Chỉ ESP32 SAVE mới ghi NVS.",
        "Giữ một test checklist theo map RED và BLUE, tốc độ thấp và tốc độ thi đấu, nguồn đầy và nguồn giảm, không tải và có tải.",
        "Trước khi thi đấu, tạo bản release chỉ chứa state đang dùng, xóa hằng số không tham chiếu hoặc đánh dấu legacy bằng tên riêng.",
    ])
    p = doc.add_paragraph()
    p.add_run("Tóm tắt sử dụng tài liệu  ").bold = True
    p.add_run("Khi robot có hành vi sai, bắt đầu từ state đang báo trên giao diện, đối chiếu bảng luồng AUTO, kiểm tra cảm biến quyết định của state đó, sau đó mới thay PID hoặc tốc độ. Nếu giao diện báo gửi thành công nhưng cơ cấu không chạy, kiểm tra lần lượt chiều gửi COM5, UART6, MECH STATUS, fault ESP32 và zero phần mềm.")

    core = doc.core_properties
    core.title = "Báo cáo kỹ thuật hệ thống điều khiển robot ROBOCON 2026"
    core.subject = "Teensy 4.1, ESP32 S3 và giao diện Web Serial"
    core.author = "Dự án ROBOCON 2026"
    core.keywords = "ROBOCON, Teensy 4.1, ESP32 S3, Mecanum, PID, telemetry, Web Serial"
    core.comments = "Tài liệu phản ánh source hiện có ngày 08 tháng 09 năm 2026"
    doc.settings.update_fields_on_open = True
    doc.save(OUT)
    print(OUT)


if __name__ == "__main__":
    build_report()
