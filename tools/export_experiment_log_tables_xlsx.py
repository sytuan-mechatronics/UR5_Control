"""Export the 4 official experiment log tables to a single XLSX workbook.

This script avoids external dependencies by generating a minimal XLSX file
with 4 worksheets directly from stdlib XML/ZIP utilities.
"""

from __future__ import annotations

import html
from pathlib import Path
from zipfile import ZIP_DEFLATED, ZipFile


ROOT = Path(__file__).resolve().parents[1]
OUT_PATH = ROOT / "EXPERIMENT_LOG_TABLES.xlsx"


WORKFLOW_ROWS = [
    ("run_id", "Ma lan chay tong", "chuoi"),
    ("workflow_id", "Ma workflow", "chuoi"),
    ("scenario_case", "Truong hop thuc nghiem tong, vd 5a/5b/5c", "chuoi"),
    ("ur5_joint_speed_rad_s", "Toc do joint UR5 duoc cau hinh", "rad/s"),
    ("ur5_linear_speed_m_s", "Toc do linear UR5 duoc cau hinh", "m/s"),
    ("ur5_pick_approach_speed_m_s", "Toc do UR5 khi ha vao diem pick", "m/s"),
    ("mir_travel_time_s", "Tong thoi gian MIR di chuyen", "s"),
    ("mir_stop_error_m", "Sai so dung cua MIR so voi diem dich", "m"),
    ("feed_wait_time_s", "Thoi gian cho cap phoi neu co", "s"),
    ("vision_confirm_result", "Vision co xac nhan phoi hay khong", "true/false hoac chuoi mo ta"),
    ("vision_confidence", "Do tin cay xac nhan vision", "0..1"),
    ("ur5_cycle_time_s", "Tong thoi gian phan UR5 thuc hien", "s"),
    ("ur5_parts_found", "So phoi tim thay", "so nguyen"),
    ("ur5_parts_picked", "So phoi gap thanh cong", "so nguyen"),
    ("workflow_total_time_s", "Tong thoi gian workflow tu dau den cuoi", "s"),
    ("final_status", "Trang thai ket thuc", "completed/timeout/error/aborted"),
    ("error_note", "Loi hoac canh bao chinh", "chuoi"),
]

SCENARIO1_ROWS = [
    ("job_id", "Ma job", "chuoi"),
    ("created_at", "Thoi diem tao job", "ISO UTC"),
    ("completed_at", "Thoi diem ket thuc", "ISO UTC"),
    ("duration_s", "Tong thoi gian job", "s"),
    ("station", "Tram / nguon test", "chuoi"),
    ("workflow_id", "Ma workflow hoac ma lan test", "chuoi"),
    ("experiment_stage", "Stage thuc nghiem", "1"),
    ("ur5_joint_speed_rad_s", "Toc do joint UR5 duoc cau hinh", "rad/s"),
    ("ur5_linear_speed_m_s", "Toc do linear UR5 duoc cau hinh", "m/s"),
    ("ur5_pick_approach_speed_m_s", "Toc do UR5 khi ha vao diem pick", "m/s"),
    ("cycle_status", "Trang thai ket thuc chu trinh", "done/error/aborted"),
    ("cycle_time_s", "Thoi gian 1 chu trinh motion", "s"),
    ("gripper_close_ms", "Thoi gian dong gripper", "ms"),
    ("gripper_open_ms", "Thoi gian mo gripper", "ms"),
    ("warning_count", "So canh bao trong job log", "so nguyen"),
    ("error_or_warning", "Loi/canh bao quan trong nhat", "chuoi"),
    ("note", "Ghi chu thu cong", "chuoi"),
]

SCENARIO2_ROWS = [
    ("job_id", "Ma job", "chuoi"),
    ("created_at", "Thoi diem tao job", "ISO UTC"),
    ("completed_at", "Thoi diem ket thuc", "ISO UTC"),
    ("duration_s", "Tong thoi gian job", "s"),
    ("station", "Tram / nguon test", "chuoi"),
    ("workflow_id", "Ma workflow hoac ma lan test", "chuoi"),
    ("experiment_stage", "Stage thuc nghiem", "2"),
    ("ur5_joint_speed_rad_s", "Toc do joint UR5 duoc cau hinh", "rad/s"),
    ("ur5_linear_speed_m_s", "Toc do linear UR5 duoc cau hinh", "m/s"),
    ("ur5_pick_approach_speed_m_s", "Toc do UR5 khi ha vao diem pick", "m/s"),
    ("confidence_yolo11", "Do tin cay phat hien YOLO", "0..1"),
    ("localization_error_mm", "Sai so dinh vi thuc te", "mm"),
    ("target_x_m", "Toa do target X trong he base", "m"),
    ("target_y_m", "Toa do target Y trong he base", "m"),
    ("target_z_m", "Toa do target Z trong he base", "m"),
    ("pick_result", "Ket qua gap", "success/no_detection/invalid_depth_zero/..."),
    ("retry_count", "So lan thu lai", "so nguyen"),
    ("slot_position", "Vi tri slot cua phoi duoc nhan dien", "chuoi"),
    ("selected_slot", "Slot duoc chon neu co mapping correction", "chuoi"),
    ("cycle_status", "Trang thai ket thuc", "done/error/aborted"),
    ("parts_found", "So phoi detect duoc", "so nguyen"),
    ("parts_picked", "So phoi gap thanh cong", "so nguyen"),
    ("error_or_warning", "Loi/canh bao quan trong", "chuoi"),
    ("note", "Ghi chu thu cong", "chuoi"),
]

SCENARIO3_ROWS = [
    ("job_id", "Ma job", "chuoi"),
    ("created_at", "Thoi diem tao job", "ISO UTC"),
    ("completed_at", "Thoi diem ket thuc", "ISO UTC"),
    ("duration_s", "Tong thoi gian ca lan chay", "s"),
    ("station", "Tram / nguon test", "chuoi"),
    ("workflow_id", "Ma workflow hoac ma lan test", "chuoi"),
    ("experiment_stage", "Stage thuc nghiem", "3"),
    ("ur5_joint_speed_rad_s", "Toc do joint UR5 duoc cau hinh", "rad/s"),
    ("ur5_linear_speed_m_s", "Toc do linear UR5 duoc cau hinh", "m/s"),
    ("ur5_pick_approach_speed_m_s", "Toc do UR5 khi ha vao diem pick", "m/s"),
    ("scenario_case", "Truong hop con 3a/3b/3c/3d", "chuoi"),
    ("parts_found_initial", "So phoi tim thay o lan scan dau", "so nguyen"),
    ("parts_picked_total", "Tong so phoi gap thanh cong", "so nguyen"),
    ("run_status", "Trang thai lan chay", "done/error/aborted"),
    ("tray_position", "Vi tri khay/slot cua phoi", "chuoi"),
    ("pick_order", "Thu tu gap", "so nguyen"),
    ("pick_result", "Ket qua cua phoi do", "success/fail/no_pick"),
    ("retry_count", "So lan thu lai cua phoi do", "so nguyen"),
    ("confidence_yolo11", "Confidence cua phoi do", "0..1"),
    ("part_duration_s", "Thoi gian xu ly rieng cua phoi do", "s"),
    ("error_or_warning", "Loi/canh bao quan trong", "chuoi"),
    ("note", "Ghi chu thu cong", "chuoi"),
]

SHEETS = [
    ("Workflow_Tong", "Log Workflow Tong MIR + UR5", WORKFLOW_ROWS),
    ("Scenario1_Phase1", "Log Kich Ban 1 / Phase 1", SCENARIO1_ROWS),
    ("Scenario2_Phase2", "Log Kich Ban 2 / Phase 2", SCENARIO2_ROWS),
    ("Scenario3_Phase3", "Log Kich Ban 3 / Phase 3", SCENARIO3_ROWS),
]


def _cell_ref(col_idx: int, row_idx: int) -> str:
    name = ""
    value = col_idx
    while value:
        value, rem = divmod(value - 1, 26)
        name = chr(65 + rem) + name
    return f"{name}{row_idx}"


def _inline_str(text: str) -> str:
    return (
        '<is><t xml:space="preserve">'
        f"{html.escape(str(text))}"
        "</t></is>"
    )


def _sheet_xml(title: str, rows: list[tuple[str, str, str]]) -> str:
    header = [
        ("A1", title),
        ("A2", "Ten cot"),
        ("B2", "Y nghia"),
        ("C2", "Don vi / gia tri"),
    ]
    cells = []
    for ref, value in header:
        cells.append(
            f'<c r="{ref}" t="inlineStr">{_inline_str(value)}</c>'
        )
    start_row = 3
    for row_idx, (field_name, meaning, unit) in enumerate(rows, start=start_row):
        values = [field_name, meaning, unit]
        for col_idx, value in enumerate(values, start=1):
            ref = _cell_ref(col_idx, row_idx)
            cells.append(
                f'<c r="{ref}" t="inlineStr">{_inline_str(value)}</c>'
            )
    max_row = len(rows) + 2
    dimension = f"A1:C{max_row}"
    cols = (
        '<cols>'
        '<col min="1" max="1" width="30" customWidth="1"/>'
        '<col min="2" max="2" width="55" customWidth="1"/>'
        '<col min="3" max="3" width="28" customWidth="1"/>'
        '</cols>'
    )
    return (
        '<?xml version="1.0" encoding="UTF-8" standalone="yes"?>'
        '<worksheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">'
        f'<dimension ref="{dimension}"/>'
        '<sheetViews><sheetView workbookViewId="0"/></sheetViews>'
        f"{cols}"
        '<sheetData>'
        f'<row r="1">{cells[0]}</row>'
        f'<row r="2">{cells[1]}{cells[2]}{cells[3]}</row>'
        + "".join(
            f'<row r="{row_idx}">'
            + "".join(cells[4 + (row_idx - start_row) * 3: 4 + (row_idx - start_row + 1) * 3])
            + "</row>"
            for row_idx in range(start_row, len(rows) + start_row)
        )
        + '</sheetData>'
        '</worksheet>'
    )


def build_workbook(path: Path) -> None:
    content_types = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">
  <Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>
  <Default Extension="xml" ContentType="application/xml"/>
  <Override PartName="/xl/workbook.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.sheet.main+xml"/>
  <Override PartName="/docProps/core.xml" ContentType="application/vnd.openxmlformats-package.core-properties+xml"/>
  <Override PartName="/docProps/app.xml" ContentType="application/vnd.openxmlformats-officedocument.extended-properties+xml"/>
  <Override PartName="/xl/styles.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.styles+xml"/>
  <Override PartName="/xl/theme/theme1.xml" ContentType="application/vnd.openxmlformats-officedocument.theme+xml"/>
  <Override PartName="/xl/worksheets/sheet1.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>
  <Override PartName="/xl/worksheets/sheet2.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>
  <Override PartName="/xl/worksheets/sheet3.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>
  <Override PartName="/xl/worksheets/sheet4.xml" ContentType="application/vnd.openxmlformats-officedocument.spreadsheetml.worksheet+xml"/>
</Types>
"""
    rels = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/officeDocument" Target="xl/workbook.xml"/>
  <Relationship Id="rId2" Type="http://schemas.openxmlformats.org/package/2006/relationships/metadata/core-properties" Target="docProps/core.xml"/>
  <Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/extended-properties" Target="docProps/app.xml"/>
</Relationships>
"""
    workbook = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<workbook xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main"
 xmlns:r="http://schemas.openxmlformats.org/officeDocument/2006/relationships">
  <sheets>
    <sheet name="Workflow_Tong" sheetId="1" r:id="rId1"/>
    <sheet name="Scenario1_Phase1" sheetId="2" r:id="rId2"/>
    <sheet name="Scenario2_Phase2" sheetId="3" r:id="rId3"/>
    <sheet name="Scenario3_Phase3" sheetId="4" r:id="rId4"/>
  </sheets>
</workbook>
"""
    workbook_rels = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">
  <Relationship Id="rId1" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet1.xml"/>
  <Relationship Id="rId2" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet2.xml"/>
  <Relationship Id="rId3" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet3.xml"/>
  <Relationship Id="rId4" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/worksheet" Target="worksheets/sheet4.xml"/>
  <Relationship Id="rId5" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/styles" Target="styles.xml"/>
  <Relationship Id="rId6" Type="http://schemas.openxmlformats.org/officeDocument/2006/relationships/theme" Target="theme/theme1.xml"/>
</Relationships>
"""
    styles = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<styleSheet xmlns="http://schemas.openxmlformats.org/spreadsheetml/2006/main">
  <fonts count="1"><font><sz val="11"/><name val="Calibri"/></font></fonts>
  <fills count="2"><fill><patternFill patternType="none"/></fill><fill><patternFill patternType="gray125"/></fill></fills>
  <borders count="1"><border><left/><right/><top/><bottom/><diagonal/></border></borders>
  <cellStyleXfs count="1"><xf numFmtId="0" fontId="0" fillId="0" borderId="0"/></cellStyleXfs>
  <cellXfs count="1"><xf numFmtId="0" fontId="0" fillId="0" borderId="0" xfId="0"/></cellXfs>
  <cellStyles count="1"><cellStyle name="Normal" xfId="0" builtinId="0"/></cellStyles>
</styleSheet>
"""
    theme = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<a:theme xmlns:a="http://schemas.openxmlformats.org/drawingml/2006/main" name="Office Theme">
  <a:themeElements>
    <a:clrScheme name="Office">
      <a:dk1><a:sysClr val="windowText" lastClr="000000"/></a:dk1>
      <a:lt1><a:sysClr val="window" lastClr="FFFFFF"/></a:lt1>
      <a:dk2><a:srgbClr val="1F497D"/></a:dk2>
      <a:lt2><a:srgbClr val="EEECE1"/></a:lt2>
      <a:accent1><a:srgbClr val="4F81BD"/></a:accent1>
      <a:accent2><a:srgbClr val="C0504D"/></a:accent2>
      <a:accent3><a:srgbClr val="9BBB59"/></a:accent3>
      <a:accent4><a:srgbClr val="8064A2"/></a:accent4>
      <a:accent5><a:srgbClr val="4BACC6"/></a:accent5>
      <a:accent6><a:srgbClr val="F79646"/></a:accent6>
      <a:hlink><a:srgbClr val="0000FF"/></a:hlink>
      <a:folHlink><a:srgbClr val="800080"/></a:folHlink>
    </a:clrScheme>
    <a:fontScheme name="Office">
      <a:majorFont><a:latin typeface="Calibri"/></a:majorFont>
      <a:minorFont><a:latin typeface="Calibri"/></a:minorFont>
    </a:fontScheme>
    <a:fmtScheme name="Office"><a:fillStyleLst/><a:lnStyleLst/><a:effectStyleLst/><a:bgFillStyleLst/></a:fmtScheme>
  </a:themeElements>
</a:theme>
"""
    app = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<Properties xmlns="http://schemas.openxmlformats.org/officeDocument/2006/extended-properties"
 xmlns:vt="http://schemas.openxmlformats.org/officeDocument/2006/docPropsVTypes">
  <Application>Codex</Application>
</Properties>
"""
    core = """<?xml version="1.0" encoding="UTF-8" standalone="yes"?>
<cp:coreProperties xmlns:cp="http://schemas.openxmlformats.org/package/2006/metadata/core-properties"
 xmlns:dc="http://purl.org/dc/elements/1.1/"
 xmlns:dcterms="http://purl.org/dc/terms/"
 xmlns:dcmitype="http://purl.org/dc/dcmitype/"
 xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance">
  <dc:title>Experiment Log Tables</dc:title>
  <dc:creator>Codex</dc:creator>
</cp:coreProperties>
"""

    with ZipFile(path, "w", compression=ZIP_DEFLATED) as zf:
        zf.writestr("[Content_Types].xml", content_types)
        zf.writestr("_rels/.rels", rels)
        zf.writestr("xl/workbook.xml", workbook)
        zf.writestr("xl/_rels/workbook.xml.rels", workbook_rels)
        zf.writestr("xl/styles.xml", styles)
        zf.writestr("xl/theme/theme1.xml", theme)
        zf.writestr("docProps/app.xml", app)
        zf.writestr("docProps/core.xml", core)
        for index, (_, title, rows) in enumerate(SHEETS, start=1):
            zf.writestr(f"xl/worksheets/sheet{index}.xml", _sheet_xml(title, rows))


def main() -> int:
    build_workbook(OUT_PATH)
    print(OUT_PATH)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
