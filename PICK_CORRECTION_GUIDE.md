# Pick Correction Map

Dung khi:
- Phôi giữa khay canh bang global offset thi dung.
- Cac phôi o vi tri khac van lech nhe theo tung vung.

Khong dung khi:
- Lech hang tram mm.
- Hand-eye, TCP, depth, hoac intrinsics van dang sai lon.

## Bat tinh nang

Trong `.env`:

```env
PICK_CORRECTION_ENABLED=True
PICK_CORRECTION_MAP_PATH=pick_correction_map.json
PICK_CORRECTION_STRATEGY=slot_only
```

## Y tuong

He thong tinh:

1. `p_base_raw` tu camera + hand-eye
2. cong `local correction` theo slot gan nhat
3. cong tiep `PICK_OFFSET_X/Y/Z` global

Cong thuc:

`p_base_final = p_base_raw + local_map_offset + global_pick_offset`

## Cac buoc thu thap du lieu

1. Chon 4-9 diem trai deu trong vung khay.
2. O moi diem:
   - dat phôi tai vi tri do
   - chay `tools/check_target_base_error.py --teach-expected`
   - ghi lai `p_base_raw(m)`
   - ghi lai sai so `expected - predicted`
3. Dien vao `pick_correction_map.json`:
   - `x`, `y`: lay tu `p_base_raw`
   - `dx`, `dy`, `dz`: so bu them de vision cham dung diem that

## Vi du

```json
{
  "meta": {
    "reference": "raw_base_before_global_pick_offset"
  },
  "points": [
    { "name": "center", "x": 0.6200, "y": -0.2650, "dx": 0.0000, "dy": 0.0000, "dz": 0.0000 },
    { "name": "left",   "x": 0.5900, "y": -0.2650, "dx": 0.0015, "dy": -0.0008, "dz": 0.0000 },
    { "name": "right",  "x": 0.6500, "y": -0.2650, "dx": -0.0012, "dy": 0.0007, "dz": 0.0000 },
    { "name": "front",  "x": 0.6200, "y": -0.2300, "dx": 0.0005, "dy": -0.0010, "dz": 0.0000 },
    { "name": "back",   "x": 0.6200, "y": -0.3000, "dx": -0.0006, "dy": 0.0011, "dz": 0.0000 }
  ]
}
```

## Ghi chu

- `dx/dy/dz` la don vi met.
- Sai so 1 mm = `0.001`.
- Neu lech chu yeu theo XY thi de `dz = 0`.
- `slot_only` hop voi khay co cac vi tri co dinh va tinh tay tung slot.
- Neu can noi suy giua cac diem, co the doi `PICK_CORRECTION_STRATEGY=idw`.
- Neu target nam rat xa tat ca diem da do, local correction se bo qua va chi dung global offset.
