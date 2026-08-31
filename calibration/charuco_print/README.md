# ChArUco Print Target

## A4 target for the current right-arm calibration

Clean print file with no title, dimensions, guides, or other page content:

```text
right_arm_eye_on_base_charuco_A4_11x8_square15mm_marker11mm_DICT_4X4_250_CLEAN_PRINT_ACTUAL_SIZE.pdf
```

This is the preferred print file. It is a true portrait A4 page containing
only the dimensionally correct ChArUco board, rotated 90 degrees clockwise and
centered on the page. Its printed orientation is `120 mm` wide by `165 mm`
high.

Use this file for the right-arm external-camera (eye-to-hand / eye-on-base)
calibration:

```text
right_arm_eye_on_base_charuco_A4_11x8_square15mm_marker11mm_DICT_4X4_250_PRINT_ACTUAL_SIZE.pdf
```

It is a standard A4 page. The ChArUco area is exactly `165 x 120 mm` and the
page includes a `100 mm` line and a `50 x 50 mm` square for checking printer
scale. Print using **Actual size / 100%** with all fit/shrink options disabled.
Do not use the print until the 100 mm line measures 100.0 mm.

Regenerate it with:

```bash
python3 /home/gzu/gzu_ws/tools/generate_charuco_a4.py
```

The target parameters match
`right_arm_camera2_eye_on_base_charuco.rviz`:

```text
squares X: 11
squares Y: 8
square size: 15 mm
marker size: 11 mm
dictionary: DICT_4X4_250
longest board side: 0.165 m
```

## Legacy 200 x 150 mm page

Use this PDF for the current right-arm eye-to-hand calibration:

```text
charuco_8x11_checker15mm_marker11mm_DICT_4X4_250_200x150mm_PRINT_100_PERCENT.pdf
```

Print settings:

- Print at 100% / Actual Size. Do not use Fit to Page.
- The PDF page size is 200 x 150 mm.
- Checker size: 15 mm.
- Marker size: 11 mm.
- Dictionary: ArUco DICT_4X4_250.
- Mount the print on a rigid, flat board before sampling.

MoveIt Calibration target settings:

```text
Target Type: HandEyeTarget/Charuco
squares X: 11
squares Y: 8
marker size (px): 110
square size (px): 150
marker border (bits): 1
ArUco dictionary: DICT_4X4_250
longest board side (m): 0.165
measured marker size (m): 0.011
Image Topic: /camera/color/image_raw
CameraInfo Topic: /camera/color/camera_info
```
