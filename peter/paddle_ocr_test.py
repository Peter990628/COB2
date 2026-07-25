#!/usr/bin/env python3
"""PaddleOCR를 사진 또는 웹캠으로 간단히 시험하는 프로그램.

이 파일은 ROS 2 통신을 사용하지 않는 독립 실행형 테스트 코드이다.
모델을 한 번만 불러온 뒤 사진 한 장 또는 웹캠 프레임을 OCR하고,
터미널에는 인식 문자열과 신뢰도를 출력하며 화면에는 검출 영역을 그린다.

PaddleOCR 3.x를 기준으로 작성했으며, 기존 PaddleOCR 2.x 결과 형식도
읽을 수 있도록 호환 코드를 포함한다.

실행 예시
---------
사진 한 장:

    ros2 run cobot_pjt paddle_ocr_test --image ~/Downloads/passport.jpg

웹캠:

    ros2 run cobot_pjt paddle_ocr_test --camera 0

여권 MRZ용 영문 모델과 CLAHE 전처리:

    ros2 run cobot_pjt paddle_ocr_test \
      --image ~/Downloads/passport.jpg \
      --lang en \
      --preprocess clahe \
      --confidence 0.4

웹캠 창의 키:

* SPACE: 기다리지 않고 즉시 OCR
* S: 현재 검출 결과 이미지와 JSON 저장
* Q 또는 ESC: 종료

주의
----
첫 실행 때는 PaddleOCR 모델을 인터넷에서 내려받기 때문에 오래 걸릴 수 있다.
이 테스트는 기본적으로 CPU를 사용한다. GPU 시험은 설치한 PaddlePaddle GPU
패키지와 CUDA 호환성을 먼저 확인한 뒤 ``--device gpu:0``을 사용한다.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict, dataclass
from importlib import metadata
from pathlib import Path
from typing import Any, Iterable, List, Optional, Sequence, Tuple

import cv2
import numpy as np


WINDOW_NAME = "PaddleOCR Test"


@dataclass
class OcrDetection:
    """PaddleOCR 버전과 관계없이 사용할 공통 OCR 결과 한 개."""

    polygon: List[List[float]]
    text: str
    confidence: float


def package_version(package_name: str) -> str:
    """설치된 Python 패키지 버전을 반환하고, 찾지 못하면 unknown을 반환한다."""

    try:
        return metadata.version(package_name)
    except metadata.PackageNotFoundError:
        return "unknown"


def major_version(version: str) -> int:
    """'3.2.1' 같은 버전 문자열에서 주 버전 숫자 3을 얻는다."""

    try:
        return int(version.split(".", maxsplit=1)[0])
    except (TypeError, ValueError):
        return 0


def create_ocr_engine(language: str, device: str, orientation: bool):
    """설치된 PaddleOCR 주 버전에 맞는 OCR 객체를 생성한다.

    PaddleOCR 3.x는 ``predict()``와 ``device='cpu'`` 형식을 사용한다.
    PaddleOCR 2.x는 ``ocr()``와 ``use_gpu=False`` 형식을 사용한다.
    """

    try:
        from paddleocr import PaddleOCR
    except ImportError as exc:
        raise RuntimeError(
            "PaddleOCR를 불러오지 못했습니다. 현재 Python 환경에 "
            "'paddlepaddle'과 'paddleocr'을 먼저 설치하세요."
        ) from exc

    version = package_version("paddleocr")
    major = major_version(version)
    print(
        f"[환경] paddleocr={version}, "
        f"paddlepaddle={package_version('paddlepaddle')}, "
        f"device={device}, lang={language}"
    )

    if major >= 3:
        # 최신 3.x 파이프라인은 문서 보정 기능을 기본 비활성화하여
        # 순수한 글자 검출/인식 성능과 실행 시간을 먼저 확인한다.
        kwargs = {
            "lang": language,
            "device": device,
            "use_doc_orientation_classify": False,
            "use_doc_unwarping": False,
            "use_textline_orientation": orientation,
        }

        # 현재 PP-OCRv6 모델은 한국어 모델을 제공하지 않으므로,
        # 한국어를 선택한 경우 한국어 모델이 있는 PP-OCRv5를 명시한다.
        if language == "korean":
            kwargs["ocr_version"] = "PP-OCRv5"

        return PaddleOCR(**kwargs), major

    # PaddleOCR 2.x 호환 경로이다. 최신 환경을 새로 만든다면 3.x 사용을 권장한다.
    return (
        PaddleOCR(
            lang=language,
            use_angle_cls=orientation,
            use_gpu=device.startswith("gpu"),
            show_log=True,
        ),
        major,
    )


def preprocess_image(frame: np.ndarray, mode: str) -> np.ndarray:
    """원본과 같은 크기를 유지하면서 OCR용 전처리를 적용한다."""

    if mode == "none":
        return frame

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if mode == "gray":
        processed = gray
    elif mode == "clahe":
        # 여권 MRZ처럼 대비가 약한 글자에서 국부 대비를 높인다.
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        processed = clahe.apply(gray)
    elif mode == "threshold":
        # 조명이 비교적 균일한 흑백 문서 시험용 이진화이다.
        processed = cv2.adaptiveThreshold(
            gray,
            255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
            cv2.THRESH_BINARY,
            31,
            11,
        )
    else:
        raise ValueError(f"지원하지 않는 전처리 방식입니다: {mode}")

    # PaddleOCR 입력 형식을 일정하게 유지하기 위해 다시 BGR 3채널로 만든다.
    return cv2.cvtColor(processed, cv2.COLOR_GRAY2BGR)


def _plain_result_dict(result: Any) -> dict:
    """PaddleOCR 3.x Result 객체를 일반 dict로 변환한다."""

    if isinstance(result, dict):
        data = result
    else:
        data = None

        # PaddleX 계열 Result의 json 속성은 버전에 따라 dict 또는 JSON 문자열이다.
        json_value = getattr(result, "json", None)
        if callable(json_value):
            json_value = json_value()
        if isinstance(json_value, str):
            try:
                data = json.loads(json_value)
            except json.JSONDecodeError:
                data = None
        elif isinstance(json_value, dict):
            data = json_value

        if data is None:
            to_dict = getattr(result, "to_dict", None)
            if callable(to_dict):
                converted = to_dict()
                if isinstance(converted, dict):
                    data = converted

        if data is None:
            try:
                data = dict(result)
            except (TypeError, ValueError):
                data = {}

    # 일부 Result는 실제 결과를 "res" 아래에 넣는다.
    nested = data.get("res")
    return nested if isinstance(nested, dict) else data


def _box_to_polygon(box: Sequence[float]) -> List[List[float]]:
    """[x1, y1, x2, y2] 사각형을 네 점 polygon으로 변환한다."""

    values = np.asarray(box, dtype=np.float64).reshape(-1)
    if values.size < 4:
        return []
    x1, y1, x2, y2 = values[:4]
    return [
        [float(x1), float(y1)],
        [float(x2), float(y1)],
        [float(x2), float(y2)],
        [float(x1), float(y2)],
    ]


def parse_v3_results(raw_results: Iterable[Any]) -> List[OcrDetection]:
    """PaddleOCR 3.x ``predict()`` 결과를 공통 형식으로 변환한다."""

    detections: List[OcrDetection] = []

    for result in raw_results:
        data = _plain_result_dict(result)
        texts = data.get("rec_texts")
        if texts is None:
            texts = []
        scores = data.get("rec_scores")
        if scores is None:
            scores = []
        polygons = data.get("rec_polys")
        if polygons is None:
            polygons = data.get("dt_polys")
        boxes = data.get("rec_boxes")

        for index, raw_text in enumerate(texts):
            if polygons is not None and index < len(polygons):
                points = np.asarray(
                    polygons[index],
                    dtype=np.float64,
                ).reshape(-1, 2)
                polygon = points.tolist()
            elif boxes is not None and index < len(boxes):
                polygon = _box_to_polygon(boxes[index])
            else:
                polygon = []

            score = float(scores[index]) if index < len(scores) else 0.0
            detections.append(
                OcrDetection(
                    polygon=polygon,
                    text=str(raw_text),
                    confidence=score,
                )
            )

    return detections


def _looks_like_v2_line(value: Any) -> bool:
    """값이 ``[polygon, (text, score)]`` 형태인지 확인한다."""

    return (
        isinstance(value, (list, tuple))
        and len(value) >= 2
        and isinstance(value[1], (list, tuple))
        and len(value[1]) >= 2
        and isinstance(value[1][0], str)
    )


def _iter_v2_lines(value: Any) -> Iterable[Sequence[Any]]:
    """페이지 중첩 여부가 다른 PaddleOCR 2.x 결과에서 각 행을 꺼낸다."""

    if _looks_like_v2_line(value):
        yield value
        return
    if isinstance(value, (list, tuple)):
        for child in value:
            yield from _iter_v2_lines(child)


def parse_v2_results(raw_results: Any) -> List[OcrDetection]:
    """PaddleOCR 2.x ``ocr()`` 결과를 공통 형식으로 변환한다."""

    detections: List[OcrDetection] = []
    for line in _iter_v2_lines(raw_results):
        points = np.asarray(line[0], dtype=np.float64).reshape(-1, 2)
        text, score = line[1][0], line[1][1]
        detections.append(
            OcrDetection(
                polygon=points.tolist(),
                text=str(text),
                confidence=float(score),
            )
        )
    return detections


def run_ocr(
    engine: Any,
    paddleocr_major: int,
    frame: np.ndarray,
    preprocess: str,
) -> Tuple[List[OcrDetection], float]:
    """한 프레임을 OCR하고 결과 및 소요 시간을 반환한다."""

    ocr_input = preprocess_image(frame, preprocess)
    started = time.perf_counter()

    if paddleocr_major >= 3:
        raw_results = engine.predict(ocr_input)
        detections = parse_v3_results(raw_results)
    else:
        # cls=True는 2.x에서 생성 시 활성화한 문자 방향 분류기를 사용한다.
        raw_results = engine.ocr(ocr_input, cls=True)
        detections = parse_v2_results(raw_results)

    elapsed = time.perf_counter() - started
    return detections, elapsed


def filtered_detections(
    detections: Sequence[OcrDetection],
    confidence_threshold: float,
) -> List[OcrDetection]:
    """지정한 신뢰도 이상의 결과만 남긴다."""

    return [
        detection
        for detection in detections
        if detection.confidence >= confidence_threshold
    ]


def print_detections(
    detections: Sequence[OcrDetection],
    elapsed: float,
    confidence_threshold: float,
) -> None:
    """OCR 결과를 사람이 비교하기 쉬운 형태로 터미널에 출력한다."""

    accepted = filtered_detections(detections, confidence_threshold)
    print(
        f"\n[OCR] 전체={len(detections)}개, "
        f"신뢰도 {confidence_threshold:.2f} 이상={len(accepted)}개, "
        f"처리시간={elapsed:.3f}초"
    )
    if not accepted:
        print("  인식된 문자열이 없습니다.")
        return

    for index, detection in enumerate(accepted, start=1):
        print(
            f"  {index:02d}. confidence={detection.confidence:.3f} "
            f"text={detection.text!r}"
        )


def draw_detections(
    frame: np.ndarray,
    detections: Sequence[OcrDetection],
    confidence_threshold: float,
    elapsed: float,
) -> np.ndarray:
    """원본 영상에 검출 polygon과 번호/신뢰도를 표시한다."""

    canvas = frame.copy()
    accepted = filtered_detections(detections, confidence_threshold)

    for index, detection in enumerate(accepted, start=1):
        if len(detection.polygon) < 3:
            continue

        points = np.rint(detection.polygon).astype(np.int32).reshape(-1, 1, 2)
        cv2.polylines(canvas, [points], True, (0, 255, 0), 2, cv2.LINE_AA)

        anchor_x = int(np.min(points[:, 0, 0]))
        anchor_y = max(18, int(np.min(points[:, 0, 1])) - 6)
        # OpenCV 기본 폰트는 한글 표시가 어렵기 때문에 영상에는 번호와
        # 신뢰도만 쓰고 실제 문자열은 터미널과 JSON에서 확인한다.
        label = f"{index}: {detection.confidence:.2f}"
        cv2.putText(
            canvas,
            label,
            (anchor_x, anchor_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

    status = f"OCR {elapsed:.2f}s | accepted {len(accepted)}/{len(detections)}"
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 32), (0, 0, 0), -1)
    cv2.putText(
        canvas,
        status,
        (10, 23),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return canvas


def save_result(
    output_directory: Path,
    frame: np.ndarray,
    detections: Sequence[OcrDetection],
    confidence_threshold: float,
    elapsed: float,
) -> Tuple[Path, Path]:
    """표시 영상을 JPG로, OCR 원자료를 JSON으로 함께 저장한다."""

    output_directory.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    image_path = output_directory / f"paddleocr_{stamp}.jpg"
    json_path = output_directory / f"paddleocr_{stamp}.json"

    annotated = draw_detections(
        frame,
        detections,
        confidence_threshold,
        elapsed,
    )
    if not cv2.imwrite(str(image_path), annotated):
        raise RuntimeError(f"결과 이미지를 저장하지 못했습니다: {image_path}")

    payload = {
        "elapsed_sec": elapsed,
        "confidence_threshold": confidence_threshold,
        "detections": [asdict(item) for item in detections],
    }
    json_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )

    print(f"[저장] 이미지: {image_path}")
    print(f"[저장] JSON:  {json_path}")
    return image_path, json_path


def run_image_mode(
    args: argparse.Namespace,
    engine: Any,
    paddleocr_major: int,
) -> int:
    """사진 파일 한 장을 OCR한다."""

    image_path = Path(args.image).expanduser().resolve()
    frame = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if frame is None:
        raise FileNotFoundError(f"이미지를 열 수 없습니다: {image_path}")

    print(f"[입력] 이미지: {image_path} ({frame.shape[1]}x{frame.shape[0]})")
    detections, elapsed = run_ocr(
        engine,
        paddleocr_major,
        frame,
        args.preprocess,
    )
    print_detections(detections, elapsed, args.confidence)
    save_result(
        Path(args.output_dir).expanduser(),
        frame,
        detections,
        args.confidence,
        elapsed,
    )

    if not args.no_window:
        annotated = draw_detections(
            frame,
            detections,
            args.confidence,
            elapsed,
        )
        cv2.imshow(WINDOW_NAME, annotated)
        print("[화면] 아무 키나 누르면 종료합니다.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    return 0


def run_camera_mode(
    args: argparse.Namespace,
    engine: Any,
    paddleocr_major: int,
) -> int:
    """웹캠 영상을 표시하면서 일정 간격으로 OCR한다."""

    capture = cv2.VideoCapture(args.camera)
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    if not capture.isOpened():
        raise RuntimeError(f"웹캠 {args.camera}번을 열 수 없습니다.")

    detections: List[OcrDetection] = []
    elapsed = 0.0
    last_ocr_time = -float("inf")
    last_frame: Optional[np.ndarray] = None
    immediate_ocr = True

    print(
        f"[입력] 웹캠 index={args.camera}, 요청 해상도={args.width}x{args.height}, "
        f"OCR 간격={args.interval:.2f}초"
    )
    print("[키] SPACE=즉시 OCR, S=결과 저장, Q/ESC=종료")

    try:
        while True:
            ok, frame = capture.read()
            if not ok or frame is None:
                print("[경고] 웹캠 프레임을 받지 못했습니다.")
                time.sleep(0.05)
                continue

            last_frame = frame.copy()
            now = time.monotonic()
            if immediate_ocr or now - last_ocr_time >= args.interval:
                detections, elapsed = run_ocr(
                    engine,
                    paddleocr_major,
                    frame,
                    args.preprocess,
                )
                print_detections(detections, elapsed, args.confidence)
                last_ocr_time = time.monotonic()
                immediate_ocr = False

            annotated = draw_detections(
                frame,
                detections,
                args.confidence,
                elapsed,
            )
            cv2.putText(
                annotated,
                "SPACE: OCR | S: save | Q/ESC: quit",
                (10, annotated.shape[0] - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                1,
                cv2.LINE_AA,
            )
            cv2.imshow(WINDOW_NAME, annotated)

            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), ord("Q"), 27):
                break
            if key == ord(" "):
                immediate_ocr = True
            elif key in (ord("s"), ord("S")) and last_frame is not None:
                save_result(
                    Path(args.output_dir).expanduser(),
                    last_frame,
                    detections,
                    args.confidence,
                    elapsed,
                )
    finally:
        capture.release()
        cv2.destroyAllWindows()

    return 0


def build_argument_parser() -> argparse.ArgumentParser:
    """명령행 옵션을 정의한다."""

    parser = argparse.ArgumentParser(
        description="PaddleOCR 사진/웹캠 단독 테스트",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    source = parser.add_mutually_exclusive_group()
    source.add_argument("--image", help="OCR할 사진 파일 경로")
    source.add_argument("--camera", type=int, help="사용할 웹캠 번호")

    parser.add_argument(
        "--lang",
        default="en",
        choices=("en", "korean"),
        help="인식 언어 모델(en=여권/MRZ, korean=한글+영문)",
    )
    parser.add_argument(
        "--device",
        default="cpu",
        help="PaddleOCR 3.x 실행 장치(cpu 또는 gpu:0)",
    )
    parser.add_argument(
        "--confidence",
        type=float,
        default=0.4,
        help="화면과 요약 결과에 포함할 최소 인식 신뢰도",
    )
    parser.add_argument(
        "--preprocess",
        choices=("none", "gray", "clahe", "threshold"),
        default="none",
        help="PaddleOCR 입력 전에 적용할 영상 전처리",
    )
    parser.add_argument(
        "--orientation",
        action="store_true",
        help="글자 방향 분류 사용(정방향 문서에서는 보통 불필요)",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=1.0,
        help="웹캠 모드 OCR 실행 간격(초)",
    )
    parser.add_argument("--width", type=int, default=1280, help="웹캠 요청 폭")
    parser.add_argument("--height", type=int, default=720, help="웹캠 요청 높이")
    parser.add_argument(
        "--output-dir",
        default="paddleocr_output",
        help="JPG와 JSON 결과 저장 폴더",
    )
    parser.add_argument(
        "--no-window",
        action="store_true",
        help="사진 모드에서 OpenCV 결과 창을 열지 않음",
    )
    return parser


def validate_arguments(args: argparse.Namespace) -> None:
    """잘못된 수치를 모델 로딩 전에 차단한다."""

    if not 0.0 <= args.confidence <= 1.0:
        raise ValueError("--confidence는 0.0~1.0이어야 합니다.")
    if args.interval <= 0.0:
        raise ValueError("--interval은 0보다 커야 합니다.")
    if args.width <= 0 or args.height <= 0:
        raise ValueError("--width와 --height는 0보다 커야 합니다.")

    # 입력을 생략하면 가장 흔한 0번 웹캠을 사용한다.
    if args.image is None and args.camera is None:
        args.camera = 0


def main(argv: Optional[Sequence[str]] = None) -> int:
    """프로그램 진입점."""

    parser = build_argument_parser()
    args = parser.parse_args(argv)

    try:
        validate_arguments(args)
        engine, paddleocr_major = create_ocr_engine(
            args.lang,
            args.device,
            args.orientation,
        )

        if args.image:
            return run_image_mode(args, engine, paddleocr_major)
        return run_camera_mode(args, engine, paddleocr_major)
    except KeyboardInterrupt:
        print("\n사용자가 중단했습니다.")
        return 130
    except Exception as exc:
        print(f"[오류] {type(exc).__name__}: {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
