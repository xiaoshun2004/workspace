# -*- coding: utf-8 -*-
"""
core/project_io.py
------------------
Everything that touches the disk.

LAYOUT

  <root>/
    intrinsics_library.json          reusable per (camera, resolution)
    projects/
      <project>/
        project.json                 board, camera, convention, frames, poses
        poses/
          pose_000_raw.png
          pose_000_overlay.png
          ...
        instances/
          <YYYYmmdd_HHMMSS>_<label>/
            metadata.json            camera intrinsics + resolution + model,
                                     board geometry, Tool/User identity, pose
                                     convention, remount record, solve result
            handeye.json             result + samples (paths relative)
            images/
              sample_000_raw.png
              sample_000_overlay.png

Two decisions worth stating:

1. IMAGE PATHS ARE STORED RELATIVE to the file that references them, and
   resolved on load. The reference bundle stored absolutes, which quietly
   breaks the moment a run is copied to another machine for review -- and
   reviewing a suspicious calibration on another machine is exactly what the
   images exist for.

2. WRITES ARE ATOMIC (tmp + os.replace). A calibration interrupted mid-write
   should leave the previous good file intact rather than a truncated one.

Instances are the unit of comparison: same project, different day, nudged
working distance, re-clocked bracket. Each gets its own directory and its own
copy of the metadata needed to decide, later, whether it is still valid.
"""

from __future__ import annotations

import csv
import datetime
import json
import math
import os
import shutil
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np

import handeye
from models import (
    CalibInstance,
    CalibProject,
    CameraIntrinsics,
    IntrinsicsRecord,
    Result,
)

PROJECTS_DIRNAME = "projects"
INSTANCES_DIRNAME = "instances"
POSES_DIRNAME = "poses"
IMAGES_DIRNAME = "images"
PROJECT_FILENAME = "project.json"
METADATA_FILENAME = "metadata.json"
HANDEYE_FILENAME = "handeye.json"
INTRINSICS_LIBRARY_FILENAME = "intrinsics_library.json"


def sanitize_name(name: str) -> str:
    """Filesystem-safe name. Empty input becomes 'unnamed' rather than ''."""
    cleaned = "".join(c if (c.isalnum() or c in "-_") else "_"
                      for c in (name or "").strip())
    cleaned = cleaned.strip("_")
    return cleaned or "unnamed"


def timestamp_label() -> str:
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


def iso_now() -> str:
    return datetime.datetime.now().isoformat(timespec="seconds")


def _json_safe(value):
    """Replace every non-finite float with null, recursively.

    Several model fields default to NaN and are serialised unconditionally.
    Python's json module happily emits the bare tokens NaN and Infinity, but
    neither is valid JSON (RFC 8259), and a single one of them makes the WHOLE
    file unreadable to jq, JSON.parse or any non-Python reviewer -- which is
    exactly what these files are copied to another machine for. models
    .float_or_nan() reads the nulls back as NaN, so nothing downstream changes.
    """
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    return value


def atomic_write_json(path: str, payload: dict) -> None:
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(_json_safe(payload), f, indent=2, allow_nan=False)
    os.replace(tmp, path)


def read_json(path: str) -> Result[dict]:
    if not os.path.exists(path):
        return Result.err_msg("File not found: {}".format(path))
    try:
        with open(path, "r", encoding="utf-8") as f:
            return Result.ok(json.load(f))
    except UnicodeDecodeError as exc:
        return Result.err_msg(
            "Cannot decode {} as UTF-8; the file is corrupt or is not "
            "text: {}".format(path, exc))
    except ValueError as exc:
        # JSONDecodeError is a ValueError; so is anything else json raises on
        # rubbish input. Either way the caller gets a message, not a traceback.
        return Result.err_msg("Malformed JSON in {}: {}".format(path, exc))
    except OSError as exc:
        return Result.err_msg("Cannot read {}: {}".format(path, exc))


# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
@dataclass
class ProjectPaths:
    root: str
    name: str

    @property
    def dir(self) -> str:
        return os.path.join(self.root, PROJECTS_DIRNAME, sanitize_name(self.name))

    @property
    def json_path(self) -> str:
        return os.path.join(self.dir, PROJECT_FILENAME)

    @property
    def poses_dir(self) -> str:
        return os.path.join(self.dir, POSES_DIRNAME)

    @property
    def instances_dir(self) -> str:
        return os.path.join(self.dir, INSTANCES_DIRNAME)

    def pose_image(self, index: int, overlay: bool = False) -> str:
        return os.path.join(self.poses_dir, "pose_{:03d}_{}.png".format(
            index, "overlay" if overlay else "raw"))

    def ensure(self) -> None:
        os.makedirs(self.poses_dir, exist_ok=True)
        os.makedirs(self.instances_dir, exist_ok=True)


@dataclass
class InstancePaths:
    project: ProjectPaths
    folder: str                 # "<stamp>_<label>"

    @property
    def dir(self) -> str:
        return os.path.join(self.project.instances_dir, self.folder)

    @property
    def metadata_path(self) -> str:
        return os.path.join(self.dir, METADATA_FILENAME)

    @property
    def handeye_path(self) -> str:
        return os.path.join(self.dir, HANDEYE_FILENAME)

    @property
    def images_dir(self) -> str:
        return os.path.join(self.dir, IMAGES_DIRNAME)

    def sample_image(self, index: int, overlay: bool = False) -> str:
        return os.path.join(self.images_dir, "sample_{:03d}_{}.png".format(
            index, "overlay" if overlay else "raw"))

    def ensure(self) -> None:
        os.makedirs(self.images_dir, exist_ok=True)


# ---------------------------------------------------------------------------
# Image helpers
# ---------------------------------------------------------------------------
def save_image(path: str, bgr: np.ndarray) -> Result[str]:
    """Write a frame, creating parents. Costs nothing and is the only way to
    re-solve or audit a suspicious result later."""
    os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
    if not cv2.imwrite(path, bgr):
        return Result.err_msg("Failed to write image: {}".format(path))
    return Result.ok(path)


def load_image(path: str) -> Result[np.ndarray]:
    """Read one image as BGR for the import-a-library workflow.

    cv2.imread signals every failure -- missing file, unreadable bytes, an
    unsupported format -- by returning None, so the reason is reconstructed
    here rather than surfacing as a NoneType error three calls later.
    """
    if not os.path.exists(path):
        return Result.err_msg("Image not found: {}".format(path))
    image = cv2.imread(path, cv2.IMREAD_COLOR)
    if image is None:
        return Result.err_msg(
            "Could not decode {} as an image.".format(path))
    return Result.ok(image)


def parse_pose_csv(path: str) -> Result[List[Tuple[Optional[str], List[float]]]]:
    """Parse a batch pose file for imported image samples.

    Two row shapes are accepted, and a file uses one of them throughout:

        x,y,z,rx,ry,rz               applied IN ORDER to samples without a pose
        filename,x,y,z,rx,ry,rz      matched to a sample by image file name

    Values are the controller's own raw numbers (mm, and whatever angle unit
    the solved convention says), exactly as the pose-entry dialog takes them.
    A first row that does not parse as numbers is treated as a header; '#'
    lines and blank lines are skipped; any later unparsable row is an error
    naming the row, because silently skipping a pose would misalign every
    in-order assignment after it.
    """
    read_err = None
    header_skipped = False
    rows: List[Tuple[Optional[str], List[float]]] = []
    try:
        with open(path, "r", encoding="utf-8-sig", newline="") as f:
            for number, row in enumerate(csv.reader(f), 1):
                cells = [c.strip() for c in row if c.strip() != ""]
                if not cells or cells[0].startswith("#"):
                    continue
                name: Optional[str] = None
                values = cells
                if len(cells) == 7:
                    name, values = cells[0], cells[1:]
                elif len(cells) != 6:
                    read_err = ("Row {} has {} values; expected 6 "
                                "(x,y,z,rx,ry,rz) or 7 (filename first)."
                                .format(number, len(cells)))
                    break
                try:
                    numbers = [float(v) for v in values]
                except ValueError:
                    if not rows and not header_skipped:
                        header_skipped = True   # first content row is a header
                        continue
                    read_err = ("Row {} is not numeric: {}".format(
                        number, ",".join(cells)))
                    break
                if not all(math.isfinite(v) for v in numbers):
                    read_err = "Row {} contains a non-finite value.".format(
                        number)
                    break
                rows.append((name, numbers))
    except OSError as exc:
        return Result.err_msg("Cannot read {}: {}".format(path, exc))
    except UnicodeDecodeError as exc:
        return Result.err_msg("Cannot decode {} as UTF-8: {}".format(
            path, exc))
    if read_err is not None:
        return Result.err_msg(read_err)
    if not rows:
        return Result.err_msg("No pose rows found in {}.".format(path))
    named = sum(1 for name, _values in rows if name is not None)
    if named not in (0, len(rows)):
        return Result.err_msg(
            "Mixed row shapes: {} of {} rows carry a filename. Use one shape "
            "for the whole file, or the in-order rows land on the wrong "
            "samples.".format(named, len(rows)))
    return Result.ok(rows)


def _relative(path: str, base_dir: str) -> str:
    if not path:
        return ""
    try:
        return os.path.relpath(path, base_dir).replace(os.sep, "/")
    except ValueError:
        return path        # different drive on Windows; keep the absolute


def _absolute(path: str, base_dir: str) -> str:
    if not path:
        return ""
    if os.path.isabs(path):
        return path
    return os.path.normpath(os.path.join(base_dir, path))


# ---------------------------------------------------------------------------
# Projects
# ---------------------------------------------------------------------------
def list_projects(root: str) -> List[str]:
    base = os.path.join(root, PROJECTS_DIRNAME)
    if not os.path.isdir(base):
        return []
    names = []
    for entry in sorted(os.listdir(base)):
        if os.path.isfile(os.path.join(base, entry, PROJECT_FILENAME)):
            names.append(entry)
    return names


def save_project(root: str, project: CalibProject) -> Result[str]:
    paths = ProjectPaths(root=root, name=project.name)
    paths.ensure()
    project.modified = iso_now()
    if not project.created:
        project.created = project.modified
    project.renumber()

    payload = project.to_dict()
    for pose_dict in payload["poses"]:
        pose_dict["image_path"] = _relative(pose_dict["image_path"], paths.dir)
        pose_dict["overlay_path"] = _relative(pose_dict["overlay_path"],
                                              paths.dir)
    atomic_write_json(paths.json_path, payload)
    return Result.ok(paths.json_path)


def load_project(root: str, name: str) -> Result[CalibProject]:
    paths = ProjectPaths(root=root, name=name)
    read = read_json(paths.json_path)
    if read.is_err:
        return Result.err_msg(read.message)
    project = CalibProject.from_dict(read.data)
    for pose in project.poses:
        pose.image_path = _absolute(pose.image_path, paths.dir)
        pose.overlay_path = _absolute(pose.overlay_path, paths.dir)
    if not project.name:
        project.name = name
    return Result.ok(project)


def delete_project(root: str, name: str) -> Result[None]:
    paths = ProjectPaths(root=root, name=name)
    if not os.path.isdir(paths.dir):
        return Result.err_msg("No such project: {}".format(name))
    shutil.rmtree(paths.dir)
    return Result.ok(None)


def save_pose_images(root: str, project_name: str, index: int,
                     raw_bgr: np.ndarray,
                     overlay_bgr: Optional[np.ndarray] = None
                     ) -> Result[Tuple[str, str]]:
    """Store the reference imagery for one authored pose. Returns absolute
    (raw, overlay) paths; save_project() relativises them on write."""
    paths = ProjectPaths(root=root, name=project_name)
    paths.ensure()
    raw_path = paths.pose_image(index, overlay=False)
    written = save_image(raw_path, raw_bgr)
    if written.is_err:
        return Result.err_msg(written.message)
    overlay_path = ""
    if overlay_bgr is not None:
        overlay_path = paths.pose_image(index, overlay=True)
        written = save_image(overlay_path, overlay_bgr)
        if written.is_err:
            return Result.err_msg(written.message)
    return Result.ok((raw_path, overlay_path))


# ---------------------------------------------------------------------------
# Instances
# ---------------------------------------------------------------------------
def new_instance_paths(root: str, project_name: str,
                       label: str = "") -> InstancePaths:
    """Mint a directory for a run that has not been saved yet.

    The stamp only has one-second resolution and the label is derived
    deterministically from state, so two saves in the same second would
    otherwise land in the same folder and the second would overwrite the
    first's metadata, result and sample images. Probe for a free name instead.
    """
    project = ProjectPaths(root=root, name=project_name)
    base = timestamp_label()
    if label:
        base = "{}_{}".format(base, sanitize_name(label))
    folder = base
    attempt = 2
    while os.path.exists(os.path.join(project.instances_dir, folder)):
        folder = "{}-{}".format(base, attempt)
        attempt += 1
    paths = InstancePaths(project=project, folder=folder)
    paths.ensure()
    return paths


def instance_paths(root: str, project_name: str,
                   folder: str) -> InstancePaths:
    """Paths for an instance that already exists on disk.

    Needed to re-save an instance in place after re-solving it from its own
    stored samples, as opposed to new_instance_paths() which always mints a
    fresh timestamped folder.
    """
    return InstancePaths(project=ProjectPaths(root=root, name=project_name),
                         folder=folder)


def list_instances(root: str, project_name: str) -> List[str]:
    base = ProjectPaths(root=root, name=project_name).instances_dir
    if not os.path.isdir(base):
        return []
    return [e for e in sorted(os.listdir(base), reverse=True)
            if os.path.isfile(os.path.join(base, e, METADATA_FILENAME))]


def save_instance(paths: InstancePaths, instance: CalibInstance,
                  samples: Sequence[handeye.HandEyeSample]) -> Result[str]:
    """Write metadata.json and handeye.json for one calibration run.

    Both files are written even when the solve failed or has not run: the
    samples and the camera metadata are the expensive part to reproduce, and a
    failed run is exactly the one worth inspecting afterwards.
    """
    paths.ensure()
    if not instance.created:
        instance.created = iso_now()
    if not instance.name:
        instance.name = paths.folder

    board_geometry = handeye.board_geometry(instance.board)
    metadata = instance.metadata_dict()
    metadata["board_geometry"] = board_geometry
    metadata["n_samples"] = len(samples)
    metadata["image_dir"] = IMAGES_DIRNAME
    atomic_write_json(paths.metadata_path, metadata)

    payload = {
        "camera": (None if instance.intrinsics is None
                   else instance.intrinsics.to_dict()),
        "board": board_geometry,
        "image_dir": IMAGES_DIRNAME,
        "convention": instance.convention.to_dict(),
        "frames": instance.frames.to_dict(),
        "samples": [],
    }
    if instance.result is not None:
        payload["result"] = instance.result.to_dict()
    for s in samples:
        d = handeye.sample_to_dict(s)
        d["image_path"] = _relative(d["image_path"], paths.dir)
        d["overlay_path"] = _relative(d["overlay_path"], paths.dir)
        payload["samples"].append(d)
    atomic_write_json(paths.handeye_path, payload)
    return Result.ok(paths.dir)


def load_instance(instance_dir: str
                  ) -> Result[Tuple[CalibInstance, List[handeye.HandEyeSample]]]:
    meta = read_json(os.path.join(instance_dir, METADATA_FILENAME))
    if meta.is_err:
        return Result.err_msg(meta.message)
    instance = CalibInstance.from_metadata(meta.data)

    samples: List[handeye.HandEyeSample] = []
    he = read_json(os.path.join(instance_dir, HANDEYE_FILENAME))
    if he.is_ok:
        for d in he.data.get("samples", []):
            sample = handeye.sample_from_dict(d)
            sample.image_path = _absolute(sample.image_path, instance_dir)
            sample.overlay_path = _absolute(sample.overlay_path, instance_dir)
            samples.append(sample)
    return Result.ok((instance, samples))


def save_sample_images(paths: InstancePaths, index: int, raw_bgr: np.ndarray,
                       overlay_bgr: Optional[np.ndarray] = None
                       ) -> Result[Tuple[str, str]]:
    paths.ensure()
    raw_path = paths.sample_image(index, overlay=False)
    written = save_image(raw_path, raw_bgr)
    if written.is_err:
        return Result.err_msg(written.message)
    overlay_path = ""
    if overlay_bgr is not None:
        overlay_path = paths.sample_image(index, overlay=True)
        written = save_image(overlay_path, overlay_bgr)
        if written.is_err:
            return Result.err_msg(written.message)
    return Result.ok((raw_path, overlay_path))


def latest_instance_with_result(root: str, project_name: str
                                ) -> Result[Tuple[CalibInstance, str]]:
    """Most recent solved instance of a project, used to seed the auto-realign
    servo with a previous X. Newest first; the first one carrying a result
    wins."""
    base = ProjectPaths(root=root, name=project_name).instances_dir
    for folder in list_instances(root, project_name):
        loaded = load_instance(os.path.join(base, folder))
        if loaded.is_ok and loaded.data[0].result is not None:
            return Result.ok((loaded.data[0], os.path.join(base, folder)))
    return Result.err_msg(
        "No solved instance found for project '{}'.".format(project_name))


# ---------------------------------------------------------------------------
# Intrinsics library
# ---------------------------------------------------------------------------
class IntrinsicsLibrary:
    """Reusable intrinsics, keyed by (camera, width, height).

    Intrinsics scale with resolution -- fx, fy, ppx and ppy all do -- so a
    record is only ever offered back at exactly the resolution it was measured
    at. A USB webcam has no factory intrinsics at all, so for those this
    library is the only source; for a RealSense it lets a proper calibration
    replace the factory values permanently if you want it to.

    A file that exists but cannot be read is NOT the same as no file at all:
    in memory both look like an empty library, and saving over the former
    would destroy every record it holds. load_error keeps that distinction
    alive until save() can act on it.
    """

    def __init__(self, path: str) -> None:
        self._path = path
        self._records: List[IntrinsicsRecord] = []
        self.load_error = ""    # non-empty when the file exists but is unreadable
        self.load()

    @property
    def path(self) -> str:
        return self._path

    def load(self) -> Result[int]:
        self._records = []
        self.load_error = ""
        if not os.path.exists(self._path):
            return Result.ok(0)
        read = read_json(self._path)
        if read.is_err:
            self.load_error = read.message
            return Result.err_msg(read.message)
        for d in read.data.get("records", []):
            self._records.append(IntrinsicsRecord.from_dict(d))
        return Result.ok(len(self._records))

    def save(self) -> Result[str]:
        """Write the library out, never destroying a file we could not read.

        An unreadable library is indistinguishable from an empty one once it
        is in memory, so writing straight over it would silently throw away
        every calibrated record it contains. Move it aside first: the operator
        keeps the bytes and can recover the records by hand.
        """
        note = ""
        if self.load_error and os.path.exists(self._path):
            backup = "{}.bad-{}".format(self._path, timestamp_label())
            try:
                os.replace(self._path, backup)
            except OSError as exc:
                return Result.err_msg(
                    "The intrinsics library {} exists but could not be read "
                    "({}), and it could not be moved aside either ({}). "
                    "Nothing was written -- fix or move the file by hand, "
                    "otherwise its stored records would be lost.".format(
                        self._path, self.load_error, exc))
            note = ("The previous intrinsics library could not be read ({}); "
                    "it was kept as {} and a fresh one written.".format(
                        self.load_error, backup))
            self.load_error = ""
        atomic_write_json(self._path, {
            "records": [r.to_dict() for r in self._records],
            "modified": iso_now(),
        })
        return Result(True, self._path, note)

    def records(self) -> List[IntrinsicsRecord]:
        return list(self._records)

    def find(self, camera_key: str, width: int,
             height: int) -> Optional[IntrinsicsRecord]:
        for r in self._records:
            if r.key() == (camera_key, int(width), int(height)):
                return r
        return None

    def put(self, record: IntrinsicsRecord) -> Result[str]:
        """Insert or replace. One record per (camera, resolution): keeping
        several would only invite picking the wrong one."""
        if not record.timestamp:
            record.timestamp = iso_now()
        self._records = [r for r in self._records if r.key() != record.key()]
        self._records.append(record)
        return self.save()

    def remove(self, camera_key: str, width: int, height: int) -> Result[None]:
        before = len(self._records)
        self._records = [r for r in self._records
                         if r.key() != (camera_key, int(width), int(height))]
        if len(self._records) == before:
            return Result.err_msg("No stored intrinsics for that camera and "
                                  "resolution.")
        saved = self.save()
        if saved.is_err:
            return Result.err_msg(saved.message)
        return Result.ok(None)


def default_root() -> str:
    return os.path.join(os.path.expanduser("~"), ".config", "nova5_calib")


def intrinsics_library_path(root: str) -> str:
    return os.path.join(root, INTRINSICS_LIBRARY_FILENAME)


# ---------------------------------------------------------------------------
# Validity checking
# ---------------------------------------------------------------------------
def check_instance_applicable(instance: CalibInstance,
                              live_intrinsics: CameraIntrinsics,
                              live_frames=None, *,
                              live_convention=None) -> Result[None]:
    """Can this stored calibration be used against the current live setup?

    Four ways a calibration silently stops being true: the stream resolution
    changed (intrinsics scale with it), the taught TCP or User frame changed
    (T_cam2mount is relative to the reported frame), the pose convention
    changed (the controller's six numbers now decode to a different rotation),
    or there is no solve in the instance at all.
    """
    if instance.result is None:
        return Result.err_msg("Instance '{}' has no solved result.".format(
            instance.name))
    if instance.intrinsics is None:
        return Result.err_msg(
            "Instance '{}' stored no camera intrinsics, so its validity "
            "cannot be checked.".format(instance.name))
    if instance.intrinsics.resolution() != live_intrinsics.resolution():
        return Result.err_msg(
            "Calibration was captured at {}x{} but the live stream is {}x{}. "
            "Intrinsics scale with resolution, so this calibration does not "
            "apply -- restream at the captured resolution or "
            "re-calibrate.".format(
                instance.intrinsics.width, instance.intrinsics.height,
                live_intrinsics.width, live_intrinsics.height))
    if live_frames is not None:
        frames_ok = instance.frames.compare(live_frames)
        if frames_ok.is_err:
            return frames_ok
    if live_convention is not None:
        # verified/note are bookkeeping; everything else changes what the six
        # reported numbers MEAN, and so what T_cam2mount was measured against.
        stored = instance.convention.to_dict()
        live = live_convention.to_dict()
        keys = ("order", "intrinsic", "degrees", "rotvec", "quat_order",
                "translation_scale")
        if any(stored[k] != live[k] for k in keys):
            return Result.err_msg(
                "Pose convention changed: calibrated with '{}' at translation "
                "scale {:g}, but the live setting is '{}' at scale {:g}. The "
                "stored transforms were built by decoding the controller's six "
                "numbers that way, so they do not transfer -- restore the "
                "calibrated convention or re-calibrate.".format(
                    instance.convention.label(),
                    float(instance.convention.translation_scale),
                    live_convention.label(),
                    float(live_convention.translation_scale)))
    return Result.ok(None)