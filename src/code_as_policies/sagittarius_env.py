import time
from pathlib import Path

import actionlib
import cv2
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge
from sagittarius_object_color_detector.msg import SGRCtrlAction, SGRCtrlGoal, SGRCtrlResult
from sensor_msgs.msg import Image


class SagittariusEnv:
    """Bridge environment for Interactive_Demo -> Sagittarius_ws backend."""

    # Fallback calibration and tabletop semantics used when external config is
    # missing or when the LMP refers to named regions instead of detected blocks.
    _FALLBACK_LINEAR_REGRESSION = {
        "k1": -0.00048266229370656023,
        "b1": 0.3498628285678873,
        "k2": -0.0005388671980101526,
        "b2": 0.17532044287260315,
    }
    _FALLBACK_HSV_RANGES = {
        "red": {"hmin": 360.0, "hmax": 21.0, "smin": 32.0, "smax": 255.0, "vmin": 0.0, "vmax": 255.0},
        "green": {"hmin": 69.0, "hmax": 138.0, "smin": 90.0, "smax": 255.0, "vmin": 0.0, "vmax": 255.0},
        "blue": {"hmin": 168.0, "hmax": 267.0, "smin": 41.0, "smax": 255.0, "vmin": 0.0, "vmax": 255.0},
    }
    _FIXED_BOWL_POSITIONS = {
    }
    _MAP_AREA_POSITIONS = {
        "area a": np.array([0.16, -0.25, 0.03], dtype=np.float32),
        "area b": np.array([0.16, 0.25, 0.03], dtype=np.float32),
        "area c": np.array([0.26, -0.25, 0.03], dtype=np.float32),
        "area d": np.array([0.26, 0.25, 0.03], dtype=np.float32),
    }
    _TABLETOP_NAMED_POINTS = {
        "top left corner": (0.35, 0.13),
        "top side": (0.35, 0.00),
        "top right corner": (0.35, -0.13),
        "left side": (0.25, 0.13),
        "middle": (0.25, 0.00),
        "right side": (0.25, -0.13),
        "bottom left corner": (0.15, 0.13),
        "bottom side": (0.15, 0.00),
        "bottom right corner": (0.15, -0.13),
    }

    def __init__(
        self,
        arm_name="sgr532",
        vision_config_path=None,
        camera_topic="/usb_cam/image_raw",
        action_timeout=30.0,
        camera_timeout=3.0,
        refresh_interval=0.5,
        pick_retry_attempts=2,
        pick_retry_refresh=True,
    ):
        # Bootstrap ROS I/O, perception defaults, cached state, and the action
        # client used for all downstream motion requests.
        try:
            rospy.init_node("code_as_policies_brain", anonymous=True)
        except rospy.exceptions.ROSException:
            pass

        self.arm_name = arm_name
        self.camera_topic = camera_topic
        self.action_timeout = float(action_timeout)
        self.camera_timeout = float(camera_timeout)
        self.refresh_interval = float(refresh_interval)
        self.pick_retry_attempts = max(0, int(pick_retry_attempts))
        self.pick_retry_refresh = bool(pick_retry_refresh)

        self.default_pick_z = 0.02
        self.default_place_z = 0.02
        self.safe_default_pos = np.array([0.20, 0.00, 0.08], dtype=np.float32)
        self.workspace_bounds = np.float32([[0.00, 0.35], [-0.40, 0.40], [0.00, 0.30]])

        self.min_contour_area = 500.0
        self.max_block_contour_area = 100000.0
        self.stable_detection_frames = 5
        self.stable_detection_min_hits = 3
        self.stable_detection_max_drift_px = 18.0
        self.last_view_detections_meta = {}
        self.last_scan_meta = []

        self.canonical_obs_pose = [0.20, 0.00, 0.15, 0.0, 1.57, 0.0]
        self.canonical_settle_time = 0.8

        self.bridge = CvBridge()
        self.cache_video = []
        self._last_camera_image = None
        self._last_ee_pos = np.array([0.20, 0.00, 0.15], dtype=np.float32)
        self._last_refresh_at = 0.0

        self.vision_config_path = self._resolve_vision_config_path(vision_config_path)
        self.vision_config = self._load_vision_config(self.vision_config_path)
        self.linear_regression = self._parse_linear_regression(self.vision_config)
        self.color_ranges = self._parse_color_ranges(self.vision_config)
        self.fixed_bowl_positions = {
            self._normalize_name(name): pos.copy()
            for name, pos in self._FIXED_BOWL_POSITIONS.items()
        }
        self.object_catalog = self._build_object_catalog()
        self.named_positions = self._build_named_positions()
        self.last_view_detections = {}
        self.world_state = {}
        self._publish_world_state(self.world_state)

        self.ctrl_action_name = f"/{self.arm_name}/sgr_ctrl"
        self.ctrl_client = actionlib.SimpleActionClient(self.ctrl_action_name, SGRCtrlAction)
        rospy.loginfo("Waiting for Sagittarius Control Action Server: %s", self.ctrl_action_name)
        if not self.ctrl_client.wait_for_server(timeout=rospy.Duration.from_sec(15.0)):
            raise RuntimeError(
                f"Cannot connect to action server '{self.ctrl_action_name}'. "
                "Please launch sagittarius_ws first."
            )
        rospy.loginfo("Connected to Sagittarius control action server.")
        self.refresh_world_state(force=True)

    # Vision config loading -------------------------------------------------
    def _resolve_vision_config_path(self, explicit_path):
        # Prefer an explicit path, then the repo-local config, then a ROS param.
        if explicit_path:
            path = Path(explicit_path).expanduser().resolve()
            if path.exists():
                return str(path)

        repo_default = (
            Path(__file__).resolve().parents[1]
            / "sagittarius_ws/src/sagittarius_arm_ros/sagittarius_perception/"
            / "sagittarius_object_color_detector/config/vision_config.yaml"
        )
        if repo_default.exists():
            return str(repo_default)

        param_path = rospy.get_param("~vision_config", "")
        if param_path:
            path = Path(param_path).expanduser().resolve()
            if path.exists():
                return str(path)

        rospy.logwarn("vision_config.yaml not found. Falling back to built-in HSV + regression defaults.")
        return ""

    def _load_vision_config(self, vision_config_path):
        # Load YAML defensively so the environment can still run with defaults.
        if not vision_config_path:
            return {}
        try:
            with open(vision_config_path, "r", encoding="utf-8") as handle:
                content = yaml.safe_load(handle.read())
                return content if isinstance(content, dict) else {}
        except Exception as exc:
            rospy.logwarn("Failed to load vision config from %s: %s", vision_config_path, exc)
            return {}

    def _parse_linear_regression(self, config):
        # Merge runtime calibration with built-in pixel-to-robot fallback values.
        params = dict(self._FALLBACK_LINEAR_REGRESSION)
        regression = config.get("LinearRegression", {})
        for key in ("k1", "b1", "k2", "b2"):
            if key in regression:
                params[key] = float(regression[key])
        return params

    def _parse_color_ranges(self, config):
        # Convert HSV config into OpenCV-ready lower/upper bounds.
        merged = dict(self._FALLBACK_HSV_RANGES)
        for key, value in config.items():
            if key == "LinearRegression":
                continue
            if not isinstance(value, dict):
                continue
            required = {"hmin", "hmax", "smin", "smax", "vmin", "vmax"}
            if not required.issubset(value.keys()):
                continue
            merged[key] = value

        parsed = {}
        for color, value in merged.items():
            if color == "customize":
                continue
            lower = np.array(
                [float(value["hmin"]) / 2.0, float(value["smin"]), float(value["vmin"])],
                dtype=np.float32,
            )
            upper = np.array(
                [float(value["hmax"]) / 2.0, float(value["smax"]), float(value["vmax"])],
                dtype=np.float32,
            )
            parsed[color] = (lower, upper, bool(lower[0] > upper[0]))
        return parsed

    # Static world semantics ------------------------------------------------
    def _build_object_catalog(self):
        # The dynamic object catalog is currently color blocks only.
        return [f"{color} block" for color in sorted(self.color_ranges.keys())]

    def _build_named_positions(self):
        # Build aliases for language-facing tabletop regions and named points.
        named = {}

        for area_name, pos in self._MAP_AREA_POSITIONS.items():
            key = self._normalize_name(area_name)
            named[key] = pos.copy()
            suffix = key.split(" ", 1)[1]
            named[self._normalize_name(f"zone {suffix}")] = pos.copy()
            named[self._normalize_name(f"region {suffix}")] = pos.copy()
            named[self._normalize_name(f"drop {key}")] = pos.copy()
            named[self._normalize_name(f"map {key}")] = pos.copy()

        for point_name, (x_pos, y_pos) in self._TABLETOP_NAMED_POINTS.items():
            named[self._normalize_name(point_name)] = np.array([x_pos, y_pos, self.default_place_z], dtype=np.float32)

        return named

    def _normalize_name(self, obj_name):
        # Normalize user-facing names so language variants resolve consistently.
        normalized = str(obj_name).lower().replace("_", " ").strip()
        if normalized.startswith("the "):
            normalized = normalized[4:]
        return " ".join(normalized.split())

    # World-state bookkeeping ----------------------------------------------
    def _publish_world_state(self, positions):
        # Canonicalize coordinates and refresh the lookup tables used by the LMP.
        published = {}
        for name, pos in positions.items():
            normalized = self._normalize_name(name)
            xyz = np.asarray(pos, dtype=np.float32).reshape(-1)
            if xyz.shape[0] == 2:
                default_z = self.default_place_z if normalized.endswith("bowl") else self.default_pick_z
                xyz = np.array([xyz[0], xyz[1], default_z], dtype=np.float32)
            else:
                xyz = xyz[:3].astype(np.float32)
            published[normalized] = xyz.copy()

        self.world_state = published
        self._object_positions = {name: pos.copy() for name, pos in published.items()}
        self._object_list = sorted(published.keys())
        self.obj_name_to_id = {name: idx for idx, name in enumerate(self._object_list)}

    def merge_observations(self, existing, new):
        # Merge fresh detections into the cached world state without dropping
        # previously known objects that may be temporarily occluded.
        merged = {self._normalize_name(name): np.asarray(pos, dtype=np.float32).reshape(-1)[:3].copy()
                  for name, pos in existing.items()}
        for name, pos in new.items():
            normalized = self._normalize_name(name)
            xyz = np.asarray(pos, dtype=np.float32).reshape(-1)
            if xyz.shape[0] == 2:
                default_z = self.default_place_z if normalized.endswith("bowl") else self.default_pick_z
                xyz = np.array([xyz[0], xyz[1], default_z], dtype=np.float32)
            else:
                xyz = xyz[:3].astype(np.float32)
            merged[normalized] = xyz.copy()
        return merged

    # Perception helpers ----------------------------------------------------
    def _make_mask(self, hsv_image, lower, upper, wraps_hue):
        # Red-like ranges may wrap around the HSV hue boundary and need two masks.
        if wraps_hue:
            lower_1 = np.array([0.0, lower[1], lower[2]], dtype=np.float32)
            upper_1 = np.array([upper[0], upper[1], upper[2]], dtype=np.float32)
            lower_2 = np.array([lower[0], lower[1], lower[2]], dtype=np.float32)
            upper_2 = np.array([180.0, upper[1], upper[2]], dtype=np.float32)
            return cv2.add(cv2.inRange(hsv_image, lower_1, upper_1), cv2.inRange(hsv_image, lower_2, upper_2))
        return cv2.inRange(hsv_image, lower, upper)

    def _pixel_to_robot_xy(self, pixel_x, pixel_y):
        # Apply the calibrated linear mapping from image pixels to tabletop XY.
        x_pos = self.linear_regression["k1"] * float(pixel_y) + self.linear_regression["b1"]
        y_pos = self.linear_regression["k2"] * float(pixel_x) + self.linear_regression["b2"]
        return np.array([x_pos, y_pos], dtype=np.float32)

    def _contour_center(self, contour):
        # Use image moments so we can map each detected block to a robot target.
        moment = cv2.moments(contour)
        if moment["m00"] <= 0:
            return None
        center_x = float(moment["m10"] / moment["m00"])
        center_y = float(moment["m01"] / moment["m00"])
        return center_x, center_y

    def _detect_objects(self, cv_image):
        # Detect colored blocks from a single camera frame and attach lightweight
        # metadata that later stability checks can inspect.
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        detections = {}
        meta = {}

        for color, (lower, upper, wraps_hue) in self.color_ranges.items():
            mask = self._make_mask(hsv_image, lower, upper, wraps_hue)
            mask = cv2.erode(mask, None, iterations=2)
            mask = cv2.dilate(mask, None, iterations=2)
            mask = cv2.GaussianBlur(mask, (5, 5), 0)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= self.min_contour_area]
            if not contours:
                continue

            contours = sorted(contours, key=cv2.contourArea, reverse=True)

            # New mode: bowls are fixed anchors, so vision only returns block positions.
            block_candidates = [
            cnt for cnt in contours
            if self.min_contour_area <= cv2.contourArea(cnt) <= self.max_block_contour_area]
            if not block_candidates:
                continue

            primary = block_candidates[0]
            primary_center = self._contour_center(primary)
            if primary_center is None:
                continue

            area = float(cv2.contourArea(primary))
            primary_xy = self._pixel_to_robot_xy(primary_center[0], primary_center[1])

            name = f"{color} block"
            detections[name] = np.array(
                [primary_xy[0], primary_xy[1], self.default_pick_z],
                dtype=np.float32,
                )
            
            meta[name] = {
                "pixel_center": [float(primary_center[0]), float(primary_center[1])],
                "area": area,
                }

        return detections, meta

    # Multi-frame observation and refresh ----------------------------------
    def observe_current_view(self, frames_per_pose=3):
        # Aggregate several frames so short-lived noise does not immediately
        # perturb the task-level world state.
        num_frames = max(1, int(frames_per_pose))
        hits = {}
        meta = {"frames": [], "accepted": {}, "rejected": {}}

        for frame_idx in range(num_frames):
            image = self.get_camera_image(timeout=self.camera_timeout, raise_on_timeout=False)
            if image is None:
                meta["frames"].append({"frame": frame_idx, "detections": {}})
                continue

            detections, det_meta = self._detect_objects(image)
            frame_record = {}

            for name, xyz in detections.items():
                info = det_meta.get(name, {})
                pixel_center = info.get("pixel_center")
                area = info.get("area")

                frame_record[name] = {
                    "pixel_center": pixel_center,
                    "area": area,
                    "xyz": xyz.tolist(),
                    }

                hits.setdefault(name, []).append({
                    "xyz": np.asarray(xyz, dtype=np.float32),
                    "pixel_center": np.asarray(pixel_center, dtype=np.float32),
                    "area": float(area) if area is not None else None,
                    })

            meta["frames"].append({"frame": frame_idx, "detections": frame_record})

            if frame_idx + 1 < num_frames:
                time.sleep(0.05)

        accepted = {}

        for name, samples in hits.items():
            if len(samples) < self.stable_detection_min_hits:
                meta["rejected"][name] = {
                    "reason": "not_enough_hits",
                    "hits": len(samples),
                    }
                continue

            pixel_centers = np.array([s["pixel_center"] for s in samples], dtype=np.float32)
            median_center = np.median(pixel_centers, axis=0)
            drift = float(np.max(np.linalg.norm(pixel_centers - median_center, axis=1)))

            if drift > self.stable_detection_max_drift_px:
                meta["rejected"][name] = {
                    "reason": "pixel_drift_too_large",
                    "hits": len(samples),
                    "max_drift_px": drift,
                    }
                continue

            xyzs = np.array([s["xyz"] for s in samples], dtype=np.float32)
            median_xyz = np.median(xyzs, axis=0).astype(np.float32)

            accepted[name] = median_xyz
            meta["accepted"][name] = {
                "hits": len(samples),
                "median_pixel_center": median_center.tolist(),
                "max_drift_px": drift,
                "xyz": median_xyz.tolist(),
                }

        self.last_view_detections = {name: pos.copy() for name, pos in accepted.items()}
        self.last_view_detections_meta = meta
        return {name: pos.copy() for name, pos in accepted.items()}

    def scan_world_state(self, settle_time=None, frames_per_pose=None):
        # Public entry point for a canonical observation pass.
        return self.scan_world_state_rpy(
            settle_time=self.canonical_settle_time if settle_time is None else settle_time,
            frames_per_pose=frames_per_pose,
        )

    def scan_world_state_rpy(self, settle_time=None, frames_per_pose=None):
        # Move to the observation pose, wait for the scene to settle, then
        # rebuild the visible portion of the world state.
        if frames_per_pose is None:
            frames_per_pose = self.stable_detection_frames
        else:
            frames_per_pose = max(1, int(frames_per_pose))

        if settle_time is None:
            settle_time = self.canonical_settle_time

        pose = self.canonical_obs_pose
        self.last_scan_meta = []

        ok = self.movep_rpy(pose)
        pose_meta = {"pose": list(pose), "move_ok": bool(ok), "view": {}}

        if not ok:
            self.last_scan_meta.append(pose_meta)
            return {name: pos.copy() for name, pos in self.world_state.items()}

        time.sleep(max(0.0, float(settle_time)))
        current_view = self.observe_current_view(frames_per_pose=frames_per_pose)
        pose_meta["view"] = {name: pos.tolist() for name, pos in current_view.items()}
        pose_meta["view_meta"] = self.last_view_detections_meta
        self.last_scan_meta.append(pose_meta)

        merged = self.merge_observations(self.world_state, current_view)
        self._publish_world_state(merged)
        self._last_refresh_at = time.time()
        return {name: pos.copy() for name, pos in self.world_state.items()}
        


    def refresh_world_state(self, force=False):
        now = time.time()
        if not force and now - self._last_refresh_at < self.refresh_interval:
            return list(self._object_list)

        if not self.movep_rpy(self.canonical_obs_pose):
            rospy.logwarn("Failed to move to canonical observation pose for world refresh.")
            return list(self._object_list)

        time.sleep(self.canonical_settle_time)

        current_view = self.observe_current_view(frames_per_pose=self.stable_detection_frames)
        if current_view:
            merged = self.merge_observations(self.world_state, current_view)
            self._publish_world_state(merged)

        self._last_refresh_at = now
        return list(self._object_list)

    def refresh_world_state_strict(self, frames_per_pose=None, settle_time=None):
        if frames_per_pose is None:
            frames_per_pose = self.stable_detection_frames
        else:
            frames_per_pose = max(1, int(frames_per_pose))

        if settle_time is None:
            settle_time = self.canonical_settle_time

        if not self.movep_rpy(self.canonical_obs_pose):
            rospy.logwarn("Failed to move to canonical observation pose for strict world refresh.")
            return None

        time.sleep(max(0.0, float(settle_time)))

        current_view = self.observe_current_view(frames_per_pose=frames_per_pose)
        self._publish_world_state(current_view)
        self._last_refresh_at = time.time()
        return {name: pos.copy() for name, pos in self.world_state.items()}


    @property
    def object_list(self):
        # Expose the current object catalog while opportunistically refreshing it.
        self.refresh_world_state(force=False)
        return list(self._object_list)

    # Query surfaces used by the LMP ---------------------------------------
    def get_camera_image(self, timeout=None, raise_on_timeout=True):
        # Read one ROS image message and cache the last successful frame.
        use_timeout = self.camera_timeout if timeout is None else float(timeout)
        try:
            msg = rospy.wait_for_message(self.camera_topic, Image, timeout=use_timeout)
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self._last_camera_image = cv_image
            return cv_image
        except Exception as exc:
            if raise_on_timeout:
                raise RuntimeError(f"Failed to read image from {self.camera_topic}: {exc}") from exc
            rospy.logwarn_throttle(5.0, "Camera frame unavailable on %s: %s", self.camera_topic, exc)
            return None

    def get_obj_pos(self, obj_name):
        # Resolve an object or named location into XYZ, refreshing perception
        # on demand before falling back to a safe default.
        normalized = self._normalize_name(obj_name)
        if normalized in self.named_positions:
            return self.named_positions[normalized].copy()
        if normalized in self.fixed_bowl_positions:
            return self.fixed_bowl_positions[normalized].copy()

        self.refresh_world_state(force=False)
        if normalized in self.world_state:
            return self.world_state[normalized].copy()

        self.refresh_world_state(force=True)
        if normalized in self.world_state:
            return self.world_state[normalized].copy()

        has_explicit_type = normalized.endswith(" block")
        color = normalized.split(" ")[0] if normalized else ""
        if not has_explicit_type and color:
            for object_type in ("block",):
                alias = f"{color} {object_type}"
                if alias in self.world_state:
                    return self.world_state[alias].copy()

        rospy.logwarn("Object '%s' is not visible. Returning safe default position.", obj_name)
        return self.safe_default_pos.copy()

    def get_bounding_box(self, obj_name):
        # Approximate each object as an axis-aligned box for stack-height logic.
        normalized = self._normalize_name(obj_name)
        center = self.get_obj_pos(normalized)
        if "block" in normalized:
            half_xy, height = 0.02, 0.04
        elif "bowl" in normalized:
            half_xy, height = 0.045, 0.06
        else:
            half_xy, height = 0.03, 0.04

        bbox_min = np.array(
            [center[0] - half_xy, center[1] - half_xy, max(0.0, center[2] - height / 2.0)],
            dtype=np.float32,
        )
        bbox_max = np.array(
            [center[0] + half_xy, center[1] + half_xy, center[2] + height / 2.0],
            dtype=np.float32,
        )
        return bbox_min, bbox_max

    # Motion and action helpers --------------------------------------------
    def _object_height(self, obj_name):
        # Provide coarse object heights for placement and stacking heuristics.
        normalized = self._normalize_name(obj_name)
        if "block" in normalized:
            return 0.04
        if "bowl" in normalized:
            return 0.06
        return 0.04

    def _resolve_place_xyz(self, place_xyz, object_name=None, target_name=None):
        # Lift the placement height when stacking onto another block.
        resolved = np.asarray(place_xyz, dtype=np.float32).reshape(-1)[:3].copy()
        if not isinstance(target_name, str):
            return resolved

        normalized_target = self._normalize_name(target_name)
        if not normalized_target.endswith("block"):
            return resolved

        try:
            _, target_bbox_max = self.get_bounding_box(normalized_target)
        except Exception as exc:
            rospy.logwarn(
                "Failed to resolve stacking height for target '%s': %s",
                target_name,
                exc
            )
            return resolved

        moving_height = self._object_height(object_name)
        resolved[2] = float(target_bbox_max[2] + moving_height / 2.0)
        rospy.loginfo(
            "Adjusted place height for stacking: object=%s target=%s place_z=%.4f",
            object_name,
            target_name,
            resolved[2]
        )
        return resolved
		
    def get_ee_pos(self):
        # The backend does not stream EE state here, so we expose the last
        # command-confirmed position tracked by this wrapper.
        return self._last_ee_pos.copy()

    def _normalize_xyz(self, position, default_z):
        # Accept either XY or XYZ input and coerce it into a safe float32 target.
        xyz = np.asarray(position, dtype=np.float32).reshape(-1)
        if xyz.shape[0] < 2:
            raise ValueError(f"Position must contain at least x/y values, got: {position}")
        if xyz.shape[0] == 2:
            xyz = np.array([xyz[0], xyz[1], default_z], dtype=np.float32)
        else:
            xyz = xyz[:3]
        if not np.isfinite(xyz).all():
            raise ValueError(f"Non-finite position values: {position}")
        if xyz[2] < 0.0:
            xyz[2] = default_z
        return xyz

    def _in_workspace(self, xyz):
        # Reject goals outside the tabletop envelope before asking ROS to plan.
        bounds = self.workspace_bounds
        return bool(
            bounds[0, 0] <= xyz[0] <= bounds[0, 1]
            and bounds[1, 0] <= xyz[1] <= bounds[1, 1]
            and bounds[2, 0] <= xyz[2] <= bounds[2, 1]
        )

    def _build_goal(self, action_type, grasp_type, xyz):
        # Translate local XYZ requests into the action message understood by
        # the Sagittarius control server.
        goal = SGRCtrlGoal()
        goal.action_type = action_type
        goal.grasp_type = grasp_type
        goal.pos_x = float(xyz[0])
        goal.pos_y = float(xyz[1])
        goal.pos_z = float(xyz[2])
        return goal

    def _result_to_text(self, code):
        # Convert backend result enums into readable logs.
        mapping = {
            int(SGRCtrlResult.SUCCESS): "SUCCESS",
            int(SGRCtrlResult.ERROR): "ERROR",
            int(SGRCtrlResult.PREEMPT): "PREEMPT",
            int(SGRCtrlResult.PLAN_NOT_FOUND): "PLAN_NOT_FOUND",
            int(SGRCtrlResult.GRASP_FAILD): "GRASP_FAILD",
        }
        return mapping.get(int(code), f"UNKNOWN({code})")

    def _result_to_reason(self, code):
        # Convert backend result enums into stable programmatic reason strings.
        mapping = {
            int(SGRCtrlResult.SUCCESS): "success",
            int(SGRCtrlResult.ERROR): "error",
            int(SGRCtrlResult.PREEMPT): "preempt",
            int(SGRCtrlResult.PLAN_NOT_FOUND): "plan_not_found",
            int(SGRCtrlResult.GRASP_FAILD): "grasp_failed",
        }
        return mapping.get(int(code), "unknown")

    def _send_goal(self, goal, stage_name):
        # Shared action wrapper that turns ROS action outcomes into small
        # structured dicts for the notebook-side controller.
        self.ctrl_client.send_goal(goal)
        finished = self.ctrl_client.wait_for_result(timeout=rospy.Duration.from_sec(self.action_timeout))
        if not finished:
            self.ctrl_client.cancel_goal()
            rospy.logwarn("%s timed out while waiting for %s.", stage_name, self.ctrl_action_name)
            return {
                "success": False,
                "reason": "timeout",
                "stage": stage_name,
                "result_code": None,
                "result_text": "TIMEOUT",
            }

        result = self.ctrl_client.get_result()
        if result is None:
            rospy.logwarn("%s failed: action server returned no result.", stage_name)
            return {
                "success": False,
                "reason": "no_result",
                "stage": stage_name,
                "result_code": None,
                "result_text": "NO_RESULT",
            }

        result_code = int(result.result)
        if result_code != int(SGRCtrlResult.SUCCESS):
            result_text = self._result_to_text(result_code)
            rospy.logwarn("%s failed with %s.", stage_name, result_text)
            return {
                "success": False,
                "reason": self._result_to_reason(result_code),
                "stage": stage_name,
                "result_code": result_code,
                "result_text": result_text,
            }
        return {
            "success": True,
            "reason": "success",
            "stage": stage_name,
            "result_code": result_code,
            "result_text": self._result_to_text(result_code),
        }

    def _send_goal_and_check(self, goal, stage_name):
        # Convenience wrapper for stages that only care about pass/fail.
        return bool(self._send_goal(goal, stage_name).get("success", False))

    def movep(self, position):
        # Move in XYZ using backend-selected orientation heuristics.
        xyz = self._normalize_xyz(position, default_z=self.default_place_z)
        if not self._in_workspace(xyz):
            rospy.logwarn("movep rejected: target %s is outside workspace bounds.", xyz.tolist())
            return False
        move_goal = self._build_goal(SGRCtrlGoal.ACTION_TYPE_XYZ, SGRCtrlGoal.GRASP_NONE, xyz)
        ok = self._send_goal_and_check(move_goal, "MOVE_XYZ")
        if ok:
            self._last_ee_pos = xyz.copy()
        return ok

    def movep_rpy(self, pose):
        # Move to an explicit XYZ + RPY pose, typically for observation scans.
        values = np.asarray(pose, dtype=np.float32).reshape(-1)
        if values.shape[0] != 6:
            raise ValueError(f"RPY pose must be [x, y, z, roll, pitch, yaw], got: {pose}")
        
        xyz = values[:3]
        if not self._in_workspace(xyz):
            rospy.logwarn("movep_rpy rejected: target %s is outside workspace bounds.", xyz.tolist())
            return False

        goal = SGRCtrlGoal()
        goal.action_type = SGRCtrlGoal.ACTION_TYPE_XYZ_RPY
        goal.grasp_type = SGRCtrlGoal.GRASP_NONE
        goal.pos_x, goal.pos_y, goal.pos_z = map(float, values[:3])
        goal.pos_roll, goal.pos_pitch, goal.pos_yaw = map(float, values[3:6])

        ok = self._send_goal_and_check(goal, "MOVE_XYZ_RPY")
        if ok:
            self._last_ee_pos = xyz.copy()
        return ok
        	
    # Post-action state updates --------------------------------------------
    def set_object_position(self, object_name, xyz, source="action"):
        # Apply a trusted state update after a successful manipulation.
        normalized = self._normalize_name(object_name)
        if not normalized:
            return False
        if normalized in self.fixed_bowl_positions:
            rospy.loginfo("Ignoring %s update for fixed object '%s'.", source, normalized)
            return False

        target_xyz = self._normalize_xyz(xyz, default_z=self.default_place_z)
        positions = self.merge_observations(self.world_state, {normalized: target_xyz})
        self._publish_world_state(positions)
        return True

    def update_world_state_after_action(self, object_name, place_xyz, target_name=None):
        # Only movable blocks are updated from action outcomes in the current model.
        if not isinstance(object_name, str):
            return False

        normalized = self._normalize_name(object_name)
        if not normalized.endswith("block"):
            rospy.loginfo("Skipping action map update for non-block object '%s'.", normalized)
            return False

        try:
            updated = self.set_object_position(normalized, place_xyz, source="action")
        except Exception as exc:
            rospy.logwarn(
                "Failed to update world state after action for %s -> %s: %s",
                object_name,
                target_name,
                exc,
            )
            return False
        return updated

    # High-level pick/place execution --------------------------------------
    def step(self, action):
        # Execute one pick/place request with workspace checks, stack-aware
        # placement, and perception-assisted retries after grasp failures.
        object_name = action.get("object_name") if isinstance(action, dict) else None
        target_name = action.get("target_name") if isinstance(action, dict) else None
        try:
            pick_xyz = self._normalize_xyz(action["pick"], default_z=self.default_pick_z)
            place_xyz = self._normalize_xyz(action["place"], default_z=self.default_place_z)
            place_xyz = self._resolve_place_xyz(place_xyz, object_name=object_name, target_name=target_name,)
            
        except Exception as exc:
            rospy.logwarn("Invalid action payload %s: %s", action, exc)
            return {"success": False, "reason": "invalid_action"}

        if not self._in_workspace(pick_xyz):
            rospy.logwarn("Pick rejected: target %s outside workspace.", pick_xyz.tolist())
            return {"success": False, "reason": "pick_out_of_workspace"}
        if not self._in_workspace(place_xyz):
            rospy.logwarn("Place rejected: target %s outside workspace.", place_xyz.tolist())
            return {"success": False, "reason": "place_out_of_workspace"}

        rospy.loginfo(
            "Executing action via %s | object=%s target=%s pick=%s place=%s",
            self.ctrl_action_name,
            object_name,
            target_name,
            pick_xyz,
            place_xyz,
        )

        normalized_object = self._normalize_name(object_name) if isinstance(object_name, str) else ""
        max_pick_attempts = self.pick_retry_attempts + 1
        pick_attempts = 0
        recovered_after_grasp_fail = False

        for attempt_idx in range(max_pick_attempts):
            pick_attempts = attempt_idx + 1
            pick_goal = self._build_goal(SGRCtrlGoal.ACTION_TYPE_PICK_XYZ, SGRCtrlGoal.GRASP_OPEN, pick_xyz)
            pick_result = self._send_goal(
                pick_goal,
                f"PICK_XYZ attempt {pick_attempts}/{max_pick_attempts}",
            )
            if pick_result.get("success", False):
                recovered_after_grasp_fail = pick_attempts > 1
                break

            if pick_result.get("reason") != "grasp_failed":
                return {
                    "success": False,
                    "reason": "pick_failed",
                    "pick_failure_reason": pick_result.get("reason", "unknown"),
                    "pick_result": pick_result,
                    "attempts": pick_attempts,
                }

            if pick_attempts >= max_pick_attempts:
                return {
                    "success": False,
                    "reason": "pick_grasp_failed",
                    "attempts": pick_attempts,
                    "pick_result": pick_result,
                }

            if not normalized_object or not self.pick_retry_refresh:
                return {
                    "success": False,
                    "reason": "pick_grasp_failed",
                    "attempts": pick_attempts,
                    "retry_skipped_reason": "missing_object_name" if not normalized_object else "retry_refresh_disabled",
                    "pick_result": pick_result,
                }

            rospy.logwarn(
                "Pick attempt %d/%d grasp failed for '%s'. Refreshing world state before retry.",
                pick_attempts,
                max_pick_attempts,
                normalized_object,
            )
            refreshed_state = self.refresh_world_state_strict()
            if refreshed_state is None:
                return {
                    "success": False,
                    "reason": "refresh_failed_after_pick_fail",
                    "attempts": pick_attempts,
                    "pick_result": pick_result,
                }
            if normalized_object not in refreshed_state:
                rospy.logwarn(
                    "Object '%s' is not visible after grasp failure. Stopping retry.",
                    normalized_object,
                )
                return {
                    "success": False,
                    "reason": "object_not_visible_after_pick_fail",
                    "attempts": pick_attempts,
                    "pick_result": pick_result,
                }

            pick_xyz = self._normalize_xyz(refreshed_state[normalized_object], default_z=self.default_pick_z)
            if not self._in_workspace(pick_xyz):
                rospy.logwarn(
                    "Refreshed pick target %s for '%s' is outside workspace.",
                    pick_xyz.tolist(),
                    normalized_object,
                )
                return {
                    "success": False,
                    "reason": "pick_out_of_workspace_after_refresh",
                    "attempts": pick_attempts,
                    "pick_result": pick_result,
                }
            rospy.loginfo(
                "Retrying pick for '%s' at refreshed position %s.",
                normalized_object,
                pick_xyz.tolist(),
            )

        place_goal = self._build_goal(SGRCtrlGoal.ACTION_TYPE_PUT_XYZ, SGRCtrlGoal.GRASP_NONE, place_xyz)
        place_result = self._send_goal(place_goal, "PUT_XYZ")
        if not place_result.get("success", False):
            return {
                "success": False,
                "reason": "put_failed",
                "put_failure_reason": place_result.get("reason", "unknown"),
                "put_result": place_result,
                "attempts": pick_attempts,
                "recovered_after_grasp_fail": recovered_after_grasp_fail,
            }

        self._last_ee_pos = np.array(
            [place_xyz[0], place_xyz[1], min(0.20, place_xyz[2] + 0.08)],
            dtype=np.float32,
        )
        self.update_world_state_after_action(object_name, place_xyz, target_name=target_name)
        return {
            "success": True,
            "attempts": pick_attempts,
            "recovered_after_grasp_fail": recovered_after_grasp_fail,
        }

    def step_sim_and_render(self):
        # Compatibility stub retained for code paths shared with the original
        # simulated tabletop demo.
        # Real robot path: no physics stepping required.
        return None
