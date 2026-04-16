# For licensing see accompanying LICENSE file.
# Copyright (C) 2025 Apple Inc. All Rights Reserved.

import copy
import os
import time
import torch

from boxfusion.measurement import (
    DepthMeasurementInfo,
    ImageMeasurementInfo)

from boxfusion.batching import (
    Measurement,
    PosedImage,    
    PosedDepth,
    BatchedSensors,
    Sensors)

from typing import Dict, List

IGNORE_KEYS = ["sensor_info", "__key__", "gt", "video_info", "meta"]
FAST_DEPTH_STATS_MODE = os.environ.get("BOXFUSION_FAST_DEPTH_STATS_MODE", "").strip().lower()

def move_device_like(src: torch.Tensor, dst: torch.Tensor) -> torch.Tensor:
    try:
        return src.to(dst)
    except:
        return src.to(dst.device)

def move_to_current_device(x, t):
    if isinstance(x, (list, tuple)):
        return [move_device_like(x_, t) for x_ in x]
    
    return move_device_like(x, t)

def move_input_to_current_device(batched_input: Sensors, t: torch.Tensor):
    # Assume only two levels of nesting for now.
    return { name: { name_: move_to_current_device(m, t) for name_, m in s.items() } for name, s in batched_input.items() }

class Augmentor(object):
    def __init__(self, measurement_keys=None):
        self.measurement_keys = measurement_keys
        self.last_profile = {}

    def package(self, sample) -> Dict[str, Dict[str, Measurement]]:
        # Simply everything into "Packages" to make it more amenable for a training pipeline.
        # Essentially return Dict
        # Make sure everything is contiguous. channels -> first.
        package_t0 = time.perf_counter()
        profile = {
            "sensor_info_deepcopy_sec": 0.0,
            "measurement_wrap_sec": 0.0,
            "total_sec": 0.0,
        }
        result = {}
        for sensor_name, sensor_data in sample.items():
            if sensor_name in IGNORE_KEYS:
                continue

            if not isinstance(sensor_data, dict):
                continue

            sensor_result = {}
            deepcopy_t0 = time.perf_counter()
            sensor_info = copy.deepcopy(getattr(sample["sensor_info"], sensor_name))
            profile["sensor_info_deepcopy_sec"] += float(time.perf_counter() - deepcopy_t0)
            for measurement_name, measurement in sensor_data.items():
                measurement_key = os.path.join(sensor_name, measurement_name)
                if (self.measurement_keys is not None) and (measurement_key not in self.measurement_keys):
                    # Make sure to delete from sensor info as well.
                    if sensor_info.has(measurement_name):
                        sensor_info.remove(measurement_name)
                    continue

                measurement_info = getattr(sensor_info, measurement_name)
                wrap_t0 = time.perf_counter()
                if isinstance(measurement_info, DepthMeasurementInfo):
                    sensor_result[measurement_name] = PosedDepth(
                        sample[sensor_name][measurement_name][-1],
                        measurement_info,
                        sensor_info)
                elif isinstance(measurement_info, ImageMeasurementInfo):
                    sensor_result[measurement_name] = PosedImage(
                        sample[sensor_name][measurement_name][-1],
                        measurement_info,
                        sensor_info)
                profile["measurement_wrap_sec"] += float(time.perf_counter() - wrap_t0)

            # Don't include if empty.
            if sensor_result:
                result[sensor_name] = sensor_result

        profile["total_sec"] = float(time.perf_counter() - package_t0)
        self.last_profile = profile
        return result

class Preprocessor(object):
    def __init__(self,
                 square_pad=[256, 384, 512, 640, 768, 896, 1024],
                 size_divisibility=32,
                 pixel_mean=[123.675, 116.28, 103.53],
                 pixel_std=[58.395, 57.12, 57.375],
                 device=None):
        self.square_pad = square_pad
        self.size_divisibility = size_divisibility
        self.pixel_mean = torch.tensor(pixel_mean).view(-1, 1, 1)
        self.pixel_std = torch.tensor(pixel_std).view(-1, 1, 1)
        self.device = device
        self.last_profile = {}

    @staticmethod
    def standardize_depth_map(img, trunc_value=0.1):
        # Always do this on CPU! MPS has some surprising behavior.
        device = img.device
        img = img.cpu()
        img[img <= 0.0] = torch.nan

        if FAST_DEPTH_STATS_MODE == "kthvalue":
            valid_img = img[torch.isfinite(img)]
            if len(valid_img) <= 1:
                trunc_mean = torch.tensor(0.0).to(img)
                trunc_std = torch.tensor(1.0).to(img)
            else:
                lo_idx = max(1, int(trunc_value * len(valid_img)))
                hi_idx = min(len(valid_img), max(lo_idx, int((1 - trunc_value) * len(valid_img))))
                lo = torch.kthvalue(valid_img, lo_idx).values
                hi = torch.kthvalue(valid_img, hi_idx).values
                trunc_img = valid_img[(valid_img >= lo) & (valid_img <= hi)]
                if len(trunc_img) <= 1:
                    trunc_mean = valid_img.mean()
                    trunc_std = torch.sqrt(valid_img.var() + 1e-2)
                else:
                    trunc_mean = trunc_img.mean()
                    trunc_var = trunc_img.var()
                    trunc_std = torch.sqrt(trunc_var + 1e-2)
        else:
            sorted_img = torch.sort(torch.flatten(img))[0]
            # Remove nan, nan at the end of sort
            num_nan = sorted_img.isnan().sum()
            if num_nan > 0:
                sorted_img = sorted_img[:-num_nan]
            # Remove outliers
            trunc_img = sorted_img[int(trunc_value * len(sorted_img)): int((1 - trunc_value) * len(sorted_img))]
            if len(trunc_img) <= 1:
                # guard against no valid Jasper.
                trunc_mean = torch.tensor(0.0).to(img)
                trunc_std = torch.tensor(1.0).to(img)
            else:
                trunc_mean = trunc_img.mean()
                trunc_var = trunc_img.var()
                trunc_std = torch.sqrt(trunc_var + 1e-2)

        # Replace nan by mean
        img = torch.nan_to_num(img, nan=trunc_mean)

        # Standardize
        img = (img - trunc_mean) / trunc_std

        # return the scale parameters for encoding.
        return img.to(device), torch.tensor([trunc_mean, trunc_std]).to(device)

    def normalize(self, batched_input: Sensors, profile=None):
        # Happens in-place.
        for sensor_name, sensor in batched_input.items():
            for measurement_name, measurement in sensor.items():
                if measurement_name in ["features"]:
                    continue

                if measurement.__orig_class__ in (PosedDepth,):
                    t0 = time.perf_counter()
                    measurement.data, scaling = Preprocessor.standardize_depth_map(measurement.data)
                    measurement.info = measurement.info.normalize(scaling[None])
                    if profile is not None:
                        profile["depth_normalize_sec"] += float(time.perf_counter() - t0)
                elif measurement.__orig_class__ in (PosedImage,):
                    t0 = time.perf_counter()
                    measurement.data = (measurement.data.float() - self.pixel_mean.to(measurement.data)) / self.pixel_std.to(measurement.data)
                    if profile is not None:
                        profile["image_normalize_sec"] += float(time.perf_counter() - t0)

        return batched_input

    def batch(self, batched_inputs: List[Sensors], profile=None) -> List[BatchedSensors]:
        sensor_names = batched_inputs[0].keys()
        result = {}
        for sensor_name in sensor_names:
            measurement_names = batched_inputs[0][sensor_name].keys()
            sensor_result = {}
            for measurement_name in measurement_names:
                batched_measurements = [bi[sensor_name][measurement_name] for bi in batched_inputs]
                if measurement_name in ["features"]:
            
                    sensor_result["features"] = batched_measurements[0]
                    continue

                # Hacky way to pass some additional constraints.
                batching_kwargs = {}
                if batched_measurements[0].__orig_class__ in (PosedDepth,):
                    # Very bad, but assume the PosedImage here gets processed first, so that
                    # square_pad and rgb_size are assigned.
                    rgb_to_depth_ratio = round(rgb_size[0] / batched_measurements[0].info.size[0])
                    if rgb_to_depth_ratio not in [1, 2, 4]:
                        raise ValueError(f"Unsupported rgb -> depth ratio: {rgb_to_depth_ratio}")

                    # note: square_pad should always be divisible by the given ratios: e.g. 1, 2, 4.
                    batching_kwargs = dict(
                        size_divisibility=self.size_divisibility,
                        padding_constraints={
                            "size_divisibility": self.size_divisibility,
                            "square_size": square_pad // rgb_to_depth_ratio
                        })
                elif batched_measurements[0].__orig_class__ in (PosedImage,):
                    # Backbone sizes are computed w.r.t image. We may need
                    # to adjust them to depth or other sensors with different sizes.
                    square_pad = self.square_pad
                    rgb_size = batched_measurements[0].info.size
                    if isinstance(square_pad, (list,)):
                        longest_edge = max([max(bm.info.size) for bm in batched_measurements])
                        # print('square_pad',square_pad,longest_edge)
                        square_pad = int(min([s for s in square_pad if s >= longest_edge]))

                    batching_kwargs = dict(
                        size_divisibility=self.size_divisibility,
                        padding_constraints={
                            "size_divisibility": self.size_divisibility,
                            "square_size": square_pad
                        })

                batch_t0 = time.perf_counter()
                batched_measurements = Measurement.batch(
                    batched_measurements,
                    **batching_kwargs)
                batch_elapsed = float(time.perf_counter() - batch_t0)
                if profile is not None:
                    if batched_measurements.__orig_class__ in (PosedDepth,):
                        profile["depth_batch_sec"] += batch_elapsed
                    elif batched_measurements.__orig_class__ in (PosedImage,):
                        profile["image_batch_sec"] += batch_elapsed

                sensor_result[measurement_name] = batched_measurements

            result[sensor_name] = sensor_result

        return result

    def __call__(self, batches):
        for batch in batches:
            if isinstance(batch, tuple):
                # Probably inference with GT.
                input_, gt_ = batch
                if self.device is not None:
                    input_ = move_input_to_current_device(input_, self.device)

                yield self.preprocess([input_]), gt_
            else:
                yield self.preprocess(batch)

    def preprocess(self, batched_inputs: List[Sensors]) -> List[Sensors]:
        profile = {
            "normalize_total_sec": 0.0,
            "depth_normalize_sec": 0.0,
            "image_normalize_sec": 0.0,
            "batch_total_sec": 0.0,
            "image_batch_sec": 0.0,
            "depth_batch_sec": 0.0,
            "total_sec": 0.0,
        }
        preprocess_t0 = time.perf_counter()
        normalize_t0 = time.perf_counter()
        batched_inputs = [self.normalize(bi, profile=profile) for bi in batched_inputs]
        profile["normalize_total_sec"] = float(time.perf_counter() - normalize_t0)

        batch_t0 = time.perf_counter()
        result = self.batch(batched_inputs, profile=profile)
        profile["batch_total_sec"] = float(time.perf_counter() - batch_t0)
        profile["total_sec"] = float(time.perf_counter() - preprocess_t0)
        self.last_profile = profile
        return result
