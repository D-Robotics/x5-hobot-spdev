# Copyright (c) 2025 D-Robotics Co,.Ltd. All Rights Reserved.
#
# The material in this file is confidential and contains trade secrets
# of D-Robotics Co,.Ltd. This is proprietary information owned by
# D-Robotics Co,.Ltd. No part of this work may be disclosed,
# reproduced, copied, transmitted, or used in any way for any purpose,
# without the express written permission of D-Robotics Co,.Ltd.


from typing import Dict, List, Any, overload, Optional
import numpy as np


# Enum: Tensor data types supported by HB_HBMRuntime
class hbDNNDataType:
    S4: int  # Signed 4-bit integer
    U4: int  # Unsigned 4-bit integer
    S8: int  # Signed 8-bit integer
    U8: int  # Unsigned 8-bit integer
    F16: int  # 16-bit floating point (half precision)
    S16: int  # Signed 16-bit integer
    U16: int  # Unsigned 16-bit integer
    F32: int  # 32-bit floating point (single precision)
    S32: int  # Signed 32-bit integer
    U32: int  # Unsigned 32-bit integer
    F64: int  # 64-bit floating point (double precision)
    S64: int  # Signed 64-bit integer
    U64: int  # Unsigned 64-bit integer
    BOOL8: int  # 8-bit boolean type
    MAX: int  # Max enum value, placeholder


# Enum: Quantization types for tensor data
class hbDNNQuantiType:
    NONE: int  # No quantization applied
    SCALE: int  # Scale-based quantization


# Class: Quantization parameters for tensor
class QuantParams:
    quant_type: hbDNNQuantiType
    """The quantization type used."""

    axis: int
    """The axis along which quantization is applied."""

    scale: np.ndarray
    """Scale factors array used for quantization."""

    zero_point: np.ndarray
    """Zero-point offsets array used for quantization."""


# Class: Scheduling parameters for a single model
class SchedParam:
    priority: int
    """Scheduling priority of the model (0-255).
     Higher value means higher priority."""

    customId: int
    """Custom user-defined ID for task identification."""

    bpu_cores: List[int]
    """List of BPU core indices to run the model on."""

    deviceId: int
    """Device ID on which the model runs."""


class HB_HBMRuntime:
    """
    Main class representing HB_HBMRuntime runtime environment.
    Provides model loading, scheduling, and inference APIs.
    """

    # Constructors
    def __init__(self, model_file: str):
        """
        Initialize the runtime with a single model file path.

        :param model_file: Path to a single model file
        """
        ...

    def __init__(self, model_files: List[str]):
        """
        Initialize the runtime with multiple model file paths.

        :param model_files: List of model file paths
        """
        ...

    @staticmethod
    def version() -> str:
        """
        Get the version string of the HB_HBMRuntime library.

        :return: Version string
        """
        ...

    # Model metadata properties

    @property
    def model_names(self) -> List[str]:
        """
        List of model names loaded in the runtime.

        :return: List of model name strings
        """
        ...

    @property
    def model_count(self) -> int:
        """
        Number of models currently loaded.

        :return: Number of loaded models
        """
        ...

    # Input tensor metadata

    @property
    def input_counts(self) -> Dict[str, int]:
        """
        Mapping from model name to number of input tensors.

        :return: Dict mapping model name to input tensor count
        """
        ...

    @property
    def input_names(self) -> Dict[str, List[str]]:
        """
        Mapping from model name to list of input tensor names.

        :return: Dict mapping model name to list of input tensor names
        """
        ...

    @property
    def input_descs(self) -> Dict[str, Dict[str, str]]:
        """
        Input tensor descriptions indexed by model name and input tensor name.

        :return: Nested dict {model_name: {input_name: description}}
        """
        ...

    @property
    def input_shapes(self) -> Dict[str, Dict[str, List[int]]]:
        """
        Shapes of input tensors for each model and input name.

        :return: Nested dict {model_name: {input_name: [dim1, dim2, ...]}}
        """
        ...

    @property
    def input_dtypes(self) -> Dict[str, Dict[str, hbDNNDataType]]:
        """
        Data types of input tensors for each model and input name.

        :return: Nested dict {model_name: {input_name: data_type}}
        """
        ...

    @property
    def input_quants(self) -> Dict[str, Dict[str, QuantParams]]:
        """
        Quantization parameters of input tensors for each model and input name.

        :return: Nested dict {model_name: {input_name: QuantParams}}
        """
        ...

    @property
    def input_strides(self) -> Dict[str, Dict[str, List[int]]]:
        """
        Memory strides for input tensors for each model and input name.

        :return: Nested dict {model_name: {input_name:
         [stride_dim1, stride_dim2, ...]}}
        """
        ...

    # Output tensor metadata

    @property
    def output_counts(self) -> Dict[str, int]:
        """
        Number of output tensors per model.

        :return: Dict mapping model name to output tensor count
        """
        ...

    @property
    def output_names(self) -> Dict[str, List[str]]:
        """
        List of output tensor names per model.

        :return: Dict mapping model name to list of output tensor names
        """
        ...

    @property
    def output_descs(self) -> Dict[str, Dict[str, str]]:
        """
        Output tensor descriptions indexed by model and output tensor name.

        :return: Nested dict {model_name: {output_name: description}}
        """
        ...

    @property
    def output_shapes(self) -> Dict[str, Dict[str, List[int]]]:
        """
        Shapes of output tensors for each model and output name.

        :return: Nested dict {model_name: {output_name: [dim1, dim2, ...]}}
        """
        ...

    @property
    def output_dtypes(self) -> Dict[str, Dict[str, hbDNNDataType]]:
        """
        Data types of output tensors for each model and output name.

        :return: Nested dict {model_name: {output_name: data_type}}
        """
        ...

    @property
    def output_quants(self) -> Dict[str, Dict[str, QuantParams]]:
        """
        Quantization parameters for output tensors
         for each model and output name.

        :return: Nested dict {model_name: {output_name: QuantParams}}
        """
        ...

    @property
    def output_strides(self) -> Dict[str, Dict[str, List[int]]]:
        """
        Memory strides for output tensors for each model and output name.

        :return: Nested dict {model_name:
        {output_name: [stride_dim1, stride_dim2, ...]}}
        """
        ...

    # Model and HBM file descriptions

    @property
    def model_descs(self) -> Dict[str, str]:
        """
        Descriptions of loaded models.

        :return: Dict mapping model name to description string
        """
        ...

    @property
    def hbm_descs(self) -> Dict[str, str]:
        """
        Descriptions read from HBM files associated with models.

        :return: Dict mapping model name to HBM file description string
        """
        ...

    # Scheduling parameters

    @property
    def sched_params(self) -> Dict[str, SchedParam]:
        """
        Scheduling parameters for all loaded models.

        :return: Dict mapping model name to SchedParam object
        """
        ...

    def set_scheduling_params(
        self,
        priority: Optional[Dict[str, int]] = ...,
        bpu_cores: Optional[Dict[str, List[int]]] = ...,
        custom_id: Optional[Dict[str, int]] = ...,
        device_id: Optional[Dict[str, int]] = ...,
    ) -> None:
        """
        Set scheduling parameters for models. Each argument is optional and
        can update the corresponding field in the scheduling parameters.

        :param priority: Mapping from model name to scheduling priority (0-255)
        :param bpu_cores: Mapping from model name to list of BPU core indices
        :param custom_id: Mapping from model name to custom IDs
        :param device_id: Mapping from model name to device IDs
        """
        ...

    # Inference methods with overloads for different input types

    @overload
    def run(self, input_tensor: np.ndarray, **kwargs: Any) \
            -> Dict[str, Dict[str, np.ndarray]]:
        """
        Run inference with a single input tensor for a single-input model.

        :param input_tensor: numpy ndarray as input tensor
        :return: Nested dict {model_name: {output_name: numpy ndarray output}}
        """
        ...

    @overload
    def run(self, input_tensors: Dict[str, np.ndarray], **kwargs: Any) \
            -> Dict[str, Dict[str, np.ndarray]]:
        """
        Run inference with a dict of input tensor name
        to tensor for a single model.

        :param input_tensors: Dict mapping input tensor names to numpy ndarrays
        :return: Nested dict {model_name: {output_name: numpy ndarray output}}
        """
        ...

    @overload
    def run(self, multi_input_tensors: Dict[str, Dict[str, np.ndarray]], **kwargs: Any) -> Dict[str, Dict[str, np.ndarray]]:
        """
        Run inference with multiple models, each having multiple input tensors.

        :param multi_input_tensors: Dict mapping model names to dicts
        of input names to numpy ndarrays
        :return: Nested dict {model_name: {output_name: numpy ndarray output}}
        """
        ...

    def run(self, *args, **kwargs) -> Dict[str, Dict[str, np.ndarray]]:
        """
        General run method dispatching to specific overloads based on input.

        :return: Nested dict {model_name: {output_name: numpy ndarray output}}
        """
        ...
