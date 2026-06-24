// Copyright (c) 2025 D-Robotics Co,.Ltd. All Rights Reserved.
//
// The material in this file is confidential and contains trade secrets
// of D-Robotics Co,.Ltd. This is proprietary information owned by
// D-Robotics Co,.Ltd. No part of this work may be disclosed,
// reproduced, copied, transmitted, or used in any way for any purpose,
// without the express written permission of D-Robotics Co,.Ltd.

#include "HB_RuntimeUtils.hpp"

/**
 * @brief Calculate the total number of elements in a given NumPy array.
 *
 * This function multiplies all dimensions of the input NumPy array
 * to determine the total number of scalar elements it contains.
 *
 * @param[in] arr A py::array object representing the input NumPy array.
 * @return int64_t The total number of elements in the array.
 *
 * @note This function assumes the input array has a valid shape.
 *       For example, an array with shape (3, 224, 224) returns 150528.
 */
int64_t get_array_total_elements(const py::array& arr) {
    const ssize_t* shape = arr.shape();  // Get pointer to shape array, e.g., [3, 224, 224]
    int64_t total = 1;

    // Iterate through all dimensions and multiply sizes
    for (int i = 0; i < arr.ndim(); ++i) {
        total *= shape[i];  // Accumulate total number of elements
    }

    return total;
}

/**
 * @brief Get the size in bytes of a single element of a given tensor data type.
 *
 * This function maps the enum value representing the tensor data type
 * to its corresponding element size in bytes.
 *
 * @param[in] type Integer representing the hbDNN tensor data type.
 *             Supported types include:
//  *             - HB_DNN_TENSOR_TYPE_BOOL8, HB_DNN_TENSOR_TYPE_S8, HB_DNN_TENSOR_TYPE_U8 (1 byte)
//  *             - HB_DNN_TENSOR_TYPE_F16, HB_DNN_TENSOR_TYPE_S16, HB_DNN_TENSOR_TYPE_U16 (2 bytes)
//  *             - HB_DNN_TENSOR_TYPE_F32, HB_DNN_TENSOR_TYPE_S32, HB_DNN_TENSOR_TYPE_U32 (4 bytes)
//  *             - HB_DNN_TENSOR_TYPE_F64, HB_DNN_TENSOR_TYPE_S64, HB_DNN_TENSOR_TYPE_U64 (8 bytes)
 *
 *             - HB_DNN_IMG_TYPE_Y, (1 byte)
 *             - HB_DNN_IMG_TYPE_NV12, HB_DNN_IMG_TYPE_NV12_SEPARATE, （12bit ≈ 2byte）
 *             - HB_DNN_IMG_TYPE_YUV444, HB_DNN_IMG_TYPE_RGB, HB_DNN_IMG_TYPE_BGR,（24bit = 3byte）
 *             - HB_DNN_TENSOR_TYPE_S4,HB_DNN_TENSOR_TYPE_U4,（4bit ≈ 1btye）
 *             - HB_DNN_TENSOR_TYPE_BOOL8, HB_DNN_TENSOR_TYPE_S8, HB_DNN_TENSOR_TYPE_U8 (1 byte)
 *             - HB_DNN_TENSOR_TYPE_F16, HB_DNN_TENSOR_TYPE_S16, HB_DNN_TENSOR_TYPE_U16 (2 bytes)
 *             - HB_DNN_TENSOR_TYPE_F32, HB_DNN_TENSOR_TYPE_S32, HB_DNN_TENSOR_TYPE_U32 (4 bytes)
 *             - HB_DNN_TENSOR_TYPE_F64, HB_DNN_TENSOR_TYPE_S64, HB_DNN_TENSOR_TYPE_U64 (8 bytes)
 *
 * @return int32_t Size in bytes of one element of the specified type.
 *
 * @throws std::runtime_error If the input type is not supported.
 */
 // 这里可能要改成 bit 返回比较好，如果是字节数，可能不够精确。
int32_t get_element_size(int32_t type) {
  switch (type) {
    // case HB_DNN_TENSOR_TYPE_BOOL8:
    case HB_DNN_IMG_TYPE_NV12:
    case HB_DNN_IMG_TYPE_Y:
    case HB_DNN_TENSOR_TYPE_S4:
    case HB_DNN_TENSOR_TYPE_U4:
    case HB_DNN_TENSOR_TYPE_S8:
    case HB_DNN_TENSOR_TYPE_U8:
      return 1;  ///< 1 byte per element (8-bit types)
    // case HB_DNN_IMG_TYPE_NV12:
    case HB_DNN_IMG_TYPE_NV12_SEPARATE:
    case HB_DNN_TENSOR_TYPE_F16:
    case HB_DNN_TENSOR_TYPE_S16:
    case HB_DNN_TENSOR_TYPE_U16:
      return 2;  ///< 2 bytes per element (16-bit types)
    case HB_DNN_IMG_TYPE_YUV444:
    case HB_DNN_IMG_TYPE_RGB:
    case HB_DNN_IMG_TYPE_BGR:
      return 3;  ///< 3 bytes per element (24-bit types)
    case HB_DNN_TENSOR_TYPE_F32:
    case HB_DNN_TENSOR_TYPE_S32:
    case HB_DNN_TENSOR_TYPE_U32:
      return 4;  ///< 4 bytes per element (32-bit types)
    case HB_DNN_TENSOR_TYPE_F64:
    case HB_DNN_TENSOR_TYPE_S64:
    case HB_DNN_TENSOR_TYPE_U64:
      return 8;  ///< 8 bytes per element (64-bit types)
    default:
      throw std::runtime_error("GetElementSize failed! input tensor type not supported");
      return -1;  ///< Unreachable, but avoids compiler warning
  }
}


/**
 * @brief Calculate the product of elements in a dimension array.
 *
 * This function computes the total number of elements represented by
 * the shape dimensions of a tensor or array.
 *
 * @tparam T The integer type of the dimension elements (e.g., int, size_t).
 *
 * @param[in] dim Pointer to an array containing the size of each dimension.
 * @param[in] dim_num Number of dimensions (length of the dim array).
 *
 * @return int64_t The total number of elements (product of all dimension sizes).
 *
 * @note The result may overflow if the product exceeds int64_t range.
 */
template <typename T>
int64_t get_prod_size(T const *dim, uint32_t dim_num) {
  int64_t size{1};  ///< Initialize product to 1
  for (uint32_t idx{0}; idx < dim_num; idx++) {
    size *= dim[idx];  ///< Multiply by each dimension size
  }
  return size;  ///< Return the total product
}

/**
 * @brief Recursively copy input tensor data to output buffer with padding based on strides.
 *
 * This function copies data from the input buffer to the output buffer,
 * handling multi-dimensional tensors where output strides may include padding.
 * It uses recursion to handle each dimension and copies data block-wise.
 *
 * @param[out] output_ptr Pointer to the output buffer where padded data will be written.
 * @param[in] input_ptr Pointer to the input data buffer.
 * @param[in] dim_num Number of dimensions of the tensor.
 * @param[in] dim Pointer to an array representing the size of each dimension.
 * @param[in] stride Pointer to an array representing the stride (in bytes) for each dimension in the output buffer.
 * @param[in] element_size Size in bytes of each individual element in the tensor.
 *
 * @note If the tensor is 1-dimensional, a simple memcpy is used.
 *       For higher dimensions, the function recurses over each sub-dimension.
 */
// void add_padding_core(void *output_ptr, void const *input_ptr, uint32_t dim_num,
//                       uint32_t const *dim, int64_t const *stride,
//                       uint32_t element_size) {
//   if (dim_num == 1) {
//     // For 1D tensor, directly copy the data block
//     memcpy(output_ptr, input_ptr, element_size * dim[0]);
//     return;
//   }

//   char const *in_ptr{reinterpret_cast<char const *>(input_ptr)};
//   char *out_ptr{reinterpret_cast<char *>(output_ptr)};
//   for (int32_t idx{0U}; idx < dim[0]; idx++) {
//     // Calculate the size of the sub-block for the remaining dimensions
//     auto size{get_prod_size(dim + 1, dim_num - 1) * element_size};
//     // Calculate pointer to current input sub-block
//     char const *input{in_ptr + idx * size};
//     // Calculate pointer to current output sub-block using stride (which may include padding)
//     void *output{out_ptr + stride[0] * idx};
//     // Recurse for next dimension
//     add_padding_core(output, input, dim_num - 1, dim + 1, stride + 1,
//                      element_size);
//   }
// }

void add_padding_core(void *output_ptr, void const *input_ptr, uint32_t dim_num,
                      uint32_t const *dim, int64_t const *stride,
                      float_t element_size) {
  if (dim_num == 1) {
    // 对于 1D tensor，需要特殊处理 12bit 元素
    uint32_t num_elements = dim[0];
    if (element_size == 1.5f) {
      // 处理 12bit 元素
      uint8_t const *in_byte_ptr = reinterpret_cast<uint8_t const *>(input_ptr);
      uint8_t *out_byte_ptr = reinterpret_cast<uint8_t *>(output_ptr);
      
      // 每 2 个元素占用 3 个字节
      for (uint32_t i = 0; i < num_elements; i += 2) {
        if (i + 1 < num_elements) {
          // 处理一对元素 (2个元素 = 3字节)
          uint16_t elem1 = *reinterpret_cast<uint16_t const *>(in_byte_ptr);
          uint16_t elem2 = *(in_byte_ptr + 2);
          
          // 将 12bit 数据解包并重新打包到输出（这里需要具体的数据布局信息）
          // 假设输入是紧凑的 12bit 数据
          uint32_t packed = (elem1 & 0xFFF) | ((elem2 & 0xFFF) << 12);
          *reinterpret_cast<uint32_t *>(out_byte_ptr) = packed;
          
          in_byte_ptr += 3;
          out_byte_ptr += 3; // 或者根据 stride 调整
        } else {
          // 处理最后一个单独的元素
          uint16_t elem = *reinterpret_cast<uint16_t const *>(in_byte_ptr);
          *reinterpret_cast<uint16_t *>(out_byte_ptr) = elem & 0xFFF;
        }
      }
    } else {
      // 对于整数字节大小，使用 memcpy
      memcpy(output_ptr, input_ptr, static_cast<size_t>(element_size * num_elements));
    }
    return;
  }

  char const *in_ptr{reinterpret_cast<char const *>(input_ptr)};
  char *out_ptr{reinterpret_cast<char *>(output_ptr)};
  
  for (uint32_t idx = 0; idx < dim[0]; idx++) {
    // 计算子块的大小（以字节为单位）
    uint32_t sub_block_elements = get_prod_size(dim + 1, dim_num - 1);
    size_t sub_block_bytes = static_cast<size_t>(element_size * sub_block_elements);
    
    // 确保字节数是整数（对于 12bit 需要特殊处理）
    if (element_size == 1.5f) {
      // 对于 12bit，每 2 个元素占用 3 字节
      sub_block_bytes = (sub_block_elements * 3 + 1) / 2; // 向上取整
    }
    
    char const *input = in_ptr + idx * sub_block_bytes;
    
    // 计算输出指针，考虑 stride（以字节为单位）
    void *output = out_ptr + static_cast<size_t>(stride[0] * idx * element_size);
    
    // 递归处理下一个维度
    add_padding_core(output, input, dim_num - 1, dim + 1, stride + 1, element_size);
  }
}
/**
 * @brief Copy input tensor data into output buffer with padding according to strides.
 *
 * This function serves as a wrapper that performs basic null pointer checks
 * before calling the recursive core function `add_padding_core` to copy
 * and pad the tensor data.
 *
 * @param[out] output Pointer to the output buffer where the padded tensor data will be stored.
 * @param[in] input Pointer to the input tensor data buffer.
 * @param[in] dim_num Number of dimensions of the tensor.
 * @param[in] dim Pointer to an array holding the size of each dimension.
 * @param[in] stride Pointer to an array holding the stride (in bytes) for each dimension in the output buffer.
 * @param[in] element_size Size (in bytes) of each tensor element.
 *
 * @return int32_t Returns 0 on success, -1 if any input pointer is null.
 */
// int32_t add_padding(void *output, void const *input, uint32_t dim_num,
//                     const uint32_t *dim, const int64_t *stride,
//                     uint32_t element_size) {
//   LOGE_AND_RETURN_IF_NULL(output, -1)
//   LOGE_AND_RETURN_IF_NULL(input, -1)
//   LOGE_AND_RETURN_IF_NULL(dim, -1)
//   LOGE_AND_RETURN_IF_NULL(stride, -1)

//   add_padding_core(output, input, dim_num, dim, stride, element_size);
//   return 0;
// }
int32_t add_padding(void *output, void const *input, uint32_t dim_num,
                    const uint32_t *dim, const int64_t *stride,
                    float_t element_size) {
  LOGE_AND_RETURN_IF_NULL(output, -1)
  LOGE_AND_RETURN_IF_NULL(input, -1)
  LOGE_AND_RETURN_IF_NULL(dim, -1)
  LOGE_AND_RETURN_IF_NULL(stride, -1)

  add_padding_core(output, input, dim_num, dim, stride, element_size);
  return 0;
}
