/*
 * WiVRn VR streaming
 * Copyright (C) 2024  Patrick Nicolas <patricknicolas@laposte.net>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */
#include "video_encoder_vulkan.h"

#include "encoder/yuv_converter.h"
#include "util/u_logging.h"
#include "utils/wivrn_vk_bundle.h"
#include <iostream>
#include <stdexcept>

static uint32_t align(uint32_t value, uint32_t alignment)
{
	if (alignment == 0)
		return value;
	return alignment * (1 + (value - 1) / alignment);
}

static vk::VideoEncodeCapabilitiesKHR patch_capabilities(vk::VideoEncodeCapabilitiesKHR caps)
{
	if (caps.rateControlModes & (vk::VideoEncodeRateControlModeFlagBitsKHR::eCbr | vk::VideoEncodeRateControlModeFlagBitsKHR::eVbr) and caps.maxBitrate == 0)
	{
		U_LOG_W("Invalid encode capabilities, disabling rate control");
		caps.rateControlModes = vk::VideoEncodeRateControlModeFlagBitsKHR::eDefault;
	}
	return caps;
}

size_t slot_info::get_slot()
{
	auto i = std::ranges::min_element(frames);
	return i - std::begin(frames);
}
void slot_info::clear()
{
	std::ranges::fill(frames, -1);
}

std::optional<size_t> slot_info::get_ref()
{
	auto i = std::ranges::max_element(frames);
	if (*i >= 0)
		return i - std::begin(frames);
	return {};
}

vk::VideoFormatPropertiesKHR video_encoder_vulkan::select_video_format(
        vk::raii::PhysicalDevice & physical_device,
        const vk::PhysicalDeviceVideoFormatInfoKHR & format_info)
{
	for (const auto & video_fmt_prop: physical_device.getVideoFormatPropertiesKHR(format_info))
	{
		// TODO: do something smart if there is more than one
		return video_fmt_prop;
	}
	throw std::runtime_error("No suitable image format found");
}

video_encoder_vulkan::video_encoder_vulkan(wivrn_vk_bundle & vk, vk::Rect2D rect, vk::VideoEncodeCapabilitiesKHR in_encode_caps, float fps, uint64_t bitrate) :
        vk(vk), rect(rect), encode_caps(patch_capabilities(in_encode_caps)), fps(fps)
{
	// Initialize Rate control
	U_LOG_D("Supported rate control modes: %s", vk::to_string(encode_caps.rateControlModes).c_str());

	if (encode_caps.rateControlModes & (vk::VideoEncodeRateControlModeFlagBitsKHR::eCbr | vk::VideoEncodeRateControlModeFlagBitsKHR::eVbr))
	{
		U_LOG_D("Maximum bitrate: %ld", encode_caps.maxBitrate / 1'000'000);
		if (encode_caps.maxBitrate < bitrate)
		{
			U_LOG_W("Configured bitrate %ldMB/s is higher than max supported %ld",
			        bitrate / 1'000'000,
			        encode_caps.maxBitrate / 1'000'000);
		}
	}

	rate_control_layer = vk::VideoEncodeRateControlLayerInfoKHR{
	        .averageBitrate = std::min(bitrate, encode_caps.maxBitrate),
	        .maxBitrate = std::min(2 * bitrate, encode_caps.maxBitrate),
	        .frameRateNumerator = uint32_t(fps * 1'000'000),
	        .frameRateDenominator = 1'000'000,
	};
	rate_control = vk::VideoEncodeRateControlInfoKHR{
	        .layerCount = 1,
	        .pLayers = &rate_control_layer,
	        .virtualBufferSizeInMs = 5'000,
	        .initialVirtualBufferSizeInMs = 4'000,
	};

	if (encode_caps.rateControlModes & vk::VideoEncodeRateControlModeFlagBitsKHR::eCbr)
	{
		rate_control_layer.maxBitrate = rate_control_layer.averageBitrate;
		rate_control->rateControlMode = vk::VideoEncodeRateControlModeFlagBitsKHR::eCbr;
	}
	else if (encode_caps.rateControlModes & vk::VideoEncodeRateControlModeFlagBitsKHR::eVbr)
	{
		rate_control->rateControlMode = vk::VideoEncodeRateControlModeFlagBitsKHR::eVbr;
	}
	else
	{
		U_LOG_W("No suitable rate control available, reverting to default");
		rate_control.reset();
	}
}

void video_encoder_vulkan::init(const vk::VideoCapabilitiesKHR & video_caps,
                                const vk::VideoProfileInfoKHR & video_profile,
                                size_t num_input_images,
                                void * video_session_create_next,
                                void * session_params_next)
{
	fence = vk.device.createFence({});

	vk::VideoProfileListInfoKHR video_profile_list{
	        .profileCount = 1,
	        .pProfiles = &video_profile,
	};

	// Input image
	vk::VideoFormatPropertiesKHR picture_format;
	{
		vk::PhysicalDeviceVideoFormatInfoKHR video_fmt{
		        .pNext = &video_profile_list,
		        .imageUsage = vk::ImageUsageFlagBits::eVideoEncodeSrcKHR,
		};

		picture_format = select_video_format(vk.physical_device, video_fmt);

		if (picture_format.format != vk::Format::eG8B8R82Plane420Unorm)
		{
			throw std::runtime_error("Unsupported format " +
			                         vk::to_string(picture_format.format) +
			                         " for encoder input image");
		}

		vk::Extent3D aligned_extent{
		        .width = align(rect.extent.width, encode_caps.encodeInputPictureGranularity.width),
		        .height = align(rect.extent.height, encode_caps.encodeInputPictureGranularity.height),
		        .depth = 1,
		};

		// TODO: check format capabilities
		vk::ImageCreateInfo img_create_info{
		        .pNext = &video_profile_list,
		        .flags = picture_format.imageCreateFlags,
		        .imageType = picture_format.imageType,
		        .format = picture_format.format,
		        .extent = aligned_extent,
		        .mipLevels = 1,
		        .arrayLayers = 1,
		        .samples = vk::SampleCountFlagBits::e1,
		        .tiling = picture_format.imageTiling,
		        .usage = vk::ImageUsageFlagBits::eTransferDst |
		                 vk::ImageUsageFlagBits::eVideoEncodeSrcKHR,
		        .sharingMode = vk::SharingMode::eExclusive,
		};

		for (size_t i = 0; i < num_input_images; ++i)
		{
			input_images.emplace_back(image_allocation(
			        vk.device,
			        img_create_info,
			        {
			                .usage = VMA_MEMORY_USAGE_AUTO,
			        }));
		}
	}

	// Decode picture buffer (DPB) images
	vk::VideoFormatPropertiesKHR reference_picture_format;
	{
		vk::PhysicalDeviceVideoFormatInfoKHR video_fmt{
		        .pNext = &video_profile_list,
		        .imageUsage = vk::ImageUsageFlagBits::eVideoEncodeDpbKHR,
		};

		reference_picture_format = select_video_format(vk.physical_device, video_fmt);

		// TODO: check format capabilities
		// TODO: use multiple images if array levels are not supported

		vk::Extent3D aligned_extent{
		        .width = align(rect.extent.width, video_caps.pictureAccessGranularity.width),
		        .height = align(rect.extent.height, video_caps.pictureAccessGranularity.height),
		        .depth = 1,
		};

		vk::ImageCreateInfo img_create_info{
		        .pNext = &video_profile_list,
		        .flags = reference_picture_format.imageCreateFlags,
		        .imageType = reference_picture_format.imageType,
		        .format = reference_picture_format.format,
		        .extent = aligned_extent,
		        .mipLevels = 1,
		        .arrayLayers = num_dpb_slots,
		        .samples = vk::SampleCountFlagBits::e1,
		        .tiling = reference_picture_format.imageTiling,
		        .usage = vk::ImageUsageFlagBits::eVideoEncodeDpbKHR,
		        .sharingMode = vk::SharingMode::eExclusive,
		};

		dpb_image = image_allocation(
		        vk.device,
		        img_create_info,
		        {
		                .usage = VMA_MEMORY_USAGE_AUTO,
		        });
	}

	// Output buffer
	{
		// very conservative bound
		output_buffer_size = rect.extent.width * rect.extent.height * 3;
		output_buffer_size = align(output_buffer_size, video_caps.minBitstreamBufferSizeAlignment);
		output_buffer = buffer_allocation(
		        vk.device,
		        {.pNext = &video_profile_list,
		         .size = output_buffer_size,
		         .usage = vk::BufferUsageFlagBits::eVideoEncodeDstKHR,
		         .sharingMode = vk::SharingMode::eExclusive},
		        {
		                .flags = VMA_ALLOCATION_CREATE_HOST_ACCESS_SEQUENTIAL_WRITE_BIT,
		                .usage = VMA_MEMORY_USAGE_AUTO,
		        });
	}

	// video session
	{
		vk::ExtensionProperties std_header_version = this->std_header_version();

		video_session =
		        vk.device.createVideoSessionKHR(vk::VideoSessionCreateInfoKHR{
		                .pNext = video_session_create_next,
		                .queueFamilyIndex = vk.encode_queue_family_index,
		                //.flags = vk::VideoSessionCreateFlagBitsKHR::eAllowEncodeParameterOptimizations,
		                .pVideoProfile = &video_profile,
		                .pictureFormat = picture_format.format,
		                .maxCodedExtent = rect.extent,
		                .referencePictureFormat = reference_picture_format.format,
		                .maxDpbSlots = num_dpb_slots,
		                .maxActiveReferencePictures = 1,
		                .pStdHeaderVersion = &std_header_version,
		        });

		auto video_req = video_session.getMemoryRequirements();
		// FIXME: allocating on a single device memory seems to fail
		std::vector<vk::BindVideoSessionMemoryInfoKHR> video_session_bind;
		for (const auto & req: video_req)
		{
			vk::MemoryAllocateInfo alloc_info{
			        .allocationSize = req.memoryRequirements.size,
			        .memoryTypeIndex = vk.get_memory_type(req.memoryRequirements.memoryTypeBits, {}),
			};

			const auto & mem_item = mem.emplace_back(vk.device.allocateMemory(alloc_info));
			video_session_bind.push_back({
			        .memoryBindIndex = req.memoryBindIndex,
			        .memory = *mem_item,
			        .memoryOffset = 0,
			        .memorySize = alloc_info.allocationSize,
			});
		}
		video_session.bindMemory(video_session_bind);
	}

	// input image view
	for (size_t i = 0; i < num_input_images; ++i)
	{
		vk::ImageViewCreateInfo img_view_create_info{
		        .image = input_images[i],
		        .viewType = vk::ImageViewType::e2D,
		        .format = picture_format.format,
		        .components = picture_format.componentMapping,
		        .subresourceRange = {.aspectMask = vk::ImageAspectFlagBits::eColor,
		                             .baseMipLevel = 0,
		                             .levelCount = 1,
		                             .baseArrayLayer = 0,
		                             .layerCount = 1},
		};
		input_image_views.push_back(vk.device.createImageView(img_view_create_info));
	}

	// DPB image views
	{
		vk::ImageViewCreateInfo img_view_create_info{
		        .image = dpb_image,
		        .viewType = vk::ImageViewType::e2D,
		        .format = reference_picture_format.format,
		        .components = reference_picture_format.componentMapping,
		        .subresourceRange = {.aspectMask = vk::ImageAspectFlagBits::eColor,
		                             .baseMipLevel = 0,
		                             .levelCount = 1,
		                             .layerCount = 1},
		};
		for (size_t i = 0; i < num_dpb_slots; ++i)
		{
			img_view_create_info.subresourceRange.baseArrayLayer = i;
			dpb_image_views.push_back(vk.device.createImageView(img_view_create_info));
		}
	}

	// DPB video picture resource info
	{
		for (auto & dpb_image_view: dpb_image_views)
		{
			dpb_resource.push_back(
			        {
			                .codedExtent = rect.extent,
			                .imageViewBinding = *dpb_image_view,
			        });
		}
	}

	// DPB slot info
	{
		auto std_slots = setup_slot_info(num_dpb_slots);
		assert(std_slots.size() == num_dpb_slots);
		for (size_t i = 0; i < num_dpb_slots; ++i)
		{
			dpb_slots.push_back({
			        .pNext = std_slots[i],
			        .slotIndex = -1,
			        .pPictureResource = nullptr,
			});
		}

		dpb_status = slot_info(num_dpb_slots);
	}

	// video session parameters
	{
		video_session_parameters = vk.device.createVideoSessionParametersKHR({
		        .pNext = session_params_next,
		        .videoSession = *video_session,
		});
	}

	// query pool
	{
		vk::StructureChain query_pool_create = {
		        vk::QueryPoolCreateInfo{
		                .queryType = vk::QueryType::eVideoEncodeFeedbackKHR,
		                .queryCount = 1,

		        },
		        vk::QueryPoolVideoEncodeFeedbackCreateInfoKHR{
		                .pNext = &video_profile,
		                .encodeFeedbackFlags =
		                        vk::VideoEncodeFeedbackFlagBitsKHR::estreamBufferOffsetBit |
		                        vk::VideoEncodeFeedbackFlagBitsKHR::estreamBytesWrittenBit,
		        },
		};

		query_pool = vk.device.createQueryPool(query_pool_create.get());
	}

	// command pool and buffer
	{
		command_pool = vk.device.createCommandPool({
		        .flags = vk::CommandPoolCreateFlagBits::eResetCommandBuffer,
		        .queueFamilyIndex = vk.encode_queue_family_index,
		});

		command_buffer = std::move(vk.device.allocateCommandBuffers({.commandPool = *command_pool,
		                                                             .commandBufferCount = 1})[0]);
	}
}

video_encoder_vulkan::~video_encoder_vulkan()
{
}

std::vector<uint8_t> video_encoder_vulkan::get_encoded_parameters(void * next)
{
	auto [feedback, encoded] = vk.device.getEncodedVideoSessionParametersKHR({
	        .pNext = next,
	        .videoSessionParameters = *video_session_parameters,
	});
	return encoded;
}

void video_encoder_vulkan::Encode(size_t index, bool idr, std::chrono::steady_clock::time_point target_timestamp)
{
	command_buffer.reset();
	command_buffer.begin(vk::CommandBufferBeginInfo{
	        .flags = vk::CommandBufferUsageFlagBits::eOneTimeSubmit,
	});
	command_buffer.resetQueryPool(*query_pool, 0, 1);

	// slot: where the encoded picture will be stored in DPB
	size_t slot = dpb_status.get_slot();
	// ref_slot: which image to use as reference
	auto ref_slot = dpb_status.get_ref();
	assert(not(ref_slot and (*ref_slot == slot)));
	if (not ref_slot)
	{
		ref_slot.reset();
		dpb_status.clear();
		frame_num = 0;
		for (auto & slot: dpb_slots)
		{
			slot.slotIndex = -1;
			slot.pPictureResource = nullptr;
		}
	}
	dpb_status[slot] = frame_num;

	dpb_slots[slot].slotIndex = -1;
	dpb_slots[slot].pPictureResource = &dpb_resource[slot];

	{
		vk::VideoBeginCodingInfoKHR video_coding_begin_info{
		        .pNext = (session_initialized and rate_control) ? &rate_control.value() : nullptr,
		        .videoSession = *video_session,
		        .videoSessionParameters = *video_session_parameters,
		};
		video_coding_begin_info.setReferenceSlots(dpb_slots);
		command_buffer.beginVideoCodingKHR(video_coding_begin_info);
	}

	if (not session_initialized)
	{
		// Initialize encoding session and rate control
		vk::VideoCodingControlInfoKHR control_info{
		        .flags = vk::VideoCodingControlFlagBitsKHR::eReset,
		};
		if (rate_control)
		{
			control_info.flags |= vk::VideoCodingControlFlagBitsKHR::eEncodeRateControl;
			control_info.pNext = &rate_control.value();
		}
		command_buffer.controlVideoCodingKHR(control_info);

		// Set decoded picture buffer to correct layout
		vk::ImageMemoryBarrier2 dpb_barrier{
		        .srcStageMask = vk::PipelineStageFlagBits2KHR::eNone,
		        .srcAccessMask = vk::AccessFlagBits2::eNone,
		        .dstStageMask = vk::PipelineStageFlagBits2KHR::eVideoEncodeKHR,
		        .dstAccessMask = vk::AccessFlagBits2::eVideoEncodeReadKHR | vk::AccessFlagBits2::eVideoEncodeWriteKHR,
		        .oldLayout = vk::ImageLayout::eUndefined,
		        .newLayout = vk::ImageLayout::eVideoEncodeDpbKHR,
		        .image = dpb_image,
		        .subresourceRange = {.aspectMask = vk::ImageAspectFlagBits::eColor,
		                             .baseMipLevel = 0,
		                             .levelCount = 1,
		                             .baseArrayLayer = 0,
		                             .layerCount = uint32_t(dpb_slots.size())},
		};
		command_buffer.pipelineBarrier2({
		        .imageMemoryBarrierCount = 1,
		        .pImageMemoryBarriers = &dpb_barrier,
		});
		session_initialized = true;
	}

	dpb_slots[slot].slotIndex = slot;
	vk::VideoEncodeInfoKHR encode_info{
	        .pNext = encode_info_next(frame_num, slot, ref_slot),
	        .dstBuffer = output_buffer,
	        .dstBufferOffset = 0,
	        .dstBufferRange = output_buffer_size,
	        .srcPictureResource = {.codedExtent = rect.extent,
	                               .baseArrayLayer = 0,
	                               .imageViewBinding = *input_image_views[index]},
	        .pSetupReferenceSlot = &dpb_slots[slot],
	};
	if (ref_slot)
		encode_info.setReferenceSlots(dpb_slots[*ref_slot]);

	command_buffer.beginQuery(*query_pool, 0, {});
	command_buffer.encodeVideoKHR(encode_info);
	command_buffer.endQuery(*query_pool, 0);
	command_buffer.endVideoCodingKHR(vk::VideoEndCodingInfoKHR{});
	command_buffer.end();

	vk::SubmitInfo2 submit{};
	vk::CommandBufferSubmitInfo cmd_info{
	        .commandBuffer = *command_buffer,
	};
	submit.setCommandBufferInfos(cmd_info);
	vk.encode_queue.submit2(submit, *fence);
	++frame_num;
	if (not ref_slot)
	{
		send_idr_data();
	}
	if (auto res = vk.device.waitForFences(*fence, true, 1'000'000'000);
	    res != vk::Result::eSuccess)
	{
		throw std::runtime_error("wait for fences: " + vk::to_string(res));
	}

	// Feedback = offset / size / has overrides
	auto [res, feedback] = query_pool.getResults<uint32_t>(0, 1, 3 * sizeof(uint32_t), 0, vk::QueryResultFlagBits::eWait);
	if (res != vk::Result::eSuccess)
	{
		std::cerr << "device.getQueryPoolResults: " << vk::to_string(res) << std::endl;
	}

	vk.device.resetFences(*fence);

	SendData({((uint8_t *)output_buffer.map()) + feedback[0], feedback[1]}, true);
}

void video_encoder_vulkan::PresentImage(size_t index, yuv_converter & src_yuv, vk::raii::CommandBuffer & cmd_buf)
{
	vk::ImageMemoryBarrier2 barrier{
	        .srcStageMask = vk::PipelineStageFlagBits2KHR::eNone,
	        .srcAccessMask = vk::AccessFlagBits2::eNone,
	        .dstStageMask = vk::PipelineStageFlagBits2KHR::eTransfer,
	        .dstAccessMask = vk::AccessFlagBits2::eTransferWrite,
	        .oldLayout = vk::ImageLayout::eUndefined,
	        .newLayout = vk::ImageLayout::eTransferDstOptimal,
	        .image = input_images[index],
	        .subresourceRange = {.aspectMask = vk::ImageAspectFlagBits::eColor,
	                             .baseMipLevel = 0,
	                             .levelCount = 1,
	                             .baseArrayLayer = 0,
	                             .layerCount = 1},
	};
	vk::DependencyInfo dep_info{
	        .imageMemoryBarrierCount = 1,
	        .pImageMemoryBarriers = &barrier,
	};
	cmd_buf.pipelineBarrier2(dep_info);

	cmd_buf.copyImage(
	        src_yuv.luma,
	        vk::ImageLayout::eTransferSrcOptimal,
	        input_images[index],
	        vk::ImageLayout::eTransferDstOptimal,
	        vk::ImageCopy{
	                .srcSubresource = {
	                        .aspectMask = vk::ImageAspectFlagBits::eColor,
	                        .layerCount = 1,
	                },
	                .srcOffset = {
	                        .x = rect.offset.x,
	                        .y = rect.offset.y,
	                },
	                .dstSubresource = {
	                        .aspectMask = vk::ImageAspectFlagBits::ePlane0,
	                        .layerCount = 1,
	                },
	                .extent = {
	                        rect.extent.width,
	                        rect.extent.height,
	                        1,
	                },
	        });
	cmd_buf.copyImage(
	        src_yuv.chroma,
	        vk::ImageLayout::eTransferSrcOptimal,
	        input_images[index],
	        vk::ImageLayout::eTransferDstOptimal,
	        vk::ImageCopy{.srcSubresource = {
	                              .aspectMask = vk::ImageAspectFlagBits::eColor,
	                              .layerCount = 1,
	                      },
	                      .srcOffset = {
	                              .x = rect.offset.x / 2,
	                              .y = rect.offset.y / 2,
	                      },
	                      .dstSubresource = {
	                              .aspectMask = vk::ImageAspectFlagBits::ePlane1,
	                              .layerCount = 1,
	                      },
	                      .extent = {
	                              rect.extent.width / 2,
	                              rect.extent.height / 2,
	                              1,
	                      }});

	barrier.srcStageMask = vk::PipelineStageFlagBits2KHR::eTransfer;
	barrier.srcAccessMask = vk::AccessFlagBits2::eTransferWrite;
	barrier.dstStageMask = vk::PipelineStageFlagBits2KHR::eTopOfPipe;
	barrier.dstAccessMask = vk::AccessFlagBits2::eMemoryRead | vk::AccessFlagBits2::eMemoryWrite;
	barrier.oldLayout = vk::ImageLayout::eTransferDstOptimal,
	barrier.newLayout = vk::ImageLayout::eVideoEncodeSrcKHR,
	cmd_buf.pipelineBarrier2(dep_info);
}
