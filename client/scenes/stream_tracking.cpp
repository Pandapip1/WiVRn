/*
 * WiVRn VR streaming
 * Copyright (C) 2022  Guillaume Meunier <guillaume.meunier@centraliens.net>
 * Copyright (C) 2022  Patrick Nicolas <patricknicolas@laposte.net>
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

#include "application.h"
#include "stream.h"
#include "utils/ranges.h"
#include <spdlog/spdlog.h>
#include <thread>

static from_headset::tracking::pose locate_space(device_id device, XrSpace space, XrSpace reference, XrTime time)
{
	XrSpaceVelocity velocity{
	        .type = XR_TYPE_SPACE_VELOCITY,
	};

	XrSpaceLocation location{
	        .type = XR_TYPE_SPACE_LOCATION,
	        .next = &velocity,
	};

	xrLocateSpace(space, reference, time, &location);

	from_headset::tracking::pose res{
	        .device = device,
	        .pose = location.pose,
	        .linear_velocity = velocity.linearVelocity,
	        .angular_velocity = velocity.angularVelocity,
	        .flags = 0,
	        .timestamp = time,
	};

	if (location.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)
		res.flags |= from_headset::tracking::orientation_valid;

	if (location.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT)
		res.flags |= from_headset::tracking::position_valid;

	if (velocity.velocityFlags & XR_SPACE_VELOCITY_LINEAR_VALID_BIT)
		res.flags |= from_headset::tracking::linear_velocity_valid;

	if (velocity.velocityFlags & XR_SPACE_VELOCITY_ANGULAR_VALID_BIT)
		res.flags |= from_headset::tracking::angular_velocity_valid;

	if (location.locationFlags & XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT)
		res.flags |= from_headset::tracking::orientation_tracked;

	if (location.locationFlags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT)
		res.flags |= from_headset::tracking::position_tracked;

	return res;
}

static std::optional<std::array<from_headset::hand_tracking::pose, XR_HAND_JOINT_COUNT_EXT>> locate_hands(xr::hand_tracker & hand, XrSpace space, XrTime time)
{
	auto joints = hand.locate(space, time);

	if (joints)
	{
		std::array<from_headset::hand_tracking::pose, XR_HAND_JOINT_COUNT_EXT> poses;
		for (int i = 0; i < XR_HAND_JOINT_COUNT_EXT; i++)
		{
			poses[i] = {
			        .pose = (*joints)[i].first.pose,
			        .linear_velocity = (*joints)[i].second.linearVelocity,
			        .angular_velocity = (*joints)[i].second.angularVelocity,
			        .radius = uint16_t((*joints)[i].first.radius * 10'000),
			};

			if ((*joints)[i].first.locationFlags & XR_SPACE_LOCATION_ORIENTATION_VALID_BIT)
				poses[i].flags |= from_headset::hand_tracking::orientation_valid;

			if ((*joints)[i].first.locationFlags & XR_SPACE_LOCATION_POSITION_VALID_BIT)
				poses[i].flags |= from_headset::hand_tracking::position_valid;

			if ((*joints)[i].second.velocityFlags & XR_SPACE_VELOCITY_LINEAR_VALID_BIT)
				poses[i].flags |= from_headset::hand_tracking::linear_velocity_valid;

			if ((*joints)[i].second.velocityFlags & XR_SPACE_VELOCITY_ANGULAR_VALID_BIT)
				poses[i].flags |= from_headset::hand_tracking::angular_velocity_valid;

			if ((*joints)[i].first.locationFlags & XR_SPACE_LOCATION_ORIENTATION_TRACKED_BIT)
				poses[i].flags |= from_headset::hand_tracking::orientation_tracked;

			if ((*joints)[i].first.locationFlags & XR_SPACE_LOCATION_POSITION_TRACKED_BIT)
				poses[i].flags |= from_headset::hand_tracking::position_tracked;
		}

		return poses;
	}
	else
		return std::nullopt;
}

static device_id cast_id(to_headset::input_pacing_control::target target)
{
	using t = to_headset::input_pacing_control::target;
	switch (target)
	{
		case t::head:
			return device_id::HEAD;
		case t::left_aim:
			return device_id::LEFT_AIM;
		case t::left_grip:
			return device_id::LEFT_GRIP;
		case t::right_aim:
			return device_id::RIGHT_AIM;
		case t::right_grip:
			return device_id::RIGHT_GRIP;
		case t::left_hand:
		case t::right_hand:
			break;
	}
	throw std::out_of_range("bad device id");
}

void scenes::stream::tracking()
{
#ifdef __ANDROID__
	// Runtime may use JNI and needs the thread to be attached
	application::instance().setup_jni();
#endif
	using target = to_headset::input_pacing_control::target;
	std::vector<std::pair<device_id, XrSpace>> spaces = {
	        {device_id::HEAD, application::view()},
	        {device_id::LEFT_AIM, application::left_aim()},
	        {device_id::LEFT_GRIP, application::left_grip()},
	        {device_id::RIGHT_AIM, application::right_aim()},
	        {device_id::RIGHT_GRIP, application::right_grip()}};

	struct sampling_info
	{
		std::chrono::steady_clock::time_point next_sample = std::chrono::steady_clock::now();
		std::chrono::nanoseconds period{1'000'000};
		XrDuration offset{0};
		XrSpace space{XR_NULL_HANDLE}; // null for hands
	};

	// Merge samples that are 0.5ms away
	const std::chrono::nanoseconds merge_threshold(500'000);

	std::array<sampling_info, std::tuple_size_v<decltype(to_headset::input_pacing_control::items)>> sampling;

	sampling[size_t(target::head)].space = application::view();
	sampling[size_t(target::left_aim)].space = application::left_aim();
	sampling[size_t(target::left_grip)].space = application::left_grip();
	sampling[size_t(target::right_aim)].space = application::right_aim();
	sampling[size_t(target::right_grip)].space = application::right_grip();

	XrSpace view_space = application::view();

	from_headset::tracking packet{};
	std::vector<target> targets;

	while (not exiting)
	{
		try
		{
			{
				auto control = input_pacing_control.exchange({});
				if (control)
				{
					for (size_t i = 0; i < control->items.size(); ++i)
					{
						const auto & item = control->items[i];
						sampling[i].period = std::chrono::nanoseconds(item.period);
						sampling[i].next_sample += std::chrono::nanoseconds(item.phase);
						sampling[i].offset = item.offset;
					}
				}
			}
			packet.device_poses.clear();

			auto next = std::ranges::min_element(sampling, {}, [](auto & x) { return x.next_sample; });
			// Collect the data we should gather this iteration
			targets.clear();
			for (size_t i = 0; i < sampling.size(); ++i)
			{
				if (sampling[i].next_sample < next->next_sample + merge_threshold)
				{
					targets.push_back(target(i));
					sampling[i].next_sample += sampling[i].period;
				}
			}
			std::this_thread::sleep_until(next->next_sample);
			XrTime now = instance.now();

			packet.production_timestamp = now;
			packet.flags = 0;
			from_headset::hand_tracking hands{
			        .production_timestamp = now,
			};

			std::lock_guard lock(local_floor_mutex);
			for (auto & target: targets)
			{
				auto & sample_info = sampling[size_t(target)];
				for (XrDuration Δt = 0; Δt <= sample_info.offset; Δt += std::max<XrDuration>(1, sample_info.offset))
				{
					try
					{
						switch (target)
						{
							case target::head: {
								auto [flags, views] = session.locate_views(XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO, now + Δt, view_space);
								assert(views.size() == packet.views.size());
								for (auto [i, j]: utils::zip(views, packet.views))
								{
									j.pose = i.pose;
									j.fov = i.fov;
								}

								packet.flags = flags;
							}
							case target::left_aim:
							case target::left_grip:
							case target::right_aim:
							case target::right_grip:
								packet.device_poses.push_back(locate_space(cast_id(target), sample_info.space, local_floor, now + Δt));
								break;

							case target::left_hand:
								if (application::get_hand_tracking_supported())
								{
									hands.hand = xrt::drivers::wivrn::from_headset::hand_tracking::left;
									hands.joints = locate_hands(application::get_left_hand(), local_floor, hands.timestamp);
									network_session->send_stream(hands);
								}
								break;
							case target::right_hand:

								if (application::get_hand_tracking_supported())
								{
									hands.hand = xrt::drivers::wivrn::from_headset::hand_tracking::right;
									hands.joints = locate_hands(application::get_right_hand(), local_floor, hands.timestamp);
									network_session->send_stream(hands);
								}
								break;
						}
					}
					catch (const std::system_error & e)
					{
						if (e.code().category() != xr::error_category() or
						    e.code().value() != XR_ERROR_TIME_INVALID)
							throw;
					}
				}
			}
			if (not packet.device_poses.empty())
				network_session->send_stream(packet);
		}
		catch (std::exception & e)
		{
			spdlog::info("Exception in tracking thread, exiting: {}", e.what());
			exit();
		}
	}
}

void scenes::stream::operator()(to_headset::input_pacing_control && packet)
{
	motion_to_photon = packet.items[size_t(to_headset::input_pacing_control::target::head)].offset;
	input_pacing_control = packet;
}
