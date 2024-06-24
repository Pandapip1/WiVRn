/*
 * WiVRn VR streaming
 * Copyright (C) 2022-2024  Guillaume Meunier <guillaume.meunier@centraliens.net>
 * Copyright (C) 2022-2024  Patrick Nicolas <patricknicolas@laposte.net>
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

#pragma once

#include "clock_offset.h"
#include "os/os_time.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <list>
#include <mutex>

template <typename Derived, typename Data, bool extrapolate = false, size_t MaxSamples = 10>
class history
{
	struct TimedData : public Data
	{
		XrTime produced_timestamp;
		XrTime received;
		XrTime at_timestamp_ns;
	};

	std::mutex mutex;
	std::list<TimedData> data;

	XrTime last_get = 0;
	// How long data has been waiting before being consumed
	XrDuration prediction_age = 0;
	// How much in the future shall data be requested
	XrDuration prediction_offset = 0;

protected:
	void add_sample(XrTime produced_timestamp, XrTime timestamp, const Data & sample, const clock_offset & offset)
	{
		XrTime received = os_monotonic_get_ns();
		XrTime produced = offset.from_headset(produced_timestamp);
		XrTime t = offset.from_headset(timestamp);
		std::lock_guard lock(mutex);

		// Discard outdated data, packets could be reordered
		if (not data.empty())
		{
			if (data.back().produced_timestamp > produced)
				return;
		}

		// Only keep one predicted value
		// It should be the last one
		if (not data.empty() and t != produced and data.back().at_timestamp_ns != data.back().produced_timestamp)
			data.pop_back();

		// Insert the new sample
		auto it = std::lower_bound(data.begin(), data.end(), t, [](TimedData & sample, uint64_t t) { return sample.at_timestamp_ns < t; });
		if (it == data.end())
			data.emplace_back(sample, produced, received, t);
		else if (it->at_timestamp_ns == t)
			*it = TimedData(sample, produced, received, t);
		else
			data.emplace(it, sample, produced, received, t);

		while (data.size() > MaxSamples)
			data.pop_front();
	}

public:
	Data get_at(XrTime at_timestamp_ns, bool save_stats = true)
	{
		std::lock_guard lock(mutex);
		XrTime now = os_monotonic_get_ns();
		last_get = now;

		if (data.empty())
		{
			return {};
		}

		if (at_timestamp_ns - data.back().at_timestamp_ns > 1'000'000'000)
		{
			// stale data
			data.clear();
			return {};
		}

		if (data.size() == 1)
		{
			return data.front();
		}

		// Old data
		if (data.front().at_timestamp_ns > at_timestamp_ns)
		{
			if (extrapolate)
			{
				auto second = data.begin();
				auto first = second++;
				return Derived::extrapolate(*first, *second, first->at_timestamp_ns, second->at_timestamp_ns, at_timestamp_ns);
			}
			else
				return data.front();
		}

		// Data between 2 known samples
		for (auto after = data.begin(), before = after++; after != data.end(); before = after++)
		{
			if (after->at_timestamp_ns > at_timestamp_ns)
			{
				float t = float(after->at_timestamp_ns - at_timestamp_ns) /
				          (after->at_timestamp_ns - before->at_timestamp_ns);

				// This is a prediction, attempt to tune values
				if (save_stats and at_timestamp_ns > now)
				{
					XrDuration d = at_timestamp_ns - std::max(before->produced_timestamp, after->produced_timestamp);
					prediction_offset = std::max(prediction_offset, d);

					d = now - std::max(before->received, after->received);
					prediction_age = std::lerp(prediction_age, d, 0.2);
				}
				return Derived::interpolate(*before, *after, t);
			}
		}

		// Data after the latest sample
		if (save_stats)
		{
			XrDuration d = at_timestamp_ns - data.back().produced_timestamp;
			prediction_offset = std::max(prediction_offset, d);

			d = now - data.back().received;
			prediction_age = std::lerp(prediction_age, d, 0.2);
		}

		if (extrapolate)
		{
			auto prev = data.rbegin();
			auto last = prev++;
			return Derived::extrapolate(*prev, *last, prev->at_timestamp_ns, last->at_timestamp_ns, at_timestamp_ns);
		}
		else
		{
			return data.back();
		}
	}

	std::tuple<XrTime, XrDuration, XrDuration> get_stats()
	{
		std::lock_guard lock(mutex);
		std::tuple<XrTime, XrDuration, XrDuration> res{last_get, prediction_age, prediction_offset};
		prediction_offset = 0;
		return res;
	}
};
