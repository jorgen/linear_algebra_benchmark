#include <random>
#include <chrono>
#include <vector>
#include <stdio.h>

#define generate_run_function(NAME) \
	std::vector<Vec> run_##NAME##(AABB *boxes, size_t boxes_count, Vec *pos) \
	{ \
		std::vector<Vec> result; \
	    result.resize(boxes_count); \
		auto start = std::chrono::high_resolution_clock::now(); \
		for (size_t n = 0; n < 1000; n++) \
		{ \
			for (size_t i = 0; i < boxes_count; i++) \
			{ \
				result[i] = NAME(boxes[i], *pos); \
			} \
		}  \
		auto end = std::chrono::high_resolution_clock::now(); \
		std::chrono::duration<double> elapsed_seconds = end - start; \
		fprintf(stdout, "Time " # NAME " is %f\n", elapsed_seconds.count()); \
		return result; \
	}

#define generate_run_function_result_as_parameter(NAME) \
	std::vector<Vec> run_##NAME##(AABB *boxes, size_t boxes_count, Vec *pos) \
	{ \
		std::vector<Vec> result; \
	    result.resize(boxes_count); \
		auto start = std::chrono::high_resolution_clock::now(); \
		for (size_t n = 0; n < 1000; n++) \
		{ \
			for (size_t i = 0; i < boxes_count; i++) \
			{ \
				NAME(boxes[i], *pos, result[i]); \
			} \
		}  \
		auto end = std::chrono::high_resolution_clock::now(); \
		std::chrono::duration<double> elapsed_seconds = end - start; \
		fprintf(stdout, "Time " # NAME " is %f\n", elapsed_seconds.count()); \
		return result; \
	}

struct Vec
{
	union
	{
		struct
		{
			double x;
			double y;
			double z;
		};
		double p[3];
	};

	bool operator==(const Vec &other) const 
	{
		return memcmp(p, other.p, sizeof(p)) == 0;
	}
	bool operator!=(const Vec &other) const 
	{
		return memcmp(p, other.p, sizeof(p));
	}
	inline void set(double x, double y, double z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}
};
struct AABB
{
	Vec min;
	Vec max;

	inline Vec center() const
	{
		Vec c;
		c.x = max.x + min.x;
		c.y = max.y + min.y;
		c.z = max.z + min.z;
		c.x *= 0.5;
		c.y *= 0.5;
		c.z *= 0.5;
		return c;
	}
};

void setup(AABB *boxes, size_t boxes_size, Vec *pos)
{
	std::vector<double> ran_list;
	ran_list.resize(509);
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(1.0, 2.0);
	for (auto &n : ran_list)
	{
		n = dis(gen);
	}

	uint64_t ran_index = 0;

	for (size_t box_index = 0; box_index < boxes_size; box_index++)
	{
		auto &box = boxes[box_index];
		for (int i = 0; i < 3; i++)
		{
			box.min.p[i] = ran_list[ran_index++ % ran_list.size()];
			box.max.p[i] = ran_list[ran_index++ % ran_list.size()];
			if (box.max.x < box.min.x)
				std::swap(box.min.x, box.max.x);
			if (box.max.y < box.min.y)
				std::swap(box.min.y, box.max.y);
			if (box.max.z < box.min.z)
				std::swap(box.min.z, box.max.z);
		}
	}
	for (int i = 0; i < 3; i++)
		pos->p[i] = ran_list[ran_index++ % ran_list.size()];

}

inline Vec furthest_point_of_aabb(const AABB & __restrict bb, const Vec & __restrict from)
{
	Vec r;

	double x_diff_min = std::abs(bb.min.x - from.x);
	double y_diff_min = std::abs(bb.min.y - from.y);
	double z_diff_min = std::abs(bb.min.z - from.z);
	double x_diff_max = std::abs(bb.max.x - from.x);
	double y_diff_max = std::abs(bb.max.y - from.y);
	double z_diff_max = std::abs(bb.max.z - from.z);

	r.x = x_diff_min < x_diff_max ? bb.max.x : bb.min.x;
	r.y = y_diff_min < y_diff_max ? bb.max.y : bb.min.y;
	r.z = z_diff_min < z_diff_max ? bb.max.z : bb.min.z;

	return r;
}

inline void furthest_point_of_aabb_no_return(const AABB & __restrict bb, const Vec & __restrict from, Vec & __restrict r)
{
	double x_diff_min = bb.min.x > from.x ? bb.min.x - from.x : from.x - bb.min.x;
	double y_diff_min = bb.min.y > from.y ? bb.min.y - from.y : from.y - bb.min.y;
	double z_diff_min = bb.min.z > from.z ? bb.min.z - from.z : from.z - bb.min.z;
	double x_diff_max = bb.max.x > from.x ? bb.max.x - from.x : from.x - bb.max.x;
	double y_diff_max = bb.max.y > from.y ? bb.max.y - from.y : from.y - bb.max.y;
	double z_diff_max = bb.max.z > from.z ? bb.max.z - from.z : from.z - bb.max.z;

	r.x = x_diff_min < x_diff_max ? bb.max.x : bb.min.x;
	r.y = y_diff_min < y_diff_max ? bb.max.y : bb.min.y;
	r.z = z_diff_min < z_diff_max ? bb.max.z : bb.min.z;
}

Vec furthest_point_of_aabb_2(const AABB & __restrict bb, const Vec & __restrict from)
{
    Vec r;
    Vec c = bb.center();

    if (from.x >= c.x && from.y >= c.y && from.z >= c.z)
        r.set(bb.min.x, bb.min.y, bb.min.z);
    else if (from.x < c.x && from.y >= c.y && from.z >= c.z)
        r.set(bb.max.x, bb.min.y, bb.min.z);
    else if (from.x >= c.x && from.y < c.y && from.z >= c.z)
        r.set(bb.min.x, bb.max.y, bb.min.z);
    else if (from.x < c.x && from.y < c.y && from.z >= c.z)
        r.set(bb.max.x, bb.max.y, bb.min.z);
    else if (from.x >= c.x && from.y >= c.y && from.z < c.z)
        r.set(bb.min.x, bb.min.y, bb.max.z);
    else if (from.x < c.x && from.y >= c.y && from.z < c.z)
        r.set(bb.max.x, bb.min.y, bb.max.z);
    else if (from.x >= c.x && from.y < c.y && from.z < c.z)
        r.set(bb.min.x, bb.max.y, bb.max.z);
    else //(from.x < c.x && from.y < c.y && from.z < c.z)
        r.set(bb.max.x, bb.max.y, bb.max.z);

    return r;
}

void furthest_point_of_aabb_2_no_return(const AABB & __restrict bb, const Vec & __restrict from, Vec & __restrict r)
{
    Vec c = bb.center();

    if (from.x >= c.x && from.y >= c.y && from.z >= c.z)
        r.set(bb.min.x, bb.min.y, bb.min.z);
    else if (from.x < c.x && from.y >= c.y && from.z >= c.z)
        r.set(bb.max.x, bb.min.y, bb.min.z);
    else if (from.x >= c.x && from.y < c.y && from.z >= c.z)
        r.set(bb.min.x, bb.max.y, bb.min.z);
    else if (from.x < c.x && from.y < c.y && from.z >= c.z)
        r.set(bb.max.x, bb.max.y, bb.min.z);
    else if (from.x >= c.x && from.y >= c.y && from.z < c.z)
        r.set(bb.min.x, bb.min.y, bb.max.z);
    else if (from.x < c.x && from.y >= c.y && from.z < c.z)
        r.set(bb.max.x, bb.min.y, bb.max.z);
    else if (from.x >= c.x && from.y < c.y && from.z < c.z)
        r.set(bb.min.x, bb.max.y, bb.max.z);
    else //(from.x < c.x && from.y < c.y && from.z < c.z)
        r.set(bb.max.x, bb.max.y, bb.max.z);
}

Vec furthest_point_of_aabb_3(const AABB & __restrict bb, const Vec & __restrict from)
{
    Vec r;
    double cx = (bb.min.x + bb.max.x) * 0.5;
    double cy = (bb.min.y + bb.max.y) * 0.5;
    double cz = (bb.min.z + bb.max.z) * 0.5;

    if (from.x >= cx)
        if (from.y >= cy)
            if (from.z >= cz)
                r.set(bb.min.x, bb.min.y, bb.min.z);
            else
                r.set(bb.min.x, bb.min.y, bb.max.z);
        else
            if (from.z >= cz)
                r.set(bb.min.x, bb.max.y, bb.min.z);
            else
                r.set(bb.min.x, bb.max.y, bb.max.z);
    else
        if (from.y >= cy)
            if (from.z >= cz)
                r.set(bb.max.x, bb.min.y, bb.min.z);
            else
                r.set(bb.max.x, bb.min.y, bb.max.z);
        else
            if (from.z >= cz)
                r.set(bb.max.x, bb.max.y, bb.min.z);
            else
                r.set(bb.max.x, bb.max.y, bb.max.z);

    return r;
}

void furthest_point_of_aabb_3_no_return(const AABB & __restrict bb, const Vec & __restrict from, Vec & __restrict r)
{
    double cx = (bb.min.x + bb.max.x) * 0.5;
    double cy = (bb.min.y + bb.max.y) * 0.5;
    double cz = (bb.min.z + bb.max.z) * 0.5;

    if (from.x >= cx)
        if (from.y >= cy)
            if (from.z >= cz)
                r.set(bb.min.x, bb.min.y, bb.min.z);
            else
                r.set(bb.min.x, bb.min.y, bb.max.z);
        else
            if (from.z >= cz)
                r.set(bb.min.x, bb.max.y, bb.min.z);
            else
                r.set(bb.min.x, bb.max.y, bb.max.z);
    else
        if (from.y >= cy)
            if (from.z >= cz)
                r.set(bb.max.x, bb.min.y, bb.min.z);
            else
                r.set(bb.max.x, bb.min.y, bb.max.z);
        else
            if (from.z >= cz)
                r.set(bb.max.x, bb.max.y, bb.min.z);
            else
                r.set(bb.max.x, bb.max.y, bb.max.z);
}

generate_run_function(furthest_point_of_aabb);
generate_run_function_result_as_parameter(furthest_point_of_aabb_no_return);
generate_run_function(furthest_point_of_aabb_2);
generate_run_function_result_as_parameter(furthest_point_of_aabb_2_no_return);
generate_run_function(furthest_point_of_aabb_3);
generate_run_function_result_as_parameter(furthest_point_of_aabb_3_no_return);

void verify_results(const std::vector<std::vector<Vec>> &results)
{
	if (results.size() < 2)
		return;

	for (size_t result_i = 1; result_i < results.size(); result_i++)
	{
		for (size_t i = 0; i < results[result_i].size(); i++)
		{
			if (results[0][i] != results[result_i][i])
			{
				fprintf(stderr, "Results are not correct %zu - %zu\n", result_i, i);
				return;
			}
		}
	}
}

int main()
{
	std::vector<AABB> boxes;
	boxes.resize(1 << 12);
	Vec pos;
	setup(boxes.data(), boxes.size(), &pos);
	std::vector<std::vector<Vec>> results;
	results.push_back(run_furthest_point_of_aabb(boxes.data(), boxes.size(), &pos));
	results.push_back(run_furthest_point_of_aabb_no_return(boxes.data(), boxes.size(), &pos));
	results.push_back(run_furthest_point_of_aabb_2(boxes.data(), boxes.size(), &pos));
	results.push_back(run_furthest_point_of_aabb_2_no_return(boxes.data(), boxes.size(), &pos));
	results.push_back(run_furthest_point_of_aabb_3(boxes.data(), boxes.size(), &pos));
	results.push_back(run_furthest_point_of_aabb_3_no_return(boxes.data(), boxes.size(), &pos));

	verify_results(results);
	return 0;
}